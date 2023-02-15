#include "CMCorrector.h"

#include "Platform/Common/File.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Settings.h"

#include <numeric>
#include <algorithm>
#include <chrono>

CMCorrector::CMCorrector()
{
  load();
}

void CMCorrector::execute(tf::Subflow&)
{
  // register debug responses and drawings
  debug();

  if (!lastModuleConfig.has_value() && cmCorrectorStatus.state == CMCorrectorStatus::CalibrationState::inactive && theBehaviorData.behaviorState == BehaviorData::BehaviorState::calibrateCameraMatrix)
  {
    lastModuleConfig = ModuleManager::sendModuleRequest({{"MotionRequest", "CMCorrector"}, {"HeadAngleRequest", "CMCorrector"}});

    start();
    return;
  }
  else if (theBehaviorData.behaviorState != BehaviorData::BehaviorState::calibrateCameraMatrix)
  {
    if (lastModuleConfig.has_value())
    {
      ModuleManager::sendModuleRequest(*lastModuleConfig);
      lastModuleConfig.reset();
    }

    stop();
    cmCorrectorStatus.state = CMCorrectorStatus::CalibrationState::inactive;
    return;
  }
  // abort calibration if timeout is reached
  else if (static_cast<int>(theFrameInfo.time - startTimestamp) > static_cast<int>(calibrationTimeout))
  {
    if (lastModuleConfig.has_value())
    {
      SystemCall::text2Speech("Calibration time out");

      ModuleManager::sendModuleRequest(*lastModuleConfig);
      lastModuleConfig.reset();
    }

    stop();
    cmCorrectorStatus.state = CMCorrectorStatus::CalibrationState::finished;
    return;
  }

  const auto keyPressed = [&]()
  {
    return theFrameInfo.getTimeSince(stateBeginTimestamp) > 1000 && theKeyStates.pressed[KeyStates::headMiddle];
  };

  // state machine
  switch (cmCorrectorStatus.state)
  {
  case CMCorrectorStatus::CalibrationState::positioning:
    if (keyPressed())
      gotoState(CMCorrectorStatus::CalibrationState::stand);
    // update(MotionRequest&) transitions to wait when position is reached
    break;
  case CMCorrectorStatus::CalibrationState::stand:
    if (keyPressed())
    {
      samples.clear();
      gotoState(CMCorrectorStatus::CalibrationState::captureUpper);
    }
    break;
  case CMCorrectorStatus::CalibrationState::wait:
    if (theFrameInfo.getTimeSince(stateBeginTimestamp) > static_cast<int>(waitTimeAfterWalking))
    {
      samples.clear();
      gotoState(CMCorrectorStatus::CalibrationState::captureUpper);
    }
    break;
  case CMCorrectorStatus::CalibrationState::captureUpper:
    if (0 < captureHeadPosition && captureHeadPosition < upperHeadAngleRequests.size())
      captureSamples();
    else if (captureHeadPosition >= upperHeadAngleRequests.size())
    {
      if (getNumberOfSamples() >= minLines[0])
        gotoState(CMCorrectorStatus::CalibrationState::optimizeUpper);
      else
        gotoState(CMCorrectorStatus::CalibrationState::captureUpper);
    }
    break;
  case CMCorrectorStatus::CalibrationState::captureLower:
    if (0 < captureHeadPosition && captureHeadPosition < lowerHeadAngleRequests.size())
      captureSamples();
    else if (captureHeadPosition >= lowerHeadAngleRequests.size())
    {
      if (getNumberOfSamples() >= minLines[1])
        gotoState(CMCorrectorStatus::CalibrationState::optimizeLower);
      else
        gotoState(CMCorrectorStatus::CalibrationState::captureLower);
    }
    break;
  case CMCorrectorStatus::CalibrationState::optimizeUpper:
    if (optimizeUpper())
      if (cmCorrectorStatus.stage == 1)
        gotoState(CMCorrectorStatus::CalibrationState::positioning, 2);
      else
        gotoState(CMCorrectorStatus::CalibrationState::captureLower);
    else if (cmCorrectorStatus.stage == 1)
    {
      samples.clear();
      gotoState(CMCorrectorStatus::CalibrationState::captureUpper);
    }
    else
      gotoState(CMCorrectorStatus::CalibrationState::positioning);
    break;
  case CMCorrectorStatus::CalibrationState::optimizeLower:
    if (optimizeLower())
    {
      save();
      gotoState(CMCorrectorStatus::CalibrationState::finished);
    }
    else
      gotoState(CMCorrectorStatus::CalibrationState::positioning);
    break;
  }
}

void CMCorrector::update(CameraCalibration& cameraCalibration)
{
  // provide camera calibration
  cameraCalibration = localCalibration;
}

void CMCorrector::update(HeadAngleRequest& headAngleRequest)
{
  const bool upper = cmCorrectorStatus.state == CMCorrectorStatus::CalibrationState::captureUpper;
  const bool lower = cmCorrectorStatus.state == CMCorrectorStatus::CalibrationState::captureLower;
  const bool localize = cmCorrectorStatus.state == CMCorrectorStatus::CalibrationState::positioning;
  if (!upper && !lower && !localize)
    return;

  const std::vector<HeadAngleRequest>& headAngleRequests = upper ? upperHeadAngleRequests : lower ? lowerHeadAngleRequests : localizeHeadAngleRequests;

  if (captureHeadPosition >= headAngleRequests.size())
  {
    if (localize)
      captureHeadPosition = 0;
    else
      return;
  }

  headAngleRequest = headAngleRequests[captureHeadPosition];

  const Vector2a headRequest(headAngleRequest.pan, headAngleRequest.tilt);
  const Vector2a headSensor(theJointSensorData.angles[Joints::Joint::headYaw], theJointSensorData.angles[Joints::Joint::headPitch]);

  // wait until head reached position and stopped moving
  if ((headRequest - headSensor).norm() < 10_deg && (headSensor - lastHeadSensor).norm() < 0.1_deg)
    ++captureHeadPosition;

  lastHeadSensor = headSensor;
}

void CMCorrector::update(MotionRequest& motionRequest)
{
  if (cmCorrectorStatus.state == CMCorrectorStatus::CalibrationState::inactive)
  {
    motionRequest.motion = MotionRequest::Motion::specialAction;
    motionRequest.specialActionRequest.specialAction = SpecialActionRequest::sitDown;
  }
  else if (cmCorrectorStatus.state == CMCorrectorStatus::CalibrationState::positioning)
  {
    const auto mirrorX = [](const Pose2f& pose) -> Pose2f
    {
      return {-pose.rotation, {pose.translation.x(), -pose.translation.y()}};
    };
    const auto mirrorY = [](const Pose2f& pose) -> Pose2f
    {
      return {Angle::normalize(180_deg - pose.rotation), {-pose.translation.x(), pose.translation.y()}};
    };

    const Pose2f scaledCalibrationPose{
        calibrationPose.rotation, calibrationPose.translation.x() * theFieldDimensions.xPosOpponentGroundline, calibrationPose.translation.y() * theFieldDimensions.yPosLeftSideline};

    const std::array<Pose2f, 4> mirroredPoses = {
        scaledCalibrationPose - theRobotPose, mirrorX(scaledCalibrationPose) - theRobotPose, mirrorY(scaledCalibrationPose) - theRobotPose, mirrorX(mirrorY(scaledCalibrationPose)) - theRobotPose};

    for (unsigned char i = 0; i < mirroredPoses.size(); ++i)
      if (mirroredPoses[i].translation.norm() < mirroredPoses[nearestCalibrationPose].translation.norm() - 500.f)
        nearestCalibrationPose = i;

    const Pose2f& pose = mirroredPoses[nearestCalibrationPose];
    motionRequest.motion = MotionRequest::Motion::walk;
    motionRequest.walkRequest.requestType = WalkRequest::RequestType::destination;
    motionRequest.walkRequest.request = pose;

    if (std::abs(motionRequest.walkRequest.request.rotation) < 10_deg && motionRequest.walkRequest.request.translation.norm() < 200.f && theFrameInfo.getTimeSince(stateBeginTimestamp) > 5000)
      gotoState(CMCorrectorStatus::CalibrationState::wait);
  }
  else
  {
    motionRequest.motion = MotionRequest::Motion::specialAction;
    motionRequest.specialActionRequest.specialAction = SpecialActionRequest::stand;
  }

  if (theFallDownState.state == FallDownState::State::onGround)
  {
    motionRequest.motion = MotionRequest::Motion::specialAction;
    motionRequest.specialActionRequest.mirror = false;

    switch (theFallDownState.direction)
    {
    case FallDownState::Direction::back:
      motionRequest.specialActionRequest.specialAction = theMotionSettings.standUpMotionBackSafest;
      break;
    case FallDownState::Direction::front:
      motionRequest.specialActionRequest.specialAction = theMotionSettings.standUpMotionFrontSafest;
      break;
    case FallDownState::Direction::right:
      motionRequest.specialActionRequest.mirror = true;
      [[fallthrough]];
    case FallDownState::Direction::left:
      motionRequest.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID::standUpSideNao;
      break;
    }
  }
}

void CMCorrector::update(CMCorrectorStatus& cmCorrectorStatus)
{
  const bool upper = cmCorrectorStatus.state == CMCorrectorStatus::CalibrationState::captureUpper;
  this->cmCorrectorStatus.progress = static_cast<float>(getNumberOfSamples()) / minLines[upper ? 0 : 1];

  cmCorrectorStatus = this->cmCorrectorStatus;
}

bool CMCorrector::optimizeUpper()
{
  CameraCalibration calibration = localCalibration;

  static constexpr int dim = 4;
  using VectorDa = Eigen::Matrix<Angle, dim, 1>;

  const auto [upperSamples, _] = separateSamples(samples);

  calibration.bodyRotationCorrection = Vector2a::Zero();
  calibration.upperCameraRotationCorrection = Vector3a::Zero();
  const auto upperCameraMatrixError = [&, &upperSamples = upperSamples](const VectorDa& input)
  {
    // set camera calibration based on input vector
    calibration.bodyRotationCorrection = input.head<2>();
    calibration.upperCameraRotationCorrection.head<2>() = input.tail<2>();

    return calcError(upperSamples, calibration, true);
  };

  const auto start = std::chrono::steady_clock::now();
  const auto [optim, bestError] = optimizeFunction<Angle, dim>(VectorDa::Constant(-12_deg),
      VectorDa::Constant(12_deg),
      {VectorDa::Constant(6_deg), VectorDa::Constant(3_deg), VectorDa::Constant(1.5_deg), VectorDa::Constant(0.75_deg), VectorDa::Constant(0.375_deg), VectorDa::Constant(0.1875_deg), VectorDa::Constant(0.09375_deg), VectorDa::Constant(0.046875_deg)},
      upperCameraMatrixError);
  const auto end = std::chrono::steady_clock::now();
  OUTPUT_TEXT("CMCorrector: Upper camera optimization finished! (" << static_cast<int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()) << "ms)");

  // apply result to camera calibration
  calibration.bodyRotationCorrection = optim.head<2>();
  calibration.upperCameraRotationCorrection.head<2>() = optim.tail<2>();

  printStatistics(samples, calibration);

  const auto& errorLimits = cmCorrectorStatus.stage == 1 ? firstStageErrorLimits : secondStageErrorLimits;
  if (bestError < errorLimits[0])
  {
    // apply new camera calibration if everything was successful
    localCalibration = calibration;
    annotateCalibration();
    return true;
  }
  else
  {
    ANNOTATION("CMCorrector", "Calibration failed! Error=" << static_cast<Angle>(bestError));
    OUTPUT_ERROR("CMCorrector: Calibration failed! Error=" << static_cast<Angle>(bestError));
    return false;
  }
}

bool CMCorrector::optimizeLower()
{
  CameraCalibration calibration = localCalibration;

  static constexpr int dim = 3;
  using VectorDa = Eigen::Matrix<Angle, dim, 1>;

  calibration.lowerCameraRotationCorrection = Vector3a::Zero();
  const auto lowerCameraMatrixError = [&](const VectorDa& input)
  {
    // set camera calibration based on input vector
    calibration.lowerCameraRotationCorrection = input;

    return calcError(samples, calibration, true);
  };

  const auto start = std::chrono::steady_clock::now();
  const auto [optim, bestError] = optimizeFunction<Angle, dim>(VectorDa::Constant(-12_deg),
      VectorDa::Constant(12_deg),
      {VectorDa::Constant(6_deg), VectorDa::Constant(3_deg), VectorDa::Constant(1.5_deg), VectorDa::Constant(0.75_deg), VectorDa::Constant(0.375_deg), VectorDa::Constant(0.1875_deg), VectorDa::Constant(0.09375_deg), VectorDa::Constant(0.046875_deg)},
      lowerCameraMatrixError);
  const auto end = std::chrono::steady_clock::now();
  OUTPUT_TEXT("CMCorrector: Lower camera optimization finished! (" << static_cast<int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()) << "ms)");

  // apply result to camera calibration
  calibration.lowerCameraRotationCorrection = optim;

  printStatistics(samples, calibration);

  const auto& errorLimits = cmCorrectorStatus.stage == 1 ? firstStageErrorLimits : secondStageErrorLimits;
  if (bestError < errorLimits[1])
  {
    // apply new camera calibration if everything was successful
    localCalibration = calibration;
    annotateCalibration();
    return true;
  }
  else
  {
    ANNOTATION("CMCorrector", "Calibration failed! Error=" << static_cast<Angle>(bestError));
    OUTPUT_ERROR("CMCorrector: Calibration failed! Error=" << static_cast<Angle>(bestError));
    return false;
  }
}

void CMCorrector::captureSamples()
{
  if (cmCorrectorStatus.state != CMCorrectorStatus::CalibrationState::captureUpper && cmCorrectorStatus.state != CMCorrectorStatus::CalibrationState::captureLower)
    return;

  const bool upper = cmCorrectorStatus.state == CMCorrectorStatus::CalibrationState::captureUpper;
  auto sample = captureSample(upper);
  if (!sample.has_value())
    return;

  samples.push_back(std::move(*sample));
}

size_t CMCorrector::getNumberOfSamples() const
{
  const auto [upperSamples, lowerSamples] = separateSamples(samples);
  const bool upper = cmCorrectorStatus.state == CMCorrectorStatus::CalibrationState::captureUpper;
  return countLines(upper ? upperSamples : lowerSamples);
}

std::optional<CMCorrector::Sample> CMCorrector::captureSample(bool upper) const
{
  std::vector<Geometry::Line> lines;
  for (const CLIPFieldLinesPercept::FieldLine& line : theCLIPFieldLinesPercept.lines)
  {
    if (line.fromUpper != upper)
      continue;

    Vector2f lineDirection = (line.endInImage - line.startInImage).cast<float>();

    // skip lines that are too short
    if (std::abs(lineDirection.y()) < minLineLength[line.fromUpper ? 0 : 1].y() && std::abs(lineDirection.x()) < minLineLength[line.fromUpper ? 0 : 1].x())
      continue;

    lines.emplace_back(line.startInImage, lineDirection);
  }

  if (lines.empty())
    return {};

  const Vector2a headPosition{theJointSensorData.angles[Joints::headYaw], theJointSensorData.angles[Joints::headPitch]};

  return {{std::move(lines), headPosition, theTorsoMatrix, upper}};
}

size_t CMCorrector::countLines(const std::vector<Sample>& samples)
{
  return std::accumulate(samples.begin(),
      samples.end(),
      size_t{0},
      [](size_t val, const Sample& sample)
      {
        return val + sample.lines.size();
      });
}

CameraMatrix CMCorrector::getCameraMatrix(const CameraCalibration& cc, const TorsoMatrix& tm, const Vector2a& hp, bool upper) const
{
  const RobotCameraMatrix rm(theRobotDimensions, hp.x(), hp.y(), cc, upper);
  return {tm, rm, cc};
}

std::optional<Geometry::Line> CMCorrector::transformImageToRobot(const Geometry::Line& line, const CameraMatrix& cm, const CameraInfo& ci)
{
  Vector2f startFieldPos, endFieldPos;
  const Vector2f end = line.base + line.direction;

  if (!Transformation::imageToRobot(line.base.x(), line.base.y(), cm, ci, startFieldPos))
    return {};
  if (!Transformation::imageToRobot(end.x(), end.y(), cm, ci, endFieldPos))
    return {};

  return {{startFieldPos, endFieldPos - startFieldPos}};
}

std::vector<std::optional<Geometry::Line>> CMCorrector::transformSampleToRobot(const Sample& sample, const CameraCalibration& cc) const
{
  // transform percepts to field
  std::vector<std::optional<Geometry::Line>> robotLines;
  robotLines.reserve(sample.lines.size());

  const CameraMatrix cm = getCameraMatrix(cc, sample.torsoMatrix, sample.headPosition, sample.upper);
  const CameraInfo& ci = sample.upper ? theCameraInfoUpper : theCameraInfo;

  std::transform(sample.lines.begin(),
      sample.lines.end(),
      std::back_inserter(robotLines),
      [&](const Geometry::Line& line)
      {
        return transformImageToRobot(line, cm, ci);
      });

  return robotLines;
}

std::vector<std::optional<Geometry::Line>> CMCorrector::transformSampleToRobot(const Sample& sample, const Pose3f& torsoNeckMatrix, const Pose3f& neckCameraMatrix) const
{
  // transform percepts to field
  std::vector<std::optional<Geometry::Line>> robotLines;
  robotLines.reserve(sample.lines.size());

  CameraMatrix cm;
  cm.computeCameraMatrix(sample.torsoMatrix, torsoNeckMatrix, neckCameraMatrix, sample.headPosition.x(), sample.headPosition.y());
  const CameraInfo& ci = sample.upper ? theCameraInfoUpper : theCameraInfo;

  std::transform(sample.lines.begin(),
      sample.lines.end(),
      std::back_inserter(robotLines),
      [&](const Geometry::Line& line)
      {
        return transformImageToRobot(line, cm, ci);
      });

  return robotLines;
}

std::vector<std::optional<Geometry::Line>> CMCorrector::transformSamplesToRobot(const std::vector<Sample>& samples, const CameraCalibration& cc) const
{
  const Pose3f torsoNeckMatrix = CameraMatrix::getTorsoNeckMatrix(theRobotDimensions, cc);
  std::optional<Pose3f> neckCameraMatrixUpper, neckCameraMatrixLower;

  std::vector<std::vector<std::optional<Geometry::Line>>> robotLines;
  std::transform(samples.begin(),
      samples.end(),
      std::back_inserter(robotLines),
      [&](const Sample& sample)
      {
        if (sample.upper && !neckCameraMatrixUpper.has_value())
          neckCameraMatrixUpper = RobotCameraMatrix::getNeckCameraMatrix(theRobotDimensions, cc, true);
        if (!sample.upper && !neckCameraMatrixLower.has_value())
          neckCameraMatrixLower = RobotCameraMatrix::getNeckCameraMatrix(theRobotDimensions, cc, false);

        return transformSampleToRobot(sample, torsoNeckMatrix, sample.upper ? *neckCameraMatrixUpper : *neckCameraMatrixLower);
        //return transformSampleToRobot(sample, cc);
      });

  return join(robotLines);
}

Angle CMCorrector::getTotalError(const std::vector<std::optional<Geometry::Line>>& robotLines)
{
  if (robotLines.size() < 2)
    return std::numeric_limits<float>::infinity();

  const auto lineToAngle = [](const std::optional<Geometry::Line>& line)
  {
    if (line.has_value())
      return line->direction.angle();
    else
      return std::numeric_limits<float>::signaling_NaN();
  };

  std::vector<Angle> lineAngles;
  lineAngles.reserve(robotLines.size());

  std::transform(robotLines.begin(), robotLines.end(), std::back_inserter(lineAngles), lineToAngle);

  Angle error = 0;
  for (auto itA = lineAngles.cbegin(); itA < lineAngles.cend(); ++itA)
  {
    for (auto itB = itA + 1; itB < lineAngles.cend(); ++itB)
    {
      const Angle a = *itA, b = *itB;

      if (a == a && b == b)
      {
        Angle angle = std::abs(a - b);
        if (angle > 180_deg)
          angle = 360_deg - angle;
        if (angle > 90_deg)
          angle = 180_deg - angle;
        if (angle > 45_deg)
          angle = 90_deg - angle;

        error += angle;
      }
      else
      {
        error += 45_deg;
      }
    }
  }

  return static_cast<Angle>(error) / (robotLines.size() * (robotLines.size() - 1));
}

Angle CMCorrector::getTotalErrorApprox(const std::vector<std::optional<Geometry::Line>>& robotLines)
{
  if (robotLines.empty())
    return std::numeric_limits<float>::infinity();

  const auto addQuarterVec = [](const Vector2f sum, const std::optional<Geometry::Line>& line) -> Vector2f
  {
    if (line.has_value())
      return sum + Vector2f{1.f, 0}.rotate(line->direction.angle() * 4.f);
    else
      return sum - sum.normalized(); // penalize failed transformations
  };

  const Vector2f quarterVec = std::accumulate(robotLines.begin(), robotLines.end(), Vector2f(0.f, 0.f), addQuarterVec);

  // estimate angle error
  return (1.f - quarterVec.norm() / robotLines.size()) * 45_deg;
}

std::tuple<std::array<unsigned int, 180>, unsigned int> CMCorrector::getAngleDistribution(const std::vector<std::optional<Geometry::Line>>& robotLines)
{
  std::tuple<std::array<unsigned int, 180>, unsigned int> ret({0}, 0);

  for (const auto& line : robotLines)
  {
    if (line.has_value())
    {
      Angle angle = line->direction.angle();

      if (angle < 0_deg)
        angle += 360_deg;

      const size_t index = static_cast<size_t>(angle.toDegrees()) % 180;
      std::get<0>(ret)[index]++;
    }
    else
      std::get<1>(ret)++;
  }

  return ret;
}

Angle CMCorrector::calcError(const std::vector<Sample>& samples, const CameraCalibration& cc, bool approx) const
{
  const std::vector<std::optional<Geometry::Line>> robotLines = transformSamplesToRobot(samples, cc);

  return approx ? getTotalErrorApprox(robotLines) : getTotalError(robotLines);
}

std::tuple<std::vector<CMCorrector::Sample>, std::vector<CMCorrector::Sample>> CMCorrector::separateSamples(const std::vector<Sample>& samples)
{
  std::vector<Sample> upper, lower;

  for (const Sample& sample : samples)
    (sample.upper ? upper : lower).emplace_back(sample);

  return {std::move(upper), std::move(lower)};
}

template <typename T> std::vector<T> CMCorrector::join(std::vector<std::vector<T>>& vec)
{
  const size_t totalSize = std::accumulate(vec.begin(),
      vec.end(),
      size_t{0},
      [](size_t val, const std::vector<T>& vec)
      {
        return val + vec.size();
      });

  std::vector<T> ret;
  ret.reserve(totalSize);

  for (std::vector<T>& v : vec)
    std::move(v.begin(), v.end(), std::back_inserter(ret));

  return ret;
}

template <typename T, int dim>
std::pair<Eigen::Matrix<T, dim, 1>, float> CMCorrector::optimizeFunction(
    const Eigen::Matrix<T, dim, 1>& min, const Eigen::Matrix<T, dim, 1>& max, const std::vector<Eigen::Matrix<T, dim, 1>>& stepSizes, std::function<float(const Eigen::Matrix<T, dim, 1>&)> func)
{
  using Vector = Eigen::Matrix<T, dim, 1>;

  float bestError = std::numeric_limits<float>::infinity();
  Vector bestInput = Vector::Constant(std::numeric_limits<T>::signaling_NaN());

  Vector stepMin = min, stepMax = max;

  for (const Vector& stepSize : stepSizes)
  {
    std::tie(bestInput, bestError) = optimizeFunction<T, dim>(stepMin, stepMax, stepSize, func);

    stepMin = bestInput - stepSize;
    stepMax = bestInput + stepSize;
  }

  return {bestInput, bestError};
}

template <typename T, int dim>
std::pair<Eigen::Matrix<T, dim, 1>, float> CMCorrector::optimizeFunction(
    const Eigen::Matrix<T, dim, 1>& min, const Eigen::Matrix<T, dim, 1>& max, const Eigen::Matrix<T, dim, 1>& stepSize, std::function<float(const Eigen::Matrix<T, dim, 1>&)> func)
{
  using Vector = Eigen::Matrix<T, dim, 1>;
  using NewVector = Eigen::Matrix<T, dim - 1, 1>;

  float bestError = std::numeric_limits<float>::infinity();
  Vector bestInput = Vector::Constant(std::numeric_limits<T>::signaling_NaN());
  float retError = bestError;
  NewVector retInput = NewVector::Constant(std::numeric_limits<T>::signaling_NaN());

  for (Vector input = min; input[0] <= max[0]; input[0] += stepSize[0])
  {
    if constexpr (dim > 1)
    {
      const auto newFunc = [&](const NewVector& val)
      {
        input.template tail<dim - 1>() = val;
        return func(input);
      };
      std::tie(retInput, retError) = optimizeFunction<T, dim - 1>(min.template tail<dim - 1>(), max.template tail<dim - 1>(), stepSize.template tail<dim - 1>(), newFunc);
    }
    else
    {
      retError = func(input);
    }

    if (retError < bestError)
    {
      bestInput[0] = input[0];
      bestInput.template tail<dim - 1>() = retInput;
      bestError = retError;
    }
  }

  return {bestInput, bestError};
}

/**
 * Start camera calibration with given head position (optional).
 */
void CMCorrector::start()
{
  startTimestamp = theFrameInfo.time;
  gotoState(CMCorrectorStatus::CalibrationState::wait, 1);
  samples.clear();
}

void CMCorrector::gotoState(CMCorrectorStatus::CalibrationState state, unsigned char stage)
{
  stateBeginTimestamp = theFrameInfo.time;
  cmCorrectorStatus.state = state;
  if (stage != 0)
    cmCorrectorStatus.stage = stage;

  if (state == CMCorrectorStatus::CalibrationState::captureUpper || state == CMCorrectorStatus::CalibrationState::captureLower)
    captureHeadPosition = 0;
};

/**
 * Stop camera calibration.
 */
void CMCorrector::stop()
{
  if (cmCorrectorStatus.state == CMCorrectorStatus::CalibrationState::inactive)
    return;

  gotoState(CMCorrectorStatus::CalibrationState::stand);
}

/**
 * Load camera calibration from file.
 */
void CMCorrector::load()
{
  InMapFile stream("./Config/Robots/" + Global::getSettings().robotName + "/" + Global::getSettings().bodyName + "/cameraCalibration.cfg");
  if (stream.exists())
  {
    stream >> localCalibration;
    annotateCalibration();
    return;
  }

  // if head-body combination does not exist, combine both configs
  CameraCalibration head, body;
  InMapFile streamHead("./Config/Robots/" + Global::getSettings().robotName + "/" + Global::getSettings().robotName + "/cameraCalibration.cfg");
  InMapFile streamBody("./Config/Robots/" + Global::getSettings().bodyName + "/" + Global::getSettings().bodyName + "/cameraCalibration.cfg");
  if (streamHead.exists() && streamBody.exists())
  {
    streamHead >> head;
    streamBody >> body;

    localCalibration.lowerCameraRotationCorrection = head.lowerCameraRotationCorrection;
    localCalibration.upperCameraRotationCorrection = head.upperCameraRotationCorrection;
    localCalibration.bodyRotationCorrection = body.bodyRotationCorrection;
    annotateCalibration();
    return;
  }

  // fallback to default config (e.g., from V6 folder)
  InMapFile streamDefault("cameraCalibration.cfg");
  if (streamDefault.exists())
    streamDefault >> localCalibration;
  annotateCalibration();
}

/**
 * Save camera calibration to file.
 */
void CMCorrector::save() const
{
  std::string dir = std::string("mkdir -p ") + std::string(File::getBHDir()) + "/Config/Robots/" + Global::getSettings().robotName + "/" + Global::getSettings().bodyName;
  system(dir.c_str());

  OutMapFile stream("./Config/Robots/" + Global::getSettings().robotName + "/" + Global::getSettings().bodyName + "/cameraCalibration.cfg");
  if (stream.exists())
  {
    stream << localCalibration;
  }
  else
  {
    OUTPUT_ERROR("Saving to config failed!");
  }
}

void CMCorrector::reset()
{
  samples.clear();
  localCalibration = CameraCalibration();
}

void CMCorrector::printStatistics(const std::vector<Sample>& samples, const CameraCalibration& cc) const
{
  const auto [upperSamples, lowerSamples] = separateSamples(samples);

  const Angle totalError = calcError(samples, cc);
  const Angle upperError = calcError(upperSamples, cc);
  const Angle lowerError = calcError(lowerSamples, cc);

  const Angle totalErrorApprox = calcError(samples, cc, true);
  const Angle upperErrorApprox = calcError(upperSamples, cc, true);
  const Angle lowerErrorApprox = calcError(lowerSamples, cc, true);

  const size_t totalSamplesCount = samples.size();
  const size_t upperSamplesCount = upperSamples.size();
  const size_t lowerSamplesCount = lowerSamples.size();

  const size_t totalLines = countLines(samples);
  const size_t upperLines = countLines(upperSamples);
  const size_t lowerLines = countLines(lowerSamples);

  OUTPUT_TEXT("CMCorrector statistics:");
  OUTPUT_TEXT("Samples: total=" << totalSamplesCount << ", upper=" << upperSamplesCount << ", lower=" << lowerSamplesCount);
  OUTPUT_TEXT("Lines: total=" << totalLines << ", upper=" << upperLines << ", lower=" << lowerLines);
  OUTPUT_TEXT("Error: total=" << totalError << ", upper=" << upperError << ", lower=" << lowerError);
  OUTPUT_TEXT("ErrorApprox: total=" << totalErrorApprox << ", upper=" << upperErrorApprox << ", lower=" << lowerErrorApprox);

  const auto [dist, failed] = getAngleDistribution(transformSamplesToRobot(samples, cc));
  for (unsigned int elem : dist)
  {
    PLOT("module:CMCorrector:distribution", elem);
  }
  OUTPUT_TEXT("Failed transformations: " << failed);
}

void CMCorrector::annotateCalibration() const
{
  const auto& upper = localCalibration.upperCameraRotationCorrection;
  const auto& lower = localCalibration.lowerCameraRotationCorrection;
  const auto& body = localCalibration.bodyRotationCorrection;
  ANNOTATION("CMCorrector",
      "upper=(" << upper.x() << "," << upper.y() << "," << upper.z() << "), lower=(" << lower.x() << "," << lower.y() << "," << lower.z() << "), body=(" << body.x() << "," << body.y() << ")");
}

void CMCorrector::debug()
{
  DECLARE_PLOT("module:CMCorrector:distribution");

  DECLARE_DEBUG_DRAWING("module:CMCorrector:field", "drawingOnField");
  COMPLEX_DRAWING("module:CMCorrector:field")
  {
    const auto drawLines = [](const auto& lines, const auto color)
    {
      for (const auto& line : lines)
      {
        if (line.has_value())
        {
          const Vector2f end = line->base + line->direction;
          LINE("module:CMCorrector:field", line->base.x(), line->base.y(), end.x(), end.y(), 10, Drawings::PenStyle::solidPen, color);
        }
      }
    };

    const auto [upperSamples, lowerSamples] = separateSamples(samples);

    drawLines(transformSamplesToRobot(lowerSamples, theCameraCalibration), ColorRGBA(255, 255, 255, 100));
    drawLines(transformSamplesToRobot(upperSamples, theCameraCalibration), ColorRGBA(0, 0, 0, 100));
  }

  DEBUG_RESPONSE_ONCE("module:CMCorrector:print")
  printStatistics(samples, theCameraCalibration);

  DEBUG_RESPONSE("module:CMCorrector:captureUpper")
  {
    cmCorrectorStatus.state = CMCorrectorStatus::CalibrationState::captureUpper;
    captureSamples();
  }

  DEBUG_RESPONSE("module:CMCorrector:captureLower")
  {
    cmCorrectorStatus.state = CMCorrectorStatus::CalibrationState::captureLower;
    captureSamples();
  }

  DEBUG_RESPONSE_ONCE("module:CMCorrector:optimizeUpper")
  optimizeUpper();
  DEBUG_RESPONSE_ONCE("module:CMCorrector:optimizeLower")
  optimizeLower();

  DEBUG_RESPONSE_ONCE("module:CMCorrector:start") start();
  DEBUG_RESPONSE_ONCE("module:CMCorrector:stop") stop();
  DEBUG_RESPONSE_ONCE("module:CMCorrector:load_config") load();
  DEBUG_RESPONSE_ONCE("module:CMCorrector:save_config") save();
  DEBUG_RESPONSE_ONCE("module:CMCorrector:reset") reset();
}

MAKE_MODULE(CMCorrector, perception)
