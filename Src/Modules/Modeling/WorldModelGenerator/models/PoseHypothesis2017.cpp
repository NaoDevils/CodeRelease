/**
 * @file PoseHypothesis2017.cpp
 *
 * Implementation of a class which represents a single world map in form of an EKF
 *
 * @author <a href="mailto:stefan.tasse@tu-dortmund.de">Stefan Tasse</a>
 * @author <a href="mailto:daniel.hauschildt@tu-dortmund.de">Daniel Hauschildt</a>
 * @author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>

 * @issue check the copy constructors
 *
 */

#include "PoseHypothesis2017.h"
#include "Tools/Modeling/PoseGenerator.h"

tls_type PoseHypothesis2017::KalmanStateUpdate* PoseHypothesis2017::stateUpdate = 0;
tls_type PoseHypothesis2017::KalmanInfiniteLineStateUpdate* PoseHypothesis2017::stateUpdateInfiniteLine = 0;

tls_type uint64_t PoseHypothesis2017::lastUniqueId = 0;

namespace
{
  /** HELPER FUNCTIONS **/
  void extractTeammateNumbers(std::string str, std::vector<int> &teammates)
  {
    std::string::size_type idx = 0;
    while (idx < str.size())
    {
      str = str.substr(idx);
      int i = std::stoi(str, &idx);
      teammates.push_back(i);
      // skip ','
      idx++;
    }
  }

  const Teammate *getTeammateByNumber(const TeammateData &theTeammateData, const int &number)
  {
    for (const auto &tm : theTeammateData.teammates)
    {
      if (tm.number == number)
      {
        return &tm;
      }
    }
    return 0;
  }
}

PoseHypothesis2017::PoseHypothesis2017(const Pose2f & newPose, float _positionConfidence, float _symmetryConfidence, const unsigned &timeStamp, const SelfLocator2017Parameters& parameters) :
  positionConfidence(_positionConfidence),
  normalizedPositionConfidence(_positionConfidence),
  symmetryConfidence(_symmetryConfidence),
  mirrored(nullptr),
  initialized(false),
  sensorUpdated(false)
{
  covariance = Eigen::Matrix<double, totalDimension, totalDimension>::Identity();
  covariance *= 100000.0;
  _uniqueId = getNextUniqueId();
  creationTime = timeStamp;
  init(newPose, parameters);
}
PoseHypothesis2017::PoseHypothesis2017(const PoseHypothesis2017& other, const unsigned &timeStamp)
  : _uniqueId(0)
  , positionConfidence(0.f)
  , normalizedPositionConfidence(0.)
  , symmetryConfidence(0.f)
  , mirrored(nullptr)
  , initialized(false)
  , sensorUpdated(false)
  , creationTime(timeStamp)
{
  *this = other;
}

PoseHypothesis2017& PoseHypothesis2017::operator=(const PoseHypothesis2017& other)
{
  _uniqueId = getNextUniqueId();
  if (this == &other)
  {
    return *this;
  }
  covariance = other.covariance;
  initialized = other.initialized;
  normalizedPositionConfidence = other.normalizedPositionConfidence;
  observationAnglesWithLocalFeaturePerceptionsSpherical.clear();
  observationsAsNormalsWithLocalFeaturePerceptionsInfiniteLines.clear();
  positionConfidence = other.positionConfidence;
  singleMeasurementCovariance_2x2 = other.singleMeasurementCovariance_2x2;
  state = other.state;
  symmetryConfidence = other.symmetryConfidence;
  creationTime = other.creationTime;
  sensorUpdated = other.sensorUpdated;
  mirrored = other.mirrored;
  return *this;
}

PoseHypothesis2017::~PoseHypothesis2017()
{
  // Remove reference of mirrored hypothesis to this one
  if (mirrored && (mirrored->mirroredHypothesis() == this))
    mirrored->mirroredHypothesis() = nullptr;
}

void PoseHypothesis2017::cleanup()
{
  if (stateUpdate != 0)
  {
    delete stateUpdate;
    stateUpdate = 0;
  }
  if (stateUpdateInfiniteLine != 0)
  {
    delete stateUpdateInfiniteLine;
    stateUpdateInfiniteLine = 0;
  }
}

void PoseHypothesis2017::init(const Pose2f& newPose, const SelfLocator2017Parameters& parameters)
{
  initialized = true;


  // set the new state
  state[0] = newPose.translation.x();
  state[1] = newPose.translation.y();
  state[2] = newPose.rotation;

  // set initial noise from "measurement noise"
  covariance = Eigen::Matrix<double, totalDimension, totalDimension>::Identity();
  covariance *= 100000;

  singleMeasurementCovariance_2x2 = Matrix2d::Zero();
  singleMeasurementCovariance_2x2(0, 0) = parameters.sensorUpdate.verticalAngleVariance;
  singleMeasurementCovariance_2x2(1, 1) = parameters.sensorUpdate.horizontalAngleVariance;

  poseCovarianceForLineMatching = Matrix3d::Zero();
  poseCovarianceForLineMatching(0, 0) = parameters.matching.positionVarianceForLines;
  poseCovarianceForLineMatching(1, 1) = parameters.matching.positionVarianceForLines;
  poseCovarianceForLineMatching(2, 2) = parameters.matching.orientationVarianceForLines;

  if (!stateUpdate)
  {
    Matrix2d singleMeasurementCovariance = Matrix2d::Zero();
    singleMeasurementCovariance(0, 0) = parameters.sensorUpdate.verticalAngleVariance;
    singleMeasurementCovariance(1, 1) = parameters.sensorUpdate.horizontalAngleVariance;

    stateUpdate = new PoseHypothesis2017::KalmanStateUpdate();
    stateUpdate->initMeasurementCovariance(singleMeasurementCovariance, parameters.sensorUpdate.correlationFactorBetweenMeasurements);
  }

  if (!stateUpdateInfiniteLine)
  {
    Matrix3d singleInfiniteLineMeasurementCovariance = Matrix3d::Zero();
    singleInfiniteLineMeasurementCovariance(0, 0) = parameters.sensorUpdate.projectiveNormalVariance;
    singleInfiniteLineMeasurementCovariance(1, 1) = parameters.sensorUpdate.projectiveNormalVariance;
    singleInfiniteLineMeasurementCovariance(2, 2) = parameters.sensorUpdate.projectiveNormalVariance;

    stateUpdateInfiniteLine = new PoseHypothesis2017::KalmanInfiniteLineStateUpdate();
    stateUpdateInfiniteLine->initMeasurementCovarianceInfiniteLine(singleInfiniteLineMeasurementCovariance, parameters.sensorUpdate.correlationFactorBetweenMeasurements);
  }
}

void PoseHypothesis2017::predict(const Pose2f & odometryDelta, const SelfLocator2017Parameters& parameters)
{
  Pose2f tempPose;
  getRobotPose(tempPose);
  tempPose += odometryDelta;
  state[0] = tempPose.translation.x();
  state[1] = tempPose.translation.y();
  state[2] = Angle::normalize(tempPose.rotation);

  // add robot process noise
  const float minPositionVariance = 100 * parameters.processUpdate.positionVariance;
  const float minRotationVariance = 100 * parameters.processUpdate.rotationVariance;
  const float& minPositionChange = parameters.processUpdate.minPositionChageForCovarianceUpdate;
  const float& minRotationChange = parameters.processUpdate.minRotationChageForCovarianceUpdate;
  if ((!parameters.processUpdate.odometryBasedVarianceUpdate || std::abs(odometryDelta.translation.x()) < minPositionChange) || covariance(0, 0) < minPositionVariance)
    covariance(0, 0) += parameters.processUpdate.positionVariance;
  if ((!parameters.processUpdate.odometryBasedVarianceUpdate || std::abs(odometryDelta.translation.y()) < minPositionChange) || covariance(1, 1) < minPositionVariance)
    covariance(1, 1) += parameters.processUpdate.positionVariance;
  if ((!parameters.processUpdate.odometryBasedVarianceUpdate || std::abs(odometryDelta.rotation) < minRotationChange) || covariance(2, 2) < minRotationVariance)
    covariance(2, 2) += parameters.processUpdate.rotationVariance;

  sensorUpdated = false;
}

void PoseHypothesis2017::fillCorrectionMatrices(
  const LineMatchingResult & theLineMatchingResult,
  const CLIPCenterCirclePercept &theCenterCirclePercept,
  const CLIPGoalPercept &theGoalPercept,
  const PenaltyCrossPercept &thePenaltyCrossPercept,
  const FieldDimensions & theFieldDimensions,
  const CameraMatrix & theCameraMatrix,
  const CameraMatrixUpper & theCameraMatrixUpper,
  const SelfLocator2017Parameters &parameters)
{
  Pose2f  robotPose;
  getRobotPose(robotPose);

  MySphericalObservationVector& sphericalObservations = observationAnglesWithLocalFeaturePerceptionsSpherical;
  MyInfiniteLineObservationVector& infiniteLineObservations = observationsAsNormalsWithLocalFeaturePerceptionsInfiniteLines;

  // Clear old spherical observations
  sphericalObservations.clear();
  // Clear old infinite line observations
  infiniteLineObservations.clear();


  // Update with feature percepts (angles)
  // update with center circle
  if (theCenterCirclePercept.centerCircleWasSeen
    && theCenterCirclePercept.centerCircle.locationOnField.cast<float>().norm() > 50)
    // the center might also be under or even behind the robot;
    // don't use this, too much trouble with periodicities and singularities etc.
  {
    const float &cameraHeight = theCenterCirclePercept.fromUpper ? theCameraMatrixUpper.translation.z() : theCameraMatrix.translation.z();

    SphericalObservation<totalDimension> observation;
    calculateSphericalObservation(observation, theCenterCirclePercept.centerCircle.locationOnField, Vector2d(0, 0), cameraHeight, parameters);

    if (calculateMeasurementLikelihoodSpherical(observation.realAngles, observation.nominalAngles, parameters) > 0)
    {
      sphericalObservations.push_back(observation);

      Vector2f obsInFieldCoords = robotPose * Vector2f(theCenterCirclePercept.centerCircle.locationOnField.cast<float>());
      LINE("module:SelfLocator2017:correspondences",
        robotPose.translation.x(), robotPose.translation.y(), obsInFieldCoords.x(), obsInFieldCoords.y(), 30, Drawings::dashedPen, ColorRGBA(200, 200, 200));
      ARROW("module:SelfLocator2017:correspondences",
        obsInFieldCoords.x(), obsInFieldCoords.y(), 0, 0, 30, Drawings::solidPen, ColorRGBA(200, 200, 200));
    }
  }

  // update with penalty cross
  if (thePenaltyCrossPercept.penaltyCrossWasSeen
    && thePenaltyCrossPercept.pointOnField.x() > 50)
    // don't update with too close percept
    // too much trouble with periodicities and singularities etc.
  {
    const float &cameraHeight = thePenaltyCrossPercept.fromUpper ? theCameraMatrixUpper.translation.z() : theCameraMatrix.translation.z();
    Vector2d penaltyMarkOpp(theFieldDimensions.xPosOpponentPenaltyMark, 0);
    Vector2d penaltyMarkOwn(theFieldDimensions.xPosOwnPenaltyMark, 0);
    Vector2f relativePosition(thePenaltyCrossPercept.pointOnField.x(), thePenaltyCrossPercept.pointOnField.y());

    Vector2d obsInFieldCoords = (robotPose * relativePosition).cast<double>();
    bool useOwnMark =
      (obsInFieldCoords - penaltyMarkOwn).norm()
      < (obsInFieldCoords - penaltyMarkOpp).norm();

    const Vector2d &penaltyMark = useOwnMark ? penaltyMarkOwn : penaltyMarkOpp;
    SphericalObservation<totalDimension> observation;
    calculateSphericalObservation(observation, relativePosition, penaltyMark, cameraHeight, parameters);

    if (calculateMeasurementLikelihoodSpherical(observation.realAngles, observation.nominalAngles, parameters) > 0)
    {
      sphericalObservations.push_back(observation);

      LINE("module:SelfLocator2017:correspondences",
        robotPose.translation.x(), robotPose.translation.y(), obsInFieldCoords.x(), obsInFieldCoords.y(), 30, Drawings::dashedPen, ColorRGBA(200, 200, 200));
      ARROW("module:SelfLocator2017:correspondences",
        obsInFieldCoords.x(), obsInFieldCoords.y(), penaltyMark.x(), penaltyMark.y(), 30, Drawings::solidPen, ColorRGBA(200, 200, 200));
    }
  }

  // update with goal posts
  if (theGoalPercept.numberOfGoalPosts > 0
    && theGoalPercept.numberOfGoalPosts < 3) // one of them is a false positive anyway and using any of them is risky
  {
    for (auto &goalPost : theGoalPercept.goalPosts)
    {
      SphericalObservation<totalDimension> observation;
      if (findGoalPostMatch(goalPost, theCameraMatrix, theCameraMatrixUpper, theFieldDimensions, parameters, observation))
      {
        sphericalObservations.push_back(observation);

        Vector2f obsInFieldCoords = robotPose * Vector2f(goalPost.locationOnField.cast<float>());
        LINE("module:SelfLocator2017:correspondences",
          robotPose.translation.x(), robotPose.translation.y(), obsInFieldCoords.x(), obsInFieldCoords.y(), 30, Drawings::dashedPen, ColorRGBA(200, 200, 0));
        ARROW("module:SelfLocator2017:correspondences",
          obsInFieldCoords.x(), obsInFieldCoords.y(),
          observation.nominalAngles.x(), observation.nominalAngles.y(),
          30, Drawings::solidPen, ColorRGBA(200, 200, 0));
      }
    }
  }

  // Fieldlines
  std::vector<LineMatchingResult::FieldLine> correspondencesForObservations;
  double likelihoodThreshold = 0.1;
  bool foundCorrespondenceInLineMatch = false;

  if (theLineMatchingResult.containsUniqueMatches())
  {
    foundCorrespondenceInLineMatch = theLineMatchingResult.getCorrespondencesForLocalizationHypothesis(
      robotPose,
      poseCovarianceForLineMatching,
      likelihoodThreshold,
      singleMeasurementCovariance_2x2,
      correspondencesForObservations,
      parameters.debugging.displayWarnings);

    if (foundCorrespondenceInLineMatch)
    {
      for (unsigned int i = 0; i < theLineMatchingResult.observations.size(); i++)
      {
        const LineMatchingResult::FieldLine & observedLine = theLineMatchingResult.observations[i];
        const LineMatchingResult::FieldLine & correspondingFieldLineSegment = correspondencesForObservations[i];

        // check for false positives on center circle
        // Filter critical correspondences on center circle
        // (Center line and perpendicular part of center circle)
        if (!theCenterCirclePercept.centerCircleWasSeen
          && correspondingFieldLineSegment.start.norm() < theFieldDimensions.centerCircleRadius * 1.2
          && correspondingFieldLineSegment.end.norm() < theFieldDimensions.centerCircleRadius * 1.2)
        {
          continue; // this is likely a fragment of the center circle
        }
        else
        {
          const float cameraHeight = static_cast<float>(observedLine.cameraHeight);

          SphericalObservation<totalDimension> start, end;

          calculateSphericalObservation(start, observedLine.start, correspondingFieldLineSegment.start, cameraHeight, parameters);
          calculateSphericalObservation(end, observedLine.end, correspondingFieldLineSegment.end, cameraHeight, parameters);

          if (calculateMeasurementLikelihoodSpherical(start.realAngles, start.nominalAngles, parameters) > 0)
          {
            sphericalObservations.push_back(start);
          }

          if (calculateMeasurementLikelihoodSpherical(end.realAngles, end.nominalAngles, parameters) > 0)
          {
            sphericalObservations.push_back(end);
          }
        }
      }
    }
  }
  // Update with infinite lines (normals)
  else if (theLineMatchingResult.containsNonUniqueMatches())
  {
    foundCorrespondenceInLineMatch = theLineMatchingResult.getCorrespondencesForLocalizationHypothesis(
      robotPose,
      poseCovarianceForLineMatching,
      likelihoodThreshold,
      singleMeasurementCovariance_2x2,
      correspondencesForObservations,
      parameters.debugging.displayWarnings);

    // filter "critical" correspondences in combination with onlyObservedOneFieldLine:
    // * looking out of the field and only seeing something corresponding to the own penalty line
    //   -> this is likely the goal line, as that one would also be seen, otherwise
    // * a single segment of the center circle generated a line observation
    if (foundCorrespondenceInLineMatch && theLineMatchingResult.onlyObservedOneFieldLine)
    {
      const LineMatchingResult::FieldLine & firstObservation = theLineMatchingResult.observations[0];
      const LineMatchingResult::FieldLine & firstCorrespondence = correspondencesForObservations[0];
      double observationDirection = robotPose.rotation + (firstObservation.start + firstObservation.end).angle();// angle of pose to middle of observed line in abs coords

      // penalty area situation
      bool isPenaltyAreaSituation = false;
      if (fabs(observationDirection) < pi_2)
      {
        // looking to opponent goal direction
        isPenaltyAreaSituation = (firstCorrespondence.end.x() == theFieldDimensions.xPosOpponentPenaltyArea
          &&firstCorrespondence.start.x() == theFieldDimensions.xPosOpponentPenaltyArea);
      }
      else
      {
        // looking to own goal direction
        isPenaltyAreaSituation = (firstCorrespondence.end.x() == theFieldDimensions.xPosOwnPenaltyArea
          &&firstCorrespondence.start.x() == theFieldDimensions.xPosOwnPenaltyArea);
      }

      // center circle situation
      bool lineFragmentNearCenterCircle = false;
      if (!theCenterCirclePercept.centerCircleWasSeen)
      {
        for (unsigned int i = 0; i < theLineMatchingResult.observations.size(); i++)
        {
          const LineMatchingResult::FieldLine & correspondingFieldLineSegment = correspondencesForObservations[i];
          if (correspondingFieldLineSegment.start.norm() < theFieldDimensions.centerCircleRadius * 2.0
            && correspondingFieldLineSegment.end.norm() < theFieldDimensions.centerCircleRadius * 2.0)
          {
            lineFragmentNearCenterCircle = true;
          }
        }
      }

      if (lineFragmentNearCenterCircle || isPenaltyAreaSituation)
      {
        foundCorrespondenceInLineMatch = false; // do not accept the match and do not use it to update the pose
      }

      if (firstCorrespondence.start == firstCorrespondence.end)
      {
        // It's just a point -> Will generate invalid state
        if (parameters.debugging.displayWarnings)
          OUTPUT_WARNING("PoseHypothesis2017 detected that LineMatchingResult found corresponding segment which is a point.");
        foundCorrespondenceInLineMatch = false;
      }
    }

    if (foundCorrespondenceInLineMatch)
    {
      for (unsigned int i = 0; i < theLineMatchingResult.observations.size(); i++)
      {
        const LineMatchingResult::FieldLine & observedLine = theLineMatchingResult.observations[i];
        const LineMatchingResult::FieldLine & correspondingFieldLineSegment = correspondencesForObservations[i];

        if (correspondingFieldLineSegment.start == correspondingFieldLineSegment.end)
        {
          // It's just a point -> Will generate invalid state
          if (parameters.debugging.displayWarnings)
            OUTPUT_WARNING("PoseHypothesis2017 detected that LineMatchingResult found corresponding segment which is a point.");
          continue;
        }

        InfiniteLineObservation<totalDimension> observation;

        calculateInfiniteLineObservation(observation, observedLine, correspondingFieldLineSegment, parameters);

        infiniteLineObservations.push_back(observation);
      }
    }
  }

  // Debug line matching
  Pose2f requestCorrespondencesForThisPose;
  MODIFY("module:SelfLocator2017:requestLineCorrespondencesForThisPose", requestCorrespondencesForThisPose);
  MODIFY("module:SelfLocator2017:lineCorrespondenceLikelihoodThreshold", likelihoodThreshold);

  bool debugSpecificPose = requestCorrespondencesForThisPose.translation.x() != 0
    || requestCorrespondencesForThisPose.translation.y() != 0
    || requestCorrespondencesForThisPose.rotation != 0;
  if (debugSpecificPose)
  {
    theLineMatchingResult.drawRequestedCorrespondences(requestCorrespondencesForThisPose, poseCovarianceForLineMatching, likelihoodThreshold, singleMeasurementCovariance_2x2);
  }
  else if (foundCorrespondenceInLineMatch)
  {
    theLineMatchingResult.drawRequestedCorrespondences(robotPose, poseCovarianceForLineMatching, likelihoodThreshold, singleMeasurementCovariance_2x2);
  }
}

void PoseHypothesis2017::updateStateRotationWithLocalFieldLines(
  const CLIPFieldLinesPercept &theFieldLinesPercept,
  const SelfLocator2017Parameters &parameters)
{
  if (containsInvalidValues())
    return;

  // Adjust hypotheses to best fitting angle
  double bestOrientationFromPercepts = 0.0;
  if (findBestAngle(theFieldLinesPercept, bestOrientationFromPercepts))
  {
    double tempRotation = bestOrientationFromPercepts;
    while (std::abs(Angle::normalize(state[2] - tempRotation)) > pi_4)
      tempRotation = Angle::normalize(tempRotation + pi_2);
    //if (theSensorCalibration.gyroZGain == 0) // NaoV4 should use this fully
    //  newPositionConfidence = 1;
    const float &weight = parameters.processUpdate.adjustRotationToBestFittingAngle;
    double diff = Angle::normalize(tempRotation - state[2]);
    state[2] = Angle::normalize(state[2] + weight * diff);

    sensorUpdated = true;
  }
}

bool PoseHypothesis2017::updatePositionConfidenceWithLocalFeaturePerceptionsWeighted(
  const CLIPFieldLinesPercept &theFieldLinesPercept,
  const CLIPCenterCirclePercept &theCenterCirclePercept,
  const CLIPGoalPercept &theGoalPercept,
  const PenaltyCrossPercept &thePenaltyCrossPercept,
  const FieldDimensions & theFieldDimensions,
  const SelfLocator2017Parameters & parameters)
{
  bool update = false;
  Pose2f  robotPose;
  getRobotPose(robotPose);
  static int lh[5] = { 0, 2, 4, 6, 9 };
  static int lv[6] = { 1, 3, 5, 7, 8, 10 };
  static std::vector<int> linesHorizontal(&lh[0], &lh[0] + 5);
  static std::vector<int> linesVertical(&lv[0], &lv[0] + 6);
  // LEVEL 1
  if (parameters.sensorUpdate.use1stLevelUpdate)
    update |= updatePositionConfidenceWithSingleLines(theFieldLinesPercept, theFieldDimensions, parameters,
      robotPose, linesHorizontal, linesVertical);
  // LEVEL 2
  if (parameters.sensorUpdate.use2ndLevelUpdate)
    update |= updatePositionConfidenceWithLineCrossings(theFieldLinesPercept, theFieldDimensions, parameters,
      robotPose, linesHorizontal, linesVertical);
  // LEVEL 3
  if (parameters.sensorUpdate.use3rdLevelUpdate)
    update |= updatePositionConfidenceWithLineAndLandmark(
      theFieldLinesPercept, theCenterCirclePercept, theGoalPercept,
      thePenaltyCrossPercept, theFieldDimensions, parameters,
      robotPose, linesHorizontal, linesVertical);

  return update;
}

// LEVEL 1
bool PoseHypothesis2017::updatePositionConfidenceWithSingleLines(
  const CLIPFieldLinesPercept & theFieldLinesPercept,
  const FieldDimensions & theFieldDimensions,
  const SelfLocator2017Parameters & parameters,
  const Pose2f &robotPose,
  const std::vector<int> &linesHorizontal,
  const std::vector<int> &linesVertical)
{
  float correspondence = 0.f;

  std::vector<CLIPFieldLinesPercept::FieldLine>::const_iterator percept = theFieldLinesPercept.lines.begin();
  std::vector<CLIPFieldLinesPercept::FieldLine>::const_iterator perceptEnd = theFieldLinesPercept.lines.end();
  bool foundMatch = false;
  bool update = false;
  for (; percept != perceptEnd; ++percept)
  {
    foundMatch = false;
    correspondence = 0.0;
    Vector2f perceptFieldStart = Transformation::robotToField(robotPose, (*percept).startOnField);
    Vector2f perceptFieldEnd = Transformation::robotToField(robotPose, (*percept).endOnField);
    //double perceptDistanceRelative = (((*percept).startOnField + (*percept).endOnField)*0.5).abs();
    float angle = (perceptFieldEnd - perceptFieldStart).angle();
    float length = (perceptFieldEnd - perceptFieldStart).norm();
    float horizontalAngle = std::min<float>(std::abs(Angle::normalize(angle + pi_2)),
      std::abs(Angle::normalize(angle - pi_2)));
    float verticalAngle = std::min<float>(std::abs(angle),
      std::abs(Angle::normalize(angle + pi)));
    bool isHorizontal = horizontalAngle < parameters.sensorUpdate.worstAngleDifference; // TODO
    bool isVertical = verticalAngle < parameters.sensorUpdate.worstAngleDifference; // TODO
    int minDistanceID = -1;

    if (percept->isPlausible
      && length > parameters.sensorUpdate.minLineLengthOnField)
    {
      float minDistance = 100000.f;
      Vector2f perceptCenterField = ((perceptFieldEnd + perceptFieldStart)*0.5f);

      if (isHorizontal)
      {
        for (std::vector<int>::const_iterator line = linesHorizontal.begin(); line != linesHorizontal.end(); ++line)
        {
          const FieldDimensions::LinesTable::Line &fieldLine = theFieldDimensions.fieldLines.lines[(*line)];
          float allowedOffset = fieldLine.length / 10;
          float minY = std::min(fieldLine.from.y(), fieldLine.to.y()) - allowedOffset;
          float maxY = std::max(fieldLine.from.y(), fieldLine.to.y()) + allowedOffset;
          float perceptDistance = std::abs(fieldLine.from.x() - perceptCenterField.x());
          if (perceptDistance < minDistance
            && perceptFieldEnd.y() > minY
            && perceptFieldEnd.y() < maxY
            && perceptFieldStart.y() > minY
            && perceptFieldStart.y() < maxY)
          {
            minDistance = perceptDistance;
            minDistanceID = *line;
          }
        }
      }
      else if (isVertical)
      {
        for (std::vector<int>::const_iterator line = linesVertical.begin(); line != linesVertical.end(); ++line)
        {
          const FieldDimensions::LinesTable::Line &fieldLine = theFieldDimensions.fieldLines.lines[(*line)];
          float allowedOffset = fieldLine.length / 10;
          float minX = std::min(fieldLine.from.x(), fieldLine.to.x()) - allowedOffset;
          float maxX = std::max(fieldLine.from.x(), fieldLine.to.x()) + allowedOffset;
          float perceptDistance = std::abs(fieldLine.from.y() - perceptCenterField.y());
          if (perceptDistance < minDistance
            && perceptFieldEnd.x() > minX
            && perceptFieldEnd.x() < maxX
            && perceptFieldStart.x() > minX
            && perceptFieldStart.x() < maxX)
          {
            minDistance = perceptDistance;
            minDistanceID = *line;
          }
        }
      }
      if (minDistanceID >= 0 &&
        length / theFieldDimensions.fieldLines.lines[minDistanceID].length < parameters.sensorUpdate.lineLengthMatchFactorMax)
      {
        // TODO: use field dimensions methods?
        float distanceToFieldLine = isHorizontal ? std::abs(theFieldDimensions.fieldLines.lines[minDistanceID].from.x() - robotPose.translation.x())
          : std::abs(theFieldDimensions.fieldLines.lines[minDistanceID].from.y() - robotPose.translation.y());
        angle = isHorizontal ? horizontalAngle : verticalAngle;
        // x percent distance error ok
        float distanceErrorPercent = std::max((minDistance / (distanceToFieldLine + 1) - parameters.sensorUpdate.maxDistanceError), 0.f);
        correspondence = (1.f - std::min<float>(1.f, (distanceToFieldLine / 1000.f) * distanceErrorPercent))
          * (1.f - angle / parameters.sensorUpdate.worstAngleDifference);
        foundMatch = (correspondence > 0.01f);
      }

      if (foundMatch)
        updatePositionConfidence(true, parameters.sensorUpdate.influenceOfNewLineMeasurementOnPositionConfidence, percept->validity*correspondence);
      else
        updatePositionConfidence(false, parameters.sensorUpdate.influenceOfNewLineMeasurementOnPositionConfidence, percept->validity);

      update |= foundMatch;
    }
  }
  return update;
}

// LEVEL 2
bool PoseHypothesis2017::updatePositionConfidenceWithLineCrossings(
  const CLIPFieldLinesPercept & theFieldLinesPercept,
  const FieldDimensions & theFieldDimensions,
  const SelfLocator2017Parameters & parameters,
  const Pose2f &robotPose,
  const std::vector<int> &linesHorizontal,
  const std::vector<int> &linesVertical)
{
  return false;
}

// LEVEL 3
bool PoseHypothesis2017::updatePositionConfidenceWithLineAndLandmark(
  const CLIPFieldLinesPercept & theFieldLinesPercept,
  const CLIPCenterCirclePercept &theCenterCirclePercept,
  const CLIPGoalPercept &theGoalPercept,
  const PenaltyCrossPercept &thePenaltyCrossPercept,
  const FieldDimensions & theFieldDimensions,
  const SelfLocator2017Parameters & parameters,
  const Pose2f &robotPose,
  const std::vector<int> &linesHorizontal,
  const std::vector<int> &linesVertical)
{
  if (containsInvalidValues())
    return false;

  Pose2f pose;
  float weight = 0.f;
  bool foundMatch = false;
  bool update = false;

  // Update from Center Circle and Center Line
  float perceptWeight = 0.f;
  if (PoseGenerator::getPoseFromCenterCircleAndCenterLine(theFieldLinesPercept, theCenterCirclePercept, pose, perceptWeight) > 0)
  {
    Pose2f symmetricPose(Angle::normalize(pose.rotation + pi), pose.translation * -1);
    //Pose2f &closestPose = robotPose.translation.x() < 0 ? pose : symmetricPose;
    const Pose2f &closestPose = getClosestPose(robotPose, pose, symmetricPose);
    weight = perceptWeight * calcWeightForPoseDifference(closestPose - robotPose, parameters);
    update |= (foundMatch = weight > 0.f);
    updatePositionConfidence(foundMatch, parameters.sensorUpdate.influenceOfNewCenterCircleMeasurementOnPositionConfidence, weight);
    DRAW_ROBOT_POSE("module:SelfLocator2017:poseFromCenterCircle", closestPose, ColorRGBA(255 - (unsigned char)(perceptWeight * 255.f), (unsigned char)(perceptWeight * 255.f), 0));
  }

  // Update from Penalty Cross and Line
  foundMatch = false;
  if (PoseGenerator::getPoseFromPenaltyCrossAndLine(theFieldDimensions, theFieldLinesPercept, thePenaltyCrossPercept, pose, perceptWeight))
  {
    Pose2f symmetricPose(Angle::normalize(pose.rotation + pi), pose.translation * -1);
    //Pose2f &closestPose = robotPose.translation.x() > 0 ? pose : symmetricPose;
    const Pose2f &closestPose = getClosestPose(robotPose, pose, symmetricPose);
    weight = perceptWeight * calcWeightForPoseDifference(closestPose - robotPose, parameters);
    update |= (foundMatch = weight > 0.f);
    updatePositionConfidence(foundMatch, parameters.sensorUpdate.influenceOfNewPenaltyCrossMeasurementOnPositionConfidence, weight);
    DRAW_ROBOT_POSE("module:SelfLocator2017:poseFromPenaltyCross", closestPose, ColorRGBA(0, 0, 255));
  }
  return update;
}

bool PoseHypothesis2017::updatePositionConfidenceWithLocalFeaturePerceptionsSpherical(
  const LineMatchingResult &theLineMatchingResult,
  const CLIPCenterCirclePercept &theCenterCirclePercept,
  const CLIPGoalPercept &theGoalPercept,
  const PenaltyCrossPercept &thePenaltyCrossPercept,
  const FieldDimensions & theFieldDimensions,
  const CameraMatrix & theCameraMatrix,
  const CameraMatrixUpper & theCameraMatrixUpper,
  const SelfLocator2017Parameters & parameters)
{
  if (containsInvalidValues())
    return false;

  Pose2f  robotPose;
  getRobotPose(robotPose);

  std::vector<LineMatchingResult::FieldLine> correspondencesForObservations;
  correspondencesForObservations.clear();
  double likelihoodThreshold = 0.1;
  bool update = false;

  // update with field lines
  if (theLineMatchingResult.containsUniqueMatches())
  {
    bool foundCorrespondenceInLineMatch = theLineMatchingResult.getCorrespondencesForLocalizationHypothesis(robotPose, poseCovarianceForLineMatching, likelihoodThreshold, singleMeasurementCovariance_2x2, correspondencesForObservations);
    updatePositionConfidence(foundCorrespondenceInLineMatch, parameters.sensorUpdate.influenceOfNewLineMeasurementOnPositionConfidence, 1.f);
    update |= foundCorrespondenceInLineMatch;
  }

  // update with center circle
  if (theCenterCirclePercept.centerCircleWasSeen
    && theCenterCirclePercept.centerCircle.locationOnField.cast<float>().norm() > 50)
    // the center might also be under or even behind the robot;
    // don't use this, too much trouble with periodicities and singularities etc.
  {
    const float &cameraHeight = theCenterCirclePercept.fromUpper ? theCameraMatrixUpper.translation.z() : theCameraMatrix.translation.z();
    SphericalObservation<totalDimension> observation;
    calculateSphericalObservation(observation, theCenterCirclePercept.centerCircle.locationOnField, Vector2d(0, 0), cameraHeight, parameters);

    if (calculateMeasurementLikelihoodSpherical(observation.realAngles, observation.nominalAngles, parameters) > 0)
    {
      Vector2d relativePosition(theCenterCirclePercept.centerCircle.locationOnField.cast<double>());
      Vector2d expectedMeasurementCartesian = Transformation::fieldToRobot(robotPose, Vector2f::Zero()).cast<double>();
      float distanceErrorFactor = 1.f - std::min(1.f, static_cast<float>((expectedMeasurementCartesian - relativePosition).norm()) / 2000);
      updatePositionConfidence(true, parameters.sensorUpdate.influenceOfNewCenterCircleMeasurementOnPositionConfidence, distanceErrorFactor);
    }
    else
    {
      updatePositionConfidence(false, parameters.sensorUpdate.influenceOfNewCenterCircleMeasurementOnPositionConfidence, 1.f);
    }
    update = true;
  }

  // update with penalty cross
  if (thePenaltyCrossPercept.penaltyCrossWasSeen
    && thePenaltyCrossPercept.pointOnField.x() > 50)
    // dont update with too close percept
    // too much trouble with periodicities and singularities etc.
  {
    const float &cameraHeight = thePenaltyCrossPercept.fromUpper ? theCameraMatrixUpper.translation.z() : theCameraMatrix.translation.z();
    Vector2d penaltyMarkOpp(theFieldDimensions.xPosOpponentPenaltyMark, 0);
    Vector2d penaltyMarkOwn(theFieldDimensions.xPosOwnPenaltyMark, 0);
    Vector2d relativePosition = thePenaltyCrossPercept.pointOnField.cast<double>();

    Vector2d obsInFieldCoords = Transformation::robotToField(robotPose, relativePosition.cast<float>()).cast<double>();
    bool useOwnMark =
      (obsInFieldCoords - penaltyMarkOwn).norm()
      < (obsInFieldCoords - penaltyMarkOpp).norm();

    const Vector2d &penaltyMark = useOwnMark ? penaltyMarkOwn : penaltyMarkOpp;

    SphericalObservation<totalDimension> observation;
    calculateSphericalObservation(observation, relativePosition, penaltyMark, cameraHeight, parameters);

    if (calculateMeasurementLikelihoodSpherical(observation.realAngles, observation.nominalAngles, parameters) > 0)
    {
      Vector2d expectedMeasurementCartesian = Transformation::fieldToRobot(robotPose, penaltyMark.cast<float>()).cast<double>();
      float distanceErrorFactor = parameters.sensorUpdate.maxInfluenceOnPositionConfidencePenaltyCrossOnly -
        std::min(parameters.sensorUpdate.maxInfluenceOnPositionConfidencePenaltyCrossOnly,
          static_cast<float>((expectedMeasurementCartesian - relativePosition).norm() / 2000));
      updatePositionConfidence(true, parameters.sensorUpdate.influenceOfNewPenaltyCrossMeasurementOnPositionConfidence, distanceErrorFactor);
    }
    else
    {
      updatePositionConfidence(false, parameters.sensorUpdate.influenceOfNewPenaltyCrossMeasurementOnPositionConfidence, 1.f);
    }
    update = true;
  }

  // update with goal posts
  if (theGoalPercept.numberOfGoalPosts > 0
    && theGoalPercept.numberOfGoalPosts < 3) // one of them is a false positive anyway and using any of them is risky
  {
    for (int i = 0; i < theGoalPercept.numberOfGoalPosts; i++)
    {
      SphericalObservation<totalDimension> observation;
      if (findGoalPostMatch(theGoalPercept.goalPosts[i], theCameraMatrix, theCameraMatrixUpper, theFieldDimensions, parameters, observation))
      {
        updatePositionConfidence(true, parameters.sensorUpdate.influenceOfNewGoalMeasurementOnPositionConfidence, 1.f);
      }
      else
      {
        updatePositionConfidence(false, parameters.sensorUpdate.influenceOfNewGoalMeasurementOnPositionConfidence, 1.f);
      }
      update = true;
    }
  }
  return update;
}
void PoseHypothesis2017::updateStateWithLocalFeaturePerceptionsSpherical(const SelfLocator2017Parameters & parameters)
{
  if (containsInvalidValues())
    return;

  //will be filled in fillCorrectionMatrices
  MySphericalObservationVector& sphericalObservations = observationAnglesWithLocalFeaturePerceptionsSpherical;

  if (sphericalObservations.empty()) return;

  MySphericalObservationVector::size_type dimensionOfMeasurementVector = sphericalObservations.size();

  if (parameters.debugging.displayWarnings && dimensionOfMeasurementVector > maxMeasurementDimensionSpherical)
  {
    OUTPUT_WARNING("PoseHypothesis2017::updateWithLocalFeaturePerceptions(...): Consider increasing maxMeasurementDimension!");
  }
  while (dimensionOfMeasurementVector > maxMeasurementDimensionSpherical)
  {
    sphericalObservations.pop_back();
    dimensionOfMeasurementVector = sphericalObservations.size();
  }

  if (!stateUpdate)
  {
    OUTPUT_ERROR("Cannot update hypothesis state because no kalman filter for spherical observations is present. This should've been done in init method.");
    return;
  }

  Eigen::Matrix<double, totalDimension, 1> correction = stateUpdate->updateWithLocalObservations(sphericalObservations, covariance);
  if (correction.isMuchSmallerThan(0.1))
    OUTPUT_TEXT("Minimal state correction neglected");
  else
  {
    state += correction;
    sensorUpdated = true;
  }
}

bool PoseHypothesis2017::updatePositionConfidenceWithLocalFeaturePerceptionsInfiniteLines(
  const LineMatchingResult & theLineMatchingResult,
  const CLIPCenterCirclePercept &theCenterCirclePercept,
  const FieldDimensions & theFieldDimensions,
  const SelfLocator2017Parameters & parameters)
{
  if (containsInvalidValues())
    return false;

  Pose2f  robotPose;
  getRobotPose(robotPose);

  double likelihoodThreshold = 0.1;
  bool update = false;

  // update with field lines
  bool foundCorrespondenceInLineMatch = false;
  if (theLineMatchingResult.containsNonUniqueMatches())
  {
    std::vector<LineMatchingResult::FieldLine> correspondencesForObservations;
    foundCorrespondenceInLineMatch = theLineMatchingResult.getCorrespondencesForLocalizationHypothesis(robotPose, poseCovarianceForLineMatching, likelihoodThreshold, singleMeasurementCovariance_2x2, correspondencesForObservations);

    updatePositionConfidence(foundCorrespondenceInLineMatch, parameters.sensorUpdate.influenceOfNewInfiniteLineMeasurementOnPositionConfidence, 1.f);
    update = true;
  }
  return update;
}
void PoseHypothesis2017::updateStateWithLocalFeaturePerceptionsInfiniteLines(const SelfLocator2017Parameters & parameters)
{
  if (containsInvalidValues())
    return;

  MyInfiniteLineObservationVector& infiniteLineObservations = observationsAsNormalsWithLocalFeaturePerceptionsInfiniteLines;

  if (infiniteLineObservations.empty()) return;

  MyInfiniteLineObservationVector::size_type dimensionOfMeasurementVector = infiniteLineObservations.size();

  if (parameters.debugging.displayWarnings && dimensionOfMeasurementVector > maxMeasurementDimensionInfiniteLines)
  {
    OUTPUT_WARNING("PoseHypothesis2017::updateStateWithLocalFeaturePerceptionsInfiniteLines(...): Consider increasing maxMeasurementDimension!");
  }
  while (dimensionOfMeasurementVector > maxMeasurementDimensionInfiniteLines)
  {
    infiniteLineObservations.pop_back();
    dimensionOfMeasurementVector = infiniteLineObservations.size();
  }

  if (!stateUpdateInfiniteLine)
  {
    OUTPUT_ERROR("Cannot update hypothesis state because no kalman filter for infinite lines is present. This should've been done in init method.");
    return;
  }

  Eigen::Matrix<double, totalDimension, 1> correction = stateUpdateInfiniteLine->updateWithLocalInfiniteLineObservations(infiniteLineObservations, covariance);
  if (correction.isMuchSmallerThan(0.1))
    OUTPUT_TEXT("Minimal state correction neglected");
  else
  {
    state += correction;
    sensorUpdated = true;
  }
}

void PoseHypothesis2017::updateWithLocalFeaturePerceptionsCartesian(
  const CLIPCenterCirclePercept &theCenterCirclePercept,
  const PenaltyCrossPercept &thPenaltyCrossPercept,
  const SelfLocator2017Parameters & parameters)
{
  OUTPUT_ERROR("Not implemented yet");
}

bool PoseHypothesis2017::findGoalPostMatch(const CLIPGoalPercept::GoalPost & goalPost,
  const CameraMatrix & theCameraMatrix,
  const CameraMatrixUpper & theCameraMatrixUpper,
  const FieldDimensions & theFieldDimensions,
  const SelfLocator2017Parameters& parameters,
  SphericalObservation<totalDimension> &bestObservation)
{
  double likelihood;
  Vector2d correspondenceInGlobalCoords;

  const float& cameraHeight = goalPost.fromUpper ? theCameraMatrixUpper.translation.z() : theCameraMatrix.translation.z();
  double bestLikelihood = 0;

  if (goalPost.goalPostSide != CLIPGoalPercept::GoalPost::rightPost) // unknown or left
  {
    SphericalObservation<totalDimension> observationOwn, observationOpp;

    // left own goal post ("left" as seen from the middle of the field)
    correspondenceInGlobalCoords.x() = theFieldDimensions.xPosOwnGroundline;
    correspondenceInGlobalCoords.y() = theFieldDimensions.yPosRightGoal;
    calculateSphericalObservation(observationOwn, goalPost.locationOnField, correspondenceInGlobalCoords, cameraHeight, parameters);
    likelihood = calculateMeasurementLikelihoodSpherical(observationOwn.realAngles, observationOwn.nominalAngles, parameters);
    if (likelihood > bestLikelihood)
    {
      bestLikelihood = likelihood;
      bestObservation = observationOwn;
    }

    // left opponent goal post ("left" as seen from the middle of the field)
    correspondenceInGlobalCoords *= -1;
    calculateSphericalObservation(observationOpp, goalPost.locationOnField, correspondenceInGlobalCoords, cameraHeight, parameters);
    likelihood = calculateMeasurementLikelihoodSpherical(observationOpp.realAngles, observationOpp.nominalAngles, parameters);
    if (likelihood > bestLikelihood)
    {
      bestLikelihood = likelihood;
      bestObservation = observationOpp;
    }
  }

  if (goalPost.goalPostSide != CLIPGoalPercept::GoalPost::leftPost) // unknown or right
  {
    SphericalObservation<totalDimension> observationOwn, observationOpp;

    // right own goal post ("right" as seen from the middle of the field)
    correspondenceInGlobalCoords.x() = theFieldDimensions.xPosOwnGroundline;
    correspondenceInGlobalCoords.y() = theFieldDimensions.yPosLeftGoal;
    calculateSphericalObservation(observationOwn, goalPost.locationOnField, correspondenceInGlobalCoords, cameraHeight, parameters);
    likelihood = calculateMeasurementLikelihoodSpherical(observationOwn.realAngles, observationOwn.nominalAngles, parameters);
    if (likelihood > bestLikelihood)
    {
      bestLikelihood = likelihood;
      bestObservation = observationOwn;
    }

    // left opponent goal post ("left" as seen from the middle of the field)
    correspondenceInGlobalCoords *= -1;
    calculateSphericalObservation(observationOpp, goalPost.locationOnField, correspondenceInGlobalCoords, cameraHeight, parameters);
    likelihood = calculateMeasurementLikelihoodSpherical(observationOpp.realAngles, observationOpp.nominalAngles, parameters);
    if (likelihood > bestLikelihood)
    {
      bestLikelihood = likelihood;
      bestObservation = observationOpp;
    }
  }

  return bestLikelihood > 0;
}

void PoseHypothesis2017::calculateSphericalObservation(SphericalObservation<totalDimension> &observation,
  const Vector2d &relativePosition, const Vector2d &globalCoordinates, const float &cameraHeight, const SelfLocator2017Parameters &parameters)
{
  // some helpful variables
  const double p2l_x = globalCoordinates.x() - state[0]; // pose to landmark, x
  const double p2l_y = globalCoordinates.y() - state[1]; // pose to landmark, y
  const double d2 = p2l_x * p2l_x + p2l_y * p2l_y;
  const double d = sqrt(d2);
  const double cameraHeight2 = static_cast<double>(cameraHeight) * cameraHeight;

  // calculate observed angles
  observation.realAngles.x() = atan2(cameraHeight, relativePosition.norm()); // vertical angle
  observation.realAngles.y() = relativePosition.angle(); // horizontal angle

  // calculate expected angles
  observation.nominalAngles.x() = atan2(cameraHeight, d); // vertical angle
  observation.nominalAngles.y() = Angle::normalize(atan2(p2l_y, p2l_x) - state[2]); // horizontal angle

  // calculate angleObservationMeasurementModelJacobian_H
  observation.measurementModelJacobian(0, 0) = cameraHeight * p2l_x / (d*(cameraHeight2 + d2));
  observation.measurementModelJacobian(0, 1) = cameraHeight * p2l_y / (d*(cameraHeight2 + d2));
  observation.measurementModelJacobian(0, 2) = 0;
  observation.measurementModelJacobian(1, 0) = p2l_y / d2;
  observation.measurementModelJacobian(1, 1) = -p2l_x / d2;
  observation.measurementModelJacobian(1, 2) = -1;

  // Weight of the observation (far away observations will might have more error due to camera matrix, etc...)
  const double fullWeightAngle = parameters.sensorUpdate.maxVerticalAngleFullObservationWeight;
  observation.weight = observation.nominalAngles.x() / (pi_2 - fullWeightAngle);
  observation.weight = std::min(observation.weight, 1.);
}

void PoseHypothesis2017::calculateInfiniteLineObservation(InfiniteLineObservation<totalDimension> &observation, const LineMatchingResult::FieldLine &observedLine,
  const LineMatchingResult::FieldLine &correspondingFieldLineSegment, const SelfLocator2017Parameters &parameters)
{
  Pose2f pose;
  getRobotPose(pose);
  Vector2f localStart = Transformation::fieldToRobot(pose, correspondingFieldLineSegment.start.cast<float>());
  Vector2f localEnd = Transformation::fieldToRobot(pose, correspondingFieldLineSegment.end.cast<float>());
  Vector3d toStart(localStart.x(), localStart.y(), -observedLine.cameraHeight), toEnd(localEnd.x(), localEnd.y(), -observedLine.cameraHeight);

  // Calculate measurement 
  observation.realNormals = Vector3d(observedLine.start.x(), observedLine.start.y(), -observedLine.cameraHeight).cross(
    Vector3d(observedLine.end.x(), observedLine.end.y(), -observedLine.cameraHeight));

  // calculate expected measurement
  observation.nominalNormals = toStart.cross(toEnd);

  // calculate infiniteLineMeasurementModelJacobian_H
  if (parameters.processUpdate.jacobianMeasurementCalculation == SelfLocator2017Parameters::ProcessUpdate::vector)
  {
    // Helpers
    Vector2d wcsDiff = correspondingFieldLineSegment.end - correspondingFieldLineSegment.start;

    Vector3d wcsNormal = Vector3d(correspondingFieldLineSegment.start.x(), correspondingFieldLineSegment.start.y(), -observedLine.cameraHeight).cross(
      Vector3d(correspondingFieldLineSegment.end.x(), correspondingFieldLineSegment.end.y(), -observedLine.cameraHeight));

    double lengthRatio = 1 / (wcsNormal.norm() + wcsDiff.norm());
    double lengthRatio3 = lengthRatio * lengthRatio * lengthRatio;

    // d_nx / d_px:
    observation.measurementModelJacobian(0, 0) = wcsDiff.y() * observation.nominalNormals.x() * observation.nominalNormals.z() * lengthRatio3;
    // d_nx / d_py:
    observation.measurementModelJacobian(0, 1) = wcsDiff.x() * observation.nominalNormals.x() * observation.nominalNormals.z() * lengthRatio3;
    // d_nx / d_ptheta:
    observation.measurementModelJacobian(0, 2) = observation.nominalNormals.y() * lengthRatio;
    // d_ny / d_px:
    observation.measurementModelJacobian(1, 0) = wcsDiff.y() * observation.nominalNormals.y() * observation.nominalNormals.z() * lengthRatio3;
    // d_ny / d_py:
    observation.measurementModelJacobian(1, 1) = wcsDiff.x() * observation.nominalNormals.y() * observation.nominalNormals.z() * lengthRatio3;
    // d_ny / d_ptheta:
    observation.measurementModelJacobian(1, 2) = -observation.nominalNormals.x() * lengthRatio;
    // d_nz / d_px:
    observation.measurementModelJacobian(2, 0) = -wcsDiff.y() * lengthRatio;
    // d_nz / d_py:
    observation.measurementModelJacobian(2, 1) = wcsDiff.x() * lengthRatio;
    // d_nz / d_ptheta:
    observation.measurementModelJacobian(2, 2) = 0;
  }
  else
  {
    // some helpful variables
    const double sin_p = sin(state[2]);
    const double cos_p = cos(state[2]);
    const double l_x_1 = correspondingFieldLineSegment.start.x();
    const double l_y_1 = correspondingFieldLineSegment.start.y();
    const double l_x_2 = correspondingFieldLineSegment.end.x();
    const double l_y_2 = correspondingFieldLineSegment.end.y();
    const double p_x = state[0];
    const double p_y = state[1];
    const double cameraHeight2 = observedLine.cameraHeight * observedLine.cameraHeight;
    const double t1 = l_x_1 * l_y_2 - l_x_2 * l_y_1 - l_x_1 * p_y + l_y_1 * p_x + l_x_2 * p_y - l_y_2 * p_x; // Z-Value Start X End in RCS (robot coord) (not normalized)
    const double t2 = cameraHeight2 * (sqr(l_x_1 - l_x_2) + sqr(l_y_1 - l_y_2)) + t1 * t1; // Start X End (WCS) and Z-Value of Start X End (RCS) -> SquaredNorm
    const double t3 = 1 / sqrt(t2); // 1 / Length; For normalization
    const double t4 = t3 * t3*t3;

    // d_nx / d_px:
    observation.measurementModelJacobian(0, 0) = observedLine.cameraHeight * (l_y_1 - l_y_2) *(cos_p*(l_y_1 - l_y_2) - sin_p * (l_x_1 - l_x_2)) *t1 *t4;
    // d_nx / d_py:
    observation.measurementModelJacobian(0, 1) = -observedLine.cameraHeight * (l_x_1 - l_x_2) *(cos_p*(l_y_1 - l_y_2) - sin_p * (l_x_1 - l_x_2)) *t1 *t4;
    // d_nx / d_ptheta:
    observation.measurementModelJacobian(0, 2) = observedLine.cameraHeight * (cos_p*(l_x_1 - l_x_2) + sin_p * (l_y_1 - l_y_2)) * t3;
    // d_ny / d_px:
    observation.measurementModelJacobian(1, 0) = -observedLine.cameraHeight * (l_y_1 - l_y_2) *(cos_p*(l_x_1 - l_x_2) + sin_p * (l_y_1 - l_y_2)) *t1 *t4;
    // d_ny / d_py:
    observation.measurementModelJacobian(1, 1) = observedLine.cameraHeight * (l_x_1 - l_x_2) *(cos_p*(l_x_1 - l_x_2) + sin_p * (l_y_1 - l_y_2)) *t1 *t4;
    // d_ny / d_ptheta:
    observation.measurementModelJacobian(1, 2) = observedLine.cameraHeight * (cos_p*(l_y_1 - l_y_2) - sin_p * (l_x_1 - l_x_2)) * t3;
    // d_nz / d_px:
    observation.measurementModelJacobian(2, 0) = cameraHeight2 * (l_y_1 - l_y_2) * (sqr(l_x_1 - l_x_2) + sqr(l_y_1 - l_y_2)) * t4;
    // d_nz / d_py:
    observation.measurementModelJacobian(2, 1) = -cameraHeight2 * (l_x_1 - l_x_2) * (sqr(l_x_1 - l_x_2) + sqr(l_y_1 - l_y_2)) * t4;
    // d_nz / d_ptheta:
    observation.measurementModelJacobian(2, 2) = 0;
  }

  // Normalize normals
  observation.realNormals.normalize();
  observation.nominalNormals.normalize();

  // Weight of the observation (far away observations will might have more error due to camera matrix, etc...)
  // some helpful variables
  const Vector2d poseXY(state[0], state[1]);
  const double dStart = (correspondingFieldLineSegment.start - poseXY).norm();
  const double dEnd = (correspondingFieldLineSegment.end - poseXY).norm();
  const double angleStart = atan2(observedLine.cameraHeight, dStart);
  const double angleEnd = atan2(observedLine.cameraHeight, dEnd);
  const double fullWeightAngle = parameters.sensorUpdate.maxVerticalAngleFullObservationWeight;
  // Clip
  double weightStart = angleStart / (pi_2 - fullWeightAngle);
  double weightEnd = angleEnd / (pi_2 - fullWeightAngle);
  weightStart = std::min(weightStart, 1.);
  weightEnd = std::min(weightEnd, 1.);
  observation.weight = ((weightStart + weightEnd) / 2);
}

void PoseHypothesis2017::updateSymmetryByComparingRemoteToLocalModels(
  const BallModel & theBallModel,
  const RemoteBallModel & theRemoteBallModel,
  const LocalRobotMap & theLocalRobotMap,
  const RemoteRobotMap & theRemoteRobotMap,
  const TeammateData & theTeammateData,
  const FrameInfo & frameInfo,
  const SelfLocator2017Parameters & parameters)
{
  // the maps and models only contain the means, not the covariances
  // so we can only compare the differences, not the likelihoods (which would make more sense)
  float difference = 0;
  float differenceOfMirrored = 0;

  const float minDiffToMatter = 500.f;

  Vector2f localModelInGlobalCoords, localModelInGlobalCoordsMirrored;
  Pose2f pose;
  getRobotPose(pose);

  // Check ball
  if (frameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 1000
    && frameInfo.getTimeSince(theRemoteBallModel.timeWhenLastSeen) < 1500
    && theBallModel.validity > 0.7f && theRemoteBallModel.validity > 0.8f
    && theBallModel.estimate.velocity.norm() < 100 && theRemoteBallModel.velocity.norm() < 100)
  {
    localModelInGlobalCoords = pose * theBallModel.estimate.position;
    difference = (theRemoteBallModel.position - localModelInGlobalCoords).norm();
    localModelInGlobalCoordsMirrored = localModelInGlobalCoords * -1;
    differenceOfMirrored = (theRemoteBallModel.position - localModelInGlobalCoordsMirrored).norm();

    const bool isThisPositionMoreLikely = difference < differenceOfMirrored;
    const float& closestDifference = isThisPositionMoreLikely ? difference : differenceOfMirrored;

    if (std::abs(difference - differenceOfMirrored) > minDiffToMatter
      && closestDifference < parameters.symmetryUpdate.maxDistanceToClosestRemoteModel)
    {
      std::vector<int> teammates;
      extractTeammateNumbers(theRemoteBallModel.teammates, teammates);

      for (const auto &tmNumber : teammates)
      {
        const Teammate *teammate = getTeammateByNumber(theTeammateData, tmNumber);
        if (teammate)
        {
          const float &baseConfidence = teammate->isGoalkeeper ? parameters.symmetryUpdate.influenceOfNewBallMeasurementByGoalie : parameters.symmetryUpdate.influenceOfNewBallMeasurement;
          const float stateValue = 1.f - static_cast<float>(teammate->sideConfidence.confidenceState) / SideConfidence::numOfConfidenceStates;
          updateSymmetryConfidence(isThisPositionMoreLikely, baseConfidence * stateValue * teammate->sideConfidence.sideConfidence);
        }
      }
    }
  }

  // Use only ball for symmetry
  // Works better for now
  return;

  // Update based on robot map
  difference = 0;
  differenceOfMirrored = 0;

  float smallestDiff;
  float smallestDiffOfMirrored;

  const float bigDiff = 1000000.f;


  // test if own pose is somewhere in the remote robot map
  smallestDiff = bigDiff;
  smallestDiffOfMirrored = bigDiff;
  localModelInGlobalCoords = pose.translation;
  localModelInGlobalCoordsMirrored = localModelInGlobalCoords * -1;
  for (const auto &robot : theRemoteRobotMap.robots)
  {
    float tempDiff = (localModelInGlobalCoords - robot.pose.translation).norm();
    if (tempDiff < smallestDiff)
    {
      smallestDiff = tempDiff * robot.validity;
    }
    tempDiff = (localModelInGlobalCoordsMirrored - robot.pose.translation).norm();
    if (tempDiff < smallestDiffOfMirrored)
    {
      smallestDiffOfMirrored = tempDiff * robot.validity;
    }
  }
  if (smallestDiff < minDiffToMatter || smallestDiffOfMirrored < minDiffToMatter)
  {
    difference += (smallestDiff * parameters.symmetryUpdate.influenceOfNewTeammateRobotMeasurement);
    differenceOfMirrored += (smallestDiffOfMirrored * parameters.symmetryUpdate.influenceOfNewTeammateRobotMeasurement);
  }


  // test other local observations
  for (const auto &local : theLocalRobotMap.robots)
  {
    smallestDiff = bigDiff;
    smallestDiffOfMirrored = bigDiff;
    localModelInGlobalCoords = local.pose.translation;
    localModelInGlobalCoordsMirrored = localModelInGlobalCoords * -1;
    
    const RobotMapEntry *smallestDiffEntry = 0, *smallestDiffMirroredEntry = 0;
    for (const auto &remote : theRemoteRobotMap.robots)
    {
      float tempDiff = (localModelInGlobalCoords - remote.pose.translation).norm();
      if (tempDiff < smallestDiff)
      {
        smallestDiff = tempDiff * remote.validity;
        smallestDiffEntry = &remote;
      }
      tempDiff = (localModelInGlobalCoordsMirrored - remote.pose.translation).norm();
      if (tempDiff < smallestDiffOfMirrored)
      {
        smallestDiffOfMirrored = tempDiff * remote.validity;
        smallestDiffMirroredEntry = &remote;
      }
    }

    if (smallestDiffEntry && smallestDiffMirroredEntry)
    {
      const bool isTeammate = (local.robotType == RobotEstimate::teammateRobot
        || smallestDiffEntry->robotType == RobotEstimate::teammateRobot
        || smallestDiffMirroredEntry->robotType == RobotEstimate::teammateRobot);

      const float influence = isTeammate ?
        parameters.symmetryUpdate.influenceOfNewTeammateRobotMeasurement :
        parameters.symmetryUpdate.influenceOfNewOpponentRobotMeasurement;

      difference += (smallestDiff * influence);
      differenceOfMirrored += (smallestDiffOfMirrored * influence);
    }
  }

  float diff = std::abs(difference - differenceOfMirrored);
  if (diff > (minDiffToMatter * parameters.symmetryUpdate.influenceOfNewTeammateRobotMeasurement))
  {
    float influence = (diff / (difference + differenceOfMirrored));
    updateSymmetryConfidence(difference < differenceOfMirrored, influence);
  }
}

void PoseHypothesis2017::draw(ColorRGBA myselfColor) const
{
  GaussianDistribution2D gd;

  extractGaussianDistribution2DFromStateEstimation(gd);
  drawGaussian(gd, myselfColor, 50, true);

  Pose2f pose;
  getRobotPose(pose);

  POSE_2D_SAMPLE("module:SelfLocator2017:hypotheses", pose, myselfColor);

  DRAWTEXT("module:SelfLocator2017:hypotheses", pose.translation.x(), pose.translation.y() - 180, 50, myselfColor, "pc: " << positionConfidence);
  DRAWTEXT("hypotheses", pose.translation.x(), pose.translation.y() - 300, 50, myselfColor, "sc: " << symmetryConfidence);
}

void PoseHypothesis2017::drawGaussian(const GaussianDistribution2D &gd, const ColorRGBA &color, int penSize, bool drawCenter) const
{
  // TODO: clean up this method
  Pose2f mean = Pose2f(gd.mean.cast<float>());
  double averageX = mean.translation.x();
  double averageY = mean.translation.y();


  if (drawCenter)
  {
    CIRCLE("module:SelfLocator2017:GaussianTools:covarianceEllipse",
      averageX, averageY, 60, 20, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, color);
  }

  if (gd.mean.norm() > 20000 || gd.covariance.col(0).norm() > 10000. * 10000. || gd.covariance.col(1).norm() > 10000. * 10000.)
  {
    return;
  }

  // Vector2<double> eigenVec0 = getEigenVectors(gaussian.covariance).c[0]; 
  Eigen::EigenSolver<Matrix2d> es(gd.covariance); //constructor calls compute() and calculates eigenvectors and values
  Vector2d eigenVec0 = es.eigenvectors().col(0).real();
  //Vector2d eigenVec1 = es.eigenvectors().col(1).real();


  //double temp2 = toDegrees(gaussian.covariance.c[0].angle());
  //double temp = toDegrees(eigenVec0.angle());

  Matrix2d eigenVals = es.eigenvalues().real().asDiagonal();
  double eigenVal0 = sgn(eigenVals.col(0).x())*sqrt(std::abs(eigenVals.col(0).x()));   // std-deviation to make it drawable on the field
  double eigenVal1 = sgn(eigenVals.col(1).y())*sqrt(std::abs(eigenVals.col(1).y()));


  // comment out to watch eigenvectors
  //Vector2<double> eigenDraw1, eigenDraw2;
  //eigenDraw1 = eigenVec0.normalize() * eigenVal0;
  //eigenDraw2 = eigenVec1.normalize() * eigenVal1;

  //LINE("GaussianTools:covarianceEllipse", averageX, averageY,averageX+eigenDraw1.x,averageY+eigenDraw1.y,20,Drawings::ps_solid,color);
  //LINE("GaussianTools:covarianceEllipse", averageX, averageY,averageX+eigenDraw2.x,averageY+eigenDraw2.y,20,Drawings::ps_solid,color);

  COMPLEX_DRAWING("module:SelfLocator2017:GaussianTools:covarianceEllipse")
  {
    double alpha = atan2(eigenVec0.y(), eigenVec0.x());
    double a = eigenVal0;
    double b = eigenVal1;
    double x1 = 0;
    double x2 = 0;
    double y1 = 0;
    double y2 = 0;

    int step = (int)(a / 10);
    if (!step)
    {
      step = 1;
    }

    for (int xc = 0; (static_cast<double>(xc) + step) < a; xc += step)
    {
      x1 = xc;
      y1 = (sqrt(1 - sqr(x1 / a))* b);
      x2 = static_cast<double>(xc) + step;
      y2 = (sqrt(1 - sqr(x2 / a))* b);


      LINE("module:SelfLocator2017:GaussianTools:covarianceEllipse", cos(alpha)*(x1)+averageX - sin(alpha)*(y1),
        sin(alpha)*(x1)+cos(alpha)*(y1)+averageY,
        cos(alpha)*(x2)+averageX - sin(alpha)*(y2),
        sin(alpha)*(x2)+cos(alpha)*(y2)+averageY,
        penSize, Drawings::solidPen,
        color);
      LINE("module:SelfLocator2017:GaussianTools:covarianceEllipse", cos(alpha)*(x1)+averageX - sin(alpha)*(-y1),
        sin(alpha)*(x1)+cos(alpha)*(-y1) + averageY,
        cos(alpha)*(x2)+averageX - sin(alpha)*(-y2),
        sin(alpha)*(x2)+cos(alpha)*(-y2) + averageY,
        penSize, Drawings::solidPen,
        color);
      LINE("module:SelfLocator2017:GaussianTools:covarianceEllipse", cos(alpha)*(-x1) + averageX - sin(alpha)*(y1),
        sin(alpha)*(-x1) + cos(alpha)*(y1)+averageY,
        cos(alpha)*(-x2) + averageX - sin(alpha)*(y2),
        sin(alpha)*(-x2) + cos(alpha)*(y2)+averageY,
        penSize, Drawings::solidPen,
        color);
      LINE("module:SelfLocator2017:GaussianTools:covarianceEllipse", cos(alpha)*(-x1) + averageX - sin(alpha)*(-y1),
        sin(alpha)*(-x1) + cos(alpha)*(-y1) + averageY,
        cos(alpha)*(-x2) + averageX - sin(alpha)*(-y2),
        sin(alpha)*(-x2) + cos(alpha)*(-y2) + averageY,
        penSize, Drawings::solidPen,
        color);
    }

    x1 = a;
    y1 = 0;

    LINE("module:SelfLocator2017:GaussianTools:covarianceEllipse", cos(alpha)*(x1)+averageX - sin(alpha)*(y1),
      sin(alpha)*(x1)+cos(alpha)*(y1)+averageY,
      cos(alpha)*(x2)+averageX - sin(alpha)*(y2),
      sin(alpha)*(x2)+cos(alpha)*(y2)+averageY,
      penSize, Drawings::solidPen,
      color);
    LINE("module:SelfLocator2017:GaussianTools:covarianceEllipse", cos(alpha)*(x1)+averageX - sin(alpha)*(-y1),
      sin(alpha)*(x1)+cos(alpha)*(-y1) + averageY,
      cos(alpha)*(x2)+averageX - sin(alpha)*(-y2),
      sin(alpha)*(x2)+cos(alpha)*(-y2) + averageY,
      penSize, Drawings::solidPen,
      color);
    LINE("module:SelfLocator2017:GaussianTools:covarianceEllipse", cos(alpha)*(-x1) + averageX - sin(alpha)*(y1),
      sin(alpha)*(-x1) + cos(alpha)*(y1)+averageY,
      cos(alpha)*(-x2) + averageX - sin(alpha)*(y2),
      sin(alpha)*(-x2) + cos(alpha)*(y2)+averageY,
      penSize, Drawings::solidPen,
      color);
    LINE("module:SelfLocator2017:GaussianTools:covarianceEllipse", cos(alpha)*(-x1) + averageX - sin(alpha)*(-y1),
      sin(alpha)*(-x1) + cos(alpha)*(-y1) + averageY,
      cos(alpha)*(-x2) + averageX - sin(alpha)*(-y2),
      sin(alpha)*(-x2) + cos(alpha)*(-y2) + averageY,
      penSize, Drawings::solidPen,
      color);
  }
}

// Do not call this, it writes over array boundaries!
#if 0
bool PoseHypothesis2017::test()
{
  bool result = true;

  Vector3<double> stateBefore(-1500, -1500, 0), stateAfter1, stateAfter2, stateAfter3, stateAfter4;
  state[0] = stateBefore.x();
  state[1] = stateBefore.y();
  state[2] = stateBefore.z;
  covariance.setToIdentity();
  covariance *= 100000.0;
  covariance.setMember(3, 3, 10);

  Vector2<double> observedStartForZeroAngle(1000, 500);
  Vector2<double> observedEndForZeroAngle(1000, -500);
  Vector2<double> observedStart = observedStartForZeroAngle;
  Vector2<double> observedEnd = observedEndForZeroAngle;
  Vector2<double> correspondenceStart(0, 500 - 1500);
  Vector2<double> correspondenceEnd(0, -500 - 1500);

  std::vector<Vector3<double> > observationsAsNormals;
  std::vector<Vector2<double> > correspondencesInGlobalCoords;

  // update test 1
  observationsAsNormals.clear();
  correspondencesInGlobalCoords.clear();

  Vector3<double> observedProjectiveGeometryNormal = Vector3<double>(observedStart.x,
    observedStart.y,
    -cameraHeight)
    ^
    Vector3<double>(observedEnd.x,
      observedEnd.y,
      -cameraHeight);
  observedProjectiveGeometryNormal.normalize();
  observationsAsNormals.push_back(observedProjectiveGeometryNormal);
  correspondencesInGlobalCoords.push_back(correspondenceStart);
  correspondencesInGlobalCoords.push_back(correspondenceEnd);

  CALL_KALMAN_INFINITE_LINE_UPDATE_FUNCTION(3, observationsAsNormals, correspondencesInGlobalCoords);

  result = result && (state[0] > stateBefore.x);
  stateAfter1.x = state[0];
  stateAfter1.y = state[1];
  stateAfter1.z = state[2];


  // update test 2
  stateBefore.z = pi_2;
  observedStart = observedStartForZeroAngle;
  observedEnd = observedEndForZeroAngle;
  observedStart.rotateRight();
  observedEnd.rotateRight();
  state[0] = stateBefore.x;
  state[1] = stateBefore.y;
  state[2] = stateBefore.z;
  covariance.setToIdentity();
  covariance *= 100000.0;
  covariance.setMember(3, 3, 10);
  observationsAsNormals.clear();
  correspondencesInGlobalCoords.clear();

  observedProjectiveGeometryNormal = Vector3<double>(observedStart.x,
    observedStart.y,
    -cameraHeight)
    ^
    Vector3<double>(observedEnd.x,
      observedEnd.y,
      -cameraHeight);
  observedProjectiveGeometryNormal.normalize();

  observationsAsNormals.push_back(observedProjectiveGeometryNormal);
  correspondencesInGlobalCoords.push_back(correspondenceStart);
  correspondencesInGlobalCoords.push_back(correspondenceEnd);

  CALL_KALMAN_INFINITE_LINE_UPDATE_FUNCTION(3, observationsAsNormals, correspondencesInGlobalCoords);

  result = result && (state[0] > stateBefore.x);
  stateAfter2.x = state[0];
  stateAfter2.y = state[1];
  stateAfter2.z = state[2];


  // update test 3
  stateBefore.z = -pi_2;
  observedStart = observedStartForZeroAngle;
  observedEnd = observedEndForZeroAngle;
  observedStart.rotateLeft();
  observedEnd.rotateLeft();
  state[0] = stateBefore.x;
  state[1] = stateBefore.y;
  state[2] = stateBefore.z;
  covariance.setToIdentity();
  covariance *= 100000.0;
  covariance.setMember(3, 3, 10);
  observationsAsNormals.clear();
  correspondencesInGlobalCoords.clear();

  observedProjectiveGeometryNormal = Vector3<double>(observedStart.x,
    observedStart.y,
    -cameraHeight)
    ^
    Vector3<double>(observedEnd.x,
      observedEnd.y,
      -cameraHeight);
  observedProjectiveGeometryNormal.normalize();

  observationsAsNormals.push_back(observedProjectiveGeometryNormal);
  correspondencesInGlobalCoords.push_back(correspondenceStart);
  correspondencesInGlobalCoords.push_back(correspondenceEnd);

  CALL_KALMAN_INFINITE_LINE_UPDATE_FUNCTION(3, observationsAsNormals, correspondencesInGlobalCoords);

  result = result && (state[0] > stateBefore.x);
  stateAfter3.x = state[0];
  stateAfter3.y = state[1];
  stateAfter3.z = state[2];


  // update test 4
  double angle = pi_2 - 0.01;
  stateBefore.z = angle;
  observedStart = observedStartForZeroAngle;
  observedEnd = observedEndForZeroAngle;
  observedStart.rotate(-angle);
  observedEnd.rotate(-angle);
  state[0] = stateBefore.x;
  state[1] = stateBefore.y;
  state[2] = stateBefore.z;
  covariance.setToIdentity();
  covariance *= 100000.0;
  covariance.setMember(3, 3, 10);
  observationsAsNormals.clear();
  correspondencesInGlobalCoords.clear();

  observedProjectiveGeometryNormal = Vector3<double>(observedStart.x,
    observedStart.y,
    -cameraHeight)
    ^
    Vector3<double>(observedEnd.x,
      observedEnd.y,
      -cameraHeight);
  observedProjectiveGeometryNormal.normalize();

  observationsAsNormals.push_back(observedProjectiveGeometryNormal);
  correspondencesInGlobalCoords.push_back(correspondenceStart);
  correspondencesInGlobalCoords.push_back(correspondenceEnd);

  CALL_KALMAN_INFINITE_LINE_UPDATE_FUNCTION(3, observationsAsNormals, correspondencesInGlobalCoords);

  result = result && (state[0] > stateBefore.x);
  stateAfter4.x = state[0];
  stateAfter4.y = state[1];
  stateAfter4.z = state[2];


  return result;
}




#endif
