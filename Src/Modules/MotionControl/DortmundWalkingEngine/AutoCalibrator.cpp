#include "AutoCalibrator.h"
#include "Tools/Settings.h"
#include "Tools/Motion/ForwardKinematic.h"
#include "Platform/File.h"
#include "Tools/Build.h"

AutoCalibrator::AutoCalibrator()
{
  for (int i = 0; i < JoinedIMUData::numOfInertialDataSources; i++)
  {
    localWalkCalibration.imuAngleOffsets[i].fill(0_deg);
  }
}

AutoCalibrator::~AutoCalibrator()
{
  if constexpr (Build::targetRobot())
  {
    OutTextFile walkCalibration(File::getPersistentDir() + "walkCalibration.value");
    if (walkCalibration.exists())
      walkCalibration << localWalkCalibration;
  }
}

void AutoCalibrator::reset()
{
  timeCalibrationStart = theFrameInfo.time;
  calibrationState = CalibrationState::inactive;
  localWalkCalibration = WalkCalibration();
  for (int i = 0; i < JoinedIMUData::numOfInertialDataSources; i++)
  {
    gyroDataBuffersX[i].fill(theJoinedIMUData.imuData[i].gyro.x());
    gyroDataBuffersY[i].fill(theJoinedIMUData.imuData[i].gyro.y());
    gyroDataBuffersZ[i].fill(theJoinedIMUData.imuData[i].gyro.z());
    accDataBuffersX[i].fill(theJoinedIMUData.imuData[i].acc.x());
    accDataBuffersY[i].fill(theJoinedIMUData.imuData[i].acc.y());
    accDataBuffersZ[i].fill(theJoinedIMUData.imuData[i].acc.z());
    angleDataBuffersX[i].fill(theJoinedIMUData.imuData[i].angle.x());
    angleDataBuffersY[i].fill(theJoinedIMUData.imuData[i].angle.y());
  }
  fieldLevelingPoints.clear();

  pressedHeadFront = false; // todo false
  manualCalibrationTrigger = false;
}

void AutoCalibrator::saveCalibration()
{
  std::list<std::string> names = File::getFullNames("walkCalibration.cfg");
  bool foundFile = false;
  std::string fullPath = "walkCalibration.cfg";
  for (auto& path : names)
  {
    InMapFile file(path.c_str());
    if (file.exists())
    {
      foundFile = true;
      fullPath = path;
      break;
    }
  }
  if (foundFile)
  {
    OutMapFile stream(fullPath);
    stream << localWalkCalibration;
  }
  else
    OUTPUT_ERROR("AutoCalibrator: WalkCalibration could not be saved!");
}

void AutoCalibrator::loadCalibration()
{
  std::list<std::string> names = File::getFullNames("walkCalibration.cfg");
  bool foundFile = false;
  std::string fullPath = "walkCalibration.cfg";
  for (auto& path : names)
  {
    InMapFile file(path.c_str());
    if (file.exists())
    {
      foundFile = true;
      fullPath = path;
      break;
    }
  }
  if (foundFile)
  {
    InMapFile stream(fullPath);
    stream >> localWalkCalibration;
  }
  else
  {
    OUTPUT_ERROR("AutoCalibrator: WalkCalibration could not be loaded!");
    localWalkCalibration = WalkCalibration();
  }
  fieldInclinationFromConfig = localWalkCalibration.fieldInclination;

  if constexpr (Build::targetRobot())
  {
    InTextFile walkCalibration(File::getPersistentDir() + "walkCalibration.value");
    if (walkCalibration.exists())
    {
      walkCalibration >> localWalkCalibration;
      if (useFieldInclinationFromFile)
        localWalkCalibration.fieldInclination = fieldInclinationFromConfig;
    }
    else
      OUTPUT_ERROR("AutoCalibrator: WalkCalibration could not be loaded on robot!");
  }
  calibrationLoaded = true;
}

void AutoCalibrator::execute(tf::Subflow&)
{
  DECLARE_PLOT("module:AutoCalibrator:angleError.y");
  DECLARE_PLOT("module:AutoCalibrator:bodyGravityError.x");
  DECLARE_PLOT("module:AutoCalibrator:gyroSum");
  DECLARE_PLOT("module:AutoCalibrator:gyroVariance");
  DECLARE_PLOT("module:AutoCalibrator:gyroThreshold");

  DEBUG_RESPONSE_ONCE("module:AutoCalibrator:resetCalibration")
  {
    reset();
  }

  DEBUG_RESPONSE_ONCE("module:AutoCalibrator:calibrateBodyAngle")
  {
    reset();
    calibrationState = CalibrationState::bodyAngle;
    manualCalibrationTrigger = true;
  }

  DEBUG_RESPONSE_ONCE("module:AutoCalibrator:calibrateWalk")
  {
    reset();
    calibrationState = CalibrationState::walk;
    manualCalibrationTrigger = true;
  }

  DEBUG_RESPONSE_ONCE("module:AutoCalibrator:saveCalibration")
  {
    saveCalibration();
  }

  if (!calibrationLoaded)
    loadCalibration();

  for (int i = 0; i < JoinedIMUData::numOfInertialDataSources; i++)
  {
    gyroDataBuffersX[i].push_front(theJoinedIMUData.imuData[i].gyro.x());
    gyroDataBuffersY[i].push_front(theJoinedIMUData.imuData[i].gyro.y());
    gyroDataBuffersZ[i].push_front(theJoinedIMUData.imuData[i].gyro.z());
    accDataBuffersX[i].push_front(theJoinedIMUData.imuData[i].acc.x());
    accDataBuffersY[i].push_front(theJoinedIMUData.imuData[i].acc.y());
    accDataBuffersZ[i].push_front(theJoinedIMUData.imuData[i].acc.z());
    angleDataBuffersX[i].push_front(theJoinedIMUData.imuData[i].angle.x());
    angleDataBuffersY[i].push_front(theJoinedIMUData.imuData[i].angle.y());
  }

  if (calibrationState == CalibrationState::bodyAngle && theKeySymbols.pressed_and_released[KeyStates::headFront]
      && bodyAngleCalibrationState != BodyAngleCalibrationState::fieldLeveling && !pressedHeadFront)
  {
    pressedHeadFront = true;
    SystemCall::text2Speech("Activated field leveling!");
  }

  // switching of states trigger by behavior
  if (!manualCalibrationTrigger)
  {
    if (theBehaviorData.behaviorState == BehaviorData::calibrateBody)
    {
      if (calibrationState == CalibrationState::inactive)
      {
        reset();
        calibrationState = CalibrationState::bodyAngle;
      }
    }
    else if (theBehaviorData.behaviorState == BehaviorData::calibrateWalk)
    {
      if (calibrationState == CalibrationState::inactive && !localWalkCalibration.walkCalibrated)
        calibrationState = CalibrationState::walk;
    }
    else // TODO: do we need special handling of cancelation by behavior?
      calibrationState = CalibrationState::inactive;
  }

  if (calibrationState == CalibrationState::inactive)
  {
    timeCalibrationStart = theFrameInfo.time;
    if (moduleConfigSwitched) // should not happen, since we only switch during calibration
    {
      OUTPUT_WARNING("AutoCalibrator did not reset module config after calibration!");
      ModuleManager::sendModuleRequest(oldConfig);
      moduleConfigSwitched = false;
    }
  }
  else if (calibrationState == CalibrationState::bodyAngle && !localWalkCalibration.bodyAngleCalibrated)
  {
    calibrateBodyGravity();
  }
  else if (calibrationState == CalibrationState::walk && !localWalkCalibration.walkCalibrated)
    calibrateWalk();

  lastCalibrationState = calibrationState;

  Vector2a fieldInclination;
  if (theRobotPose.validity > 0.4f)
  {
    fieldInclination = localWalkCalibration.fieldInclination;
    fieldInclination.rotate(theRobotPose.rotation + (mirrorFieldInclination ? 180_deg : 0_deg));
  }
  else
  {
    fieldInclination = Vector2a::Zero();
  }
  const Rangef exponentialFactorLimit(0.005f, 1.f);
  float exponentialFactor = exponentialFactorLimit.limit((theRobotPose.validity - 0.4f) / 10.f);

  fieldInclination = (1 - exponentialFactor) * lastFieldInclination + exponentialFactor * fieldInclination;
  localWalkCalibration.legJointCalibration[Joints::lAnkleRoll - Joints::lHipYawPitch] = -fieldInclination.x();
  localWalkCalibration.legJointCalibration[Joints::lAnklePitch - Joints::lHipYawPitch] = fieldInclination.y();
  localWalkCalibration.legJointCalibration[Joints::rAnkleRoll - Joints::lHipYawPitch] = -fieldInclination.x();
  localWalkCalibration.legJointCalibration[Joints::rAnklePitch - Joints::lHipYawPitch] = fieldInclination.y();

  lastFieldInclination = fieldInclination;
}

void AutoCalibrator::update(WalkCalibration& walkCalibration)
{
  walkCalibration = localWalkCalibration;
}

void AutoCalibrator::update(MotionRequest& motionRequest)
{
  if (calibrationState == CalibrationState::inactive)
  {
    motionRequest.motion = MotionRequest::walk;
    motionRequest.walkRequest = WalkRequest();
  }
  if (calibrationState == CalibrationState::bodyAngle)
  {
    motionRequest.motion = MotionRequest::walk;
    motionRequest.walkRequest = WalkRequest();
  }
  if (calibrationState == CalibrationState::walk)
  {
    // TODO:
    motionRequest.motion = MotionRequest::walk;
    motionRequest.walkRequest = WalkRequest();
  }
}

void AutoCalibrator::update(HeadControlRequest& headControlRequest)
{
  if (calibrationState == CalibrationState::inactive)
  {
    headControlRequest.controlType = HeadControlRequest::direct;
    headControlRequest.pan = 0_deg;
    headControlRequest.tilt = 20_deg;
  }
  if (calibrationState == CalibrationState::bodyAngle)
  {
    headControlRequest.controlType = HeadControlRequest::direct;
    headControlRequest.pan = 0_deg;
    headControlRequest.tilt = 20_deg;
    if (bodyAngleCalibrationState == BodyAngleCalibrationState::fieldLeveling && fieldLevelingState != FieldLevelingState::adjustingFootPitch)
    {
      headControlRequest.controlType = HeadControlRequest::localize;
    }
  }
  if (calibrationState == CalibrationState::walk)
  {
    // TODO: move head randomly vs controlled?
    // TODO: for example up and down to stress test tilt sensor control
    // TODO: also add arm back movement in the future -> provide arm contact
    headControlRequest.controlType = HeadControlRequest::localize;
  }
}

void AutoCalibrator::calibrateBodyGravity()
{
  if (lastCalibrationState != calibrationState)
  {
    bodyAngleCalibrationState = BodyAngleCalibrationState::adjustingBodyAngle;
    fieldLevelingState = FieldLevelingState::waitingForManualPositioning;
    fieldLevelingPoints.clear();

    timeInternalStateSwitch = theFrameInfo.time;
    // turn off sensor control
    localWalkCalibration.deactivateSensorControl = true;
    // save current providers, switch provider to self
    oldConfig = ModuleManager::sendModuleRequest({{"MotionRequest", "AutoCalibrator"}, {"HeadControlRequest", "AutoCalibrator"}});
    moduleConfigSwitched = true;
  }

  float bodyGravityErrorX = accDataBuffersX[anglesource].average();
  PLOT("module:AutoCalibrator:bodyGravityError.x", std::abs(bodyGravityErrorX));

  Angle gyroVariance = (gyroDataBuffersY[anglesource].getVariance() + gyroDataBuffersX[anglesource].getVariance());
  PLOT("module:AutoCalibrator:gyroVariance", gyroVariance.toDegrees());
  PLOT("module:AutoCalibrator:gyroThreshold", gyroMaxVariance.toDegrees());

  // if robot is stable, check for error, otherwise, let robot come to rest
  if (localWalkCalibration.bodyAngleCalibrated == false && theFrameInfo.getTimeSince(timeCalibrationStart) > 1500)
  {
    if (bodyAngleCalibrationState == BodyAngleCalibrationState::adjustingBodyAngle)
    {
      if (gyroDataBuffersX[anglesource].full() && gyroDataBuffersY[anglesource].full() && gyroVariance < gyroMaxVariance)
      {
        if (std::abs(bodyGravityErrorX) < targetBodyGravityMaxDiff)
        {
          if (pressedHeadFront)
          {
            timeInternalStateSwitch = theFrameInfo.time;
            bodyAngleCalibrationState = BodyAngleCalibrationState::fieldLeveling;
          }
          else
          {
            localWalkCalibration.fieldInclination = fieldInclinationFromConfig;
            localWalkCalibration.bodyAngleCalibrated = true;
          }
        }
        else
        {
          localWalkCalibration.legJointCalibration[Joints::lHipPitch - Joints::lHipYawPitch] -= bodyGravityErrorX * bodyGravityCorrectionP;
          localWalkCalibration.legJointCalibration[Joints::rHipPitch - Joints::lHipYawPitch] -= bodyGravityErrorX * bodyGravityCorrectionP;
          gyroDataBuffersX[anglesource].clear();
          gyroDataBuffersY[anglesource].clear();
        }
      }
    }
    else if (bodyAngleCalibrationState == BodyAngleCalibrationState::fieldLeveling)
    {
      if (fieldLevelingState == FieldLevelingState::waitingForManualPositioning)
      {
        if (theKeySymbols.pressed_and_released[KeyStates::headMiddle])
        {
          if (fieldLevelingPoints.size() >= 4)
          {
            Angle pitch = 0_deg;
            int pitchSum = 0;
            Angle roll = 0_deg;
            int rollSum = 0;
            for (auto const& [translation, inclinaton] : fieldLevelingPoints)
            {
              if (inclinaton.y() != 0_deg)
              {
                pitch += inclinaton.y();
                pitchSum++;
              }
              if (inclinaton.x() != 0_deg)
              {
                roll += inclinaton.x();
                rollSum++;
              }
            }

            localWalkCalibration.fieldInclination.x() = roll / rollSum;
            localWalkCalibration.fieldInclination.y() = pitch / pitchSum;

            localWalkCalibration.legJointCalibration[Joints::lAnklePitch - Joints::lHipYawPitch] = 0_deg;
            localWalkCalibration.legJointCalibration[Joints::rAnklePitch - Joints::lHipYawPitch] = 0_deg;

            localWalkCalibration.bodyAngleCalibrated = true;
          }
          else
          {
            SystemCall::text2Speech("Not enough points for field leveling. Needing at least " + std::to_string(4 - fieldLevelingPoints.size()) + " more points!");
          }
        }
        else if (theKeySymbols.pressed_and_released[KeyStates::headFront] || theKeySymbols.pressed_and_released[KeyStates::headRear])
        {
          opponentHalf = theKeySymbols.pressed_and_released[KeyStates::headRear];
          if (opponentHalf)
            SystemCall::text2Speech("Opponent half.");
          else
            SystemCall::text2Speech("Own half.");
          fieldLevelingState = FieldLevelingState::adjustingFootPitch;
        }
      }
      else if (fieldLevelingState == FieldLevelingState::adjustingFootPitch)
      {
        if (gyroDataBuffersX[anglesource].full() && gyroDataBuffersY[anglesource].full() && gyroVariance < gyroMaxVariance)
        {
          if (std::abs(bodyGravityErrorX) < targetBodyGravityMaxDiff)
          {
            fieldLevelingState = FieldLevelingState::waitingForGoodLocalization;
          }
          else
          {
            localWalkCalibration.legJointCalibration[Joints::lAnklePitch - Joints::lHipYawPitch] -= bodyGravityErrorX * bodyGravityCorrectionP;
            localWalkCalibration.legJointCalibration[Joints::rAnklePitch - Joints::lHipYawPitch] -= bodyGravityErrorX * bodyGravityCorrectionP;
            gyroDataBuffersX[anglesource].clear();
            gyroDataBuffersY[anglesource].clear();
          }
        }
      }
      else if (fieldLevelingState == FieldLevelingState::waitingForGoodLocalization)
      {
        if (theRobotPose.validity >= minValidityForRobotPose)
        {
          Angle footPitch = localWalkCalibration.legJointCalibration[Joints::lAnklePitch - Joints::lHipYawPitch];
          Pose2f robotPose = theRobotPose;
          if (opponentHalf)
          {
            robotPose.rotate(180_deg);
            robotPose.translation.x() *= -1.f;
            robotPose.translation.y() *= -1.f;
          }

          Vector2a ankle = Vector2a::Zero();
          Angle rotation = robotPose.rotation;
          bool paralellPosition = true;
          if (std::abs(rotation) < 5_deg)
          {
            ankle.y() = footPitch;
          }
          else if (std::abs(std::abs(rotation) - 180_deg) < 5_deg)
          {
            ankle.y() = -footPitch;
          }
          else if (std::abs(rotation - 90_deg) < 5_deg)
          {
            ankle.x() = -footPitch;
          }
          else if (std::abs(rotation + 90_deg) < 5_deg)
          {
            ankle.x() = footPitch;
          }
          else
            paralellPosition = false;

          if (paralellPosition)
          {
            fieldLevelingPoints.emplace_back(std::make_tuple(robotPose.translation, ankle));
            SystemCall::text2Speech("Now put me in another place.");
            OUTPUT_TEXT("Now put me in another place. (" + std::to_string(fieldLevelingPoints.size()) + "/4)");
          }
          else
          {
            SystemCall::text2Speech("Please put me paralell to the field lines!");
            OUTPUT_TEXT("Please position the robot paralell to the field lines!");
          }

          fieldLevelingState = FieldLevelingState::waitingForManualPositioning;
          localWalkCalibration.legJointCalibration[Joints::lAnklePitch - Joints::lHipYawPitch] = 0_deg;
          localWalkCalibration.legJointCalibration[Joints::rAnklePitch - Joints::lHipYawPitch] = 0_deg;
          gyroDataBuffersX[anglesource].clear();
          gyroDataBuffersY[anglesource].clear();
        }
      }
    }
  }

  if (theFrameInfo.getTimeSince(timeCalibrationStart) > 40000 && !(bodyAngleCalibrationState == BodyAngleCalibrationState::fieldLeveling))
  {
    localWalkCalibration.bodyAngleCalibrated = true; // TODO: failed state needed
    SystemCall::text2Speech("Timed out walk calibration!");
    OUTPUT_TEXT("!!!Timed out walk calibration!!!");
  }

  if (std::max(theFrameInfo.getTimeSince(timeInternalStateSwitch), theFrameInfo.getTimeSince(timeCalibrationStart)) > 3000) // minimal time for calibration
  {
    if ((theBehaviorData.behaviorState != BehaviorData::calibrateBody && !manualCalibrationTrigger) || localWalkCalibration.bodyAngleCalibrated)
    {
      for (int i = 0; i < JoinedIMUData::numOfInertialDataSources; i++)
      {
        localWalkCalibration.imuAngleOffsets[i].x() = angleDataBuffersX[i].average();
        localWalkCalibration.imuAngleOffsets[i].y() = angleDataBuffersY[i].average();
      }
      localWalkCalibration.gravity = accDataBuffersZ[anglesource].average();

      ASSERT(localWalkCalibration.gravity < 12.f && localWalkCalibration.gravity > 7.f && localWalkCalibration.legJointCalibration[Joints::lHipPitch - Joints::lHipYawPitch] < 27.73_deg
          && localWalkCalibration.legJointCalibration[Joints::lHipPitch - Joints::lHipYawPitch] > -88_deg && localWalkCalibration.legJointCalibration[Joints::rHipPitch - Joints::lHipYawPitch] < 27.73_deg
          && localWalkCalibration.legJointCalibration[Joints::rHipPitch - Joints::lHipYawPitch] > -88_deg);

      if (manualCalibrationTrigger)
        SystemCall::text2Speech("Finished calibrating walk!");

      OUTPUT_TEXT("Calibrated Walk in "
          << theFrameInfo.getTimeSince(timeCalibrationStart) / 1000.f << "s and using a hip pitch offset of "
          << localWalkCalibration.legJointCalibration[Joints::lHipPitch - Joints::lHipYawPitch].toDegrees() << "°");
      localWalkCalibration.deactivateSensorControl = false;
      ModuleManager::sendModuleRequest(oldConfig);
      moduleConfigSwitched = false;
      manualCalibrationTrigger = false;
    }
  }
}

void AutoCalibrator::calibrateWalk()
{
  if (lastCalibrationState != calibrationState)
  {
    walkCalibrationState = WalkCalibrationState::adjustingSensorControl;
    // turn off sensor control
    // save current providers, switch provider to self
  }
  // TODO!
  localWalkCalibration.walkCalibrated = true;

  if (theBehaviorData.behaviorState != BehaviorData::calibrateWalk || localWalkCalibration.walkCalibrated)
  {
    // return providers to original
  }
}

MAKE_MODULE(AutoCalibrator, dortmundWalkingEngine)
