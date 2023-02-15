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
  if constexpr (Build::targetRobot())
  {
    InTextFile walkCalibration(File::getPersistentDir() + "walkCalibration.value");
    if (walkCalibration.exists())
      walkCalibration >> localWalkCalibration;
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
  DECLARE_PLOT("module:AutoCalibrator:comError.x");

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
    timeInternalStateSwitch = theFrameInfo.time;
    // turn off sensor control
    localWalkCalibration.deactivateSensorControl = true;
    // save current providers, switch provider to self
    oldConfig = ModuleManager::sendModuleRequest({{"MotionRequest", "AutoCalibrator"}, {"HeadControlRequest", "AutoCalibrator"}});
    moduleConfigSwitched = true;
  }

  float bodyGravityErrorX = accDataBuffersX[anglesource].average();
  PLOT("module:AutoCalibrator:bodyGravityError.x", std::abs(bodyGravityErrorX));
  //Angle gyroSum = (std::abs(gyroDataBuffersY[anglesource].sum()) + std::abs(gyroDataBuffersX[anglesource].sum()));
  //PLOT("module:AutoCalibrator:gyroSum", gyroSum.toDegrees());
  //PLOT("module:AutoCalibrator:gyroThreshold", gyroAbsMaxDeviationPerFrame.toDegrees() * IMU_BUFFER_LENGTH);

  Angle gyroVariance = (gyroDataBuffersY[anglesource].getVariance() + gyroDataBuffersX[anglesource].getVariance());
  PLOT("module:AutoCalibrator:gyroVariance", gyroVariance.toDegrees());
  PLOT("module:AutoCalibrator:gyroThreshold", gyroMaxVariance.toDegrees());


  // if robot is stable, check for error, otherwise, let robot come to rest
  if (localWalkCalibration.bodyAngleCalibrated == false && theFrameInfo.getTimeSince(timeCalibrationStart) > 1500 && gyroDataBuffersX[anglesource].full()
      && gyroDataBuffersY[anglesource].full() && gyroVariance < gyroMaxVariance)
  //&& gyroSum < (gyroAbsMaxDeviationPerFrame * IMU_BUFFER_LENGTH))
  {
    if (bodyAngleCalibrationState == BodyAngleCalibrationState::adjustingBodyAngle)
    {
      if (std::abs(bodyGravityErrorX) < targetBodyGravityMaxDiff)
      {
        timeInternalStateSwitch = theFrameInfo.time;
        bodyAngleCalibrationState = BodyAngleCalibrationState::checkBodyAngleWithSensorControl;
      }
      else
      {
        localWalkCalibration.legJointCalibration[Joints::lHipPitch - Joints::lHipYawPitch] -= bodyGravityErrorX * bodyAngleCorrectionP;
        localWalkCalibration.legJointCalibration[Joints::rHipPitch - Joints::lHipYawPitch] -= bodyGravityErrorX * bodyAngleCorrectionP;
        gyroDataBuffersX[anglesource].clear();
        gyroDataBuffersY[anglesource].clear();
      }
    }
    else if (bodyAngleCalibrationState == BodyAngleCalibrationState::checkBodyAngleWithSensorControl)
    {
      // TODO!
      if (theFrameInfo.getTimeSince(timeInternalStateSwitch) < 2000)
        localWalkCalibration.deactivateSensorControl = false;
      else if (std::abs(bodyGravityErrorX) >= targetBodyGravityMaxDiff)
      {
        timeInternalStateSwitch = theFrameInfo.time;
        bodyAngleCalibrationState = BodyAngleCalibrationState::adjustingCoM;
      }
      else
        localWalkCalibration.bodyAngleCalibrated = true;
    }
    else if (bodyAngleCalibrationState == BodyAngleCalibrationState::adjustingCoM)
    {
      // TODO
      localWalkCalibration.bodyAngleCalibrated = true;
    }
  }

  if (theFrameInfo.getTimeSince(timeCalibrationStart) > 40000)
  {
    localWalkCalibration.bodyAngleCalibrated = true; // TODO: failed state needed
    SystemCall::text2Speech("Timed out walk calibration!");
    OUTPUT_TEXT("!!!Timed out walk calibration!!!");
  }


  if (std::max(theFrameInfo.getTimeSince(timeInternalStateSwitch), theFrameInfo.getTimeSince(timeCalibrationStart)) > 3000) // minimal time for calibration
  {
    if (theBehaviorData.behaviorState != BehaviorData::calibrateBody || localWalkCalibration.bodyAngleCalibrated)
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

      OUTPUT_TEXT("Calibrated Walk in " << theFrameInfo.getTimeSince(timeCalibrationStart) / 1000.f << "s");
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
