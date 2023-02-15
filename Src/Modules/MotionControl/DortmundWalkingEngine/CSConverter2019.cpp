#include "CSConverter2019.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Motion/ForwardKinematic.h"

//#define LOGGING
#include "Tools/Debugging/CSVLogger.h"

using namespace DWE;

CSConverter2019::CSConverter2019()
{
  reset();
}

void CSConverter2019::reset()
{
  isRunning = false;
  speedDependentTilt = 0.f;
  isInstantKickRunning = false;
  footInAir = -1;
  robotPosition = 0.f;
  lastTargetCoM = 0.f;
  lastSpeed = 0.f;
  acc = 0.f;

  odometry = 0.f;
  lastPositionBetweenFeet = 0.f;
  offsetToRobotPoseAfterPreview = 0.f;
  originWCS = 0.f;
  lastTorsoMatrix = Pose3f();

  resetSensorControl();
}

CSConverter2019::~CSConverter2019() {}

void CSConverter2019::update(KinematicRequest& kinematicRequest)
{
  updateKinematicRequest(kinematicRequest);

  PLOT("module:CSConverter2019:TargetCoM.x", theTargetCoM.x);
  PLOT("module:CSConverter2019:TargetCoM.y", theTargetCoM.y);
  PLOT("module:CSConverter2019:Footpos[LEFT_FOOT].x", kinematicRequest.leftFoot[0]);
  PLOT("module:CSConverter2019:Footpos[LEFT_FOOT].y", kinematicRequest.leftFoot[1]);
  PLOT("module:CSConverter2019:Footpos[LEFT_FOOT].z", kinematicRequest.leftFoot[2]);
  PLOT("module:CSConverter2019:Footpos[LEFT_FOOT].r", kinematicRequest.leftFoot[3]);
  PLOT("module:CSConverter2019:Footpos[LEFT_FOOT].rx", kinematicRequest.leftFoot[4]);
  PLOT("module:CSConverter2019:Footpos[LEFT_FOOT].ry", kinematicRequest.leftFoot[5]);
  PLOT("module:CSConverter2019:Footpos[RIGHT_FOOT].x", kinematicRequest.rightFoot[0]);
  PLOT("module:CSConverter2019:Footpos[RIGHT_FOOT].y", kinematicRequest.rightFoot[1]);
  PLOT("module:CSConverter2019:Footpos[RIGHT_FOOT].z", kinematicRequest.rightFoot[2]);
  PLOT("module:CSConverter2019:Footpos[RIGHT_FOOT].r", kinematicRequest.rightFoot[3]);
  PLOT("module:CSConverter2019:Footpos[RIGHT_FOOT].rx", kinematicRequest.rightFoot[4]);
  PLOT("module:CSConverter2019:Footpos[RIGHT_FOOT].ry", kinematicRequest.rightFoot[5]);
  DECLARE_PLOT("module:CSConverter2019:xOffset");
};

void CSConverter2019::clearKinematicRequest(KinematicRequest& kinematicRequest)
{
  for (int i = 2; i < Joints::numOfJoints; i++)
  {
    kinematicRequest.offsets.angles[i] = 0;
  }
  for (int i = 0; i < 6; i++)
  {
    kinematicRequest.leftFoot[i] = JointAngles::ignore;
    kinematicRequest.rightFoot[i] = JointAngles::ignore;
  }
  kinematicRequest.kinematicType = KinematicRequest::feet;
}

KinematicRequest::KinematicType CSConverter2019::determineKinematicType()
{
  if (currentStep.onFloor[LEFT_FOOT] && currentStep.onFloor[RIGHT_FOOT])
  {
    return KinematicRequest::feet;
  }
  else if (currentStep.onFloor[LEFT_FOOT] && !currentStep.onFloor[RIGHT_FOOT])
  {
    return KinematicRequest::bodyAndLeftFoot;
  }
  else if (!currentStep.onFloor[LEFT_FOOT] && currentStep.onFloor[RIGHT_FOOT])
  {
    return KinematicRequest::bodyAndRightFoot;
  }
  else
  {
    return KinematicRequest::feet;
  }
}

void CSConverter2019::determineRunningState()
{
  if (!isRunning && theFootpositions.running && theFootpositions.currentState != goingToReady && theFootpositions.currentState != goingToStandby)
  {
    isRunning = true;
  }

  if (isRunning && (!theFootpositions.running || theFootpositions.currentState == goingToReady || theFootpositions.currentState == goingToStandby))
  {
    reset();
    isRunning = false;
  }
}

void CSConverter2019::applySpeedDependentTilt(Footpositions& fp)
{
  const WalkingEngineParams& curparams = theWalkingEngineParams;
  // Apply speed dependent tilt
  float xFactor = std::abs(fp.speed.translation.x() * 1000)
      / (fp.speed.translation.x() >= 0 ? curparams.speedLimits.xForward * curparams.speedLimits.speedFactor : curparams.speedLimits.xBackward * curparams.speedLimits.speedFactor);
  speedDependentTilt = xFactor * (fp.speed.translation.x() >= 0 ? curparams.comOffsets.tiltSpeedDependent[0] : curparams.comOffsets.tiltSpeedDependent[1]);

  fp.pitch += speedDependentTilt;
  fp.roll += 0;
}

void CSConverter2019::applyComShift(float& xOffset, float& yOffset)
{
  float baseXOffset = xOffset;
  Vector3f rotAcc;
  //if(true) // TODO Parameter
  {
    RotationMatrix rot(theJoinedIMUData.imuData[anglesource].angle.x(), theJoinedIMUData.imuData[anglesource].angle.y(), 0_deg);
    rotAcc = rot * (theJoinedIMUData.imuData[anglesource].acc);
  }
  //else
  //{
  //  rotAcc = theJoinedIMUData.imuData[anglesource].acc;
  //}

  static int zerocount = 0;
  if (theFootpositions.speed == Pose2f() && zerocount <= 100)
    zerocount++;
  else if (!(theFootpositions.speed == Pose2f()))
    zerocount = 0;

  if (zerocount < 100)
  {
    // accXAlpha code
    accXBuffer.push_front(rotAcc.x() * -0.01558f);
    float avgAccX = accXBuffer.average();
    float accFactor = std::abs(accXBuffer[0]) / 0.3f;
    float alphaFactor = comShiftParameters.accXAlpha * accFactor;
    avgAccX = sgn(avgAccX) * std::min<float>(std::abs(avgAccX), 0.03f) - baseXOffset;
    xOffset = (baseXOffset * (1 - alphaFactor) - avgAccX * alphaFactor);
    // end of accXAlpha code
  }

  // addded 4.2019: x offset shift trough actual step acceleration
  float speedX = theFootpositions.speed.translation.x();
  float stepAccX = lastSpeedX - speedX;
  if ((theFootpositions.phase == WalkingPhase::firstSingleSupport || theFootpositions.phase == WalkingPhase::secondSingleSupport) && theFootpositions.frameInPhase == 0)
    interpolStartTime = theFrameInfo.time;

  if (theFootpositions.phase != WalkingPhase::firstSingleSupport && theFootpositions.phase != WalkingPhase::secondSingleSupport)
  {
    lastStepAccX = stepAccX;
    lastSpeedX = speedX;
  }
  interpolRatio = std::min(theFrameInfo.getTimeSince(interpolStartTime) / (float)comShiftParameters.accInterpolTime, 1.f);

  if (stepAccX < 0 && speedX > 0) // only when accelerating forward
    xOffset -= sgn(stepAccX) * std::min(std::abs((lastStepAccX * (1 - interpolRatio) + stepAccX * interpolRatio) * comShiftParameters.stepAccAlpha), 0.05f);

  PLOT("module:CSConverter2019:xOffset", xOffset);
  PLOT("module:CSConverter2019:yOffset", yOffset);
}

Point CSConverter2019::handleArmContactState()
{
  const WalkingEngineParams& curparams = theWalkingEngineParams;

  ArmContact::ArmContactState stateLeftArm = theArmContact.armContactStateLeft;
  ArmContact::ArmContactState stateRightArm = theArmContact.armContactStateRight;
  Point comShift;

  if ((stateLeftArm == ArmContact::ArmContactState::Back || stateLeftArm == ArmContact::ArmContactState::Front) && stateRightArm == ArmContact::ArmContactState::None)
  {
    comShift.y = curparams.comOffsets.yArmContact[0];
    comShift.x = curparams.comOffsets.xArmContact;
  }
  else if ((stateRightArm == ArmContact::ArmContactState::Back || stateRightArm == ArmContact::ArmContactState::Front) && stateLeftArm == ArmContact::ArmContactState::None)
  {
    comShift.y = curparams.comOffsets.yArmContact[1];
    comShift.x = curparams.comOffsets.xArmContact;
  }
  else if ((stateRightArm == ArmContact::ArmContactState::Back && stateLeftArm == ArmContact::ArmContactState::Back)
      || (stateRightArm == ArmContact::ArmContactState::Front && stateLeftArm == ArmContact::ArmContactState::Front))
  {
    comShift.y = 0.f;
    comShift.x = curparams.comOffsets.xArmContact;
  }

  return comShift;
}

void CSConverter2019::applyOffsets(KinematicRequest& kinematicRequest)
{
  const WalkingEngineParams& curparams = theWalkingEngineParams;

  if (footInAir == RIGHT_FOOT)
  {
    for (int i = 0; i < 6; i++)
    {
      kinematicRequest.offsets.angles[i + (int)Joints::lHipYawPitch] = curparams.jointCalibration.offsetLeft[i];
    }
  }
  if (footInAir == LEFT_FOOT)
  {
    for (int i = 0; i < 6; i++)
    {
      kinematicRequest.offsets.angles[i + (int)Joints::rHipYawPitch] = curparams.jointCalibration.offsetRight[i];
    }
  }

  if (useLegJointBalancing && !theWalkCalibration.deactivateSensorControl)
  {
    if (theFallDownState.state != FallDownState::upright || !isRunning)
    {
      resetSensorControl();
      return;
    }

    float newComX = theRobotModel.centerOfMass.x();

    for (int i = Joints::lHipYawPitch; i <= Joints::rAnkleRoll; i++)
      if (kinematicRequest.offsets.angles[i] == JointAngles::ignore || kinematicRequest.offsets.angles[i] == JointAngles::off)
      {
        resetSensorControl();
        return;
      }

    {
      Angle angleErrorX = (legJointBalanceParams.targetAngleX - angleX);
      Angle angleErrorY = (legJointBalanceParams.targetAngleY - angleY);

      Angle angleErrorXAnkle = angleErrorX * legJointBalanceParams.ankleHipRatioX;
      Angle angleErrorXHip = angleErrorX * (1.f - legJointBalanceParams.ankleHipRatioX);
      Angle angleErrorYAnkle = angleErrorY * legJointBalanceParams.ankleHipRatioY;
      Angle angleErrorYHip = angleErrorY * (1.f - legJointBalanceParams.ankleHipRatioX);

      pidAngleXAnkle_sum += angleErrorXAnkle;
      pidAngleXHip_sum += angleErrorXHip;
      pidAngleYAnkle_sum += angleErrorYAnkle;
      pidAngleYHip_sum += angleErrorYHip;

      /*     float anklePIDMultiplicatorX = theMotionRequest.walkRequest.isZeroSpeed() ? 0.f : legJointBalanceParams.ankleParams.pidMultiplicatorX;
      float anklePIDMultiplicatorY = theMotionRequest.walkRequest.isZeroSpeed() ? 0.f : legJointBalanceParams.ankleParams.pidMultiplicatorY;
      float hipPIDMultiplicatorX = theMotionRequest.walkRequest.isZeroSpeed() ? 0.f : legJointBalanceParams.hipParams.pidMultiplicatorX;
      float hipPIDMultiplicatorY = theMotionRequest.walkRequest.isZeroSpeed() ? 0.f : legJointBalanceParams.hipParams.pidMultiplicatorY;*/
      float anklePIDMultiplicatorX, anklePIDMultiplicatorY, hipPIDMultiplicatorX, hipPIDMultiplicatorY;
      if (theSpeedRequest.translation.norm() == 0 && theSpeedRequest.rotation == 0)
      {
        if (angleErrorX < (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[0]) || angleErrorX > (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[1])
            || (angleErrorY < theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0] + 1_deg) || angleErrorY > (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[1] - 1_deg))
        {
          anklePIDMultiplicatorX = legJointBalanceParams.ankleParams.pidMultiplicatorX;
          anklePIDMultiplicatorY = legJointBalanceParams.ankleParams.pidMultiplicatorY;
          hipPIDMultiplicatorX = legJointBalanceParams.hipParams.pidMultiplicatorX;
          hipPIDMultiplicatorY = legJointBalanceParams.hipParams.pidMultiplicatorY;
        }
        else
        {
          anklePIDMultiplicatorX = 0.f;
          anklePIDMultiplicatorY = 0.f;
          hipPIDMultiplicatorX = 0.f;
          hipPIDMultiplicatorY = 0.f;
        }
      }
      else
      {
        anklePIDMultiplicatorX = legJointBalanceParams.ankleParams.pidMultiplicatorX;
        anklePIDMultiplicatorY = legJointBalanceParams.ankleParams.pidMultiplicatorY;
        hipPIDMultiplicatorX = legJointBalanceParams.hipParams.pidMultiplicatorX;
        hipPIDMultiplicatorY = legJointBalanceParams.hipParams.pidMultiplicatorY;
      }
      float filteredErrorX_Ankle = angleErrorXAnkle * legJointBalanceParams.ankleParams.p_x * anklePIDMultiplicatorX + pidAngleXAnkle_sum * legJointBalanceParams.ankleParams.i_x * anklePIDMultiplicatorX
          + (pidAngleXAnkle_last - angleErrorXAnkle) * legJointBalanceParams.ankleParams.d_x * anklePIDMultiplicatorX;
      float filteredErrorY_Ankle = angleErrorYAnkle * legJointBalanceParams.ankleParams.p_y * anklePIDMultiplicatorY + pidAngleYAnkle_sum * legJointBalanceParams.ankleParams.i_y * anklePIDMultiplicatorY
          + (pidAngleYAnkle_last - angleErrorYAnkle) * legJointBalanceParams.ankleParams.d_y * anklePIDMultiplicatorY;

      float filteredErrorX_Hip = angleErrorXHip * legJointBalanceParams.hipParams.p_x * hipPIDMultiplicatorX + pidAngleXHip_sum * legJointBalanceParams.hipParams.i_x * hipPIDMultiplicatorX
          + (pidAngleXHip_last - angleErrorXHip) * legJointBalanceParams.hipParams.d_x * hipPIDMultiplicatorX;
      float filteredErrorY_Hip = angleErrorYHip * legJointBalanceParams.hipParams.p_y * hipPIDMultiplicatorY + pidAngleYHip_sum * legJointBalanceParams.hipParams.i_y * hipPIDMultiplicatorY
          + (pidAngleYHip_last - angleErrorYHip) * legJointBalanceParams.hipParams.d_y * hipPIDMultiplicatorY;

      PLOT("module:MotionCombinator:UpperBodyBalancer:errorX_Ankle", filteredErrorX_Ankle);
      PLOT("module:MotionCombinator:UpperBodyBalancer:errorY_Ankle", filteredErrorY_Ankle);
      PLOT("module:MotionCombinator:UpperBodyBalancer:errorX_Hip", filteredErrorX_Hip);
      PLOT("module:MotionCombinator:UpperBodyBalancer:errorY_Hip", filteredErrorY_Hip);

      pidAngleXAnkle_last = angleErrorXAnkle;
      pidAngleYAnkle_last = angleErrorYAnkle;
      pidAngleXHip_last = angleErrorXHip;
      pidAngleYHip_last = angleErrorYHip;

      // TODO: interpolate towards offsets?
      if (!balanceSupportLegOnly || footInAir == RIGHT_FOOT)
      {
        kinematicRequest.offsets.angles[Joints::lHipPitch] -= filteredErrorY_Hip;
        kinematicRequest.offsets.angles[Joints::lHipRoll] += filteredErrorX_Hip;
        kinematicRequest.offsets.angles[Joints::lAnklePitch] -= filteredErrorY_Ankle;
        kinematicRequest.offsets.angles[Joints::lAnkleRoll] += filteredErrorX_Ankle;
      }
      if (!balanceSupportLegOnly || footInAir == LEFT_FOOT)
      {
        kinematicRequest.offsets.angles[Joints::rHipPitch] -= filteredErrorY_Hip;
        kinematicRequest.offsets.angles[Joints::rHipRoll] += filteredErrorX_Hip;
        kinematicRequest.offsets.angles[Joints::rAnklePitch] -= filteredErrorY_Ankle;
        kinematicRequest.offsets.angles[Joints::rAnkleRoll] += filteredErrorX_Ankle;
      }

      if (!kinematicRequest.offsets.isValid())
      {
        kinematicRequest.offsets = JointAngles();
        {
          std::string logDir = "";
#ifdef TARGET_ROBOT
          logDir = "../logs/";
#endif
          OutMapFile stream(logDir + "balancing.log");
          stream << filteredErrorX_Hip << "\n";
          stream << filteredErrorY_Hip << "\n";
          stream << filteredErrorX_Ankle << "\n";
          stream << filteredErrorY_Ankle << "\n";
        }
      }
      lastComX = newComX;
    }
  }
}

void CSConverter2019::resetSensorControl()
{
  pidAngleXAnkle_sum = 0.f;
  pidAngleXHip_sum = 0.f;
  pidAngleYAnkle_sum = 0.f;
  pidAngleYHip_sum = 0.f;
  pidAngleXAnkle_last = 0.f;
  pidAngleYAnkle_last = 0.f;
  pidAngleXHip_last = 0.f;
  pidAngleYHip_last = 0.f;
  accXBuffer.fill(0);
  lastSpeedX = 0.f;
  lastStepAccX = 0.f;
  interpolRatio = 0;
  interpolStartTime = theFrameInfo.time;
}

void CSConverter2019::updateKinematicRequest(KinematicRequest& kinematicRequest)
{
  Point targetCoM = theTargetCoM;
  Footpositions fp = theFootpositions;
  const WalkingEngineParams& curparams = theWalkingEngineParams;

  clearKinematicRequest(kinematicRequest);
  determineRunningState();
  determineFootInAir(fp);

  gyroX = theJoinedIMUData.imuData[anglesource].gyro.x();
  gyroY = theJoinedIMUData.imuData[anglesource].gyro.y();
  angleX = theJoinedIMUData.imuData[anglesource].angle.x();
  angleY = theJoinedIMUData.imuData[anglesource].angle.y();

  if (isRunning)
  {
    applySpeedDependentTilt(fp);
    isInstantKickRunning = fp.customStepRunning;

    toRobotCoords(&currentStep, targetCoM, fp, theActualCoMRCS);

    LOG("WalkingEngine", "WCS left Foot x", fp.footPos[LEFT_FOOT].x);
    LOG("WalkingEngine", "WCS left Foot y", fp.footPos[LEFT_FOOT].y);
    LOG("WalkingEngine", "WCS left Foot z", fp.footPos[LEFT_FOOT].z);
    LOG("WalkingEngine", "WCS left Foot r", fp.footPos[LEFT_FOOT].r);
    LOG("WalkingEngine", "WCS right Foot x", fp.footPos[RIGHT_FOOT].x);
    LOG("WalkingEngine", "WCS right Foot y", fp.footPos[RIGHT_FOOT].y);
    LOG("WalkingEngine", "WCS right Foot z", fp.footPos[RIGHT_FOOT].z);
    LOG("WalkingEngine", "WCS right Foot r", fp.footPos[RIGHT_FOOT].r);
    LOG("WalkingEngine", "RCS roll", fp.roll);
    LOG("WalkingEngine", "RCS pitch", fp.pitch);

    LOG("WalkingEngine", "RCS left Foot x", currentStep.footPos[LEFT_FOOT].x);
    LOG("WalkingEngine", "RCS left Foot y", currentStep.footPos[LEFT_FOOT].y);
    LOG("WalkingEngine", "RCS left Foot z", currentStep.footPos[LEFT_FOOT].z);
    LOG("WalkingEngine", "RCS left Foot r", currentStep.footPos[LEFT_FOOT].r);
    LOG("WalkingEngine", "RCS right Foot x", currentStep.footPos[RIGHT_FOOT].x);
    LOG("WalkingEngine", "RCS right Foot y", currentStep.footPos[RIGHT_FOOT].y);
    LOG("WalkingEngine", "RCS right Foot z", currentStep.footPos[RIGHT_FOOT].z);

    LOG("WalkingEngine", "actual CoM x", theActualCoMRCS.x);
    LOG("WalkingEngine", "actual CoM y", theActualCoMRCS.y);
    LOG("WalkingEngine", "actual CoM z", theActualCoMRCS.z);
  }
  else
  {
    resetSensorControl();
    currentStep = fp;
    targetCoM = 0;
  }

  Point speed = (targetCoM - lastTargetCoM) / theFrameInfo.cycleTime;
  acc = speed - lastSpeed / theFrameInfo.cycleTime;

  lastTargetCoM = targetCoM;
  lastSpeed = speed;

  kinematicRequest.kinematicType = determineKinematicType();
  float xOffset = theWalkingEngineParams.comOffsets.xFixed + theWalkCalibration.comOffset.x();
  float yOffset = theWalkingEngineParams.comOffsets.yFixed + theWalkCalibration.comOffset.y();
  if (isRunning)
  {
    if (!theWalkCalibration.deactivateSensorControl)
      applyComShift(xOffset, yOffset);
    if (fp.speed.translation.x() > 0)
      xOffset += curparams.comOffsets.xSpeedDependent * fp.speed.translation.x() * 1000 / (curparams.speedLimits.xForward * curparams.speedLimits.speedFactor);
    yOffset += curparams.comOffsets.ySpeedDependent[fp.speed.translation.y() < 0] * std::abs(fp.speed.translation.y() * 1000) / (curparams.speedLimits.y * curparams.speedLimits.speedFactor);
  }
  Point comShift = handleArmContactState();


  // position left foot
  kinematicRequest.leftFoot[0] = (float)(currentStep.footPos[LEFT_FOOT].x * 1000 - xOffset * 1000 - comShift.x * 1000);
  kinematicRequest.leftFoot[1] = (float)(currentStep.footPos[LEFT_FOOT].y * 1000 - yOffset * 1000 - comShift.y * 1000);
  kinematicRequest.leftFoot[2] = (float)(currentStep.footPos[LEFT_FOOT].z * 1000);

  // position right foot
  kinematicRequest.rightFoot[0] = (float)(currentStep.footPos[RIGHT_FOOT].x * 1000 - xOffset * 1000 - comShift.x * 1000);
  kinematicRequest.rightFoot[1] = (float)(currentStep.footPos[RIGHT_FOOT].y * 1000 - yOffset * 1000 - comShift.y * 1000);
  kinematicRequest.rightFoot[2] = (float)(currentStep.footPos[RIGHT_FOOT].z * 1000);

  // rotation left food
  kinematicRequest.leftFoot[3] = (float)(currentStep.footPos[LEFT_FOOT].rx);
  kinematicRequest.leftFoot[4] = (float)(currentStep.footPos[LEFT_FOOT].ry);
  kinematicRequest.leftFoot[5] = (float)(currentStep.footPos[LEFT_FOOT].r);

  // rotation right food
  kinematicRequest.rightFoot[3] = (float)(currentStep.footPos[RIGHT_FOOT].rx);
  kinematicRequest.rightFoot[4] = (float)(currentStep.footPos[RIGHT_FOOT].ry);
  kinematicRequest.rightFoot[5] = (float)(currentStep.footPos[RIGHT_FOOT].r);

  applyOffsets(kinematicRequest);
  // TODO: CoP influence on sensor error

  // check for NaN
  for (int i = 0; i < 6; i++)
    ASSERT(kinematicRequest.leftFoot[i] == kinematicRequest.leftFoot[i]);
}

void CSConverter2019::determineFootInAir(const Footposition& curPos)
{
  // is a foot not on ground? If yes, which one?
  footInAir = -1;
  if (determineSupportFootByFSR)
  {
    if (theFsrSensorData.calcSupportFoot() >= 0.f)
      footInAir = LEFT_FOOT;
    else
      footInAir = RIGHT_FOOT;
    return;
  }
  if (!curPos.onFloor[LEFT_FOOT])
    footInAir = LEFT_FOOT;
  else if (!curPos.onFloor[RIGHT_FOOT])
    footInAir = RIGHT_FOOT;
}

void CSConverter2019::applyFootPitchRollPD(Footposition& curPos)
{
  const WalkingEngineParams& curparams = theWalkingEngineParams;

  // PD controller for footpitch using body angle
  // Vector2f angleg = theJoinedIMUData.imuData[anglesource].angle;
  float roll = curparams.footMovement.footPitchPD[0] * (theJoinedIMUData.imuData[anglesource].angle.x())
      + curparams.footMovement.footPitchPD[1] * (theJoinedIMUData.imuData[anglesource].gyro.x()) * 0.0075f;
  float pitch = curparams.footMovement.footPitchPD[0] * (theJoinedIMUData.imuData[anglesource].angle.y())
      + curparams.footMovement.footPitchPD[1] * (theJoinedIMUData.imuData[anglesource].gyro.y()) * 0.0075f;
  PLOT("module:CSConverter2019:rollPD", roll);
  PLOT("module:CSConverter2019:pitchPD", pitch);

  // TODO: apply only for support foot?
  // add roll, pitch to both foot rotation
  if (footInAir == RIGHT_FOOT)
  {
    curPos.footPos[RIGHT_FOOT].ry -= pitch;
    curPos.footPos[RIGHT_FOOT].rx -= roll;
  }
  else if (footInAir == LEFT_FOOT)
  {
    curPos.footPos[LEFT_FOOT].ry -= pitch;
    curPos.footPos[LEFT_FOOT].rx -= roll;
  }

  if (rotateLegWithBody)
  {
    if (footInAir != -1) // So yes, there is one not on ground
    {
      float yFactor = std::abs(curPos.speed.translation.y()) / (curparams.speedLimits.y * curparams.speedLimits.speedFactor); // fraction of current speed and maximum speed in y-directon
      float xStepHeight = curPos.speed.translation.x() > 0 ? curparams.footMovement.stepHeight[0] : curparams.footMovement.stepHeight[1];
      float yStepHeight = curparams.footMovement.stepHeight[2];

      float walkStepHeight = (1 - yFactor) * xStepHeight + yFactor * yStepHeight; // actual walkStepHeight resulting from xStepHeight and yStepHeight
      float heightOverGroundFactor = curPos.footPos[footInAir].z / walkStepHeight; // fraction of current height over ground vs. maximum height over ground

      float scaledRoll = heightOverGroundFactor * roll;
      float scaledPitch = heightOverGroundFactor * pitch;

      // rotate vector from stand foot to swing foot around scaledRoll, scaledPitch
      // and add to stand footPos to get adjusted swing foot position
      // similar to z and rotation adaption due to roll/tiltController activated with steppingRotSpeedCapture
      // ------------ DONT USE BOTH! ---------------
      Point stanceToAirFootVec = curPos.footPos[footInAir] - curPos.footPos[!footInAir]; // Vector from stand to air foot origin

      // Rotate vector in robot coordinate system
      stanceToAirFootVec.rotate2D(-curPos.direction);
      stanceToAirFootVec.rotateAroundX(-scaledRoll);
      stanceToAirFootVec.rotateAroundY(-scaledPitch);
      stanceToAirFootVec.rotate2D(curPos.direction);

      curPos.footPos[footInAir] = curPos.footPos[!footInAir] + stanceToAirFootVec;
      PLOT("module:CSConverter2019:scaledRollPD", scaledRoll);
      PLOT("module:CSConverter2019:scaledPitchPD", scaledPitch);
    }
  }
}

void CSConverter2019::calculateOdometry(const Footposition& curPos)
{
  Point positionBetweenFeet = (curPos.footPos[RIGHT_FOOT] + curPos.footPos[LEFT_FOOT]) * 0.5;
  positionBetweenFeet.r = robotPosition.r;

  if (odometryVariant == Parameters::OdometryVariant::ODOMETRY_FROM_WALKING_ENGINE)
  {
    offsetToRobotPoseAfterPreview = theFootSteps.robotPoseAfterStep - positionBetweenFeet;
    offsetToRobotPoseAfterPreview.rotate2D(-positionBetweenFeet.r);

    odometry = positionBetweenFeet - lastPositionBetweenFeet;
    odometry.rotate2D(-lastPositionBetweenFeet.r);

    originWCS = positionBetweenFeet;

    lastPositionBetweenFeet = positionBetweenFeet;
  }
  else if (odometryVariant == Parameters::OdometryVariant::ODOMETRY_FROM_INERTIA_MATRIX)
  {
    // Using the odometry calculated by the TorsoMatrixProvider we have to use the vector
    // from a foot on the ground to the origin given by the Provider. Then we can add this
    // vector to the foot positions in world coordinate system to get the origin in the
    // coordinate system used here. This way we can calculate the offset from the current origin
    // to the final position of the robot.

    // First, convert the fromOriginToFoot from RCS to WCS
    Limbs::Limb limb = (theTorsoMatrix.leftSupportFoot ? Limbs::footLeft : Limbs::footRight);
    Point fromOriginToFoot(theRobotModel.limbs[limb].translation.x() / 1000, theRobotModel.limbs[limb].translation.y() / 1000, 0, theRobotModel.limbs[limb].rotation.getZAngle());

    fromOriginToFoot.rotate2D(positionBetweenFeet.r);

    LOG("WalkingEngine", "fromOriginToFoot x", fromOriginToFoot.x);
    LOG("WalkingEngine", "fromOriginToFoot y", fromOriginToFoot.y);
    LOG("WalkingEngine", "fromOriginToFoot r", fromOriginToFoot.r);

    if (theTorsoMatrix.leftSupportFoot)
      originWCS = curPos.footPos[LEFT_FOOT] - fromOriginToFoot;
    else
      originWCS = curPos.footPos[RIGHT_FOOT] - fromOriginToFoot;

    offsetToRobotPoseAfterPreview = originWCS * -1 + theFootSteps.robotPoseAfterStep;

    // Set correct rotation from current pose to afterPreview
    offsetToRobotPoseAfterPreview.r = -positionBetweenFeet.r + theFootSteps.robotPoseAfterStep.r;

    // Convert the vector from the WCS to the RCS
    offsetToRobotPoseAfterPreview.rotate2D(-positionBetweenFeet.r);

    // Now calculate the offset. Code from TorsoMatrixProvider.

    if (lastTorsoMatrix.translation.z() != 0.)
    {
      Pose3f odometryOffset3D(lastTorsoMatrix);
      odometryOffset3D.conc(theTorsoMatrix.offset);
      odometryOffset3D.conc(theTorsoMatrix.inverse());

      odometry.r = odometryOffset3D.rotation.getZAngle();

      odometry.x = odometryOffset3D.translation.x() / 1000;
      odometry.y = odometryOffset3D.translation.y() / 1000;
    }
    (Pose3f&)lastTorsoMatrix = theTorsoMatrix;
  }

  ASSERT(odometry.x == odometry.x);
  ASSERT(odometry.y == odometry.y);
  ASSERT(odometry.r == odometry.r);

  LOG("WalkingEngine", "Odometry x", odometry.x);
  LOG("WalkingEngine", "Odometry y", odometry.y);
  LOG("WalkingEngine", "Odometry r", odometry.r);
  LOG("WalkingEngine", "offsetToRobotPoseAfterPreview x", offsetToRobotPoseAfterPreview.x);
  LOG("WalkingEngine", "offsetToRobotPoseAfterPreview y", offsetToRobotPoseAfterPreview.y);
  LOG("WalkingEngine", "offsetToRobotPoseAfterPreview r", offsetToRobotPoseAfterPreview.r);
}

void CSConverter2019::toRobotCoords(StepData* requiredOffset, Point& newCoMTarget, Footposition& curPos, Point CoM)
{
  if (!curPos.customStepRunning)
  {
    applyFootPitchRollPD(curPos);
  }

  for (int foot = LEFT_FOOT; foot <= RIGHT_FOOT; foot++)
  {
    // Calculate the distance between current CoM and foot
    requiredOffset->footPos[foot] = curPos.footPos[foot] - newCoMTarget;

    // Rotate the foot position relative to CoM around the z axis
    requiredOffset->footPos[foot].rotate2D(-curPos.direction);

    // Rotate the foot position relative to CoM around the x axis (roll)
    requiredOffset->footPos[foot].rotateAroundX(-curPos.roll);

    // Rotate the foot position relative to CoM around the y axis (pitch)
    // TODO: Why originally not rotateAroundY ???
    //requiredOffset->footPos[foot].rotateAroundY(-curPos.pitch);

    // Add the position of the CoM to get the foot position in robot coordinate system
    requiredOffset->footPos[foot] += CoM;

    // Set the foot pitch
    requiredOffset->footPos[foot].ry = -curPos.pitch;
    requiredOffset->footPos[foot].ry += curPos.footPos[foot].ry;

    // Set the foot roll
    requiredOffset->footPos[foot].rx = -curPos.roll;
    requiredOffset->footPos[foot].rx += curPos.footPos[foot].rx;

    // Set the foot Orientation
    requiredOffset->footPos[foot].r = -curPos.direction;
    requiredOffset->footPos[foot].r += curPos.footPos[foot].r;

    // Set the foot onFloor
    requiredOffset->onFloor[foot] = curPos.onFloor[foot];
  }

  Point gCoM = CoM;
  gCoM.rotate2D(curPos.direction);
  robotPosition = newCoMTarget - gCoM;
  robotPosition.r = curPos.direction;

  // Calculate new position based on the definition of our robot coordinate frame.
  // This will be used for odometry and for the offsetToRobotPoseAfterPreview.
  // The CameraMatrix calculation must be based on this same robot coordinate frame to provide consistent information!
  calculateOdometry(curPos);
}

void CSConverter2019::update(WalkingInfo& walkingInfo)
{
  updateWalkingInfo(walkingInfo);
};

void CSConverter2019::updateWalkingInfo(WalkingInfo& walkingInfo)
{
  const WalkingEngineParams& curparams = theWalkingEngineParams;

  walkingInfo.robotPosition.translation.x() = robotPosition.x;
  walkingInfo.robotPosition.translation.y() = robotPosition.y;
  walkingInfo.robotPosition.rotation = robotPosition.r;
  walkingInfo.isCustomStepRunning = isInstantKickRunning;
  walkingInfo.isLeavingPossible = theFootpositions.currentState == standby;
  walkingInfo.isRunning = isRunning;
  walkingInfo.desiredBodyRot.x() = 0.f;
  walkingInfo.desiredBodyRot.y() = speedDependentTilt;
  walkingInfo.onFloor[RIGHT_FOOT] = currentStep.onFloor[RIGHT_FOOT];
  walkingInfo.onFloor[LEFT_FOOT] = currentStep.onFloor[LEFT_FOOT];

  Pose3f limbs[Limbs::numOfLimbs];
  JointAngles jA;
  for (int i = 0; i < 12; i++)
    jA.angles[i + Joints::lHipYawPitch] = curparams.jointCalibration.jointCalibration[i];
  for (int i = 0; i < 6; i++)
    jA.angles[i + Joints::lHipYawPitch] += curparams.jointCalibration.offsetLeft[i];
  for (int i = 0; i < 6; i++)
    jA.angles[i + Joints::rHipYawPitch] += curparams.jointCalibration.offsetRight[i];
  ForwardKinematic::calculateLegChain(currentStep.onFloor[0], jA, theRobotDimensions, limbs);
  Pose3f l;
  if (currentStep.onFloor[0])
    l = limbs[Limbs::footLeft];
  else
    l = limbs[Limbs::footRight];

  walkingInfo.desiredBodyRot.y() -= l.rotation.getYAngle(); // Maybe that's a problem
  // TODO Check if right
  //walkingInfo.desiredBodyRot.y() -= l.rotation.inverse().getYAngle();

  if (isRunning)
  {
    walkingInfo.expectedAcc.x() = acc.x;
    walkingInfo.expectedAcc.y() = acc.y;

    walkingInfo.odometryOffset.translation.x() = odometry.x * 1000;
    walkingInfo.odometryOffset.translation.y() = odometry.y * 1000;
    walkingInfo.odometryOffset.rotation = odometry.r;

    walkingInfo.offsetToRobotPoseAfterPreview.translation.x() = offsetToRobotPoseAfterPreview.x * 1000;
    walkingInfo.offsetToRobotPoseAfterPreview.translation.y() = offsetToRobotPoseAfterPreview.y * 1000;
    walkingInfo.offsetToRobotPoseAfterPreview.rotation = offsetToRobotPoseAfterPreview.r;

    walkingInfo.ballCSinWEWCS.x() = originWCS.x;
    walkingInfo.ballCSinWEWCS.y() = originWCS.y;

    walkingInfo.lastUsedFootPositions = theFootpositions;
  }
  else
  {
    walkingInfo.expectedAcc.x() = 0;
    walkingInfo.expectedAcc.y() = 0;

    walkingInfo.odometryOffset.translation.x() = 0;
    walkingInfo.odometryOffset.translation.y() = 0;
    walkingInfo.odometryOffset.rotation = 0;
  }
}

MAKE_MODULE(CSConverter2019, dortmundWalkingEngine)
