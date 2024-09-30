#include "CSConverter2019.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/BHMath.h"

//#define LOGGING
#include "Tools/Debugging/CSVLogger.h"
#include "Tools/Math/Bspline.h"

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
  PLOT("module:CSConverter2019:Footpos[LEFT_FOOT].rx", kinematicRequest.leftFoot[3]);
  PLOT("module:CSConverter2019:Footpos[LEFT_FOOT].ry", kinematicRequest.leftFoot[4]);
  PLOT("module:CSConverter2019:Footpos[LEFT_FOOT].r", kinematicRequest.leftFoot[5]);
  PLOT("module:CSConverter2019:Footpos[RIGHT_FOOT].x", kinematicRequest.rightFoot[0]);
  PLOT("module:CSConverter2019:Footpos[RIGHT_FOOT].y", kinematicRequest.rightFoot[1]);
  PLOT("module:CSConverter2019:Footpos[RIGHT_FOOT].z", kinematicRequest.rightFoot[2]);
  PLOT("module:CSConverter2019:Footpos[RIGHT_FOOT].rx", kinematicRequest.rightFoot[3]);
  PLOT("module:CSConverter2019:Footpos[RIGHT_FOOT].ry", kinematicRequest.rightFoot[4]);
  PLOT("module:CSConverter2019:Footpos[RIGHT_FOOT].r", kinematicRequest.rightFoot[5]);
  DECLARE_PLOT("module:CSConverter2019:xOffset");
  DECLARE_PLOT("module:CSConverter2019:factor");
  DECLARE_PLOT("module:CSConverter2019:rollPD");
  DECLARE_PLOT("module:CSConverter2019:rollAngle");
  DECLARE_PLOT("module:CSConverter2019:pitchPD");
  DECLARE_PLOT("module:CSConverter2019:pitchAngle");
  DECLARE_DEBUG_RESPONSE("module:CSConverter2019:forceParalellFeet");

  DECLARE_PLOT("module:CSConverter2019:filteredJointError:lHipYawPitch:angle");
  DECLARE_PLOT("module:CSConverter2019:filteredJointError:lHipPitch:angle");
  DECLARE_PLOT("module:CSConverter2019:filteredJointError:lHipRoll:angle");
  DECLARE_PLOT("module:CSConverter2019:filteredJointError:lKneePitch:angle");
  DECLARE_PLOT("module:CSConverter2019:filteredJointError:lAnklePitch:angle");
  DECLARE_PLOT("module:CSConverter2019:filteredJointError:lAnkleRoll:angle");
  DECLARE_PLOT("module:CSConverter2019:filteredJointError:rHipYawPitch:angle");
  DECLARE_PLOT("module:CSConverter2019:filteredJointError:rHipRoll:angle");
  DECLARE_PLOT("module:CSConverter2019:filteredJointError:rHipPitch:angle");
  DECLARE_PLOT("module:CSConverter2019:filteredJointError:rKneePitch:angle");
  DECLARE_PLOT("module:CSConverter2019:filteredJointError:rAnklePitch:angle");
  DECLARE_PLOT("module:CSConverter2019:filteredJointError:rAnkleRoll:angle");

  DECLARE_PLOT("module:CSConverter2019:Ankle:filteredError:x");
  DECLARE_PLOT("module:CSConverter2019:Ankle:filteredError:y");
  DECLARE_PLOT("module:CSConverter2019:Hip:filteredError:x");
  DECLARE_PLOT("module:CSConverter2019:Hip:filteredError:y");
  DECLARE_PLOT("module:CSConverter2019:Hip:angleErrorX");
  DECLARE_PLOT("module:CSConverter2019:Hip:angleErrorY");
  DECLARE_PLOT("module:CSConverter2019:Ankle:angleErrorX");
  DECLARE_PLOT("module:CSConverter2019:Ankle:angleErrorY");

  DECLARE_PLOT("module:CSConverter2019:Ankle:errorSum:x");
  DECLARE_PLOT("module:CSConverter2019:Ankle:errorSum:y");
  DECLARE_PLOT("module:CSConverter2019:Hip:errorSum:x");
  DECLARE_PLOT("module:CSConverter2019:Hip:errorSum:y");
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
      / (fp.speed.translation.x() >= 0 ? curparams.speedLimits.xForward * curparams.speedFactor : curparams.speedLimits.xBackward * curparams.speedFactor);
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
    RotationMatrix rot(
        theJoinedIMUData.imuData[theSensorControlParams.coMShifting.anglesource].angle.x(), theJoinedIMUData.imuData[theSensorControlParams.coMShifting.anglesource].angle.y(), 0_deg);
    rotAcc = rot * (theJoinedIMUData.imuData[theSensorControlParams.coMShifting.anglesource].acc);
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
    float alphaFactor = theSensorControlParams.coMShifting.accXAlpha * accFactor;
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
  interpolRatio = std::min(theFrameInfo.getTimeSince(interpolStartTime) / (float)theSensorControlParams.coMShifting.accInterpolTime, 1.f);

  if (stepAccX < 0 && speedX > 0) // only when accelerating forward
    xOffset -= sgn(stepAccX) * std::min(std::abs((lastStepAccX * (1 - interpolRatio) + stepAccX * interpolRatio) * theSensorControlParams.coMShifting.stepAccAlpha), 0.05f);

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
  if (footInAir == RIGHT_FOOT)
  {
    for (int i = 0; i < 6; i++)
    {
      kinematicRequest.offsets.angles[i + (int)Joints::lHipYawPitch] = theWalkingEngineParams.jointCalibration.offsetLeft[i];
    }
  }
  if (footInAir == LEFT_FOOT)
  {
    for (int i = 0; i < 6; i++)
    {
      kinematicRequest.offsets.angles[i + (int)Joints::rHipYawPitch] = theWalkingEngineParams.jointCalibration.offsetRight[i];
    }
  }

  for (size_t i = 0; i < jointError_sum.angles.size(); i++)
  {
    jointError_sum.angles[i] += theJointError.angles[i];
  }

  JointAngles filteredJointError;
  if (theSensorControlParams.sensorControlActivation.activateJointErrorPID && !theWalkCalibration.deactivateSensorControl)
  {
    for (size_t i = 0; i < filteredJointError.angles.size(); i++)
    {
      // clang-format off
      filteredJointError.angles[i] = theJointError.angles[i] * theSensorControlParams.jointErrorPID.pid.P +
                                     jointError_sum.angles[i] * theSensorControlParams.jointErrorPID.pid.I +
                                    (jointError_last.angles[i] - theJointError.angles[i]) * theSensorControlParams.jointErrorPID.pid.D;
      // clang-format on
    }

    kinematicRequest.offsets.angles[Joints::lHipYawPitch] += filteredJointError.angles[Joints::lHipYawPitch];
    kinematicRequest.offsets.angles[Joints::lHipPitch] += filteredJointError.angles[Joints::lHipPitch];
    kinematicRequest.offsets.angles[Joints::lHipRoll] += filteredJointError.angles[Joints::lHipRoll];
    kinematicRequest.offsets.angles[Joints::lKneePitch] += filteredJointError.angles[Joints::lKneePitch];
    kinematicRequest.offsets.angles[Joints::lAnklePitch] += filteredJointError.angles[Joints::lAnklePitch];
    kinematicRequest.offsets.angles[Joints::lAnkleRoll] += filteredJointError.angles[Joints::lAnkleRoll];

    kinematicRequest.offsets.angles[Joints::rHipYawPitch] += filteredJointError.angles[Joints::rHipYawPitch];
    kinematicRequest.offsets.angles[Joints::rHipPitch] += filteredJointError.angles[Joints::rHipPitch];
    kinematicRequest.offsets.angles[Joints::rHipRoll] += filteredJointError.angles[Joints::rHipRoll];
    kinematicRequest.offsets.angles[Joints::rKneePitch] += filteredJointError.angles[Joints::rKneePitch];
    kinematicRequest.offsets.angles[Joints::rAnklePitch] += filteredJointError.angles[Joints::rAnklePitch];
    kinematicRequest.offsets.angles[Joints::rAnkleRoll] += filteredJointError.angles[Joints::rAnkleRoll];

    PLOT("module:CSConverter2019:filteredJointError:lHipYawPitch:angle", filteredJointError.angles[Joints::lHipYawPitch].toDegrees());
    PLOT("module:CSConverter2019:filteredJointError:lHipPitch:angle", filteredJointError.angles[Joints::lHipPitch].toDegrees());
    PLOT("module:CSConverter2019:filteredJointError:lHipRoll:angle", filteredJointError.angles[Joints::lHipRoll].toDegrees());
    PLOT("module:CSConverter2019:filteredJointError:lKneePitch:angle", filteredJointError.angles[Joints::lKneePitch].toDegrees());
    PLOT("module:CSConverter2019:filteredJointError:lAnklePitch:angle", filteredJointError.angles[Joints::lAnklePitch].toDegrees());
    PLOT("module:CSConverter2019:filteredJointError:lAnkleRoll:angle", filteredJointError.angles[Joints::lAnkleRoll].toDegrees());
    PLOT("module:CSConverter2019:filteredJointError:rHipYawPitch:angle", filteredJointError.angles[Joints::rHipYawPitch].toDegrees());
    PLOT("module:CSConverter2019:filteredJointError:rHipRoll:angle", filteredJointError.angles[Joints::rHipRoll].toDegrees());
    PLOT("module:CSConverter2019:filteredJointError:rHipPitch:angle", filteredJointError.angles[Joints::rHipPitch].toDegrees());
    PLOT("module:CSConverter2019:filteredJointError:rKneePitch:angle", filteredJointError.angles[Joints::rKneePitch].toDegrees());
    PLOT("module:CSConverter2019:filteredJointError:rAnklePitch:angle", filteredJointError.angles[Joints::rAnklePitch].toDegrees());
    PLOT("module:CSConverter2019:filteredJointError:rAnkleRoll:angle", filteredJointError.angles[Joints::rAnkleRoll].toDegrees());
  }

  for (size_t i = 0; i < jointError_last.angles.size(); i++)
  {
    jointError_last.angles[i] = theJointError.angles[i];
  }


  if (theSensorControlParams.sensorControlActivation.activateAnkleHipPID && !theWalkCalibration.deactivateSensorControl)
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

    // clang-format off
    Angle jointErrorPitchSum = 0_deg, jointErrorRollSum = 0_deg;
    if (footInAir == RIGHT_FOOT)
    {
      jointErrorPitchSum = filteredJointError.angles[Joints::lHipPitch] +
                           filteredJointError.angles[Joints::lKneePitch] +
                           filteredJointError.angles[Joints::lAnklePitch];
      jointErrorRollSum = filteredJointError.angles[Joints::lHipRoll] +
                          filteredJointError.angles[Joints::lAnkleRoll];
    }
    else
    {
      jointErrorPitchSum = filteredJointError.angles[Joints::rHipPitch] +
                           filteredJointError.angles[Joints::rKneePitch] +
                           filteredJointError.angles[Joints::rAnklePitch];
      jointErrorRollSum = filteredJointError.angles[Joints::rHipRoll] +
                          filteredJointError.angles[Joints::rAnkleRoll];
    }
    // clang-format on

    {
      Angle angleErrorX = (theSensorControlParams.ankleHipPID.targetAngle.x() - theJoinedIMUData.imuData[theSensorControlParams.ankleHipPID.anglesource].angle.x() + jointErrorRollSum);
      Angle angleErrorY = (theSensorControlParams.ankleHipPID.targetAngle.y() - theJoinedIMUData.imuData[theSensorControlParams.ankleHipPID.anglesource].angle.y() + jointErrorPitchSum);

      Angle angleErrorXAnkle = angleErrorX * theSensorControlParams.ankleHipPID.ankleHipRatio.x();
      Angle angleErrorXHip = angleErrorX * (1.f - theSensorControlParams.ankleHipPID.ankleHipRatio.x());
      Angle angleErrorYAnkle = angleErrorY * theSensorControlParams.ankleHipPID.ankleHipRatio.y();
      Angle angleErrorYHip = angleErrorY * (1.f - theSensorControlParams.ankleHipPID.ankleHipRatio.y());

      if (std::abs(angleErrorX) < 0.75_deg)
      {
        pidAngleXAnkle_sum = 0.f;
        pidAngleXHip_sum = 0.f;
      }
      else
      {
        pidAngleXAnkle_sum += angleErrorXAnkle;
        pidAngleXHip_sum += angleErrorXHip;
      }

      if (std::abs(angleErrorY) < 0.75_deg)
      {
        pidAngleYAnkle_sum = 0.f;
        pidAngleYHip_sum = 0.f;
      }
      else
      {
        pidAngleYAnkle_sum += angleErrorYAnkle;
        pidAngleYHip_sum += angleErrorYHip;
      }


      float anklePIDMultiplicatorX, anklePIDMultiplicatorY, hipPIDMultiplicatorX, hipPIDMultiplicatorY;
      if (theSpeedRequest.translation.norm() == 0 && theSpeedRequest.rotation == 0)
      {
        if (theSensorControlParams.sensorControlActivation.activateSpeedReduction)
        /*&& (angleErrorX < (theSensorControlParams.speedReduction.angleX.min) || angleErrorX > (theSensorControlParams.speedReduction.angleX.max)
                || (angleErrorY < theSensorControlParams.speedReduction.angleY.min) || angleErrorY > (theSensorControlParams.speedReduction.angleY.max)))*/
        {
          anklePIDMultiplicatorX = theSensorControlParams.ankleHipPID.ankleParams.pidMultiplicatorsX.zero;
          anklePIDMultiplicatorY = theSensorControlParams.ankleHipPID.ankleParams.pidMultiplicatorsY.zero;
          hipPIDMultiplicatorX = theSensorControlParams.ankleHipPID.hipParams.pidMultiplicatorsX.zero;
          hipPIDMultiplicatorY = theSensorControlParams.ankleHipPID.hipParams.pidMultiplicatorsY.zero;
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
        anklePIDMultiplicatorX = theSensorControlParams.ankleHipPID.ankleParams.pidMultiplicatorsX.normal;
        anklePIDMultiplicatorY = theSensorControlParams.ankleHipPID.ankleParams.pidMultiplicatorsY.normal;
        hipPIDMultiplicatorX = theSensorControlParams.ankleHipPID.hipParams.pidMultiplicatorsX.normal;
        hipPIDMultiplicatorY = theSensorControlParams.ankleHipPID.hipParams.pidMultiplicatorsY.normal;
      }

      if (theSensorControlParams.sensorControlActivation.activateCalibrationScaling)
      {
        float qualityFactor = std::clamp(1.5f - theWalkCalibration.qualityOfRobotHardware, 0.5f, 1.5f);
        anklePIDMultiplicatorX *= qualityFactor;
        anklePIDMultiplicatorY *= qualityFactor;
        hipPIDMultiplicatorX *= qualityFactor;
        hipPIDMultiplicatorY *= qualityFactor;
      }

      Angle filteredErrorX_Ankle = angleErrorXAnkle * theSensorControlParams.ankleHipPID.ankleParams.x.P * anklePIDMultiplicatorX
          + pidAngleXAnkle_sum * theSensorControlParams.ankleHipPID.ankleParams.x.I * anklePIDMultiplicatorX
          + (pidAngleXAnkle_last - angleErrorXAnkle) * theSensorControlParams.ankleHipPID.ankleParams.x.D * anklePIDMultiplicatorX;

      Angle filteredErrorY_Ankle = angleErrorYAnkle * theSensorControlParams.ankleHipPID.ankleParams.y.P * anklePIDMultiplicatorY
          + pidAngleYAnkle_sum * theSensorControlParams.ankleHipPID.ankleParams.y.I * anklePIDMultiplicatorY
          + (pidAngleYAnkle_last - angleErrorYAnkle) * theSensorControlParams.ankleHipPID.ankleParams.y.D * anklePIDMultiplicatorY;

      Angle filteredErrorX_Hip = angleErrorXHip * theSensorControlParams.ankleHipPID.hipParams.x.P * hipPIDMultiplicatorX
          + pidAngleXHip_sum * theSensorControlParams.ankleHipPID.hipParams.x.I * hipPIDMultiplicatorX
          + (pidAngleXHip_last - angleErrorXHip) * theSensorControlParams.ankleHipPID.hipParams.x.D * hipPIDMultiplicatorX;
      Angle filteredErrorY_Hip = angleErrorYHip * theSensorControlParams.ankleHipPID.hipParams.y.P * hipPIDMultiplicatorY
          + pidAngleYHip_sum * theSensorControlParams.ankleHipPID.hipParams.y.I * hipPIDMultiplicatorY
          + (pidAngleYHip_last - angleErrorYHip) * theSensorControlParams.ankleHipPID.hipParams.y.D * hipPIDMultiplicatorY;


      PLOT("module:CSConverter2019:Ankle:filteredError:x", filteredErrorX_Ankle.toDegrees());
      PLOT("module:CSConverter2019:Ankle:filteredError:y", filteredErrorY_Ankle.toDegrees());
      PLOT("module:CSConverter2019:Hip:filteredError:x", filteredErrorX_Hip.toDegrees());
      PLOT("module:CSConverter2019:Hip:filteredError:y", filteredErrorY_Hip.toDegrees());

      PLOT("module:CSConverter2019:Ankle:angleErrorX", -angleErrorXAnkle.toDegrees());
      PLOT("module:CSConverter2019:Ankle:angleErrorY", -angleErrorYAnkle.toDegrees());
      PLOT("module:CSConverter2019:Hip:angleErrorX", -angleErrorXHip.toDegrees());
      PLOT("module:CSConverter2019:Hip:angleErrorY", -angleErrorYHip.toDegrees());

      PLOT("module:CSConverter2019:Ankle:errorSum:x", pidAngleXAnkle_sum);
      PLOT("module:CSConverter2019:Ankle:errorSum:y", pidAngleYAnkle_sum);
      PLOT("module:CSConverter2019:Hip:errorSum:x", pidAngleXHip_sum);
      PLOT("module:CSConverter2019:Hip:errorSum:y", pidAngleYHip_sum);

      pidAngleXAnkle_last = angleErrorXAnkle;
      pidAngleYAnkle_last = angleErrorYAnkle;
      pidAngleXHip_last = angleErrorXHip;
      pidAngleYHip_last = angleErrorYHip;

      // TODO: interpolate towards offsets?
      if (!theSensorControlParams.ankleHipPID.balanceSupportLegOnly || footInAir == RIGHT_FOOT)
      {
        kinematicRequest.offsets.angles[Joints::lHipPitch] -= filteredErrorY_Hip;
        kinematicRequest.offsets.angles[Joints::lHipRoll] += filteredErrorX_Hip;
        kinematicRequest.offsets.angles[Joints::lAnklePitch] -= filteredErrorY_Ankle;
        kinematicRequest.offsets.angles[Joints::lAnkleRoll] += filteredErrorX_Ankle;
      }
      if (!theSensorControlParams.ankleHipPID.balanceSupportLegOnly || footInAir == LEFT_FOOT)
      {
        kinematicRequest.offsets.angles[Joints::rHipPitch] -= filteredErrorY_Hip;
        kinematicRequest.offsets.angles[Joints::rHipRoll] += filteredErrorX_Hip;
        kinematicRequest.offsets.angles[Joints::rAnklePitch] -= filteredErrorY_Ankle;
        kinematicRequest.offsets.angles[Joints::rAnkleRoll] += filteredErrorX_Ankle;
      }

      lastComX = newComX;
    }
  }

  if (!kinematicRequest.offsets.isValid())
  {
    {
      std::string logDir = "";
#ifdef TARGET_ROBOT
      logDir = "../logs/";
#endif
      OutMapFile stream(logDir + "balancing.log");
      for (int i = 0; i < Joints::numOfJoints; ++i)
        stream << kinematicRequest.offsets.angles[i] << "\n";
    }
    kinematicRequest.offsets = JointAngles();
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
  jointError_sum.angles.fill(0_deg);
  jointError_last.angles.fill(0_deg);
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
  float xOffset = theWalkingEngineParams.comOffsets.xFixed;
  float yOffset = theWalkingEngineParams.comOffsets.yFixed;
  if (isRunning)
  {
    if (theSensorControlParams.sensorControlActivation.activateCoMShifting && !theWalkCalibration.deactivateSensorControl)
      applyComShift(xOffset, yOffset);

    if (fp.speed.translation.x() > 0)
      xOffset += curparams.comOffsets.xSpeedDependent * fp.speed.translation.x() * 1000 / (curparams.speedLimits.xForward * curparams.speedFactor);
    yOffset += curparams.comOffsets.ySpeedDependent[fp.speed.translation.y() < 0] * std::abs(fp.speed.translation.y() * 1000) / (curparams.speedLimits.y * curparams.speedFactor);
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
    if (theFsrSensorData.calcSupportFoot() > 0.f)
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
  Angle staticFootPitch = 0_deg;
  Angle staticFootRoll = 0_deg;

  if (curPos.speed.translation.x() < 0.f)
  {
    staticFootPitch = theWalkingEngineParams.footMovement.footPitch * sgn(curPos.speed.translation.x());
  }
  if (curPos.speed.translation.x() < 0.f)
  {
    staticFootRoll = theWalkingEngineParams.footMovement.footRoll * sgn(curPos.speed.translation.y());
  }

  // PD controller for footpitch using body angle
  Angle dynamicFootRoll = 0_deg;
  Angle dynamicFootPitch = 0_deg;
  if (theSensorControlParams.sensorControlActivation.activateParalellFeetController && !theWalkCalibration.deactivateSensorControl)
  {
    dynamicFootRoll = theSensorControlParams.paralellFeetController.controller.P * (theJoinedIMUData.imuData[theSensorControlParams.paralellFeetController.anglesource].angle.x())
        + theSensorControlParams.paralellFeetController.controller.D * (theJoinedIMUData.imuData[theSensorControlParams.paralellFeetController.anglesource].gyro.x());
    dynamicFootPitch = theSensorControlParams.paralellFeetController.controller.P * (theJoinedIMUData.imuData[theSensorControlParams.paralellFeetController.anglesource].angle.y())
        + theSensorControlParams.paralellFeetController.controller.D * (theJoinedIMUData.imuData[theSensorControlParams.paralellFeetController.anglesource].gyro.y());
  }

  unsigned h = 0;
  for (size_t i = 0; i < sizeof(theSensorControlParams.paralellFeetController.polygon) / sizeof(theSensorControlParams.paralellFeetController.polygon[0]); i++)
  {
    h ^= (unsigned&)theSensorControlParams.paralellFeetController.polygon[i];
  }

  if (offsetSpline.size() != curPos.singleSupportDurationInFrames || lastPolygonHash != h)
  {
    lastPolygonHash = h;
    offsetSpline.resize(curPos.singleSupportDurationInFrames, 1.f);
    const int noControlPoints = 5;
    std::vector<float> controlVector(noControlPoints);
    for (int i = 0; i < noControlPoints; i++)
    {
      controlVector[i] = theSensorControlParams.paralellFeetController.polygon[i];
    }
    BSpline<float>::bspline(noControlPoints - 1, 3, &(controlVector[0]), &(offsetSpline[0]), curPos.singleSupportDurationInFrames);
  }

  float factor = offsetSpline[curPos.frameInPhase];
  bool force_both = false;
  DEBUG_RESPONSE("module:CSConverter2019:forceParalellFeet")
  {
    force_both = true;
  }

  // TODO: apply only for support foot?
  // add roll, pitch to both foot rotation
  if (!force_both && footInAir == RIGHT_FOOT)
  {
    curPos.footPos[RIGHT_FOOT].ry -= factor * (staticFootPitch + dynamicFootPitch);
    curPos.footPos[RIGHT_FOOT].rx -= factor * (staticFootRoll + dynamicFootRoll);
  }
  if (!force_both && footInAir == LEFT_FOOT)
  {
    curPos.footPos[LEFT_FOOT].ry -= factor * (staticFootPitch + dynamicFootPitch);
    curPos.footPos[LEFT_FOOT].rx -= factor * (staticFootRoll + dynamicFootRoll);
  }


  PLOT("module:CSConverter2019:factor", factor);
  PLOT("module:CSConverter2019:rollPD", toDegrees(factor * (staticFootRoll + dynamicFootRoll)));
  PLOT("module:CSConverter2019:rollAngle", toDegrees(theJoinedIMUData.imuData[theSensorControlParams.paralellFeetController.anglesource].angle.x()));
  PLOT("module:CSConverter2019:pitchPD", toDegrees(factor * (staticFootPitch + dynamicFootPitch)));
  PLOT("module:CSConverter2019:pitchAngle", toDegrees(theJoinedIMUData.imuData[theSensorControlParams.paralellFeetController.anglesource].angle.y()));
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

  //const WalkingEngineParams& curparams = theWalkingEngineParams;
  //JointAngles jA;
  //for (int i = 0; i < 12; i++)
  //  jA.angles[i + Joints::lHipYawPitch] = curparams.jointCalibration.jointCalibration[i];
  //for (int i = 0; i < 6; i++)
  //  jA.angles[i + Joints::lHipYawPitch] += curparams.jointCalibration.offsetLeft[i];
  //for (int i = 0; i < 6; i++)
  //  jA.angles[i + Joints::rHipYawPitch] += curparams.jointCalibration.offsetRight[i];

  //Pose3f limbs[Limbs::numOfLimbs];
  //ForwardKinematic::calculateLegChain(currentStep.onFloor[0], jA, theRobotDimensions, limbs);
  //Pose3f l;
  //if (currentStep.onFloor[0])
  //  l = limbs[Limbs::footLeft];
  //else
  //  l = limbs[Limbs::footRight];

  //walkingInfo.desiredBodyRot.y() -= l.rotation.getYAngle(); // Maybe that's a problem
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

    walkingInfo.offsetToRobotPoseAfterPreview.translation.x() = 0;
    walkingInfo.offsetToRobotPoseAfterPreview.translation.y() = 0;
    walkingInfo.offsetToRobotPoseAfterPreview.rotation = 0;
  }
}

MAKE_MODULE(CSConverter2019, dortmundWalkingEngine)
