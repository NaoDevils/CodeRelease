#include "CSConverter2019.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/interpolator.h"
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
CSConverter2019::~CSConverter2019()
{

}

void CSConverter2019::update(KinematicRequest &kinematicRequest)
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

void CSConverter2019::clearKinematicRequest(KinematicRequest & kinematicRequest)
{
  for(int i = 2; i < Joints::numOfJoints; i++)
  {
    kinematicRequest.offsets.angles[i] = 0;
  }
  for(int i = 0; i < 6; i++)
  {
    kinematicRequest.leftFoot[i] = JointAngles::ignore;
    kinematicRequest.rightFoot[i] = JointAngles::ignore;
  }
  kinematicRequest.kinematicType = KinematicRequest::feet;

}

KinematicRequest::KinematicType CSConverter2019::determineKinematicType()
{
  if(currentStep.onFloor[LEFT_FOOT] && currentStep.onFloor[RIGHT_FOOT])
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
  if(!isRunning &&
     theFootpositions.running &&
     theFootpositions.currentState != goingToReady &&
     theFootpositions.currentState != goingToStandby)
  {
    isRunning = true;
  }

  if(isRunning &&
    (!theFootpositions.running ||
     theFootpositions.currentState == goingToReady ||
     theFootpositions.currentState == goingToStandby))
  {
    reset();
    isRunning = false;
  }
}

void CSConverter2019::applySpeedDependentTilt(Footpositions & fp)
{
  const WalkingEngineParams &curparams = theWalkingEngineParams;
  // Apply speed dependent tilt
  float xFactor = std::abs(fp.speed.x * 1000) / (fp.speed.x >= 0 ? curparams.speedLimits.xForward : curparams.speedLimits.xBackward);
  speedDependentTilt = xFactor * (fp.speed.x >= 0 ? curparams.comOffsets.tiltSpeedDependent[0] : curparams.comOffsets.tiltSpeedDependent[1]);

  fp.pitch += speedDependentTilt;
  fp.roll += 0;
}

void CSConverter2019::applyComShift(float &xOffset, float &yOffset)
{
  const WalkingEngineParams &curparams = theWalkingEngineParams;
  Vector3f rotAcc;
  if(true) // TODO Parameter
  {
    RotationMatrix rot(theInertialSensorData.angle.x(), theInertialSensorData.angle.y(), 0_deg);
    rotAcc = rot * theInertialSensorData.acc;
  } 
  else
  {
    rotAcc = theInertialSensorData.acc;
  }

  xOffset = curparams.comOffsets.xFixed;

  static int zerocount = 0;
  if (theFootpositions.speed == Point() && zerocount <= 100) 
    zerocount++;
  else if (!(theFootpositions.speed == Point())) 
    zerocount = 0;

  if (zerocount < 100)
  {
    // accXAlpha code
    accXBuffer.push_front(rotAcc.x() * -0.01558f);
    float avgAccX = accXBuffer.average();
    float accFactor = std::abs(accXBuffer[0]) / 0.3f;
    float alphaFactor = comShiftParameters.accXAlpha*accFactor;
    avgAccX = sgn(avgAccX)*std::min<float>(std::abs(avgAccX), 0.03f) - curparams.comOffsets.xFixed;
    xOffset = (curparams.comOffsets.xFixed * (1 - alphaFactor) - avgAccX*alphaFactor);
    // end of accXAlpha code
    // added 3.2019: gyro influence on x offset
    {
      xOffset -= sgn(gyroYBuffer.average()) * std::min(0.05f, std::abs(gyroYBuffer.average() * comShiftParameters.gyroYAlpha));
    }
    // added 3.2019: angle influence on x offset
    {
      xOffset -= sgn(angleYBuffer.average()) * std::min(0.05f, std::abs(angleYBuffer.average() * comShiftParameters.angleYAlpha));
    }
    // added 4.2019: y offset shift
    yOffset = (curparams.comOffsets.yFixed);
    yOffset -= sgn(gyroXBuffer.average()) * std::min(0.05f, std::abs(gyroXBuffer.average() * comShiftParameters.gyroXAlpha));
    yOffset -= sgn(angleXBuffer.average()) * std::min(0.05f, std::abs(angleXBuffer.average() * comShiftParameters.angleXAlpha));
    
  }

  // addded 4.2019: x offset shift trough actual step acceleration
  float speedX = theFootpositions.speed.x;
  float stepAccX = lastSpeedX - speedX;
  if ((theFootpositions.phase == WalkingPhase::firstSingleSupport || theFootpositions.phase == WalkingPhase::secondSingleSupport) &&
    theFootpositions.frameInPhase == 0)
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
}

Point CSConverter2019::handleArmContactState()
{
  const WalkingEngineParams &curparams = theWalkingEngineParams;

  ArmContact::ArmContactState stateLeftArm = theArmContact.armContactStateLeft;
  ArmContact::ArmContactState stateRightArm = theArmContact.armContactStateRight;
  Point comShift;
  comShift = 0.f;

  if ((stateLeftArm == ArmContact::ArmContactState::Back || 
       stateLeftArm == ArmContact::ArmContactState::Front) 
      && 
      stateRightArm == ArmContact::ArmContactState::None)
  {
    comShift.y = curparams.comOffsets.yArmContact[0];
    comShift.x = curparams.comOffsets.xArmContact;
  }
  else if ((stateRightArm == ArmContact::ArmContactState::Back || 
            stateRightArm == ArmContact::ArmContactState::Front)
           && 
           stateLeftArm == ArmContact::ArmContactState::None)
  {
    comShift.y = curparams.comOffsets.yArmContact[1];
    comShift.x = curparams.comOffsets.xArmContact;
  }
  else if ((stateRightArm == ArmContact::ArmContactState::Back && 
            stateLeftArm == ArmContact::ArmContactState::Back)
           || 
           (stateRightArm == ArmContact::ArmContactState::Front && 
            stateLeftArm == ArmContact::ArmContactState::Front)) 
  {
    comShift.y = 0.f;
    comShift.x = curparams.comOffsets.xArmContact;
  }

  return comShift;
}

void CSConverter2019::applyOffsets(KinematicRequest& kinematicRequest)
{
  const WalkingEngineParams &curparams = theWalkingEngineParams;

  if (currentStep.onFloor[LEFT_FOOT] && !currentStep.onFloor[RIGHT_FOOT])
  {
    for (int i = 0; i < 6; i++)
    {
      kinematicRequest.offsets.angles[i + (int)Joints::lHipYawPitch] = curparams.jointCalibration.offsetLeft[i];
    }
  }
  if (!currentStep.onFloor[LEFT_FOOT] && currentStep.onFloor[RIGHT_FOOT])
  {
    for (int i = 0; i<6; i++)
    {
      kinematicRequest.offsets.angles[i + (int)Joints::rHipYawPitch] = curparams.jointCalibration.offsetRight[i];
    }
  }

  if (useLegJointBalancing)
  {
    if (theFallDownState.state != FallDownState::upright)
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

    Angle angleX = useAngleBuffer ? angleXBuffer.average() : angleXBuffer[0];
    Angle angleY = useAngleBuffer ? angleYBuffer.average() : angleYBuffer[0];

    float gyroX = useGyroBuffer ? gyroXBuffer.average() : gyroXBuffer[0];
    float gyroY = useGyroBuffer ? gyroYBuffer.average() : gyroYBuffer[0];

    const Angle &angleYDiffAnkleMax =
      (angleY < legJointBalanceParams.targetAngleY) ?
      (legJointBalanceParams.ankleParams.deltaAngleBack) : (legJointBalanceParams.ankleParams.deltaAngleFront);
    const Angle &angleYDiffHipMax =
      (angleY < legJointBalanceParams.targetAngleY) ?
      (legJointBalanceParams.hipParams.deltaAngleBack) : (legJointBalanceParams.hipParams.deltaAngleFront);
    {
      Angle angleErrorX = (legJointBalanceParams.targetAngleX - angleX);
      Angle angleErrorY = (legJointBalanceParams.targetAngleY - angleY);

      float influenceFactorAnkleY = (angleYDiffAnkleMax == 0.f) ? 1.f : std::sin(pi_2*std::min<Angle>(std::abs(angleErrorY), angleYDiffAnkleMax)/angleYDiffAnkleMax);
      float influenceFactorAnkleX = (legJointBalanceParams.hipParams.deltaAngleX == 0.f) ? 1.f : std::sin(pi_2*std::min<Angle>(std::abs(angleErrorX), 
        legJointBalanceParams.hipParams.deltaAngleX)/legJointBalanceParams.hipParams.deltaAngleX);
      float influenceFactorHipY = (angleYDiffHipMax == 0.f) ? 1.f : std::sin(pi_2*std::min<Angle>(std::abs(angleErrorY), angleYDiffHipMax)/angleYDiffHipMax);
      float influenceFactorHipX = (legJointBalanceParams.hipParams.deltaAngleX == 0.f) ? 1.f : std::sin(pi_2*std::min<Angle>(std::abs(angleErrorX), 
        legJointBalanceParams.hipParams.deltaAngleX)/legJointBalanceParams.hipParams.deltaAngleX);

      pidGyroX_sum += gyroX;
      pidAngleX_sum += angleErrorX;
      pidGyroY_sum += gyroY;
      pidAngleY_sum += angleErrorY;

      const float& ankleRatioX = legJointBalanceParams.ankleParams.angleGyroRatioX;
      const float& ankleRatioY = legJointBalanceParams.ankleParams.angleGyroRatioY;
      const float& hipRatioX = legJointBalanceParams.hipParams.angleGyroRatioX;
      const float& hipRatioY = legJointBalanceParams.hipParams.angleGyroRatioY;

      float filteredErrorX_Ankle =
        (gyroX * ankleRatioX + angleErrorX * (1 - ankleRatioX)) * legJointBalanceParams.ankleParams.p_x * legJointBalanceParams.ankleParams.pidMultiplicator +
        (pidGyroX_sum * ankleRatioX + pidAngleX_sum * (1 - ankleRatioX)) * legJointBalanceParams.ankleParams.i_x * legJointBalanceParams.ankleParams.pidMultiplicator +
        ((pidGyroX_last - gyroX) * ankleRatioX + (pidAngleX_last - angleErrorX) * (1 - ankleRatioX)) * legJointBalanceParams.ankleParams.d_x * legJointBalanceParams.ankleParams.pidMultiplicator;
      float filteredErrorY_Ankle =
        (gyroY * ankleRatioY + angleErrorY * (1 - ankleRatioY)) * legJointBalanceParams.ankleParams.p_y * legJointBalanceParams.ankleParams.pidMultiplicator +
        (pidGyroY_sum * ankleRatioY + pidAngleY_sum * (1 - ankleRatioY)) * legJointBalanceParams.ankleParams.i_y * legJointBalanceParams.ankleParams.pidMultiplicator +
        ((pidGyroY_last - gyroY) * ankleRatioY + (pidAngleY_last - angleErrorY) * (1 - ankleRatioY)) * legJointBalanceParams.ankleParams.d_y * legJointBalanceParams.ankleParams.pidMultiplicator;

      float filteredErrorX_Hip =
        (gyroX * hipRatioX + angleErrorX * (1 - hipRatioX)) * legJointBalanceParams.hipParams.p_x * legJointBalanceParams.hipParams.pidMultiplicator +
        (pidGyroX_sum * hipRatioX + pidAngleX_sum * (1 - hipRatioX)) * legJointBalanceParams.hipParams.i_x * legJointBalanceParams.hipParams.pidMultiplicator +
        ((pidGyroX_last - gyroX) * hipRatioX + (pidAngleX_last - angleErrorX) * (1 - hipRatioX)) * legJointBalanceParams.hipParams.d_x * legJointBalanceParams.hipParams.pidMultiplicator;
      float filteredErrorY_Hip =
        (gyroY * hipRatioY + angleErrorY * (1 - hipRatioY)) * legJointBalanceParams.hipParams.p_y * legJointBalanceParams.hipParams.pidMultiplicator +
        (pidGyroY_sum * hipRatioY + pidAngleY_sum * (1 - hipRatioY)) * legJointBalanceParams.hipParams.i_y * legJointBalanceParams.hipParams.pidMultiplicator+
        ((pidGyroY_last - gyroY) * hipRatioY + (pidAngleY_last - angleErrorY) * (1 - hipRatioY)) * legJointBalanceParams.hipParams.d_y * legJointBalanceParams.hipParams.pidMultiplicator;

      PLOT("module:MotionCombinator:UpperBodyBalancer:errorX_Ankle",filteredErrorX_Ankle);
      PLOT("module:MotionCombinator:UpperBodyBalancer:errorY_Ankle",filteredErrorY_Ankle);
      PLOT("module:MotionCombinator:UpperBodyBalancer:errorX_Hip",filteredErrorX_Hip);
      PLOT("module:MotionCombinator:UpperBodyBalancer:errorY_Hip",filteredErrorY_Hip);

      pidAngleX_last = angleErrorX;
      pidGyroX_last = gyroX;
      pidAngleY_last = angleErrorY;
      pidGyroY_last = gyroY;

      kinematicRequest.offsets.angles[Joints::lHipPitch] += influenceFactorHipY * filteredErrorY_Hip;
      kinematicRequest.offsets.angles[Joints::rHipPitch] += influenceFactorHipY * filteredErrorY_Hip;
    
      kinematicRequest.offsets.angles[Joints::lAnklePitch] += influenceFactorAnkleY * filteredErrorY_Ankle;
      kinematicRequest.offsets.angles[Joints::rAnklePitch] += influenceFactorAnkleY * filteredErrorY_Ankle;

      kinematicRequest.offsets.angles[Joints::lHipRoll] += influenceFactorHipX * filteredErrorX_Hip;
      kinematicRequest.offsets.angles[Joints::rHipRoll] += influenceFactorHipX * filteredErrorX_Hip;
      kinematicRequest.offsets.angles[Joints::lAnkleRoll] += influenceFactorAnkleX * filteredErrorX_Ankle;
      kinematicRequest.offsets.angles[Joints::rAnkleRoll] += influenceFactorAnkleX * filteredErrorX_Ankle;

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
  if (footInAir == LEFT_FOOT)
  {
    sensorError.x() += ankleOffsetInfluenceOnSensorError * kinematicRequest.offsets.angles[Joints::rAnklePitch];
    sensorError.y() += ankleOffsetInfluenceOnSensorError * kinematicRequest.offsets.angles[Joints::rAnkleRoll];
  }
  else
  {
    sensorError.x() += ankleOffsetInfluenceOnSensorError * kinematicRequest.offsets.angles[Joints::lAnklePitch];
    sensorError.y() += ankleOffsetInfluenceOnSensorError * kinematicRequest.offsets.angles[Joints::lAnkleRoll];
  }

}

void CSConverter2019::resetSensorControl()
{
  pidGyroX_last = pidGyroY_last = 0.f;
  pidGyroX_sum = pidGyroY_sum = 0.f;
  pidAngleX_last = pidAngleY_last = 0.f;
  pidAngleX_sum = pidAngleY_sum = 0.f;
  gyroXBuffer.fill(0.f);
  gyroYBuffer.fill(0.f);
  angleXBuffer.fill(0.f);
  angleYBuffer.fill(0.f);
  accXBuffer.fill(0);
  lastSpeedX = 0.f;
  lastStepAccX = 0.f;
  interpolRatio = 0;
  interpolStartTime = theFrameInfo.time;
}

void CSConverter2019::updateKinematicRequest(KinematicRequest & kinematicRequest)
{
  Point targetCoM = theTargetCoM;
  Footpositions fp = theFootpositions;
  const WalkingEngineParams &curparams = theWalkingEngineParams;

  clearKinematicRequest(kinematicRequest);
  determineRunningState();
  
  if(isRunning)
  {
    gyroXBuffer.push_front(theInertialSensorData.gyro.x());
    gyroYBuffer.push_front(theInertialSensorData.gyro.y());
    
    angleXBuffer.push_front(useIMUModel ? theIMUModel.orientation.x() : theInertialSensorData.angle.x());
    angleYBuffer.push_front(useIMUModel ? theIMUModel.orientation.y() : theInertialSensorData.angle.y());
  
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
  sensorError = Vector2f::Zero();
  float xOffset = theWalkingEngineParams.comOffsets.xFixed;
  float yOffset = theWalkingEngineParams.comOffsets.yFixed;
  if (isRunning)
  {
    float baseXOffset = xOffset;
    float baseYOffset = yOffset;
    applyComShift(xOffset, yOffset);
    sensorError.x() += comShiftInfluenceOnSensorError * (xOffset - baseXOffset);
    sensorError.y() += comShiftInfluenceOnSensorError * (yOffset - baseYOffset);
    if (fp.speed.x > 0)
      xOffset += curparams.comOffsets.xSpeedDependent * fp.speed.x * 1000 / curparams.speedLimits.xForward;
    yOffset += curparams.comOffsets.ySpeedDependent[fp.speed.y < 0] * std::abs(fp.speed.y * 1000) / curparams.speedLimits.y;
  }
  Point comShift = handleArmContactState();


  // position left foot
  kinematicRequest.leftFoot[0] = (float)(currentStep.footPos[LEFT_FOOT].x * 1000 - xOffset * 1000 - comShift.x * 1000);
  kinematicRequest.leftFoot[1] = (float)(currentStep.footPos[LEFT_FOOT].y * 1000 - yOffset * 1000 - comShift.y* 1000) ;
  kinematicRequest.leftFoot[2] = (float)(currentStep.footPos[LEFT_FOOT].z * 1000);

  // position right foot
  kinematicRequest.rightFoot[0] = (float)(currentStep.footPos[RIGHT_FOOT].x * 1000 - xOffset * 1000 - comShift.x * 1000);
  kinematicRequest.rightFoot[1] = (float)(currentStep.footPos[RIGHT_FOOT].y * 1000 - yOffset * 1000 - comShift.y* 1000) ;
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

void CSConverter2019::determineFootInAir(const Footposition & curPos)
{
  // is a foot not on ground? If yes, which one?
  footInAir = -1;
  if(!curPos.onFloor[LEFT_FOOT])
    footInAir = LEFT_FOOT;
  else if(!curPos.onFloor[RIGHT_FOOT])
    footInAir = RIGHT_FOOT;
}

void CSConverter2019::applyFootPitchRollPD(Footposition & curPos)
{
  const WalkingEngineParams &curparams = theWalkingEngineParams;

  // PD controller for footpitch using body angle
  float roll = curparams.footMovement.footPitchPD[0] * theInertialSensorData.angle.x() +
    curparams.footMovement.footPitchPD[1] * theInertialSensorData.gyro.x() * 0.0075f;
  float pitch = curparams.footMovement.footPitchPD[0] * theInertialSensorData.angle.y() +
    curparams.footMovement.footPitchPD[1] * theInertialSensorData.gyro.y() * 0.0075f;
  PLOT("module:CSConverter2019:rollPD", roll);
  PLOT("module:CSConverter2019:pitchPD", pitch);

  // add roll, pitch to both foot rotation
  curPos.footPos[LEFT_FOOT].ry -= pitch;
  curPos.footPos[LEFT_FOOT].rx -= roll;
  curPos.footPos[RIGHT_FOOT].ry -= pitch;
  curPos.footPos[RIGHT_FOOT].rx -= roll;

  if(rotateLegWithBody)
  {
    determineFootInAir(curPos);
    if(footInAir != -1) // So yes, there is one not on ground
    {
      float yFactor = std::abs(curPos.speed.y) / curparams.speedLimits.y; // fraction of current speed and maximum speed in y-directon
      float xStepHeight = curPos.speed.x > 0 ? curparams.footMovement.stepHeight[0] : curparams.footMovement.stepHeight[1];
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

void CSConverter2019::calculateOdometry(const Footposition & curPos)
{
  Point positionBetweenFeet = (curPos.footPos[RIGHT_FOOT] + curPos.footPos[LEFT_FOOT]) * 0.5;
  positionBetweenFeet.r = robotPosition.r;

  if (odometryVariant == Odometry::ODOMETRY_FROM_WALKING_ENGINE)
  {
    offsetToRobotPoseAfterPreview = theFootSteps.robotPoseAfterStep - positionBetweenFeet;
    offsetToRobotPoseAfterPreview.rotate2D(-positionBetweenFeet.r);

    odometry = positionBetweenFeet - lastPositionBetweenFeet;
    odometry.rotate2D(-lastPositionBetweenFeet.r);

    originWCS = positionBetweenFeet;

    lastPositionBetweenFeet = positionBetweenFeet;
  }
  else if (odometryVariant == Odometry::ODOMETRY_FROM_INERTIA_MATRIX)
  {
    // Using the odometry calculated by the TorsoMatrixProvider we have to use the vector
    // from a foot on the ground to the origin given by the Provider. Then we can add this
    // vector to the foot positions in world coordinate system to get the origin in the
    // coordinate system used here. This way we can calculate the offset from the current origin 
    // to the final position of the robot.

    // First, convert the fromOriginToFoot from RCS to WCS
    Limbs::Limb limb = (theTorsoMatrix.leftSupportFoot ? Limbs::footLeft : Limbs::footRight);
    Point fromOriginToFoot(theRobotModel.limbs[limb].translation.x() / 1000,
                           theRobotModel.limbs[limb].translation.y() / 1000,
                           0,
                           theRobotModel.limbs[limb].rotation.getZAngle());

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

void CSConverter2019::toRobotCoords(StepData * requiredOffset, Point & newCoMTarget, Footposition & curPos, Point CoM)
{
  if(!curPos.customStepRunning)
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

void CSConverter2019::update(WalkingInfo &walkingInfo)
{
  updateWalkingInfo(walkingInfo);
};

void CSConverter2019::updateWalkingInfo(WalkingInfo & walkingInfo)
{
  const WalkingEngineParams &curparams = theWalkingEngineParams;

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
  walkingInfo.stabilityError = sensorError;

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

  walkingInfo.desiredBodyRot.y() -= l.rotation.getYAngle();  // Maybe that's a problem
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