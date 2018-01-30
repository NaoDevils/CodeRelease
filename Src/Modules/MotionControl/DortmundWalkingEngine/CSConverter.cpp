/*
Copyright 2011, Oliver Urbann
All rights reserved.

This file is part of MoToFlex.

MoToFlex is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

MoToFlex is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with MoToFlex.  If not, see <http://www.gnu.org/licenses/>.

Contact e-mail: oliver.urbann@tu-dortmund.de
*/

#include "Tools/Math/interpolator.h"
#include "CSConverter.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Motion/ForwardKinematic.h"

#define LOGGING
#include "Tools/Debugging/CSVLogger.h"

DECLARE_INTERPOLATE(orientationDelta, float, 0.01f, 0)
DECLARE_INTERPOLATE(speedDependentTilt, float, 0.01f, 0)
DECLARE_INTERPOLATE(xOffset, float, 0.0005f, 0)
DECLARE_INTERPOLATE_ARRAY(offsetRight, float, 0.0005f, 6)
DECLARE_INTERPOLATE_ARRAY(offsetLeft, float, 0.0005f, 6)
DECLARE_INTERPOLATE_VAR(CoMHeightOffset, float, 0)
DECLARE_INTERPOLATE(comYShift, float, 0.0001f, 0)
DECLARE_INTERPOLATE(comXShift, float, 0.0001f, 0)


CSConverter::CSConverter(
  const Footpositions         &theFootpositions,
  const TargetCoM             &theTargetCoM,
  const WalkingEngineParams   &theWalkingEngineParams,
  const ControllerParams      &theControllerParams,
  const ActualCoMRCS          &theActualCoMRCS,
  const FallDownState         &theFallDownState,
  const InertialSensorData    &theInertialSensorData,
  const FsrSensorData         &theFsrSensorData,
  const BodyTilt              &theBodyTilt,
  const TorsoMatrix           &theTorsoMatrix,
  const RobotModel            &theRobotModel,
  const RobotInfo             &theRobotInfo,
  const FootSteps             &theFootSteps,
  const ArmContact            &theArmContact,
  const RobotDimensions       &theRobotDimensions,
  const WalkCalibration       &theWalkCalibration) :
  theFootpositions(theFootpositions),
  theTargetCoM(theTargetCoM),
  theWalkingEngineParams(theWalkingEngineParams),
  theControllerParams(theControllerParams),
  theActualCoMRCS(theActualCoMRCS),
  theFallDownState(theFallDownState),
  theInertialSensorData(theInertialSensorData),
  theFsrSensorData(theFsrSensorData),
  theBodyTilt(theBodyTilt),
  theTorsoMatrix(theTorsoMatrix),
  theRobotModel(theRobotModel),
  theRobotInfo(theRobotInfo),
  theFootSteps(theFootSteps),
  theArmContact(theArmContact),
  theRobotDimensions(theRobotDimensions),
  theWalkCalibration(theWalkCalibration)
{
  reset();
}


CSConverter::~CSConverter(void)
{

}


void CSConverter::reset()
{

  lastFootPositionsValid = false;
  fallingDown = false;
  isRunning = false;
  lastPos = 0;
  RESET_INTERPOLATION(orientationDelta);
  bodyPitch[0] = bodyPitch[1] = 0.f;
}

void CSConverter::toRobotCoords(StepData *requiredOffset, Point newCoMTarget, Footposition curPos, Point CoM)
{
  StepData tempStep;
  Point temp;

#if 1 // Reactive Stepping as explained in PhD thesis

  // Calculate an extra angle to rotate around y if robot tilts, but apply only for swinging leg
  // Must be done here to avoid preview
  if (!curPos.instantKickRunning)
  {
#if 0
    if (theWalkingEngineParams.sensorControlRatio[0] > 0)
    {
      // Calculate an extra angle to rotate around y if robot tilts, but apply only for swinging leg
      int footInAir = -1;
      if (!curPos.onFloor[LEFT_FOOT])
        footInAir = LEFT_FOOT;
      else if (!curPos.onFloor[RIGHT_FOOT])
        footInAir = RIGHT_FOOT;

      static float angleY = 0;
      angleY = params.filter_alpha * -theInertialSensorData.angle.y() + (1 - params.filter_alpha) * angleY;

      if (!curPos.onFloor[LEFT_FOOT] || !curPos.onFloor[RIGHT_FOOT])
      {

        Point stanceToAirFootVec = curPos.footPos[footInAir] - curPos.footPos[!footInAir];
        stanceToAirFootVec.rotate2D(-curPos.direction);

        float intfac = 1;
        unsigned int maxTime = curPos.singleSupportLen / 3;
        if (curPos.frameInPhase < maxTime)
          intfac = (float)curPos.frameInPhase / maxTime;

        stanceToAirFootVec.rotateAroundY(intfac * angleY);
        stanceToAirFootVec.rotate2D(curPos.direction);
        Point unmodified = curPos.footPos[footInAir];

        curPos.footPos[footInAir].x = curPos.footPos[!footInAir].x + stanceToAirFootVec.x;
        curPos.footPos[footInAir].y = curPos.footPos[!footInAir].y + stanceToAirFootVec.y;
        curPos.footPos[footInAir].z = curPos.footPos[!footInAir].z + stanceToAirFootVec.z;
        curPos.footPos[footInAir].ry += intfac * angleY;
        curPos.footPos[footInAir].rx -= intfac * theInertialSensorData.angle.x();

        lastOffset[footInAir] = curPos.footPos[footInAir] - unmodified;
      }
      curPos.footPos[!footInAir] += lastOffset[!footInAir];


      lastOffset[!footInAir] *= params.standup_fac;

      newCoMTarget.z -= std::abs(angleY) *  params.lower_CoM_fac;
    }
    else
#endif
    {
      // is a foot not on ground? If yes, which one?
      int footInAir = -1;
      if (!curPos.onFloor[LEFT_FOOT])
        footInAir = LEFT_FOOT;
      else if (!curPos.onFloor[RIGHT_FOOT])
        footInAir = RIGHT_FOOT;

      if (footInAir != -1)
      {
        // So yes, there is one not on ground
        float yFac; // fraction of current speed and maximum speed
        yFac = std::abs(curPos.speed.y) / theWalkingEngineParams.speedLimits.y;
        float xStepHeight = curPos.speed.x > 0 ? theWalkingEngineParams.footMovement.stepHeight[0] : theWalkingEngineParams.footMovement.stepHeight[1];
        float stepHeight = (1 - yFac) * xStepHeight + yFac * theWalkingEngineParams.footMovement.stepHeight[2]; // For sidesteps, raise feet higher if faster, but here calculated only for the next value

        float heightOverGroundFac = curPos.footPos[footInAir].z / stepHeight; // fraction of current height over ground vs. maximum height over ground
        // PD controller for footpitch using body angle
        float cX = 0;
        float cY = theWalkingEngineParams.footMovement.footPitchPD[0] * theInertialSensorData.angle.y() +
          theWalkingEngineParams.footMovement.footPitchPD[1] * theInertialSensorData.gyro.y() * 0.0075f;

        // But it depends also on the height coefficient
        float chX = heightOverGroundFac * cX;
        float chY = heightOverGroundFac * cY;

        // rotate vector from stand foot to swing foot arounc chX,chY 
        // and add to stand foot pos to get adjusted swing foot position
        // also add chX, chY to foot rotation
        // similar to z and rotation adaption due to roll/tiltController activated with steppingRotSpeedCapture
        // ------------ DONT USE BOTH! ---------------
        Point stanceToAirFootVec = curPos.footPos[footInAir] - curPos.footPos[!footInAir]; // Vector from stance to air foot origin
        // Rotate vector in robot coordinate system
        stanceToAirFootVec.rotate2D(-curPos.direction);
        stanceToAirFootVec.rotateAroundX(-chX);
        stanceToAirFootVec.rotateAroundY(-chY);
        stanceToAirFootVec.rotate2D(curPos.direction);
        curPos.footPos[footInAir] = curPos.footPos[!footInAir] + stanceToAirFootVec;
        curPos.footPos[footInAir].ry -= chY;
        curPos.footPos[footInAir].rx -= chX;
        PLOT("module:CSConverter:angleY", theInertialSensorData.angle.y());
        PLOT("module:CSConverter:gyroY", theInertialSensorData.gyro.y());
        PLOT("module:CSConverter:cY", cY);
      }
    }
  }
#endif

  // Calculate the distance between current CoM and feet
  tempStep.footPos[RIGHT_FOOT] = curPos.footPos[RIGHT_FOOT] - newCoMTarget;
  tempStep.footPos[LEFT_FOOT] = curPos.footPos[LEFT_FOOT] - newCoMTarget;

  // Rotate around the CoM (z axis)
  *requiredOffset = tempStep;
  requiredOffset->footPos[RIGHT_FOOT].x = tempStep.footPos[RIGHT_FOOT].x*std::cos(curPos.direction) + tempStep.footPos[RIGHT_FOOT].y*std::sin(curPos.direction);
  requiredOffset->footPos[LEFT_FOOT].x = tempStep.footPos[LEFT_FOOT].x*std::cos(curPos.direction) + tempStep.footPos[LEFT_FOOT].y*std::sin(curPos.direction);

  requiredOffset->footPos[RIGHT_FOOT].y = tempStep.footPos[RIGHT_FOOT].y*std::cos(curPos.direction) - tempStep.footPos[RIGHT_FOOT].x*std::sin(curPos.direction);
  requiredOffset->footPos[LEFT_FOOT].y = tempStep.footPos[LEFT_FOOT].y*std::cos(curPos.direction) - tempStep.footPos[LEFT_FOOT].x*std::sin(curPos.direction);


  float targetRot;

  for (int foot = LEFT_FOOT; foot <= RIGHT_FOOT; foot++)
  {
    if (theWalkingEngineParams.sensorControl.steppingRotSpeedCapture > 0)
    {
      float rotSpeed = 0.f;
      if (!curPos.onFloor[foot])
      {
        targetRot = curPos.pitch;
        rotSpeed = theWalkingEngineParams.sensorControl.steppingRotSpeedCapture * sgn(targetRot - bodyPitch[foot]); // 0.04
      }
      else
      {
        targetRot = 0;
        rotSpeed = theWalkingEngineParams.sensorControl.steppingRotSpeedUprect * sgn(-bodyPitch[foot]); // 0.001
      }
      if (std::abs(std::abs(targetRot) - std::abs(bodyPitch[foot])) > std::abs(rotSpeed))
        bodyPitch[foot] += rotSpeed;
      else
        bodyPitch[foot] = targetRot;
    }

    requiredOffset->footPos[foot].rotateAroundY(-bodyPitch[foot] - theWalkCalibration.fixedOffset.ry);
    bodyTiltApplied = true;
    if (theWalkingEngineParams.sensorControl.steppingRotSpeedCapture > 0)
    {
      float dz = requiredOffset->footPos[foot].z - requiredOffset->footPos[foot].z * std::abs(std::cos(-bodyPitch[foot]));
      requiredOffset->footPos[foot] = requiredOffset->footPos[foot].norm() * (requiredOffset->footPos[foot].getPositionVecLen() - std::abs(dz));
    }
  }

  // Rotate the foot positions relative to CoM around the x axis (roll)
  tempStep = *requiredOffset;
  tempStep.footPos[RIGHT_FOOT].y = requiredOffset->footPos[RIGHT_FOOT].y*std::cos(-curPos.roll) - requiredOffset->footPos[RIGHT_FOOT].z*std::sin(-curPos.roll);
  tempStep.footPos[RIGHT_FOOT].z = requiredOffset->footPos[RIGHT_FOOT].y*std::sin(-curPos.roll) + requiredOffset->footPos[RIGHT_FOOT].z*std::cos(-curPos.roll);

  tempStep.footPos[LEFT_FOOT].y = requiredOffset->footPos[LEFT_FOOT].y*std::cos(-curPos.roll) - requiredOffset->footPos[LEFT_FOOT].z*std::sin(-curPos.roll);
  tempStep.footPos[LEFT_FOOT].z = requiredOffset->footPos[LEFT_FOOT].y*std::sin(-curPos.roll) + requiredOffset->footPos[LEFT_FOOT].z*std::cos(-curPos.roll);
  *requiredOffset = tempStep;

  // Add the position of the CoM to get the feet positions in robot coordinate system
  requiredOffset->footPos[RIGHT_FOOT] += CoM;
  requiredOffset->footPos[LEFT_FOOT] += CoM;

  // Set the foot pitch
  requiredOffset->footPos[LEFT_FOOT].ry = -bodyPitch[LEFT_FOOT] - theWalkCalibration.fixedOffset.ry;
  requiredOffset->footPos[RIGHT_FOOT].ry = -bodyPitch[RIGHT_FOOT] - theWalkCalibration.fixedOffset.ry;
  requiredOffset->footPos[LEFT_FOOT].ry += curPos.footPos[LEFT_FOOT].ry;
  requiredOffset->footPos[RIGHT_FOOT].ry += curPos.footPos[RIGHT_FOOT].ry;

  // Set the foot roll
  requiredOffset->footPos[LEFT_FOOT].rx = -curPos.roll;
  requiredOffset->footPos[RIGHT_FOOT].rx = -curPos.roll;
  requiredOffset->footPos[LEFT_FOOT].rx += curPos.footPos[LEFT_FOOT].rx;
  requiredOffset->footPos[RIGHT_FOOT].rx += curPos.footPos[RIGHT_FOOT].rx;

  // Set the foot Orientation
  requiredOffset->footPos[RIGHT_FOOT].r = curPos.footPos[RIGHT_FOOT].r - curPos.direction;
  requiredOffset->footPos[LEFT_FOOT].r = curPos.footPos[LEFT_FOOT].r - curPos.direction;

  requiredOffset->onFloor[LEFT_FOOT] = curPos.onFloor[LEFT_FOOT];
  requiredOffset->onFloor[RIGHT_FOOT] = curPos.onFloor[RIGHT_FOOT];

  Point gCoM = CoM;
  gCoM.rotate2D(curPos.direction);
  robotPosition = newCoMTarget - gCoM;
  robotPosition.r = curPos.direction;

  // Calculate new position based on the definition of our robot coordinate frame.
  // This will be used for odometry and for the offsetToRobotPoseAfterPreview.
  // The CameraMatrix calculation must be based on this same 
  //   robot coordinate frame to provide consistent information!


  // ... for position between feet with rotation of body
  Point newPos = (curPos.footPos[RIGHT_FOOT] + curPos.footPos[LEFT_FOOT])*0.5;
  newPos.r = robotPosition.r;

  fixedOdometryRobotPose.translation.x() = newPos.x * 1000;
  fixedOdometryRobotPose.translation.y() = newPos.y * 1000;
  fixedOdometryRobotPose.rotation = newPos.r;

#define ODOMETRY_FROM_INERTIA_MATRIX

  // ODOMETRY_FROM_WALKING_ENGINE
  if (params.odometryVariant == 1)
  {
    // This offsetToRobotPoseAfterPreview where used for the odometry calculated here.

    offsetToRobotPoseAfterPreview = theFootSteps.robotPoseAfterStep;
    offsetToRobotPoseAfterPreview -= newPos;
    offsetToRobotPoseAfterPreview.rotate2D(-newPos.r);

    // odometry for pose between feet
    odometry = newPos - lastPos;
    odometry.rotate2D(-lastPos.r);
    if (theRobotInfo.hasFeature(RobotInfo::zGyro))
      odometry.r = theInertialSensorData.gyro.z() * 0.01f; // was -gyro, but BHuman changed sign at NaoProvider
    lastPos = newPos;
    originWCS = lastPos;
  }

  if (params.odometryVariant == 0)
  {
    //ODOMETRY_FROM_INERTIA_MATRIX

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

    fromOriginToFoot.rotate2D(newPos.r);

    if (theTorsoMatrix.leftSupportFoot)
      originWCS = curPos.footPos[LEFT_FOOT] - fromOriginToFoot;
    else
      originWCS = curPos.footPos[RIGHT_FOOT] - fromOriginToFoot;

    offsetToRobotPoseAfterPreview = originWCS*-1 + theFootSteps.robotPoseAfterStep;

    // Set correct rotation from current pose to afterPreview
    offsetToRobotPoseAfterPreview.r = -newPos.r + theFootSteps.robotPoseAfterStep.r;

    // Convert the vector from the WCS to the RCS
    offsetToRobotPoseAfterPreview.rotate2D(-newPos.r);

    // Now calculate the offset. Code from TorsoMatrixProvider.

    if (lastTorsoMatrix.translation.z() != 0.)
    {
      Pose3f odometryOffset3D(lastTorsoMatrix);
      odometryOffset3D.conc(theTorsoMatrix.offset);
      odometryOffset3D.conc(theTorsoMatrix.inverse());

      if (theRobotInfo.hasFeature(RobotInfo::zGyro))
        odometry.r = theInertialSensorData.gyro.z() * 0.01f; // was -gyro, but BHuman changed sign at NaoProvider
      else
        odometry.r = odometryOffset3D.rotation.getZAngle();

      odometry.x = odometryOffset3D.translation.x() / 1000;
      odometry.y = odometryOffset3D.translation.y() / 1000;

    }
    ASSERT(odometry.x == odometry.x);
    ASSERT(odometry.y == odometry.y);
    ASSERT(odometry.r == odometry.r);
    (Pose3f&)lastTorsoMatrix = theTorsoMatrix;

    LOG("WalkingEngine", "fromOriginToFoot x", fromOriginToFoot.x);
    LOG("WalkingEngine", "fromOriginToFoot y", fromOriginToFoot.y);
    LOG("WalkingEngine", "fromOriginToFoot r", fromOriginToFoot.r);
    //odometry=Point(odometryOffset.translation.x/1000, odometryOffset.translation.y/1000, 0, odometryOffset.rotation);
  }

  LOG("WalkingEngine", "Odometry x", odometry.x);
  LOG("WalkingEngine", "Odometry y", odometry.y);
  LOG("WalkingEngine", "Odometry r", odometry.r);
  LOG("WalkingEngine", "offsetToRobotPoseAfterPreview x", offsetToRobotPoseAfterPreview.x);
  LOG("WalkingEngine", "offsetToRobotPoseAfterPreview y", offsetToRobotPoseAfterPreview.y);
  LOG("WalkingEngine", "offsetToRobotPoseAfterPreview r", offsetToRobotPoseAfterPreview.r);

}


void CSConverter::updateKinematicRequest(KinematicRequest &kinematicRequest)
{
  DECLARE_PLOT("module:CSConverter:angleY");
  DECLARE_PLOT("module:CSConverter:gyroY");
  DECLARE_PLOT("module:CSConverter:cY");
  DECLARE_PLOT("module:CSConverter:Modification_X");
  DECLARE_PLOT("module:CSConverter:Modification_Y");
  Point targetCoM;

  params.handle();

  for (int i = 2; i < Joints::numOfJoints; i++) {
    kinematicRequest.offsets.angles[i] = 0;
  }
  for (int i = 0; i < 6; i++) {
    kinematicRequest.body[i] = JointAngles::ignore;
  }


  if (!isRunning && theFootpositions.running)
  {
    isRunning = true;
  }

  if (isRunning && !theFootpositions.running)
  {
    reset();
    isRunning = false;
  }

  bodyTiltApplied = false;

  if (isRunning)
  {
    Footpositions fp = theFootpositions;

    if (fp.speed.x >= 0)
    {
      float fac = std::abs(fp.speed.x * 1000) / theWalkingEngineParams.speedLimits.xForward;
      speedDependentTilt = fac * theWalkingEngineParams.comOffsets.tiltSpeedDependent[1];
    }
    else
    {
      float fac = std::abs(fp.speed.x * 1000) / theWalkingEngineParams.speedLimits.xBackward;
      speedDependentTilt = fac * theWalkingEngineParams.comOffsets.tiltSpeedDependent[0];
    }

    fp.pitch += theBodyTilt.y + INTERPOLATE(speedDependentTilt, speedDependentTilt);
    fp.roll += theBodyTilt.x;

    isInstantKickRunning = fp.instantKickRunning;

    targetCoM = theTargetCoM;
    targetCoM += theWalkCalibration.fixedOffset;
    fp.roll += theWalkCalibration.fixedOffset.rx;

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
    currentStep = theFootpositions;
    targetCoM = 0;
    bodyPitch[0] = bodyPitch[1] = 0.f;
  }

  Point speed = (targetCoM - lastTargetCoM) / theControllerParams.dt;
  lastTargetCoM = targetCoM;

  acc = speed - lastSpeed / theControllerParams.dt;
  lastSpeed = speed;


  isLeavingPossible = false;
  if (isRunning)
  {
    // check if the robot stands on both feets (CoM in the middle),
    // and does not move (speed of CoM < 0.01)
    if (std::abs(currentStep.footPos[LEFT_FOOT].x) < theWalkingEngineParams.walkTransition.stopPosThresholdX &&
      std::abs(currentStep.footPos[RIGHT_FOOT].x) < theWalkingEngineParams.walkTransition.stopPosThresholdX &&
      std::abs(currentStep.footPos[LEFT_FOOT].y + currentStep.footPos[RIGHT_FOOT].y) < theWalkingEngineParams.walkTransition.stopPosThresholdY &&
      std::abs(speed.x) < theWalkingEngineParams.walkTransition.stopSpeedThresholdX &&
      std::abs(speed.y) < theWalkingEngineParams.walkTransition.stopSpeedThresholdY)
      isLeavingPossible = true;
  }
  else
  {
    isLeavingPossible = true;
  }

  if (currentStep.onFloor[LEFT_FOOT] && currentStep.onFloor[RIGHT_FOOT])
    kinematicRequest.kinematicType = KinematicRequest::feet;
  if (currentStep.onFloor[LEFT_FOOT] && !currentStep.onFloor[RIGHT_FOOT])
  {
    kinematicRequest.kinematicType = KinematicRequest::bodyAndLeftFoot;
  }
  if (!currentStep.onFloor[LEFT_FOOT] && currentStep.onFloor[RIGHT_FOOT])
  {
    kinematicRequest.kinematicType = KinematicRequest::bodyAndRightFoot;
  }

  WalkingEngineParams curparams = (WalkingEngineParams &)theWalkingEngineParams;

#if 0
  RotationMatrix rotM(theInertialSensorData.angle.x(),
    theInertialSensorData.angle.y(), 0);

  Vector3f rotAcc = rotM * theInertialSensorData.acc;
#else
  const Vector3f &rotAcc = theInertialSensorData.acc;
#endif

  static int zerocount = 0;
  if (theFootpositions.speed == Point() && zerocount <= 100)
    zerocount++;
  if (!(theFootpositions.speed == Point()))
    zerocount = 0;

  double xOffset = curparams.comOffsets.xFixed;
  if (zerocount < 100)
  {
    accXBuffer.push_front(rotAcc.x() * -0.01558f);
    float avgAccX = accXBuffer.average();
    float accFactor = std::abs(accXBuffer[0]) / 0.3f;
    //double alphaFactor = theWalkingEngineParams.accXAlpha*accFactor;
    float alphaFactor = curparams.sensorControl.accXAlpha*accFactor;
    avgAccX = sgn(avgAccX)*std::min<float>(std::abs(avgAccX), 0.03f) - curparams.comOffsets.xFixed;
    xOffset = (curparams.comOffsets.xFixed * (1 - alphaFactor) - avgAccX*alphaFactor);
  }
  else
    accXBuffer.clear();

  PLOT("module:CSConverter:xOffset", xOffset);

  // Handle ArmContact while walking - Arne 10.02.17 //
  ArmContact::ArmContactState stateLeftArm = theArmContact.armContactStateLeft;
  ArmContact::ArmContactState stateRightArm = theArmContact.armContactStateRight;
  float comYShift = 0.0;
  float comXShift = 0.0;
  if ((stateLeftArm == ArmContact::ArmContactState::Back || stateLeftArm == ArmContact::ArmContactState::Front)
    && stateRightArm == ArmContact::ArmContactState::None)
  {
    comYShift = theWalkingEngineParams.comOffsets.yArmContact[0];
    comXShift = theWalkingEngineParams.comOffsets.xArmContact;
  }
  else if ((stateRightArm == ArmContact::ArmContactState::Back || stateRightArm == ArmContact::ArmContactState::Front)
    && stateLeftArm == ArmContact::ArmContactState::None)
  {
    comYShift = theWalkingEngineParams.comOffsets.yArmContact[1];
    comXShift = theWalkingEngineParams.comOffsets.xArmContact;
  }
  else if ((stateRightArm == ArmContact::ArmContactState::Back &&  stateLeftArm == ArmContact::ArmContactState::Back)
    || (stateRightArm == ArmContact::ArmContactState::Front && stateLeftArm == ArmContact::ArmContactState::Front)) {
    comXShift = theWalkingEngineParams.comOffsets.xArmContact;
  }
  // Handle ArmContact while walking - Arne 10.02.17 //

  double yOffset;
  int yDir = theFootpositions.speed.y < 0;
  yOffset = theWalkingEngineParams.comOffsets.ySpeedDependent[yDir] * 1000 * std::abs(theFootpositions.speed.y * 1000) / theWalkingEngineParams.speedLimits.y;

  double dynXOffset = 0;
  if (theFootpositions.speed.x > 0)
    dynXOffset = theWalkingEngineParams.comOffsets.xSpeedDependent * theFootpositions.speed.x * 1000 / theWalkingEngineParams.speedLimits.xForward;

  xOffset += dynXOffset;

  // position left foot
  kinematicRequest.leftFoot[0] = (float)(currentStep.footPos[LEFT_FOOT].x * 1000 - (INTERPOLATE(xOffset, (float)xOffset) * 1000) - (INTERPOLATE(comXShift, (float)comXShift) * 1000));
  kinematicRequest.leftFoot[1] = (float)(currentStep.footPos[LEFT_FOOT].y * 1000 - yOffset - theWalkingEngineParams.comOffsets.yFixed * 1000 - (INTERPOLATE(comYShift, (float)comYShift) * 1000)) ;
  kinematicRequest.leftFoot[2] = (float)(currentStep.footPos[LEFT_FOOT].z * 1000);

  // position right foot
  kinematicRequest.rightFoot[0] = (float)(currentStep.footPos[RIGHT_FOOT].x * 1000 - (INTERPOLATE(xOffset, (float)xOffset) * 1000) - (INTERPOLATE(comXShift, (float)comXShift) * 1000));
  kinematicRequest.rightFoot[1] = (float)(currentStep.footPos[RIGHT_FOOT].y * 1000 - yOffset - theWalkingEngineParams.comOffsets.yFixed * 1000 - (INTERPOLATE(comYShift, (float)comYShift) * 1000));
  kinematicRequest.rightFoot[2] = (float)(currentStep.footPos[RIGHT_FOOT].z * 1000);

  kinematicRequest.leftFoot[3] = (float)(currentStep.footPos[LEFT_FOOT].rx);
  kinematicRequest.leftFoot[4] = (float)(currentStep.footPos[LEFT_FOOT].ry);
  kinematicRequest.leftFoot[5] = (float)(currentStep.footPos[LEFT_FOOT].r);

  // rotation right food
  kinematicRequest.rightFoot[3] = (float)(currentStep.footPos[RIGHT_FOOT].rx);
  kinematicRequest.rightFoot[4] = (float)(currentStep.footPos[RIGHT_FOOT].ry);
  kinematicRequest.rightFoot[5] = (float)(currentStep.footPos[RIGHT_FOOT].r);

  LOG("KinematicRequest", "RCS left Foot x", kinematicRequest.leftFoot[0]);
  LOG("KinematicRequest", "RCS left Foot y", kinematicRequest.leftFoot[1]);
  LOG("KinematicRequest", "RCS left Foot z", kinematicRequest.leftFoot[2]);
  LOG("KinematicRequest", "RCS right Foot x", kinematicRequest.rightFoot[0]);
  LOG("KinematicRequest", "RCS right Foot y", kinematicRequest.rightFoot[1]);
  LOG("KinematicRequest", "RCS right Foot z", kinematicRequest.rightFoot[2]);

  // body tilt
  kinematicRequest.body[4] = 0;

  // body shift
  kinematicRequest.body[3] = 0;


  lastRequest = kinematicRequest;
  lastFootPositionsValid = true;
  fallingDown = false;


  for (int i = (int)Joints::lHipYawPitch; i < 12; i++)
  {
    kinematicRequest.offsets.angles[i] = 0;
  }

  if (currentStep.onFloor[LEFT_FOOT] && currentStep.onFloor[RIGHT_FOOT])
  {

  }
  if (currentStep.onFloor[LEFT_FOOT] && !currentStep.onFloor[RIGHT_FOOT])
  {
    for (int i = 0; i < 6; i++)
    {
      {
        kinematicRequest.offsets.angles[i + (int)Joints::lHipYawPitch] = theWalkingEngineParams.jointCalibration.offsetLeft[i];
      }
      INTERPOLATE_ARRAY_ELEMENT_AND_STORE(offsetLeft, kinematicRequest.offsets.angles[i + (int)Joints::lHipYawPitch], i);
    }
  }
  if (!currentStep.onFloor[LEFT_FOOT] && currentStep.onFloor[RIGHT_FOOT])
  {
    for (int i = 0; i<6; i++)
    {
      {
        kinematicRequest.offsets.angles[i + (int)Joints::rHipYawPitch] = theWalkingEngineParams.jointCalibration.offsetRight[i];
      }

      INTERPOLATE_ARRAY_ELEMENT_AND_STORE(offsetRight, kinematicRequest.offsets.angles[i + (int)Joints::rHipYawPitch], i);
    }
  }
  for (int i = 0; i < 6; i++)
    ASSERT(kinematicRequest.leftFoot[i] == kinematicRequest.leftFoot[i]);
}

void CSConverter::updateWalkingInfo(WalkingInfo &walkingInfo)
{
  walkingInfo.robotPosition.translation.x() = robotPosition.x;
  walkingInfo.robotPosition.translation.y() = robotPosition.y;
  walkingInfo.robotPosition.rotation = robotPosition.r;
  walkingInfo.isInstantKickRunning = isInstantKickRunning;
  walkingInfo.isRunning = isRunning;
  walkingInfo.bodyTiltApplied = bodyTiltApplied;
  walkingInfo.desiredBodyRot.y() = 0;
  walkingInfo.onFloor[0] = currentStep.onFloor[0];
  walkingInfo.onFloor[1] = currentStep.onFloor[1];
  if (bodyTiltApplied)
  {
    walkingInfo.desiredBodyRot.x() = 0;
    walkingInfo.desiredBodyRot.y() = speedDependentTilt + theWalkCalibration.fixedOffset.ry;
  }
  Pose3f limbs[Limbs::numOfLimbs];
  JointAngles jA;
  for (int i = 0; i < 12; i++)
    jA.angles[i + Joints::lHipYawPitch] = theWalkingEngineParams.jointCalibration.jointCalibration[i];
  for (int i = 0; i < 6; i++)
    jA.angles[i + Joints::lHipYawPitch] += theWalkingEngineParams.jointCalibration.offsetLeft[i];
  for (int i = 0; i < 6; i++)
    jA.angles[i + Joints::rHipYawPitch] += theWalkingEngineParams.jointCalibration.offsetRight[i];
  ForwardKinematic::calculateLegChain(currentStep.onFloor[0], jA, theRobotDimensions, limbs);
  Pose3f l;
  if (currentStep.onFloor[0])
    l = limbs[Limbs::footLeft];
  else
    l = limbs[Limbs::footRight];

  walkingInfo.desiredBodyRot.y() -= l.rotation.getYAngle();  // Maybe that's a problem

  //for (int i = 2; i < 5; i++)
  //  walkingInfo.desiredBodyRot.y() += theWalkingEngineParams.jointCalibration.jointCalibration[currentStep.onFloor[1] * 6 + i];
  if (fallingDown)
    walkingInfo.isLeavingPossible = true;
  else
    walkingInfo.isLeavingPossible = isLeavingPossible;

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
