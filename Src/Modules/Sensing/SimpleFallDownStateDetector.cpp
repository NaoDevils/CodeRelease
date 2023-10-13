/**
* @file SimpleFallDownStateDetector.cpp
*
* This file implements a module that provides information about the current state of the robot's body.
*
* @author Dominik Brämer
*/

#include "SimpleFallDownStateDetector.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Build.h"

SimpleFallDownStateDetector::SimpleFallDownStateDetector()
{
  DECLARE_PLOT("module:SimpleFallDownStateDetector:AccZ:AccZCounter");
  DECLARE_PLOT("module:SimpleFallDownStateDetector:FSR:fsrRef");

  stateQueue.push(FallDownState::undefined);
  lastDirection = FallDownState::none;
  lastState = FallDownState::undefined;
  onGroundCounter = 0;
  standUpCounter = 0;
  accZCounter = 0;
}

void SimpleFallDownStateDetector::update(FallDownState& fallDownState)
{
  updateVariable(fallDownState);
  detectFlying(fallDownState);
  detectHeldOnOneShoulder(fallDownState);
  detectDiving(fallDownState);
  detectFalling(fallDownState);
  detectOnGround(fallDownState);
  detectLyiningStill(fallDownState);
  detectDirection(fallDownState);
  detectStandingUp(fallDownState);
  detectUpright(fallDownState);
  resolveFallDownState(fallDownState);
}

void SimpleFallDownStateDetector::updateVariable(FallDownState& fallDownState)
{
  angleXZ = theJoinedIMUData.imuData[anglesource].angle.y();
  angleYZ = theJoinedIMUData.imuData[anglesource].angle.x();

  gyroX = theJoinedIMUData.imuData[gyrosource].gyro.x().toDegrees();
  gyroY = theJoinedIMUData.imuData[gyrosource].gyro.y().toDegrees();

  float accZValue = theJoinedIMUData.imuData[accsource].acc.z();
  if (accZValue > 0.25)
    accZCounter++;
  else
    accZCounter = 0;

  PLOT("module:SimpleFallDownStateDetector:AccZ:AccZCounter", accZCounter);
  PLOT("module:SimpleFallDownStateDetector:FSR:fsrRef", theFsrSensorData.fsrRef);
  mightUpright = (angleXZ <= fallDownAngleFront && angleXZ >= fallDownAngleBack && std::abs(angleYZ) <= fallDownAngleSide);

  lastDirection = fallDownState.direction;

  fsrSum = theFsrSensorData.leftTotal + theFsrSensorData.rightTotal;
  if (fsrMin > fsrSum)
    fsrMin = fsrSum;
  checkFsrSum(fallDownState);

  fallDownState.standUpOnlyWhenLyingStill = standUpOnlyWhenLyingStill;
}

void SimpleFallDownStateDetector::detectFlying(FallDownState& fallDownState)
{
  (abs(theFsrSensorData.calcSupportFoot()) < 0.1f) ? (doubleSupport = true) : (doubleSupport = false);

  bool endFlying = false;
  if (mightFlying() && mightUpright)
    stateQueue.push(FallDownState::flying);
  else if (fallDownState.state == FallDownState::flying && ((fsrSum - fsrMin) > theFsrSensorData.fsrRef * fsrWeightOnGround) && doubleSupport)
    endFlying = true;

  if (fallDownState.state == FallDownState::flying && !endFlying)
    stateQueue.push(FallDownState::flying);
}

void SimpleFallDownStateDetector::detectHeldOnOneShoulder(FallDownState& fallDownState)
{
  if (notOnGround() && std::abs(angleYZ) > minHeldOnShoulderAngle && std::abs(angleYZ) < maxHeldOnShoulderAngle && std::abs(angleXZ) < heldOnShoulderTolerance)
    stateQueue.push(FallDownState::heldOnOneShoulder);
}

void SimpleFallDownStateDetector::detectFalling(FallDownState& fallDownState)
{
  // state falling if average angle of robot is too high and robot is not trying to get up
  if (lastState == FallDownState::standingUp)
  {
    if (angleXZ > fallDownAngleFront + standingUpTolerance || angleXZ < fallDownAngleBack - standingUpTolerance || std::abs(angleYZ) > fallDownAngleSide + standingUpTolerance)
      stateQueue.push(FallDownState::falling);
  }
  else
  {
    if (angleXZ > fallDownAngleFront || angleXZ < fallDownAngleBack || std::abs(angleYZ) > fallDownAngleSide)
      stateQueue.push(FallDownState::falling);
  }
}

void SimpleFallDownStateDetector::detectOnGround(FallDownState& fallDownState)
{
  if ((angleXZ > uprightAngleThreshold || (angleXZ > 30_deg && angleYZ > 30_deg) || (angleXZ > 30_deg && angleYZ < -30_deg)))
    stateQueue.push(FallDownState::onGround);
  else if ((angleXZ < -uprightAngleThreshold || (angleXZ < -30_deg && angleYZ > 30_deg) || (angleXZ < -30_deg && angleYZ < -30_deg)))
    stateQueue.push(FallDownState::onGround);
  else if (angleYZ < -uprightAngleThreshold)
    stateQueue.push(FallDownState::onGround);
  else if (angleYZ > uprightAngleThreshold)
    stateQueue.push(FallDownState::onGround);
}

void SimpleFallDownStateDetector::detectLyiningStill(FallDownState& fallDownState)
{
  if (abs(gyroX) < maxLyingStillThreshold && abs(gyroY) < maxLyingStillThreshold && (fallDownState.state == FallDownState::onGround || fallDownState.state == FallDownState::onGroundLyingStill))
    stateQueue.push(FallDownState::onGroundLyingStill);
}

void SimpleFallDownStateDetector::detectDiving(FallDownState& fallDownState)
{
  if (theMotionInfo.inBlockMotion())
    stateQueue.push(FallDownState::diving);
}

void SimpleFallDownStateDetector::detectDirection(FallDownState& fallDownState)
{
  if (angleXZ > fallDownAngleFront)
    fallDownState.direction = FallDownState::front;
  else if (angleXZ < fallDownAngleBack)
    fallDownState.direction = FallDownState::back;
  else if (angleYZ < -uprightAngleThreshold)
    fallDownState.direction = FallDownState::left;
  else if (angleYZ > uprightAngleThreshold)
    fallDownState.direction = FallDownState::right;
  else if (lastState != FallDownState::upright && gyroY > directionGyroThreshold)
    fallDownState.direction = FallDownState::front;
  else if (lastState != FallDownState::upright && gyroY < -directionGyroThreshold)
    fallDownState.direction = FallDownState::back;
  else
    fallDownState.direction = FallDownState::none;
}

void SimpleFallDownStateDetector::detectStandingUp(FallDownState& fallDownState)
{
  if (theMotionInfo.inStandUpMotion())
    stateQueue.push(FallDownState::standingUp);
}

void SimpleFallDownStateDetector::detectUpright(FallDownState& fallDownState)
{
  if (mightUpright && fallDownState.direction == FallDownState::none)
    stateQueue.push(FallDownState::upright);
}

void SimpleFallDownStateDetector::checkFsrSum(FallDownState& fallDownState)
{
  if constexpr (Build::targetRobot())
  {
    float fsrDbgCountThreshold = fsrDbgThreshold / theFrameInfo.cycleTime;

    if (theGameInfo.state == STATE_INITIAL && fallDownState.state == FallDownState::upright && fsrDbg)
      fsrDbgCounter++;
    else
      fsrDbgCounter = 0;

    if (fsrDbgCounter > fsrDbgCountThreshold)
    {
      if (fsrSum < 1.5f)
      {
        SystemCall::text2Speech("Please calibrate FSR sensors.");
        fsrDbg = false;
      }
      else if (fsrSum > theFsrSensorData.fsrRef * 0.8f)
        fsrDbg = false;
    }
  }
}

void SimpleFallDownStateDetector::resolveFallDownState(FallDownState& fallDownState)
{
  float maxOnGroundCounter = secondsOnGround / theFrameInfo.cycleTime;
  float maxStandUpCounter = secondsUntilStandUp / theFrameInfo.cycleTime;
  float maxLyingStillCounter = secondsUntilLyingStill / theFrameInfo.cycleTime;
  float minHeldOnShoulderCounter = secondsUntilHeldOnShoulder / theFrameInfo.cycleTime;
  FallDownState::Direction direction = fallDownState.direction;

  bool flying = false;
  bool heldOnOneShoulder = false;
  bool falling = false;
  bool onGround = false;
  bool onGroundLyingStill = false;
  bool standingUp = false;
  bool upright = false;
  bool diving = false;

  while (!stateQueue.empty())
  {
    FallDownState::State state = stateQueue.front();

    if (state == FallDownState::flying)
      flying = true;
    if (state == FallDownState::heldOnOneShoulder)
      heldOnOneShoulder = true;
    if (state == FallDownState::falling)
      falling = true;
    if (state == FallDownState::onGround)
      onGround = true;
    if (state == FallDownState::onGroundLyingStill)
      onGroundLyingStill = true;
    if (state == FallDownState::standingUp)
      standingUp = true;
    if (state == FallDownState::upright)
      upright = true;
    if (state == FallDownState::diving)
      diving = true;

    stateQueue.pop();
  }

  if constexpr (Build::targetSimulator())
  {
    if (enableForcedUprightDebugMode)
    {
      if (upright)
        fallDownState.state = FallDownState::upright;
      flying = false;
      falling = false;
      onGround = false;
      onGroundLyingStill = false;
      standingUp = false;
      diving = false;
    }
  }

  if (theBehaviorData.behaviorState == BehaviorData::testingJoints)
  {
    fallDownState.state = FallDownState::upright;
    lastState = fallDownState.state;
    return;
  }

  if (heldOnOneShoulder)
  {
    heldOnShoulderCounter++;
    if (heldOnShoulderCounter > minHeldOnShoulderCounter)
      flying = true;
  }
  else
  {
    heldOnShoulderCounter = 0;
  }

  if (flying)
  {
    fallDownState.state = FallDownState::flying;
  }
  else if (onGroundLyingStill)
  {
    lyingStillCounter++;
    if (lyingStillCounter > maxLyingStillCounter)
      fallDownState.state = FallDownState::onGroundLyingStill;
  }
  else if (falling && onGround)
  {
    onGroundCounter++;
    if (lastDirection != direction)
      onGroundCounter = 0;
    if (onGroundCounter > maxOnGroundCounter)
      fallDownState.state = FallDownState::onGround;
  }
  else if ((standingUp && onGround) || (standingUp && falling))
  {
    if (lastState == FallDownState::onGroundLyingStill || lastState == FallDownState::onGround || lastState == FallDownState::tryStandingUp)
      fallDownState.state = FallDownState::tryStandingUp;
    else if (falling)
      fallDownState.state = FallDownState::falling;
  }
  else if (standingUp)
  {
    standUpCounter++;
    if (standUpCounter > maxStandUpCounter)
    {
      fallDownState.state = FallDownState::standingUp;
    }

    if (lastState == FallDownState::standingUp && mightFlying())
    {
      fallDownState.state = FallDownState::flying;
    }
  }
  else if (diving)
  {
    fallDownState.state = FallDownState::diving;
  }
  else if (falling)
  {
    fallDownState.state = FallDownState::falling;

    if (theMotionInfo.motion == MotionInfo::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead)
      fallDownState.state = FallDownState::upright;
  }
  else if (upright)
  {
    fallDownState.state = FallDownState::upright;
  }
  else
  {
    fallDownState.state = FallDownState::undefined;
  }

  if (lastState != fallDownState.state)
  {
    onGroundCounter = 0;
    standUpCounter = 0;
    lyingStillCounter = 0;
  }
  else if (!onGroundLyingStill)
  {
    lyingStillCounter = 0;
  }

  lastState = fallDownState.state;
}

bool SimpleFallDownStateDetector::mightFlying()
{
  if (!enableFlyingDetection)
    return false;

  float accZCountThreshold = accZThreshold / theFrameInfo.cycleTime;
  if (notOnGround() && accZCounter > accZCountThreshold)
    return true;

  return false;
}

bool SimpleFallDownStateDetector::notOnGround()
{
  if (!enableFlyingDetection)
    return false;

  return (fsrSum - fsrMin) < fsrThreshold;
}

MAKE_MODULE(SimpleFallDownStateDetector, sensing)
