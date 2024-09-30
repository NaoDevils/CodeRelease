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
#include "Tools/Debugging/DebugDrawings.h"
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

  for (int i = 0; i < FsrSensorData::numOfFsrSensorPositions; i++)
  {
    leftFsrBufferArray[i].reserve(fsrBufferSize);
    rightFsrBufferArray[i].reserve(fsrBufferSize);
  }
}

void SimpleFallDownStateDetector::update(FallDownState& fallDownState)
{
  updateVariable(fallDownState);
  detectDisturbanceOfTheForce();
  detectSilenceOfTheForce();
  detectFlying(fallDownState);
  detectHeldOnOneShoulder(fallDownState);
  detectDiving(fallDownState);
  detectWideStance(fallDownState);
  detectFalling(fallDownState);
  detectOnGround(fallDownState);
  detectLyiningStill(fallDownState);
  detectDirection(fallDownState);
  detectStandingUp(fallDownState);
  detectUpright(fallDownState);
  resolveFallDownState(fallDownState);
}

void SimpleFallDownStateDetector::update(FsrModelData& fsrModelData)
{
  if (leftFsrBufferArray[0].capacity() != fsrBufferSize)
  {
    for (int i = 0; i < FsrSensorData::numOfFsrSensorPositions; i++)
    {
      leftFsrBufferArray[i].reserve(fsrBufferSize);
      rightFsrBufferArray[i].reserve(fsrBufferSize);
    }
  }

  unsigned int prevLeftFsrMinAttack = leftFsrMinAttack;
  unsigned int prevLeftFsrMaxAttack = leftFsrMaxAttack;
  unsigned int prevRightFsrMinAttack = rightFsrMinAttack;
  unsigned int prevRightFsrMaxAttack = rightFsrMaxAttack;
  for (int i = 0; i < FsrSensorData::numOfFsrSensorPositions; i++)
  {
    if (theFsrSensorData.left[i] != SensorData::off)
    {
      if (std::min(std::max(theFsrSensorData.left[i], 0.f), naoWeightinKg) < leftFsrMinValue && theFsrSensorData.left[i] > -naoWeightinKg * negativeSlope)
        leftFsrMinAttack++;
      if (std::min(std::max(theFsrSensorData.left[i], 0.f), naoWeightinKg) > leftFsrMaxValue && theFsrSensorData.left[i] < naoWeightinKg * positiveSlope)
        leftFsrMaxAttack++;

      if (leftFsrMinAttack >= fsrAttack)
      {
        leftFsrMinValue = std::min(std::max(*std::min_element(theFsrSensorData.left.begin(), theFsrSensorData.left.end()), 0.f), naoWeightinKg);
        leftFsrMinAttack = 0;
      }
      if (leftFsrMaxAttack >= fsrAttack)
      {
        leftFsrMaxValue = std::min(std::max(*std::max_element(theFsrSensorData.left.begin(), theFsrSensorData.left.end()), 0.f), naoWeightinKg);
        leftFsrMaxAttack = 0;
      }
    }
    if (theFsrSensorData.right[i] != SensorData::off)
    {
      if (std::min(std::max(theFsrSensorData.right[i], 0.f), naoWeightinKg) < rightFsrMinValue && theFsrSensorData.right[i] > -naoWeightinKg * negativeSlope)
        rightFsrMinAttack++;
      if (std::min(std::max(theFsrSensorData.right[i], 0.f), naoWeightinKg) > rightFsrMaxValue && theFsrSensorData.right[i] < naoWeightinKg * positiveSlope)
        rightFsrMaxAttack++;

      if (rightFsrMinAttack >= fsrAttack)
      {
        rightFsrMinValue = std::min(std::max(*std::min_element(theFsrSensorData.right.begin(), theFsrSensorData.right.end()), 0.f), naoWeightinKg);
        rightFsrMinAttack = 0;
      }
      if (rightFsrMaxAttack >= fsrAttack)
      {
        rightFsrMaxValue = std::min(std::max(*std::max_element(theFsrSensorData.right.begin(), theFsrSensorData.right.end()), 0.f), naoWeightinKg);
        rightFsrMaxAttack = 0;
      }
    }
  }
  if (prevLeftFsrMinAttack == leftFsrMinAttack)
    leftFsrMinAttack = 0;
  if (prevLeftFsrMaxAttack == leftFsrMaxAttack)
    leftFsrMaxAttack = 0;
  if (prevRightFsrMinAttack == rightFsrMinAttack)
    rightFsrMinAttack = 0;
  if (prevRightFsrMaxAttack == rightFsrMaxAttack)
    rightFsrMaxAttack = 0;

  calculateFsrModelData(fsrModelData);

  float fsrCorrectionValue = 1.f;
  while (fsrModelData.total * fsrCorrectionValue > naoWeightinKg)
    fsrCorrectionValue -= 0.01f;

  if (fsrCorrectionValue < 0.f)
    fsrCorrectionValue = 0.f;

  if (fsrCorrectionValue != 1.f)
    recalculateFsrModelData(fsrModelData, fsrCorrectionValue);

  smoothingFsrModelData(fsrModelData);

  if (fsrModelData.leftTotal < naoWeightinKg / 2 * fsrFlyingThreshold)
    fsrModelData.leftFootOnGround = false;
  else
    fsrModelData.leftFootOnGround = true;

  if (fsrModelData.rightTotal < naoWeightinKg / 2 * fsrFlyingThreshold)
    fsrModelData.rightFootOnGround = false;
  else
    fsrModelData.rightFootOnGround = true;


  float maxFsrReadyCounter = fsrModelSettleTime / theFrameInfo.cycleTime;
  if (!fsrModelReady)
  {
    if (fsrModelReadyCounter <= maxFsrReadyCounter && senseSilenceOfTheForce)
      fsrModelReadyCounter++;

    if (fsrModelReadyCounter > maxFsrReadyCounter)
      fsrModelReady = true;
  }

  fsrModelData.modelDataReady = fsrModelReady;
  localFsrModelData = fsrModelData;
}

void SimpleFallDownStateDetector::calculateFsrModelData(FsrModelData& fsrModelData, float fsrCorrectionValue)
{
  fsrModelData.leftTotal = 0.f;
  fsrModelData.rightTotal = 0.f;
  for (int i = 0; i < FsrSensorData::numOfFsrSensorPositions; i++)
  {
    if (leftFsrMaxValue != leftFsrMinValue && leftFsrMinValue != INFINITY && leftFsrMaxValue != -INFINITY)
    {
      fsrModelData.left[i] = ((theFsrSensorData.left[i] - leftFsrMinValue) / (leftFsrMaxValue - leftFsrMinValue)) * naoWeightinKg * fsrCorrectionValue;
      fsrModelData.leftTotal += fsrModelData.left[i];
    }
    else if (leftFsrMinValue == INFINITY && leftFsrMaxValue == -INFINITY)
    {
      fsrModelData.left[i] = naoWeightinKg / 8;
      fsrModelData.leftTotal += fsrModelData.left[i];
    }
    if (rightFsrMaxValue != rightFsrMinValue && rightFsrMinValue != INFINITY && rightFsrMaxValue != -INFINITY)
    {
      fsrModelData.right[i] = ((theFsrSensorData.right[i] - rightFsrMinValue) / (rightFsrMaxValue - rightFsrMinValue)) * naoWeightinKg * fsrCorrectionValue;
      fsrModelData.rightTotal += fsrModelData.right[i];
    }
    else if (rightFsrMinValue == INFINITY && rightFsrMaxValue == -INFINITY)
    {
      fsrModelData.right[i] = naoWeightinKg / 8;
      fsrModelData.rightTotal += fsrModelData.right[i];
    }
  }
  fsrModelData.total = fsrModelData.leftTotal + fsrModelData.rightTotal;
}

void SimpleFallDownStateDetector::smoothingFsrModelData(FsrModelData& fsrModelData)
{
  fsrModelData.leftTotal = 0.f;
  fsrModelData.rightTotal = 0.f;
  for (int i = 0; i < FsrSensorData::numOfFsrSensorPositions; i++)
  {
    if (fsrModelData.left[i] != SensorData::off)
    {
      leftFsrBufferArray[i].push_front(fsrModelData.left[i]);
      fsrModelData.left[i] = std::min(std::max(leftFsrBufferArray[i].average(), 0.f), naoWeightinKg);
      fsrModelData.leftTotal += fsrModelData.left[i];
    }
    if (fsrModelData.right[i] != SensorData::off)
    {
      rightFsrBufferArray[i].push_front(fsrModelData.right[i]);
      fsrModelData.right[i] = std::min(std::max(rightFsrBufferArray[i].average(), 0.f), naoWeightinKg);
      fsrModelData.rightTotal += fsrModelData.right[i];
    }
  }
  fsrModelData.total = fsrModelData.leftTotal + fsrModelData.rightTotal;
}

void SimpleFallDownStateDetector::updateVariable(FallDownState& fallDownState)
{
  angleXZ = theJoinedIMUData.imuData[anglesource].angle.y();
  angleYZ = theJoinedIMUData.imuData[anglesource].angle.x();

  gyroX = theJoinedIMUData.imuData[gyrosource].gyro.x().toDegrees();
  gyroY = theJoinedIMUData.imuData[gyrosource].gyro.y().toDegrees();

  mightUpright = (angleXZ <= fallDownAngleFront && angleXZ >= fallDownAngleBack && std::abs(angleYZ) <= fallDownAngleSide);
  notLying = (angleXZ <= fallDownAngleFront + standingUpTolerance && angleXZ >= fallDownAngleBack - standingUpTolerance && std::abs(angleYZ) <= fallDownAngleSide + standingUpTolerance);
  lastDirection = fallDownState.direction;

  fsrSumR = localFsrModelData.rightTotal;
  fsrSumL = localFsrModelData.leftTotal;
  if (fsrMinR > fsrSumR)
    fsrMinR = fsrSumR;
  if (fsrMinL > fsrSumL)
    fsrMinL = fsrSumL;

  if (fsrModelReady)
    checkFsrSum(fallDownState);

  fallDownState.standUpOnlyWhenLyingStill = standUpOnlyWhenLyingStill;
}

void SimpleFallDownStateDetector::detectFlying(FallDownState& fallDownState)
{
  bool endFlying = false;
  if (mightFlying() && lastState != FallDownState::tryStandingUp && lastState != FallDownState::standingUp && lastState != FallDownState::falling)
    stateQueue.push(FallDownState::flying);
  else if (fallDownState.state == FallDownState::flying && ((fsrSumR - fsrMinR) + (fsrSumL - fsrMinL) > naoWeightinKg * fsrOnGroundThreshold) && localFsrModelData.rightFootOnGround
      && localFsrModelData.leftFootOnGround)
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

void SimpleFallDownStateDetector::detectWideStance(FallDownState& fallDownState)
{
  if (theMotionInfo.inBlockMotion() && notLying)
    stateQueue.push(FallDownState::wideStance);
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

  if (angleXZ > fallDownAngleFront && angleYZ < -uprightAngleThreshold)
    fallDownState.tilt = FallDownState::frontLeft;
  else if (angleXZ > fallDownAngleFront && angleYZ > uprightAngleThreshold)
    fallDownState.tilt = FallDownState::frontRight;
  else if (angleXZ < fallDownAngleBack && angleYZ < -uprightAngleThreshold)
    fallDownState.tilt = FallDownState::backLeft;
  else if (angleXZ < fallDownAngleBack && angleYZ > uprightAngleThreshold)
    fallDownState.tilt = FallDownState::backRight;
  else if (lastState != FallDownState::upright && gyroY > directionGyroThreshold && gyroX < -directionGyroThreshold)
    fallDownState.tilt = FallDownState::frontLeft;
  else if (lastState != FallDownState::upright && gyroY > directionGyroThreshold && gyroX > directionGyroThreshold)
    fallDownState.tilt = FallDownState::frontRight;
  else if (lastState != FallDownState::upright && gyroY < -directionGyroThreshold && gyroX < -directionGyroThreshold)
    fallDownState.tilt = FallDownState::backLeft;
  else if (lastState != FallDownState::upright && gyroY < -directionGyroThreshold && gyroX > directionGyroThreshold)
    fallDownState.tilt = FallDownState::backRight;
  else
    fallDownState.tilt = FallDownState::notPresent;
}

void SimpleFallDownStateDetector::detectStandingUp(FallDownState& fallDownState)
{
  if (theMotionInfo.inStandUpMotion())
    stateQueue.push(FallDownState::standingUp);
}

void SimpleFallDownStateDetector::detectUpright(FallDownState& fallDownState)
{
  fallDownState.mightUpright = mightUpright;
  fallDownState.notLying = notLying;
  if (mightUpright && fallDownState.direction == FallDownState::none)
    stateQueue.push(FallDownState::upright);
}

void SimpleFallDownStateDetector::detectDisturbanceOfTheForce()
{
  if (senseDisturbanceOfTheForce)
    return;

  if (std::abs(gyroX) > forceDisturbanceThreshold || std::abs(gyroY) > forceDisturbanceThreshold)
    senseDisturbanceOfTheForce = true;
}

void SimpleFallDownStateDetector::detectSilenceOfTheForce()
{
  if (senseDisturbanceOfTheForce)
  {
    if (std::abs(gyroX) < forceSilenceThreshold && std::abs(gyroY) < forceSilenceThreshold)
      senseSilenceOfTheForce = true;
  }
}

void SimpleFallDownStateDetector::checkFsrSum(FallDownState& fallDownState)
{
  if constexpr (Build::targetRobot())
  {
    float fsrDbgCountThreshold = fsrDbgThreshold / theFrameInfo.cycleTime;

    if (theGameInfo.inPreGame() && fallDownState.state == FallDownState::upright && fsrDbg)
      fsrDbgCounter++;
    else
      fsrDbgCounter = 0;

    if (fsrDbgCounter > fsrDbgCountThreshold)
    {
      if (fsrSumR < (naoWeightinKg / 2 * fsrCheckThreshold) || fsrSumL < (naoWeightinKg / 2 * fsrCheckThreshold))
      {
        SystemCall::text2Speech("Please check FSR sensors for problems.");
        enableFlyingDetection = false;
        fsrBroken = true;
        fsrDbg = false;
      }
      else
        fsrDbg = false;
    }

    if (theRobotInfo.transitionToFramework == 0.f)
      fsrMessageActive = false;

    if (fsrBroken && theRobotInfo.transitionToFramework > 0.f && !fsrMessageActive)
    {
      SystemCall::text2Speech("Please check FSR sensors for problems.");
      fsrMessageActive = true;
    }
  }
}

void SimpleFallDownStateDetector::resolveFallDownState(FallDownState& fallDownState)
{
  float maxOnGroundCounter = secondsOnGround / theFrameInfo.cycleTime;
  float maxStandUpCounter = secondsUntilStandUp / theFrameInfo.cycleTime;
  float maxLyingStillCounter = secondsUntilLyingStill / theFrameInfo.cycleTime;
  float minHeldOnShoulderCounter = secondsUntilHeldOnShoulder / theFrameInfo.cycleTime;
  float minWideStanceFlyingCounter = secondsUntilWideStanceFlying / theFrameInfo.cycleTime;
  FallDownState::Direction direction = fallDownState.direction;

  bool flying = false;
  bool heldOnOneShoulder = false;
  bool falling = false;
  bool onGround = false;
  bool onGroundLyingStill = false;
  bool standingUp = false;
  bool upright = false;
  bool diving = false;
  bool wideStance = false;

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
    if (state == FallDownState::wideStance)
      wideStance = true;

    stateQueue.pop();
  }
  fallDownState.notOnGround = notOnGround();

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

  if (wideStance && flying)
  {
    flying = false;
    wideStanceFlyingCounter++;
    if (wideStanceFlyingCounter > minWideStanceFlyingCounter)
      flying = true;
  }
  else
  {
    wideStanceFlyingCounter = 0;
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

  if (!notOnGround() || lastState == FallDownState::onGround || lastState == FallDownState::onGroundLyingStill)
    lastOnGroundTimestamp = theFrameInfo.time;

  if (notOnGround() && mightUpright && (unsigned int)theFrameInfo.getTimeSince(lastOnGroundTimestamp) > minAirTime)
    return true;

  return false;
}

bool SimpleFallDownStateDetector::notOnGround()
{
  if (!enableFlyingDetection)
    return false;

  return (fsrSumR - fsrMinR) < naoWeightinKg / 2 * fsrFlyingThreshold && (fsrSumL - fsrMinL) < naoWeightinKg / 2 * fsrFlyingThreshold;
}

MAKE_MODULE(SimpleFallDownStateDetector, sensing)
