#include "MotionMindfulness.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Build.h"

MAKE_MODULE(MotionMindfulness, motionControl)

MotionMindfulness::MotionMindfulness()
{
  // Initialize fsr
  for (int i = 0; i < FsrSensorData::numOfFsrSensorPositions; i++)
    fsrLeft[i] = theFsrSensorData.left[i];
  for (int i = 0; i < FsrSensorData::numOfFsrSensorPositions; i++)
    fsrRight[i] = theFsrSensorData.right[i];

  // Initialize imu
  for (int i = 0; i < 3; i++)
  {
    acc[i] = theJoinedIMUData.imuData[anglesource].acc[i];
    gyro[i] = theJoinedIMUData.imuData[anglesource].gyro[i];
    if (i < 2)
      angle[i] = theJoinedIMUData.imuData[anglesource].angle[i];
  }

  // Initialize kick
  lastEngineKickTime = theFrameInfo.time - timeAfterEngineKick;
  lastCustomKickTime = theFrameInfo.time - timeAfterCustomKick;

  // Initialize frame rate
  frequency = static_cast<int>(1 / theFrameInfo.cycleTime);

  // Initialize heatState
  lastStateKneePitchLeft = 0;
  lastStateKneePitchRight = 0;
  lastStateAnklePitchLeft = 0;
  lastStateAnklePitchRight = 0;

  // Initialize walk frequency
  beginStepLeft = theFrameInfo.time;
  beginStepRight = theFrameInfo.time;
  stepTime.fill(0);

  // Initialize fall angle
  bufferAngleX.fill(0);
  bufferAngleY.fill(0);

  bufferSpeedForward.push_front(0.0f);
  bufferSpeedBackward.push_front(0.0f);
  bufferRequestSpeedForward.push_front(0.0f);
  bufferRequestSpeedBackward.push_front(0.0f);

  bufferSpeedLeft.push_front(0.0f);
  bufferSpeedRight.push_front(0.0f);
  bufferRequestSpeedLeft.push_front(0.0f);
  bufferRequestSpeedRight.push_front(0.0f);

  decraseIncreaseFactorForward = initialDecreaseIncreaseFactor;
  decraseIncreaseFactorBackward = initialDecreaseIncreaseFactor;
  decraseIncreaseFactorLeft = initialDecreaseIncreaseFactor;
  decraseIncreaseFactorRight = initialDecreaseIncreaseFactor;

  localMotionState.walkingStatus.decreaseIncreaseFactors[0] = initialDecreaseIncreaseFactor;
  localMotionState.walkingStatus.decreaseIncreaseFactors[1] = initialDecreaseIncreaseFactor;
  localMotionState.walkingStatus.decreaseIncreaseFactors[2] = initialDecreaseIncreaseFactor;
  localMotionState.walkingStatus.decreaseIncreaseFactors[3] = initialDecreaseIncreaseFactor;

  for (int i = 0; i < 4; i++)
    lastUpdatedWalkingError[i] = theFrameInfo.time;

  if constexpr (Build::targetRobot())
  {
    InTextFile motionMindfulnessDecreaseIncreaseFactorsFile(File::getPersistentDir() + "decreaseIncreaseFactors.value");
    WalkingStatus walkingStatus;
    if (motionMindfulnessDecreaseIncreaseFactorsFile.exists())
    {
      motionMindfulnessDecreaseIncreaseFactorsFile >> walkingStatus;
      decraseIncreaseFactorForward = walkingStatus.decreaseIncreaseFactors[0];
      decraseIncreaseFactorBackward = walkingStatus.decreaseIncreaseFactors[1];
      decraseIncreaseFactorLeft = walkingStatus.decreaseIncreaseFactors[2];
      decraseIncreaseFactorRight = walkingStatus.decreaseIncreaseFactors[3];
      ANNOTATION("MotionState", "Loaded " << File::getPersistentDir() << "decreaseIncreaseFactors.value");
    }
  }

  InMapFile stream("csConverter2019.cfg");
  if (stream.exists())
  {
    stream >> csConverterParams;
  }
}

MotionMindfulness::~MotionMindfulness()
{
  if constexpr (Build::targetRobot())
  {
    float minFactor = 0.75;
    SpeedLimits limits = theWalkingEngineParams.speedLimits;
    if (backwardUpdated || maxSpeedRequestBackward > minFactor * limits.xBackward * limits.speedFactor)
    {
      limits.xBackward *= localMotionState.walkingStatus.speedFactorBackward * limits.speedFactor;
    }
    else
    {
      limits.xBackward *= limits.speedFactor;
    }

    if (forwardUpdated || maxSpeedRequestForward > minFactor * limits.xForward * limits.speedFactor)
    {
      limits.xForward *= localMotionState.walkingStatus.speedFactorForward * limits.speedFactor;
      limits.xForwardArmContact *= localMotionState.walkingStatus.speedFactorForward * limits.speedFactor;
      limits.xForwardOmni *= localMotionState.walkingStatus.speedFactorForward * limits.speedFactor;
    }
    else
    {
      limits.xBackward *= limits.speedFactor;
    }

    if ((leftUpdated || lastMaxRequestLeft > minFactor * limits.y * limits.speedFactor) || (rightUpdated || lastMaxRequestRight > minFactor * limits.y * limits.speedFactor))
    {
      limits.y *= std::max<float>(localMotionState.walkingStatus.speedFactorLeft, localMotionState.walkingStatus.speedFactorRight) * limits.speedFactor;
      limits.yArmContact *= std::max<float>(localMotionState.walkingStatus.speedFactorLeft, localMotionState.walkingStatus.speedFactorRight) * limits.speedFactor;
    }
    else
    {
      limits.xBackward *= limits.speedFactor;
    }

    limits.speedFactor = 1.f;

    OutTextFile motionMindfulnessSpeedLimitsFile(File::getPersistentDir() + "speedLimits.value");
    if (motionMindfulnessSpeedLimitsFile.exists())
      motionMindfulnessSpeedLimitsFile << limits;

    const WalkingStatus& walkingStatus = localMotionState.walkingStatus;
    OutTextFile motionMindfulnessDecreaseIncreaseFactorsFile(File::getPersistentDir() + "decreaseIncreaseFactors.value");
    if (motionMindfulnessDecreaseIncreaseFactorsFile.exists())
      motionMindfulnessDecreaseIncreaseFactorsFile << walkingStatus;
  }
}

void MotionMindfulness::update(MotionState& theMotionState)
{
  DECLARE_PLOT("representation:MotionState:stepTime");
  DECLARE_PLOT("module:MotionMindfulness:walkingForwardBackward");
  DECLARE_PLOT("module:MotionMindfulness:walkingSidewards");
  DECLARE_PLOT("module:MotionMindfulness:walkingErrors:forward");
  DECLARE_PLOT("module:MotionMindfulness:walkingErrors:backward");
  DECLARE_PLOT("module:MotionMindfulness:walkingErrors:left");
  DECLARE_PLOT("module:MotionMindfulness:walkingErrors:right");
  DECLARE_PLOT("module:MotionMindfulness:walkingStatus:stumble");
  DECLARE_PLOT("module:MotionMindfulness:walkingStatus:stumblingForward");
  DECLARE_PLOT("module:MotionMindfulness:walkingStatus:stumblingBackward");
  DECLARE_PLOT("module:MotionMindfulness:walkingStatus:stumblingLeft");
  DECLARE_PLOT("module:MotionMindfulness:walkingStatus:stumblingRight");
  if (activate)
  {
    // Check for problems and sanity
    checkForStiffnessProblems(localMotionState);
    localMotionState.jointStatus.usableLeftLeg = !(hasProblem(localMotionState, MotionState::legLeftStiffness));
    localMotionState.jointStatus.usableRightLeg = !(hasProblem(localMotionState, MotionState::legRightStiffness));
    checkForStandUpProblems(localMotionState);
    checkForFsrSanity(localMotionState);
    // Set the usability of the fsr sensors to false if one of the sensors is not working correctly
    localMotionState.fsrStatus.usableLeft = !std::any_of(localMotionState.fsrStatus.sensorStatusLeft.cbegin(),
        localMotionState.fsrStatus.sensorStatusLeft.cend(),
        [](bool i)
        {
          return !i;
        });
    localMotionState.fsrStatus.usableRight = !std::any_of(localMotionState.fsrStatus.sensorStatusRight.cbegin(),
        localMotionState.fsrStatus.sensorStatusRight.cend(),
        [](bool i)
        {
          return !i;
        });
    checkForImuSanity(localMotionState);
    checkFramerate(localMotionState);
    checkForWalkProblems(localMotionState);
    checkForKickProblems(localMotionState);
    checkForHeatProblems(localMotionState);

    // Detect and predict motion related values
    stepTimeDetection(localMotionState);
    fallAngleDetection(localMotionState);

    kinematicStumbleDetection(localMotionState);
    angleVarianceStumbleDetection(localMotionState);
    fsrVarianceStumbleDetection(localMotionState);
    stumbleDetectionAccumulator(localMotionState);

    walkSpeedProtocol(localMotionState);
    walkSpeedFactorPrediction(localMotionState);
    theMotionState = localMotionState;

    // [0]->walking left errors
    // [1]->walking right errors
    // [2]->walking forward errors
    // [3]->walking backward errors
    PLOT("module:MotionMindfulness:walkingErrors:forward", localMotionState.walkingStatus.walkingErrors[2]);
    PLOT("module:MotionMindfulness:walkingErrors:backward", localMotionState.walkingStatus.walkingErrors[3]);
    PLOT("module:MotionMindfulness:walkingErrors:left", localMotionState.walkingStatus.walkingErrors[0]);
    PLOT("module:MotionMindfulness:walkingErrors:right", localMotionState.walkingStatus.walkingErrors[1]);
  }
}

bool MotionMindfulness::hasProblem(MotionState& motionState, MotionState::MotionStateError motionStateError)
{
  for (auto& problem : motionState.motionProblems)
    if (problem == motionStateError)
      return true;
  return false;
}

void MotionMindfulness::checkForKickProblems(MotionState& motionState)
{
  /***********************************/
  /* kick engine check               */
  /***********************************/
  bool kickActive = true;
  if (theMotionRequest.motion == MotionRequest::kick)
  {
    lastEngineKickTime = theFrameInfo.time;
  }
  else
  {
    kickActive = false;
  }

  unsigned currentTime = theFrameInfo.time;

  if (!kickActive) // The kick is done
  {
    /*
    * If the time difference between the current time and the time of the last frame of the kick
    * is less or equal than a defined period of time and the robot is fallen the engineKickFailure kicks in
    */
    if (currentTime - lastEngineKickTime <= timeAfterEngineKick && theFallDownState.state == FallDownState::onGround)
      if (!hasProblem(motionState, MotionState::engineKickFailure))
        motionState.motionProblems.push_back(MotionState::engineKickFailure);
  }
  else
  {
    /***********************************/
    /* reset values                    */
    /***********************************/
    auto it = std::find(motionState.motionProblems.begin(), motionState.motionProblems.end(), MotionState::engineKickFailure);
    if (it != motionState.motionProblems.end())
      motionState.motionProblems.erase(it);
  }

  /***********************************/
  /* custom step kick check          */
  /***********************************/
  kickActive = true;
  if (theMotionInfo.walkKicking)
  {
    lastCustomKickTime = theFrameInfo.time;
  }
  else
  {
    kickActive = false;
  }

  currentTime = theFrameInfo.time;

  if (!kickActive) // the kick is done
  {
    /*
    * If the time difference between the current time and the time of the last frame of the kick
    * is less or equal than a defined period of time and the robot is fallen the customKickFailure kicks in
    */
    if (currentTime - lastCustomKickTime <= timeAfterCustomKick && theFallDownState.state == FallDownState::onGround)
      if (!hasProblem(motionState, MotionState::customKickFailure))
        motionState.motionProblems.push_back(MotionState::customKickFailure);
  }
  else
  {
    /***********************************/
    /* reset values                    */
    /***********************************/
    auto it = std::find(motionState.motionProblems.begin(), motionState.motionProblems.end(), MotionState::customKickFailure);
    if (it != motionState.motionProblems.end())
      motionState.motionProblems.erase(it);
  }
}

void MotionMindfulness::checkForWalkProblems(MotionState& motionState)
{
  /***********************************/
  /* do nothing if we not walking    */
  /***********************************/
  if (theMotionRequest.motion != MotionRequest::walk)
  {
    // Update errors in MotionState (init)
    for (int i = 0; i < 4; i++)
      motionState.walkingStatus.walkingErrors[i] = errorsWalking[i];
    return;
  }

  if (startTimestamp == 0)
    startTimestamp = theFrameInfo.time;

  Angle angleX = theJoinedIMUData.imuData[anglesource].angle.x();
  Angle angleY = theJoinedIMUData.imuData[anglesource].angle.y();

  motionState.walkingStatus.fallDownSpeedReductionFactor.x() = 1.f;
  motionState.walkingStatus.fallDownSpeedReductionFactor.y() = 1.f;

  /***********************************/
  /* walking left check              */
  /***********************************/
  if (angleX > (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[1] + csConverterParams.legJointBalanceParams.targetAngleX))
  {
    motionState.walkingStatus.fallDownSpeedReductionFactor.y() = fallDownFactor
        * std::pow(fallDownExponentialBasis, std::abs((angleX - theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[1]) / theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[1]));
    updateWalkErrors(motionState, true);
  }
  else
  {
    updateWalkErrors(motionState, false);
  }

  /***********************************/
  /* walking right check             */
  /***********************************/
  if (angleX < (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[0] + csConverterParams.legJointBalanceParams.targetAngleX))
  {
    motionState.walkingStatus.fallDownSpeedReductionFactor.y() = fallDownFactor
        * std::pow(fallDownExponentialBasis, std::abs((angleX - theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[0]) / theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[0]));

    updateWalkErrors(motionState, true);
  }
  else
  {
    updateWalkErrors(motionState, false);
  }

  /***********************************/
  /* walking forward check           */
  /***********************************/
  if (angleY < (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0] + csConverterParams.legJointBalanceParams.targetAngleY))
  {
    motionState.walkingStatus.fallDownSpeedReductionFactor.x() = fallDownFactor
        * std::pow(fallDownExponentialBasis, std::abs((angleY - theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0]) / theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0]));

    updateWalkErrors(motionState, true);
  }
  else
  {
    updateWalkErrors(motionState, false);
  }

  /***********************************/
  /* walking backward check          */
  /***********************************/
  if (angleY > (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[1] + csConverterParams.legJointBalanceParams.targetAngleY))
  {
    motionState.walkingStatus.fallDownSpeedReductionFactor.x() = fallDownFactor
        * std::pow(fallDownExponentialBasis, std::abs((angleY - theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[1]) / theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[1]));

    updateWalkErrors(motionState, true);
  }
  else
  {
    updateWalkErrors(motionState, false);
  }

  /*
  * If the frame counter reaches a minimum value the walking failure kicks in and the frame counter is set to the max value.
  * Therefore the robot has to run at minimum (maxAllowedUnstableFrames - minUnstableFrames) before the failure is reset.
  */
  if (errorsWalking[0] >= minUnstableFrames)
  {
    if (!hasProblem(motionState, MotionState::walkingLeftFailure))
    {
      motionState.motionProblems.push_back(MotionState::walkingLeftFailure);
      errorsWalking[0] = maxAllowedUnstableFrames;
    }
  }
  else
  {
    /***********************************/
    /* reset values                    */
    /***********************************/
    auto it = std::find(motionState.motionProblems.begin(), motionState.motionProblems.end(), MotionState::walkingLeftFailure);
    if (it != motionState.motionProblems.end())
      motionState.motionProblems.erase(it);
  }

  if (errorsWalking[1] >= minUnstableFrames)
  {
    if (!hasProblem(motionState, MotionState::walkingRightFailure))
    {
      motionState.motionProblems.push_back(MotionState::walkingRightFailure);
      errorsWalking[1] = maxAllowedUnstableFrames;
    }
  }
  else
  {
    /***********************************/
    /* reset values                    */
    /***********************************/
    auto it = std::find(motionState.motionProblems.begin(), motionState.motionProblems.end(), MotionState::walkingRightFailure);
    if (it != motionState.motionProblems.end())
      motionState.motionProblems.erase(it);
  }

  if (errorsWalking[2] >= minUnstableFrames)
  {
    if (!hasProblem(motionState, MotionState::walkingForwardFailure))
    {
      motionState.motionProblems.push_back(MotionState::walkingForwardFailure);
      errorsWalking[2] = maxAllowedUnstableFrames;
    }
  }
  else
  {
    /***********************************/
    /* reset values                    */
    /***********************************/
    auto it = std::find(motionState.motionProblems.begin(), motionState.motionProblems.end(), MotionState::walkingForwardFailure);
    if (it != motionState.motionProblems.end())
      motionState.motionProblems.erase(it);
  }

  if (errorsWalking[3] >= minUnstableFrames)
  {
    if (!hasProblem(motionState, MotionState::walkingBackwardFailure))
    {
      motionState.motionProblems.push_back(MotionState::walkingBackwardFailure);
      errorsWalking[3] = maxAllowedUnstableFrames;
    }
  }
  else
  {
    /***********************************/
    /* reset values                    */
    /***********************************/
    auto it = std::find(motionState.motionProblems.begin(), motionState.motionProblems.end(), MotionState::walkingBackwardFailure);
    if (it != motionState.motionProblems.end())
      motionState.motionProblems.erase(it);
  }

  /*
  * Update errors in MotionState
  * [0] -> walking left errors
  * [1] -> walking right errors
  * [2] -> walking forward errors
  * [3] -> walking backward errors
  */
  for (int i = 0; i < 4; i++)
    motionState.walkingStatus.walkingErrors[i] = errorsWalking[i];
}

void MotionMindfulness::checkFramerate(MotionState& motionState)
{
  /***********************************/
  /* reset values                    */
  /***********************************/
  for (int problem = MotionState::motionFrameRateAbnormal; problem <= MotionState::cognitionFrameRateAbnormal; problem++)
  {
    auto it = std::find(motionState.motionProblems.begin(), motionState.motionProblems.end(), problem);
    if (it != motionState.motionProblems.end())
      motionState.motionProblems.erase(it);
  }

  /***********************************/
  /* frame rate check                */
  /***********************************/
  if (theRobotHealth.motionFrameRate + 1 < frequency)
    motionFrameError++;
  else
    motionFrameError = 0;

  if (theRobotHealth.cognitionFrameRate + 1 < 30)
    cognitionFrameError++;
  else
    cognitionFrameError = 0;

  // If more than 5 frames are slower than the target frequency a failure is thrown
  if (motionFrameError > 5 && !hasProblem(motionState, MotionState::motionFrameRateAbnormal))
    motionState.motionProblems.push_back(MotionState::motionFrameRateAbnormal);

  // If more than 5 frames are slower than the target frequency a failure is thrown
  if (cognitionFrameError > 5 && !hasProblem(motionState, MotionState::cognitionFrameRateAbnormal))
    motionState.motionProblems.push_back(MotionState::cognitionFrameRateAbnormal);

  /***********************************/
  /* update status                   */
  /***********************************/
  motionState.frameRateStatus.motionFrameRate = theRobotHealth.motionFrameRate;
  motionState.frameRateStatus.cognitionFrameRate = theRobotHealth.cognitionFrameRate;
}

void MotionMindfulness::checkForImuSanity(MotionState& motionState)
{
  /***********************************/
  /* reset imu sensorStatus          */
  /***********************************/
  motionState.imuStatus.accStatus.fill(true);
  motionState.imuStatus.gyroStatus.fill(true);
  motionState.imuStatus.angleStatus.fill(true);

  /***********************************/
  /* imu sanity check                */
  /***********************************/
  for (int i = 0; i < 3; i++)
  {
    if (acc[i] == (theJoinedIMUData.imuData[anglesource].acc[i]))
    {
      if (errorsAcc[i] <= maxImuErrors)
        errorsAcc[i]++;
    }
    else
    {
      errorsAcc[i] = 0;
    }
    if (gyro[i] == (theJoinedIMUData.imuData[anglesource].gyro[i]))
    {
      if (errorsGyro[i] <= maxImuErrors)
        errorsGyro[i]++;
    }
    else
    {
      errorsGyro[i] = 0;
    }
    if (i < 2)
    {
      if (angle[i] == (theJoinedIMUData.imuData[anglesource].angle[i]))
      {
        if (errorsAngle[i] <= maxImuErrors)
          errorsAngle[i]++;
      }
      else
      {
        errorsAngle[i] = 0;
      }
    }

    /*
    * If the values are equal for max..Errors than the status of this sensor is set to false
    * therefore it is enougth that only one value is different to reset the status
    */
    if (errorsAcc[i] > maxImuErrors)
      motionState.imuStatus.accStatus[i] = false;
    if (errorsGyro[i] > maxImuErrors)
      motionState.imuStatus.gyroStatus[i] = false;
    if (i < 2)
      if (errorsAngle[i] > maxImuErrors)
        motionState.imuStatus.angleStatus[i] = false;

    /***********************************/
    /* update last imu values          */
    /***********************************/
    acc[i] = theJoinedIMUData.imuData[anglesource].acc[i];
    gyro[i] = theJoinedIMUData.imuData[anglesource].gyro[i];
    if (i < 2)
      angle[i] = theJoinedIMUData.imuData[anglesource].angle[i];
  }
}

void MotionMindfulness::checkForFsrSanity(MotionState& motionState)
{
  /***********************************/
  /* do nothing if we are walking    */
  /***********************************/
  if (theMotionRequest.motion == MotionRequest::walk || theFallDownState.state != FallDownState::upright)
    return;

  /***********************************/
  /* reset value                     */
  /***********************************/
  motionState.fsrStatus.sensorStatusLeft.fill(true);
  motionState.fsrStatus.sensorStatusRight.fill(true);


  /***********************************/
  /* fsr sanity check                */
  /***********************************/
  for (int i = 0; i < FsrSensorData::numOfFsrSensorPositions; i++)
  {
    /* 
    * If the value for each fsr is the same as the one from the frame before or 
    * the value jumps for more than 500 a frame counter is counted up until it reaches a max value
    * otherwise the frame counter is set to zero
    */
    if (fsrLeft[i] == theFsrSensorData.left[i] || std::abs(fsrLeft[i] - theFsrSensorData.left[i]) > 0.35f)
    {
      if (errorsFsrLeft[i] <= maxFsrErrors)
        errorsFsrLeft[i]++;
    }
    else
    {
      errorsFsrLeft[i] = 0;
    }

    if (fsrRight[i] == theFsrSensorData.right[i] || std::abs(fsrRight[i] - theFsrSensorData.right[i]) > 0.35f)
    {
      if (errorsFsrRight[i] <= maxFsrErrors)
        errorsFsrRight[i]++;
    }
    else
    {
      errorsFsrRight[i] = 0;
    }

    // If the frame counter is greater than a max value the fsrStatus set to false
    if (errorsFsrLeft[i] > maxFsrErrors)
      motionState.fsrStatus.sensorStatusLeft[i] = false;
    if (errorsFsrRight[i] > maxFsrErrors)
      motionState.fsrStatus.sensorStatusRight[i] = false;

    /***********************************/
    /* update last fsr values          */
    /***********************************/
    fsrLeft[i] = theFsrSensorData.left[i];
    fsrRight[i] = theFsrSensorData.right[i];
  }
}

void MotionMindfulness::checkForStandUpProblems(MotionState& motionState)
{
  // Filter all non stand up special actions
  if (theMotionInfo.inStandUpMotion())
  {
    /***********************************/
    /* reset standUpTries              */
    /***********************************/
    unsigned currentTime = theFrameInfo.time;

    if (lastStandUpTime == -1)
    {
      lastStandUpTime = currentTime;
      lastResetTime = currentTime;
    }

    if (theFallDownState.state == FallDownState::upright)
    {
      // Set standUpTries to zero to reset if the robot somehow managed to stand up
      if (!hasProblem(motionState, MotionState::standUpFailure))
      {
        standUpTries[motionId] = 0;
        standUp[motionId] = false;
      }

      // Set standUpTries to zero to reset the failure after a period of time
      if (resetStandUpFailureOnUpright && currentTime - lastResetTime > resetStandUpFailureTime)
      {
        if (hasProblem(motionState, MotionState::standUpFailure))
        {
          standUpTries[motionId] = 0;
          standUp[motionId] = false;
        }
        lastResetTime = currentTime;
      }

      unableToStandUp = false;
      lastStandUpTime = currentTime;
    }

    /***********************************/
    /* reset failure                   */
    /***********************************/
    auto it = std::find(motionState.motionProblems.begin(), motionState.motionProblems.end(), MotionState::standUpFailure);
    if (it != motionState.motionProblems.end())
      motionState.motionProblems.erase(it);

    /***********************************/
    /* stand up check                  */
    /***********************************/
    if (theFallDownState.state == FallDownState::standingUp)
    {
      motionId = theMotionRequest.specialActionRequest.specialAction;
      onGround = false;
    }

    if (!onGround && theFallDownState.state == FallDownState::onGround)
    {
      if (currentTime - lastStandUpTime < maxPeriodOfTimeBetweenStandUps)
      {
        if (standUpTries[motionId] < maxStandUpTries)
          standUpTries[motionId]++;
      }
      else
      {
        standUpTries[motionId] = 0;
      }
      motionState.standUpStatus.totalOnGround++;
      lastStandUpTime = currentTime;
      onGround = true;
    }

    /*
    * if the robot laying on the ground for more than maxPeriodOfTimeToStandUp milliseconds or
    * if the robot has tired maxStandUpTries to getting up without success the standUpFrontFailure/standUpBackFailure kicks in
    */
    if (currentTime - lastStandUpTime > maxPeriodOfTimeToStandUp && !hasProblem(motionState, MotionState::standUpFailure))
    {
      motionState.motionProblems.push_back(MotionState::standUpFailure);
      unableToStandUp = true;
    }
    if (standUpTries[motionId] >= maxStandUpTries)
    {
      motionState.motionProblems.push_back(MotionState::standUpFailure);
      standUp[motionId] = true;
    }
  }

  /*
  * Update MotionState
  */
  motionState.standUpStatus.standUpTries[motionId] = standUpTries[motionId];
  motionState.standUpStatus.currentId = motionId;
  motionState.standUpStatus.standUp[motionId] = standUp[motionId];
  motionState.standUpStatus.unableToStandUp = unableToStandUp;
}

void MotionMindfulness::checkForStiffnessProblems(MotionState& motionState)
{
  /*
  * Return if the robot is sitting down or if the robot is playing dead
  * to avoid some complications with these states
  */
  if (theMotionRequest.motion == MotionRequest::specialAction
      && (theMotionRequest.specialActionRequest.specialAction == theMotionRequest.specialActionRequest.playDead
          || theMotionRequest.specialActionRequest.specialAction == theMotionRequest.specialActionRequest.sitDown))
    return;

  /***********************************/
  /* leg stiffness check             */
  /***********************************/
  for (int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
  {
    /*
    * We need to check if the senor angle is greater than the target angle
    * because otherwise we run into possible false negatives for the cases like
    * the sensor angle is -5 and the target angle is 5
    */
    if (theJointSensorData.angles[i] > theJointRequest.angles[i])
    {
      /*
      * If the target angle plus an epsilon neighbourhood is less than the actual angle and the target stiffness is greater than zero 
      * and the current Ampere is zero a frame counter is counted up otherwise the frame counter is set to zero.
      * This allowes a minimal error for all leg joints for a short period of time which sometimes occur but only 50 frames per default.
      * For more than 50 frames in a row the leg stiffness failure is thrown.
      * This means if a V6 robot stays for more than 600 milliseconds in this state the failure is thrown.
      */
      if (theJointRequest.angles[i] + minJointDiff < theJointSensorData.angles[i] && theJointRequest.stiffnessData.stiffnesses[i] > 0 && theJointSensorData.currents[i] == 0)
        framesWithoutJointCurrent[i]++;
      else
        framesWithoutJointCurrent[i] = 0;
    }
    else
    {
      // The same as above but for the other direction
      if (theJointRequest.angles[i] - minJointDiff > theJointSensorData.angles[i] && theJointRequest.stiffnessData.stiffnesses[i] > 0 && theJointSensorData.currents[i] == 0)
        framesWithoutJointCurrent[i]++;
      else
        framesWithoutJointCurrent[i] = 0;
    }

    if (framesWithoutJointCurrent[i] > maxFramesWithoutJointActivation)
    {
      MotionState::MotionStateError problem = (i < Joints::rHipYawPitch) ? MotionState::legLeftStiffness : MotionState::legRightStiffness;
      if (!hasProblem(motionState, problem))
        motionState.motionProblems.push_back(problem);
    }
  }
}

void MotionMindfulness::checkForHeatProblems(MotionState& motionState)
{
  /***********************************/
  /* reset failure                   */
  /***********************************/
  for (int problem = MotionState::kneePitchLeftTooHot; problem <= MotionState::anklePitchRightTooHot; problem++)
  {
    auto it = std::find(motionState.motionProblems.begin(), motionState.motionProblems.end(), problem);
    if (it != motionState.motionProblems.end())
      motionState.motionProblems.erase(it);
  }

  motionState.heatStatus.kneePitchLeftStat = theJointSensorData.status[Joints::lKneePitch];
  motionState.heatStatus.kneePitchRightStat = theJointSensorData.status[Joints::rKneePitch];
  motionState.heatStatus.anklePitchLeftStat = theJointSensorData.status[Joints::lAnklePitch];
  motionState.heatStatus.anklePitchRightStat = theJointSensorData.status[Joints::rAnklePitch];

  if (motionState.heatStatus.kneePitchLeftStat >= 2)
  {
    if (!hasProblem(motionState, MotionState::kneePitchLeftTooHot))
      motionState.motionProblems.push_back(MotionState::kneePitchLeftTooHot);
  }

  if (motionState.heatStatus.kneePitchRightStat >= 2)
  {
    if (!hasProblem(motionState, MotionState::kneePitchRightTooHot))
      motionState.motionProblems.push_back(MotionState::kneePitchRightTooHot);
  }

  if (motionState.heatStatus.anklePitchLeftStat >= 2)
  {
    if (!hasProblem(motionState, MotionState::anklePitchLeftTooHot))
      motionState.motionProblems.push_back(MotionState::anklePitchLeftTooHot);
  }

  if (motionState.heatStatus.anklePitchRightStat >= 2)
  {
    if (!hasProblem(motionState, MotionState::anklePitchRightTooHot))
      motionState.motionProblems.push_back(MotionState::anklePitchRightTooHot);
  }

  // Like the diagnosis error of softbank don't reset the error after reaching a status of zero
  if (lastStateKneePitchLeft >= 2 && motionState.heatStatus.kneePitchLeftStat == 1)
  {
    if (!hasProblem(motionState, MotionState::kneePitchLeftTooHot))
      motionState.motionProblems.push_back(MotionState::kneePitchLeftTooHot);
  }

  if (lastStateKneePitchRight >= 2 && motionState.heatStatus.kneePitchRightStat == 1)
  {
    if (!hasProblem(motionState, MotionState::kneePitchRightTooHot))
      motionState.motionProblems.push_back(MotionState::kneePitchRightTooHot);
  }

  if (lastStateAnklePitchLeft >= 2 && motionState.heatStatus.anklePitchLeftStat == 1)
  {
    if (!hasProblem(motionState, MotionState::anklePitchLeftTooHot))
      motionState.motionProblems.push_back(MotionState::anklePitchLeftTooHot);
  }

  if (lastStateAnklePitchRight >= 2 && motionState.heatStatus.anklePitchRightStat == 1)
  {
    if (!hasProblem(motionState, MotionState::anklePitchRightTooHot))
      motionState.motionProblems.push_back(MotionState::anklePitchRightTooHot);
  }

  lastStateKneePitchLeft = motionState.heatStatus.kneePitchLeftStat;
  lastStateKneePitchRight = motionState.heatStatus.kneePitchRightStat;
  lastStateAnklePitchLeft = motionState.heatStatus.anklePitchLeftStat;
  lastStateAnklePitchRight = motionState.heatStatus.anklePitchRightStat;
}

void MotionMindfulness::stepTimeDetection(MotionState& motionState)
{
  float currentFsrAverageLeft = theFsrSensorData.leftTotal / static_cast<float>(theFsrSensorData.numOfFsrSensorPositions);
  float currentFsrAverageRight = theFsrSensorData.rightTotal / static_cast<float>(theFsrSensorData.numOfFsrSensorPositions);

  if (currentFsrAverageLeft < minFsrLeft)
    minFsrLeft = currentFsrAverageLeft;
  if (currentFsrAverageRight < minFsrRight)
    minFsrRight = currentFsrAverageRight;

  if (currentFsrAverageLeft < minFsrLeft + 0.15f)
  {
    if (!lastFsrLeft)
      beginStepLeft = theFrameInfo.time;
    lastFsrLeft = true;
  }
  else
  {
    if (lastFsrLeft)
    {
      endStepLeft = theFrameInfo.time;
      stepTime.push_front(std::abs(beginStepLeft - endStepLeft));
    }
    lastFsrLeft = false;
  }

  if (currentFsrAverageRight < minFsrRight + 0.15f)
  {
    if (!lastFsrRight)
      beginStepRight = theFrameInfo.time;
    lastFsrRight = true;
  }
  else
  {
    if (lastFsrRight)
    {
      endStepRight = theFrameInfo.time;
      stepTime.push_front(std::abs(beginStepRight - endStepRight));
    }
    lastFsrRight = false;
  }

  motionState.walkingStatus.averageStepTime = stepTime.average();
  PLOT("representation:MotionState:stepTime", stepTime.front());
}

void MotionMindfulness::fallAngleDetection(MotionState& motionState)
{
  float fsrFeedback = theFsrSensorData.leftTotal + theFsrSensorData.rightTotal;

  if (theFallDownState.state == FallDownState::onGround)
  {
    bufferAngleX.fill(0);
    bufferAngleY.fill(0);
    maxSpeedForward = 0.f;
    maxSpeedBackward = 0.f;
    maxSpeedLeft = 0.f;
    maxSpeedRight = 0.f;
    bufferSpeedForward.clear();
    bufferSpeedBackward.clear();
    bufferSpeedLeft.clear();
    bufferSpeedRight.clear();
  }

  if (theFallDownState.state == FallDownState::upright && theMotionRequest.motion == MotionRequest::walk && fsrFeedback >= minWeight)
  {
    bufferAngleX.push_front(theJoinedIMUData.imuData[anglesource].angle.x());
    bufferAngleY.push_front(theJoinedIMUData.imuData[anglesource].angle.y());

    // Save the max angle of walking fluctuation without falling for the back
    if (minAngleX > bufferAngleX.back())
      minAngleX = bufferAngleX.back();
    // The front
    if (maxAngleX < bufferAngleX.back())
      maxAngleX = bufferAngleX.back();

    // The left side
    if (minAngleY > bufferAngleY.back())
      minAngleY = bufferAngleY.back();
    // The right side
    if (maxAngleY < bufferAngleY.back())
      maxAngleY = bufferAngleY.back();
  }


  motionState.walkingStatus.predFallDownAngleFront = maxAngleX;
  motionState.walkingStatus.predFallDownAngleBack = minAngleX;
  motionState.walkingStatus.predFallDownAngleLeft = minAngleY;
  motionState.walkingStatus.predFallDownAngleRight = maxAngleY;
}

void MotionMindfulness::walkSpeedProtocol(MotionState& motionState)
{
  if (theFallDownState.state == FallDownState::upright && theMotionRequest.motion == MotionRequest::walk)
  {
    float speedX = theSpeedInfo.speed.translation[0];
    float requestedSpeedX = theSpeedRequest.translation.x();

    lastMaxForward = maxSpeedForward;
    lastMaxBackward = maxSpeedBackward;
    lastMaxRequestForward = maxSpeedRequestForward;
    lastMaxRequestBackward = maxSpeedRequestBackward;

    lastMaxLeft = maxSpeedLeft;
    lastMaxRight = maxSpeedRight;
    lastMaxRequestLeft = maxSpeedRequestLeft;
    lastMaxRequestRight = maxSpeedRequestRight;

    if (speedX > minSpeedForDetection / 1000)
    {
      bufferSpeedForward.push_front(speedX);
      if (countSpeedForward < bufferSpeedForward.capacity())
        countSpeedForward++;
      else
        bufferForwardFull = true;
    }
    else if (speedX < -minSpeedForDetection / 1000)
    {
      bufferSpeedBackward.push_front(abs(speedX));
      if (countSpeedBackward < bufferSpeedBackward.capacity())
        countSpeedBackward++;
      else
        bufferBackwardFull = true;
    }

    if (requestedSpeedX > minSpeedForDetection / 1000)
    {
      bufferRequestSpeedForward.push_front(requestedSpeedX);
    }
    else if (requestedSpeedX < -minSpeedForDetection / 1000)
    {
      bufferRequestSpeedBackward.push_front(abs(requestedSpeedX));
    }

    float speedY = theSpeedInfo.speed.translation[1];
    float requestedSpeedY = theSpeedRequest.translation.y();

    if (speedY > 0)
    {
      bufferSpeedLeft.push_front(speedY);
      if (countSpeedLeft < bufferSpeedLeft.capacity())
        countSpeedLeft++;
      else
        bufferLeftFull = true;
    }
    else if (speedY < 0)
    {
      bufferSpeedRight.push_front(abs(speedY));
      if (countSpeedRight < bufferSpeedRight.capacity())
        countSpeedRight++;
      else
        bufferRightFull = true;
    }

    if (requestedSpeedY > 0)
    {
      bufferRequestSpeedLeft.push_front(requestedSpeedX);
    }
    else if (requestedSpeedY < 0)
    {
      bufferRequestSpeedRight.push_front(abs(requestedSpeedX));
    }

    if (maxSpeedForward < bufferSpeedForward.average())
      maxSpeedForward = bufferSpeedForward.average();
    if (maxSpeedBackward < bufferSpeedBackward.average())
      maxSpeedBackward = bufferSpeedBackward.average();
    if (maxSpeedLeft < bufferSpeedLeft.average())
      maxSpeedLeft = bufferSpeedLeft.average();
    if (maxSpeedRight < bufferSpeedRight.average())
      maxSpeedRight = bufferSpeedRight.average();

    if (maxSpeedRequestForward < bufferRequestSpeedForward.average())
      maxSpeedRequestForward = bufferRequestSpeedForward.average();
    if (maxSpeedRequestBackward < bufferRequestSpeedBackward.average())
      maxSpeedRequestBackward = bufferRequestSpeedBackward.average();
    if (maxSpeedRequestLeft < bufferRequestSpeedLeft.average())
      maxSpeedRequestLeft = bufferRequestSpeedLeft.average();
    if (maxSpeedRequestRight < bufferRequestSpeedRight.average())
      maxSpeedRequestRight = bufferRequestSpeedRight.average();

    motionState.walkingStatus.maxSpeedForward = maxSpeedForward * 1000.0f;
    motionState.walkingStatus.maxSpeedBackward = maxSpeedBackward * 1000.0f;
    motionState.walkingStatus.maxSpeedLeft = maxSpeedLeft * 1000.0f;
    motionState.walkingStatus.maxSpeedRight = maxSpeedRight * 1000.0f;
  }
}

void MotionMindfulness::walkSpeedFactorPrediction(MotionState& motionState)
{
  if (bufferForwardFull && lastMaxForward == maxSpeedForward)
    bufferForwardFullAndStable = true;
  else
    bufferForwardFullAndStable = false;
  if (bufferBackwardFull && lastMaxBackward == maxSpeedBackward)
    bufferBackwardFullAndStable = true;
  else
    bufferBackwardFullAndStable = false;
  if (bufferLeftFull && lastMaxLeft == maxSpeedLeft)
    bufferLeftFullAndStable = true;
  else
    bufferLeftFullAndStable = false;
  if (bufferRightFull && lastMaxRight == maxSpeedRight)
    bufferRightFullAndStable = true;
  else
    bufferRightFullAndStable = false;

  if (bufferForwardFullAndStable)
  {
    float speedFactorForward = ((100 / (theWalkingEngineParams.speedLimits.xForward * theWalkingEngineParams.speedLimits.speedFactor)) * (maxSpeedForward * 1000.0f)) / 100.0f;
    float speedFactorParam = decraseIncreaseFactorForward / 100;
    if (!hasProblem(motionState, MotionState::walkingForwardFailure) && speedFactorForward > upscaleThreshold)
    {
      speedFactorForward = 1.f + speedFactorParam;
      forwardUpdated = true;

      countSpeedForward = 0; // Cooldown of 10 seconds before a new speed factor is calculated
      bufferForwardFull = false;

      motionState.walkingStatus.speedFactorForward = speedFactorForward;
      decraseIncreaseFactorForward = decraseIncreaseFactorForward - (decraseIncreaseFactorForward * annealingFactor);
      if (decraseIncreaseFactorForward < minDecreaseIncreaseFactor)
        decraseIncreaseFactorForward = minDecreaseIncreaseFactor;
    }
    else if ((hasProblem(motionState, MotionState::walkingForwardFailure) || motionState.walkingStatus.stumblingForward) && speedFactorForward > downscaleThreshold)
    {
      speedFactorForward = 1.f - speedFactorParam;
      forwardUpdated = true;

      countSpeedForward = 0; // Cooldown of 10 seconds before a new speed factor is calculated
      bufferForwardFull = false;

      motionState.walkingStatus.speedFactorForward = speedFactorForward;
      decraseIncreaseFactorForward = decraseIncreaseFactorForward - (decraseIncreaseFactorForward * annealingFactor);
      if (decraseIncreaseFactorForward < minDecreaseIncreaseFactor)
        decraseIncreaseFactorForward = minDecreaseIncreaseFactor;
    }
  }
  else
  {
    motionState.walkingStatus.speedFactorForward = 1.f;
  }
  motionState.walkingStatus.decreaseIncreaseFactors[0] = decraseIncreaseFactorForward;

  if (bufferBackwardFullAndStable)
  {
    float speedFactorBackward = ((100 / (theWalkingEngineParams.speedLimits.xBackward * theWalkingEngineParams.speedLimits.speedFactor)) * (maxSpeedBackward * 1000.0f)) / 100.0f;
    float speedFactorParam = decraseIncreaseFactorBackward / 100;
    if (!hasProblem(motionState, MotionState::walkingBackwardFailure) && speedFactorBackward > upscaleThreshold)
    {
      speedFactorBackward = 1.f + speedFactorParam;
      backwardUpdated = true;

      countSpeedBackward = 0; // Cooldown of 10 seconds before a new speed factor is calculated
      bufferBackwardFull = false;

      motionState.walkingStatus.speedFactorBackward = speedFactorBackward;
      decraseIncreaseFactorBackward = decraseIncreaseFactorBackward - (decraseIncreaseFactorBackward * annealingFactor);
      if (decraseIncreaseFactorBackward < minDecreaseIncreaseFactor)
        decraseIncreaseFactorBackward = minDecreaseIncreaseFactor;
    }
    else if ((hasProblem(motionState, MotionState::walkingBackwardFailure) || motionState.walkingStatus.stumblingBackward) && speedFactorBackward > downscaleThreshold)
    {
      speedFactorBackward = 1.f - speedFactorParam;
      backwardUpdated = true;

      countSpeedBackward = 0; // Cooldown of 10 seconds before a new speed factor is calculated
      bufferBackwardFull = false;

      motionState.walkingStatus.speedFactorBackward = speedFactorBackward;
      decraseIncreaseFactorBackward = decraseIncreaseFactorBackward - (decraseIncreaseFactorBackward * annealingFactor);
      if (decraseIncreaseFactorBackward < minDecreaseIncreaseFactor)
        decraseIncreaseFactorBackward = minDecreaseIncreaseFactor;
    }
  }
  else
  {
    motionState.walkingStatus.speedFactorBackward = 1.f;
  }
  motionState.walkingStatus.decreaseIncreaseFactors[1] = decraseIncreaseFactorBackward;

  if (bufferLeftFullAndStable)
  {
    float speedFactorLeft = ((100 / (theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedLimits.speedFactor)) * (maxSpeedLeft * 1000.0f)) / 100.0f;
    float speedFactorParam = decraseIncreaseFactorLeft / 100;
    if (!hasProblem(motionState, MotionState::walkingLeftFailure) && speedFactorLeft > upscaleThreshold)
    {
      speedFactorLeft = 1.f + speedFactorParam;
      leftUpdated = true;

      countSpeedLeft = 0; // Cooldown of 3 seconds before a new speed factor is calculated
      bufferLeftFull = false;

      motionState.walkingStatus.speedFactorLeft = speedFactorLeft;
      decraseIncreaseFactorLeft = decraseIncreaseFactorLeft - (decraseIncreaseFactorLeft * annealingFactor);
      if (decraseIncreaseFactorLeft < minDecreaseIncreaseFactor)
        decraseIncreaseFactorLeft = minDecreaseIncreaseFactor;
    }
    else if ((hasProblem(motionState, MotionState::walkingLeftFailure) || motionState.walkingStatus.stumblingLeft) && speedFactorLeft > downscaleThreshold)
    {
      speedFactorLeft = 1.f - speedFactorParam;
      leftUpdated = true;

      countSpeedLeft = 0; // Cooldown of 3 seconds before a new speed factor is calculated
      bufferLeftFull = false;

      motionState.walkingStatus.speedFactorLeft = speedFactorLeft;
      decraseIncreaseFactorLeft = decraseIncreaseFactorLeft - (decraseIncreaseFactorLeft * annealingFactor);
      if (decraseIncreaseFactorLeft < minDecreaseIncreaseFactor)
        decraseIncreaseFactorLeft = minDecreaseIncreaseFactor;
    }
  }
  else
  {
    motionState.walkingStatus.speedFactorLeft = 1.f;
  }
  motionState.walkingStatus.decreaseIncreaseFactors[2] = decraseIncreaseFactorLeft;

  if (bufferRightFullAndStable)
  {
    float speedFactorRight = ((100 / (theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedLimits.speedFactor)) * (maxSpeedRight * 1000.0f)) / 100.0f;
    float speedFactorParam = decraseIncreaseFactorRight / 100;
    if (!hasProblem(motionState, MotionState::walkingRightFailure) && speedFactorRight > upscaleThreshold)
    {
      speedFactorRight = 1.f + speedFactorParam;
      rightUpdated = true;

      countSpeedRight = 0; // Cooldown of 3 seconds before a new speed factor is calculated
      bufferRightFull = false;

      motionState.walkingStatus.speedFactorRight = speedFactorRight;
      decraseIncreaseFactorRight = decraseIncreaseFactorRight - (decraseIncreaseFactorRight * annealingFactor);
      if (decraseIncreaseFactorRight < minDecreaseIncreaseFactor)
        decraseIncreaseFactorRight = minDecreaseIncreaseFactor;
    }
    else if ((hasProblem(motionState, MotionState::walkingRightFailure) || motionState.walkingStatus.stumblingRight) && speedFactorRight > downscaleThreshold)
    {
      speedFactorRight = 1.f - speedFactorParam;
      rightUpdated = true;

      countSpeedRight = 0; // Cooldown of 3 seconds before a new speed factor is calculated
      bufferRightFull = false;

      motionState.walkingStatus.speedFactorRight = speedFactorRight;
      decraseIncreaseFactorRight = decraseIncreaseFactorRight - (decraseIncreaseFactorRight * annealingFactor);
      if (decraseIncreaseFactorRight < minDecreaseIncreaseFactor)
        decraseIncreaseFactorRight = minDecreaseIncreaseFactor;
    }
  }
  else
  {
    motionState.walkingStatus.speedFactorRight = 1.f;
  }
  motionState.walkingStatus.decreaseIncreaseFactors[3] = decraseIncreaseFactorRight;
}

void MotionMindfulness::kinematicStumbleDetection(MotionState& motionState)
{
  float lAnklePitch = theKinematicRequest.offsets.angles[Joints::lAnklePitch];
  float rAnklePitch = theKinematicRequest.offsets.angles[Joints::rAnklePitch];

  float lAnkleRoll = theKinematicRequest.offsets.angles[Joints::lAnkleRoll];
  float rAnkleRoll = theKinematicRequest.offsets.angles[Joints::rAnkleRoll];

  float anklePitch = std::max(std::abs(lAnklePitch), std::abs(rAnklePitch));
  float ankleRoll = std::max(std::abs(lAnkleRoll), std::abs(rAnkleRoll));

  bufferKinematicPitchOffsets.push_front(anklePitch);
  bufferKinematicRollOffsets.push_front(ankleRoll);

  if (hasProblem(motionState, MotionState::walkingForwardFailure) || hasProblem(motionState, MotionState::walkingBackwardFailure)
      || hasProblem(motionState, MotionState::walkingLeftFailure) || hasProblem(motionState, MotionState::walkingRightFailure) || theFrameInfo.getTimeSince(startTimestamp) > 45000)
  {
    for (size_t i = 0; i < bufferKinematicPitchOffsets.size(); i++)
    {
      if (maxPitchOffset < bufferKinematicPitchOffsets[i])
        maxPitchOffset = bufferKinematicPitchOffsets[i];

      if (maxRollOffset < bufferKinematicRollOffsets[i])
        maxRollOffset = bufferKinematicRollOffsets[i];
    }
    usableKinematicOffsets = true;
  }


  float stumblePitch = 0.f;
  if (usableKinematicOffsets && maxPitchOffset > 0.f)
    stumblePitch = anklePitch / maxPitchOffset;

  float stumbleRoll = 0.f;
  if (usableKinematicOffsets && maxRollOffset > 0.f)
    stumbleRoll = ankleRoll / maxRollOffset;

  motionState.walkingStatus.kinematicStumble = std::max(stumblePitch, stumbleRoll);
}

void MotionMindfulness::angleVarianceStumbleDetection(MotionState& motionState)
{
  if (theFallDownState.state == FallDownState::upright && theMotionRequest.motion == MotionRequest::walk)
  {
    if (hasProblem(motionState, MotionState::walkingForwardFailure) || hasProblem(motionState, MotionState::walkingBackwardFailure)
        || hasProblem(motionState, MotionState::walkingLeftFailure) || hasProblem(motionState, MotionState::walkingRightFailure) || theFrameInfo.getTimeSince(startTimestamp) > 45000)
    {
      usableAngleValues = true;
    }

    float angleX = theJoinedIMUData.imuData[anglesource].angle.x();
    float angleY = theJoinedIMUData.imuData[anglesource].angle.y();

    angleValuesX.push_front(angleX);
    angleValuesY.push_front(angleY);

    if (angleValuesX.full() && angleValuesY.full())
    {
      bufferAngleValuesX.push_front(angleValuesX[angleValuesX.size() - 1]);
      bufferAngleValuesY.push_front(angleValuesY[angleValuesY.size() - 1]);

      if (bufferAngleValuesX.full() && bufferAngleValuesY.full())
      {
        float tmpMinAngleX = FLT_MAX;
        float tmpMaxAngleX = -FLT_MAX;
        float tmpMinAngleY = FLT_MAX;
        float tmpMaxAngleY = -FLT_MAX;
        // Assuming bufferAngleValuesX and bufferAngleValuesY has the same size
        for (size_t i = 0; i < bufferAngleValuesX.size(); i++)
        {
          float tmpAngleX = bufferAngleValuesX[i];
          float tmpAngleY = bufferAngleValuesY[i];

          if (minVarianceX > tmpAngleX)
            minVarianceX = tmpAngleX;
          if (maxVarianceX < tmpAngleX)
            maxVarianceX = tmpAngleX;

          if (minVarianceY > tmpAngleY)
            minVarianceY = tmpAngleY;
          if (maxVarianceY < tmpAngleY)
            maxVarianceY = tmpAngleY;

          if (tmpMinAngleX > tmpAngleX)
            tmpMinAngleX = tmpAngleX;
          if (tmpMaxAngleX < tmpAngleX)
            tmpMaxAngleX = tmpAngleX;

          if (tmpMinAngleY > tmpAngleY)
            tmpMinAngleY = tmpAngleY;
          if (tmpMaxAngleY < tmpAngleY)
            tmpMaxAngleY = tmpAngleY;
        }

        if (usableAngleValues)
        {
          float tmpAngleScatterX = tmpMaxAngleX - tmpMinAngleX;
          float angleScatterX = maxVarianceX - minVarianceX;

          float tmpAngleScatterY = tmpMaxAngleY - tmpMinAngleY;
          float angleScatterY = maxVarianceY - minVarianceY;

          float angleScatterRatioX;
          if (angleScatterX > 0)
            angleScatterRatioX = tmpAngleScatterX / angleScatterX;
          else
            angleScatterRatioX = 0;

          float angleScatterRatioY;
          if (angleScatterY > 0)
            angleScatterRatioY = tmpAngleScatterY / angleScatterY;
          else
            angleScatterRatioY = 0;

          motionState.walkingStatus.angleStumble = std::max(angleScatterRatioX, angleScatterRatioY);
        }
      }
    }
  }
  else
  {
    angleValuesX.clear();
    angleValuesY.clear();
    motionState.walkingStatus.angleStumble = 0.f;
  }
}

void MotionMindfulness::fsrVarianceStumbleDetection(MotionState& motionState)
{
  if (theFallDownState.state == FallDownState::upright && theMotionRequest.motion == MotionRequest::walk)
  {
    if (hasProblem(motionState, MotionState::walkingForwardFailure) || hasProblem(motionState, MotionState::walkingBackwardFailure)
        || hasProblem(motionState, MotionState::walkingLeftFailure) || hasProblem(motionState, MotionState::walkingRightFailure) || theFrameInfo.getTimeSince(startTimestamp) > 45000)
    {
      usableFsrValues = true;
    }

    fsrValuesLeft.push_front(theFsrSensorData.left);
    fsrValuesRight.push_front(theFsrSensorData.right);

    if (fsrValuesLeft.full() && fsrValuesRight.full())
    {
      bufferFsrValuesLeft.push_front(fsrValuesLeft.back());
      bufferFsrValuesRight.push_front(fsrValuesRight.back());

      // Assuming bufferFsrValuesLeft and bufferFsrValuesRight has the same size
      if (bufferFsrValuesLeft.full() && bufferFsrValuesRight.full())
      {
        std::array<float, FsrSensorData::numOfFsrSensorPositions> tmpMinLeft = {FLT_MAX};
        std::array<float, FsrSensorData::numOfFsrSensorPositions> tmpMaxLeft = {-FLT_MAX};
        std::array<float, FsrSensorData::numOfFsrSensorPositions> tmpMinRight = {FLT_MAX};
        std::array<float, FsrSensorData::numOfFsrSensorPositions> tmpMaxRight = {-FLT_MAX};

        std::array<float, FsrSensorData::numOfFsrSensorPositions> fsrScatterLeft = {0};
        std::array<float, FsrSensorData::numOfFsrSensorPositions> fsrScatterRight = {0};

        std::array<float, FsrSensorData::numOfFsrSensorPositions> tmpFsrScatterLeft = {0};
        std::array<float, FsrSensorData::numOfFsrSensorPositions> tmpFsrScatterRight = {0};

        for (size_t i = 0; i < bufferFsrValuesLeft.size(); i++)
        {
          std::array<float, FsrSensorData::numOfFsrSensorPositions> tmpFsrDataLeft = bufferFsrValuesLeft[i];
          std::array<float, FsrSensorData::numOfFsrSensorPositions> tmpFsrDataRight = bufferFsrValuesRight[i];

          for (int j = 0; j < FsrSensorData::numOfFsrSensorPositions; j++)
          {
            if (minVarianceLeft[j] > tmpFsrDataLeft[j])
              minVarianceLeft[j] = tmpFsrDataLeft[j];
            if (maxVarianceLeft[j] < tmpFsrDataLeft[j])
              maxVarianceLeft[j] = tmpFsrDataLeft[j];

            if (minVarianceRight[j] > tmpFsrDataRight[j])
              minVarianceRight[j] = tmpFsrDataRight[j];
            if (maxVarianceRight[j] < tmpFsrDataRight[j])
              maxVarianceRight[j] = tmpFsrDataRight[j];

            if (tmpMinLeft[j] > tmpFsrDataLeft[j])
              tmpMinLeft[j] = tmpFsrDataLeft[j];
            if (tmpMaxLeft[j] < tmpFsrDataLeft[j])
              tmpMaxLeft[j] = tmpFsrDataLeft[j];

            if (tmpMinRight[j] > tmpFsrDataRight[j])
              tmpMinRight[j] = tmpFsrDataRight[j];
            if (tmpMaxRight[j] < tmpFsrDataRight[j])
              tmpMaxRight[j] = tmpFsrDataRight[j];

            fsrScatterLeft[j] = maxVarianceLeft[j] - minVarianceLeft[j];
            fsrScatterRight[j] = maxVarianceRight[j] - minVarianceRight[j];
            tmpFsrScatterLeft[j] = tmpMaxLeft[j] - tmpMinLeft[j];
            tmpFsrScatterRight[j] = tmpMaxRight[j] - tmpMinRight[j];
          }
        }

        if (usableFsrValues)
        {
          float maxScatter = 0.f;
          for (int i = 0; i < FsrSensorData::numOfFsrSensorPositions; i++)
          {
            float tmpScatter;
            if (fsrScatterLeft[i] == 0.f || fsrScatterRight[i] == 0.f)
              tmpScatter = 0.f;
            else
              tmpScatter = std::min((tmpFsrScatterLeft[i] / fsrScatterLeft[i]), (tmpFsrScatterRight[i] / fsrScatterRight[i]));

            if (maxScatter < tmpScatter)
              maxScatter = tmpScatter;
          }
          motionState.walkingStatus.fsrStumble = maxScatter;
        }
      }
    }
  }
  else
  {
    fsrValuesLeft.clear();
    fsrValuesRight.clear();
    motionState.walkingStatus.fsrStumble = 0.f;
  }
}

void MotionMindfulness::stumbleDetectionAccumulator(MotionState& motionState)
{
  float angleStumble = motionState.walkingStatus.angleStumble;
  float kinematicStumble = motionState.walkingStatus.kinematicStumble;
  float fsrStumble = motionState.walkingStatus.fsrStumble;

  float sumOfWeights = angleStumblingWeight + kinematicStumblingWeight + fsrStumblingWeight;
  if (sumOfWeights == 0.f)
    sumOfWeights = 1.f;

  float stumble = ((angleStumblingWeight * angleStumble) + (kinematicStumblingWeight * kinematicStumble) + (fsrStumblingWeight * fsrStumble)) / sumOfWeights;
  motionState.walkingStatus.stumble = stumble;

  if (stumble > stumblingThreshold)
  {
    if (walkingForward)
      motionState.walkingStatus.stumblingForward = true;
    if (walkingBackward)
      motionState.walkingStatus.stumblingBackward = true;
    if (walkingLeft)
      motionState.walkingStatus.stumblingLeft = true;
    if (walkingRight)
      motionState.walkingStatus.stumblingRight = true;
  }
  else
  {
    motionState.walkingStatus.stumblingForward = false;
    motionState.walkingStatus.stumblingBackward = false;
    motionState.walkingStatus.stumblingLeft = false;
    motionState.walkingStatus.stumblingRight = false;
  }

  PLOT("module:MotionMindfulness:walkingStatus:stumble", motionState.walkingStatus.stumble);
  PLOT("module:MotionMindfulness:walkingStatus:stumblingForward", motionState.walkingStatus.stumblingForward);
  PLOT("module:MotionMindfulness:walkingStatus:stumblingBackward", motionState.walkingStatus.stumblingBackward);
  PLOT("module:MotionMindfulness:walkingStatus:stumblingLeft", motionState.walkingStatus.stumblingLeft);
  PLOT("module:MotionMindfulness:walkingStatus:stumblingRight", motionState.walkingStatus.stumblingRight);
}

void MotionMindfulness::updateWalkErrors(MotionState& motionState, const bool increase)
{
  unsigned currentTime = theFrameInfo.time;
  unsigned resetThreshold = resetWalkingErrorThreshold;

  walkingForward = false;
  walkingBackward = false;
  walkingLeft = false;
  walkingRight = false;

  /***********************************/
  /* walking sidewards check         */
  /***********************************/
  if (std::abs(theMotionRequest.walkRequest.request.translation[0]) <= maxSpeedForwardBackward
      && std::abs(theMotionRequest.walkRequest.request.translation[1]) > std::abs(theMotionRequest.walkRequest.request.translation[0]))
  {
    /***********************************/
    /* walking left check              */
    /***********************************/
    if (theMotionRequest.walkRequest.request.translation[1] > minSpeedForDetection)
    {
      walkingLeft = true;
      if (increase)
      {
        if (errorsWalking[0] < maxAllowedUnstableFrames)
        {
          errorsWalking[0]++;
          lastUpdatedWalkingError[0] = currentTime;
        }
      }
      else
      {
        if ((hasProblem(motionState, MotionState::walkingLeftFailure) || currentTime - lastUpdatedWalkingError[3] > resetThreshold) && errorsWalking[0] > 0)
          errorsWalking[0]--;
      }
      PLOT("module:MotionMindfulness:walkingSidewards", errorsWalking[0] + 1);
    }
    /***********************************/
    /* walking right check             */
    /***********************************/
    else if (theMotionRequest.walkRequest.request.translation[1] < -minSpeedForDetection)
    {
      walkingRight = true;
      if (increase)
      {
        if (errorsWalking[1] < maxAllowedUnstableFrames)
        {
          errorsWalking[1]++;
          lastUpdatedWalkingError[1] = currentTime;
        }
      }
      else
      {
        if ((hasProblem(motionState, MotionState::walkingRightFailure) || currentTime - lastUpdatedWalkingError[3] > resetThreshold) && errorsWalking[1] > 0)
          errorsWalking[1]--;
      }
      PLOT("module:MotionMindfulness:walkingSidewards", -1 * (errorsWalking[1] + 1));
    }
    else
    {
      PLOT("module:MotionMindfulness:walkingSidewards", 0);
    }
  }
  /***********************************/
  /* walking lateral check           */
  /***********************************/
  else if (std::abs(theMotionRequest.walkRequest.request.translation[1]) <= maxSpeedSideward
      && std::abs(theMotionRequest.walkRequest.request.translation[0]) > std::abs(theMotionRequest.walkRequest.request.translation[1]))
  {
    /***********************************/
    /* walking forward check           */
    /***********************************/
    if (theMotionRequest.walkRequest.request.translation[0] > minSpeedForDetection)
    {
      walkingForward = true;
      if (increase)
      {
        if (errorsWalking[2] < maxAllowedUnstableFrames && currentTime - lastEngineKickTime > timeAfterEngineKick && currentTime - lastCustomKickTime > timeAfterCustomKick)
        {
          errorsWalking[2]++;
          lastUpdatedWalkingError[2] = currentTime;
        }
      }
      else
      {
        if ((hasProblem(motionState, MotionState::walkingForwardFailure) || currentTime - lastUpdatedWalkingError[3] > resetThreshold) && errorsWalking[2] > 0)
          errorsWalking[2]--;
      }
      PLOT("module:MotionMindfulness:walkingForwardBackward", errorsWalking[2] + 1);
    }
    /***********************************/
    /* walking backward check          */
    /***********************************/
    else if (theMotionRequest.walkRequest.request.translation[0] < -minSpeedForDetection)
    {
      walkingBackward = true;
      if (increase)
      {
        if (errorsWalking[3] < maxAllowedUnstableFrames)
        {
          errorsWalking[3]++;
          lastUpdatedWalkingError[3] = currentTime;
        }
      }
      else
      {
        if ((hasProblem(motionState, MotionState::walkingBackwardFailure) || currentTime - lastUpdatedWalkingError[3] > resetThreshold) && errorsWalking[3] > 0)
          errorsWalking[3]--;
      }
      PLOT("module:MotionMindfulness:walkingForwardBackward", -1 * (errorsWalking[3] + 1));
    }
    else
    {
      PLOT("module:MotionMindfulness:walkingForwardBackward", 0);
    }
  }

  else
  {
    PLOT("module:MotionMindfulness:walkingSidewards", 0);
    PLOT("module:MotionMindfulness:walkingForwardBackward", 0);
  }
}
