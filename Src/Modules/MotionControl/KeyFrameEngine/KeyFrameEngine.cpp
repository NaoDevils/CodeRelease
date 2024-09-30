/**
 * @file KeyFrameEngine.cpp
 * This file declares a module that handles key frame motions.
 * @author <a href="ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
 * @author Diana Kleingarn
 * @author Dominik Brämer
 */

#include "KeyFrameEngine.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Range.h"
#include "Tools/Settings.h"
#include "Platform/File.h"
#include <sstream>
#include <iomanip>
#include <stdlib.h>
#include <cmath>
#include <cstdlib>

MAKE_MODULE(KeyFrameEngine, motionControl)

CycleLocal<KeyFrameEngine*> KeyFrameEngine::theInstance(nullptr);

void KeyFrameMotion::KeyFrame::mirror()
{
  const auto leftArm = armsAngles.begin();
  const auto rightArm = leftArm + armsAngles.size() / 2;
  std::swap_ranges(leftArm, rightArm, rightArm);

  const auto leftLeg = legsAngles.begin();
  const auto rightLeg = leftLeg + legsAngles.size() / 2;
  std::swap_ranges(leftLeg, rightLeg, rightLeg);

  const auto leftArmStiffness = stiffnesses.begin() + Joints::Joint::firstLeftArmJoint;
  const auto rightArmStiffness = stiffnesses.begin() + Joints::Joint::firstRightArmJoint;
  const auto leftLegStiffness = stiffnesses.begin() + Joints::Joint::lHipYawPitch;
  const auto rightLegStiffness = stiffnesses.begin() + Joints::Joint::rHipYawPitch;
  std::swap_ranges(leftArmStiffness, rightArmStiffness, rightArmStiffness);
  std::swap_ranges(leftLegStiffness, rightLegStiffness, rightLegStiffness);

  const auto m = [](auto it)
  {
    auto& v = *it;
    if (v != JointAngles::off && v != JointAngles::ignore)
      v = -v;
  };

  m(headAngles.begin()); // head yaw

  for (size_t i = 1; i <= 4; ++i) // lShoulderRoll, lElbowYaw, lElbowRoll, lWristYaw
  {
    m(leftArm + i);
    m(rightArm + i);
  }

  m(leftLeg + 1); // hipRoll
  m(rightLeg + 1); // hipRoll
  m(leftLeg + 5); // ankleRoll
  m(rightLeg + 5); // ankleRoll
}

KeyFrameEngine::KeyFrameEngine()
{
  theInstance = this;
  countStandUpStarts.resize(2);
  for (int j = 0; j < 2; j++)
  {
    countStandUpStarts[j].resize((SpecialActionRequest::lastStandUpMotion - SpecialActionRequest::firstStandUpMotion) + 1);
    for (int i = 0; i < static_cast<int>(countStandUpStarts[j].size()); i++)
      countStandUpStarts[j][i] = 1;
  }
  countStandUpAborts.resize(2);
  for (int j = 0; j < 2; j++)
  {
    countStandUpAborts[j].resize((SpecialActionRequest::lastStandUpMotion - SpecialActionRequest::firstStandUpMotion) + 1);
    for (int i = 0; i < static_cast<int>(countStandUpAborts[j].size()); i++)
      countStandUpAborts[j][i] = 0;
  }
  standUpStatistic.resize(2);
  for (int j = 0; j < 2; j++)
  {
    standUpStatistic[j].resize((SpecialActionRequest::lastStandUpMotion - SpecialActionRequest::firstStandUpMotion) + 1);
    for (int i = 0; i < static_cast<int>(standUpStatistic[j].size()); i++)
      standUpStatistic[j][i] = 0;
  }

  lJointErrorBuffer.fill(0.f);
  rJointErrorBuffer.fill(0.f);
  jointErrorBuffer.fill(0.f);
}

void KeyFrameEngine::init(SpecialActionsOutput& specialActionsOutput)
{
  if (!initialized)
  {
    keyFrameMotions = loadKeyFrameMotions();
    initEngineData(specialActionsOutput); // set to current joint angles and default stiffness
  }

  specialActionsOutput.standUpStatisticFront = std::vector<float>(standUpStatistic[0].begin(), standUpStatistic[0].end());
  specialActionsOutput.standUpStatisticBack = std::vector<float>(standUpStatistic[1].begin(), standUpStatistic[1].end());

  // only relevant on first frame, done bc we could start the framework with keyFrameEngine ratio == 1.f
  initialized = true;
}

void KeyFrameEngine::initEngineData(SpecialActionsOutput& specialActionsOutput)
{
  for (int i = 0; i < Joints::numOfJoints; i++)
    specialActionsOutput.angles[i] = theJointSensorData.angles[i];
  specialActionsOutput.isLeavingPossible = true;
  specialActionsOutput.isMotionFinished = false;
  specialActionsOutput.stiffnessData.stiffnesses = defaultStiffnesses;
  specialActionsOutput.lCompensatedID = Joints::headYaw;
  specialActionsOutput.lError = 0_deg;
  specialActionsOutput.rCompensatedID = Joints::headYaw;
  specialActionsOutput.rError = 0_deg;
  ASSERT(specialActionsOutput.isValid());

  // index to first frame
  currentKeyFrameIndex = 0;
  phase = 0.f;

  // set originFrame to sensor values and default stiffness
  originFrame = getKeyFrameFromJointRequest(specialActionsOutput);
  originFrame.stiffnesses = defaultStiffnesses;

  // other members are already initialized with defaults

  // set correct ID
  selectActiveMotion(theMotionSelection.specialActionRequest.specialAction, theMotionSelection.specialActionRequest.mirror, specialActionsOutput);

  // reset sensor control
  angleYBuffer.fill(0);
  gyroYBuffer.fill(0);
  gyroXBuffer.fill(0);
  hipYawBuffer.fill(theJointSensorData.angles[Joints::lHipYawPitch]);
  lAnkleRollBuffer.fill(theJointSensorData.angles[Joints::lAnkleRoll]);
  rAnkleRollBuffer.fill(theJointSensorData.angles[Joints::rAnkleRoll]);
  for (int i = 0; i < 5; i++)
    for (int j = 0; j < Joints::numOfJoints; j++)
      jointAngleBuffer[i].angles[j] = theJointSensorData.angles[j];
  engineDataReset = true;
}

void KeyFrameEngine::update(SpecialActionsOutput& specialActionsOutput)
{
  DECLARE_PLOT("module:KeyFrameEngine:hipYawPitchError");
  DECLARE_PLOT("module:KeyFrameEngine:realAngleError");
  DECLARE_PLOT("module:KeyFrameEngine:setAngleErrorFront");
  DECLARE_PLOT("module:KeyFrameEngine:setAngleErrorBack");
  DECLARE_PLOT("module:KeyFrameEngine:targetAngle");
  DECLARE_PLOT("module:KeyFrameEngine:currentAngle");
  DECLARE_PLOT("module:KeyFrameEngine:desiredGyroY");
  DECLARE_PLOT("module:KeyFrameEngine:torsoY");
  DECLARE_PLOT("module:KeyFrameEngine:error");

  MODIFY("module:KeyFrameEngine:debugMode", debugMode);
  bool playNextFrameInDebugMode = false;
  DEBUG_RESPONSE_ONCE("module:KeyFrameEngine:playNextFrame")
  {
    playNextFrameInDebugMode = true;
  }
  DEBUG_RESPONSE_ONCE("loadKeyFrames")
  {
    keyFrameMotions = loadKeyFrameMotions();
  }
  DEBUG_RESPONSE_ONCE("printKeyFrame")
  {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << "headAngles = [" << theJointSensorData.angles[Joints::headYaw] << "deg," << theJointSensorData.angles[Joints::headPitch] << "deg];" << std::endl;
    ss << "armsAngles = [" << toDegrees(theJointSensorData.angles[Joints::lShoulderPitch]) << "deg," << toDegrees(theJointSensorData.angles[Joints::lShoulderRoll]) << "deg,"
       << toDegrees(theJointSensorData.angles[Joints::lElbowYaw]) << "deg," << toDegrees(theJointSensorData.angles[Joints::lElbowRoll]) << "deg,"
       << "-90deg"
       << "," //fixed value
       << "0deg"
       << ", " //fixed value
       << toDegrees(theJointSensorData.angles[Joints::rShoulderPitch]) << "deg," << toDegrees(theJointSensorData.angles[Joints::rShoulderRoll]) << "deg,"
       << toDegrees(theJointSensorData.angles[Joints::rElbowYaw]) << "deg," << toDegrees(theJointSensorData.angles[Joints::rElbowRoll]) << "deg,"
       << "90deg"
       << "," //fixed value
       << "0deg" << //fixed value
        "];" << std::endl;
    ss << "legsAngles = [";
    for (int i = Joints::lHipYawPitch; i <= Joints::rAnkleRoll; i++)
      ss << ((i != Joints::rHipYawPitch) ? "" : " ") << toDegrees(theJointSensorData.angles[i]) << "deg" << ((i != Joints::rAnkleRoll) ? "," : "");
    ss << "];";

    OUTPUT_TEXT(ss.str());
  }

  if (lastPhaseUpdateTimestamp == 0 && theFrameInfo.time > 0)
  {
    currentCycleTime = static_cast<unsigned>(theFrameInfo.cycleTime * 1000);
  }
  else
  {
    currentCycleTime = theFrameInfo.getTimeSince(lastPhaseUpdateTimestamp);
  }
  lastPhaseUpdateTimestamp = theFrameInfo.time;

  // update sensor/request buffers
  angleYBuffer.push_front(theJoinedIMUData.imuData[anglesource].angle.y());
  gyroYBuffer.push_front(theJoinedIMUData.imuData[anglesource].gyro.y());
  gyroXBuffer.push_front(theJoinedIMUData.imuData[anglesource].gyro.x());
  hipYawBuffer.push_front(specialActionsOutput.angles[Joints::lHipYawPitch]);
  lAnkleRollBuffer.push_front(specialActionsOutput.angles[Joints::lAnkleRoll]);
  rAnkleRollBuffer.push_front(specialActionsOutput.angles[Joints::rAnkleRoll]);

  handleInitialFallDownProtection(specialActionsOutput);

  if (theFallDownState.state == FallDownState::upright)
    prevFlying = false;

  if (!initialized)
  {
    init(specialActionsOutput);
  }
  else if (!areMotionsPrioritized() && theMotionSelection.ratios[MotionRequest::specialAction] < 1.f)
  {
    if (!engineDataReset || theMotionSelection.ratios[MotionRequest::specialAction] == 0.f)
      initEngineData(specialActionsOutput);
  }
  else
  {
    // Set the right special actions which are needed to get the right transitions between special action and behavior
    handleTransitions(specialActionsOutput);

    if (prevFlying)
      return;

    // Update specialActionsOutput
    specialActionsOutput.isLeavingPossible = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex).leavingPossible;
    specialActionsOutput.isMotionStable = isMotionStable(specialActionsOutput);
    specialActionsOutput.executedSpecialAction.specialAction = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID;
    specialActionsOutput.executedSpecialAction.mirror = currentKeyFrameMirror;

    // Abort keyframe motion if leaving is possible and the current motion request is not a special action
    if (specialActionsOutput.isLeavingPossible && theMotionSelection.targetMotion != MotionRequest::specialAction)
      return;

    // Prevent damage of stucked arms by pause the current keyframe for maxWaitForArms milliseconds
    if (prevArmsStuck && theFrameInfo.getTimeSince(keyFrameFinishedTimestamp) < maxWaitForArmsTime)
      return;
    else if (prevArmsStuck && theFrameInfo.getTimeSince(keyFrameFinishedTimestamp) >= maxWaitForArmsTime)
    {
      prevArmsStuck = false;
      keyFrameFinishedTimestamp = theFrameInfo.time;
      armsStuckCount += 1;
    }
    if (armsStuckCount >= 3)
    {
      if constexpr (Build::targetRobot())
        SystemCall::text2Speech("Help, arm stuck.");
      priorizeMotion(SpecialActionRequest::untangleArms, false);
      armsStuckCount = 0;
    }

    plotAngleAtKeyframeTarget(specialActionsOutput);

    KeyFrameMotion::KeyFrame& currentKeyFrame = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex);
    // interpolate returns true if phase >= 1.f, i.e. single keyFrame was executed
    if (interpolate(specialActionsOutput) && (!debugMode || playNextFrameInDebugMode))
    {
      // check if robots joints and upper body are ok after interpolation, otherwise cancel motion
      if ((!verifyRobotPosition(specialActionsOutput) || (!areMotionsPrioritized() && theFallDownState.state == FallDownState::falling && !specialActionsOutput.isFallProtectionNeeded))
          && !debugModeForWalkingCalibration)
      {
        // handles the fall down protection between two key frames
        lastFinishedKeyFrameMotionIndex = currentKeyFrameMotionIndex;
        lastFinishedKeyFrameIndex = currentKeyFrameIndex;

        chooseFallDownProtection(specialActionsOutput, currentKeyFrame.fallDownProtection, currentKeyFrame.holdFallDownProtection);
        if (areMotionsPrioritized())
          return;
      }

      // Update last finished key frame index and las finished key frame motion index
      if (lastFinishedKeyFrameIndex != currentKeyFrameIndex || lastFinishedKeyFrameMotionIndex != currentKeyFrameMotionIndex)
      {
        lastFinishedKeyFrameIndex = currentKeyFrameIndex;
        lastFinishedKeyFrameMotionIndex = currentKeyFrameMotionIndex;
      }

      // Pauses keyframe for stabilization
      if (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex).waitForStable
          && theFrameInfo.getTimeSince(keyFrameFinishedTimestamp) < maxWaitForStableTime && !isStable())
        return;

      // Wait for arm angle to be nearly reached
      if (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex).armAngleReached && !armAngleReached(specialActionsOutput)
          && theFrameInfo.getTimeSince(keyFrameFinishedTimestamp) < maxWaitForArmAngleTime)
        return;

      // Pauses keyframe if robot gets disturbed
      if (armsStuck() == ArmsStuckState::stuck && !prevArmsStuck && !debugModeForWalkingCalibration)
      {
        if (theFallDownState.direction == FallDownState::back)
        {
          priorizeMotion(SpecialActionRequest::freeArmsBack, false);
        }
        else
          prevArmsStuck = true;
        return;
      }
      else if (armsStuck() == ArmsStuckState::lyingOnArm && !prevArmsStuck && !debugModeForWalkingCalibration)
      {
        priorizeMotion(SpecialActionRequest::freeArmsFront, false);
      }

      // Hold keyfame for at least x milliseconds
      if (theFrameInfo.getTimeSince(keyFrameFinishedTimestamp) < (int)currentKeyFrame.holdFrame)
        return;

      SpecialActionRequest::SpecialActionID nextKeyFrame = selectNextMotionID(currentKeyFrame.nextKeyFrameIDs, currentKeyFrame.nextKeyFrameConditions);
      if (nextKeyFrame != SpecialActionRequest::none)
      {
        clearPrioritizedMotions();
        initEngineData(specialActionsOutput);
        if (std::find(ignoreInitialKeyFrameIfChained.begin(), ignoreInitialKeyFrameIfChained.end(), nextKeyFrame) != ignoreInitialKeyFrameIfChained.end())
          ignoreInitialKeyFrame = true;
        selectActiveMotion(nextKeyFrame, false, specialActionsOutput);
        phase = 0.f;
        return;
      }

      // Set current frame as originFrame for possible transition to next motion!
      originFrame = getKeyFrameFromJointRequest(specialActionsOutput);
      phase = 0.f; // reset phase

      // check if we are finished with the motion. if yes we can leave or run the next or repeat the last output forever
      int numberOfKeyFrames = static_cast<int>(keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.size());
      if (currentKeyFrameIndex >= numberOfKeyFrames - 1)
      {
        SpecialActionRequest::SpecialActionID nextKeyFrameMotion =
            selectNextMotionID(keyFrameMotions.at(currentKeyFrameMotionIndex).nextKeyFrameMotionIDs, keyFrameMotions.at(currentKeyFrameMotionIndex).nextMotionConditions);

        if constexpr (Build::targetSimulator())
        {
          if (ignoreStandUpNotWorkingInSimulator)
            standUpNotWorking = false;
        }

        if (standUpNotWorking)
        {
          selectActiveMotion(SpecialActionRequest::lying, false, specialActionsOutput);
          return;
        }
        else if (!areMotionsPrioritized() && nextKeyFrameMotion != SpecialActionRequest::none)
        {
          if (std::find(ignoreInitialKeyFrameIfChained.begin(), ignoreInitialKeyFrameIfChained.end(), nextKeyFrameMotion) != ignoreInitialKeyFrameIfChained.end())
            ignoreInitialKeyFrame = true;
          selectActiveMotion(nextKeyFrameMotion, false, specialActionsOutput);
          return;
        }
        else if (!areMotionsPrioritized() && theFallDownState.state != FallDownState::falling && initialKeyFrameMotionID != theMotionSelection.specialActionRequest.specialAction
            && theMotionSelection.targetMotion == MotionRequest::specialAction) // next motion is selected
        {
          selectActiveMotion(theMotionSelection.specialActionRequest.specialAction, theMotionSelection.specialActionRequest.mirror, specialActionsOutput);
          ignoreInitialKeyFrame = false;
          return;
        }

        if (!specialActionsOutput.isMotionFinished)
          keyFrameMotionFinishedTimestamp = theFrameInfo.time;
        specialActionsOutput.isMotionFinished = true;
        phase = 1.f;
        stabilize(specialActionsOutput);
      }

      if (currentKeyFrameIndex < numberOfKeyFrames - 1)
        currentKeyFrameIndex++; // next keyFrame
    }

    engineDataReset = false;
  }
}

void KeyFrameEngine::plotAngleAtKeyframeTarget(SpecialActionsOutput& specialActionsOutput)
{
  KeyFrameMotion::KeyFrame& currentKeyFrame = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex);

  Angle currentAngle = theJoinedIMUData.imuData[anglesource].angle.y();
  // check if sensor y angle of upper body is too different from intended angle for this key frame
  bool useAngleAtKeyFrameTarget = currentKeyFrame.angleAtKeyFrameError[0] >= 0 && currentKeyFrame.angleAtKeyFrameError[1] >= 0;
  if (useAngleAtKeyFrameTarget)
  {
    PLOT("module:KeyFrameEngine:realAngleError", (currentAngle.toDegrees() - currentKeyFrame.angleAtKeyFrameTarget.toDegrees()));
    PLOT("module:KeyFrameEngine:setAngleErrorFront", currentKeyFrame.angleAtKeyFrameError[0].toDegrees());
    PLOT("module:KeyFrameEngine:setAngleErrorBack", -currentKeyFrame.angleAtKeyFrameError[1].toDegrees());
  }
  else
  {
    PLOT("module:KeyFrameEngine:realAngleError", 0);
    PLOT("module:KeyFrameEngine:setAngleErrorFront", 0);
    PLOT("module:KeyFrameEngine:setAngleErrorBack", 0);
  }
  PLOT("module:KeyFrameEngine:targetAngle", currentKeyFrame.angleAtKeyFrameTarget.toDegrees());
  PLOT("module:KeyFrameEngine:currentAngle", currentAngle.toDegrees());
}

bool KeyFrameEngine::interpolate(SpecialActionsOutput& specialActionsOutput)
{
  if (currentKeyFrameMotionIndex == SpecialActionRequest::none)
    return false;

  JointRequest currentRequest = specialActionsOutput;
  // do not run this method if interpolation is already finished;
  if (phase == 1.f)
    return true;
  KeyFrameMotion& currentKeyFrameMotion = keyFrameMotions.at(currentKeyFrameMotionIndex);
  KeyFrameMotion::KeyFrame currentKeyFrame = currentKeyFrameMotion.keyFrames.at(currentKeyFrameIndex);

  if (!areMotionsPrioritized() && theFallDownState.state == FallDownState::falling && !debugModeForWalkingCalibration && !specialActionsOutput.isFallProtectionNeeded)
    chooseFallDownProtection(specialActionsOutput, currentKeyFrame.fallDownProtection, currentKeyFrame.holdFallDownProtection);

  if (currentKeyFrameMirror)
    currentKeyFrame.mirror();

  if (phase == 0.f)
  {
    keyFrameWasFinished = false;
    if (currentKeyFrameIndex == 0)
    {
      lastKeyFrame = originFrame;
    }
    else
    {
      lastKeyFrame = currentKeyFrameMotion.keyFrames.at(currentKeyFrameIndex - 1);
      if (currentKeyFrameMirror)
        lastKeyFrame.mirror();
    }
    lastKeyFrame = setKeyFrameAngles(lastKeyFrame);
  }

  phase += (1.f / currentKeyFrame.duration) * currentCycleTime;
  phase = std::min(1.f, phase);

  // interpolation type linear or sine for now
  float ratio = (currentKeyFrame.intType == KeyFrameMotion::KeyFrame::KeyFrameInterpolationType::linear) ? phase : (std::cos(phase * pi) - 1.f) / -2.f;
  for (int i = 0; i < Joints::firstArmJoint; i++)
  {
    if (currentKeyFrame.headAngles[i] == JointAngles::off)
      currentRequest.angles[i] = theJointSensorData.angles[i];
    else if (currentKeyFrame.headAngles[i] == JointAngles::ignore)
      currentRequest.angles[i] = JointAngles::ignore;
    else
      currentRequest.angles[i] = currentKeyFrame.headAngles[i] * ratio + lastKeyFrame.headAngles[i] * (1.f - ratio);
  }
  for (int i = Joints::firstArmJoint; i < Joints::lHipYawPitch; i++)
  {
    if (currentKeyFrame.armsAngles[i - Joints::firstArmJoint] == JointAngles::off)
      currentRequest.angles[i] = theJointSensorData.angles[i];
    else if (currentKeyFrame.armsAngles[i - Joints::firstArmJoint] == JointAngles::ignore)
      currentRequest.angles[i] = JointAngles::ignore;
    else
      currentRequest.angles[i] = currentKeyFrame.armsAngles[i - Joints::firstArmJoint] * ratio + lastKeyFrame.armsAngles[i - Joints::firstArmJoint] * (1.f - ratio);
  }
  for (int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
  {
    if (currentKeyFrame.legsAngles[i - Joints::lHipYawPitch] == JointAngles::off)
      currentRequest.angles[i] = theJointSensorData.angles[i];
    else if (currentKeyFrame.legsAngles[i - Joints::lHipYawPitch] == JointAngles::ignore)
      currentRequest.angles[i] = JointAngles::ignore;
    else
    {
      currentRequest.angles[i] = currentKeyFrame.legsAngles[i - Joints::lHipYawPitch] * ratio + lastKeyFrame.legsAngles[i - Joints::lHipYawPitch] * (1.f - ratio);
      float angleChange = (lastKeyFrame.legsAngles[i - Joints::lHipYawPitch] - currentKeyFrame.legsAngles[i - Joints::lHipYawPitch])
          * ((1000.f / currentKeyFrame.duration) * theFrameInfo.cycleTime);
      if (angleChange > degPerFrame[i])
        if (textOutput == true)
          OUTPUT_TEXT("Angle increment for "
              << Joints::getName((Joints::Joint)i) << " (joint " << i << ") is greater than allowed in step " << currentKeyFrameIndex << " in KeyFrameMotion "
              << SpecialActionRequest::getName(keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID)); //
    }
  }

  // do not use playDead stiffnesses when transitioning to another motion
  if (currentKeyFrameMotion.keyFrameID != SpecialActionRequest::SpecialActionID::playDead || theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::SpecialActionID::playDead)
  {
    for (int i = 0; i < Joints::numOfJoints; i++)
    {
      if (!areMotionsPrioritized() && currentKeyFrame.stiffnesses[i] > 0 && currentKeyFrame.stiffnesses[i] <= 100)
        currentRequest.stiffnessData.stiffnesses[i] =
            std::min(currentRequest.stiffnessData.stiffnesses[i] + 10, std::max(specialActionsOutput.stiffnessData.stiffnesses[i] - 10, currentKeyFrame.stiffnesses[i]));
      else
        currentRequest.stiffnessData.stiffnesses[i] = currentKeyFrame.stiffnesses[i];
    }
  }

  gatherRequestDiff(specialActionsOutput, false);
  specialActionsOutput.angles = currentRequest.angles;
  specialActionsOutput.stiffnessData = currentRequest.stiffnessData;
  gatherRequestDiff(specialActionsOutput, true);

  // Use stabilization to correct interpolated values
  currentJointAngleID++;
  currentJointAngleID = currentJointAngleID % 5;
  for (int j = 0; j < Joints::numOfJoints; j++)
    jointAngleBuffer[currentJointAngleID].angles[j] = specialActionsOutput.angles[j];

  stabilize(specialActionsOutput);
  if (!currentKeyFrame.stabilize && keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID >= SpecialActionRequest::SpecialActionID::firstStandUpMotion
      && keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID <= SpecialActionRequest::SpecialActionID::lastStandUpMotion)
    compensateJointError(specialActionsOutput);

  // check if interpolation is done
  if (phase >= 1.f)
  {
    keyFrameWasFinished = true;
    keyFrameFinishedTimestamp = theFrameInfo.time;
  }
  return phase >= 1.f;
}

void KeyFrameEngine::gatherRequestDiff(SpecialActionsOutput& specialActionsOutput, bool afterInterpolation)
{
  if (!afterInterpolation)
  {
    jointRequestDiff[Joints::lHipYawPitch] = specialActionsOutput.angles[Joints::lHipYawPitch];
    jointRequestDiff[Joints::lHipPitch] = specialActionsOutput.angles[Joints::lHipPitch];
    jointRequestDiff[Joints::lKneePitch] = specialActionsOutput.angles[Joints::lKneePitch];
    jointRequestDiff[Joints::lAnklePitch] = specialActionsOutput.angles[Joints::lAnklePitch];

    jointRequestDiff[Joints::rHipYawPitch] = specialActionsOutput.angles[Joints::rHipYawPitch];
    jointRequestDiff[Joints::rHipPitch] = specialActionsOutput.angles[Joints::rHipPitch];
    jointRequestDiff[Joints::rKneePitch] = specialActionsOutput.angles[Joints::rKneePitch];
    jointRequestDiff[Joints::rAnklePitch] = specialActionsOutput.angles[Joints::rAnklePitch];
  }
  else
  {
    jointRequestDiff[Joints::lHipYawPitch] = std::abs(jointRequestDiff[Joints::lHipYawPitch] - specialActionsOutput.angles[Joints::lHipYawPitch]);
    jointRequestDiff[Joints::lHipPitch] = std::abs(jointRequestDiff[Joints::lHipPitch] - specialActionsOutput.angles[Joints::lHipPitch]);
    jointRequestDiff[Joints::lKneePitch] = std::abs(jointRequestDiff[Joints::lKneePitch] - specialActionsOutput.angles[Joints::lKneePitch]);
    jointRequestDiff[Joints::lAnklePitch] = std::abs(jointRequestDiff[Joints::lAnklePitch] - specialActionsOutput.angles[Joints::lAnklePitch]);

    jointRequestDiff[Joints::rHipYawPitch] = std::abs(jointRequestDiff[Joints::rHipYawPitch] - specialActionsOutput.angles[Joints::rHipYawPitch]);
    jointRequestDiff[Joints::rHipPitch] = std::abs(jointRequestDiff[Joints::rHipPitch] - specialActionsOutput.angles[Joints::rHipPitch]);
    jointRequestDiff[Joints::rKneePitch] = std::abs(jointRequestDiff[Joints::rKneePitch] - specialActionsOutput.angles[Joints::rKneePitch]);
    jointRequestDiff[Joints::rAnklePitch] = std::abs(jointRequestDiff[Joints::rAnklePitch] - specialActionsOutput.angles[Joints::rAnklePitch]);
  }
}

void KeyFrameEngine::compensateJointError(SpecialActionsOutput& specialActionsOutput)
{
  if (currentKeyFrameMotionIndex == SpecialActionRequest::none)
    return;

  if (!compensateHipError(specialActionsOutput))
  {
    compensateLegError(specialActionsOutput, true);
    compensateLegError(specialActionsOutput, false);
  }
}

bool KeyFrameEngine::compensateHipError(SpecialActionsOutput& specialActionsOutput)
{
  KeyFrameMotion& currentKeyFrameMotion = keyFrameMotions.at(currentKeyFrameMotionIndex);

  float reductionRate = currentKeyFrameMotion.reductionRate;
  std::vector<float> hipYawCorrectionFactors = currentKeyFrameMotion.hipYawCorrectionFactors;
  Angle maxCompensation = currentKeyFrameMotion.maxCompensation;
  Angle compensationThreshold = currentKeyFrameMotion.compensationThreshold;

  Angle lHipYawPitchDiff = theJointSensorData.angles[Joints::lHipYawPitch] - specialActionsOutput.angles[Joints::lHipYawPitch];
  Angle rHipYawPitchDiff = theJointSensorData.angles[Joints::rHipYawPitch] - specialActionsOutput.angles[Joints::rHipYawPitch];

  Angle currentError = 0_deg;
  if (std::abs(lHipYawPitchDiff) >= std::abs(rHipYawPitchDiff))
    currentError = lHipYawPitchDiff;
  else
    currentError = rHipYawPitchDiff;

  if (currentError > compensationThreshold + std::min(jointRequestDiff[Joints::lHipYawPitch], jointRequestDiff[Joints::rHipYawPitch]))
  {
    lCompensationReductionFactor = std::max(0.f, lCompensationReductionFactor - lCompensationReductionFactor * reductionRate);
    rCompensationReductionFactor = std::max(0.f, rCompensationReductionFactor - rCompensationReductionFactor * reductionRate);

    if (specialActionsOutput.lCompensatedID == Joints::lHipYawPitch || specialActionsOutput.rCompensatedID == Joints::rHipYawPitch)
    {
      currentError = Angle::fromDegrees(std::copysignf(currentError.toDegrees(), specialActionsOutput.lError.toDegrees())); // Is this nessesary?
      lJointErrorBuffer.push_front(currentError.toDegrees());
      rJointErrorBuffer.push_front(currentError.toDegrees());
      jointErrorBuffer = lJointErrorBuffer;
    }
    else
    {
      lJointErrorBuffer.fill(0.f);
      rJointErrorBuffer.fill(0.f);
      lJointErrorBuffer.push_front(currentError.toDegrees());
      rJointErrorBuffer.push_front(currentError.toDegrees());
      jointErrorBuffer = lJointErrorBuffer;
    }

    if (jointErrorBuffer.average() >= 0)
    {
      specialActionsOutput.angles[Joints::lHipPitch] += std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * hipYawCorrectionFactors[0] * lCompensationReductionFactor), maxCompensation);
      specialActionsOutput.angles[Joints::rHipPitch] += std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * hipYawCorrectionFactors[0] * rCompensationReductionFactor), maxCompensation);
    }
    else
    {
      specialActionsOutput.angles[Joints::lHipPitch] -= std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * hipYawCorrectionFactors[0] * lCompensationReductionFactor), maxCompensation);
      specialActionsOutput.angles[Joints::rHipPitch] -= std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * hipYawCorrectionFactors[0] * rCompensationReductionFactor), maxCompensation);
    }
    specialActionsOutput.angles[Joints::lHipRoll] += std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * hipYawCorrectionFactors[1] * lCompensationReductionFactor), maxCompensation);
    specialActionsOutput.angles[Joints::rHipRoll] -= std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * hipYawCorrectionFactors[1] * rCompensationReductionFactor), maxCompensation);
    specialActionsOutput.angles[Joints::lAnkleRoll] += std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * hipYawCorrectionFactors[2] * lCompensationReductionFactor), maxCompensation);
    specialActionsOutput.angles[Joints::rAnkleRoll] -= std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * hipYawCorrectionFactors[2] * rCompensationReductionFactor), maxCompensation);

    specialActionsOutput.lCompensatedID = Joints::lHipYawPitch;
    specialActionsOutput.lError = currentError;
    specialActionsOutput.rCompensatedID = Joints::rHipYawPitch;
    specialActionsOutput.rError = currentError;

    return true;
  }

  lCompensationReductionFactor = 1.f;
  rCompensationReductionFactor = 1.f;

  return false;
}

void KeyFrameEngine::compensateLegError(SpecialActionsOutput& specialActionsOutput, bool leftLeg)
{
  KeyFrameMotion& currentKeyFrameMotion = keyFrameMotions.at(currentKeyFrameMotionIndex);

  float errorMultiplier = currentKeyFrameMotion.errorMultiplier;
  float reductionRate = currentKeyFrameMotion.reductionRate;
  std::vector<float> hipCorrectionFactors = currentKeyFrameMotion.hipCorrectionFactors;
  std::vector<float> kneeCorrectionFactors = currentKeyFrameMotion.kneeCorrectionFactors;
  std::vector<float> ankleCorrectionFactors = currentKeyFrameMotion.ankleCorrectionFactors;
  Angle maxCompensation = currentKeyFrameMotion.maxCompensation;
  Angle compensationThreshold = currentKeyFrameMotion.compensationThreshold;

  std::vector<Joints::Joint> joints{Joints::rHipPitch, Joints::rKneePitch, Joints::rAnklePitch};
  unsigned int compensatedID = specialActionsOutput.rCompensatedID;
  Angle prevError = specialActionsOutput.rError;
  if (leftLeg)
  {
    joints = {Joints::lHipPitch, Joints::lKneePitch, Joints::lAnklePitch};
    compensatedID = specialActionsOutput.lCompensatedID;
    prevError = specialActionsOutput.lError;
  }

  Angle hipPitchDiff = theJointSensorData.angles[joints[0]] - specialActionsOutput.angles[joints[0]];
  Angle kneePitchDiff = theJointSensorData.angles[joints[1]] - specialActionsOutput.angles[joints[1]];
  Angle anklePitchDiff = theJointSensorData.angles[joints[2]] - specialActionsOutput.angles[joints[2]];
  std::vector<std::tuple<Angle, Joints::Joint>> leg{std::make_tuple(hipPitchDiff, joints[0]), std::make_tuple(kneePitchDiff, joints[1]), std::make_tuple(anklePitchDiff, joints[2])};
  std::sort(leg.begin(), leg.end(), sortJoints);

  Joints::Joint currentJoint = Joints::headYaw;
  Angle currentError = 0_deg;
  for (std::tuple<Angle, Joints::Joint> jointInfo : leg)
  {
    Angle error = std::get<0>(jointInfo);
    Joints::Joint joint = std::get<1>(jointInfo);

    Angle absError = std::abs(error);
    if (compensatedID != joint)
      absError = absError - std::abs(prevError) * errorMultiplier;

    if (absError > compensationThreshold + jointRequestDiff[joint])
    {
      currentJoint = joint;
      currentError = error;
      break;
    }
  }

  float currentReductionFactor = 1.f;
  if (leftLeg && compensatedID == currentJoint)
  {
    lCompensationReductionFactor = std::max(0.f, lCompensationReductionFactor - lCompensationReductionFactor * reductionRate);
    currentReductionFactor = lCompensationReductionFactor;
    currentError = Angle::fromDegrees(std::copysignf(currentError.toDegrees(), prevError.toDegrees())); // Is this nessesary?
    lJointErrorBuffer.push_front(currentError.toDegrees());
    jointErrorBuffer = lJointErrorBuffer;
  }
  else if (leftLeg)
  {
    lCompensationReductionFactor = 1.f;
    currentReductionFactor = lCompensationReductionFactor;
    lJointErrorBuffer.fill(0.f);
    lJointErrorBuffer.push_front(currentError.toDegrees());
    jointErrorBuffer = lJointErrorBuffer;
  }

  if (!leftLeg && compensatedID == currentJoint)
  {
    rCompensationReductionFactor = std::max(0.f, rCompensationReductionFactor - rCompensationReductionFactor * reductionRate);
    currentReductionFactor = rCompensationReductionFactor;
    currentError = Angle::fromDegrees(std::copysignf(currentError.toDegrees(), prevError.toDegrees())); // Is this nessesary?
    rJointErrorBuffer.push_front(currentError.toDegrees());
    jointErrorBuffer = rJointErrorBuffer;
  }
  else if (!leftLeg)
  {
    rCompensationReductionFactor = 1.f;
    currentReductionFactor = rCompensationReductionFactor;
    rJointErrorBuffer.fill(0.f);
    rJointErrorBuffer.push_front(currentError.toDegrees());
    jointErrorBuffer = rJointErrorBuffer;
  }


  if (currentJoint == joints[0])
  {
    if (jointErrorBuffer.average() >= 0)
    {
      specialActionsOutput.angles[joints[1]] -= std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * hipCorrectionFactors[0] * currentReductionFactor), maxCompensation);
      specialActionsOutput.angles[joints[2]] += std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * hipCorrectionFactors[1] * currentReductionFactor), maxCompensation);
    }
    else
    {
      specialActionsOutput.angles[joints[1]] += std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * hipCorrectionFactors[0] * currentReductionFactor), maxCompensation);
      specialActionsOutput.angles[joints[2]] -= std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * hipCorrectionFactors[1] * currentReductionFactor), maxCompensation);
    }
  }
  else if (currentJoint == joints[1])
  {
    if (jointErrorBuffer.average() >= 0)
    {
      specialActionsOutput.angles[joints[0]] -= std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * kneeCorrectionFactors[0] * currentReductionFactor), maxCompensation);
    }
    else
    {
      specialActionsOutput.angles[joints[0]] += std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * kneeCorrectionFactors[0] * currentReductionFactor), maxCompensation);
    }
  }
  else if (currentJoint == joints[2])
  {
    if (jointErrorBuffer.average() >= 0)
    {
      specialActionsOutput.angles[joints[0]] -= std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * ankleCorrectionFactors[0] * currentReductionFactor), maxCompensation);
      specialActionsOutput.angles[joints[1]] -= std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * ankleCorrectionFactors[1] * currentReductionFactor), maxCompensation);
    }
    else
    {
      specialActionsOutput.angles[joints[0]] += std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * ankleCorrectionFactors[0] * currentReductionFactor), maxCompensation);
      specialActionsOutput.angles[joints[1]] += std::min(Angle::fromDegrees(std::abs(jointErrorBuffer.average()) * ankleCorrectionFactors[1] * currentReductionFactor), maxCompensation);
    }
  }

  if (leftLeg)
  {
    specialActionsOutput.lCompensatedID = currentJoint;
    specialActionsOutput.lError = currentError;
  }
  else
  {
    specialActionsOutput.rCompensatedID = currentJoint;
    specialActionsOutput.rError = currentError;
  }
}

void KeyFrameEngine::stabilize(SpecialActionsOutput& specialActionsOutput)
{
  if (currentKeyFrameMotionIndex == SpecialActionRequest::none || !theFallDownState.mightUpright)
    return;

  static Angle lastErrorY = 0;
  static Angle errorSumY = 0;
  static Angle lastErrorX = 0;
  static Angle errorSumX = 0;

  Rangef angleErrorLimit(-5_deg, 5_deg);

  KeyFrameMotion& currentKeyFrameMotion = keyFrameMotions.at(currentKeyFrameMotionIndex);
  KeyFrameMotion::KeyFrame& currentKeyFrame = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex);

  if (!currentKeyFrame.stabilize)
  {
    lastErrorY = 0;
    errorSumY = 0;
    lastErrorX = 0;
    errorSumX = 0;
    return;
  }

  bool legAngleNotDefined = false;
  if (theFallDownState.mightUpright)
  {
    for (int i = 0; i < (int)currentKeyFrame.legsAngles.size(); i++)
    {
      if (currentKeyFrame.legsAngles[i] == 10000 || currentKeyFrame.legsAngles[i] == 20000)
      {
        legAngleNotDefined = true;
        break;
      }
    }
  }
  if (legAngleNotDefined && !stabilizeWarningActive)
  {
    OUTPUT_WARNING("Stabilization, for key frame index " + std::to_string(currentKeyFrameIndex) + " in key frame motion "
        + SpecialActionRequest::getName(currentKeyFrameMotion.keyFrameID) + ", is disabled because it is not safe to use!");
    stabilizeWarningActive = true;
    return;
  }
  else if (legAngleNotDefined)
  {
    return;
  }

  // set desired stance with joint request data from <sensorDelay> frames ago
  // newest data is not set yet, so use current request if sensorDelay == 0
  ASSERT(theWalkingEngineParams.jointSensorDelayFrames <= MAX_DELAY_FRAMES);
  RobotModel desiredStance;
  if (theWalkingEngineParams.jointSensorDelayFrames == 0)
    desiredStance.setJointData((JointAngles&)specialActionsOutput, theRobotDimensions, theMassCalibration);
  else
    desiredStance.setJointData(jointAngleBuffer[(currentJointAngleID + 5 - (theWalkingEngineParams.jointSensorDelayFrames - 1)) % 5], theRobotDimensions, theMassCalibration);

  // get desired gyro with request data from <sensorDelay - 1> frames ago
  RobotModel lastDesiredStance;
  lastDesiredStance.setJointData(jointAngleBuffer[(currentJointAngleID + 5 - (theWalkingEngineParams.jointSensorDelayFrames - 2)) % 5], theRobotDimensions, theMassCalibration);
  Angle desiredGyroY = desiredStance.limbs[Limbs::torso].rotation.getYAngle() - lastDesiredStance.limbs[Limbs::torso].rotation.getYAngle();

  //Y - using body angle and desired gyro for stabilization
  {
    Angle currentAngle = theJoinedIMUData.imuData[anglesource].angle.y();
    Angle error = desiredStance.limbs[Limbs::torso].rotation.getYAngle() - currentAngle;
    if (error > maxAngleDifference)
      error = maxAngleDifference;
    if (error < -maxAngleDifference)
      error = -maxAngleDifference;
    Angle controlledCorrection = currentKeyFrameMotion.YstabilizationP * error + currentKeyFrameMotion.YstabilizationI * errorSumY + currentKeyFrameMotion.YstabilizationD * (lastErrorY - error);
    PLOT("module:KeyFrameEngine:error", error.toDegrees());
    PLOT("module:KeyFrameEngine:desiredGyroY", desiredGyroY.toDegrees());
    Angle torsoY = desiredStance.limbs[Limbs::torso].rotation.getYAngle();
    PLOT("module:KeyFrameEngine:torsoY", torsoY.toDegrees());
    // TODO: verify if this makes sense
    if (theSensorControlParams.sensorControlActivation.activateSpeedReduction
        && ((error < theSensorControlParams.speedReduction.angleY.min) || (error > theSensorControlParams.speedReduction.angleY.max)))
    {
      if (currentAngle < theWalkingEngineParams.walkTransition.fallDownAngleFront && currentAngle > theWalkingEngineParams.walkTransition.fallDownAngleBack)
      {
        specialActionsOutput.angles[Joints::lHipPitch] += hipInfluence * controlledCorrection;
        specialActionsOutput.angles[Joints::rHipPitch] += hipInfluence * controlledCorrection;

        specialActionsOutput.angles[Joints::lKneePitch] += kneeInfluence * controlledCorrection;
        specialActionsOutput.angles[Joints::rKneePitch] += kneeInfluence * controlledCorrection;

        specialActionsOutput.angles[Joints::lAnklePitch] += ankleInfluence * controlledCorrection;
        specialActionsOutput.angles[Joints::rAnklePitch] += ankleInfluence * controlledCorrection;
      }
    }

    lastErrorY = error;
    errorSumY += error;
    errorSumY = angleErrorLimit.limit(errorSumY);
  }

  //X - using gyro and angle here for immediate stabilization
  {
    Angle currentAngle = theJoinedIMUData.imuData[anglesource].angle.x();
    float footZDiff = theRobotModel.soleLeft.translation.z() - theRobotModel.soleRight.translation.z();

    Angle desiredAngle = footZDiff > 0 ? desiredStance.limbs[Limbs::footLeft].rotation.getXAngle() : desiredStance.limbs[Limbs::footRight].rotation.getXAngle();
    Angle error = desiredAngle - currentAngle;
    if (error > maxAngleDifference)
      error = maxAngleDifference;
    if (error < -maxAngleDifference)
      error = -maxAngleDifference;
    Angle controlledCorrection = currentKeyFrameMotion.XstabilizationP * error + currentKeyFrameMotion.XstabilizationI * errorSumX + currentKeyFrameMotion.XstabilizationD * (lastErrorX - error);
    if (footZDiff < 5.f)
    {
      specialActionsOutput.angles[Joints::lAnkleRoll] += ankleRollInfluence * controlledCorrection;
      Angle ankleRollError = lAnkleRollBuffer[theWalkingEngineParams.jointSensorDelayFrames - 1] - specialActionsOutput.angles[Joints::lAnkleRoll];
      specialActionsOutput.angles[Joints::lHipRoll] += hipRollInfluence * controlledCorrection + ankleRollError;
    }
    if (footZDiff > -5.f)
    {
      specialActionsOutput.angles[Joints::rAnkleRoll] += ankleRollInfluence * controlledCorrection;
      Angle ankleRollError = rAnkleRollBuffer[theWalkingEngineParams.jointSensorDelayFrames - 1] - specialActionsOutput.angles[Joints::rAnkleRoll];
      specialActionsOutput.angles[Joints::rHipRoll] += hipRollInfluence * controlledCorrection + ankleRollError;
    }

    lastErrorX = error;
    errorSumX += error;
    errorSumX = angleErrorLimit.limit(errorSumX);
  }
}

bool KeyFrameEngine::isStable()
{
  return std::abs(gyroYBuffer.average()) < maxAvgGyroForStable && std::abs(theJoinedIMUData.imuData[anglesource].angle.x()) < maxAngleXForStable;
}

bool KeyFrameEngine::checkCondition(KeyFrameMotion::KeyFrame::Conditions condition)
{ //toDo: check all possible conditions
  switch (condition)
  {
  case KeyFrameMotion::KeyFrame::none:
    return false;
    break;
  case KeyFrameMotion::KeyFrame::execute:
    return true;
    break;
  case KeyFrameMotion::KeyFrame::lyingOnFront:
    if ((theFallDownState.state == FallDownState::onGround || theFallDownState.state == FallDownState::onGroundLyingStill) && theFallDownState.direction == FallDownState::front)
    {
      return true;
    }
    else
    {
      return false;
    }
    break;
  case KeyFrameMotion::KeyFrame::lyingOnBack:
    if ((theFallDownState.state == FallDownState::onGround || theFallDownState.state == FallDownState::onGroundLyingStill) && theFallDownState.direction == FallDownState::back)
    {
      return true;
    }
    else
    {
      return false;
    }
    break;
  case KeyFrameMotion::KeyFrame::lyingOnSide:
    if ((theFallDownState.state == FallDownState::onGround || theFallDownState.state == FallDownState::onGroundLyingStill)
        && (theFallDownState.direction == FallDownState::left || theFallDownState.direction == FallDownState::right))
    {
      return true;
    }
    else
    {
      return false;
    }
    break;
  case KeyFrameMotion::KeyFrame::notOnGround:
    return theFallDownState.notOnGround; // no ground contact
    break;
  case KeyFrameMotion::KeyFrame::mightUpright:
    return theFallDownState.mightUpright;
    break;
  case KeyFrameMotion::KeyFrame::notLying:
    return theFallDownState.notLying;
    break;
  case KeyFrameMotion::KeyFrame::useStandUpStatistic:
    return false;
    break;
  default:
    return false;
    break;
  }
}

SpecialActionRequest::SpecialActionID KeyFrameEngine::selectNextMotionID(std::vector<SpecialActionRequest::SpecialActionID> nextMotionIDs, std::vector<KeyFrameMotion::KeyFrame::Conditions> conditions)
{
  // Assert length should be equal
  unsigned int idx = 0;
  SpecialActionRequest::SpecialActionID finalMotionID = SpecialActionRequest::none;
  float bestStandUpRatio = 1;
  unsigned int useStandUpStatisticCount = 0;
  unsigned int standUpTriesDone = 0;

  unsigned int localFrontBackIdx = 0;
  if (lastStandUpStartMotion == SpecialActionRequest::standUpFront)
    localFrontBackIdx = 0;
  else if (lastStandUpStartMotion == SpecialActionRequest::standUpBack || lastStandUpStartMotion == SpecialActionRequest::sit)
    localFrontBackIdx = 1;

  //std::vector<SpecialActionRequest::SpecialActionID> possibleStandUpIDs;
  std::priority_queue<std::tuple<int, SpecialActionRequest::SpecialActionID>, std::vector<std::tuple<int, SpecialActionRequest::SpecialActionID>>, std::greater<std::tuple<int, SpecialActionRequest::SpecialActionID>>> possibleStandUpIDs;
  for (KeyFrameMotion::KeyFrame::Conditions condition : conditions)
  {
    if (condition == KeyFrameMotion::KeyFrame::Conditions::useStandUpStatistic)
    {
      if (standUpStatistic[localFrontBackIdx][nextMotionIDs[idx] - SpecialActionRequest::firstStandUpMotion] < standUpPossibleThreshold
          && standUpStatistic[localFrontBackIdx][nextMotionIDs[idx] - SpecialActionRequest::firstStandUpMotion] < bestStandUpRatio)
      {
        bestStandUpRatio = standUpStatistic[localFrontBackIdx][nextMotionIDs[idx] - SpecialActionRequest::firstStandUpMotion];
        finalMotionID = nextMotionIDs[idx];
      }

      if (countStandUpStarts[localFrontBackIdx][nextMotionIDs[idx] - SpecialActionRequest::firstStandUpMotion] >= minStandUpTries + 1)
        standUpTriesDone++;

      useStandUpStatisticCount++;
      possibleStandUpIDs.push(std::make_tuple(countStandUpStarts[localFrontBackIdx][nextMotionIDs[idx] - SpecialActionRequest::firstStandUpMotion], nextMotionIDs[idx]));
    }
    idx++;
  }

  if (standUpTriesDone < useStandUpStatisticCount)
    finalMotionID = std::get<1>(possibleStandUpIDs.top());
  else if (possibleStandUpIDs.size() > 0 && finalMotionID == SpecialActionRequest::none)
    standUpNotWorking = true;

  if (finalMotionID == SpecialActionRequest::none)
  {
    idx = 0;
    for (KeyFrameMotion::KeyFrame::Conditions condition : conditions)
    {
      if (checkCondition(condition))
      {
        if (nextMotionIDs[idx] == SpecialActionRequest::rip)
          standUpNotWorking = true;
        return nextMotionIDs[idx];
      }
      idx++;
    }
  }

  return finalMotionID;
}

KeyFrameEngine::ArmsStuckState KeyFrameEngine::armsStuck()
{
  KeyFrameMotion& currentKeyFrameMotion = keyFrameMotions.at(currentKeyFrameMotionIndex);
  KeyFrameMotion::KeyFrame currentKeyFrame = currentKeyFrameMotion.keyFrames.at(currentKeyFrameIndex);
  if (!currentKeyFrame.useArmProblemDetection[0] && !currentKeyFrame.useArmProblemDetection[1])
    return ArmsStuckState::fine;

  //lying on arms
  if (currentKeyFrame.useArmProblemDetection[1] && (theFallDownState.state == FallDownState::onGround || theFallDownState.state == FallDownState::onGroundLyingStill)
      && theFallDownState.direction == FallDownState::front
      && (std::abs(theJointError.angles[Joints::lElbowRoll]) > maxElbowError || std::abs(theJointError.angles[Joints::rElbowRoll]) > maxElbowError
          || std::abs(theJointError.angles[Joints::lElbowYaw]) > maxElbowError || std::abs(theJointError.angles[Joints::rElbowYaw]) > maxElbowError))
  {
    return ArmsStuckState::lyingOnArm;
  }

  //stuck arms
  if (currentKeyFrame.useArmProblemDetection[0] && (theFallDownState.state == FallDownState::onGround || theFallDownState.state == FallDownState::onGroundLyingStill) && !areMotionsPrioritized()
      && (std::abs(theJointError.angles[Joints::lShoulderRoll]) > maxShoulderRollError || std::abs(theJointError.angles[Joints::rShoulderRoll]) > maxShoulderRollError))
  {
    return ArmsStuckState::stuck;
  }
  else if (currentKeyFrame.useArmProblemDetection[0] && (theFallDownState.state == FallDownState::onGround || theFallDownState.state == FallDownState::onGroundLyingStill)
      && !areMotionsPrioritized() && theFallDownState.direction == FallDownState::back
      && (std::abs(theJointError.angles[Joints::lShoulderPitch]) > maxShoulderPitchError || std::abs(theJointError.angles[Joints::rShoulderPitch]) > maxShoulderPitchError))
  {
    return ArmsStuckState::stuck;
  }

  return ArmsStuckState::fine;
}

bool KeyFrameEngine::armAngleReached(SpecialActionsOutput& specialActionsOutput)
{
  KeyFrameMotion& currentKeyFrameMotion = keyFrameMotions.at(currentKeyFrameMotionIndex);
  if (std::abs(theJointSensorData.angles[Joints::lShoulderPitch] - specialActionsOutput.angles[Joints::lShoulderPitch]) < currentKeyFrameMotion.localArmAngleReachedThreshold
      && std::abs(theJointSensorData.angles[Joints::rShoulderPitch] - specialActionsOutput.angles[Joints::rShoulderPitch]) < currentKeyFrameMotion.localArmAngleReachedThreshold)
    return true;

  return false;
}

std::vector<KeyFrameMotion> KeyFrameEngine::loadKeyFrameMotions()
{
  std::vector<KeyFrameMotion> keyFrameMotions;
  for (int i = 0; i < SpecialActionRequest::numOfSpecialActionIDs; i++)
  {
    if ((SpecialActionRequest::SpecialActionID)i == SpecialActionRequest::none)
      continue;
    std::string name = SpecialActionRequest::getName((SpecialActionRequest::SpecialActionID)i);
    std::string path = std::string(File::getBHDir()) + "/Config/KeyFrameEngine/" + name + ".kfm";
    InMapFile stream(path);
    if (stream.exists())
    {
      KeyFrameMotion kfm;
      stream >> kfm;
      // Forces correct keyFrameID. Otherwise, e.g., malformed files are set to playDead by default.
      kfm.keyFrameID = static_cast<SpecialActionRequest::SpecialActionID>(i);
      keyFrameMotions.push_back(std::move(kfm));
    }
    else
    {
      OUTPUT_WARNING("KeyFrameEngine: missing ID " << name << " on load");
    }
  }
  return keyFrameMotions;
}

void KeyFrameEngine::selectActiveMotion(SpecialActionRequest::SpecialActionID id, bool mirror, SpecialActionsOutput& specialActionsOutput)
{
  if (id == SpecialActionRequest::none && initialKeyFrameMotionID != SpecialActionRequest::none)
    id = initialKeyFrameMotionID;
  const SpecialActionRequest::SpecialActionID defaultMotion = SpecialActionRequest::playDead;
  bool foundMotion = false;
  for (int i = 0; i < (int)keyFrameMotions.size(); i++)
    if (id == keyFrameMotions.at(i).keyFrameID)
    {
      currentKeyFrameMotionIndex = i;
      foundMotion = true;
      break;
    }
  if (foundMotion)
    specialActionsOutput.executedSpecialAction.specialAction = id;
  else if (id == defaultMotion)
    specialActionsOutput.executedSpecialAction.specialAction = keyFrameMotions.at(0).keyFrameID; // we should at least have one motion loaded..
  else
    selectActiveMotion(defaultMotion, mirror, specialActionsOutput);
  specialActionsOutput.executedSpecialAction.mirror = mirror;
  specialActionsOutput.isMotionFinished = false;
  currentKeyFrameIndex = 0;
  currentKeyFrameMirror = mirror;
  prevArmsStuck = false;
  standTransitionActive = false;
  stabilizeWarningActive = false;
}

bool KeyFrameEngine::areMotionsPrioritized()
{
  return !specialActionIDQueue.empty() && !specialActionMirrorQueue.empty();
}

void KeyFrameEngine::priorizeMotion(SpecialActionRequest::SpecialActionID id, bool mirror, unsigned timer)
{
  specialActionIDQueue.push(id);
  specialActionMirrorQueue.push(mirror);
  specialActionTimerQueue.push(timer);
}

void KeyFrameEngine::clearPrioritizedMotions()
{
  while (!specialActionIDQueue.empty())
    specialActionIDQueue.pop();
  while (!specialActionMirrorQueue.empty())
    specialActionMirrorQueue.pop();
  while (!specialActionTimerQueue.empty())
    specialActionTimerQueue.pop();
}

void KeyFrameEngine::selectPrioritizedMotion(SpecialActionsOutput& specialActionsOutput)
{
  if (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID != specialActionIDQueue.front())
  {
    initEngineData(specialActionsOutput);
    selectActiveMotion(specialActionIDQueue.front(), specialActionMirrorQueue.front(), specialActionsOutput);
    phase = 0.f;
  }

  if (specialActionsOutput.isMotionFinished && theFrameInfo.getTimeSince(keyFrameMotionFinishedTimestamp) >= (int)specialActionTimerQueue.front())
  {
    specialActionIDQueue.pop();
    specialActionMirrorQueue.pop();
    specialActionTimerQueue.pop();
  }
}

KeyFrameMotion::KeyFrame KeyFrameEngine::setKeyFrameAngles(const KeyFrameMotion::KeyFrame& keyFrame)
{
  KeyFrameMotion::KeyFrame result;
  for (int i = 0; i < 2; i++)
    if (keyFrame.headAngles[i] == JointAngles::off || keyFrame.headAngles[i] == JointAngles::ignore)
      result.headAngles[i] = theJointSensorData.angles[i];
    else
      result.headAngles[i] = keyFrame.headAngles[i];
  for (int i = 0; i < 12; i++)
    if (keyFrame.armsAngles[i] == JointAngles::off || keyFrame.armsAngles[i] == JointAngles::ignore)
      result.armsAngles[i] = theJointSensorData.angles[i + 2];
    else
      result.armsAngles[i] = keyFrame.armsAngles[i];
  for (int i = 0; i < 12; i++)
    if (keyFrame.legsAngles[i] == JointAngles::off || keyFrame.legsAngles[i] == JointAngles::ignore)
      result.legsAngles[i] = theJointSensorData.angles[i + 14];
    else
      result.legsAngles[i] = keyFrame.legsAngles[i];
  return result;
}

bool KeyFrameEngine::standUpMotionActive()
{
  return keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID >= SpecialActionRequest::firstStandUpMotion
      && keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID <= SpecialActionRequest::lastStandUpMotion
      && keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID != SpecialActionRequest::lying && keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID != SpecialActionRequest::freeArmsFront
      && keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID != SpecialActionRequest::freeArmsBack
      && keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID != SpecialActionRequest::untangleArms;
}

bool KeyFrameEngine::inStandUpMotion()
{
  return keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID >= SpecialActionRequest::firstStandUpMotion
      && keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID <= SpecialActionRequest::lastStandUpMotion;
}

void KeyFrameEngine::calculateStandUpStatistic(SpecialActionsOutput& specialActionsOutput)
{
  if (theFallDownState.state == FallDownState::upright)
  {
    standUpStarted = false;
  }
  if (lastStandUpStartMotion == SpecialActionRequest::standUpFront)
    globalFrontBackIdx = 0;
  else if (lastStandUpStartMotion == SpecialActionRequest::standUpBack)
    globalFrontBackIdx = 1;


  if (standUpStarted && (theFallDownState.state == FallDownState::flying || theRobotInfo.penalty != PENALTY_NONE))
  {
    if (countStandUpStarts[globalFrontBackIdx][keyFrameMotions.at(lastStandUpMotionIndex).keyFrameID - SpecialActionRequest::firstStandUpMotion] > 1)
      countStandUpStarts[globalFrontBackIdx][keyFrameMotions.at(lastStandUpMotionIndex).keyFrameID - SpecialActionRequest::firstStandUpMotion]--;
    standUpStarted = false;
  }

  if (standUpStarted && (specialActionsOutput.isFallProtectionNeeded || keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::lying))
  {
    countStandUpAborts[globalFrontBackIdx][keyFrameMotions.at(lastStandUpMotionIndex).keyFrameID - SpecialActionRequest::firstStandUpMotion]++;
    standUpStarted = false;
    standUpStatistic[globalFrontBackIdx][keyFrameMotions.at(lastStandUpMotionIndex).keyFrameID - SpecialActionRequest::firstStandUpMotion] =
        (float)countStandUpAborts[globalFrontBackIdx][keyFrameMotions.at(lastStandUpMotionIndex).keyFrameID - SpecialActionRequest::firstStandUpMotion]
        / (float)countStandUpStarts[globalFrontBackIdx][keyFrameMotions.at(lastStandUpMotionIndex).keyFrameID - SpecialActionRequest::firstStandUpMotion];
  }

  if (standUpMotionActive() && currentKeyFrameMotionIndex != lastStandUpMotionIndex)
  {
    countStandUpStarts[globalFrontBackIdx][keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID - SpecialActionRequest::firstStandUpMotion]++;
    lastStandUpMotionIndex = currentKeyFrameMotionIndex;
    standUpStarted = true;
  }

  if (lastStandUpStartMotion == SpecialActionRequest::sit)
    globalFrontBackIdx = 1;

  specialActionsOutput.standUpStatisticFront = std::vector<float>(standUpStatistic[0].begin(), standUpStatistic[0].end());
  specialActionsOutput.standUpStatisticBack = std::vector<float>(standUpStatistic[1].begin(), standUpStatistic[1].end());
}

bool KeyFrameEngine::verifyRobotPosition(SpecialActionsOutput& specialActionsOutput)
{
  //KeyFrameMotion& currentKeyFrameMotion = keyFrameMotions.at(currentKeyFrameMotionIndex);
  KeyFrameMotion::KeyFrame& currentKeyFrame = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex);

  Angle currentAngle = theJoinedIMUData.imuData[anglesource].angle.y();
  // check if sensor y angle of upper body is too different from intended angle for this key frame
  bool useAngleAtKeyFrameTarget = currentKeyFrame.angleAtKeyFrameError[0] >= 0 && currentKeyFrame.angleAtKeyFrameError[1] >= 0;

  if (useAngleAtKeyFrameTarget)
  {
    if ((currentAngle - currentKeyFrame.angleAtKeyFrameTarget) < 0)
    {
      if (std::abs(currentAngle - currentKeyFrame.angleAtKeyFrameTarget) > currentKeyFrame.angleAtKeyFrameError[1])
        return false;
    }
    else
    {
      if (std::abs(currentAngle - currentKeyFrame.angleAtKeyFrameTarget) > currentKeyFrame.angleAtKeyFrameError[0])
        return false;
    }
  }
  Angle maxArmAngleDiff = 0_deg;
  Angle maxLegAngleDiff = 0_deg;
  for (int i = Joints::firstArmJoint; i < Joints::lHipYawPitch; i++)
    maxArmAngleDiff = std::max<Angle>(maxArmAngleDiff, std::abs(specialActionsOutput.angles[i] - theJointSensorData.angles[i]));
  for (int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
    maxLegAngleDiff = std::max<Angle>(maxLegAngleDiff, std::abs(specialActionsOutput.angles[i] - theJointSensorData.angles[i]));
  return true;
}

bool KeyFrameEngine::isMotionStable(SpecialActionsOutput& specialActionsOutput)
{
  return keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID != SpecialActionRequest::playDead
      && (specialActionsOutput.isLeavingPossible || keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::sitDown
          || keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::penaltyGoaliePrepareDive)
      && isStable();
  ;
}

void KeyFrameEngine::handleTransitions(SpecialActionsOutput& specialActionsOutput)
{
  if (standUpNotWorking && theGameInfo.inPreGame())
    standUpNotWorking = false;

  // Correction of the stand-up pipeline to set the correct stand-up routine if the wrong one was queued
  if (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::standUpFront)
  {
    if (theFallDownState.direction == FallDownState::back)
    {
      ignoreInitialKeyFrame = true;
      initialKeyFrameMotionID = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID;
      selectActiveMotion(SpecialActionRequest::standUpBack, false, specialActionsOutput);
    }
    else if (theFallDownState.direction == FallDownState::left || theFallDownState.direction == FallDownState::right)
    {
      ignoreInitialKeyFrame = true;
      initialKeyFrameMotionID = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID;
      selectActiveMotion(SpecialActionRequest::standUpSide, false, specialActionsOutput);
    }
  }
  if (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::standUpBack)
  {
    if (theFallDownState.direction == FallDownState::front)
    {
      ignoreInitialKeyFrame = true;
      initialKeyFrameMotionID = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID;
      selectActiveMotion(SpecialActionRequest::standUpFront, false, specialActionsOutput);
    }
    else if (theFallDownState.direction == FallDownState::left || theFallDownState.direction == FallDownState::right)
    {
      ignoreInitialKeyFrame = true;
      initialKeyFrameMotionID = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID;
      selectActiveMotion(SpecialActionRequest::standUpSide, false, specialActionsOutput);
    }
  }
  if (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::standUpSide)
  {
    if (!standUpSideLegit && theFallDownState.direction == FallDownState::front)
    {
      ignoreInitialKeyFrame = true;
      initialKeyFrameMotionID = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID;
      selectActiveMotion(SpecialActionRequest::standUpFront, false, specialActionsOutput);
    }
    else if (!standUpSideLegit && theFallDownState.direction == FallDownState::back)
    {
      ignoreInitialKeyFrame = true;
      initialKeyFrameMotionID = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID;
      selectActiveMotion(SpecialActionRequest::standUpBack, false, specialActionsOutput);
    }
    else
      standUpSideLegit = true;
  }
  else
  {
    standUpSideLegit = false;
  }

  // Restart stand-up after altering the stand-up motion and lying on the ground
  if (SpecialActionRequest::firstStandUpMotion <= initialKeyFrameMotionID && initialKeyFrameMotionID <= SpecialActionRequest::lastStandUpMotion
      && (theFallDownState.state == FallDownState::onGround || theFallDownState.state == FallDownState::onGroundLyingStill) && !areMotionsPrioritized() && specialActionsOutput.isMotionFinished)
  {
    if (theFallDownState.direction == FallDownState::back)
    {
      initEngineData(specialActionsOutput);
      ignoreInitialKeyFrame = true;
      selectActiveMotion(SpecialActionRequest::standUpBack, false, specialActionsOutput);
      phase = 0.f;
    }
    else if (theFallDownState.direction == FallDownState::front)
    {
      initEngineData(specialActionsOutput);
      ignoreInitialKeyFrame = true;
      selectActiveMotion(SpecialActionRequest::standUpFront, false, specialActionsOutput);
      phase = 0.f;
    }
    else if (theFallDownState.direction == FallDownState::left || theFallDownState.direction == FallDownState::right)
    {
      initEngineData(specialActionsOutput);
      ignoreInitialKeyFrame = true;
      selectActiveMotion(SpecialActionRequest::standUpSide, false, specialActionsOutput);
      phase = 0.f;
    }
  }

  // Track the initial stand-up motion
  if (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::standUpBack)
  {
    lastStandUpStartMotion = SpecialActionRequest::standUpBack;
  }
  if (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::standUpFront)
  {
    lastStandUpStartMotion = SpecialActionRequest::standUpFront;
  }
  if (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::sit)
  {
    lastStandUpStartMotion = SpecialActionRequest::sit;
  }

  // Change fall down protection to a better suited version for this situation
  if (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::sit)
  {
    if (theFallDownState.direction == FallDownState::front && theJoinedIMUData.imuData[anglesource].angle.y() > sitAbortThreshold)
    {
      clearPrioritizedMotions();
      priorizeMotion(SpecialActionRequest::playDead, false, saveFallTime);
      priorizeMotion(SpecialActionRequest::lying, false, lyingTime);
    }
  }

  // Clear fall down protection queue if the robot is in upright position
  if (theFallDownState.mightUpright && theFrameInfo.getTimeSince(keyFrameFinishedTimestamp) > mightUprightTime)
    clearPrioritizedMotions();

  // Recalculate the current stand-up statistics
  calculateStandUpStatistic(specialActionsOutput);

  // Set stand if the robot should start a stand-up routine but is in upright position
  if (theFallDownState.mightUpright
      && (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::standUpBack || keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::standUpFront
          || keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::standUpSide))
  {
    initialKeyFrameMotionID = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID;
    ignoreInitialKeyFrame = true;

    initEngineData(specialActionsOutput);
    selectActiveMotion(SpecialActionRequest::stand, false, specialActionsOutput);
    phase = 0.f;

    specialActionsOutput.isFallProtectionNeeded = false;
  }

  // Enqueue stand as selected motion in initial and if the motion selector use a blank special action request
  if (!prevFlying && theMotionRequest.motion != MotionRequest::specialAction
      && (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::sitDown || keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::playDead)
      && theFallDownState.state == FallDownState::upright)
  {
    initEngineData(specialActionsOutput);
    selectActiveMotion(SpecialActionRequest::stand, false, specialActionsOutput);
    standTransitionActive = true;

    if (specialActionsOutput.isFallProtectionNeeded && keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID != SpecialActionRequest::sitDown)
      phase = 0.f;
    else
      phase = 1.f;

    if (theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::sitDown)
      initialKeyFrameMotionID = SpecialActionRequest::sitDown;
    if (theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead)
      initialKeyFrameMotionID = SpecialActionRequest::playDead;
    if (!theGameInfo.inPreGame())
      initialKeyFrameMotionID = SpecialActionRequest::stand;
  }

  // Set stand as selected motion or leave sit down as it is in flying
  if (!prevFlying && theFallDownState.state == FallDownState::flying)
  {
    clearPrioritizedMotions();

    if (initialKeyFrameMotionID != SpecialActionRequest::sitDown && initialKeyFrameMotionID != SpecialActionRequest::penaltyGoaliePrepareDive)
    {
      initEngineData(specialActionsOutput);
      selectActiveMotion(SpecialActionRequest::stand, false, specialActionsOutput);
    }

    specialActionsOutput.isFallProtectionNeeded = false;
    prevFlying = true;
    phase = 1.f;
  }

  // Correct prevFlying state if falling is detected
  if (prevFlying && theFallDownState.state == FallDownState::falling)
    prevFlying = false;

  // Set lying as selected motion after fall down protection
  if (resetFallDownProtectionNeeded(specialActionsOutput))
    specialActionsOutput.isFallProtectionNeeded = false;

  // Set the initial key frame motion id
  if (keyFrameMotions.at(currentKeyFrameMotionIndex).initialKeyFrame && !areMotionsPrioritized() && !standTransitionActive && !ignoreInitialKeyFrame)
    initialKeyFrameMotionID = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID;

  // Override active motion with priorized motion
  if (areMotionsPrioritized())
    selectPrioritizedMotion(specialActionsOutput);
}

bool KeyFrameEngine::resetFallDownProtectionNeeded(SpecialActionsOutput& specialActionsOutput)
{
  return specialActionsOutput.isFallProtectionNeeded && (theFallDownState.state == FallDownState::onGround || theFallDownState.state == FallDownState::onGroundLyingStill);
}

void KeyFrameEngine::handleInitialFallDownProtection(SpecialActionsOutput& specialActionsOutput)
{
  if (!theMotionRequest.inStandUpMotion() && !initialFallDownProtectionActive && theFallDownState.state == FallDownState::falling)
  {
    chooseFallDownProtection(specialActionsOutput, SpecialActionRequest::none, 0);
    initialFallDownProtectionActive = true;
  }
  else if (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::flying)
  {
    initialFallDownProtectionActive = false;
    clearPrioritizedMotions();
  }
}

void KeyFrameEngine::chooseFallDownProtection(SpecialActionsOutput& specialActionsOutput, SpecialActionRequest::SpecialActionID id, unsigned timer)
{
  if (areMotionsPrioritized() || keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::sitDown)
    return;

  specialActionsOutput.isFallProtectionNeeded = true;
  specialActionsOutput.isMotionFinished = true;
  if (theFallDownState.state == FallDownState::falling && id == SpecialActionRequest::none)
  {
    if (theFallDownState.direction == FallDownState::front)
      switch (theFallDownState.tilt)
      {
      case FallDownState::frontLeft:
        priorizeMotion(SpecialActionRequest::saveFallFront, true, saveFallFrontTime);
        break;
      case FallDownState::frontRight:
        priorizeMotion(SpecialActionRequest::saveFallFront, false, saveFallFrontTime);
        break;
      default:
        priorizeMotion(SpecialActionRequest::saveFallFront, std::rand() % 2 > 0, saveFallFrontTime);
      }
    else if (theFallDownState.direction == FallDownState::back)
      priorizeMotion(SpecialActionRequest::saveFallBack, false, saveFallBackTime);
    else
      priorizeMotion(SpecialActionRequest::saveFall, false, saveFallTime);

    priorizeMotion(SpecialActionRequest::lying, false, lyingTime);
  }
  else if (id != SpecialActionRequest::none)
  {
    priorizeMotion(id, false, timer);
  }
  else
  {
    //KeyFrameMotion& currentKeyFrameMotion = keyFrameMotions.at(currentKeyFrameMotionIndex);
    KeyFrameMotion::KeyFrame& currentKeyFrame = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex);

    Angle currentAngle = theJoinedIMUData.imuData[anglesource].angle.y();
    // check if sensor y angle of upper body is too different from intended angle for this key frame
    bool useAngleAtKeyFrameTarget = currentKeyFrame.angleAtKeyFrameError[0] >= 0 && currentKeyFrame.angleAtKeyFrameError[1] >= 0;
    // if in stand up motion and falling back
    if (useAngleAtKeyFrameTarget && inStandUpMotion() && (currentAngle - currentKeyFrame.angleAtKeyFrameTarget) < 0
        && std::abs(currentAngle - currentKeyFrame.angleAtKeyFrameTarget) > currentKeyFrame.angleAtKeyFrameError[1] && theFallDownState.notLying)
    {
      priorizeMotion(SpecialActionRequest::sit, false, saveFallBackTime);
    }
    else if (inStandUpMotion())
    {
      specialActionsOutput.isFallProtectionNeeded = true;
      specialActionsOutput.isMotionFinished = true;
      priorizeMotion(SpecialActionRequest::playDead, false, saveFallTime);
      priorizeMotion(SpecialActionRequest::lying, false, lyingTime);
    }
  }
}

KeyFrameMotion::KeyFrame KeyFrameEngine::getKeyFrameFromJointRequest(const JointRequest& jointRequest)
{
  KeyFrameMotion::KeyFrame kf;

  kf.stiffnesses = jointRequest.stiffnessData.stiffnesses;
  for (int i = 0; i < Joints::numOfJoints; i++)
  {
    if (i < Joints::firstArmJoint)
      kf.headAngles[i] = jointRequest.angles[i];
    else if (i < Joints::lHipYawPitch)
      kf.armsAngles[i - Joints::firstArmJoint] = jointRequest.angles[i];
    else
      kf.legsAngles[i - Joints::lHipYawPitch] = jointRequest.angles[i];
  }

  return kf;
}

bool KeyFrameEngine::handleMessage(InMessage& message)
{
  return *theInstance && (*theInstance)->handleMessage2(message);
}

bool KeyFrameEngine::handleMessage2(InMessage& message)
{
  if (message.getMessageID() == idKeyFrameMotions)
  {
    STREAM_EXT(message.bin, keyFrameMotions);
    return true;
  }
  else
    return false;
}
