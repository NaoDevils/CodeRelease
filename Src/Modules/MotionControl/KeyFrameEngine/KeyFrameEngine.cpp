/**
 * @file KeyFrameEngine.cpp
 * This file declares a module that handles key frame motions.
 * @author <a href="ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
 * @author Diana Kleingarn
 * @author Dominik Brämer
 */

#include "KeyFrameEngine.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Settings.h"
#include "Platform/File.h"
#include <sstream>
#include <iomanip>

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
}

void KeyFrameEngine::init(SpecialActionsOutput& specialActionsOutput)
{
  if (!initialized)
  {
    keyFrameMotions = loadKeyFrameMotions();
    initEngineData(specialActionsOutput);
  }
  // set to current joint angles and default stiffness
  // TODO: what if angles are illegal/undefined? can that even happen?


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

  // update sensor/request buffers
  angleYBuffer.push_front(theJoinedIMUData.imuData[anglesource].angle.y());
  gyroYBuffer.push_front(theJoinedIMUData.imuData[anglesource].gyro.y());
  gyroXBuffer.push_front(theJoinedIMUData.imuData[anglesource].gyro.x());
  hipYawBuffer.push_front(specialActionsOutput.angles[Joints::lHipYawPitch]);
  lAnkleRollBuffer.push_front(specialActionsOutput.angles[Joints::lAnkleRoll]);
  rAnkleRollBuffer.push_front(specialActionsOutput.angles[Joints::rAnkleRoll]);

  handleInitialFallDownProtection();

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
      prevArmsStuck = false;

    // interpolate returns true if phase >= 1.f, i.e. single keyFrame was executed
    if (interpolate(specialActionsOutput) && (!debugMode || playNextFrameInDebugMode))
    {
      // check if robots joints and upper body are ok after interpolation, otherwise cancel motion
      if (!verifyRobotPosition(specialActionsOutput) && !debugModeForWalkingCalibration)
      {
        // handles the fall down protection between two key frames
        // TODO implement fall down protection while interpolating
        KeyFrameMotion::KeyFrame& currentKeyFrame = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex);
        lastFinishedKeyFrameIndex = currentKeyFrameIndex;
        lastFinishedKeyFrameMotionIndex = currentKeyFrameMotionIndex;

        specialActionsOutput.isFallProtectionNeeded = true;
        specialActionsOutput.isMotionFinished = true;
        chooseFallDownProtection(currentKeyFrame.fallDownProtection, currentKeyFrame.holdFallDownProtection, currentKeyFrame.activateFallDownProtection);
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

      // Pauses keyframe if robot gets disturbed
      if (armsStuck() && !prevArmsStuck && !debugModeForWalkingCalibration && useStuckArmsDetection)
      {
        prevArmsStuck = true;
        return;
      }

      // Set current frame as originFrame for possible transition to next motion!
      originFrame = getKeyFrameFromJointRequest(specialActionsOutput);

      phase = 0.f; // reset phase
      currentKeyFrameIndex++; // next keyFrame
      keyFrameFinishedTimestamp = theFrameInfo.time;
      // check if we are finished with the motion. if yes we can leave or run the next or repeat the last output forever
      if (currentKeyFrameIndex >= static_cast<int>(keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.size()))
      {
        if (!areMotionsPrioritized() && keyFrameMotions.at(currentKeyFrameMotionIndex).nextKeyFrameID != SpecialActionRequest::none)
        {
          selectActiveMotion(keyFrameMotions.at(currentKeyFrameMotionIndex).nextKeyFrameID, false, specialActionsOutput);
        }
        else if (!areMotionsPrioritized() && initialKeyFrameMotionID != theMotionSelection.specialActionRequest.specialAction && theMotionSelection.targetMotion == MotionRequest::specialAction) // next motion is selected
        {
          selectActiveMotion(theMotionSelection.specialActionRequest.specialAction, theMotionSelection.specialActionRequest.mirror, specialActionsOutput);
        }
        else // same motion, repeat the finished interpolate with phase = 1.f
        {
          if (!specialActionsOutput.isMotionFinished)
            keyFrameMotionFinishedTimestamp = theFrameInfo.time;
          currentKeyFrameIndex = (int)keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.size() - 1;
          specialActionsOutput.isMotionFinished = true;
          phase = 1.f;
          stabilize(specialActionsOutput);
        }
      }
    }

    engineDataReset = false;
  }
}

bool KeyFrameEngine::interpolate(SpecialActionsOutput& specialActionsOutput)
{
  JointRequest currentRequest = specialActionsOutput;
  // do not run this method if interpolation is already finished;
  if (phase == 1.f)
    return true;
  KeyFrameMotion& currentKeyFrameMotion = keyFrameMotions.at(currentKeyFrameMotionIndex);
  KeyFrameMotion::KeyFrame currentKeyFrame = currentKeyFrameMotion.keyFrames.at(currentKeyFrameIndex);

  if (!areMotionsPrioritized() && theFallDownState.state == FallDownState::falling)
  {
    specialActionsOutput.isFallProtectionNeeded = true;
    specialActionsOutput.isMotionFinished = true;
    chooseFallDownProtection(currentKeyFrame.fallDownProtection, currentKeyFrame.holdFallDownProtection, currentKeyFrame.activateFallDownProtection);
  }

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
  phase += (1000.f / currentKeyFrame.duration) * theFrameInfo.cycleTime;
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
          OUTPUT_TEXT("Angle increment for joint"
              << i << " is greater than allowed in step" << currentKeyFrameIndex << " in KeyFrameMotion " << keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID);
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

  specialActionsOutput.angles = currentRequest.angles;
  specialActionsOutput.stiffnessData = currentRequest.stiffnessData;

  // Use stabilization to correct interpolated values
  currentJointAngleID++;
  currentJointAngleID = currentJointAngleID % 5;
  for (int j = 0; j < Joints::numOfJoints; j++)
    jointAngleBuffer[currentJointAngleID].angles[j] = specialActionsOutput.angles[j];

  stabilize(specialActionsOutput);

  // check if interpolation is done
  if (phase >= 1.f)
    keyFrameWasFinished = true;
  return phase >= 1.f;
}

void KeyFrameEngine::stabilize(SpecialActionsOutput& specialActionsOutput)
{
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
    if ((error < theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0]) || (error > theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[1]))
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

bool KeyFrameEngine::armsStuck()
{
  if (theFallDownState.state == FallDownState::onGround && !areMotionsPrioritized()
      && (std::abs(theJointError.angles[Joints::lShoulderRoll]) > maxArmError || std::abs(theJointError.angles[Joints::rShoulderRoll]) > maxArmError)
      && std::abs(theJoinedIMUData.imuData[anglesource].angle.y()) > (bodyAngleOnGround - angleDiffOnGround)
      && std::abs(theJoinedIMUData.imuData[anglesource].angle.y()) < (bodyAngleOnGround + angleDiffOnGround))
  {
    return true;
  }

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
  const SpecialActionRequest::SpecialActionID defaultMotion = SpecialActionRequest::playDead;
  bool foundMotion = false;
  for (int i = 0; i < (int)keyFrameMotions.size(); i++)
    if (id == keyFrameMotions.at(i).keyFrameID)
    {
      currentKeyFrameMotionIndex = i;
      foundMotion = true;
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
  if (currentKeyFrameMotionIndex != specialActionIDQueue.front())
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

bool KeyFrameEngine::verifyRobotPosition(SpecialActionsOutput& specialActionsOutput)
{
  //KeyFrameMotion& currentKeyFrameMotion = keyFrameMotions.at(currentKeyFrameMotionIndex);
  KeyFrameMotion::KeyFrame& currentKeyFrame = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex);

  Angle currentAngle = theJoinedIMUData.imuData[anglesource].angle.y();
  // check if sensor y angle of upper body is too different from intended angle for this key frame

  if (currentKeyFrame.useAngleAtKeyFrameTarget)
  {
    PLOT("module:KeyFrameEngine:realAngleError", (currentAngle.toDegrees() - currentKeyFrame.angleAtKeyFrameTarget.toDegrees()));
    PLOT("module:KeyFrameEngine:setAngleErrorFront", currentKeyFrame.angleAtKeyFrameErrorFront.toDegrees());
    PLOT("module:KeyFrameEngine:setAngleErrorBack", -currentKeyFrame.angleAtKeyFrameErrorBack.toDegrees());
  }
  else
  {
    PLOT("module:KeyFrameEngine:realAngleError", 0);
    PLOT("module:KeyFrameEngine:setAngleErrorFront", 0);
    PLOT("module:KeyFrameEngine:setAngleErrorBack", 0);
  }
  PLOT("module:KeyFrameEngine:targetAngle", currentKeyFrame.angleAtKeyFrameTarget.toDegrees());
  PLOT("module:KeyFrameEngine:currentAngle", currentAngle.toDegrees());

  if (currentKeyFrame.useAngleAtKeyFrameTarget)
  {
    if ((currentAngle - currentKeyFrame.angleAtKeyFrameTarget) < 0)
    {
      if (std::abs(currentAngle - currentKeyFrame.angleAtKeyFrameTarget) > currentKeyFrame.angleAtKeyFrameErrorBack)
        return false;
    }
    else
    {
      if (std::abs(currentAngle - currentKeyFrame.angleAtKeyFrameTarget) > currentKeyFrame.angleAtKeyFrameErrorFront)
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
  if (theMotionRequest.motion != MotionRequest::specialAction
      && (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::sitDown || keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::playDead)
      && theFallDownState.state == FallDownState::upright)
  {
    initEngineData(specialActionsOutput);
    selectActiveMotion(SpecialActionRequest::stand, false, specialActionsOutput);
    phase = 0.f;
  }

  if (!prevFlying && theFallDownState.state == FallDownState::flying)
  {
    clearPrioritizedMotions();

    if (initialKeyFrameMotionID != SpecialActionRequest::sitDown)
    {
      initEngineData(specialActionsOutput);
      selectActiveMotion(SpecialActionRequest::stand, false, specialActionsOutput);
    }

    specialActionsOutput.isFallProtectionNeeded = false;
    prevFlying = true;
  }
  if (resetFallDownProtectionNeeded(specialActionsOutput))
  {
    initEngineData(specialActionsOutput);
    specialActionsOutput.isFallProtectionNeeded = false;
    selectActiveMotion(SpecialActionRequest::lying, false, specialActionsOutput);
    phase = 0.f;
  }
  if (areMotionsPrioritized())
    selectPrioritizedMotion(specialActionsOutput);

  if (keyFrameMotions.at(currentKeyFrameMotionIndex).initialKeyFrame && !areMotionsPrioritized())
    initialKeyFrameMotionID = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID;
}

bool KeyFrameEngine::resetFallDownProtectionNeeded(SpecialActionsOutput& specialActionsOutput)
{
  return specialActionsOutput.isFallProtectionNeeded && (theFallDownState.state == FallDownState::onGround || theFallDownState.state == FallDownState::onGroundLyingStill);
}

void KeyFrameEngine::handleInitialFallDownProtection()
{
  if (!theMotionRequest.inStandUpMotion() && !initialFallDownProtectionActive && theFallDownState.state == FallDownState::falling)
  {
    chooseFallDownProtection(SpecialActionRequest::none, 0, true);
    initialFallDownProtectionActive = true;
  }
  else if (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::flying)
  {
    initialFallDownProtectionActive = false;
  }
}

void KeyFrameEngine::chooseFallDownProtection(SpecialActionRequest::SpecialActionID id, unsigned timer, bool advancedProtection)
{
  if (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::sitDown)
    return;

  if (advancedProtection)
  {
    if (id == SpecialActionRequest::none)
    {
      if (theFallDownState.direction == FallDownState::front)
        priorizeMotion(SpecialActionRequest::saveFallFront, false, saveFallFrontTime);
      else if (theFallDownState.direction == FallDownState::back)
        priorizeMotion(SpecialActionRequest::saveFallBack, false, saveFallBackTime);
      else
        priorizeMotion(SpecialActionRequest::saveFall, false, saveFallTime);

      priorizeMotion(SpecialActionRequest::lying, false, lyingTime);
    }
    else
    {
      priorizeMotion(id, false, timer);
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
