/**
 * @file KeyFrameEngine.cpp
 * This file declares a module that handles key frame motions.
 * @author <a href="ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
 */

#include "KeyFrameEngine.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Settings.h"
#include "Platform/File.h"

MAKE_MODULE(KeyFrameEngine, motionControl)

KeyFrameEngine::KeyFrameEngine()
{
  
}

void KeyFrameEngine::init(SpecialActionsOutput& specialActionsOutput)
{
  if (!initialized)
  {
    loadKeyFrameMotions();
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
    specialActionsOutput.angles[i] = theJointAngles.angles[i];
  specialActionsOutput.isLeavingPossible = true;
  specialActionsOutput.stiffnessData.stiffnesses = defaultStiffnesses;
  ASSERT(specialActionsOutput.isValid());

  // index to first frame
  currentKeyFrameIndex = 0;
  phase = 0.f;

  // set originFrame to sensor values and default stiffness
  originFrame.stiffnesses = defaultStiffnesses;
  std::copy(specialActionsOutput.angles.data() + 0, specialActionsOutput.angles.data() + Joints::firstArmJoint - 1, originFrame.headAngles);
  std::copy(specialActionsOutput.angles.data() + Joints::firstArmJoint, specialActionsOutput.angles.data() + Joints::lHipYawPitch - 1, originFrame.armsAngles);
  std::copy(specialActionsOutput.angles.data() + Joints::lHipYawPitch, specialActionsOutput.angles.data() + Joints::numOfJoints - 1, originFrame.legsAngles);
  // other members are already initialized with defaults

  // set correct ID
  selectActiveMotion(theMotionSelection.specialActionRequest.specialAction, specialActionsOutput);

  // reset sensor control
  angleYBuffer.fill(0);
  gyroYBuffer.fill(0);
  hipYawBuffer.fill(theJointAngles.angles[Joints::lHipYawPitch]);
  engineDataReset = true;
}

void KeyFrameEngine::update(SpecialActionsOutput& specialActionsOutput)
{
  DECLARE_PLOT("module:KeyFrameEngine:hipYawPitchError");

  DEBUG_RESPONSE_ONCE("loadKeyFrames") { loadKeyFrameMotions(); }
  // update sensor buffers
  angleYBuffer.push_front(useIMUModel ? theIMUModel.orientation.y() : theInertialSensorData.angle.y());
  gyroYBuffer.push_front(theInertialSensorData.gyro.y());
  hipYawBuffer.push_front(specialActionsOutput.angles[Joints::lHipYawPitch]);

  if (!initialized)
  {
    init(specialActionsOutput);
  }
  else if (theMotionSelection.ratios[MotionRequest::specialAction] < 1.f)
  {
    if (!engineDataReset || theMotionSelection.ratios[MotionRequest::specialAction] == 0.f)
      initEngineData(specialActionsOutput);
  }
  else
  {
    if (theMotionRequest.motion != MotionRequest::specialAction
      && (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::sitDown || keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::playDead))
    {
      selectActiveMotion(SpecialActionRequest::stand, specialActionsOutput);
      phase = 0.f;
    }
    else
    {
      specialActionsOutput.isLeavingPossible = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex).leavingPossible;
      specialActionsOutput.isMotionStable = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID != SpecialActionRequest::playDead
        && (specialActionsOutput.isLeavingPossible || keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID == SpecialActionRequest::sitDown) && isStable() && std::abs(angleYBuffer.average()) < 15_deg && std::abs(theInertialSensorData.angle.x()) < 15_deg;
      //keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex).leavingPossible&& phase == 1.f;
      specialActionsOutput.executedSpecialAction.specialAction = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID;

      if (specialActionsOutput.isLeavingPossible
        && theMotionRequest.motion != MotionRequest::specialAction)
      {
        return;
      }

      if (interpolate(specialActionsOutput)) // true if phase >= 1.f, i.e. single keyFrame was executed
      {
        // check if robots joints and upper body are ok after interpolation, otherwise cancel motion
        // TODO Ingmar 12.10.2019: verify if this actually works
        // TODO Janine 13.10.2019: it does not work
        if (!verifyRobotPosition(specialActionsOutput))
        {
          selectActiveMotion(SpecialActionRequest::playDead, specialActionsOutput);
          currentKeyFrameIndex = 0;
          lastFinishedKeyFrameIndex = 0;
          keyFrameFinishedTimestamp = theFrameInfo.time;
          phase = 0.f;
          return;
        }
        if (lastFinishedKeyFrameIndex != currentKeyFrameIndex || lastFinishedKeyFrameMotionIndex != currentKeyFrameMotionIndex)
        {
          lastFinishedKeyFrameIndex = currentKeyFrameIndex;
          lastFinishedKeyFrameMotionIndex = currentKeyFrameMotionIndex;
          keyFrameFinishedTimestamp = theFrameInfo.time;
        }
        if (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex).waitForStable
          && theFrameInfo.getTimeSince(keyFrameFinishedTimestamp) < maxWaitForStableTime
          && !isStable())
          return;
        // Set current frame as originFrame for possible transition to next motion! Only angles and stiffnesses needed.
        originFrame.stiffnesses = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex).stiffnesses;
        memcpy(&originFrame.headAngles[0], &keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex).headAngles[0], 2 * sizeof(float));
        memcpy(&originFrame.armsAngles[0], &keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex).armsAngles[0], 12 * sizeof(float));
        memcpy(&originFrame.legsAngles[0], &keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex).legsAngles[0], 12 * sizeof(float));
        phase = 0.f; // reset phase
        currentKeyFrameIndex++; // next keyFrame
        // check if we are finished with the motion. if yes we can leave or run the next or repeat the last output forever
        if (currentKeyFrameIndex >= static_cast<int>(keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.size()))
        {
          if (keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrameID != theMotionSelection.specialActionRequest.specialAction
            && theMotionSelection.targetMotion == MotionRequest::specialAction) // next motion is selected
          {
            selectActiveMotion(theMotionSelection.specialActionRequest.specialAction, specialActionsOutput);
          }
          else // same motion, repeat the finished interpolate with phase = 1.f
          {
            currentKeyFrameIndex = (int)keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.size() - 1;
            phase = 1.f;
          }
        }
        else
        {
          KeyFrameMotion& currentKeyFrameMotion = keyFrameMotions.at(currentKeyFrameMotionIndex);
          //KeyFrameMotion::KeyFrame& currentKeyFrame = currentKeyFrameMotion.keyFrames.at(currentKeyFrameIndex);
          const KeyFrameMotion::KeyFrame& lastKeyFrameRef = (currentKeyFrameIndex == 0) ?
            originFrame : currentKeyFrameMotion.keyFrames.at(currentKeyFrameIndex - 1);
          KeyFrameMotion::KeyFrame lastKeyFrame = setKeyFrameAngles(lastKeyFrameRef);
        }
      }
      stabilize(specialActionsOutput);
    }
    engineDataReset = false;
  }
}

bool KeyFrameEngine::interpolate(SpecialActionsOutput& specialActionsOutput)
{
  KeyFrameMotion& currentKeyFrameMotion = keyFrameMotions.at(currentKeyFrameMotionIndex);
  KeyFrameMotion::KeyFrame& currentKeyFrame = currentKeyFrameMotion.keyFrames.at(currentKeyFrameIndex);
  if (phase == 0.f)
  {
    const KeyFrameMotion::KeyFrame& lastKeyFrameRef = (currentKeyFrameIndex == 0) ?
      originFrame : currentKeyFrameMotion.keyFrames.at(currentKeyFrameIndex - 1);
    lastKeyFrame = setKeyFrameAngles(lastKeyFrameRef);
  }
  phase += (1000.f/ currentKeyFrame.duration) * theFrameInfo.cycleTime;
  phase = std::min(1.f, phase);
  float ratio = (currentKeyFrame.intType == KeyFrameMotion::KeyFrame::KeyFrameInterpolationType::linear) ? phase : (std::cos(phase * pi) - 1.f) / -2.f;
  for (int i = 0; i < Joints::firstArmJoint; i++)
  {
    if (currentKeyFrame.headAngles[i] == JointAngles::off)
      specialActionsOutput.angles[i] = theJointAngles.angles[i];
    else if (currentKeyFrame.headAngles[i] == JointAngles::ignore)
      specialActionsOutput.angles[i] = JointAngles::ignore;
    else
      specialActionsOutput.angles[i] = currentKeyFrame.headAngles[i] * ratio + lastKeyFrame.headAngles[i] * (1.f - ratio);
  }
  for (int i = Joints::firstArmJoint; i < Joints::lHipYawPitch; i++)
  {
    if (currentKeyFrame.armsAngles[i - Joints::firstArmJoint] == JointAngles::off)
      specialActionsOutput.angles[i] = theJointAngles.angles[i];
    else if (currentKeyFrame.armsAngles[i - Joints::firstArmJoint] == JointAngles::ignore)
      specialActionsOutput.angles[i] = JointAngles::ignore;
    else
      specialActionsOutput.angles[i] = currentKeyFrame.armsAngles[i - Joints::firstArmJoint] * ratio + lastKeyFrame.armsAngles[i - Joints::firstArmJoint] * (1.f - ratio);
  }
  for (int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
  {
    if (currentKeyFrame.legsAngles[i - Joints::lHipYawPitch] == JointAngles::off)
      specialActionsOutput.angles[i] = theJointAngles.angles[i];
    else if (currentKeyFrame.legsAngles[i - Joints::lHipYawPitch] == JointAngles::ignore)
      specialActionsOutput.angles[i] = JointAngles::ignore;
    else
      specialActionsOutput.angles[i] = currentKeyFrame.legsAngles[i - Joints::lHipYawPitch] * ratio + lastKeyFrame.legsAngles[i - Joints::lHipYawPitch] * (1.f - ratio);
  }
  for (int i = 0; i < Joints::numOfJoints; i++)
  {
    if (currentKeyFrame.stiffnesses[i] > 0 && currentKeyFrame.stiffnesses[i] <= 100)
      specialActionsOutput.stiffnessData.stiffnesses[i] =
      std::min(specialActionsOutput.stiffnessData.stiffnesses[i] + 10, std::max(specialActionsOutput.stiffnessData.stiffnesses[i] - 10, currentKeyFrame.stiffnesses[i]));
    else
      specialActionsOutput.stiffnessData.stiffnesses[i] = currentKeyFrame.stiffnesses[i];
  }
  return phase >= 1.f;
}

void KeyFrameEngine::stabilize(SpecialActionsOutput& specialActionsOutput)
{
  static Angle lastError = 0;
  static Angle errorSum = 0;
  static unsigned int lastKeyFrameFinishedTimestamp = 0;
  
  KeyFrameMotion& currentKeyFrameMotion = keyFrameMotions.at(currentKeyFrameMotionIndex);
  KeyFrameMotion::KeyFrame& currentKeyFrame = keyFrameMotions.at(currentKeyFrameMotionIndex).keyFrames.at(currentKeyFrameIndex);

  if (!currentKeyFrame.stabilize || lastKeyFrameFinishedTimestamp != keyFrameFinishedTimestamp)
  {
    lastError = 0;
    errorSum = 0;
    return;
  }

  
  {
    Angle currentAngle = useIMUModel ? theIMUModel.orientation.y() : theInertialSensorData.angle.y();
    Angle error = currentKeyFrame.angleAtKeyFrameTarget - currentAngle;
    Angle controlledCorrection = currentKeyFrameMotion.stabilizationP * error + currentKeyFrameMotion.stabilizationI * errorSum + currentKeyFrameMotion.stabilizationD * (lastError - error);

    Angle anklePitchCorrection = specialActionsOutput.angles[Joints::lAnklePitch] + ankleInfluence * controlledCorrection;
    
    if (currentKeyFrame.useAngleAtKeyFrameTarget && std::abs(error) > 15_deg)
    {
      abortMotion = true;
    }
    else
    {
      specialActionsOutput.angles[Joints::lHipPitch] += hipInfluence * controlledCorrection;
      specialActionsOutput.angles[Joints::rHipPitch] += hipInfluence * controlledCorrection;

      specialActionsOutput.angles[Joints::lAnklePitch] += (anklePitchCorrection > minAnklePitchAngle) ? ankleInfluence * controlledCorrection : 0.f;
      specialActionsOutput.angles[Joints::rAnklePitch] += (anklePitchCorrection > minAnklePitchAngle) ? ankleInfluence * controlledCorrection : 0.f;
    }

    lastError = error;
    errorSum += error;
  }


  if (useHipYawCorrection 
    && specialActionsOutput.angles[Joints::lHipYawPitch] <= theJointCalibration.joints[Joints::lHipYawPitch].maxAngle
    && specialActionsOutput.angles[Joints::lHipYawPitch] >= theJointCalibration.joints[Joints::lHipYawPitch].minAngle)
  {
    Angle hipYawError = hipYawBuffer[4] - theJointAngles.angles[Joints::lHipYawPitch];
    specialActionsOutput.angles[Joints::lHipPitch] = std::max(
      std::min<Angle>(theJointCalibration.joints[Joints::lHipPitch].maxAngle, specialActionsOutput.angles[Joints::lHipPitch] + hipYawError), 
        theJointCalibration.joints[Joints::lHipPitch].minAngle);
    specialActionsOutput.angles[Joints::rHipPitch] = std::max(
      std::min<Angle>(theJointCalibration.joints[Joints::rHipPitch].maxAngle, specialActionsOutput.angles[Joints::rHipPitch] + hipYawError),
      theJointCalibration.joints[Joints::rHipPitch].minAngle);
  }
  PLOT("module:KeyFrameEngine:hipYawPitchError", hipYawBuffer[4] - theJointAngles.angles[Joints::lHipYawPitch]);
}

bool KeyFrameEngine::isStable()
{
  return std::abs(gyroYBuffer.average()) < maxAvgGyroForStable;
}

void KeyFrameEngine::loadKeyFrameMotions()
{
  keyFrameMotions.clear();
  for (int i = 0; i < SpecialActionRequest::numOfSpecialActionIDs; i++)
  {
    std::string name = SpecialActionRequest::getName((SpecialActionRequest::SpecialActionID)i);
    std::string path = std::string(File::getBHDir()) + "/Config/KeyFrameEngine/" + name + ".kfm";
    InMapFile stream(path);
    if (stream.exists())
    {
      KeyFrameMotion kfm;
      stream >> kfm;
      keyFrameMotions.push_back(kfm);
    }
    else
    {
      OUTPUT_WARNING("KeyFrameEngine: missing ID " << name << " on load");
    }
  }
}

void KeyFrameEngine::selectActiveMotion(SpecialActionRequest::SpecialActionID id, SpecialActionsOutput& specialActionsOutput)
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
    selectActiveMotion(defaultMotion, specialActionsOutput);
  currentKeyFrameIndex = 0;
}

KeyFrameMotion::KeyFrame KeyFrameEngine::setKeyFrameAngles(const KeyFrameMotion::KeyFrame& keyFrame)
{
  KeyFrameMotion::KeyFrame result;
  for (int i = 0; i < 2; i++)
    if (keyFrame.headAngles[i] == JointAngles::off || keyFrame.headAngles[i] == JointAngles::ignore)
      result.headAngles[i] = theJointAngles.angles[i];
    else
      result.headAngles[i] = keyFrame.headAngles[i];
  for (int i = 0; i < 12; i++)
    if (keyFrame.armsAngles[i] == JointAngles::off || keyFrame.armsAngles[i] == JointAngles::ignore)
      result.armsAngles[i] = theJointAngles.angles[i + 2];
    else
      result.armsAngles[i] = keyFrame.armsAngles[i];
  for (int i = 0; i < 12; i++)
    if (keyFrame.legsAngles[i] == JointAngles::off || keyFrame.legsAngles[i] == JointAngles::ignore)
      result.legsAngles[i] = theJointAngles.angles[i + 14];
    else
      result.legsAngles[i] = keyFrame.legsAngles[i];
  return result;
}

bool KeyFrameEngine::verifyRobotPosition(SpecialActionsOutput& specialActionsOutput)
{
  // check if sensor y angle of upper body is too different from intended angle for this key frame
  /*
  if (currentKeyFrame.useAngleAtKeyFrameTarget && std::abs(currentAngle - currentKeyFrame.angleAtKeyFrameTarget) > maxAngleDifference)
  {
    return false;
  }*/
  Angle maxArmAngleDiff = 0_deg;
  Angle maxLegAngleDiff = 0_deg;
  for (int i = Joints::firstArmJoint; i < Joints::lHipYawPitch; i++)
    maxArmAngleDiff = std::max<Angle>(maxArmAngleDiff, std::abs(specialActionsOutput.angles[i] - theJointAngles.angles[i]));
  for (int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
    maxLegAngleDiff = std::max<Angle>(maxLegAngleDiff, std::abs(specialActionsOutput.angles[i] - theJointAngles.angles[i]));
  //if (maxArmAngleDiff > 30_deg || maxLegAngleDiff > 30_deg) // arm or leg might have started in wrong/strange position so that key frame was not reachable
  //  return false;
  return true;
}