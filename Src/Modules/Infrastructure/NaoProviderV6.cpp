/**
 * @file Modules/Infrastructure/NaoProviderV6.cpp
 * The file declares a module that provides information from the Nao via LoLA.
 * @author TheCanadianGuy & schwingmar
 */

#include "NaoProviderV6.h"

MAKE_MODULE(NaoProviderV6, motionInfrastructure)

#ifdef TARGET_ROBOT

#include "Tools/Debugging/Modify.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Settings.h"

#include "ndevilsbase/ndevils.h"

#include <cstdio>
#include <cstring>
#include <algorithm>

PROCESS_LOCAL NaoProviderV6* NaoProviderV6::theInstance = nullptr;

NaoProviderV6::NaoProviderV6()
{
  NaoProviderV6::theInstance = this;
  
  for(int i = 0; i < Joints::numOfJoints; ++i)
    clippedLastFrame[i] = SensorData::off;
  gyroBufferX.fill(0.f);
  gyroBufferY.fill(0.f);
  gyroBufferZ.fill(0.f);
}

NaoProviderV6::~NaoProviderV6()
{
  NaoProviderV6::theInstance = nullptr;
}

void NaoProviderV6::finishFrame()
{
  if(theInstance)
    theInstance->send();
}

void NaoProviderV6::waitForFrameData()
{
  DEBUG_RESPONSE_ONCE("module:NaoProvider:robotName")
  {
    if(Global::getSettings().robotName == Global::getSettings().bodyName)
      OUTPUT_TEXT("Hi, I am " << Global::getSettings().robotName << ".");
    else
      OUTPUT_TEXT("Hi, I am " << Global::getSettings().robotName << " (using " << Global::getSettings().bodyName << "s Body).");
    OUTPUT(idRobotname, bin, Global::getSettings().robotName << Global::getSettings().bodyName << Global::getSettings().location << Global::getSettings().naoVersion);
  }

  if(theInstance)
    theInstance->naoBody.wait();
}

void NaoProviderV6::send()
{
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:lag100") SystemCall::sleep(100);
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:lag200") SystemCall::sleep(200);
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:lag300") SystemCall::sleep(300);
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:lag1000") SystemCall::sleep(1000);
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:lag3000") SystemCall::sleep(3000);
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:lag6000") SystemCall::sleep(6000);
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:segfault") *(volatile char*)0 = 0;

  DEBUG_RESPONSE("module:NaoProviderV6:ClippingInfo")
  {
    for(int i = 0; i < Joints::numOfJoints; ++i)
    {
      if(i == Joints::rHipYawPitch) // missing on Nao
        ++i;

      if(theJointRequest.angles[i] != SensorData::off)
      {
        if(theJointRequest.angles[i] > theJointCalibration.joints[i].maxAngle)
        {
          if(clippedLastFrame[i] != theJointCalibration.joints[i].maxAngle)
          {
            char tmp[64];
            sprintf(tmp, "warning: clipped joint %s at %.03f, requested %.03f.", Joints::getName(static_cast<Joints::Joint>(i)), theJointCalibration.joints[i].maxAngle.toDegrees(), theJointRequest.angles[i].toDegrees());
            OUTPUT_TEXT(tmp);
            clippedLastFrame[i] = theJointCalibration.joints[i].maxAngle;
          }
        }
        else if(theJointRequest.angles[i] < theJointCalibration.joints[i].minAngle)
        {
          if(clippedLastFrame[i] != theJointCalibration.joints[i].minAngle)
          {
            char tmp[64];
            sprintf(tmp, "warning: clipped joint %s at %.04f, requested %.03f.", Joints::getName(static_cast<Joints::Joint>(i)), theJointCalibration.joints[i].minAngle.toDegrees(), theJointRequest.angles[i].toDegrees());
            OUTPUT_TEXT(tmp);
            clippedLastFrame[i] = theJointCalibration.joints[i].minAngle;
          }
        }
        else
          clippedLastFrame[i] = SensorData::off;
      }
    }
  }

  NDActuatorData* actuators;
  naoBody.openActuators(actuators);
  ASSERT(NDJoints::headYaw == 0);
  ASSERT(static_cast<int>(Joints::numOfJoints) - 1 == NDJoints::numOfJoints); //rHipYawPitch missin lbh

  memcpy(&actuators->positions[0], &theJointRequest.angles[0], 7 * sizeof(float)); // head + left arm
  memcpy(&actuators->positions[NDJoints::lHipYawPitch], &theJointRequest.angles[Joints::lHipYawPitch], 6 * sizeof(float)); // left leg
  memcpy(&actuators->positions[NDJoints::rHipRoll], &theJointRequest.angles[Joints::rHipRoll], 5 * sizeof(float)); // right leg
  memcpy(&actuators->positions[NDJoints::rShoulderPitch], &theJointRequest.angles[Joints::rShoulderPitch], 5 * sizeof(float)); // right arm
  actuators->positions[NDJoints::lHand] = theJointRequest.angles[Joints::lHand];
  actuators->positions[NDJoints::rHand] = theJointRequest.angles[Joints::rHand];
  copyStiffness(theJointRequest, actuators);

  for (int i = 0; i < NDJoints::numOfJoints; ++i)
  {
    if (actuators->positions[i] == SensorData::off)
    {
      actuators->positions[i] = 0.f;
      actuators->stiffness[i] = 0.f;
    }
  }

  const LEDRequest& ledRequest(theLEDRequest);
  on = (theFrameInfo.time / 50 & 8) != 0;
  fastOn = (theFrameInfo.time / 10 & 8) != 0;
  copyLEDs(ledRequest, actuators);
  actuators->sonars[0] = false; // TODO
  actuators->sonars[1] = false; // TODO

  naoBody.closeActuators();
}

void NaoProviderV6::copyStiffness(const JointRequest &theJointRequest, NDActuatorData* actuators)
{
  actuators->stiffness[NDJoints::headYaw] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::headYaw]) / 100.f;
  actuators->stiffness[NDJoints::headPitch] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::headPitch]) / 100.f;
  actuators->stiffness[NDJoints::lShoulderPitch] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::lShoulderPitch]) / 100.f;
  actuators->stiffness[NDJoints::lShoulderRoll] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::lShoulderRoll]) / 100.f;
  actuators->stiffness[NDJoints::lElbowYaw] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::lElbowYaw]) / 100.f;
  actuators->stiffness[NDJoints::lElbowRoll] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::lElbowRoll]) / 100.f;
  actuators->stiffness[NDJoints::lWristYaw] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::lWristYaw]) / 100.f;
  actuators->stiffness[NDJoints::lHipYawPitch] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch]) / 100.f;
  actuators->stiffness[NDJoints::lHipRoll] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::lHipRoll]) / 100.f;
  actuators->stiffness[NDJoints::lHipPitch] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::lHipPitch]) / 100.f;
  actuators->stiffness[NDJoints::lKneePitch] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::lKneePitch]) / 100.f;
  actuators->stiffness[NDJoints::lAnklePitch] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::lAnklePitch]) / 100.f;
  actuators->stiffness[NDJoints::lAnkleRoll] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::lAnkleRoll]) / 100.f;
  actuators->stiffness[NDJoints::rHipRoll] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::rHipRoll]) / 100.f;
  actuators->stiffness[NDJoints::rHipPitch] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::rHipPitch]) / 100.f;
  actuators->stiffness[NDJoints::rKneePitch] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::rKneePitch]) / 100.f;
  actuators->stiffness[NDJoints::rAnklePitch] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::rAnklePitch]) / 100.f;
  actuators->stiffness[NDJoints::rAnkleRoll] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::rAnkleRoll]) / 100.f;
  actuators->stiffness[NDJoints::rShoulderPitch] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::rShoulderPitch]) / 100.f;
  actuators->stiffness[NDJoints::rShoulderRoll] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::rShoulderRoll]) / 100.f;
  actuators->stiffness[NDJoints::rElbowYaw] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::rElbowYaw]) / 100.f;
  actuators->stiffness[NDJoints::rElbowRoll] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::rElbowRoll]) / 100.f;
  actuators->stiffness[NDJoints::rWristYaw] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::rWristYaw]) / 100.f;
  actuators->stiffness[NDJoints::lHand] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::lHand]) / 100.f;
  actuators->stiffness[NDJoints::rHand] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[Joints::rHand]) / 100.f;
}

void NaoProviderV6::copyLEDs(const LEDRequest &theLEDRequest, NDActuatorData* actuators)
{
  for (int i = 0; i < NDRGBLEDs::numOfRGBLEDs; i++)
    setLED(
      theLEDRequest.ledStates[LEDRequest::chestRed + i],
      actuators->chestLEDs[NDRGBLEDs::r + i]
    );

  for (int i = 0; i < NDREyeLEDs::numOfREyeLEDs * NDRGBLEDs::numOfRGBLEDs; i++)
    setLED(
      theLEDRequest.ledStates[LEDRequest::faceRightRed0Deg + i],
      actuators->rEyeLEDs[i / NDREyeLEDs::numOfREyeLEDs][i % NDREyeLEDs::numOfREyeLEDs]
    );

  for (int i = 0; i < NDLEarLEDs::numOfLEarLEDs; i++)
    setLED(
      theLEDRequest.ledStates[LEDRequest::earsLeft0Deg + i],
      actuators->lEarLEDs[NDLEarLEDs::lEarDeg0 + i]
    );

  for (int i = 0; i < NDREarLEDs::numOfREarLEDs; i++)
    setLED(
      theLEDRequest.ledStates[LEDRequest::earsRight0Deg + i],
      actuators->rEarLEDs[NDREarLEDs::rEarDeg0 - i]
    );

  for (int i = 0; i < NDLEyeLEDs::numOfLEyeLEDs * NDRGBLEDs::numOfRGBLEDs; i++)
    setLED(
      theLEDRequest.ledStates[LEDRequest::faceLeftRed0Deg + i],
      actuators->lEyeLEDs[i/NDLEyeLEDs::numOfLEyeLEDs][(NDLEyeLEDs::numOfLEyeLEDs * NDRGBLEDs::numOfRGBLEDs + NDLEyeLEDs::lEyeDeg0 - i) % NDLEyeLEDs::numOfLEyeLEDs]
    );

  for (int i = 0; i < NDRGBLEDs::numOfRGBLEDs; i++)
    setLED(
      theLEDRequest.ledStates[LEDRequest::footLeftRed + i],
      actuators->lFootLEDs[NDRGBLEDs::r + i]
    );

  for (int i = 0; i < NDRGBLEDs::numOfRGBLEDs; i++)
    setLED(
      theLEDRequest.ledStates[LEDRequest::footRightRed + i],
      actuators->rFootLEDs[NDRGBLEDs::r + i]
    );

  for (int i = 0; i < NDSkullLEDs::numOfSkullLEDs; i++)
    setLED(
      theLEDRequest.ledStates[LEDRequest::headLedRearLeft0 + i],
      actuators->skullLEDs[(NDSkullLEDs::rearLeft0 + i) % NDSkullLEDs::numOfSkullLEDs]
    );
}

void NaoProviderV6::setLED(const LEDRequest::LEDState &state, float& actuator)
{
  actuator = (state == LEDRequest::on ||
                      (state == LEDRequest::blinking && on) ||
                      (state == LEDRequest::fastBlinking && fastOn))
                     ? 1.0f : (state == LEDRequest::half ? 0.5f : 0.0f);
}

void NaoProviderV6::update(FrameInfo& frameInfo)
{
  frameInfo.time = std::max(frameInfo.time + 1, SystemCall::getCurrentSystemTime());
  frameInfo.cycleTime = 0.012f;
}

void NaoProviderV6::update(FsrSensorData& fsrSensorData)
{
  float* sensors = &naoBody.getSensors()->fsr[0];
  fsrSensorData.leftTotal = 0.f;
  fsrSensorData.rightTotal = 0.f;

  for (size_t i = 0; i < fsrSensorData.left.size(); ++i)
  {
    fsrSensorData.left[i] = sensors[i];
    fsrSensorData.leftTotal += fsrSensorData.left[i];
  }
  for (size_t i = 0; i < fsrSensorData.right.size(); ++i)
  {
    fsrSensorData.right[i] = sensors[NDFsrs::rFrontLeft + i];
    fsrSensorData.rightTotal += fsrSensorData.right[i];
  }
  
}

void NaoProviderV6::update(InertialSensorData& inertialSensorData)
{
  float* gyro = &naoBody.getSensors()->gyro[0];
  float* acc = &naoBody.getSensors()->acc[0];
  float* angle = &naoBody.getSensors()->angle[0];
   
  // remove gyro bias
  {
    gyroBufferX.push_front(gyro[0]);
    gyroBufferY.push_front(gyro[1]);
    gyroBufferZ.push_front(gyro[2]);
    float maxErrSum = 0.0001f;
    MODIFY("module:NaoProviderV6:maxErrSum", maxErrSum);
    float minChangeOfBias = 0.5f;
    MODIFY("module:NaoProviderV6:minChangeOfBias", minChangeOfBias);
    bool biasChanged = false;

    if (naoBody.getTransitionToBhuman() == 0.f)
    {
      float average = gyroBufferX.average();
      float errSum = 0.f;
      for (int i = 0; i < 20; i++) errSum += (gyroBufferX[i] - average) * (gyroBufferX[i] - average);
      if (errSum < maxErrSum)
      {
        gyroXBias = average;
        if(std::abs(average - gyroXBias) > minChangeOfBias)
        {
          biasChanged = true;
        }
      }
      average = gyroBufferY.average();
      errSum = 0.f;
      for (int i = 0; i < 20; i++) errSum += (gyroBufferY[i] - average) * (gyroBufferY[i] - average);
      if (errSum < maxErrSum)
      {
        gyroYBias = average;
        if(std::abs(average - gyroYBias) > minChangeOfBias)
        {
          biasChanged = true;
        }
      }
      average = gyroBufferZ.average();
      errSum = 0.f;
      for (int i = 0; i < 20; i++) errSum += (gyroBufferZ[i] - average) * (gyroBufferZ[i] - average);
      if (errSum < maxErrSum)
      {
        gyroZBias = average;
        if(std::abs(average - gyroZBias) > minChangeOfBias)
        {
          biasChanged = true;
        }
      }

      if(biasChanged && !soundPlayed)
      {
        soundPlayed = true;
        SystemCall::playSound("gyroscope_bias_calibrated.wav");
      }
    }
  }
  inertialSensorData.gyro.x() = gyro[0] - gyroXBias;
  inertialSensorData.gyro.y() = gyro[1] - gyroYBias;
  inertialSensorData.gyro.z() = gyro[2] - gyroZBias;

  inertialSensorData.acc.x() = -acc[0];
  inertialSensorData.acc.y() = -acc[1];
  inertialSensorData.acc.z() = -acc[2];

  inertialSensorData.angle.x() = angle[0];
  inertialSensorData.angle.y() = angle[1];
  
  PLOT("module:NaoProviderV6:gyroXBias", Angle(gyroXBias).toDegrees());
  PLOT("module:NaoProviderV6:gyroYBias", Angle(gyroYBias).toDegrees());
  PLOT("module:NaoProviderV6:gyroZBias", Angle(gyroZBias).toDegrees());
  PLOT("module:NaoProviderV6:gyroX", inertialSensorData.gyro.x().toDegrees());
  PLOT("module:NaoProviderV6:gyroY", inertialSensorData.gyro.y().toDegrees());
  PLOT("module:NaoProviderV6:gyroZ", inertialSensorData.gyro.z().toDegrees());
  PLOT("module:NaoProviderV6:accX", inertialSensorData.acc.x());
  PLOT("module:NaoProviderV6:accY", inertialSensorData.acc.y());
  PLOT("module:NaoProviderV6:accZ", inertialSensorData.acc.z());
  PLOT("module:NaoProviderV6:angleX", inertialSensorData.angle.x().toDegrees());
  PLOT("module:NaoProviderV6:angleY", inertialSensorData.angle.y().toDegrees());
}

void NaoProviderV6::copyJointValues(float* source, Angle* target)
{
  memcpy(target, source, 7 * sizeof(float)); // head + left arm
  memcpy(target + Joints::lHipYawPitch, source + NDJoints::lHipYawPitch, 6 * sizeof(float)); // left leg
  memcpy(target + Joints::rHipRoll, source + NDJoints::rHipRoll, 5 * sizeof(float)); // right leg
  memcpy(target + Joints::rShoulderPitch, source + NDJoints::rShoulderPitch, 5 * sizeof(float)); // right arm
  target[Joints::lHand] = source[NDJoints::lHand];
  target[Joints::rHand] = source[NDJoints::rHand];
}

void NaoProviderV6::copyCurrents(float* source, short* target)
{
  for (int i = 0; i < 7; i++) 
    target[i] = static_cast<short>(source[i] * 1000.f); // head + left arm
  for (int i = 0; i < 6; i++) 
    target[i + Joints::lHipYawPitch] = static_cast<short>(source[i + NDJoints::lHipYawPitch] * 1000.f);
  for (int i = 0; i < 5; i++) 
    target[i + Joints::rHipRoll] = static_cast<short>(source[i + NDJoints::rHipRoll] * 1000.f);
  for (int i = 0; i < 5; i++) 
    target[i + Joints::rShoulderPitch] = static_cast<short>(source[i + NDJoints::rShoulderPitch] * 1000.f);
  target[Joints::lHand] = static_cast<short>(source[NDJoints::lHand]*1000.f);
  target[Joints::rHand] = static_cast<short>(source[NDJoints::rHand]*1000.f);
}

void NaoProviderV6::copyTemperatures(float* source, unsigned char* target)
{
  for (int i = 0; i < 7; i++) 
    target[i] = static_cast<unsigned char>(source[i]); // head + left arm
  for (int i = 0; i < 6; i++) 
    target[i + Joints::lHipYawPitch] = static_cast<unsigned char>(source[i + NDJoints::lHipYawPitch]);
  for (int i = 0; i < 5; i++)
    target[i + Joints::rHipRoll] = static_cast<unsigned char>(source[i + NDJoints::rHipRoll]);
  for (int i = 0; i < 5; i++) 
    target[i + Joints::rShoulderPitch] = static_cast<unsigned char>(source[i + NDJoints::rShoulderPitch]);
  target[Joints::lHand] = static_cast<unsigned char>(source[NDJoints::lHand]);
  target[Joints::rHand] = static_cast<unsigned char>(source[NDJoints::rHand]);
}

void NaoProviderV6::copyTemperatureStatus(int* source, unsigned char* target)
{
  for (int i = 0; i < 7; i++)
    target[i] = static_cast<unsigned char>(source[i]); // head + left arm
  for (int i = 0; i < 6; i++)
    target[i + Joints::lHipYawPitch] = static_cast<unsigned char>(source[i + NDJoints::lHipYawPitch]);
  for (int i = 0; i < 5; i++)
    target[i + Joints::rHipRoll] = static_cast<unsigned char>(source[i + NDJoints::rHipRoll]);
  for (int i = 0; i < 5; i++)
    target[i + Joints::rShoulderPitch] = static_cast<unsigned char>(source[i + NDJoints::rShoulderPitch]);
  target[Joints::lHand] = static_cast<unsigned char>(source[NDJoints::lHand]);
  target[Joints::rHand] = static_cast<unsigned char>(source[NDJoints::rHand]);
}

void NaoProviderV6::update(JointSensorData& jointSensorData)
{
  NDSensorData* sensors = naoBody.getSensors();

  copyJointValues(&sensors->position[0], &jointSensorData.angles[0]);
  copyCurrents(&sensors->current[0], &jointSensorData.currents[0]);
  copyTemperatures(&sensors->temperature[0], &jointSensorData.temperatures[0]);
  copyTemperatureStatus(&sensors->status[0], &jointSensorData.status[0]);
  jointSensorData.angles[Joints::rHipYawPitch] = jointSensorData.angles[Joints::lHipYawPitch];
  jointSensorData.currents[Joints::rHipYawPitch] = jointSensorData.currents[Joints::lHipYawPitch];
  jointSensorData.temperatures[Joints::rHipYawPitch] = jointSensorData.temperatures[Joints::lHipYawPitch];
  jointSensorData.status[Joints::rHipYawPitch] = jointSensorData.status[Joints::lHipYawPitch];
  jointSensorData.timestamp = theFrameInfo.time;
}

void NaoProviderV6::update(KeyStates& keyStates)
{
  float* sensors = &naoBody.getSensors()->touch[0];

  for(int i = 0; i < 3; ++i) keyStates.pressed[KeyStates::headFront + i] = sensors[NDTouchs::headFront + i] != 0.f; // head
  for (int i = 0; i < 3; ++i) keyStates.pressed[KeyStates::lHandBack+i] = sensors[NDTouchs::lHandBack+i] != 0.f;
  for (int i = 0; i < 3; ++i) keyStates.pressed[KeyStates::rHandBack+i] = sensors[NDTouchs::rHandBack+i] != 0.f;
  for (int i = 0; i < 2; ++i) keyStates.pressed[KeyStates::leftFootLeft+i] = sensors[NDTouchs::lBumperLeft+i] != 0.f;
  for (int i = 0; i < 2; ++i) keyStates.pressed[KeyStates::rightFootLeft+i] = sensors[NDTouchs::rBumperLeft+i] != 0.f;
  keyStates.pressed[KeyStates::chest] = sensors[NDTouchs::chestButton] != 0.f;
}

void NaoProviderV6::update(SystemSensorData& systemSensorData)
{
  float* sensors = naoBody.getSensors()->battery;
  if(theFrameInfo.getTimeSince(lastBodyTemperatureReadTime) > 10000)
  {
    lastBodyTemperatureReadTime = theFrameInfo.time;
    systemSensorData.cpuTemperature = naoBody.getCPUTemperature();
  }
  systemSensorData.batteryCurrent = sensors[NDBattery::current];
  systemSensorData.batteryLevel = sensors[NDBattery::charge];
  systemSensorData.batteryTemperature = sensors[NDBattery::temperature];
  // TODO: what about NDBattery::status?
}

void NaoProviderV6::update(UsSensorData& usSensorData)
{
  // TODO: usrequest not used!
  float* sensors = naoBody.getSensors()->sonar;
  if(theUSRequest.receiveMode != -1)
  {
    // only one measurement for each side
    usSensorData.left[0] = sensors[0]; 
    usSensorData.right[0] = sensors[1];
    
    usSensorData.actuatorMode = static_cast<UsSensorData::UsActuatorMode>(theUSRequest.receiveMode);
    usSensorData.timeStamp = theFrameInfo.time;
  }

  PLOT("module:NaoProviderV6:usLeft", usSensorData.left[0]);
  PLOT("module:NaoProviderV6:usRight", usSensorData.right[0]);
}

#endif // TARGET_ROBOT
