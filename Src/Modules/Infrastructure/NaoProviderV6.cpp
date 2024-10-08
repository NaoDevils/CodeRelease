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
#include "Tools/Debugging/Annotation.h"
#include "Tools/Settings.h"
#include "Tools/Module/Blackboard.h"
#include "Representations/Infrastructure/JointRequest.h"

#include "naodevilsbase/naodevilsbase.h"

#include <cstdio>
#include <cstring>
#include <algorithm>
#include <numeric>

CycleLocal<NaoProviderV6*> NaoProviderV6::theInstance(nullptr);

#ifdef TARGET_ROBOT
int NaoProviderV6::firstSensorTimestamp = 0;
#endif

NaoProviderV6::NaoProviderV6()
{
  NaoProviderV6::theInstance = this;

  for (int i = 0; i < Joints::numOfJoints; ++i)
    clippedLastFrame[i] = SensorData::off;
  gyroBuffer.fill(Vector3a::Zero());
}

NaoProviderV6::~NaoProviderV6()
{
  theInstance.reset();
}

void NaoProviderV6::finishFrame()
{
  if (*theInstance)
    (*theInstance)->send();
}

void NaoProviderV6::waitForFrameData()
{
  DEBUG_RESPONSE_ONCE("module:NaoProvider:robotName")
  {
    if (Global::getSettings().robotName == Global::getSettings().bodyName)
      OUTPUT_TEXT("Hi, I am " << Global::getSettings().robotName << ".");
    else
      OUTPUT_TEXT("Hi, I am " << Global::getSettings().robotName << " (using " << Global::getSettings().bodyName << "s Body).");

    Global::getSettings().write(Global::getDebugOut().bin);
    Global::getDebugOut().finishMessage(idRobotname);
  }

  const int oldSensorTimestamp = (*theInstance)->naoBody.getSensors().timestamp;
  if (firstSensorTimestamp == 0)
    firstSensorTimestamp = oldSensorTimestamp;

  if (*theInstance)
    (*theInstance)->naoBody.wait();

  const int newSensorTimestamp = (*theInstance)->naoBody.getSensors().timestamp;
  const int diffSensorTimestamp = newSensorTimestamp - oldSensorTimestamp;
  const int diffFirstSensorTimestamp = newSensorTimestamp - firstSensorTimestamp;

  if (firstSensorTimestamp > 0 && diffFirstSensorTimestamp > 10000 && newSensorTimestamp > 0 && oldSensorTimestamp > 0 && diffSensorTimestamp > 0.012f * 1000.f * 1.1f)
  {
    ANNOTATION("NaoProvider", "Skipped frame(s): Time difference was: " << diffSensorTimestamp << "ms");
    DEBUG_RESPONSE("module:NaoProviderV6:printSkippedFrames")
    OUTPUT_WARNING("NaoProvider: Skipped frame(s)! Time difference was: " << diffSensorTimestamp << "ms");
  }
}

void NaoProviderV6::send()
{
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:lag12") SystemCall::sleep(12);
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:lag100") SystemCall::sleep(100);
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:lag200") SystemCall::sleep(200);
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:lag300") SystemCall::sleep(300);
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:lag1000") SystemCall::sleep(1000);
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:lag3000") SystemCall::sleep(3000);
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:lag6000") SystemCall::sleep(6000);
  DEBUG_RESPONSE_ONCE("module:NaoProviderV6:segfault") * (volatile char*)0 = 0;

  NDData::ActuatorData& actuators = naoBody.openActuators();
  const JointRequest& theJointRequest = Blackboard::get<JointRequest>();

  /* POSITION */
  // apply joint decalibration and calibration
  std::array<Angle, Joints::Joint::numOfJoints> positions = theJointRequest.angles;
  for (size_t i = 0; i < positions.size(); ++i)
  {
    if (positions[i] != SensorData::off)
    {
      positions[i] += theJointDeCalibration.joints[i].offset + theJointCalibration.joints[i].offset;
    }
  }
  copyAndCastWithSourceMapping(positions, actuators.positions, jointsToBase);

  /* STIFFNESS */
  // map int [0...100] to float [0.f...1.f]
  std::array<float, Joints::Joint::numOfJoints> stiffnessData;
  for (size_t i = 0; i < Joints::Joint::numOfJoints; ++i)
    stiffnessData[i] = static_cast<float>(theJointRequest.stiffnessData.stiffnesses[i]) / 100.f;
  copyAndCastWithSourceMapping(stiffnessData, actuators.stiffness, jointsToBase);

  for (int i = 0; i < NDData::Joint::numOfJoints; ++i)
  {
    if (actuators.positions[i] == SensorData::off)
    {
      actuators.positions[i] = 0.f;
      actuators.stiffness[i] = -1.f;
    }
  }

  /* LEDs */
  actuators.chestLEDs = theLEDRequest.chest;
  actuators.lFootLEDs = theLEDRequest.leftFoot;
  actuators.rFootLEDs = theLEDRequest.rightFoot;

  for (size_t color = 0; color < NDData::RGBLED::numOfRGBLEDs; ++color)
    for (size_t led = 0; led < NDData::LEyeLED::numOfLEyeLEDs; ++led)
      actuators.lEyeLEDs[color][led] = theLEDRequest.leftEye[lEyeLEDsFromBase[led]][color];

  for (size_t color = 0; color < NDData::RGBLED::numOfRGBLEDs; ++color)
    for (size_t led = 0; led < NDData::REyeLED::numOfREyeLEDs; ++led)
      actuators.rEyeLEDs[color][led] = theLEDRequest.rightEye[rEyeLEDsFromBase[led]][color];

  copyAndCastWithTargetMapping(theLEDRequest.leftEar, actuators.lEarLEDs, lEarLEDsFromBase);
  copyAndCastWithTargetMapping(theLEDRequest.rightEar, actuators.rEarLEDs, rEarLEDsFromBase);
  copyAndCastWithTargetMapping(theLEDRequest.head, actuators.skullLEDs, skullLEDsFromBase);

  /* SONAR */
  actuators.sonars[0] = true;
  actuators.sonars[1] = true;

  naoBody.closeActuators();
}

void NaoProviderV6::update(FrameInfo& frameInfo)
{
  frameInfo.time = std::max(frameInfo.time + 1, SystemCall::getCurrentSystemTime());
  frameInfo.cycleTime = 0.012f;
}

void NaoProviderV6::update(FsrSensorData& fsrSensorData)
{
  const auto& fsr = naoBody.getSensors().fsr;
  unsigned currentTime = theFrameInfo.time;

  copyAndCastWithTargetMapping(fsr, fsrSensorData.left, lFsrToBase);
  copyAndCastWithTargetMapping(fsr, fsrSensorData.right, rFsrToBase);

  fsrSensorData.leftTotal = std::accumulate(fsrSensorData.left.begin(), fsrSensorData.left.end(), 0.f);
  fsrSensorData.rightTotal = std::accumulate(fsrSensorData.right.begin(), fsrSensorData.right.end(), 0.f);

  // update fsrRef during set
  const auto& angle = naoBody.getSensors().angle;
  if (theGameInfo.state != STATE_SET)
  {
    updateFsrRef = true;
    setState = false;
  }
  if (theGameInfo.state == STATE_SET && abs(angle[0]) < 10 && abs(angle[1]) < 10 && !setState)
  {
    setStarted = currentTime;
    setState = true;
  }

  if (theGameInfo.state == STATE_SET && abs(angle[0]) < 10 && abs(angle[1]) < 10 && currentTime - setStarted > 7000 && updateFsrRef)
  {
    fsrSensorData.fsrRef = fsrSensorData.leftTotal + fsrSensorData.rightTotal;
    updateFsrRef = false;
  }
}

void NaoProviderV6::update(InertialSensorData& inertialSensorData)
{
  const auto& gyro = naoBody.getSensors().gyro;
  const auto& acc = naoBody.getSensors().acc;
  const auto& angle = naoBody.getSensors().angle;

  inertialSensorData.gyro.x() = gyro[0];
  inertialSensorData.gyro.y() = gyro[1];
  inertialSensorData.gyro.z() = gyro[2];

  inertialSensorData.acc.x() = -acc[0];
  inertialSensorData.acc.y() = -acc[1];
  inertialSensorData.acc.z() = -acc[2];

  inertialSensorData.angle.x() = angle[0];
  inertialSensorData.angle.y() = angle[1];

  // remove gyro bias
  gyroBuffer.push_front(inertialSensorData.gyro);

  DECLARE_PLOT("module:NaoProviderV6:errSum");
  DECLARE_PLOT("module:NaoProviderV6:errSumThreshold");

  if (naoBody.getTransitionToFramework() == 0.f)
  {
    Vector3a average = gyroBuffer.average();
    Angle errSum = 0.f;
    for (unsigned i = 0; i < GYRO_BUFFER_LENGTH; i++)
      errSum += (gyroBuffer[i] - average).norm() * (gyroBuffer[i] - average).norm();

    if (errSum < (gyroBiasMaxErrorPerFrame * GYRO_BUFFER_LENGTH))
      gyroBias = average;

    if (gyroBias.norm() > gyroBiasThresholdForTTS && !soundPlayed)
    {
      soundPlayed = true;
      SystemCall::text2Speech("gyroscope bias calibrated");
      OUTPUT_WARNING("Gyroscope bias calibrated");
    }
    PLOT("module:NaoProviderV6:errSum", errSum.toDegrees());
    PLOT("module:NaoProviderV6:errSumThreshold", (gyroBiasMaxErrorPerFrame.toDegrees() * GYRO_BUFFER_LENGTH));
  }

  inertialSensorData.gyro -= gyroBias;

  PLOT("module:NaoProviderV6:gyroXBias", gyroBias.x().toDegrees());
  PLOT("module:NaoProviderV6:gyroYBias", gyroBias.y().toDegrees());
  PLOT("module:NaoProviderV6:gyroZBias", gyroBias.z().toDegrees());
}


void NaoProviderV6::update(JointSensorData& jointSensorData)
{
  const NDData::SensorData& sensors = naoBody.getSensors();

  auto currents = sensors.current;
  for (float& current : currents)
    current *= 1000.f;

  copyAndCastWithTargetMapping(sensors.position, jointSensorData.angles, jointsToBase);
  copyAndCastWithTargetMapping(currents, jointSensorData.currents, jointsToBase);
  copyAndCastWithTargetMapping(sensors.temperature, jointSensorData.temperatures, jointsToBase);
  copyAndCastWithTargetMapping(sensors.status, jointSensorData.status, jointsToBase);

  for (size_t i = 0; i < Joints::numOfJoints; i++)
  {
    jointSensorData.angles[i] -= theJointCalibration.joints[i].offset;
    if (i >= Joints::firstLeftLegJoint)
      jointSensorData.angles[i] -= theWalkCalibration.legJointCalibration[i - Joints::firstLeftLegJoint];
  }

  jointSensorData.timestamp = theFrameInfo.time;
}

void NaoProviderV6::update(KeyStates& keyStates)
{
  const auto& touch = naoBody.getSensors().touch;

  copyAndCastWithTargetMapping(touch, keyStates.pressed, keysToBase);
}

void NaoProviderV6::update(SystemSensorData& systemSensorData)
{
  const auto& battery = naoBody.getSensors().battery;
  if (theFrameInfo.getTimeSince(lastBodyTemperatureReadTime) > 10000)
  {
    lastBodyTemperatureReadTime = theFrameInfo.time;
    systemSensorData.cpuTemperature = naoBody.getCPUTemperature();
  }
  systemSensorData.batteryCurrent = battery[NDData::Battery::current];
  systemSensorData.batteryLevel = battery[NDData::Battery::charge];
  systemSensorData.batteryTemperature = battery[NDData::Battery::temperature];
  systemSensorData.chargingStatus = !(static_cast<int>(battery[NDData::Battery::status]) & 0b100000);
}

void NaoProviderV6::update(SonarSensorData& sonarSensorData)
{
  const auto& sonar = naoBody.getSensors().sonar;
  sonarSensorData.leftDistanceM = sonar.at(0);
  sonarSensorData.rightDistanceM = sonar.at(1);
  sonarSensorData.timestamp = theFrameInfo.time;
}

#endif // TARGET_ROBOT
