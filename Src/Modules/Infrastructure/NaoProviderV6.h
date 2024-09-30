/**
 * @file Modules/Infrastructure/NaoProviderV6.h
 * The file declares a module that provides information from the Nao via LoLA.
 * @author TheCanadianGuy & schwingmar
 */

#pragma once

#include "Platform/Linux/NaoBodyV6.h"
#include "Representations/Configuration/JointDeCalibration.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/SensorData/SonarSensorData.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/ProcessFramework/CycleLocal.h"

constexpr unsigned GYRO_BUFFER_LENGTH = static_cast<unsigned>(1.0 * 83);

MODULE(NaoProviderV6,
  REQUIRES(JointDeCalibration),
  REQUIRES(JointCalibration),
  REQUIRES(LEDRequest),

  PROVIDES(FrameInfo),
  REQUIRES(FrameInfo),

  REQUIRES(GameInfo),

  PROVIDES(FsrSensorData),
  PROVIDES(InertialSensorData),
  PROVIDES(JointSensorData),
  PROVIDES(KeyStates),
  PROVIDES(SystemSensorData),
  PROVIDES(SonarSensorData),
  USES(WalkCalibration),

  // Would create circular dependency, will be accessed directly in send()
  // REQUIRES(JointRequest),
  LOADS_PARAMETERS(,
    (Angle)(0.00125_deg) gyroBiasMaxErrorPerFrame,
    (Angle)(0.25_deg) gyroBiasThresholdForTTS
  )
);

#ifdef TARGET_ROBOT

/**
 * @class NaoProviderV6
 * A module that provides information from the Nao.
 */
class NaoProviderV6 : public NaoProviderV6Base
{
private:
  static CycleLocal<NaoProviderV6*> theInstance; /**< The only instance of this module. */

  NaoBodyV6 naoBody;
  float clippedLastFrame[Joints::numOfJoints]; /**< Array that indicates whether a certain joint value was clipped in the last frame (and what was the value)*/
  RingBufferWithSum<Vector3a, GYRO_BUFFER_LENGTH> gyroBuffer{Vector3a::Zero()};
  Vector3a gyroBias = Vector3a::Zero();
  bool soundPlayed = false;
  bool updateFsrRef = true;
  bool setState = false;
  unsigned setStarted = 0;
  unsigned lastBodyTemperatureReadTime = 0;
  static int firstSensorTimestamp;

  static constexpr std::array<NDData::Joint, Joints::Joint::numOfJoints> jointsToBase = {NDData::Joint::headYaw,
      NDData::Joint::headPitch,
      NDData::Joint::lShoulderPitch,
      NDData::Joint::lShoulderRoll,
      NDData::Joint::lElbowYaw,
      NDData::Joint::lElbowRoll,
      NDData::Joint::lWristYaw,
      NDData::Joint::lHand,
      NDData::Joint::rShoulderPitch,
      NDData::Joint::rShoulderRoll,
      NDData::Joint::rElbowYaw,
      NDData::Joint::rElbowRoll,
      NDData::Joint::rWristYaw,
      NDData::Joint::rHand,
      NDData::Joint::lHipYawPitch,
      NDData::Joint::lHipRoll,
      NDData::Joint::lHipPitch,
      NDData::Joint::lKneePitch,
      NDData::Joint::lAnklePitch,
      NDData::Joint::lAnkleRoll,
      NDData::Joint::lHipYawPitch, // same value!
      NDData::Joint::rHipRoll,
      NDData::Joint::rHipPitch,
      NDData::Joint::rKneePitch,
      NDData::Joint::rAnklePitch,
      NDData::Joint::rAnkleRoll};
  static_assert(jointsToBase.size() - 1 == static_cast<size_t>(NDData::Joint::numOfJoints), "Unexpected number of joints");

  static constexpr std::array<NDData::Touch, KeyStates::Key::numOfKeys> keysToBase = {NDData::Touch::headFront,
      NDData::Touch::headMiddle,
      NDData::Touch::headRear,
      NDData::Touch::lHandBack,
      NDData::Touch::lHandLeft,
      NDData::Touch::lHandRight,
      NDData::Touch::rHandBack,
      NDData::Touch::rHandLeft,
      NDData::Touch::rHandRight,
      NDData::Touch::lBumperLeft,
      NDData::Touch::lBumperRight,
      NDData::Touch::rBumperLeft,
      NDData::Touch::rBumperRight,
      NDData::Touch::chestButton};
  static_assert(keysToBase.size() == static_cast<size_t>(NDData::Touch::numOfTouchs), "Unexpected number of keys");

  static constexpr std::array<NDData::FSR, FsrSensorData::FsrSensorPosition::numOfFsrSensorPositions> lFsrToBase = {
      NDData::FSR::lFrontLeft, NDData::FSR::lFrontRight, NDData::FSR::lRearLeft, NDData::FSR::lRearRight};
  static constexpr std::array<NDData::FSR, FsrSensorData::FsrSensorPosition::numOfFsrSensorPositions> rFsrToBase = {
      NDData::FSR::rFrontLeft, NDData::FSR::rFrontRight, NDData::FSR::rRearLeft, NDData::FSR::rRearRight};
  static_assert(lFsrToBase.size() + rFsrToBase.size() == static_cast<size_t>(NDData::FSR::numOfFsrs), "Unexpected number of FSR positions");

  static constexpr std::array<LEDRequest::EyeLED, NDData::LEyeLED::numOfLEyeLEDs> lEyeLEDsFromBase = {LEDRequest::EyeLED::eye45Deg,
      LEDRequest::EyeLED::eye0Deg,
      LEDRequest::EyeLED::eye315Deg,
      LEDRequest::EyeLED::eye270Deg,
      LEDRequest::EyeLED::eye225Deg,
      LEDRequest::EyeLED::eye180Deg,
      LEDRequest::EyeLED::eye135Deg,
      LEDRequest::EyeLED::eye90Deg};
  static constexpr std::array<LEDRequest::EyeLED, NDData::REyeLED::numOfREyeLEDs> rEyeLEDsFromBase = {LEDRequest::EyeLED::eye0Deg,
      LEDRequest::EyeLED::eye45Deg,
      LEDRequest::EyeLED::eye90Deg,
      LEDRequest::EyeLED::eye135Deg,
      LEDRequest::EyeLED::eye180Deg,
      LEDRequest::EyeLED::eye225Deg,
      LEDRequest::EyeLED::eye270Deg,
      LEDRequest::EyeLED::eye315Deg};
  static_assert(static_cast<size_t>(NDData::LEyeLED::numOfLEyeLEDs) == static_cast<size_t>(LEDRequest::EyeLED::numOfEyeLEDs));
  static_assert(static_cast<size_t>(NDData::REyeLED::numOfREyeLEDs) == static_cast<size_t>(LEDRequest::EyeLED::numOfEyeLEDs));

  static constexpr std::array<LEDRequest::EarLED, NDData::LEarLED::numOfLEarLEDs> lEarLEDsFromBase = {LEDRequest::EarLED::ear0Deg,
      LEDRequest::EarLED::ear36Deg,
      LEDRequest::EarLED::ear72Deg,
      LEDRequest::EarLED::ear108Deg,
      LEDRequest::EarLED::ear144Deg,
      LEDRequest::EarLED::ear180Deg,
      LEDRequest::EarLED::ear216Deg,
      LEDRequest::EarLED::ear252Deg,
      LEDRequest::EarLED::ear288Deg,
      LEDRequest::EarLED::ear324Deg};
  static constexpr std::array<LEDRequest::EarLED, NDData::REarLED::numOfREarLEDs> rEarLEDsFromBase = {LEDRequest::EarLED::ear324Deg,
      LEDRequest::EarLED::ear288Deg,
      LEDRequest::EarLED::ear252Deg,
      LEDRequest::EarLED::ear216Deg,
      LEDRequest::EarLED::ear180Deg,
      LEDRequest::EarLED::ear144Deg,
      LEDRequest::EarLED::ear108Deg,
      LEDRequest::EarLED::ear72Deg,
      LEDRequest::EarLED::ear36Deg,
      LEDRequest::EarLED::ear0Deg};
  static_assert(static_cast<size_t>(NDData::LEarLED::numOfLEarLEDs) == static_cast<size_t>(LEDRequest::EarLED::numOfEarLEDs));
  static_assert(static_cast<size_t>(NDData::REarLED::numOfREarLEDs) == static_cast<size_t>(LEDRequest::EarLED::numOfEarLEDs));

  static constexpr std::array<LEDRequest::HeadLED, NDData::SkullLED::numOfSkullLEDs> skullLEDsFromBase = {LEDRequest::HeadLED::frontLeft0,
      LEDRequest::HeadLED::frontLeft1,
      LEDRequest::HeadLED::middleLeft0,
      LEDRequest::HeadLED::rearLeft0,
      LEDRequest::HeadLED::rearLeft1,
      LEDRequest::HeadLED::rearLeft2,
      LEDRequest::HeadLED::rearRight0,
      LEDRequest::HeadLED::rearRight1,
      LEDRequest::HeadLED::rearRight2,
      LEDRequest::HeadLED::middleRight0,
      LEDRequest::HeadLED::frontRight0,
      LEDRequest::HeadLED::frontRight1};
  static_assert(static_cast<size_t>(NDData::SkullLED::numOfSkullLEDs) == static_cast<size_t>(LEDRequest::HeadLED::numOfHeadLEDs));


public:
  NaoProviderV6();
  ~NaoProviderV6();

  /** The method is called by process Motion to send the requests to the Nao. */
  static void finishFrame();
  static void waitForFrameData();

private:
  void update(FrameInfo& frameInfo);
  void update(FsrSensorData& fsrSensorData);
  void update(InertialSensorData& inertialSensorData);
  void update(JointSensorData& jointSensorData);
  void update(KeyStates& keyStates);
  void update(SystemSensorData& systemSensorData);
  void update(SonarSensorData& sonarSensorData);

  /** The function sends a command to the Nao. */
  void send();

  template <typename S, size_t sizeS, typename T, size_t sizeT, typename M>
  constexpr void copyAndCastWithSourceMapping(const std::array<S, sizeS>& source, std::array<T, sizeT>& target, const std::array<M, sizeS>& mapping)
  {
    for (size_t i = 0; i < source.size(); ++i)
      target[mapping[i]] = static_cast<T>(source[i]);
  }

  template <typename S, size_t sizeS, typename T, size_t sizeT, typename M>
  constexpr void copyAndCastWithTargetMapping(const std::array<S, sizeS>& source, std::array<T, sizeT>& target, const std::array<M, sizeT>& mapping)
  {
    for (size_t i = 0; i < target.size(); ++i)
      target[i] = static_cast<T>(source[mapping[i]]);
  }
};

#else
//TARGET_ROBOT not defined here (Simulator).

class NaoProviderV6 : public NaoProviderV6Base
{
private:
  void update(FrameInfo& frameInfo) {}
  void update(FsrSensorData& fsrSensorData) {}
  void update(InertialSensorData& inertialSensorData) {}
  void update(JointSensorData& jointSensorData) {}
  void update(KeyStates& keyStates) {}
  void update(SystemSensorData& systemSensorData) {}
  void update(SonarSensorData& sonarSensorData) {}
  void send();

public:
  static void finishFrame() {}
  static void waitForFrameData() {}
};

#endif
