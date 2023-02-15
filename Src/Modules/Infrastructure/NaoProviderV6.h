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
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
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

  PROVIDES(FsrSensorData),
  PROVIDES(InertialSensorData),
  PROVIDES(JointSensorData),
  PROVIDES(KeyStates),
  PROVIDES(SystemSensorData),
  USES(JointRequest), // Will be accessed in send()
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

  static constexpr std::array<LEDRequest::LED, NDData::RGBLED::numOfRGBLEDs> chestLEDsFromBase = {LEDRequest::LED::chestRed, LEDRequest::LED::chestGreen, LEDRequest::LED::chestBlue};
  static constexpr std::array<LEDRequest::LED, NDData::LEarLED::numOfLEarLEDs> lEarLEDsFromBase = {LEDRequest::LED::earsLeft0Deg,
      LEDRequest::LED::earsLeft36Deg,
      LEDRequest::LED::earsLeft72Deg,
      LEDRequest::LED::earsLeft108Deg,
      LEDRequest::LED::earsLeft144Deg,
      LEDRequest::LED::earsLeft180Deg,
      LEDRequest::LED::earsLeft216Deg,
      LEDRequest::LED::earsLeft252Deg,
      LEDRequest::LED::earsLeft288Deg,
      LEDRequest::LED::earsLeft324Deg};
  static constexpr std::array<std::array<LEDRequest::LED, NDData::LEyeLED::numOfLEyeLEDs>, NDData::RGBLED::numOfRGBLEDs> lEyeLEDsFromBase = {
      {{LEDRequest::LED::faceLeftRed45Deg,
           LEDRequest::LED::faceLeftRed0Deg,
           LEDRequest::LED::faceLeftRed315Deg,
           LEDRequest::LED::faceLeftRed270Deg,
           LEDRequest::LED::faceLeftRed225Deg,
           LEDRequest::LED::faceLeftRed180Deg,
           LEDRequest::LED::faceLeftRed135Deg,
           LEDRequest::LED::faceLeftRed90Deg},
          {LEDRequest::LED::faceLeftGreen45Deg,
              LEDRequest::LED::faceLeftGreen0Deg,
              LEDRequest::LED::faceLeftGreen315Deg,
              LEDRequest::LED::faceLeftGreen270Deg,
              LEDRequest::LED::faceLeftGreen225Deg,
              LEDRequest::LED::faceLeftGreen180Deg,
              LEDRequest::LED::faceLeftGreen135Deg,
              LEDRequest::LED::faceLeftGreen90Deg},
          {LEDRequest::LED::faceLeftBlue45Deg,
              LEDRequest::LED::faceLeftBlue0Deg,
              LEDRequest::LED::faceLeftBlue315Deg,
              LEDRequest::LED::faceLeftBlue270Deg,
              LEDRequest::LED::faceLeftBlue225Deg,
              LEDRequest::LED::faceLeftBlue180Deg,
              LEDRequest::LED::faceLeftBlue135Deg,
              LEDRequest::LED::faceLeftBlue90Deg}}};
  static constexpr std::array<LEDRequest::LED, NDData::RGBLED::numOfRGBLEDs> lFootLEDsFromBase = {LEDRequest::LED::footLeftRed, LEDRequest::LED::footLeftGreen, LEDRequest::LED::footLeftBlue};
  static constexpr std::array<LEDRequest::LED, NDData::REarLED::numOfREarLEDs> rEarLEDsFromBase = {LEDRequest::LED::earsRight324Deg,
      LEDRequest::LED::earsRight288Deg,
      LEDRequest::LED::earsRight252Deg,
      LEDRequest::LED::earsRight216Deg,
      LEDRequest::LED::earsRight180Deg,
      LEDRequest::LED::earsRight144Deg,
      LEDRequest::LED::earsRight108Deg,
      LEDRequest::LED::earsRight72Deg,
      LEDRequest::LED::earsRight36Deg,
      LEDRequest::LED::earsRight0Deg};
  static constexpr std::array<std::array<LEDRequest::LED, NDData::REyeLED::numOfREyeLEDs>, NDData::RGBLED::numOfRGBLEDs> rEyeLEDsFromBase = {
      {{LEDRequest::LED::faceRightRed0Deg,
           LEDRequest::LED::faceRightRed45Deg,
           LEDRequest::LED::faceRightRed90Deg,
           LEDRequest::LED::faceRightRed135Deg,
           LEDRequest::LED::faceRightRed180Deg,
           LEDRequest::LED::faceRightRed225Deg,
           LEDRequest::LED::faceRightRed270Deg,
           LEDRequest::LED::faceRightRed315Deg},
          {LEDRequest::LED::faceRightGreen0Deg,
              LEDRequest::LED::faceRightGreen45Deg,
              LEDRequest::LED::faceRightGreen90Deg,
              LEDRequest::LED::faceRightGreen135Deg,
              LEDRequest::LED::faceRightGreen180Deg,
              LEDRequest::LED::faceRightGreen225Deg,
              LEDRequest::LED::faceRightGreen270Deg,
              LEDRequest::LED::faceRightGreen315Deg},
          {LEDRequest::LED::faceRightBlue0Deg,
              LEDRequest::LED::faceRightBlue45Deg,
              LEDRequest::LED::faceRightBlue90Deg,
              LEDRequest::LED::faceRightBlue135Deg,
              LEDRequest::LED::faceRightBlue180Deg,
              LEDRequest::LED::faceRightBlue225Deg,
              LEDRequest::LED::faceRightBlue270Deg,
              LEDRequest::LED::faceRightBlue315Deg}}};
  static constexpr std::array<LEDRequest::LED, NDData::RGBLED::numOfRGBLEDs> rFootLEDsFromBase = {LEDRequest::LED::footRightRed, LEDRequest::LED::footRightGreen, LEDRequest::LED::footRightBlue};
  static constexpr std::array<LEDRequest::LED, NDData::SkullLED::numOfSkullLEDs> skullLEDsFromBase = {LEDRequest::LED::headLedFrontLeft0,
      LEDRequest::LED::headLedFrontLeft1,
      LEDRequest::LED::headLedMiddleLeft0,
      LEDRequest::LED::headLedRearLeft0,
      LEDRequest::LED::headLedRearLeft1,
      LEDRequest::LED::headLedRearLeft2,
      LEDRequest::LED::headLedRearRight0,
      LEDRequest::LED::headLedRearRight1,
      LEDRequest::LED::headLedRearRight2,
      LEDRequest::LED::headLedMiddleRight0,
      LEDRequest::LED::headLedFrontRight0,
      LEDRequest::LED::headLedFrontRight1};

  static_assert(chestLEDsFromBase.size() + lEarLEDsFromBase.size() + lEyeLEDsFromBase.size() * lEyeLEDsFromBase[0].size() + lFootLEDsFromBase.size() + rEarLEDsFromBase.size()
              + rEyeLEDsFromBase.size() * rEyeLEDsFromBase[0].size() + rFootLEDsFromBase.size() + skullLEDsFromBase.size()
          == static_cast<size_t>(LEDRequest::LED::numOfLEDs),
      "Unexpected number of LEDs");

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
  void send();

public:
  static void finishFrame() {}
  static void waitForFrameData() {}
};

#endif
