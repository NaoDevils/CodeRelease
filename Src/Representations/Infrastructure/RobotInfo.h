/**
 * @file RobotInfo.h
 * The file declares a struct that encapsulates the structure RobotInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * It also maps the robot's name on the robot's model.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <a href="mailto:aschreck@informatik.uni-bremen.de">André Schreck</a>
 */

#pragma once

#include "RoboCupGameControlData.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Enum.h"
#include "Tools/Settings.h"

struct RobotInfo : public RoboCup::RobotInfo, public Streamable
{
public:
  ENUM(NaoType,
    H21,
    H25
  ); // need to be sorted

  ENUM(RobotFeature,
    hands,
    grippyFingers,
    wristYaws,
    zGyro,
    tactileHandSensores,
    tactileHeadSensores,
    headLEDs
  );

  int number; /**< The number of the robot. */
  RobotConfig::NaoVersion naoVersion = RobotConfig::V6;
  NaoType naoBodyType = H25;
  NaoType naoHeadType = H25;
  float transitionToFramework = 1.f; /** If naodevilsbase has given full control to framework. (0 = naodevilsbase, 1 = framework) Range: [0.0, 1.0] */

  RobotInfo();

  bool hasFeature(const RobotFeature feature) const;
  std::string getPenaltyAsString() const;

  Streamable& operator=(const Streamable& other) noexcept;

private:
  /**
   * The method makes the object streamable.
   * @param in The stream from which the object is read (if in != 0).
   * @param out The stream to which the object is written (if out != 0).
   */
  virtual void serialize(In* in, Out* out);
};
