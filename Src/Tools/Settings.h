/**
 * @file Tools/Settings.h
 * Definition of a class that provides access to settings-specific configuration directories.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Configuration/RobotConfig.h"

/**
 * @class Settings
 * The class provides access to settings-specific configuration directories.
 */
STREAMABLE(Settings,
  std::string robotName = "Nao"; /**< The name of this robot. */
  std::string bodyName = "Nao"; /**< The name of this robot's body. */

  /**
   * The function loads the settings from disk.
   * @return Whether the settings were loaded successfully.
   */
  bool load();

  void write(Out& out) const;
  void read(In& in);

  // Increase version number whenever something changes!
  static constexpr unsigned char version = 1;
  ,
  ((RobotConfig) NaoVersion)(NaoVersion::V6) naoVersion,
  (std::vector<std::string>) overlays /**< The activated config overlays. */
);
