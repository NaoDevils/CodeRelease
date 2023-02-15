#pragma once

namespace Build
{
  enum class Platform
  {
    Windows,
    Linux,
    MacOS
  };

  enum class Type
  {
    Debug,
    Develop,
    Release
  };

  enum class Target
  {
    Robot,
    Simulator,
    Tool
  };


#ifdef WINDOWS
  static constexpr Platform platform = Platform::Windows;
#elif defined(LINUX)
  static constexpr Platform platform = Platform::Linux;
#elif defined(MACOS)
  static constexpr Platform platform = Platform::MacOS;
#else
  static_assert(false, "Unknown build platform");
#endif

  static constexpr Type type = Type::CONFIGURATION;

#ifdef TARGET_ROBOT
  static constexpr Target target = Target::Robot;
#elif defined(TARGET_TOOL)
  static constexpr Target target = Target::Tool;
#else
  static constexpr Target target = Target::Simulator;
#endif

  constexpr bool platformWindows()
  {
    return platform == Platform::Windows;
  }
  constexpr bool platformLinux()
  {
    return platform == Platform::Linux;
  }
  constexpr bool platformMacOS()
  {
    return platform == Platform::MacOS;
  }

  constexpr bool typeDebug()
  {
    return type == Type::Debug;
  }
  constexpr bool typeDevelop()
  {
    return type == Type::Develop;
  }
  constexpr bool typeRelease()
  {
    return type == Type::Release;
  }

  constexpr bool targetRobot()
  {
    return target == Target::Robot;
  }
  constexpr bool targetSimulator()
  {
    return target == Target::Simulator;
  }
  constexpr bool targetTool()
  {
    return target == Target::Tool;
  }
} // namespace Build
