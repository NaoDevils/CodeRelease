#pragma once

#include <string>

extern std::string bhumandDirOnRobot; // TODO global configuration file
std::string platformDirectory();
void goToConfigDirectory(const char* argv0);
std::string linuxToPlatformPath(const std::string& path);
