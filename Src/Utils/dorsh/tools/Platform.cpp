#include "Utils/dorsh/cmdlib/ProcessRunner.h"
#include "Utils/dorsh/tools/Platform.h"
#include "Utils/dorsh/tools/StringTools.h"
#include <QDir>
#include <QRegExp>
#include <QString>

#ifdef WINDOWS
#define NOMINMAX
#include <Windows.h>
#endif

#if defined(OSX)
#include <iostream>
#include <cstdlib>
#endif

std::string bhumandDirOnRobot = "/home/nao/";

std::string makeDirectory()
{
#ifdef WINDOWS
  return "VS2019";
#elif defined(OSX)
  return "OSX";
#else
  return "Linux";
#endif
}

std::string platformDirectory()
{
#ifdef WINDOWS
  return "Windows";
#elif defined(OSX)
  return "OSX";
#else
  return "Linux";
#endif
}

void goToConfigDirectory(const char* argv0)
{
#ifdef WINDOWS
  char fileName[_MAX_PATH];
  char longFileName[_MAX_PATH];
  GetModuleFileNameA(GetModuleHandleA(0), fileName, _MAX_PATH);
  GetLongPathNameA(fileName, longFileName, _MAX_PATH);
  QString applicationPath = QString(longFileName);
  applicationPath = applicationPath.replace(QRegExp("Build\\\\\\w+\\\\dorsh\\\\\\w+\\\\dorsh.exe"), "");
#elif defined(OSX)
  QString applicationPath = QDir::cleanPath(*argv0 == '/' ? QString(argv0) : QDir::root().current().path() + "/" + argv0);
  applicationPath = applicationPath.replace(QRegExp("Build/\\w+/dorsh/\\w+/dorsh.app/Contents/MacOS/dorsh"), "");
#else
  QString applicationPath = QDir::cleanPath(*argv0 == '/' ? QString(argv0) : QDir::root().current().path() + "/" + argv0);
  applicationPath = applicationPath.replace(QRegExp("Build/\\w+/dorsh/\\w+/dorsh"), "");
#endif

  applicationPath += QString("Config");
  QDir::setCurrent(applicationPath);
}

std::string linuxToPlatformPath(const std::string& path)
{
#ifdef WINDOWS
  return toString(QString(path.c_str()).replace("/", "\\"));
#else // linux and mac
  return path;
#endif
}
