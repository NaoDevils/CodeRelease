#include "cmdlib/ProcessRunner.h"
#include "tools/Platform.h"
#include <QDir>
#include <QRegularExpression>
#include <QString>

#include <QCoreApplication>

std::string bhumandDirOnRobot = "/home/nao/";

std::string platformDirectory()
{
#ifdef WINDOWS
  return "Windows";
#else
  return "Linux";
#endif
}

void goToConfigDirectory(const char* argv0)
{
  QString applicationPath = QCoreApplication::applicationFilePath();
  applicationPath = applicationPath.replace(QRegularExpression("Build/.+?/Dorsh.*?$"), "");

  applicationPath += QString("Config");
  QDir::setCurrent(applicationPath);
}

std::string linuxToPlatformPath(const std::string& path)
{
#ifdef WINDOWS
  return QString::fromStdString(path).replace("/", "\\").toStdString();
#else // linux and mac
  return path;
#endif
}
