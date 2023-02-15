#include "Utils/dorsh/tools/Filesystem.h"

#include "Directory.h"
#include "Platform/File.h"
#include "Platform.h"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>
#include <fstream>
#include <QFile>

#include <QCoreApplication>
#include <QRegularExpression>

#if defined(LINUX) || defined(MACOS)
#include <cstdlib>
#include <sys/types.h>
#include <cerrno>
#endif

std::vector<std::string> Filesystem::getWlanConfigs()
{
  return getEntries(std::string(File::getBHDir()) + "/Config/WLAN/", true, false);
}

std::vector<std::string> Filesystem::getOverlays(const std::string prefix)
{
  return getEntries(std::string(File::getBHDir()) + "/Config/Overlays/" + prefix, false, true);
}

std::vector<std::string> Filesystem::getEntries(const std::string& directory, bool files, bool directories, const std::string& suffix, bool keepSuffixes)
{
  std::vector<std::string> entries;
  Directory d;
  if (d.open(directory + "*" + suffix))
  {
    std::string dir;
    bool isDir;
    while (d.read(dir, isDir))
      if (isDir == !files || isDir == directories)
      {
        std::string stripped = dir.replace(0, directory.length(), "");
        if (!keepSuffixes)
          stripped = stripped.replace(stripped.size() - suffix.size(), stripped.size(), "");
        if (stripped != "." && stripped != "..")
        {
          if (isDir && files)
            stripped += '/';
          entries.push_back(stripped);
        }
      }
  }
  else
    std::cout << "Cannot open " << directory << std::endl; // TODO log?
  return entries;
}

std::string Filesystem::getFileAsString(const std::string& filename)
{
  std::stringstream buf;
  std::ifstream fin(filename.c_str());

  char c;
  while (fin.good() && (c = (char)fin.get()) != EOF)
    buf << c;
  fin.close();

  return buf.str();
}

std::string Filesystem::getNaoKey()
{
  static std::string keyFile = std::string(File::getBHDir()) + linuxToPlatformPath("/Config/Keys/id_rsa_nao");

  QFile::setPermissions(QString::fromStdString(keyFile), QFile::ReadOwner); //set correct permissions, otherwise ssh will complain
  return keyFile;
}

bool Filesystem::isMultiConfig()
{
  // check if Dorsh binary is in multiconfig build directory
  const QString path = QCoreApplication::applicationDirPath();
  const QRegularExpression re("multiconfig\\/\\w+$");
  const QRegularExpressionMatch match = re.match(path);
  return match.hasMatch();
}
