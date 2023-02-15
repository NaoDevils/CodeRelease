/**
* \file Directory.cpp
* Implements a class for accessing directories.
* \author Colin Graf
*/

#ifdef WINDOWS
#include <Shlwapi.h>
#pragma comment(lib, "shlwapi.lib")
#else
#include <unistd.h>
#include <fnmatch.h>
#endif

#include "Directory.h"
#include "Platform/BHAssert.h"


bool Directory::open(const std::string& pattern)
{
  size_t end = pattern.find_last_of("/\\");
  if (end == std::string::npos)
    return false;
  filepattern = pattern.substr(end + 1);
  dirname = pattern.substr(0, end);

  try
  {
    di = std::filesystem::directory_iterator(dirname);
    return true;
  }
  catch (...)
  {
    return false;
  }
}

bool Directory::read(std::string& name, bool& isDir)
{
  while (di != std::filesystem::directory_iterator())
  {
#ifdef WINDOWS
    if (PathMatchSpec(di->path().filename().string().c_str(), filepattern.c_str()))
#else
    if (fnmatch(filepattern.c_str(), di->path().filename().string().c_str(), 0) == 0)
#endif
    {
      name = di->path().string();
      isDir = di->is_directory();

      ++di;
      return true;
    }
  }
  return false;
}
