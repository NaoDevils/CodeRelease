/**
* @file Platform/Common/File.cpp
* Declaration of class File for Windows and Linux.
*/

#ifdef WINDOWS
#include <windows.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#endif
#include <cstring>

#include "Platform/BHAssert.h"
#include "File.h"
#ifndef TARGET_TOOL
#include "Tools/Global.h"
#include "Tools/Settings.h"
#endif

#include "Tools/Configuration/RobotConfig.h"
#include "Tools/Streams/InStreams.h"
#include <filesystem>

File::File(const std::string& name, const char* mode, bool tryAlternatives)
    : stream(0)
#ifdef WINDOWS
      ,
      hFile(INVALID_HANDLE_VALUE)
#endif
{
  fullName = name;
  std::list<std::string> names = getFullNames(name);
  if (tryAlternatives)
  {
    for (auto& path : names)
    {
#if (defined(WINDOWS) || defined(MACOS))
      stream = fopen(path.c_str(), mode);
#else
      stream = fopen64(path.c_str(), mode);
#endif
      if (stream)
      {
        fullName = path;
        break;
      }
    }
  }
  else
  {
#if (defined(WINDOWS) || defined(MACOS))
    stream = fopen(names.back().c_str(), mode);
#else
    stream = fopen64(names.back().c_str(), mode);
#endif

    if (stream)
      fullName = names.back();
  }
}

std::list<std::string> File::getFullNames(const std::string& rawName)
{
#if (defined(LINUX) || defined(MACOS)) && (defined(TARGET_SIM) || defined(TARGET_TOOL))
  std::string name(rawName);
  for (int i = (int)name.length(); i >= 0; i--)
    if (name[i] == '\\')
      name[i] = '/';
#else
  const std::string& name(rawName);
#endif

  std::list<std::string> names;
  if ((name[0] != '.' || (name.size() >= 2 && name[1] == '.')) && !isAbsolute(name.c_str())) // given path is relative to getBHDir()
  {
    names = getConfigDirectories();
    const std::string prefix = std::string(getBHDir()) + "/Config/";
    for (std::string& path : names)
      path = prefix + path + name;
  }
  else
    names.push_back(name);
  return names;
}

std::list<std::string> File::getConfigDirectories()
{
  std::list<std::string> directories;
  std::list<std::string> subpaths;
  std::vector<std::string> overlays;
#ifndef TARGET_TOOL
  if (Global::settingsExist())
  {
    subpaths.push_back("Robots/" + Global::getSettings().robotName + "/" + Global::getSettings().bodyName + "/");
    subpaths.push_back("Robots/" + Global::getSettings().robotName + "/Head/");
    subpaths.push_back("Robots/" + Global::getSettings().bodyName + "/Body/");
    subpaths.push_back("Robots/" + std::string(RobotConfig::getName(Global::getSettings().naoVersion)) + "/");

    overlays = Global::getSettings().overlays;
  }
#endif
  subpaths.push_back("");

  for (std::string& overlay : overlays)
    overlay = "Overlays/" + overlay + "/";
  overlays.push_back("");

  for (const std::string& overlay : overlays)
    for (const std::string& subpath : subpaths)
      directories.emplace_back(overlay + subpath);

  return directories;
}

std::list<std::string> File::getFullNamesHierarchy(const std::string& name)
{
  std::list<std::string> names = getFullNames(name);

  auto fileDoesNotExist = [](const std::string& name)
  {
#if (defined(WINDOWS) || defined(MACOS))
    FILE* file = fopen(name.c_str(), "r");
#else
    FILE* file = fopen64(name.c_str(), "r");
#endif
    if (file)
    {
      fclose(file);
      return false;
    }
    else
    {
      return true;
    }
  };

  names.remove_if(fileDoesNotExist);
  names.reverse();
  return names;
}

File::~File()
{
#ifdef WINDOWS
  if (map != NULL)
  {
    UnmapViewOfFile(map);
    map = NULL;
  }
  if (hFileMapping != NULL)
  {
    CloseHandle(hFileMapping);
    hFileMapping = NULL;
  }
  if (hFile != INVALID_HANDLE_VALUE)
  {
    CloseHandle(hFile);
    hFile = INVALID_HANDLE_VALUE;
  }
#else
  if (map != nullptr)
  {
    ASSERT(munmap(map, size) == 0);
    map = nullptr;
    size = 0;
  }
#endif
  if (stream != 0)
    fclose((FILE*)stream);
}

void File::read(void* p, size_t size)
{
  VERIFY(!eof());
  VERIFY(fread(p, 1, size, (FILE*)stream) > 0);
}

char* File::readLine(char* p, size_t size)
{
  VERIFY(!eof());
  return fgets(p, static_cast<int>(size), (FILE*)stream);
}

void File::write(const void* p, size_t size)
{
  //if opening failed, stream will be 0 and fwrite would crash
  ASSERT(stream != 0);
#ifdef NDEBUG
  static_cast<void>(fwrite(p, 1, size, (FILE*)stream));
#else
  const size_t written = fwrite(p, 1, size, (FILE*)stream);
  if (written != size)
  {
    perror("fwrite did not write as many bytes as requested");
    //FAIL("File::write failed!");
  }
#endif
}

void File::printf(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  vfprintf((FILE*)stream, format, args);
  va_end(args);
}

bool File::eof()
{
  //never use feof(stream), because it informs you only after reading
  //too far and is never reset, e.g. if the stream grows afterwards,
  //our implementation can handle both correctly:
  if (!stream)
    return false;
  else
  {
    int c = fgetc((FILE*)stream);
    if (c == EOF)
      return true;
    else
    {
      VERIFY(ungetc(c, (FILE*)stream) != EOF);
      return false;
    }
  }
}

size_t File::getSize()
{
  if (!stream)
    return 0;
  else
  {
    FILE* fs = static_cast<FILE*>(stream);
#ifdef WINDOWS
    const long long currentPos = _ftelli64(fs);
    ASSERT(currentPos >= 0);
    VERIFY(_fseeki64(fs, 0, SEEK_END) == 0);
    const long long size = _ftelli64(fs);
    ASSERT(size >= 0);
    VERIFY(_fseeki64(fs, currentPos, SEEK_SET) == 0);
#elif defined(MACOS)
    const off_t currentPos = ftello(fs);
    ASSERT(currentPos >= 0);
    VERIFY(fseeko(fs, 0, SEEK_END) == 0);
    const off_t size = ftell(fs);
    ASSERT(size >= 0);
    VERIFY(fseeko(fs, currentPos, SEEK_SET) == 0);
#else
    const off64_t currentPos = ftello64(fs);
    ASSERT(currentPos >= 0);
    VERIFY(fseeko64(fs, 0, SEEK_END) == 0);
    const off64_t size = ftell(fs);
    ASSERT(size >= 0);
    VERIFY(fseeko64(fs, currentPos, SEEK_SET) == 0);
#endif
    return size;
  }
}

char* File::getMemoryMappedFile()
{
  if (map != nullptr)
    return map;

#ifdef WINDOWS
  hFile = CreateFile(getFullName().c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
  ASSERT(hFile != INVALID_HANDLE_VALUE);
  hFileMapping = CreateFileMapping(hFile, NULL, PAGE_READONLY, 0, 0, NULL);
  ASSERT(hFileMapping != NULL);
  LPVOID lpMap = MapViewOfFile(hFileMapping, FILE_MAP_READ, 0, 0, 0);
  ASSERT(lpMap != NULL);
  map = reinterpret_cast<char*>(lpMap);
#else
  int fd = open(getFullName().c_str(), O_RDONLY);
  ASSERT(fd >= 0);

  struct stat statbuf;
  VERIFY(fstat(fd, &statbuf) >= 0);
  size = statbuf.st_size; // remember size for munmap

  void* ptr = mmap(nullptr, statbuf.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
  ASSERT(ptr != MAP_FAILED);

  close(fd);

  map = reinterpret_cast<char*>(ptr);
#endif

  return map;
}

size_t File::getPosition()
{
#ifdef WINDOWS
  return _ftelli64(static_cast<FILE*>(stream));
#elif defined(MACOS)
  return ftello(static_cast<FILE*>(stream));
#else
  return ftello64(static_cast<FILE*>(stream));
#endif
}

#ifdef WINDOWS
const char* File::getBHDir()
{
  static char dir[MAX_PATH] = {0};
  if (!dir[0])
  {
#if defined(TARGET_SIM) || defined(TARGET_TOOL)

    // determine module file from command line
    const char* commandLine = GetCommandLine();
    size_t len;
    if (*commandLine == '"')
    {
      commandLine++;
      const char* end = strchr(commandLine, '"');
      if (end)
        len = end - commandLine;
      else
        len = strlen(commandLine);
    }
    else
      len = strlen(commandLine);
    if (len >= sizeof(dir) - 8)
      len = sizeof(dir) - 8;
    memcpy(dir, commandLine, len);
    dir[len] = '\0';

    // if there is no given directory, use the current working dir
    if (!strchr(dir, '\\'))
    {
      len = int(GetCurrentDirectory(sizeof(dir) - 9, dir));
      if (len && dir[len - 1] != '\\')
      {
        dir[len++] = '\\';
        dir[len] = '\0';
      }
    }

    //drive letter in lower case:
    if (len && dir[1] == ':')
      *dir |= tolower(*dir);

    // try to find the config directory
    char* end = dir + len - 1;
    for (;;)
    {
      if (*end == '/' || *end == '\\' || *end == ':' || end == dir - 1)
      {
        if (*end == ':')
          *(end++) = '\\';
        strcpy(end + 1, "Config");
        DWORD attr = GetFileAttributes(dir);
        if (attr != INVALID_FILE_ATTRIBUTES && attr & FILE_ATTRIBUTE_DIRECTORY)
        {
          end[end > dir ? 0 : 1] = '\0';
          for (; end >= dir; end--)
            if (*end == '\\')
              *end = '/';
          return dir;
        }
      }
      if (end < dir)
        break;
      end--;
    }
    ASSERT(false);
#else
    strcpy(dir, ".");
#endif
  }
  return dir;
}
#endif

#if defined(LINUX) || defined(MACOS)
const char* File::getBHDir()
{
  static char dir[FILENAME_MAX] = {0};
  if (!dir[0])
  {
#if defined(TARGET_SIM) || defined(TARGET_TOOL)
    VERIFY(getcwd(dir, sizeof(dir) - 7) != 0);
    char* end = dir + strlen(dir) - 1;
    struct stat buff;
    ;
    for (;;)
    {
      if (*end == '/' || end == dir - 1)
      {
        strcpy(end + 1, "Config");
        if (stat(dir, &buff) == 0)
          if (S_ISDIR(buff.st_mode))
          {
            end[end > dir ? 0 : 1] = '\0';
            return dir;
          }
      }
      if (end < dir)
        break;
      end--;
    }
#endif
    strcpy(dir, ".");
  }
  return dir;
}
#endif

#ifndef TARGET_TOOL
std::string File::getPersistentDir()
{
  const std::string persistentDir = std::string(File::getBHDir()) + "/Persistent/" + Global::getSettings().bodyName + "/";
  std::filesystem::create_directories(persistentDir);
  return persistentDir;
}
#endif

bool File::isAbsolute(const char* path)
{
  return (path[0] && path[1] == ':') || path[0] == '/' || path[0] == '\\';
}
