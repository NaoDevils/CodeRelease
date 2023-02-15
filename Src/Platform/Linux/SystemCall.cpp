/**
* @file Platform/linux/SystemCall.cpp
*
* Implementation of system calls and access to thread local storage.
* Only for use on Linux.
*
* @author <A href=mailto:brunn@sim.informatik.tu-darmstadt.de>Ronnie Brunn</A>
* @author <A href=mailto:martin@martin-loetzsch.de>Martin Lötzsch</A>
* @author <A href=mailto:risler@sim.informatik.tu-darmstadt.de>Max Risler</A>
* @author <a href=mailto:dueffert@informatik.hu-berlin.de>Uwe Düffert</a>
*/

#include "SystemCall.h"
#include "Platform/File.h"
#include "SoundPlayer.h"
#include "SoundPlayerSimultaneous.h"
#include "Tools/Debugging/Annotation.h"
#include <sys/sysinfo.h>
#include "BHAssert.h"
#include <netdb.h>
#include <arpa/inet.h>
#include <cstring>
#include <ctime>
#include <sys/statvfs.h>
#include <chrono>
#include "Tools/date.h"
#include <iostream>
#include <memory>
#include <array>
#include <sstream>
#include <sys/types.h>
#include <unistd.h>
#include <cstdio> /* defines FILENAME_MAX */


uint64_t SystemCall::base = 0;

int SystemCall::getCurrentProcessId()
{
  return ::getpid();
}

int SystemCall::getParentProcessId()
{
  return ::getppid();
}

std::string SystemCall::getCurrentWorkingDir()
{
  // Create buffer
  char buffer[FILENAME_MAX];

  // The result
  std::string current_working_dir;

  // Get dir
  if (::getcwd(buffer, FILENAME_MAX))
    current_working_dir = buffer;

  return current_working_dir;
}

std::string SystemCall::execute(const std::string& cmd)
{
  static const std::string error = "popen() failed!";

  // Declare buffer
  std::array<char, 512> buffer{0};

  // Create pipe
  std::unique_ptr<FILE, decltype(&::pclose)> pipe(::popen(cmd.c_str(), "r"), ::pclose);

  // Check that pipe is open
  if (!pipe)
    return error;

  // Get output
  std::ostringstream result;
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    result << buffer.data();

  return result.str();
}

bool SystemCall::fileExists(const std::string& file)
{
  if (FILE* f = fopen(file.c_str(), "r"))
  {
    fclose(f);
    return true;
  }
  else
    return false;
}

unsigned SystemCall::getCurrentSystemTime()
{
  return getRealSystemTime();
}

unsigned SystemCall::getRealSystemTime()
{
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

  if (!base)
  {
    std::cout << "-------- Updating time base --------" << std::endl;
    // BH way, needs NTP sync between robots
    //base = time - 10000; // avoid time == 0, because it is often used as a marker
    // NDD way, needs NTP sync before game start!

    auto dp = date::floor<date::days>(now);
    base = std::chrono::duration_cast<std::chrono::milliseconds>(dp.time_since_epoch()).count();
    std::cout << "Time base: " << base << std::endl;
  }
  return static_cast<unsigned>(time - base);
}

uint64_t SystemCall::getSystemTimeBase()
{
  if (!base)
    (void)getRealSystemTime();
  return base;
}

uint64_t SystemCall::getCurrentThreadTime()
{
  struct timespec ts;

  VERIFY(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0);

  uint64_t time = ts.tv_sec * 1000000ll + ts.tv_nsec / 1000;

  static uint64_t base = 0;
  if (!base)
    base = time - 10000 * 1000;
  return time - base;
}

const char* SystemCall::getHostName()
{
  static const char* hostname = 0;
  if (!hostname)
  {
    static char buf[100] = {0};
    VERIFY(!gethostname(buf, sizeof(buf)));
    hostname = buf;
  }
  return hostname;
}

const char* SystemCall::getHostAddr()
{
  static const char* hostaddr = 0;
#ifdef STATIC // Prevent warnings during static linking
  ASSERT(false); // should not be called
#else
  if (!hostaddr)
  {
    static char buf[100];
    hostent* hostAddr = (hostent*)gethostbyname(getHostName());
    if (hostAddr && *hostAddr->h_addr_list)
      strcpy(buf, inet_ntoa(*(in_addr*)*hostAddr->h_addr_list));
    else
      strcpy(buf, "127.0.0.1");
    hostaddr = buf;
  }
#endif
  return hostaddr;
}

SystemCall::Mode SystemCall::getMode()
{
  return physicalRobot;
}

void SystemCall::sleep(unsigned ms)
{
  usleep(1000 * ms);
}

void SystemCall::getLoad(float& mem, float load[3])
{
  struct sysinfo info;
  if (sysinfo(&info) == -1)
    load[0] = load[1] = load[2] = mem = -1.f;
  else
  {
    load[0] = float(info.loads[0]) / 65536.f;
    load[1] = float(info.loads[1]) / 65536.f;
    load[2] = float(info.loads[2]) / 65536.f;
    mem = float(info.totalram - info.freeram) / float(info.totalram);
  }
}

unsigned long long SystemCall::getFreeDiskSpace(const char* path)
{
  std::string fullPath = File::isAbsolute(path) ? path : std::string(File::getBHDir()) + "/" + path;
  struct statvfs data;
  if (!statvfs(fullPath.c_str(), &data))
    return static_cast<unsigned long long>(data.f_bavail) * static_cast<unsigned long long>(data.f_bsize);
  else
    return 0;
}

void* SystemCall::alignedMalloc(size_t size, size_t alignment)
{
  void* ptr;
  if (!posix_memalign(&ptr, alignment, size))
  {
    return ptr;
  }
  else
  {
    return nullptr;
  }
}

void SystemCall::alignedFree(void* ptr)
{
  free(ptr);
}

int SystemCall::playSound(const char* name)
{
  /*
  #ifdef TARGET_ROBOT
    fprintf(stderr, "Playing %s\n", name);
  #endif
  */
  return SoundPlayer::play(name);
}

int SystemCall::text2Speech(std::string text)
{
  std::string t = "t2s:" + text;
  return SoundPlayer::play(t.c_str());
}

int SystemCall::text2SpeechESpeak(const char* text)
{
#define ESPEAK_BINARY "/usr/bin/espeak"

  if (SystemCall::fileExists(ESPEAK_BINARY))
  {
    //text must not contain '
    if (0 != strchr(text, '\''))
    {
      fprintf(stderr, "text2speech: text must not contain single quote: %s", text);
      return 1;
    }
    else
    {
      ANNOTATION("Text2Speech", text);
      char buf[256];
      sprintf(buf, "%s -a 200 -vf5 -p75 -g14 -m '%.200s' &", ESPEAK_BINARY, text);
      return system(buf);
    }
  }
  else
  {
    fprintf(stderr, "text2speech: %s is missing: %s", ESPEAK_BINARY, text);
    return 2;
  }
}
