/**
* @file Platform/linux/Main.cpp
* Implementation of the main() function for starting and stopping the module framework.
* @author Colin Graf
*/

#include <csignal>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <string>
#include <sys/file.h> // flock
#include <fcntl.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <mutex>
#include <iostream>
#include <filesystem>
#include <atomic>
#include <signal.h>

#include "SystemCall.h"
#include "Robot.h"
#include "NaoBodyV6.h"
#include "Tools/Settings.h"
#include "naodevilsbase/naodevilsbase.h"

static pid_t pid = 0;
static pthread_t mainTid = 0;
static Robot* robot = nullptr;
static std::atomic<bool> run = true;

static_assert(std::atomic<bool>::is_always_lock_free);

namespace
{
  std::mutex gdbMutex;
  const char* gdbHelper = "/home/nao/Config/gdbCommands";
  const char* crashInfo = "/home/nao/logs/crashInfo";
  const char* extension = ".log";
  const char* execPath = nullptr;

  const char* signalToString(int signal)
  {
    static const char* segv = "SIGSEGV";
    static const char* ill = "SIGILL";
    static const char* fpe = "SIGFPE";
    static const char* abrt = "SIGABRT";
    static const char* unknown = "UNKNOWN";

    // Get the signal
    switch (signal)
    {
    case SIGSEGV:
      return segv;

    case SIGILL:
      return ill;

    case SIGFPE:
      return fpe;

    case SIGABRT:
      return abrt;

    default:
      return unknown;
    }
  }

  void handleTrapSignal(int signal)
  {
    // Get helpers
    int pid = SystemCall::getCurrentProcessId();

    // Generate GDB command
    std::ostringstream command;
    command << "/usr/bin/gdb -x " << gdbHelper << ' ' << execPath << ' ' << pid;

    {
      // SYNC GDB
      std::lock_guard<std::mutex> guard(gdbMutex);

      // Get current time
      auto now_c = std::chrono::system_clock::now();
      std::time_t time = std::chrono::system_clock::to_time_t(now_c);
      std::tm now{};

      localtime_r(&time, &now);

      // Format time
      std::array<char, 100> filename{};
      std::strftime(filename.data(), filename.size(), "_%F_%H-%M-%S", &now);

      // use USB directory if present
      // required to comply with GORE2021 rules for USB logging
      if (std::filesystem::is_directory("/home/nao/usb/logs"))
        crashInfo = "/home/nao/usb/logs/crashInfo";

      // Create output file
      std::ofstream file(std::string(crashInfo) + filename.data() + extension);

      // Execute command and save output
      std::string result = SystemCall::execute(command.str());

      // Fill file
      file << "Created because of signal (" << signal << ") [" << signalToString(signal) << ']' << std::endl;

      // Fill rest
      file << "Result of command \"" << command.str() << '"' << std::endl << std::endl << result << std::endl;

      // Flush and close
      file.flush();
      file.close();
    }

    std::exit(-signal);
  }
} // namespace

static void bhumanStart()
{
  fprintf(stderr, "NaoDevils: Start.\n");

  robot = new Robot();
  robot->start();
}

static void bhumanStop()
{
  fprintf(stderr, "NaoDevils: Stop.\n");
  robot->announceStop();
  robot->stop();
  delete robot;
  robot = 0;
  //fprintf(stderr, "BHuman: Stopped.\n");
}

static void sighandlerShutdown(int sig)
{
  if (run.exchange(false, std::memory_order_relaxed))
  {
    const char str[] = "Caught signal\nShutting down...\n";
    // ok, write is signal-safe
    write(STDERR_FILENO, str, sizeof(str) - 1);
  }

  if (mainTid != pthread_self())
    pthread_kill(mainTid, sig);
}

static void sighandlerRedirect(int sig)
{
  if (pid != 0)
    kill(pid, sig);
  run.store(false, std::memory_order_relaxed);
}

int main(int argc, char* argv[])
{
  {
    // parse command-line arguments
    bool background = false;
    bool recover = false;
    bool watchdog = false;
    const char* bhDir = "/home/nao";

    for (int i = 1; i < argc; ++i)
      if (!strcmp(argv[i], "-b"))
        background = true;
      else if (!strcmp(argv[i], "-w"))
        watchdog = true;
      else if (!strcmp(argv[i], "-c") && i + 1 < argc)
        bhDir = argv[++i];
      else
      {
        fprintf(stderr,
            "Usage: %s [-b] [-c <dir>] [-w]\n\
    -b            run in background (as daemon)\n\
    -c <dir>      used gt directory (default is /home/nao)\n\
    -w            use a watchdog for crash recovery and creating trace dumps\n",
            argv[0]);
        exit(EXIT_FAILURE);
      }

    // avoid duplicated instances
    int fd = open("/tmp/bhuman", O_CREAT, 0600);
    if (fd == -1 || flock(fd, LOCK_EX | LOCK_NB) == -1)
    {
      fprintf(stderr, "There is already an instance of this process!\n");
      exit(EXIT_FAILURE);
    }

    // start as daemon
    if (background)
    {
      fprintf(stderr, "Starting as daemon...\n");
      pid_t childPid = fork();
      if (childPid == -1)
        exit(EXIT_FAILURE);
      if (childPid != 0)
        exit(EXIT_SUCCESS);
    }

    // change working directory
    if (*bhDir && chdir(bhDir) != 0)
    {
      fprintf(stderr, "chdir to config directory failed!\n");
      exit(EXIT_FAILURE);
    }

    // the watchdog
    if (watchdog)
    {
      for (;;)
      {
        // create pipe for logging
        int stdoutPipe[2];
        int stderrPipe[2];
        bool pipeReady = true;

        if (pipe(stdoutPipe) == -1 || pipe(stderrPipe) == -1)
        {
          fprintf(stderr, "NaoDevils: Error while creating pipes for logging. All logs will be printed on console only! \n");
          pipeReady = false;
        }

        pid = fork();
        if (pid == -1)
          exit(EXIT_FAILURE);
        if (pid != 0)
        {
          int status;
          signal(SIGTERM, sighandlerRedirect);
          signal(SIGINT, sighandlerRedirect);
          if (waitpid(pid, &status, 0) != pid)
          {
            exit(EXIT_FAILURE);
          }
          signal(SIGTERM, SIG_DFL);
          signal(SIGINT, SIG_DFL);

          if (pipeReady)
          {
            // close unused write end
            close(stdoutPipe[1]);
            close(stderrPipe[1]);

            dup2(STDOUT_FILENO, stdoutPipe[0]); // redirect out-pipe to stdout
            dup2(STDERR_FILENO, stderrPipe[0]); // redirect err-pipe to stderr
          }

          // detect requested or normal exit
          bool normalExit = !run.load(std::memory_order_relaxed) || (WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);

          NaoBodyV6 naoBody;
          if (normalExit)
          {
            if (naoBody.init())
            {
              // normal shutdown
              naoBody.setCrashed(NDData::State::sigTERMState);
              naoBody.cleanup();
            }
          }
          else
          {
            // dump trace and assert trace
            if (naoBody.init())
            {
              naoBody.setCrashed(WIFSIGNALED(status) ? int(WTERMSIG(status)) : int(NDData::State::abnormalTerminationState));
              naoBody.cleanup();
            }
            Assert::logDump(true, WIFSIGNALED(status) ? int(WTERMSIG(status)) : 0);
            Assert::logDump(false, WIFSIGNALED(status) ? int(WTERMSIG(status)) : 0);
          }

          // quit here?
          if (normalExit)
            exit(WIFEXITED(status) ? WEXITSTATUS(status) : EXIT_FAILURE);

          // don't restart if the child process got killed
          if (WIFSIGNALED(status) && WTERMSIG(status) == SIGKILL)
            exit(EXIT_FAILURE);

            // restart in release mode only
#ifndef NDEBUG
          exit(EXIT_FAILURE);
#else
          // deactivate the pre-initial state
          recover = true;

          usleep(2000 * 1000);
#endif
        }
        else
        {
          if (pipeReady)
          {
            // close unused read end
            close(stdoutPipe[0]);
            close(stderrPipe[0]);

            dup2(STDOUT_FILENO, stdoutPipe[1]); // redirect stdout to out-pipe
            dup2(STDERR_FILENO, stderrPipe[1]); // redirect stderr to err-pipe
          }
          break;
        }
      }
    }

    // lock memory and avoid page faults
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
      perror("mlockall() failed");

    mainTid = pthread_self();

    std::cout << "Loading settings" << std::endl;
    // load first settings instance
    Settings settings;
    settings.recover = recover;

    if (!settings.loadingSucceeded())
      return EXIT_FAILURE;


    // register signal handler for strg+c and termination signal
    signal(SIGTERM, sighandlerShutdown);
    signal(SIGINT, sighandlerShutdown);

    // Set path to executable
    execPath = argv[0];

    // register signal handler for trap signals
    std::signal(SIGSEGV, handleTrapSignal);
    std::signal(SIGILL, handleTrapSignal);
    std::signal(SIGFPE, handleTrapSignal);
    std::signal(SIGABRT, handleTrapSignal);

    //
    bhumanStart();
  }

  while (run.load(std::memory_order_relaxed))
    pause();

  bhumanStop();

  return EXIT_SUCCESS;
}
