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
#include <unistd.h>
#include <array>
#include <chrono>
#include <mutex>

#include "SystemCall.h"
#include "Robot.h"
#include "NaoBody.h"
#include "NaoBodyV6.h"
#include "Tools/Settings.h"
//#include "libbhuman/bhuman.h"
#include "ndevilsbase/ndevils.h"

static pid_t bhumanPid = 0;
static Robot* robot = nullptr;
static bool run = true;

namespace
{
  std::mutex gdbMutex;
  const char* gdbHelper = "/home/nao/Config/gdbCommands";
  const char* crashInfo = "/home/nao/logs/crashInfo";
  const char* extension = ".log";
  const char* execPath = nullptr;
  const std::uint16_t crashInfoHistory = 8;

  void handleFileRotation(const std::string& file, std::uint16_t max)
  {
    for (int i = max - 1; i >= 1; i--)
    {
      // Get current file name
      std::string current = file + std::to_string(i) + extension;

      // Check if file exists
      if (SystemCall::fileExists(current))
        // Increase number at the end
        rename(current.c_str(), (file + std::to_string(i + 1) + extension).c_str());
    }

    if (SystemCall::fileExists(file + extension))
      rename((file + extension).c_str(), (file + std::to_string(1) + extension).c_str());
  }

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
    command << "gdb -x " << gdbHelper << ' ' << execPath << ' ' << pid;

    {
      // SYNC GDB
      std::lock_guard<std::mutex> guard(gdbMutex);

      // Execute command and save output
      std::string result = SystemCall::execute(command.str());

      // Handle crash info rotation
      handleFileRotation(crashInfo, crashInfoHistory);

      // Get current time
      auto now_c = std::chrono::system_clock::now();
      std::time_t time = std::chrono::system_clock::to_time_t(now_c);
      std::tm now{};

      localtime_r(&time, &now);

      // Get milliseconds for printing
      auto ms_c = std::chrono::duration_cast<std::chrono::milliseconds>(now_c.time_since_epoch());
      auto ms = ms_c.count() % std::chrono::milliseconds::period::den;

      // Format time
      std::array<char, 100> buffer{ };

      std::strftime(buffer.data(), buffer.size(), "%F %T", &now);

      // Create output file
      std::ofstream file(std::string(crashInfo) + extension);

      // Fill file
      file << "Created because of signal (" << signal << ") [" << signalToString(signal) << ']' << std::endl
        << "It is " << buffer.data() << '.';

      // Leading zeros for milliseconds
      file << std::setfill('0') << std::setw(3) << ms << std::endl;

      // Fill rest
      file << "Result of command \"" << command.str() << '"' << std::endl << std::endl << result << std::endl;

      // Flush and close
      file.flush();
      file.close();
    }

    std::exit(-signal);
  }
}

static void bhumanStart()
{
  fprintf(stderr, "BHuman: Start.\n");

  robot = new Robot();
  robot->start();
}

static void bhumanStop()
{
  fprintf(stderr, "BHuman: Stop.\n");
  robot->announceStop();
  robot->stop();
  delete robot;
  robot = 0;
  //fprintf(stderr, "BHuman: Stopped.\n");
}

static void sighandlerShutdown(int sig)
{
  if (run)
    printf("Caught signal %i\nShutting down...\n", sig);
  run = false;
}

static void sighandlerRedirect(int sig)
{
  //if(bhumanPid != 0)
  //kill(bhumanPid, sig);
  run = false;
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
        fprintf(stderr, "Usage: %s [-b] [-c <dir>] [-w]\n\
    -b            run in background (as daemon)\n\
    -c <dir>      used gt directory (default is /home/nao)\n\
    -w            use a watchdog for crash recovery and creating trace dumps\n", argv[0]);
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
          fprintf(stderr, "B-Human: Error while creating pipes for logging. All logs will be printed on console only! \n");
          pipeReady = false;
        }

        bhumanPid = fork();
        if (bhumanPid == -1)
          exit(EXIT_FAILURE);
        if (bhumanPid != 0)
        {
          int status;
          signal(SIGTERM, sighandlerRedirect);
          signal(SIGINT, sighandlerRedirect);
          if (waitpid(bhumanPid, &status, 0) != bhumanPid)
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
          bool normalExit = !run || (WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);

          NaoBodyV6 naoBody;
          if (normalExit)
          {
            if (naoBody.init())
            {
              // normal shutdown
              naoBody.setCrashed(NDState::sigTERMState);
              naoBody.cleanup();
            }
          }
          else
          {
            // dump trace and assert trace
            if (naoBody.init())
            {
              naoBody.setCrashed(WIFSIGNALED(status) ? int(WTERMSIG(status)) : int(abnormalTerminationState));
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
#endif

          // deactivate the pre-initial state
          recover = true;

          usleep(2000 * 1000);
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

  while (run)
    pause();

  bhumanStop();

  return EXIT_SUCCESS;
}
