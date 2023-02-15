/*
 * @file Logger.h
 * The file declares a class that implements an online logger that writes representations
 * in the background while the robot is playing soccer.
 *
 * Logfile format:
 * logFileMessageIDs | number of message ids | streamed message id names |
 * logFileStreamSpecification | streamed StreamHandler |
 * idLogFileCompressed | size of next compressed block | compressed block | size | compressed block | etc...
 * Each block is compressed using libsnappy
 *
 * Block format (after decompression):
 * | block length | number of messages | Frame | Frame | Frame | ... | Frame |
 *
 * Each frame looks like this:
 * | ProcessBegin | Log data 1 | Log data 2 | ... | Log data n | ProcessFinished |
 * Log data format:
 * | ID ( 1 byte) | Message size (3 byte) | Message |
 *
 * @author Arne Böckmann
 * @author Thomas Röfer
 */

#pragma once

#include "Blackboard.h"
#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/USBStatus.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include <mutex>
#include <memory>
#include <deque>
#include <atomic>

class Logger
{
private:
  STREAMABLE(Parameters,
    STREAMABLE(Cycle,,
      (std::string)("") cycle,
      (std::vector<std::string>) representations
    );
    ,
    (bool)(false) enabled, /**< Determines whether the logger is enabled or disabled. */
    (std::string) logFilePath, /**< Where to write the log file. */
    (int) numBuffers, /**< Max size of the buffer in bytes. */
    (int) bufferSize, /**< Size per frame in bytes. */
    (std::vector<Cycle>) representations, /**< Contains the representations that should be logged. */
    (std::vector<Cycle>) activeRepresentations, /**< Contains the representations that should be logged only when active. */
    (int) writePriority,
    (unsigned) minFreeSpace, /**< Minimum free space left on the device in MB. */
    (bool) compression, /**< Enable snappy compression. */
    (bool) verboseTTS /**< Enable verbose text-to-speech output. */
  );

  STREAMABLE(TeamList,
    STREAMABLE(Team,,
      (uint8_t) number,
      (std::string) name
    ),

    (std::vector<Team>) teams
  );


  class Loggable
  {
  public:
    Streamable* representation;
    MessageID id;

    Loggable() = default;
    Loggable(Streamable* representation, MessageID id) : representation(representation), id(id) {}
  };

  struct Cycle
  {
    int blackboardVersion = 0;
    std::vector<Loggable> loggables;
    std::vector<Loggable> activeLoggables;
    std::vector<char> streamSpecification;
    std::atomic_bool streamSpecificationDone = false;
  };

  Parameters parameters;
  TeamList teamList; /**< The list of all teams for naming the log file after the opponent. */

  std::unordered_map<char, Cycle> cycles;

  std::vector<std::unique_ptr<MessageQueue>> buffer; /**< Ring buffer of message queues. Shared with the writer thread. */
  std::deque<MessageQueue*> freeBuffers;
  std::deque<MessageQueue*> fullBuffers;
  std::mutex bufferMutex;

  std::string logFilename; /**< Path and name of the log file. Set in initial state. */
  std::string systemLogFilename; /**< Path and name of the system log file. Set in initial state. */
  bool receivedGameControllerPacket = false; /**< Ever received a packet from the GameController? */
  Thread<Logger> writerThread; /**< Used to write the buffer to disk in the background */
  Semaphore framesToWrite; /**< How many frames the writer thread should write? */
  std::atomic_bool writerIdle = true; /**< Is true if the writer thread has nothing to do. */
  unsigned writerIdleStart = 0; /**< The system time at which the writer thread went idle. */
  OutBinaryFile* file = nullptr; /**< The stream that writes the log file. */

  enum class State
  {
    waitForTransitionZero,
    initial,
    start,
    waiting,
    complain,
    prepareWriting,
    running,
    delayStopping,
    stopping,
    forceStop,
    finished,
    error,
    aborted
  };

  std::atomic<State> state = State::waitForTransitionZero;
  unsigned stateBegin = 0;

  /**
   * Generate a filename containing the robot's player number, its name, and the current
   * date and time.
   * @return A log file name that starts with the path defined in the parameters.
   */
  std::string generateFilename() const;
  std::string getPath() const;

  void runStateMachine(char processIdentifier);

  /** Get streamed data type specification to be used in the logger thread. */
  std::vector<char> getStreamSpecification();

  /** Write all loggable representations to a buffer. */
  void logFrame(char processIdentifier);

  /** Write contents of buffers to disk in the background. */
  void writeThread();

  void dumpSystemLog();

public:
  Logger(std::initializer_list<char> cycles);
  ~Logger();

  /** Has to be called in each cycle. */
  void execute(const char processIdentifier);
};
