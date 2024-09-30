/*
 * @file Logger.cpp
 * The file implements a class that implements an online logger that writes representations
 * in the background while the robot is playing soccer.
 *
 * @author Arne Böckmann
 * @author Thomas Röfer
 */

#include <iostream>
#include <iomanip>
#include <ctime>
#include <cstdlib>
#include <sstream>
#include <algorithm>
#include <time.h>
#include <filesystem>
#include <snappy-c.h>
#include <thread>
#include <chrono>
#include "Representations/Infrastructure/GameInfo.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/AnnotationManager.h"
#include "Tools/MessageQueue/LogFileFormat.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/StreamHandler.h"
#include "Logger.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/USBStatus.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/VisualRefereeBehaviorSymbols.h"
#include "Platform/SystemCall.h"
#include "Tools/Build.h"

Logger::Logger(std::initializer_list<char> cycles, const Settings& settings)
{
  for (const char cycle : cycles)
    this->cycles[cycle];

  InMapFile stream("logger.cfg");
  if (stream.exists())
    stream >> parameters;

  if constexpr (!Build::targetRobot())
  {
    parameters.enabled = false;
    parameters.logFilePath = "Logs";
  }

  if (parameters.enabled)
  {
    InMapFile stream2("teamList.cfg");
    if (stream2.exists())
      stream2 >> teamList;

    buffer.resize(parameters.numBuffers);
    for (auto& buf : buffer)
    {
      buf = std::make_unique<MessageQueue>();
      buf->setSize(parameters.bufferSize);
      freeBuffers.push_back(buf.get());
    }
    writerThread.setPriority(parameters.writePriority);

    this->settings = getSettings(settings);
  }
}

Logger::~Logger()
{
  writerThread.stop();
}

void Logger::execute(const char processIdentifier)
{
  if (parameters.enabled)
  {
    Cycle& cycle = cycles.at(processIdentifier);
    if (Blackboard::getInstance().getVersion() != cycle.blackboardVersion)
    {
      const auto convertToLoggables = [&](const auto& representations, auto& loggables)
      {
        const auto thisCycle = [&](const auto& representations)
        {
          return std::tolower(representations.cycle[0]) == processIdentifier;
        };

        const auto cycleRepresentations = std::find_if(representations.begin(), representations.end(), thisCycle);
        ASSERT(cycleRepresentations != representations.end());

        for (const std::string& representation : cycleRepresentations->representations)
        {
          if (Blackboard::getInstance().exists(representation.c_str()))
          {
            int i;
            for (i = 0; i < numOfDataMessageIDs; ++i)
              if (getName(static_cast<MessageID>(i)) == "id" + representation)
              {
                loggables.emplace_back(&Blackboard::getInstance()[representation.c_str()], static_cast<MessageID>(i));
                break;
              }
            if (i == numOfDataMessageIDs)
              OUTPUT_WARNING("Logger: " << representation << " has no message id.");
          }
          else
          {
            OUTPUT_WARNING("Logger: " << representation << " is not available in blackboard.");
          }
        }
      };

      cycle.loggables.clear();
      cycle.activeLoggables.clear();
      convertToLoggables(parameters.representations, cycle.loggables);
      convertToLoggables(parameters.activeRepresentations, cycle.activeLoggables);

      cycle.blackboardVersion = Blackboard::getInstance().getVersion();
    }

    runStateMachine(processIdentifier);
  }
}

void Logger::runStateMachine(char processIdentifier)
{
  Cycle& cycle = cycles.at(processIdentifier);

  if (processIdentifier == 'c')
  {
    try
    {
      const USBStatus& usbStatus = Blackboard::get<USBStatus>();
      const GameInfo& gameInfo = Blackboard::get<GameInfo>();
      const BehaviorData& behaviorData = Blackboard::get<BehaviorData>();
      const RobotInfo& robotInfo = Blackboard::get<RobotInfo>();
      const VisualRefereeBehaviorSymbols& visualRefereeBehaviorSymbols = Blackboard::get<VisualRefereeBehaviorSymbols>();

      receivedGameControllerPacket |= static_cast<const RoboCup::RoboCupGameControlData&>(gameInfo).packetNumber != 0 || gameInfo.secsRemaining != 0;

      const bool record = gameInfo.state == STATE_STANDBY || gameInfo.state == STATE_READY || gameInfo.state == STATE_SET || gameInfo.state == STATE_PLAYING
          || behaviorData.behaviorState >= BehaviorData::BehaviorState::firstCalibrationState || visualRefereeBehaviorSymbols.state != VisualRefereeBehaviorSymbols::State::idle;

      const bool transitionToFramework = robotInfo.transitionToFramework > 0.f;

      const int state_time = SystemCall::getTimeSince(stateBegin);
      const auto go = [&](State state)
      {
        this->state.store(state, std::memory_order_relaxed);
        stateBegin = SystemCall::getCurrentSystemTime();
      };

      // transitions
      switch (state)
      {
      case State::waitForTransitionZero:
        if (!transitionToFramework || !Build::targetRobot())
          go(State::initial);
        break;
      case State::initial:
        if (transitionToFramework
            && (usbStatus.status == USBStatus::MountStatus::readWrite || usbStatus.status == USBStatus::MountStatus::readOnly || usbStatus.status == USBStatus::MountStatus::notMounted))
          go(State::start);
        break;
      case State::start:
        if (parameters.verboseTTS && usbStatus.status != USBStatus::MountStatus::readWrite)
          go(State::complain);
        else
          go(State::waiting);
        break;
      case State::waiting:
        if (parameters.verboseTTS && state_time > 15000 && usbStatus.status != USBStatus::MountStatus::readWrite)
          go(State::complain);
        if (!transitionToFramework)
          go(State::delayStopping);
        if (record)
          go(State::prepareWriting);
        break;
      case State::complain:
        go(State::waiting);
        break;
      case State::prepareWriting:
        go(State::running);
        break;
      case State::running:
        if (!record)
          go(State::delayStopping);
        else if (file && SystemCall::getFreeDiskSpace(logFilename.c_str()) >> 20 < parameters.minFreeSpace)
          go(State::error);
        break;
      case State::delayStopping:
        if (record)
          go(State::running);
        else if (writerIdle.load(std::memory_order_acquire) && SystemCall::getTimeSince(writerIdleStart) > 100 && state_time > 1000)
          go(State::stopping);
        break;
      case State::stopping:
        if (writerThread.tryStop())
          go(State::finished);
        if (state_time > 60000)
          go(State::forceStop);
        break;
      case State::forceStop:
        go(State::finished);
        break;
      case State::finished:
        if (!transitionToFramework)
          go(State::initial);
        else if (record)
          go(State::start);
        break;
      case State::error:
        go(State::aborted);
        break;
      case State::aborted:
        break;
      }


      // actions
      switch (state)
      {
      case State::waitForTransitionZero:
        break;
      case State::initial:
        break;
      case State::start:
      {
        writerThread.start(this, &Logger::writeThread);

        if (usbStatus.status == USBStatus::MountStatus::readWrite)
          SystemCall::text2Speech("U S B logging");
      }
      break;
      case State::waiting:
        break;
      case State::complain:
        SystemCall::text2Speech("Attention! U S B logging unavailable");
        break;
      case State::prepareWriting:
      {
        const std::string filename = generateFilename();
        logFilename = filename + "_Combined.log";
        systemLogFilename = filename + "_system.log";
      }
      break;
      case State::running:
        // logFrame(processIdentifier); below
        break;
      case State::delayStopping:
        break;
      case State::stopping:
        break;
      case State::forceStop:
        writerThread.forceStop();
        break;
      case State::finished:
        break;
      case State::error:
        writerThread.announceStop();
        OUTPUT_WARNING("Logger: Disk full, logging aborted.");
        break;
      case State::aborted:
        break;
      }
    }
    catch (const std::out_of_range& e)
    {
      OUTPUT_ERROR("Logger: " << e.what());
    }
  }

  if (state == State::prepareWriting || state == State::running)
  {
    logFrame(processIdentifier);

    if (!cycle.streamSpecificationDone.load(std::memory_order_relaxed))
    {
      cycle.streamSpecification = getStreamSpecification();
      cycle.streamSpecificationDone.store(true, std::memory_order_release);
    }
  }
}

std::string Logger::getPath() const
{
  const USBStatus& usbStatus = Blackboard::get<USBStatus>();

  if (usbStatus.status == USBStatus::MountStatus::readWrite)
    return usbStatus.logPath;
  else
    return parameters.logFilePath;
}

std::string Logger::generateFilename() const
{
  std::ostringstream oss;
  oss << getPath() << "/";

  const BehaviorData& behaviorData = Blackboard::get<BehaviorData>();

  if (behaviorData.behaviorState >= BehaviorData::BehaviorState::firstCalibrationState)
  {
    oss << "Calibration_";
  }
  else if (receivedGameControllerPacket)
  {
    const GameInfo& gameInfo = Blackboard::get<GameInfo>();

    const auto oppTeam = std::find_if(teamList.teams.begin(),
        teamList.teams.end(),
        [&gameInfo](const TeamList::Team& t)
        {
          return gameInfo.oppTeamNumber == t.number;
        });

    if (oppTeam != teamList.teams.end())
      oss << oppTeam->name << "_";
    else
      oss << "Team-" << static_cast<int>(gameInfo.oppTeamNumber) << "_";

    oss << (gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT ? "ShootOut_" : gameInfo.firstHalf ? "1stHalf_" : "2ndHalf_");
  }
  else
  {
    oss << "Testing_";
  }

  std::time_t t = std::time(nullptr);
  std::tm tm;
  // thread safety needed
#ifdef WINDOWS
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm); // POSIX
#endif

  const RobotInfo& robotInfo = Blackboard::get<RobotInfo>();

  oss << std::put_time(&tm, "%Y%m%d_%H%M%S_") << robotInfo.number;

  std::string logfile = oss.str();
  std::cout << "Logger: Writing to: " << logfile << std::endl;
  return logfile;
}

std::vector<char> Logger::getStreamSpecification()
{
  std::vector<char> ret;
  OutBinarySize size;
  size << Global::getStreamHandler();
  ret.resize(size.getSize());
  OutBinaryMemory stream(ret.data());
  stream << Global::getStreamHandler();
  return ret;
}

std::vector<char> Logger::getSettings(const Settings& settings)
{
  std::vector<char> ret;
  OutBinarySize size;
  settings.write(size);
  ret.resize(size.getSize());
  OutBinaryMemory stream(ret.data());
  settings.write(stream);
  return ret;
}

void Logger::logFrame(char processIdentifier)
{
  Cycle& cycle = cycles.at(processIdentifier);

  MessageQueue* currentQueue = nullptr;
  {
    std::lock_guard<std::mutex> l(bufferMutex);

    if (freeBuffers.empty())
    {
      OUTPUT_WARNING("Logger: Writer thread too slow, discarding frame.");
      return;
    }
    currentQueue = freeBuffers.back();
    freeBuffers.pop_back();
  }

  OutMessage& out = currentQueue->out;

  out.bin << processIdentifier;
  out.finishMessage(idProcessBegin);

  // Stream all representations to the queue
  STOPWATCH("Logger")
  {
    const auto log = [&](const auto& loggables)
    {
      for (const Loggable& loggable : loggables)
      {
        out.bin << *loggable.representation;
        if (!out.finishMessage(loggable.id))
          OUTPUT_WARNING("Logging of " << ::getName(loggable.id) << " failed. The buffer is full.");
      }
    };

    log(cycle.loggables);

    const RobotInfo& robotInfo = Blackboard::get<RobotInfo>();
    const MotionInfo& motionInfo = Blackboard::get<MotionInfo>();

    const bool isActive = robotInfo.penalty == PENALTY_NONE
        && (motionInfo.motion != MotionRequest::Motion::specialAction || motionInfo.specialActionRequest.specialAction != SpecialActionRequest::SpecialActionID::playDead)
        && robotInfo.transitionToFramework > 0.f;

    if (isActive)
      log(cycle.activeLoggables);

    // Append annotations
    Global::getAnnotationManager().getOut().copyAllMessages(*currentQueue);
  }

  // Append timing data if any
  MessageQueue& timingData = Global::getTimingManager().getData();
  if (timingData.getNumberOfMessages() > 0)
    timingData.copyAllMessages(*currentQueue);

  out.bin << processIdentifier;
  out.finishMessage(idProcessFinished);

  {
    std::lock_guard<std::mutex> l(bufferMutex);
    fullBuffers.push_back(currentQueue);
  }

  framesToWrite.post(); // Signal to the writer thread that another block is ready
}

void Logger::writeThread()
{
  Thread<Logger>::setName("Logger");
  BH_TRACE_INIT("Logger");

  std::cout << "Logger started." << std::endl;

  const size_t compressedSize = snappy_max_compressed_length(parameters.bufferSize + 2 * sizeof(unsigned));
  std::vector<char> compressedBuffer;
  if (parameters.compression)
    compressedBuffer.resize(compressedSize + sizeof(unsigned)); // Also reserve 4 bytes for header

  while (writerThread.isRunning()) // Check if we are expecting more data
    if (framesToWrite.wait(100)) // Wait 100 ms for new data then check again if we should quit
    {
      writerIdle.store(false, std::memory_order_relaxed);

      MessageQueue& queue = *fullBuffers.front();
      if (queue.getNumberOfMessages() > 0)
      {
        if (!file && !logFilename.empty())
        {
          file = new OutBinaryFile(logFilename);
          if (file->exists())
          {
            *file << logFileSettings;
            file->write(settings.data(), settings.size());

            *file << logFileMessageIDs;
            queue.writeMessageIDs(*file);

            // wait for all stream specifications
            const auto isStreamSpecDone = [](const auto& entry)
            {
              const auto& [id, cycle] = entry;
              return cycle.streamSpecificationDone.load(std::memory_order_acquire);
            };
            while (!std::all_of(cycles.begin(), cycles.end(), isStreamSpecDone))
              std::this_thread::yield();

            for (const auto& [id, cycle] : cycles)
            {
              *file << logFileStreamSpecification;
              file->write(cycle.streamSpecification.data(), cycle.streamSpecification.size());
            }

            if (parameters.compression)
            {
              *file << logFileCompressed; // Write magic byte that indicates a compressed log file
            }
            else
            {
              *file << logFileUncompressed;
              queue.writeAppendableHeader(*file);
            }
          }
          else
          {
            std::cerr << "Logger: Cannot open log file!" << std::endl;
          }
        }

        if (file && file->exists())
        {
          if (parameters.compression)
          {
            size_t size = compressedSize;
            VERIFY(snappy_compress(queue.getStreamedData(), queue.getStreamedSize(), compressedBuffer.data() + sizeof(unsigned), &size) == SNAPPY_OK);
            reinterpret_cast<unsigned&>(compressedBuffer[0]) = static_cast<unsigned>(size);

            file->write(compressedBuffer.data(), size + sizeof(unsigned));
          }
          else
          {
            queue.append(*file);
          }
        }
        queue.clear();
      }
      {
        std::lock_guard<std::mutex> l(bufferMutex);
        fullBuffers.pop_front();
        freeBuffers.push_back(&queue);
      }
    }
    else if (!writerIdle.load(std::memory_order_relaxed))
    {
      writerIdleStart = SystemCall::getCurrentSystemTime();
      writerIdle.store(true, std::memory_order_release);
    }

  dumpSystemLog();

  // close file after dumpSystemLog() to prevent unmount
  if (file)
  {
    delete file;
    file = nullptr;
  }

  BH_TRACE_TERM;

  std::cout << "Logger stopped." << std::endl;
}

void Logger::dumpSystemLog()
{
  if (!systemLogFilename.empty())
  {
    const std::string output =
        "echo ====== NAODEVILS ======;"
        "journalctl --user-unit naodevils;"
        "echo ====== NAODEVILSBASE ======;"
        "journalctl --user-unit naodevilsbase;"
        "echo ====== LOLA ======;"
        "journalctl --user-unit lola;"
        "echo ====== HAL ======;"
        "journalctl --user-unit hal;"
        "echo ====== DMESG ======;"
        "dmesg;";

    const std::string command = "/bin/bash -c '(" + output + ") > " + systemLogFilename + "'";
    system(command.c_str());
  }
}
