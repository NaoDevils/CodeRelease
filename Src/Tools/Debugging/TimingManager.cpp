/**
 * File:   TimingManager.cpp
 * Author: arne
 *
 * Created on May 9, 2013, 1:37 PM
 */

#include "TimingManager.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Debugging.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include <chrono>
#include <mutex>

using namespace std;

struct TimingManager::Pimpl
{
  /**
   * NOTE: the hashmaps only work because the compiler uses a string table and
   *       allocates only one address for all const string literals with the same value.
   *       If this ever changes you have to replace the key with std::string
   */

  /**
   * Key: name of the timer
   * value: If timer has been started but not stopped, yet: the start time.
   *        Else: the time between start and stop.
   */
  unordered_map<const char*, long long> timing;
  unordered_map<const char*, unsigned short> idTable; /**< Key: name of the stopwatch. Value: the id that is used when sending timing data over the network */
  unsigned currentProcessStartTime = 0; /**< Timestamp of the current process iteration */
  unsigned frameNo = 0; /**<  Number of the current frame*/
  vector<const char*> watchNames; /**< Contains the names of the stopwatches */
  MessageQueue data; /**< Contains the timing data in streamable format inbetween frames */
  bool processRunning = false; /**< Is a process iteration running right now? */
  bool dataPrepared = false; /**< True if data hs already been prepared this frame */
  int watchNameIndex = 0; /**< Every frame a few watch names are transmitted. This is the index of the watchname that is to be transmitted next */
  bool threadTime = true; /**< Use thread time or monotonic time. */

  TimingManager* superTimingManager = nullptr;
  unordered_set<TimingManager*> subTimingManagers;
  std::mutex subTimingManagersLock;
};

TimingManager::TimingManager(TimingManager* superTimingManager) : prvt(new TimingManager::Pimpl)
{
  if (superTimingManager)
  {
    this->prvt->superTimingManager = superTimingManager;
    this->prvt->threadTime = superTimingManager->prvt->threadTime;
    std::lock_guard<std::mutex> l(superTimingManager->prvt->subTimingManagersLock);
    superTimingManager->prvt->subTimingManagers.insert(this);
  }
  else
  {
    // super timing manager needs to transfer data
    prvt->data.setSize(500000);
  }
}

TimingManager::~TimingManager()
{
  if (prvt->superTimingManager)
  {
    std::lock_guard<std::mutex> l(prvt->superTimingManager->prvt->subTimingManagersLock);
    prvt->superTimingManager->prvt->subTimingManagers.erase(this);
  }
  for (TimingManager* subTimingManager : prvt->subTimingManagers)
    subTimingManager->prvt->superTimingManager = nullptr;

  delete prvt;
}

void TimingManager::startTiming(const char* identifier, bool threadTime)
{
  const long long startTime = (prvt->threadTime && threadTime)
      ? static_cast<long long>(SystemCall::getCurrentThreadTime())
      : std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
  prvt->timing[identifier] = startTime;
  prvt->dataPrepared = false;
}

unsigned TimingManager::stopTiming(const char* identifier, bool threadTime)
{
  const long long stopTime = (prvt->threadTime && threadTime)
      ? static_cast<long long>(SystemCall::getCurrentThreadTime())
      : std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
  prvt->timing[identifier] = stopTime - prvt->timing[identifier];
  return static_cast<unsigned>(prvt->timing[identifier]);
}

void TimingManager::setThreadTime(bool threadTime)
{
  prvt->threadTime = threadTime;
  for (TimingManager* timingManager : prvt->subTimingManagers)
    timingManager->prvt->threadTime = threadTime;
}

void TimingManager::signalProcessStart()
{
  prvt->currentProcessStartTime = SystemCall::getCurrentSystemTime();
  prvt->frameNo++;
  prvt->processRunning = true;
  prvt->data.clear();
  prvt->dataPrepared = false;
}

void TimingManager::signalProcessStop()
{
  prvt->processRunning = false;

  for (TimingManager* timingManager : prvt->subTimingManagers)
    timingManager->moveDataTo(*this);
}

MessageQueue& TimingManager::getData()
{
  ASSERT(!prvt->processRunning);
  if (!prvt->dataPrepared)
  {
    prepareData();
    prvt->dataPrepared = true;
  }
  return prvt->data;
}

void TimingManager::prepareData()
{
  for (const auto& timing : prvt->timing)
  {
    if (prvt->idTable.find(timing.first) == prvt->idTable.end())
    {
      prvt->watchNames.push_back(timing.first);
      prvt->idTable[timing.first] = static_cast<unsigned short>(prvt->idTable.size()); //NOTE: this assumes that an unsigned short will always be big big enough to count the timers...
    }
  }


  /** Protocol:
   * unsigned short : number of stopwatch names (usually 3)
   * for each stopwatch name:
   *  unsigned short : id of the stopwatch
   *  string         : name of the stopwatch
   *
   * unsigned short : Total number of stopwatches
   * for each stopwatch:
   *  short : id of the stopwatch  (depending on where you start in the stream you might not know the name of this watch, yet)
   *  unsigned : time in microseconds
   *
   * unsigned : timestamp at which the last iteration started.
   * unsigned : frame number of the current frame
   */
  OutBinaryMessage& out = prvt->data.out.bin;

  // every frame we send 3 watch names
  out << (unsigned short)3; //number of names to follow
  for (int i = 0; i < 3; ++i, prvt->watchNameIndex = (prvt->watchNameIndex + 1) % prvt->watchNames.size())
  {
    const char* watchName = prvt->watchNames[prvt->watchNameIndex];
    out << prvt->idTable[watchName] << watchName;
  }

  // now write the data of all watches
  out << (unsigned short)prvt->timing.size();
  for (const pair<const char* const, long long>& it : prvt->timing)
  {
    out << prvt->idTable[it.first];
    out << (unsigned)it.second; // the cast is ok because the time between start and stop will never be bigger than an int...
    prvt->timing[it.first] = 0;
  }
  out << prvt->currentProcessStartTime;
  out << prvt->frameNo;
  if (!prvt->data.out.finishMessage(idStopwatch))
    OUTPUT_WARNING("TimingManager: queue is full!!!");
}

void TimingManager::moveDataTo(TimingManager& timingManager)
{
  for (auto& timing : prvt->timing)
  {
    if (timing.second > 0)
    {
      timingManager.prvt->timing[timing.first] += timing.second;
      timing.second = 0;
    }
  }
}
