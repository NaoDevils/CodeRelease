/**
* @file MessageQueueBase.cpp
* Implementation of the class that performs the memory management for the class MessageQueue.
* @author Martin Lötzsch
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <limits>

#include "MessageQueueBase.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/InOut.h"

MessageQueueBase::MessageQueueBase()
#ifndef TARGET_ROBOT
    : maximumSize(0x4000000), // 64 MB
      reservedSize(16384)
{
  buf = (char*)malloc(reservedSize + queueHeaderSize) + queueHeaderSize;
  ASSERT(buf);
#else
    {
#endif
}

MessageQueueBase::~MessageQueueBase()
{
  freeIndex();
  if (buf && ownedBuf)
    free(buf - queueHeaderSize);
  if (mappedIDs)
  {
    delete[] mappedIDs;
    delete[] mappedIDNames;
  }
}

void MessageQueueBase::setSize(size_t size, size_t reserveForInfrastructure)
{
  this->reserveForInfrastructure = reserveForInfrastructure;
  size = std::min(std::numeric_limits<size_t>::max() - queueHeaderSize, size);
#ifdef TARGET_ROBOT
  ASSERT(!buf);
  buf = (char*)malloc(size + queueHeaderSize) + queueHeaderSize;
  ASSERT(buf);
#else
  ASSERT(size >= usedSize);
  if (size < reservedSize)
  {
    ASSERT(ownedBuf);
    char* newBuf = (char*)realloc(buf - queueHeaderSize, size + queueHeaderSize) + queueHeaderSize;
    if (newBuf)
    {
      buf = newBuf;
      reservedSize = size;
    }
  }
#endif
  maximumSize = size;
}

void MessageQueueBase::clear()
{
  usedSize = 0;
  numberOfMessages = 0;
  writePosition = 0;
  writingOfLastMessageFailed = false;
  selectedMessageForReadingPosition = 0;
  readPosition = 0;
  lastMessage = 0;
  freeIndex();

  if (!ownedBuf)
  {
    ownedBuf = true;
    buf = reinterpret_cast<char*>(malloc(reservedSize + queueHeaderSize)) + queueHeaderSize;
    ASSERT(buf);
  }
}

void MessageQueueBase::createIndex()
{
  freeIndex();
  messageIndex = new size_t[numberOfMessages];
  selectedMessageForReadingPosition = 0;
  for (int i = 0; i < numberOfMessages; ++i)
  {
    messageIndex[i] = selectedMessageForReadingPosition;
    selectedMessageForReadingPosition += getMessageSize() + headerSize;
  }
}

void MessageQueueBase::freeIndex()
{
  if (messageIndex)
  {
    delete[] messageIndex;
    messageIndex = 0;
  }
}

void MessageQueueBase::removeMessage(int message)
{
  freeIndex();
  selectedMessageForReadingPosition = 0;
  int i;
  for (i = 0; i < message; ++i)
    selectedMessageForReadingPosition += getMessageSize() + headerSize;
  usedSize = selectedMessageForReadingPosition;
  for (++i; i < numberOfMessages; ++i)
  {
    int mlength = getMessageSize() + headerSize;
    selectedMessageForReadingPosition += mlength;
    memcpy(buf + usedSize, buf + selectedMessageForReadingPosition, mlength);
    usedSize = selectedMessageForReadingPosition;
  }
  readPosition = 0;
  --numberOfMessages;
  selectedMessageForReadingPosition = 0;
  lastMessage = 0;
}

char* MessageQueueBase::reserve(size_t size)
{
  size_t currentSize = usedSize + headerSize + writePosition;
  if (currentSize + size > maximumSize)
    return 0;
  else
  {
#ifndef TARGET_ROBOT
    size_t r = reservedSize;
    if (currentSize + size >= r)
    {
      r *= 2;
      if (currentSize + size >= r)
        r = (currentSize + size) * 4;
    }
    if (r > maximumSize)
      r = maximumSize;
    if (r > reservedSize)
    {
      ASSERT(ownedBuf);
      char* newBuf = (char*)realloc(buf - queueHeaderSize, static_cast<size_t>(r) + queueHeaderSize) + queueHeaderSize;
      if (newBuf)
      {
        buf = newBuf;
        reservedSize = r;
      }
      else
      {
        maximumSize = reservedSize;
        return 0;
      }
    }
#endif
    writePosition += static_cast<unsigned>(size);
    return buf + currentSize;
  }
}

void MessageQueueBase::setBuffer(char* buffer)
{
  if (buf && ownedBuf)
    free(buf - queueHeaderSize);

  buf = buffer;
  ownedBuf = false;
}

void MessageQueueBase::write(const void* p, size_t size)
{
  ASSERT(!messageIndex);
  if (!writingOfLastMessageFailed)
  {
    char* dest = reserve(size);
    if (dest)
      memcpy(dest, p, size);
    else
      writingOfLastMessageFailed = true;
  }
}

bool MessageQueueBase::finishMessage(MessageID id)
{
  ASSERT(buf);
  ASSERT(!messageIndex);
  bool success = !writingOfLastMessageFailed;
  if (success)
  {
    if (reserveForInfrastructure > maximumSize - usedSize - writePosition - headerSize)
      switch (id)
      { // When these messages are lost, communication might get stuck
      case idProcessBegin:
      case idProcessFinished:
      case idDebugRequest:
      case idDebugResponse:
      case idDebugDataResponse:
      case idDebugDataChangeRequest:
      case idStreamSpecification:
      case idModuleTable:
      case idModuleRequest:
      case idQueueFillRequest:
      case idLogResponse:
      case idDrawingManager:
      case idDrawingManager3D:
      case idConsole:
      case idRobotname:
      case idAudioData: // continuous data stream required
      case idExecutorObservings: // continuous data stream required
      case idPingpong:
        break; // accept
      default:
        success = false; // reject
      }

    if (success)
    {
      ASSERT(writePosition > 0);
      memcpy(buf + usedSize, (char*)&id, 1); // write the id of the message
      memcpy(buf + usedSize + 1, &writePosition, 3); // write the size of the message
      ++numberOfMessages;
      usedSize += writePosition + headerSize;
    }
  }

  writePosition = 0;
  writingOfLastMessageFailed = false;

  return success;
}

std::tuple<int, int> MessageQueueBase::countFramesAndMessages()
{
  int numberOfFrames = 0, numberOfMessagesWithinCompleteFrames = 0;
  numberOfMessages = 0;
  for (selectedMessageForReadingPosition = 0; selectedMessageForReadingPosition < usedSize; selectedMessageForReadingPosition += getMessageSize() + headerSize)
  {
    ++numberOfMessages;
    if (getMessageID() == idProcessFinished)
    {
      ++numberOfFrames;
      numberOfMessagesWithinCompleteFrames = numberOfMessages;
    }
  }

  return {numberOfFrames, numberOfMessagesWithinCompleteFrames};
}

void MessageQueueBase::removeRepetitions()
{
  ASSERT(!messageIndex);
  unsigned short messagesPerType[5][numOfMessageIDs];
  unsigned char numberOfProcesses = 0, processes[26], currentProcess = 0;

  memset(messagesPerType, 0, sizeof(messagesPerType));
  memset(processes, 255, sizeof(processes));
  selectedMessageForReadingPosition = 0;

  for (int i = 0; i < numberOfMessages; ++i)
  {
    if (getMessageID() == idProcessBegin)
    {
      unsigned char process = getData()[0] - 'a';
      if (processes[process] == 255)
        processes[process] = numberOfProcesses++;
      currentProcess = processes[process];
    }
    ++messagesPerType[currentProcess][getMessageID()];
    selectedMessageForReadingPosition += getMessageSize() + headerSize;
  }

  selectedMessageForReadingPosition = 0;
  usedSize = 0;
  int numOfDeleted = 0;
  size_t frameBegin = std::numeric_limits<size_t>::max();
  bool frameEmpty = true;

  for (int i = 0; i < numberOfMessages; ++i)
  {
    int mlength = getMessageSize() + headerSize;
    bool copy;
    switch (getMessageID())
    {
    // accept up to 20 times, process id is not important
    case idText:
      copy = --messagesPerType[currentProcess][idText] <= 20;
      break;

    // accept always, process id is not important
    case idDebugRequest:
    case idDebugResponse:
    case idDebugDataResponse:
    case idPlot:
    case idConsole:
    case idAudioData:
    case idAnnotation:
    case idLogResponse:
    case idExecutorObservings:
    case idPingpong:
      copy = true;
      break;

    // data only from latest frame
    case idStopwatch:
    case idDebugImage:
    case idDebugJPEGImage:
    case idDebugDrawing:
    case idDebugDrawing3D:
      copy = messagesPerType[currentProcess][idProcessFinished] == 1;
      break;

    // always accept, but may be reverted later
    case idProcessBegin:
      if (frameBegin != std::numeric_limits<size_t>::max()) // nothing between last idProcessBegin and this one, so remove idProcessBegin as well
      {
        usedSize = frameBegin;
        ++numOfDeleted;
      }
      currentProcess = processes[getData()[0] - 'a'];
      copy = true;
      break;

    case idProcessFinished:
      ASSERT(currentProcess == processes[getData()[0] - 'a']);
      copy = !frameEmpty; // nothing since last idProcessBegin or idProcessFinished, no new idProcessFinished required
      --messagesPerType[currentProcess][idProcessFinished];
      break;

    default:
      if (getMessageID() < numOfDataMessageIDs) // data only from latest frame
        copy = messagesPerType[currentProcess][idProcessFinished] == 1;
      else // only the latest other messages
        copy = --messagesPerType[currentProcess][getMessageID()] == 0;
    }

    if (copy)
    {
      // Remember position of begin of frame, but forget it, when another message was copied.
      // So idProcessBegin idProcessFinished+ will be removed.
      if (getMessageID() == idProcessBegin) // remember begin of frame
      {
        frameBegin = usedSize;
        frameEmpty = true; // assume next frame as empty
      }
      else if (getMessageID() == idProcessFinished)
        frameEmpty = true; // assume next frame as empty
      else // we copy a message within a frame so the idProcessBegin/Finished must stay
      {
        frameBegin = std::numeric_limits<size_t>::max();
        ;
        frameEmpty = false;
      }

      //this message is important, it shall be copied
      if (usedSize != selectedMessageForReadingPosition)
        memmove(buf + usedSize, buf + selectedMessageForReadingPosition, mlength);
      usedSize += mlength;
    }
    else
      ++numOfDeleted;
    selectedMessageForReadingPosition += mlength;
  }
  numberOfMessages -= numOfDeleted;
  readPosition = 0;
  selectedMessageForReadingPosition = 0;
  lastMessage = 0;
}

MessageID MessageQueueBase::getMessageID() const
{
  MessageID id = MessageID(buf[selectedMessageForReadingPosition]);
  return id < numOfMappedIDs ? mappedIDs[id] : id;
}

void MessageQueueBase::setSelectedMessageForReading(int message)
{
  ASSERT(message >= 0);
  ASSERT(message < numberOfMessages);

  if (messageIndex)
    selectedMessageForReadingPosition = messageIndex[message];
  else
  {
    int m = message;
    if (m >= lastMessage)
    {
      ASSERT(lastMessage < numberOfMessages);
      m -= lastMessage;
    }
    else
      selectedMessageForReadingPosition = 0;

    for (int i = 0; i < m; ++i)
      selectedMessageForReadingPosition += getMessageSize() + headerSize;
  }

  readPosition = 0;
  lastMessage = message;
}

void MessageQueueBase::read(void* p, size_t size)
{
  ASSERT(readPosition + static_cast<int>(size) <= getMessageSize());
  memcpy(p, buf + selectedMessageForReadingPosition + headerSize + readPosition, size);
  readPosition += static_cast<int>(size);
}

void MessageQueueBase::writeMessageIDs(Out& stream, MessageID numOfMessageIDs) const
{
  if (mappedIDs)
  {
    stream << numOfMappedIDs;
    for (int i = 0; i < numOfMappedIDs; ++i)
      stream << mappedIDNames[i];
  }
  else
  {
    stream << static_cast<unsigned char>(numOfMessageIDs);
    for (int i = 0; i < numOfMessageIDs; ++i)
      stream << ::getName(static_cast<MessageID>(i));
  }
}

void MessageQueueBase::readMessageIDMapping(In& stream)
{
  ASSERT(!mappedIDs);
  stream >> numOfMappedIDs;
  mappedIDs = new MessageID[numOfMappedIDs];
  mappedIDNames = new std::string[numOfMappedIDs];
  memset(mappedIDs, undefined, numOfMappedIDs);

  for (int i = 0; i < numOfMappedIDs; ++i)
  {
    stream >> mappedIDNames[i];
    for (int j = 0; j < numOfMessageIDs; ++j)
      if (mappedIDNames[i] == ::getName(static_cast<MessageID>(j)))
      {
        mappedIDs[i] = static_cast<MessageID>(j);
        break;
      }
  }
}
