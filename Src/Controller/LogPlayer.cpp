/**
* @file Controller/LogPlayer.cpp
*
* Implementation of class LogPlayer
*
* @author Martin Lötzsch
*/

#include <QImage>
#include "LogPlayer.h"
#include "Representations/AnnotationInfo.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/LowFrameRateImage.h"
#include "Representations/Infrastructure/SequenceImage.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Platform/SystemCall.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Tools/MessageQueue/LogFileFormat.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Debugging/DebugDataStreamer.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <snappy-c.h>
#include <vector>
#include <list>
#include <map>

using namespace std;
using json = nlohmann::json;

LogPlayer::LogPlayer(MessageQueue& targetQueue) : targetQueue(targetQueue), streamHandler(nullptr)
{
  init();
}

void LogPlayer::init()
{
  clear();
  stop();
  numberOfFrames = 0;
  numberOfMessagesWithinCompleteFrames = 0;
  replayOffset = 0;
  state = initial;
  loop = true; //default: loop enabled
  if (streamHandler)
  {
    delete streamHandler;
    streamHandler = nullptr;
  }
  streamSpecificationReplayed = false;
}

bool LogPlayer::open(const char* fileName)
{
  InBinaryFile file(fileName);
  if (file.exists())
  {
    init();

    char magicByte;
    file >> magicByte;

    if (magicByte == logFileMessageIDs)
    {
      readMessageIDMapping(file);
      file >> magicByte;
    }

    while (magicByte == logFileStreamSpecification)
    {
      if (!streamHandler)
        streamHandler = new StreamHandler;

      file >> *streamHandler;
      file >> magicByte;
    }

    switch (magicByte)
    {
    case logFileUncompressed: //regular log file
      //file >> *this; // copy data to memory
      mapFile(file); // map data to memory
      break;
    case logFileCompressed: //compressed log file
      while (!file.eof())
      {
        unsigned compressedSize;
        file >> compressedSize;
        ASSERT(compressedSize > 0);
        std::vector<char> compressedBuffer;
        compressedBuffer.resize(compressedSize);
        file.read(&compressedBuffer[0], (int)compressedSize);

        size_t uncompressedSize = 0;
        snappy_uncompressed_length(&compressedBuffer[0], compressedSize, &uncompressedSize);
        std::vector<char> uncompressBuffer;
        uncompressBuffer.resize(uncompressedSize);
        if (snappy_uncompress(&compressedBuffer[0], compressedSize, &uncompressBuffer[0], &uncompressedSize) != SNAPPY_OK)
          break;
        InBinaryMemory mem(&uncompressBuffer[0], uncompressedSize);
        mem >> *this;
      }
      break;
    default:
      ASSERT(false); //unknown magic byte
    }

    stop();
    std::tie(numberOfFrames, numberOfMessagesWithinCompleteFrames) = queue.countFramesAndMessages();
    createFrameIndex();
    return true;
  }
  return false;
}

void LogPlayer::play()
{
  state = playing;
}

void LogPlayer::stop()
{
  if (state == recording)
  {
    recordStop();
    return;
  }
  currentMessageNumber = -1;
  currentFrameNumber = -1;
  state = initial;
  lastImageFrameNumber = -1;
}

void LogPlayer::pause()
{
  if (getNumberOfMessages() == 0)
    state = initial;
  else
    state = paused;
}

void LogPlayer::stepBackward()
{
  pause();
  if (state == paused)
  {
    if (currentFrameNumber > 0)
      --currentFrameNumber;
    else if (loop && numberOfFrames > 0)
      currentFrameNumber = numberOfFrames - 1;
    else
      return;
    currentMessageNumber = frameIndex[currentFrameNumber];
    queue.setSelectedMessageForReading(currentMessageNumber);
    stepRepeat();
  }
}

void LogPlayer::stepImageBackward()
{
  pause();
  if (state == paused && (currentFrameNumber > 0 || (loop && numberOfFrames > 0)))
  {
    int lastImageFrameNumber = this->lastImageFrameNumber;
    do
      stepBackward();
    while (lastImageFrameNumber == this->lastImageFrameNumber && (currentFrameNumber > 0 || (loop && numberOfFrames > 0)));
  }
}

void LogPlayer::stepForward()
{
  pause();
  if (state == paused)
  {
    if (currentFrameNumber >= numberOfFrames - 1 || currentMessageNumber >= numberOfMessagesWithinCompleteFrames - 1)
    {
      if (loop && numberOfFrames > 0)
      {
        currentMessageNumber = -1;
        currentFrameNumber = -1;
      }
      else
        return;
    }
    replayStreamSpecification();
    do
    {
      copyMessage(++currentMessageNumber, targetQueue);
      if (queue.getMessageID() == idImage || queue.getMessageID() == idImageUpper || queue.getMessageID() == idJPEGImage || queue.getMessageID() == idJPEGImageUpper
          || queue.getMessageID() == idThumbnail || queue.getMessageID() == idThumbnailUpper || queue.getMessageID() == idYoloInput || queue.getMessageID() == idYoloInputUpper
          || (queue.getMessageID() == idLowFrameRateImage && queue.getMessageSize() > 1000) || (queue.getMessageID() == idLowFrameRateImageUpper && queue.getMessageSize() > 1000)
          || (queue.getMessageID() == idSequenceImage && queue.getMessageSize() > 1000) || (queue.getMessageID() == idSequenceImageUpper && queue.getMessageSize() > 1000))
        lastImageFrameNumber = currentFrameNumber;
    } while (queue.getMessageID() != idProcessFinished);
    ++currentFrameNumber;
  }
}

void LogPlayer::stepImageForward()
{
  pause();
  if (state == paused && (currentFrameNumber < numberOfFrames - 1 || (loop && numberOfFrames > 0)))
  {
    int lastImageFrameNumber = this->lastImageFrameNumber;
    do
      stepForward();
    while (lastImageFrameNumber == this->lastImageFrameNumber && (currentFrameNumber < numberOfFrames - 1 || (loop && numberOfFrames > 0)));
  }
}

void LogPlayer::stepRepeat()
{
  pause();
  if (state == paused && currentFrameNumber >= 0)
  {
    --currentFrameNumber;
    currentMessageNumber = currentFrameNumber >= 0 ? frameIndex[currentFrameNumber] - 1 : 0;
    stepForward();
  }
}

void LogPlayer::gotoFrame(int frame)
{
  pause();
  if (state == paused && frame < numberOfFrames)
  {
    currentFrameNumber = frame - 1;
    currentMessageNumber = currentFrameNumber >= 0 ? frameIndex[currentFrameNumber] - 1 : 0;
    stepForward();
  }
}

bool LogPlayer::save(const char* fileName, const StreamHandler* streamHandler)
{
  if (state == recording)
    recordStop();

  if (!getNumberOfMessages())
    return false;

  OutBinaryFile file(fileName);
  if (file.exists())
  {
    file << logFileMessageIDs; // write magic byte to indicate message id table
    writeMessageIDs(file);
    if (streamHandler || this->streamHandler)
    {
      file << logFileStreamSpecification;
      file << (this->streamHandler ? *this->streamHandler : *streamHandler);
    }
    file << logFileUncompressed; // write magic byte to indicate uncompressed log file
    file << *this;
    return true;
  }
  return false;
}

bool LogPlayer::saveImages(const bool raw, const char* fileName)
{
  int i = 0;
  Image image;
  for (currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); currentMessageNumber++)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    if (queue.getMessageID() == idImage)
    {
      in.bin >> image;
    }
    else if (queue.getMessageID() == idImageUpper)
    {
      in.bin >> image;
    }
    else if (queue.getMessageID() == idJPEGImage)
    {
      JPEGImage jpegImage;
      in.bin >> jpegImage;
      jpegImage.toImage(image);
    }
    else if (queue.getMessageID() == idJPEGImageUpper)
    {
      JPEGImage jpegImage;
      in.bin >> jpegImage;
      jpegImage.toImage(image);
    }
    else if (queue.getMessageID() == idLowFrameRateImage)
    {
      LowFrameRateImage lowFrameRateImage;
      in.bin >> lowFrameRateImage;
      if (lowFrameRateImage.imageUpdated)
        image = lowFrameRateImage.image;
      else
        continue;
    }
    else if (queue.getMessageID() == idLowFrameRateImageUpper)
    {
      LowFrameRateImage lowFrameRateImage;
      in.bin >> lowFrameRateImage;
      if (lowFrameRateImage.imageUpdated)
        image = lowFrameRateImage.image;
      else
        continue;
    }
    else if (queue.getMessageID() == idSequenceImage)
    {
      SequenceImage sequenceImage;
      in.bin >> sequenceImage;
      if (sequenceImage.noInSequence > 0)
        image = sequenceImage.image;
      else
        continue;
    }
    else if (queue.getMessageID() == idSequenceImageUpper)
    {
      SequenceImage sequenceImage;
      in.bin >> sequenceImage;
      if (sequenceImage.noInSequence > 0)
        image = sequenceImage.image;
      else
        continue;
    }
    else
      continue;

    if (!saveImage(image, fileName, i++, !raw))
    {
      stop();
      return false;
    }
  }
  stop();
  return true;
}

void LogPlayer::recordStart()
{
  state = recording;
}

void LogPlayer::recordStop()
{
  while (getNumberOfMessages() > numberOfMessagesWithinCompleteFrames)
    removeLastMessage();
  currentMessageNumber = -1;
  currentFrameNumber = -1;
  state = initial;
}

void LogPlayer::setLoop(bool loop)
{
  this->loop = loop;
}

void LogPlayer::handleMessage(InMessage& message)
{
  if (state == recording)
  {
    message >> *this;
    if (message.getMessageID() == idProcessFinished)
    {
      numberOfMessagesWithinCompleteFrames = getNumberOfMessages();
      ++numberOfFrames;
    }
  }
}

bool LogPlayer::replay()
{
  if (state == playing)
  {
    if (currentFrameNumber < numberOfFrames - 1)
    {
      replayStreamSpecification();
      do
      {
        copyMessage(++currentMessageNumber, targetQueue);
        if (queue.getMessageID() == idImage || queue.getMessageID() == idJPEGImage || queue.getMessageID() == idImageUpper || queue.getMessageID() == idJPEGImageUpper
            || queue.getMessageID() == idThumbnail || queue.getMessageID() == idThumbnailUpper || queue.getMessageID() == idYoloInput || queue.getMessageID() == idYoloInputUpper
            || (queue.getMessageID() == idLowFrameRateImage && queue.getMessageSize() > 1000) || (queue.getMessageID() == idLowFrameRateImageUpper && queue.getMessageSize() > 1000)
            || (queue.getMessageID() == idSequenceImage && queue.getMessageSize() > 1000) || (queue.getMessageID() == idSequenceImageUpper && queue.getMessageSize() > 1000))
          lastImageFrameNumber = currentFrameNumber;

      } while (queue.getMessageID() != idProcessFinished && currentMessageNumber < numberOfMessagesWithinCompleteFrames - 1);
      ++currentFrameNumber;
      if (currentFrameNumber == numberOfFrames - 1)
      {
        SystemCall::text2Speech("allright");
        if (loop) //restart in loop mode
        {
          gotoFrame(0);
          play();
        }
        else
        {
          stop();
        }
      }
      return true;
    }
    else
    {
      SystemCall::text2Speech("allright");
      if (loop) //restart in loop mode
      {
        gotoFrame(0);
        play();
      }
      else
      {
        stop();
      }
    }
  }
  return false;
}

void LogPlayer::keep(MessageID* messageIDs)
{
  stop();
  LogPlayer temp((MessageQueue&)*this);
  temp.setSize(queue.getSize());
  moveAllMessages(temp);
  for (temp.currentMessageNumber = 0; temp.currentMessageNumber < temp.getNumberOfMessages(); ++temp.currentMessageNumber)
  {
    temp.queue.setSelectedMessageForReading(temp.currentMessageNumber);
    MessageID* m = messageIDs;
    while (*m)
    {
      if (temp.queue.getMessageID() == *m || temp.queue.getMessageID() == idProcessBegin || temp.queue.getMessageID() == idProcessFinished)
      {
        temp.copyMessage(temp.currentMessageNumber, *this);
        break;
      }
      ++m;
    }
  }
  std::tie(numberOfFrames, numberOfMessagesWithinCompleteFrames) = queue.countFramesAndMessages();

  if (!frameIndex.empty())
    createFrameIndex();
}

void LogPlayer::keep(int startFrame, int endFrame)
{
  if (startFrame < 0 || startFrame >= numberOfFrames)
    return;
  if (endFrame < 0 || endFrame >= numberOfFrames)
    return;

  stop();

  LogPlayer temp((MessageQueue&)*this);
  temp.setSize(queue.getSize());
  moveAllMessages(temp);

  const int begin = frameIndex[startFrame];
  const int end = endFrame == numberOfFrames - 1 ? numberOfMessagesWithinCompleteFrames : frameIndex[endFrame + 1];

  for (temp.currentMessageNumber = begin; temp.currentMessageNumber < end; ++temp.currentMessageNumber)
    temp.copyMessage(temp.currentMessageNumber, *this);

  std::tie(numberOfFrames, numberOfMessagesWithinCompleteFrames) = queue.countFramesAndMessages();
  if (!frameIndex.empty())
    createFrameIndex();
}

void LogPlayer::remove(MessageID* messageIDs)
{
  stop();
  LogPlayer temp((MessageQueue&)*this);
  temp.setSize(queue.getSize());
  moveAllMessages(temp);
  for (temp.currentMessageNumber = 0; temp.currentMessageNumber < temp.getNumberOfMessages(); ++temp.currentMessageNumber)
  {
    temp.queue.setSelectedMessageForReading(temp.currentMessageNumber);
    MessageID* m = messageIDs;
    while (*m)
    {
      if (temp.queue.getMessageID() == *m)
        break;
      ++m;
    }
    if (!*m)
      temp.copyMessage(temp.currentMessageNumber, *this);
  }
  std::tie(numberOfFrames, numberOfMessagesWithinCompleteFrames) = queue.countFramesAndMessages();
  if (!frameIndex.empty())
    createFrameIndex();
}

void LogPlayer::remove(int startFrame, int endFrame)
{
  if (startFrame < 0 || startFrame >= numberOfFrames)
    return;
  if (endFrame < 0 || endFrame >= numberOfFrames)
    return;

  stop();

  LogPlayer temp((MessageQueue&)*this);
  temp.setSize(queue.getSize());
  moveAllMessages(temp);

  const int begin = frameIndex[startFrame];
  const int end = endFrame == numberOfFrames - 1 ? numberOfMessagesWithinCompleteFrames : frameIndex[endFrame + 1];

  for (temp.currentMessageNumber = 0; temp.currentMessageNumber < begin; ++temp.currentMessageNumber)
    temp.copyMessage(temp.currentMessageNumber, *this);

  for (temp.currentMessageNumber = end; temp.currentMessageNumber < temp.getNumberOfMessages(); ++temp.currentMessageNumber)
    temp.copyMessage(temp.currentMessageNumber, *this);

  std::tie(numberOfFrames, numberOfMessagesWithinCompleteFrames) = queue.countFramesAndMessages();
  if (!frameIndex.empty())
    createFrameIndex();
}

void LogPlayer::statistics(int frequencies[numOfDataMessageIDs], unsigned* sizes, char processIdentifier)
{
  for (int i = 0; i < numOfDataMessageIDs; ++i)
    frequencies[i] = 0;
  if (sizes)
    for (int i = 0; i < numOfDataMessageIDs; ++i)
      sizes[i] = 0;

  if (getNumberOfMessages() > 0)
  {
    int current = queue.getSelectedMessageForReading();
    char currentProcess = 0;
    for (int i = 0; i < getNumberOfMessages(); ++i)
    {
      queue.setSelectedMessageForReading(i);
      ASSERT(queue.getMessageID() < numOfDataMessageIDs);
      if (queue.getMessageID() == idProcessBegin)
        currentProcess = queue.getData()[0];
      if (!processIdentifier || processIdentifier == currentProcess)
      {
        ++frequencies[queue.getMessageID()];
        if (sizes)
          sizes[queue.getMessageID()] += queue.getMessageSize() + 4;
      }
    }
    queue.setSelectedMessageForReading(current);
  }
}

void LogPlayer::createFrameIndex()
{
  queue.createIndex();
  frameIndex.clear();
  frameIndex.reserve(numberOfFrames);
  for (int i = 0; i < getNumberOfMessages(); ++i)
  {
    queue.setSelectedMessageForReading(i);
    if (queue.getMessageID() == idProcessBegin)
      frameIndex.push_back(i);
  }
}

std::string LogPlayer::expandImageFileName(const char* fileName, int imageNumber)
{
  std::string name(fileName);
  std::string ext(".png");
  std::string::size_type p = (int)name.rfind('.');
  if ((int)p > (int)name.find_last_of("\\/"))
  {
    ext = name.substr(p);
    name = name.substr(0, p);
  }

  if (imageNumber >= 0)
  {
    char num[12];
    sprintf(num, "%03d", imageNumber);
    name = name + "_" + num + ext;
  }
  else
    name += ext;

  for (unsigned i = 0; i < name.size(); ++i)
    if (name[i] == '\\')
      name[i] = '/';
  if (name[0] != '/' && (name.size() < 2 || name[1] != ':'))
    name = File::getBHDir() + std::string("/Config/") + name;
  return name;
}

bool LogPlayer::saveImage(const Image& image, const char* fileName, int imageNumber, bool YUV2RGB)
{
  std::string name = expandImageFileName(fileName, imageNumber);
  const Image* r = &image;
  if (YUV2RGB)
  {
    r = new Image;
    const_cast<Image*>(r)->convertFromYCbCrToRGB(image);
  }
  QImage img(image.width, image.height, QImage::Format_RGB888);
  for (int y = 0; y < image.height; ++y)
  {
    const Image::Pixel* pSrc = &(*r)[y][0];
    unsigned char *p = img.scanLine(y), *pEnd = p + 3 * image.width;
    if (YUV2RGB)
      while (p != pEnd)
      {
        *p++ = pSrc->r;
        *p++ = pSrc->g;
        *p++ = pSrc->b;
        ++pSrc;
      }
    else
      while (p != pEnd)
      {
        *p++ = pSrc->y;
        *p++ = pSrc->cb;
        *p++ = pSrc->cr;
        ++pSrc;
      }
  }
  if (YUV2RGB)
    delete r;
  return img.save(name.c_str());
}

bool LogPlayer::writeTimingData(const std::string& fileName)
{
  stop();

  map<unsigned short, string> names; /**<contains a mapping from watch id to watch name */
  map<unsigned, map<unsigned short, unsigned>> timings; /**<Contains a map from watch id to timing for each existing frame*/
  map<unsigned, unsigned> processStartTimes; /**< After parsing this contains the start time of each frame (frames may be missing) */
  for (int currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); currentMessageNumber++)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    if (queue.getMessageID() == idStopwatch)
    { //NOTE: this parser is a slightly modified version of the on in TimeInfo
      //first get the names
      unsigned short nameCount;
      in.bin >> nameCount;

      for (int i = 0; i < nameCount; ++i)
      {
        string watchName;
        unsigned short watchId;
        in.bin >> watchId;
        in.bin >> watchName;
        if (names.find(watchId) == names.end()) //new name
        {
          names[watchId] = watchName;
        }
      }

      //now get timing data
      unsigned short dataCount;
      in.bin >> dataCount;

      map<unsigned short, unsigned> frameTiming;
      for (int i = 0; i < dataCount; ++i)
      {
        unsigned short watchId;
        unsigned time;
        in.bin >> watchId;
        in.bin >> time;

        frameTiming[watchId] = time;
      }
      unsigned processStartTime;
      in.bin >> processStartTime;
      unsigned frameNo;
      in.bin >> frameNo;

      timings[frameNo] = frameTiming;
      processStartTimes[frameNo] = processStartTime;
    }
  }

  //now write the data to disk
  OutTextRawFile file(fileName);
  if (!file.exists())
    return false;

  unordered_map<unsigned short, int> columns; /**< mapping from stopwatch id to table column*/
  const string sep(",");
  //write header
  file << "Frame" << sep << "StartTime" << sep;
  int column = 0;
  for (map<unsigned short, string>::iterator it = names.begin(); it != names.end(); ++it, ++column)
  {
    columns[it->first] = column;
    file << it->second << sep;
  }
  file << endl;

  //write data
  map<unsigned, unsigned>::iterator it;
  for (it = processStartTimes.begin(); it != processStartTimes.end(); ++it)
  {
    const unsigned frameNo = it->first;
    vector<long> row;
    row.resize(column, -42); //-42 is a value that can never happen because all timings are unsigned
    map<unsigned short, unsigned>& timing = timings[frameNo];
    for (const auto& t : timing)
    {
      int colIndex = columns[t.first];
      row[colIndex] = t.second;
    }

    file << frameNo << sep << it->second << sep;
    for (long value : row)
    { //write timing data
      if (value == -42)
      {
        file << "NO DATA" << sep;
      }
      else
      {
        file << value / 1000.0f << sep; // division by 1000 to convert to ms
      }
    }
    file << endl;
  }
  return true;
}

bool LogPlayer::writeAudioFile(const char* fileName, std::vector<int> audioCandidates)
{
  OutBinaryFile stream(fileName);
  if (!stream.exists())
    return false;

  int frames = 0;
  AudioData audioData;
  for (size_t i = 0; i < audioCandidates.size(); i++)
  {
    queue.setSelectedMessageForReading(audioCandidates[i]);
    if (queue.getMessageID() == idAudioData)
    {
      in.bin >> audioData;
      if (audioData.channels > 0)
        frames += unsigned(audioData.samples.size()) / audioData.channels;
    }
  }

  struct WAVHeader
  {
    char chunkId[4];
    int chunkSize;
    char format[4];
    char subchunk1Id[4];
    int subchunk1Size;
    short audioFormat;
    short numChannels;
    int sampleRate;
    int byteRate;
    short blockAlign;
    short bitsPerSample;
    char subchunk2Id[4];
    int subchunk2Size;
  };

  int length = sizeof(WAVHeader) + sizeof(float) * frames * audioData.channels;
  WAVHeader* header = (WAVHeader*)new char[length];
  *(unsigned*)header->chunkId = *(const unsigned*)"RIFF";
  header->chunkSize = length - 8;
  *(unsigned*)header->format = *(const unsigned*)"WAVE";

  *(unsigned*)header->subchunk1Id = *(const unsigned*)"fmt ";
  header->subchunk1Size = 16;
  header->audioFormat = 3;
  header->numChannels = (short)audioData.channels;
  header->sampleRate = audioData.sampleRate;
  header->byteRate = audioData.sampleRate * audioData.channels * sizeof(float);
  header->blockAlign = short(audioData.channels * sizeof(float));
  header->bitsPerSample = 32;

  *(unsigned*)header->subchunk2Id = *(const unsigned*)"data";
  header->subchunk2Size = frames * audioData.channels * sizeof(float);

  char* p = (char*)(header + 1);
  for (size_t i = 0; i < audioCandidates.size(); i++)
  {
    queue.setSelectedMessageForReading(audioCandidates[i]);
    if (queue.getMessageID() == idAudioData)
    {
      in.bin >> audioData;
      memcpy(p, audioData.samples.data(), audioData.samples.size() * sizeof(float));
      p += audioData.samples.size() * sizeof(float);
    }
  }

  stream.write(header, length);
  delete[] header;

  stop();

  return true;
}

bool LogPlayer::saveAudioFile(const char* fileName)
{
  AudioData audioData;
  std::vector<int> audioCandidates;
  for (currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); ++currentMessageNumber)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    if (queue.getMessageID() == idAudioData)
    {
      audioCandidates.push_back(currentMessageNumber);
    }
  }

  return writeAudioFile(fileName, audioCandidates);
}

int LogPlayer::saveTrueWhistleAudioFile(const char* fileName, bool split)
{
  int part = 0;
  AudioData audioData;
  RawGameInfo rawGameInfo;
  OwnTeamInfo ownTeamInfo;
  OpponentTeamInfo oppenentTeamInfo;
  int lastRawGameInfo = 0;
  int lastOwnTeamInfoScore = 0;
  int lastOppenentTeamInfoScore = 0;
  std::vector<int> messageNumberPlay;
  std::vector<int> messageNumberGoal;
  std::vector<int> messageNumberAudio;
  for (currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); ++currentMessageNumber)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    if (queue.getMessageID() == idRawGameInfo)
    {
      in.bin >> rawGameInfo;
      if (rawGameInfo.state == STATE_PLAYING && lastRawGameInfo == STATE_SET)
        messageNumberPlay.push_back(currentMessageNumber);
      lastRawGameInfo = rawGameInfo.state;
    }

    if (queue.getMessageID() == idOwnTeamInfo)
    {
      in.bin >> ownTeamInfo;
      if (ownTeamInfo.score > lastOwnTeamInfoScore)
        messageNumberGoal.push_back(currentMessageNumber);
      lastOwnTeamInfoScore = ownTeamInfo.score;
    }

    if (queue.getMessageID() == idOpponentTeamInfo)
    {
      in.bin >> oppenentTeamInfo;
      if (oppenentTeamInfo.score > lastOppenentTeamInfoScore)
        messageNumberGoal.push_back(currentMessageNumber);
      lastOppenentTeamInfoScore = oppenentTeamInfo.score;
    }

    if (queue.getMessageID() == idAudioData)
    {
      messageNumberAudio.push_back(currentMessageNumber);
    }
  }

  std::vector<int> audioCandidates;
  RingBuffer<int, 590> windowLeft;
  for (size_t i = 0; i < messageNumberPlay.size(); i++)
  {
    int id = messageNumberPlay[i];
    int after = 10;
    std::vector<int> windowRight;
    for (size_t j = 0; j < messageNumberAudio.size(); j++)
    {
      if (messageNumberAudio[j] > id)
      {
        if (after > 0)
        {
          windowRight.push_back(messageNumberAudio[j]);
          after--;
        }
        else
        {
          break;
        }
      }
      else
      {
        windowLeft.push_front(messageNumberAudio[j]);
      }
    }

    for (ptrdiff_t j = windowLeft.size() - 1; j >= 0; j--)
    {
      audioCandidates.push_back(windowLeft[j]);
    }
    windowLeft.clear();

    for (size_t j = 0; j < windowRight.size(); j++)
    {
      audioCandidates.push_back(windowRight[j]);
    }

    if (split)
    {
      if (audioCandidates.size() > 0)
      {
        std::string partName = fileName;
        partName.insert(partName.size() - 4, "_part_" + std::to_string(part));
        part++;
        bool ret = writeAudioFile(partName.c_str(), audioCandidates);

        if (!ret)
          return 2;

        audioCandidates.clear();
      }
    }
  }

  RingBuffer<int, 590> goalWindowLeft;
  for (size_t i = 0; i < messageNumberGoal.size(); i++)
  {
    int id = messageNumberGoal[i];
    int after = 10;
    std::vector<int> goalWindowRight;
    for (size_t j = 0; j < messageNumberAudio.size(); j++)
    {
      if (messageNumberAudio[j] > id)
      {
        if (after > 0)
        {
          goalWindowRight.push_back(messageNumberAudio[j]);
          after--;
        }
        else
        {
          break;
        }
      }
      else
      {
        goalWindowLeft.push_front(messageNumberAudio[j]);
      }
    }

    for (ptrdiff_t j = goalWindowLeft.size() - 1; j >= 0; j--)
    {
      audioCandidates.push_back(goalWindowLeft[j]);
    }
    goalWindowLeft.clear();

    for (size_t j = 0; j < goalWindowRight.size(); j++)
    {
      audioCandidates.push_back(goalWindowRight[j]);
    }

    if (split)
    {
      if (audioCandidates.size() > 0)
      {
        std::string partName = fileName;
        partName.insert(partName.size() - 4, "_part_" + std::to_string(part));
        part++;
        bool ret = writeAudioFile(partName.c_str(), audioCandidates);

        if (!ret)
          return 2;

        audioCandidates.clear();
      }
    }
  }

  if (messageNumberGoal.size() + messageNumberPlay.size() == 0)
    return 1;

  if (!split)
  {
    bool ret = writeAudioFile(fileName, audioCandidates);

    if (ret)
      return 0;
    else
      return 2;
  }

  return 0;
}

int LogPlayer::saveFalseWhistleAudioFile(const char* fileName, bool split)
{
  int part = 0;
  AudioData audioData;
  RawGameInfo rawGameInfo;
  OwnTeamInfo ownTeamInfo;
  OpponentTeamInfo oppenentTeamInfo;
  std::vector<char> annotation;
  int lastRawGameInfo = 0;
  int lastOwnTeamInfoScore = 0;
  int lastOppenentTeamInfoScore = 0;
  std::vector<int> messageNumberDetection;
  std::vector<int> messageNumberFalsePositive;
  std::vector<int> messageNumberPlay;
  std::vector<int> messageNumberGoal;
  std::vector<int> messageNumberAudio;
  std::vector<int> annotationCount;
  annotationCount.push_back(0);
  int annotationIdx = 0;
  int lastMessageID = 0;
  for (currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); ++currentMessageNumber)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    if (queue.getMessageID() == idAnnotation) // find detected whistle
    {
      annotation.resize(queue.getMessageSize());
      in.bin.read(&annotation[0], queue.getMessageSize());
      std::string anno(annotation.begin(), annotation.end());
      if (anno.find("Whistle was detected.") != std::string::npos)
      {
        messageNumberDetection.push_back(currentMessageNumber);
      }
      annotationCount[annotationIdx]++;
    }
    else
    {
      if (lastMessageID == idAnnotation)
      {
        annotationCount.push_back(0);
        annotationIdx++;
      }
    }

    // find true whistle
    if (queue.getMessageID() == idRawGameInfo)
    {
      in.bin >> rawGameInfo;
      if (rawGameInfo.state == STATE_PLAYING && lastRawGameInfo == STATE_SET)
        messageNumberPlay.push_back(currentMessageNumber);
      lastRawGameInfo = rawGameInfo.state;
    }

    if (queue.getMessageID() == idOwnTeamInfo)
    {
      in.bin >> ownTeamInfo;
      if (ownTeamInfo.score > lastOwnTeamInfoScore)
        messageNumberGoal.push_back(currentMessageNumber);
      lastOwnTeamInfoScore = ownTeamInfo.score;
    }

    if (queue.getMessageID() == idOpponentTeamInfo)
    {
      in.bin >> oppenentTeamInfo;
      if (oppenentTeamInfo.score > lastOppenentTeamInfoScore)
        messageNumberGoal.push_back(currentMessageNumber);
      lastOppenentTeamInfoScore = oppenentTeamInfo.score;
    }

    if (queue.getMessageID() == idAudioData)
    {
      messageNumberAudio.push_back(currentMessageNumber);
    }

    lastMessageID = queue.getMessageID();
  }
  // save only false positives
  for (size_t detectedWhistleID = 0; detectedWhistleID < messageNumberDetection.size(); detectedWhistleID++)
  {
    for (size_t currentMessageNumberPlay = 0; currentMessageNumberPlay < messageNumberPlay.size(); currentMessageNumberPlay++)
    {
      if (currentMessageNumberPlay < detectedWhistleID - (MessageID::numOfDataMessageIDs + annotationCount[detectedWhistleID])
          || currentMessageNumberPlay > detectedWhistleID + MessageID::numOfDataMessageIDs + annotationCount[detectedWhistleID])
      {
        messageNumberFalsePositive.push_back(static_cast<int>(detectedWhistleID));
      }
    }
    for (size_t currentMessageNumberGoal = 0; currentMessageNumberGoal < messageNumberGoal.size(); currentMessageNumberGoal++)
    {
      if (currentMessageNumberGoal < detectedWhistleID - (MessageID::numOfDataMessageIDs + annotationCount[detectedWhistleID])
          || currentMessageNumberGoal > detectedWhistleID + MessageID::numOfDataMessageIDs + annotationCount[detectedWhistleID])
      {
        messageNumberFalsePositive.push_back(static_cast<int>(detectedWhistleID));
      }
    }
  }

  std::vector<int> audioCandidates;
  RingBuffer<int, 590> windowLeft;
  for (size_t i = 0; i < messageNumberFalsePositive.size(); i++)
  {
    int id = messageNumberFalsePositive[i];
    int after = 10;
    std::vector<int> windowRight;
    for (size_t j = 0; j < messageNumberAudio.size(); j++)
    {
      if (messageNumberAudio[j] > id)
      {
        if (after > 0)
        {
          windowRight.push_back(messageNumberAudio[j]);
          after--;
        }
        else
        {
          break;
        }
      }
      else
      {
        windowLeft.push_front(messageNumberAudio[j]);
      }
    }

    for (ptrdiff_t j = windowLeft.size() - 1; j >= 0; j--)
    {
      audioCandidates.push_back(windowLeft[j]);
    }
    windowLeft.clear();

    for (size_t j = 0; j < windowRight.size(); j++)
    {
      audioCandidates.push_back(windowRight[j]);
    }

    if (split)
    {
      if (audioCandidates.size() > 0)
      {
        std::string partName = fileName;
        partName.insert(partName.size() - 4, "_part_" + std::to_string(part));
        part++;
        bool ret = writeAudioFile(partName.c_str(), audioCandidates);

        if (!ret)
          return 2;

        audioCandidates.clear();
      }
    }
  }

  if (messageNumberFalsePositive.size() == 0)
    return 1;

  if (!split)
  {
    bool ret = writeAudioFile(fileName, audioCandidates);

    if (ret)
      return 0;
    else
      return 2;
  }

  return 0;
}

void LogPlayer::replayStreamSpecification()
{
  if (streamHandler && !streamSpecificationReplayed)
  {
    targetQueue.out.bin << *streamHandler;
    targetQueue.out.finishMessage(idStreamSpecification);
    streamSpecificationReplayed = true;
  }
}

void LogPlayer::export_data(const std::string& file, const std::list<std::string>& ids)
{
  std::ofstream f(File::getBHDir() + std::string("/Config/Logs/" + file));
  json jOut;
  for (currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); ++currentMessageNumber)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    if (ids.size() == 0 || std::find(ids.begin(), ids.end(), ::getName(queue.getMessageID())) != ids.end())
    {
      StreamHandler currentStreamHandler;

      const char* type = ::getName(in.getMessageID()) + 2;
      const char* t = streamHandler->getString(type);
      if (streamHandler->specification.find(t) != streamHandler->specification.end())
      {
        OutJSONSize outJSONSize(true);
        {
          DebugDataStreamer streamer(*streamHandler, in.bin, type);
          outJSONSize << streamer;
          // outJSONSize.finish();
          in.resetReadPosition();
        }

        std::vector<char> jsonBuffer(outJSONSize.getSize() + 1, 0);
        {
          OutJSONMemory outJSON(jsonBuffer.data(), true);
          DebugDataStreamer streamer(*streamHandler, in.bin, type);
          outJSON << streamer;
          // outJSON.finish();
        }

        jsonBuffer[outJSONSize.getSize()] = 0;
        jOut.push_back({{"id", ::getName(queue.getMessageID())}, {"data", json::parse(jsonBuffer.data())}});
      }
    }
  }
  f << std::setw(4) << jOut;
}
