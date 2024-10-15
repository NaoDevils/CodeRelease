#include "AudioBuffer.h"
#include "Platform/File.h"
#include "Tools/Build.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/Constants.h"
#include <cmath>
#include <float.h>

AudioBuffer::AudioBuffer()
{
  windowSize = 1024;
  samplerate = 22050;
  channels = 4;
  ringbufferSize = 42;
  nnWeight = 0.8f;
  pmWeight = 0.6f;
  noiseWeight = 0.4f;
  noiseLimit = 0.25f;
  micBrokenMagSumThreshold = 0.2f;
  micBrokenThreshold = 10;
  resize2D(gradients, channels, windowSize / 2 + 1);
  resize2D(buffer, channels, windowSize);
  resize2D(helper, channels, windowSize / 2 + 1);
  resize2D(helperAmp, channels, windowSize / 2 + 1);
  resize2D(helperLogSpec, channels, windowSize / 2 + 1);
  resize2D(localFftBuffer, channels, windowSize);
  levels.resize(channels);
  spectrograms.resize(channels);
  magnitudes.resize(channels);
  logSpectrograms.resize(channels);
  drrs.resize(channels);
  maxMagHist.resize(channels);
  micsBrokenCount.resize(channels);
  micStatus.resize(channels);

  for (int c = 0; c < (int)channels; c++)
  {
    micStatus[c] = true;
  }

  confidences.reserve(ringbufferSize);
  if (ringbufferSize != 42)
  {
    for (unsigned int c = 0; c < channels; c++)
    {
      levels[c].reserve(ringbufferSize);
      spectrograms[c].reserve(ringbufferSize);
      magnitudes[c].reserve(ringbufferSize);
      logSpectrograms[c].reserve(ringbufferSize);
      drrs[c].reserve(ringbufferSize);
    }
  }
  numOfNewFrames = 0;
}

AudioBuffer::AudioBuffer(AudioBufferParameters parameters)
{
  windowSize = parameters.windowSize;
  samplerate = parameters.samplerate;
  channels = parameters.channels;
  ringbufferSize = parameters.ringbufferSize;
  nnWeight = parameters.nnWeight;
  pmWeight = parameters.pmWeight;
  noiseWeight = parameters.noiseWeight;
  noiseLimit = parameters.noiseLimit;
  micBrokenMagSumThreshold = parameters.micBrokenMagSumThreshold;
  micBrokenThreshold = parameters.micBrokenThreshold;

  nnWeight = parameters.nnWeight;
  pmWeight = parameters.pmWeight;
  noiseWeight = parameters.noiseWeight;
  overtoneBorderMultiplier = parameters.overtoneBorderMultiplier;
  minFreq = parameters.minFreq;
  currentMinFreq = minFreq;
  whistleMinMaxFreq[0] = currentMinFreq;
  minFreqBorder = parameters.minFreqBorder;
  permanentMinFreqBorder = minFreqBorder;
  maxFreq = parameters.maxFreq;
  currentMaxFreq = maxFreq;
  whistleMinMaxFreq[1] = currentMaxFreq;
  maxFreqBorder = parameters.maxFreqBorder;
  permanentMaxFreqBorder = maxFreqBorder;
  whistleCandidateThreshold = parameters.whistleCandidateThreshold;
  attentionMultiplier = parameters.attentionMultiplier;
  minWhistleFrames = parameters.minWhistleFrames;
  confidenceType = parameters.confidenceType;

  resize2D(gradients, channels, windowSize / 2 + 1);
  resize2D(buffer, channels, windowSize);
  resize2D(helper, channels, windowSize / 2 + 1);
  resize2D(helperAmp, channels, windowSize / 2 + 1);
  resize2D(helperLogSpec, channels, windowSize / 2 + 1);
  resize2D(localFftBuffer, channels, windowSize);
  levels.resize(channels);
  spectrograms.resize(channels);
  magnitudes.resize(channels);
  logSpectrograms.resize(channels);
  drrs.resize(channels);
  maxMagHist.resize(channels);
  micsBrokenCount.resize(channels);
  micStatus.resize(channels);

  for (int c = 0; c < (int)channels; c++)
  {
    micStatus[c] = true;
  }

  confidences.reserve(ringbufferSize);
  if (ringbufferSize != 42)
  {
    for (unsigned int c = 0; c < channels; c++)
    {
      levels[c].reserve(ringbufferSize);
      spectrograms[c].reserve(ringbufferSize);
      magnitudes[c].reserve(ringbufferSize);
      logSpectrograms[c].reserve(ringbufferSize);
      drrs[c].reserve(ringbufferSize);
    }
  }
  numOfNewFrames = 0;

  setupNeuralNetwork(parameters.modelPath);
}

unsigned int AudioBuffer::getNumOfNewFrames()
{
  return numOfNewFrames;
}


void AudioBuffer::add(std::vector<float> samples)
{
  unsigned int audioDataPos = 0;
  while (audioDataPos < samples.size())
  {
    unsigned int samplesLeft = (windowSize / 2) - prevSamplesLeft;
    while (audioDataPos < samples.size() && samplesLeft > 0)
    {
      for (unsigned int c = 0; c < channels; c++)
        buffer[c][ringPos] = samples[audioDataPos + c];

      audioDataPos += channels;
      samplesLeft--;
      ringPos = (ringPos + 1) % windowSize;
    }
    if (samplesLeft == 0)
    {
      for (unsigned int c = 0; c < channels; c++)
      {
        levels[c].push_front(buffer[c]);
        for (unsigned int i = 0; i < static_cast<unsigned int>(windowSize); i++)
        {
          localFftBuffer[c][i].r = buffer[c][i] * (0.54f - 0.46f * std::cos(2.f * pi * i / windowSize)); //Hamming Window
          localFftBuffer[c][i].i = 0.f;
        }
        kissFFT(localFftBuffer[c].data(), localFftBuffer[c].data());
        for (int i = 0; i < static_cast<int>(helper[c].size()); i++)
          helper[c][i] = localFftBuffer[c][i];
        spectrograms[c].push_front(helper[c]);
        float prevAmp = 0.f;
        for (unsigned int idxW = 0; idxW < static_cast<unsigned int>(windowSize / 2 + 1); idxW++)
        {
          helperAmp[c][idxW] = std::sqrt((helper[c][idxW].r * helper[c][idxW].r) + (helper[c][idxW].i * helper[c][idxW].i));
          helperLogSpec[c][idxW] = 20 * std::log10(helperAmp[c][idxW]);
          gradients[c][idxW] = helperAmp[c][idxW] - prevAmp;
          prevAmp = helperAmp[c][idxW];
        }
        magnitudes[c].push_front(helperAmp[c]);
        logSpectrograms[c].push_front(helperLogSpec[c]);

        if (spectrograms[c].size() >= whistleLength)
        {
          resize2D(tempDrr, channels, static_cast<int>(spectrograms[c][0].size()));
          resize2D(tempDirectSum, channels, static_cast<int>(spectrograms[c][0].size()));
          resize2D(tempReverbSum, channels, static_cast<int>(spectrograms[c][0].size()));

          for (int i = 0; i < static_cast<int>(spectrograms[c][0].size()); i++) // initialize with 0
          {
            tempDrr[c].push_back(0.f);
            tempDirectSum[c].push_back(0.f);
            tempReverbSum[c].push_back(0.f);
          }
          for (unsigned int idx = 0; idx < whistleLength; idx++)
          {
            for (int idx2 = 0; idx2 < static_cast<int>(magnitudes[c][idx].size()); idx2++)
            {
              if (idx < whistleLength - directSoundLength)
              {
                tempReverbSum[c][idx2] += magnitudes[c][idx][idx2] * magnitudes[c][idx][idx2];
              }
              else
              {
                tempDirectSum[c][idx2] += magnitudes[c][idx][idx2] * magnitudes[c][idx][idx2];
              }
            }
          }
          for (int idx3 = 0; idx3 < static_cast<int>(tempDirectSum[c].size()); idx3++)
          {
            tempDrr[c][idx3] = 10 * log10(tempDirectSum[c][idx3] / (tempReverbSum[c][idx3]));
          }
          drrs[c].push_front(tempDrr[c]);
        }
      }
      setMicStatus(helperAmp);
      confidences.push_front(calculateConfidences());
      numOfNewFrames++;
      prevSamplesLeft = 0;
    }
    else
    {
      prevSamplesLeft = samplesLeft;
    }
  }
}

int AudioBuffer::getMinFreqBorder()
{
  return minFreqBorder;
}

int AudioBuffer::getMaxFreqBorder()
{
  return maxFreqBorder;
}

std::vector<std::vector<unsigned int>> AudioBuffer::getWhistleCandidateIdx()
{
  unsigned int longestSequence = 0;
  unsigned int sequence = 0;
  bool whistleCandidate = false;
  bool prevWhistleCandidate = false;
  for (int i = 0; i < static_cast<int>(confidences.size()); i++)
  {
    whistleCandidate = confidences[i][confidenceType] > whistleCandidateThreshold;

    if (prevWhistleCandidate && whistleCandidate == prevWhistleCandidate)
    {
      sequence++;
    }
    else
    {
      if (sequence > longestSequence)
        longestSequence = sequence;
      sequence = 0;
    }

    prevWhistleCandidate = whistleCandidate;
  }
  unsigned int attentionLength = static_cast<unsigned int>(longestSequence * attentionMultiplier);

  prevWhistleCandidate = false;
  bool activeAttention = false;
  unsigned int attention = 0;
  unsigned int minIdx = 0;
  unsigned int maxIdx = 0;
  std::vector<std::vector<unsigned int>> detectedWhistles;
  for (int i = 0; i < static_cast<int>(confidences.size()); i++)
  {
    whistleCandidate = confidences[i][confidenceType] > whistleCandidateThreshold;

    if (!activeAttention && !prevWhistleCandidate && whistleCandidate)
    {
      minIdx = i;
      activeAttention = true;
    }
    else if (prevWhistleCandidate && !whistleCandidate)
    {
      maxIdx = i - 1;
    }

    if (activeAttention && attention > attentionLength && maxIdx - minIdx > minWhistleFrames)
    {
      std::vector<unsigned int> whistleIdx = {minIdx, maxIdx};
      detectedWhistles.push_back(whistleIdx);
      attention = 0;
      activeAttention = false;
    }

    if (activeAttention)
      attention++;

    prevWhistleCandidate = whistleCandidate;
  }

  if (attention >= confidences.size() - 1 && confidences[confidences.size() - 1][3] > 0.25)
  {
    std::vector<unsigned int> whistleIdx = {minIdx, static_cast<unsigned int>(confidences.size() - 1)};
    detectedWhistles.push_back(whistleIdx);
  }

  return detectedWhistles;
}

void AudioBuffer::setMinFreqBorder(unsigned int minFreqBorder)
{
  this->minFreqBorder = minFreqBorder;
  this->permanentMinFreqBorder = minFreqBorder;
}

void AudioBuffer::setMaxFreqBorder(unsigned int maxFreqBorder)
{
  this->maxFreqBorder = maxFreqBorder;
  this->permanentMaxFreqBorder = maxFreqBorder;
}

std::vector<std::vector<unsigned int>> AudioBuffer::getNewWhistleCandidateIdx()
{
  if (numOfNewFrames > ringbufferSize)
  {
    OUTPUT_WARNING("Number of new Frames (" + std::to_string(numOfNewFrames) + ") is greater than Buffersize (" + std::to_string(ringbufferSize) + ").");
    numOfNewFrames = ringbufferSize;
  }

  unsigned int roi = numOfNewFrames + whistleLength;

  std::vector<std::vector<unsigned int>> detectedWhistles;
  std::vector<std::vector<unsigned int>> detectedNewWhistles;
  std::vector<unsigned int> whistleIdx = {0, 0};
  detectedWhistles = getWhistleCandidateIdx();
  for (int idx = 0; idx < static_cast<int>(detectedWhistles.size()); idx++)
  {
    if (detectedWhistles[idx][0] < roi)
    {
      whistleIdx[0] = detectedWhistles[idx][0];
      whistleIdx[1] = detectedWhistles[idx][1];
      detectedNewWhistles.push_back(whistleIdx);
    }
    else
      break;
  }
  numOfNewFrames = 0;
  return detectedNewWhistles;
}

float AudioBuffer::getOvertoneBorderMultiplier()
{
  return overtoneBorderMultiplier;
}

void AudioBuffer::setOvertoneBorderMultiplier(float overtoneBorderMultiplier)
{
  this->overtoneBorderMultiplier = overtoneBorderMultiplier;
}

void AudioBuffer::resize2D(auto& buffer, unsigned int size1, unsigned int size2)
{
  buffer.resize(size1);
  if (size2 > 0)
    for (auto& buf : buffer)
      buf.resize(size2);
}

void AudioBuffer::resize3D(auto& vector, unsigned int size1, unsigned int size2, unsigned int size3)
{
  vector.resize(size1);
  for (auto& vec1 : vector)
  {
    vec1.resize(size2);
    for (auto& vec2 : vec1)
    {
      vec2.resize(size3);
    }
  }
}

std::vector<std::vector<std::vector<float>>> AudioBuffer::getLevels(unsigned int minIDX, unsigned int maxIDX)
{
  maxIDX = checkNumOfWindows(maxIDX);
  unsigned int numOfWindows = maxIDX - minIDX + 1;
  resize3D(output, channels, numOfWindows, windowSize);
  for (unsigned int c = 0; c < channels; c++)
  {
    for (unsigned int idxW = minIDX; idxW <= maxIDX; idxW++)
    {
      output[c][idxW - minIDX] = std::vector<float>(levels[c][idxW].begin(), levels[c][idxW].end());
    }
  }
  return output;
}

std::vector<std::vector<float>> AudioBuffer::getConfidences(unsigned int numOfWindows)
{
  resize2D(outGetConfidences, numOfWindows, 5);
  for (unsigned int idxW = 0; idxW < numOfWindows; idxW++)
  {
    for (unsigned int idx = 0; idx < 5; idx++)
    {
      outGetConfidences[idxW][idx] = confidences[idxW][idx];
    }
  }
  return outGetConfidences;
}

std::vector<float> AudioBuffer::nextConfidences()
{
  if (numOfNewFrames <= 0)
    return std::vector<float>();
  else
  {
    numOfNewFrames--;
    return confidences[numOfNewFrames + 1];
  }
}

std::vector<std::vector<std::vector<float>>> AudioBuffer::getLogSpectrograms(unsigned int minIDX, unsigned int maxIDX)
{
  maxIDX = checkNumOfWindows(maxIDX);
  unsigned int numOfWindows = maxIDX - minIDX + 1;
  resize3D(output, channels, numOfWindows, windowSize / 2 + 1);

  if (!logSpectrograms[0].full() && logSpectrograms.size() - 1 < maxIDX)
    maxIDX = static_cast<unsigned int>(logSpectrograms.size() - 1);

  for (unsigned int c = 0; c < channels; c++)
  {
    for (unsigned int idxW = minIDX; idxW <= maxIDX; idxW++)
    {
      for (unsigned int idxW2 = 0; idxW2 < windowSize / 2 + 1; idxW2++)
      {
        output[c][idxW - minIDX][idxW2] = logSpectrograms[c][idxW][idxW2]; //20*log10(abs(spec))
      }
    }
  }
  return output;
}

std::vector<std::vector<std::vector<float>>> AudioBuffer::getPhases(unsigned int minIDX, unsigned int maxIDX)
{
  maxIDX = checkNumOfWindows(maxIDX);
  unsigned int numOfWindows = maxIDX - minIDX + 1;
  resize3D(output, channels, numOfWindows, windowSize / 2 + 1);
  //std::atan2(spec.i, spec.r)
  for (unsigned int c = 0; c < channels; c++)
  {
    for (unsigned int idxW = minIDX; idxW <= maxIDX; idxW++)
    {
      for (unsigned int idxW2 = 0; idxW2 < windowSize / 2 + 1; idxW2++)
      {

        output[c][idxW - minIDX][idxW2] = std::atan2(spectrograms[c][idxW][idxW2].i, spectrograms[c][idxW][idxW2].r);
      }
    }
  }
  return output;
}

std::vector<std::vector<std::vector<float>>> AudioBuffer::getDrrs(unsigned int minIDX, unsigned int maxIDX)
{
  maxIDX = checkNumOfWindows(maxIDX);
  unsigned int numOfWindows = maxIDX - minIDX + 1;
  resize3D(output, channels, numOfWindows, windowSize / 2 + 1);
  for (unsigned int c = 0; c < channels; c++)
  {
    for (unsigned int idxW = minIDX; idxW <= maxIDX; idxW++)
    {
      for (unsigned int idxW2 = 0; idxW2 < windowSize / 2 + 1; idxW2++)
      {
        output[c][idxW - minIDX][idxW2] = drrs[c][idxW][idxW2];
      }
    }
  }
  return output;
}

std::vector<bool> AudioBuffer::getMicStatus()
{
  return micStatus;
}

void AudioBuffer::kissFFT(kiss_fft_cpx in[], kiss_fft_cpx out[])
{
  kiss_fft_cfg cfg;

  if ((cfg = kiss_fft_alloc(windowSize, 0 /*is_inverse_fft*/, NULL, NULL)) != NULL)
  {
    kiss_fft(cfg, in, out);
    free(cfg);
  }
  else
  {
    OUTPUT_WARNING("Not enough memory for KissFFT?");
  }
}

unsigned int AudioBuffer::checkNumOfWindows(unsigned int numOfWindows)
{
  if (numOfWindows > ringbufferSize)
  {
    OUTPUT_WARNING("Number of windows out of bounds, set number of windows to max size!");
    return ringbufferSize;
  }
  return numOfWindows;
}

void AudioBuffer::setMicStatus(std::vector<std::vector<float>> magnitudes)
{
  std::vector<float> magSum(channels);
  std::vector<float> limitCount(channels);
  int magSize = windowSize / 2 + 1;
  for (unsigned int c = 0; c < channels; c++)
  {
    magSum[c] = 0.f;
    limitCount[c] = 0.f;
    currentMaxMag[c] = -FLT_MAX;
    currentMinMag[c] = FLT_MAX;
  }

  for (unsigned int c = 0; c < channels; c++)
  {
    for (int i = 0; i < magSize; i++)
    {
      if (magnitudes[c][i] > currentMaxMag[c])
        currentMaxMag[c] = magnitudes[c][i];

      if (magnitudes[c][i] < currentMinMag[c])
        currentMinMag[c] = magnitudes[c][i];

      if (magnitudes[c][i] > noiseLimit)
        limitCount[c]++;

      magSum[c] = magSum[c] + magnitudes[c][i];
    }
    relNoise[c] = limitCount[c] / magSize;
    maxMagHist[c].push_front(currentMaxMag[c]);
    currentMeanMag[c] = magSum[c] / magSize;

    if (magSum[c] < micBrokenMagSumThreshold)
    {
      micsBrokenCount[c]++;
      if (micsBrokenCount[c] >= micBrokenThreshold)
      {
        micStatus[c] = false;
        micsBrokenCount[c] = 0;
      }
    }
    else
    {
      micsBrokenCount[c] = 0;
      micStatus[c] = true;
    }
  }
}

int AudioBuffer::getCurrentMic()
{
  for (unsigned int c = 0; c < channels; c++)
  {
    if (micStatus[c])
    {
      return c;
    }
  }
  return -1;
}

void AudioBuffer::setupNeuralNetwork(std::string modelPath)
{
  std::string filename = std::string(File::getBHDir()) + modelPath;
  oldModelPathPath = modelPath;

  // Load the model
  model = tflite::FlatBufferModel::BuildFromFile(filename.c_str());

  if (model == nullptr)
  {
    OUTPUT_WARNING("Model not found");
  }
  else
  {
    // Build the interpreter
    tflite::InterpreterBuilder builder(*model, resolver);
    builder(&interpreter);

    interpreter->SetNumThreads(1);

    // Resize input tensors
    inputTensor = interpreter->inputs()[0];
    outputTensor = interpreter->outputs()[0];
    TfLiteIntArray* inputDims = interpreter->tensor(inputTensor)->dims;
    //int batchSize = inputDims->data[0];
    int inputSize = inputDims->data[1];
    int channels = inputDims->data[2];

    std::vector<int> newInput;
    newInput.push_back(1); // Set batch size to one
    newInput.push_back(inputSize);
    newInput.push_back(channels);
    interpreter->ResizeInputTensor(inputTensor, newInput);

    // Allocate memory for the tensors
    interpreter->AllocateTensors();
  }
}

std::vector<float> AudioBuffer::calculateConfidences()
{
  int currentMic = getCurrentMic();
  float pmConfidence = 0.f;
  float nnConfidence = 0.f;
  int magSize = windowSize / 2 + 1;

  if (currentMic == -1)
  {
    std::vector<float> errorValue = {0.f, 0.f, 0.f, 0.f, 0.f};
    return errorValue;
  }

  for (int i = 0; i < magSize; i++)
  {
    interpreter->typed_tensor<float>(inputTensor)[i] = logSpectrograms[currentMic][0][i];
  }


  // Do whistle detection PM
  int peakPos = -1, overtonePeakPos = -1;
  int minPos = currentMinFreq * windowSize / samplerate;
  int maxPos = currentMaxFreq * windowSize / samplerate;

  // Find whistle peak between min. freq. position and  max. freq. position
  peakPos = minPos;
  for (int i = minPos; i <= maxPos; i++)
  {
    if (magnitudes[currentMic][0][i] > magnitudes[currentMic][0][peakPos])
      peakPos = i;
  }

  if (currentMaxMag[currentMic] == 0)
    OUTPUT_ERROR("MaxAmpZero");

  float magWeight = magnitudes[currentMic][0][peakPos] / currentMaxMag[currentMic];

  // Get min/max gradients around peakPos
  float maxGrad = gradients[currentMic][peakPos];
  float minGrad = gradients[currentMic][std::min(peakPos + 1, magSize - 1)];
  maxGrad = std::abs(maxGrad / currentMaxMag[currentMic]);
  minGrad = std::abs(minGrad / currentMaxMag[currentMic]);

  if (magnitudes[currentMic][0][peakPos] >= maxMagHist[currentMic].average())
  {
    int roi = maxPos - minPos;
    minPos = static_cast<int>((peakPos * 2) - ((roi / 2) * overtoneBorderMultiplier)); // freq. position * overtone - ROI/2 * overtoneBorderMultiplier
    maxPos = static_cast<int>((peakPos * 2) + ((roi / 2) * overtoneBorderMultiplier)); // freq. position * overtone + ROI/2 * overtoneBorderMultiplier

    // Find first overtone peak between min and max overtone freq. position
    overtonePeakPos = minPos;
    for (int i = minPos; i <= maxPos; i++)
    {
      if (magnitudes[currentMic][0][i] > magnitudes[currentMic][0][overtonePeakPos])
        overtonePeakPos = i;
    }

    // Detect whistle
    if (magnitudes[currentMic][0][overtonePeakPos] >= currentMeanMag[currentMic])
    {
      pmConfidence = std::max(minGrad, maxGrad) * magWeight;
    }
    else
    {
      pmConfidence = std::min(minGrad, maxGrad) * magWeight;
    }
  }
  else
  {
    pmConfidence = std::min(minGrad, maxGrad) * magWeight;
  }

  float* output;
  // Do readout the whistle detection NN
  output = interpreter->typed_tensor<float>(outputTensor);

  if (interpreter->Invoke() != kTfLiteOk)
    OUTPUT_ERROR("Failed to invoke whistle tflite!");

  nnConfidence = *output;


  float fusedConfidence = ((nnWeight * nnConfidence + pmWeight * pmConfidence) / (nnWeight + pmWeight)) * (1 - (noiseWeight * relNoise[currentMic]));

  std::vector<float> confidences = {pmConfidence,
      nnConfidence,
      relNoise[currentMic],
      fusedConfidence,
      (float)std::min(std::max((int)peakPos * (int)samplerate / (int)windowSize, 0), ((int)windowSize / 2 + 1) * (int)samplerate / (int)windowSize)};

  return confidences;
}

std::array<unsigned int, 2> AudioBuffer::calibrateFrequency(unsigned int detectedFrequency)
{
  whistleFreqBuffer.push_front(detectedFrequency);
  int minDiff = std::abs(whistleFreqBuffer.average() - currentMinFreq);
  int maxDiff = std::abs(whistleFreqBuffer.average() - currentMaxFreq);

  if (whistleFreqBuffer.minimum() < currentMinFreq)
  {
    minFreqBorder = minFreqBorder + std::abs(whistleFreqBuffer.minimum() - currentMinFreq);
  }
  else
  {
    minFreqBorder = std::max(permanentMinFreqBorder, minFreqBorder - std::abs(whistleFreqBuffer.minimum() - currentMinFreq));
  }

  if (whistleFreqBuffer.maximum() > currentMaxFreq)
  {
    maxFreqBorder = maxFreqBorder + std::abs(whistleFreqBuffer.maximum() - currentMaxFreq);
  }
  else
  {
    maxFreqBorder = std::max(permanentMaxFreqBorder, maxFreqBorder - std::abs(whistleFreqBuffer.maximum() - currentMaxFreq));
  }

  if (detectedFrequency > minFreq && detectedFrequency < maxFreq)
  {
    currentMinFreq = std::max(std::min(whistleFreqBuffer.average() - minDiff / 2, whistleFreqBuffer.average() - minFreqBorder), (int)minFreq - 450);
    whistleMinMaxFreq[0] = std::min(std::max(currentMinFreq, 0), ((int)windowSize / 2 + 1) * (int)samplerate / (int)windowSize);
    currentMaxFreq = std::min(std::max(whistleFreqBuffer.average() + maxDiff / 2, whistleFreqBuffer.average() + maxFreqBorder), (int)maxFreq + 300);
    whistleMinMaxFreq[1] = std::min(std::max(currentMaxFreq, 0), ((int)windowSize / 2 + 1) * (int)samplerate / (int)windowSize);
  }

  return whistleMinMaxFreq;
}

unsigned int AudioBuffer::getWindowSize()
{
  return windowSize;
}

void AudioBuffer::setWindowSize(unsigned int windowSize)
{
  this->windowSize = windowSize;

  resize2D(buffer, channels, windowSize);
  resize2D(helper, channels, windowSize / 2 + 1);
  resize2D(helperAmp, channels, windowSize / 2 + 1);
  resize2D(helperLogSpec, channels, windowSize / 2 + 1);
  resize2D(localFftBuffer, channels, windowSize);
}


unsigned int AudioBuffer::getChannels()
{
  return channels;
}

void AudioBuffer::setChannels(unsigned int channels)
{
  this->channels = channels;

  resize2D(buffer, channels, windowSize);
  resize2D(helper, channels, windowSize / 2 + 1);
  resize2D(helperAmp, channels, windowSize / 2 + 1);
  resize2D(helperLogSpec, channels, windowSize / 2 + 1);
  resize2D(localFftBuffer, channels, windowSize);
  levels.resize(channels);
  spectrograms.resize(channels);
  magnitudes.resize(channels);
  logSpectrograms.resize(channels);
  drrs.resize(channels);
  maxMagHist.resize(channels);
  micsBrokenCount.resize(channels);
  micStatus.resize(channels);
}

std::string AudioBuffer::getModelPath()
{
  return oldModelPathPath;
}

void AudioBuffer::setModelPath(std::string modelPath)
{
  setupNeuralNetwork(modelPath);
}

float AudioBuffer::getNoiseLimit()
{
  return noiseLimit;
}

void AudioBuffer::setNoiseLimit(float noiseLimit)
{
  this->noiseLimit = noiseLimit;
}

float AudioBuffer::getMicBrokenMagSumThreshold()
{
  return micBrokenMagSumThreshold;
}

void AudioBuffer::setMicBrokenMagSumThreshold(float micBrokenMagSumThreshold)
{
  this->micBrokenMagSumThreshold = micBrokenMagSumThreshold;
}

unsigned int AudioBuffer::getMicBrokenThreshold()
{
  return micBrokenThreshold;
}

void AudioBuffer::setMicBrokenThreshold(unsigned int micBrokenThreshold)
{
  this->micBrokenThreshold = micBrokenThreshold;
}

unsigned int AudioBuffer::getRingbufferSize()
{
  return ringbufferSize;
}

void AudioBuffer::setRingbufferSize(unsigned int ringbufferSize)
{
  this->ringbufferSize = ringbufferSize;

  for (unsigned int c = 0; c < channels; c++)
  {
    levels[c].reserve(ringbufferSize);
    spectrograms[c].reserve(ringbufferSize);
    magnitudes[c].reserve(ringbufferSize);
    logSpectrograms[c].reserve(ringbufferSize);
    drrs[c].reserve(ringbufferSize);
    confidences.reserve(ringbufferSize);
  }
}

unsigned int AudioBuffer::getSamplerate()
{
  return samplerate;
}

void AudioBuffer::setSamplerate(unsigned int samplerate)
{
  this->samplerate = samplerate;
}

unsigned int AudioBuffer::getWhistleLenght()
{
  return whistleLength;
}

void AudioBuffer::setWhistleLength(unsigned int whistleLength)
{
  this->whistleLength = whistleLength;
}

float AudioBuffer::getNNWeight()
{
  return nnWeight;
}

void AudioBuffer::setNNWeight(float nnWeight)
{
  this->nnWeight = nnWeight;
}

float AudioBuffer::getPMWeight()
{
  return pmWeight;
}

void AudioBuffer::setPMWeight(float pmWeight)
{
  this->pmWeight = pmWeight;
}

float AudioBuffer::getNoiseWeight()
{
  return noiseWeight;
}

void AudioBuffer::setNoiseWeight(float noiseWeight)
{
  this->noiseWeight = noiseWeight;
}

float AudioBuffer::getWhistleCandidateThreshold()
{
  return whistleCandidateThreshold;
}

void AudioBuffer::setWhistleCandidateThreshold(float whistleCandidateThreshold)
{
  this->whistleCandidateThreshold = whistleCandidateThreshold;
}

float AudioBuffer::getAttentionMultiplier()
{
  return attentionMultiplier;
}

void AudioBuffer::setAttentionMultiplier(float attentionMultiplier)
{
  this->attentionMultiplier = attentionMultiplier;
}

unsigned int AudioBuffer::getMinWhistleFrames()
{
  return minWhistleFrames;
}

void AudioBuffer::setMinWhistleFrames(unsigned int minWhistleFrames)
{
  this->minWhistleFrames = minWhistleFrames;
}

ConfidenceType AudioBuffer::getConfidenceType()
{
  return confidenceType;
}

void AudioBuffer::setConfidenceType(ConfidenceType confidenceType)
{
  this->confidenceType = confidenceType;
}

int AudioBuffer::getMinFreq()
{
  return static_cast<int>(minFreq);
}

void AudioBuffer::setMinFreq(unsigned int minFreq)
{
  this->minFreq = minFreq;
}

int AudioBuffer::getMaxFreq()
{
  return static_cast<int>(maxFreq);
}

void AudioBuffer::setMaxFreq(unsigned int maxFreq)
{
  this->maxFreq = maxFreq;
}
