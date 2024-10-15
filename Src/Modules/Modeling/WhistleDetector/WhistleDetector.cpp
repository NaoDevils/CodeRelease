#include "WhistleDetector.h"
#include "Tools/Debugging/Annotation.h"

#include <algorithm>
#include <numeric>
#include "Tools/Build.h"

#include "Platform/File.h"

#include <taskflow/taskflow.hpp>
#include <taskflow/algorithm/for_each.hpp>


WhistleDetector::WhistleDetector()
{
  setup();

  micStatus.reserve(4);
  sayMicBroken.reserve(4);
  for (unsigned int i = 0; i < 4; i++)
  {
    micStatus.push_back(true);
    sayMicBroken.push_back(false);
  }

  for (int i = 0; i < 5; i++)
    localWhistle.distanceConfidence[i] = 0.f;

  localWhistle.minFrequency = minFreq;
  localWhistle.maxFrequency = maxFreq;
}

void WhistleDetector::execute(tf::Subflow& subflow)
{
  DEBUG_RESPONSE("module:WhistleDetector:detectWhistle")
  {
    localWhistle.detectionState = WhistleDortmund::DetectionState::isDetected;
    localWhistle.lastDetectionTime = theFrameInfo.time;
    return;
  }

  if (!initialize || channels != theAudioData.channels)
  {
    if (theAudioData.isValid)
      init();
    else
    {
      localWhistle.detectionState = WhistleDortmund::DetectionState::notDetected;
      return;
    }
  }

  INIT_DEBUG_IMAGE_BLACK(SPECTROGRAM, audioBufferSize, (windowSize / 2) + 1);
  DECLARE_PLOT("representation:Whistle:pmConfidence");
  DECLARE_PLOT("representation:Whistle:nnConfidence");
  DECLARE_PLOT("representation:Whistle:noise");
  DECLARE_PLOT("representation:Whistle:confidence");
  DECLARE_PLOT("representation:Whistle:threshold");


  //update local constants
  const int channels = theAudioData.channels;

  // test all mics
  checkMics(); // TODO: check position of this

  // ask for help if needed
  if constexpr (Build::targetRobot())
  {
    if (theRobotInfo.transitionToFramework == 0.f)
      speechToggle = true;

    if (speechToggle && (theRobotInfo.transitionToFramework > 0.f) && localWhistle.whistleDetectionStatus != WhistleDortmund::WhistleDetectionStatus::alright)
    {
      int numOfBrokenMics = std::accumulate(sayMicBroken.begin(), sayMicBroken.end(), 0);

      if (numOfBrokenMics == channels)
        SystemCall::text2Speech("All Microphones broken, can't hear!");
      else
      {
        SystemCall::text2Speech("Microphone");
        for (int c = 0; c < channels; c++)
        {
          if (sayMicBroken[c])
          {
            SystemCall::text2Speech(std::to_string(c));
            sayMicBroken[c] = false;
          }
        }
        SystemCall::text2Speech("broken");
        if (numOfBrokenMics > 2)
          SystemCall::text2Speech("Localizing Sound no longer possible!");
      }

      speechToggle = false;
    }
  }
  if (theAudioData.isValid)
    audioBuffer.add(theAudioData.samples);


  STOPWATCH("WhistleDetection")
  {
    detectWhistle(localWhistle);
  }
  if (std::accumulate(detectedWhistleCandidates.begin(), detectedWhistleCandidates.end(), 0) > 0)
  {
    for (int idxCandidates = 0; idxCandidates < static_cast<int>(detectedWhistleCandidates.size()); idxCandidates++)
    {
      if (detectedWhistleCandidates[idxCandidates] && isWhistleLongEnough(idxCandidates)
          && (localWhistle.whistleDetectionStatus == WhistleDortmund::WhistleDetectionStatus::alright || localWhistle.whistleDetectionStatus == WhistleDortmund::WhistleDetectionStatus::tryLocalization))
      {
        STOPWATCH("WhistleDirection")
        {
          detectDirection(localWhistle, whistleCandiates[idxCandidates][0], whistleCandiates[idxCandidates][1]);
        }
        STOPWATCH("WhistleDistance")
        {
          detectDistance(localWhistle, whistleCandiates[idxCandidates][0], whistleCandiates[idxCandidates][1]);
        }
      }
    }
  }
  else
  {
    localWhistle.directionState = WhistleDortmund::DetectionState::dontKnow;
    localWhistle.distanceState = WhistleDortmund::DetectionState::dontKnow;
  }

  COMPLEX_IMAGE(SPECTROGRAM)
  {
    int mic = audioBuffer.getCurrentMic();

    if (mic >= 0)
    {
      std::vector<std::vector<std::vector<float>>> logSpectrograms = audioBuffer.getLogSpectrograms(0, audioBufferSize - 1);
      std::vector<bool> markedWhisteIdx = markWhistleIdx();

      float min = *std::min_element(logSpectrograms[mic][0].begin(), logSpectrograms[mic][0].end());
      float max = *std::max_element(logSpectrograms[mic][0].begin(), logSpectrograms[mic][0].end());

      int minWhistle = localWhistle.minFrequency * windowSize / theAudioData.sampleRate;
      int maxWhistle = localWhistle.maxFrequency * windowSize / theAudioData.sampleRate;

      if (min < spectrogramMin)
        spectrogramMin = min;

      if (max > spectrogramMax)
        spectrogramMax = max;

      for (int time = 0; time < static_cast<int>(logSpectrograms[mic].size()); time++)
      {
        for (int i = 0; i < static_cast<int>(logSpectrograms[mic][time].size()); i++)
        {
          int value = static_cast<int>(logSpectrograms[mic][time][i]);
          float valueClippedAndScaled = std::max((((value - spectrogramMin) / (spectrogramMax - spectrogramMin + 0.000001f)) * 255.f) - 64.f, 0.f);

          // Visualize the timespan whether a whistle is detected or not
          if (markedWhisteIdx[time])
            DEBUG_IMAGE_SET_PIXEL_YUV(
                SPECTROGRAM, static_cast<int>((logSpectrograms[mic].size() - 1) - time), static_cast<int>((logSpectrograms[mic][time].size() - 1) - i), static_cast<unsigned char>(valueClippedAndScaled), 85, 74); // Green
          else
            DEBUG_IMAGE_SET_PIXEL_YUV(SPECTROGRAM,
                static_cast<int>((logSpectrograms[mic].size() - 1) - time),
                static_cast<int>((logSpectrograms[mic][time].size() - 1) - i),
                static_cast<unsigned char>(valueClippedAndScaled),
                127,
                127); // Grey

          // Visualize the ROI of the physical model
          int midWhiste = maxWhistle - ((maxWhistle - minWhistle) / 2);
          if ((i > minWhistle && i < maxWhistle)
              || (i > (midWhiste * 2) - ((maxWhistle - minWhistle) / 2) * overtoneBorderMultiplier && i < (midWhiste * 2) + ((maxWhistle - minWhistle) / 2) * overtoneBorderMultiplier))
            DEBUG_IMAGE_SET_PIXEL_YUV(SPECTROGRAM,
                static_cast<int>((logSpectrograms[mic].size() - 1) - time),
                static_cast<int>((logSpectrograms[mic][time].size() - 1) - i),
                static_cast<unsigned char>(valueClippedAndScaled),
                192,
                117); // Blue
        }
      }
    }
  }
  SEND_DEBUG_IMAGE(SPECTROGRAM);

  DEBUG_RESPONSE("representation:Whistle:graphs")
  {
    std::vector<std::vector<float>> confidences = audioBuffer.getConfidences(1);
    int mic = audioBuffer.getCurrentMic();

    if (mic >= 0)
    {
      PLOT("representation:Whistle:pmConfidence", confidences[mic][0]);
      PLOT("representation:Whistle:nnConfidence", confidences[mic][1]);
      PLOT("representation:Whistle:noise", confidences[mic][2]);
      PLOT("representation:Whistle:confidence", confidences[mic][3]);
      PLOT("representation:Whistle:threshold", threshold + ((1 - threshold) * (noiseWeight * confidences[mic][2])));
    }
  }
}

void WhistleDetector::update(WhistleDortmund& whistle)
{
  whistle = localWhistle;
  //-------------[General]----------------
  if (windowSize != audioBuffer.getWindowSize())
    audioBuffer.setWindowSize(windowSize);
  if (theAudioData.sampleRate != audioBuffer.getSamplerate())
    audioBuffer.setSamplerate(theAudioData.sampleRate);
  if (whistleNetPath != audioBuffer.getModelPath())
    audioBuffer.setModelPath(whistleNetPath);
  if (whistleLength != audioBuffer.getWhistleLenght())
    audioBuffer.setWhistleLength(whistleLength);
  if (audioBufferSize != audioBuffer.getRingbufferSize())
    audioBuffer.setRingbufferSize(audioBufferSize);
  if (theAudioData.channels != audioBuffer.getChannels())
    audioBuffer.setChannels(theAudioData.channels);
  //-----------[Check Mics]---------------
  if (noiseLimit != audioBuffer.getNoiseLimit())
    audioBuffer.setNoiseLimit(noiseLimit);
  if (micBrokenMagSumThreshold != audioBuffer.getMicBrokenMagSumThreshold())
    audioBuffer.setMicBrokenMagSumThreshold(micBrokenMagSumThreshold);
  if (micBrokenThreshold != audioBuffer.getMicBrokenThreshold())
    audioBuffer.setMicBrokenThreshold(micBrokenThreshold);
  //----------[Whistle Detection]----------
  if (nnWeight != audioBuffer.getNNWeight())
    audioBuffer.setNNWeight(nnWeight);
  if (pmWeight != audioBuffer.getPMWeight())
    audioBuffer.setPMWeight(pmWeight);
  if (noiseWeight != audioBuffer.getNoiseWeight())
    audioBuffer.setNoiseWeight(noiseWeight);
  if (overtoneBorderMultiplier != audioBuffer.getOvertoneBorderMultiplier())
    audioBuffer.setOvertoneBorderMultiplier(overtoneBorderMultiplier);
  //----------[Whistle Calibration]----------
  if (minFreq != audioBuffer.getMinFreq())
    audioBuffer.setMinFreq(minFreq);
  if (maxFreq != audioBuffer.getMaxFreq())
    audioBuffer.setMaxFreq(maxFreq);
  if (minFreqBorder != audioBuffer.getMinFreqBorder())
    audioBuffer.setMinFreqBorder(minFreqBorder);
  if (maxFreqBorder != audioBuffer.getMaxFreqBorder())
    audioBuffer.setMaxFreqBorder(maxFreqBorder);
  //------[Whistle Candidate Detection]------
  if (whistleCandidateThreshold != audioBuffer.getWhistleCandidateThreshold())
    audioBuffer.setWhistleCandidateThreshold(whistleCandidateThreshold);
  if (attentionMultiplier != audioBuffer.getAttentionMultiplier())
    audioBuffer.setAttentionMultiplier(attentionMultiplier);
  if (minWhistleFrames != audioBuffer.getMinWhistleFrames())
    audioBuffer.setMinWhistleFrames(minWhistleFrames);
  if ((ConfidenceType)confidenceType != audioBuffer.getConfidenceType())
    audioBuffer.setConfidenceType((ConfidenceType)confidenceType);
}

void WhistleDetector::detectWhistle(WhistleDortmund& whistle)
{
  whistleCandiates = audioBuffer.getNewWhistleCandidateIdx();
  unsigned int numOfCandidates = static_cast<unsigned int>(whistleCandiates.size());
  std::vector<unsigned int> attackCount;
  std::vector<unsigned int> whistleFrequencies;
  detectedWhistleCandidates.resize(numOfCandidates);

  if (numOfCandidates == 0)
  {
    whistle.detectionState = WhistleDortmund::DetectionState::notDetected;
    return;
  }
  else if (numOfCandidates > 0)
  {
    whistleSequenceLength.reserve(numOfCandidates);
    for (unsigned int idx = 0; idx < numOfCandidates; idx++)
    {
      whistleSequenceLength[idx] = whistleCandiates[idx][1] - whistleCandiates[idx][0];
      if (whistleSequenceLength[idx] > whistleSequenceLengthThreshold)
        detectedWhistleCandidates[idx] = true;
      else
        detectedWhistleCandidates[idx] = false;
    }
  }

  if (std::accumulate(detectedWhistleCandidates.begin(), detectedWhistleCandidates.end(), 0) > 0)
  {
    attackCount.resize(numOfCandidates);
    whistleFrequencies.resize(numOfCandidates);
    std::vector<std::vector<float>> allNewConfidences = audioBuffer.getConfidences(whistleCandiates[numOfCandidates - 1][1] + 1);
    for (int idxCandidates = (numOfCandidates - 1); idxCandidates >= 0; idxCandidates--) //analyse whistle candidates
    {
      if (detectedWhistleCandidates[idxCandidates])
      {
        for (unsigned int idxSequence = whistleCandiates[idxCandidates][1]; idxSequence > whistleCandiates[idxCandidates][0]; idxSequence--) //analyse the sequence
        {
          if (allNewConfidences[idxSequence][3] > threshold + ((1 - threshold) * (noiseWeight * allNewConfidences[idxSequence][2])))
          {
            attackCount[idxCandidates]++;
            whistleFrequencies[idxCandidates] = (unsigned int)allNewConfidences[idxSequence][4];
          }
        }

        std::nth_element(whistleFrequencies.begin(), whistleFrequencies.begin() + whistleFrequencies.size() / 2, whistleFrequencies.end());
        unsigned int whistleFrequency = whistleFrequencies[whistleFrequencies.size() / 2];

        if (attackCount[idxCandidates] > whistleSequenceLength[idxCandidates] * attackFactor)
        {
          ANNOTATION("Whistle", "Whistle was detected.");
          if (freqCalibration)
          {
            std::array<unsigned int, 2> whistleMinMaxFreq = audioBuffer.calibrateFrequency(whistleFrequency);
            whistle.minFrequency = whistleMinMaxFreq[0];
            whistle.maxFrequency = whistleMinMaxFreq[1];
          }
          else
          {
            whistle.minFrequency = minFreq;
            whistle.maxFrequency = maxFreq;
          }
          whistle.detectedWhistleFrequency = whistleFrequency;
          whistle.detectionState = WhistleDortmund::DetectionState::isDetected;
          whistle.lastWhistleLength = whistleSequenceLength[idxCandidates];
          whistle.lastDetectionTime = theFrameInfo.time;
        }
        else
          detectedWhistleCandidates[idxCandidates] = false;
      }
    }
  }
  else
  {
    whistle.detectionState = WhistleDortmund::DetectionState::notDetected;
    return;
  }
}


void WhistleDetector::detectDirection(WhistleDortmund& whistle, unsigned int minIDX, unsigned int maxIDX)
{
  const unsigned int channels = theAudioData.channels;
  const unsigned int magSize = windowSize / 2 + 1;

  std::vector<std::vector<std::vector<float>>> phases = audioBuffer.getPhases((maxIDX + directionIdxOffset) - numOfDirectionDetectionFrames, maxIDX + directionIdxOffset);
  std::vector<std::vector<std::vector<float>>> levels = audioBuffer.getLevels((maxIDX + directionIdxOffset) - numOfDirectionDetectionFrames, maxIDX + directionIdxOffset);

  std::vector<std::vector<float>> phasesReshaped;
  phasesReshaped.resize(numOfDirectionDetectionFrames);

  for (unsigned int idxTime = 0; idxTime < numOfDirectionDetectionFrames; idxTime++)
    for (unsigned int idxWindow = 0; idxWindow < magSize; idxWindow++)
      for (unsigned int c = 0; c < channels; c++)
        phasesReshaped[idxTime].push_back(phases[c][idxTime][idxWindow]);


  std::vector<std::vector<float>> levelsReshaped;
  levelsReshaped.resize(numOfDirectionDetectionFrames);

  for (unsigned int idxTime = 0; idxTime < numOfDirectionDetectionFrames; idxTime++)
    for (unsigned int idxWindow = 0; idxWindow < windowSize; idxWindow++)
      for (unsigned int c = 0; c < channels; c++)
        levelsReshaped[idxTime].push_back(levels[c][idxTime][idxWindow]);

  std::vector<float> directionBuffer;
  directionBuffer.resize(numOfDirectionDetectionFrames);


  for (unsigned int idxTime = 0; idxTime < numOfDirectionDetectionFrames; idxTime++)
  {
    for (unsigned int i = 0; i < phasesReshaped[idxTime].size(); i++)
      directionInterpreter->typed_tensor<float>(inputTensorSlice)[i] = phasesReshaped[idxTime][i];
    for (unsigned int i = 0; i < levelsReshaped[idxTime].size(); i++)
      directionInterpreter->typed_tensor<float>(inputTensorLevel)[i] = levelsReshaped[idxTime][i];

    if (directionInterpreter->Invoke() != kTfLiteOk)
      OUTPUT_ERROR("Failed to invoke whistle direction tflite!");
    float* directionOutput = directionInterpreter->typed_tensor<float>(directionOutputTensor);

    float direction = *directionOutput;

    directionBuffer[idxTime] = direction;
  }
  directionHeadAngleSnapshot = theJointSensorData.angles[Joints::headYaw]; //ToDo track at correct time

  std::nth_element(directionBuffer.begin(), directionBuffer.begin() + directionBuffer.size() / 2, directionBuffer.end());
  whistle.directionState = WhistleDortmund::DetectionState::isKnown;
  whistle.rawDirection = Angle::fromDegrees(directionBuffer[directionBuffer.size() / 2]);
  whistle.correctedDirection = Angle::fromDegrees(
      static_cast<float>(static_cast<unsigned int>(std::ceil(directionBuffer[numOfDirectionDetectionFrames / 2] + directionHeadAngleSnapshot.toDegrees())) % 360));
}


void WhistleDetector::detectDistance(WhistleDortmund& whistle, unsigned int minIDX, unsigned int maxIDX)
{
  const unsigned int channels = theAudioData.channels;
  const unsigned int magSize = windowSize / 2 + 1;

  minIDX = minIDX + distanceIdxOffset;
  maxIDX = maxIDX + distanceIdxOffset;

  maxIDX = (maxIDX - minIDX == whistleLength) ? maxIDX : minIDX + whistleLength;

  if (maxIDX >= audioBufferSize)
  {
    minIDX = (audioBufferSize - 1) - whistleLength;
    maxIDX = (audioBufferSize - 1);
  }

  std::vector<std::vector<std::vector<float>>> drr = audioBuffer.getDrrs(minIDX, maxIDX);
  std::vector<std::vector<std::vector<float>>> logSpec = audioBuffer.getLogSpectrograms(minIDX, maxIDX);


  std::vector<float> drrInput;
  drrInput.reserve(whistleLength * magSize * channels);

  for (unsigned int idxTime = 0; idxTime < whistleLength; idxTime++)
    for (unsigned int idxWindow = 0; idxWindow < magSize; idxWindow++)
      for (unsigned int c = 0; c < channels; c++)
        drrInput.push_back(drr[c][idxTime][idxWindow]);

  std::vector<float> specInput;
  specInput.reserve(whistleLength * magSize * channels);

  for (unsigned int idxTime = 0; idxTime < whistleLength; idxTime++)
    for (unsigned int idxWindow = 0; idxWindow < magSize; idxWindow++)
      for (unsigned int c = 0; c < channels; c++)
        specInput.push_back(logSpec[c][idxTime][idxWindow]);


  for (int i = 0; i < static_cast<int>(drrInput.size()); i++)
    distanceInterpreter->typed_tensor<float>(inputTensorDrr)[i] = drrInput[i];
  for (int i = 0; i < static_cast<int>(specInput.size()); i++)
    distanceInterpreter->typed_tensor<float>(inputTensorDistance)[i] = specInput[i];

  if (distanceInterpreter->Invoke() != kTfLiteOk)
    OUTPUT_ERROR("Failed to invoke whistle distance tflite!");

  float maxConf = 0.f;
  int maxConfIdx = 0;
  float distance = 0.f;
  for (unsigned int i = 0; i < 5; i++)
  {
    float currentConf = distanceInterpreter->typed_tensor<float>(distanceOutputTensor)[i];
    if (maxConf <= currentConf)
    {
      maxConf = distanceInterpreter->typed_tensor<float>(distanceOutputTensor)[i];
      distance = i / 4.f;
      maxConfIdx = i;
    }
    whistle.distanceConfidence[i] = currentConf;
  }
  whistle.distanceState = WhistleDortmund::DetectionState::isKnown;
  whistle.distance = distance;
  whistle.readableDistance = readableDistance[maxConfIdx];
}

void WhistleDetector::resizeBuffer(auto& buffer, unsigned int size1, unsigned int size2)
{
  buffer.resize(size1);
  if (size2 > 0)
    for (auto& buf : buffer)
      buf.resize(size2);
}

void WhistleDetector::checkMics()
{
  micStatus = audioBuffer.getMicStatus();
  int functionalMicSum = 0;
  for (unsigned int c = 0; c < theAudioData.channels; c++)
  {
    localWhistle.micStatus[c] = micStatus[c];
    if (micStatus[c])
      functionalMicSum++;
    else
      sayMicBroken[c] = true;
  }
  switch (functionalMicSum)
  {
  case 4:
    localWhistle.whistleDetectionStatus = WhistleDortmund::WhistleDetectionStatus::alright;
    break;
  case 3:
    localWhistle.whistleDetectionStatus = WhistleDortmund::WhistleDetectionStatus::tryLocalization;
    break;
  case 2:
  case 1:
    localWhistle.whistleDetectionStatus = WhistleDortmund::WhistleDetectionStatus::onlyWhlistleDetection;
    break;
  case 0:
    localWhistle.whistleDetectionStatus = WhistleDortmund::WhistleDetectionStatus::broken;
    break;
  default:
    break;
  }

  return;
}

void WhistleDetector::setup()
{
  // get whistle direction net path / filename
  std::string filename = std::string(File::getBHDir()) + whistleDirectionNetPath;
  oldWhistleDirectionNetPath = whistleDirectionNetPath;

  // Load the model
  directionModel = tflite::FlatBufferModel::BuildFromFile(filename.c_str());

  if (directionModel == nullptr)
  {
    OUTPUT_WARNING("WhistleDirectionNet model not found");
  }
  else
  {
    // Build the interpreter
    tflite::InterpreterBuilder builder(*directionModel, directionResolver);
    builder(&directionInterpreter);

    directionInterpreter->SetNumThreads(1);

    // Resize input tensors
    inputTensorSlice = directionInterpreter->inputs()[0];
    inputTensorLevel = directionInterpreter->inputs()[1];
    directionOutputTensor = directionInterpreter->outputs()[0];
    TfLiteIntArray* input_dims = directionInterpreter->tensor(inputTensorSlice)->dims;
    int inputSize = input_dims->data[1];
    int channels = input_dims->data[2];

    std::vector<int> newInputSlice;
    newInputSlice.push_back(1); // Batch size
    newInputSlice.push_back(inputSize);
    newInputSlice.push_back(channels);
    directionInterpreter->ResizeInputTensor(inputTensorSlice, newInputSlice);

    input_dims = directionInterpreter->tensor(inputTensorLevel)->dims;
    inputSize = input_dims->data[1];
    channels = input_dims->data[2];

    std::vector<int> newInputLevel;
    newInputLevel.push_back(1); // Batch size
    newInputLevel.push_back(inputSize);
    newInputLevel.push_back(channels);
    directionInterpreter->ResizeInputTensor(inputTensorLevel, newInputLevel);

    // Allocate memory for the tensors
    directionInterpreter->AllocateTensors();
  }

  // get whistle distance net path / filename
  filename = std::string(File::getBHDir()) + whistleDistanceNetPath;
  oldWhistleDistanceNetPath = whistleDistanceNetPath;

  // Load the model
  distanceModel = tflite::FlatBufferModel::BuildFromFile(filename.c_str());

  if (distanceModel == nullptr)
  {
    OUTPUT_WARNING("WhistleDistanceNet model not found");
  }
  else
  {
    // Build the interpreter
    tflite::InterpreterBuilder builder(*distanceModel, distanceResolver);
    builder(&distanceInterpreter);

    // Resize input tensors
    inputTensorDrr = distanceInterpreter->inputs()[1];
    inputTensorDistance = distanceInterpreter->inputs()[0];
    distanceOutputTensor = distanceInterpreter->outputs()[0];
    TfLiteIntArray* input_dims = distanceInterpreter->tensor(inputTensorDistance)->dims;
    int inputRows = input_dims->data[1];
    int inputCols = input_dims->data[2];
    int channels = input_dims->data[3];

    std::vector<int> newInput;
    newInput.push_back(1); // Batch size
    newInput.push_back(inputRows);
    newInput.push_back(inputCols);
    newInput.push_back(channels);
    distanceInterpreter->ResizeInputTensor(inputTensorDistance, newInput);

    input_dims = distanceInterpreter->tensor(inputTensorDrr)->dims;
    int inputSize = input_dims->data[1];
    channels = input_dims->data[2];

    std::vector<int> newInputDrr;
    newInputDrr.push_back(1); // Batch size
    newInputDrr.push_back(inputSize);
    newInputDrr.push_back(channels);
    distanceInterpreter->ResizeInputTensor(inputTensorDrr, newInputDrr);

    distanceInterpreter->SetNumThreads(1);

    // Allocate memory for the tensors
    distanceInterpreter->AllocateTensors();
  }
}

void WhistleDetector::init()
{
  AudioBufferParameters audioBufferParameters;
  //-------------[General]----------------
  audioBufferParameters.windowSize = windowSize;
  audioBufferParameters.samplerate = theAudioData.sampleRate;
  audioBufferParameters.modelPath = whistleNetPath;
  audioBufferParameters.whistleLength = whistleLength;
  audioBufferParameters.ringbufferSize = audioBufferSize;
  audioBufferParameters.channels = theAudioData.channels;
  //-----------[Check Mics]---------------
  audioBufferParameters.noiseLimit = noiseLimit;
  audioBufferParameters.micBrokenMagSumThreshold = micBrokenMagSumThreshold;
  audioBufferParameters.micBrokenThreshold = micBrokenThreshold;
  //----------[Whistle Detection]----------
  audioBufferParameters.nnWeight = nnWeight;
  audioBufferParameters.pmWeight = pmWeight;
  audioBufferParameters.noiseWeight = noiseWeight;
  audioBufferParameters.overtoneBorderMultiplier = overtoneBorderMultiplier;
  //----------[Whistle Calibration]----------
  audioBufferParameters.minFreq = minFreq;
  audioBufferParameters.maxFreq = maxFreq;
  audioBufferParameters.minFreqBorder = minFreqBorder;
  audioBufferParameters.maxFreqBorder = maxFreqBorder;
  //------[Whistle Candidate Detection]------
  audioBufferParameters.whistleCandidateThreshold = whistleCandidateThreshold;
  audioBufferParameters.attentionMultiplier = attentionMultiplier;
  audioBufferParameters.minWhistleFrames = minWhistleFrames;
  audioBufferParameters.confidenceType = (ConfidenceType)confidenceType;

  audioBuffer = AudioBuffer(audioBufferParameters);
  channels = theAudioData.channels;

  initialize = true;
}

bool WhistleDetector::isWhistleLongEnough(int idxCandidate)
{
  if (whistleSequenceLength[idxCandidate] >= whistleLength)
  {
    return true;
  }
  else if (idxCandidate == 0)
  {
    return (whistleCandiates[idxCandidate][1] >= whistleLength - 1);
  }
  else
  {
    return (whistleCandiates[idxCandidate][1] - whistleCandiates[idxCandidate - 1][1] > whistleLength);
  }
}

std::vector<bool> WhistleDetector::markWhistleIdx()
{
  std::vector<bool> markedWhistleIdx;
  markedWhistleIdx.resize(audioBufferSize);

  for (int i = 0; i < static_cast<int>(audioBufferSize); i++)
    markedWhistleIdx[i] = false;

  for (int i = 0; i < static_cast<int>(detectedWhistleCandidates.size()); i++)
  {
    if (detectedWhistleCandidates[i])
    {
      for (unsigned int j = whistleCandiates[i][0]; j <= whistleCandiates[i][1]; j++)
        markedWhistleIdx[j] = true;
    }
  }
  return markedWhistleIdx;
}

MAKE_MODULE(WhistleDetector, modeling)
