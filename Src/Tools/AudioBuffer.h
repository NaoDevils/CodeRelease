
#pragma once

#include <kissfft/kiss_fft.h>

#include "Modules/Perception/TFlite.h"

#include "Tools/Enum.h"
#include "Tools/RingBuffer.h"
#include "Tools/RingBufferWithSum.h"
#include <vector>
#include <string>


ENUM(ConfidenceType,
        physical,
        neuralNetwork,
        noise,
        fused
);

struct AudioBufferParameters
{
  //-------------[General]----------------
  unsigned int windowSize = 1024;
  unsigned int samplerate = 22050;
  std::string modelPath = "";
  unsigned int whistleLength = 18;
  unsigned int ringbufferSize = 42;
  unsigned int channels = 4;
  //-----------[Check Mics]---------------
  float noiseLimit = 0.25f;
  float micBrokenMagSumThreshold = 0.2f;
  unsigned int micBrokenThreshold = 10;
  //----------[Whistle Detection]----------
  float nnWeight = 0.8f;
  float pmWeight = 0.6f;
  float noiseWeight = 0.4f;
  float overtoneBorderMultiplier = 1.0f;
  //----------[Whistle Calibration]----------
  unsigned int minFreq = 2500;
  unsigned int maxFreq = 3500;
  unsigned int minFreqBorder = 450;
  unsigned int maxFreqBorder = 350;
  //------[Whistle Candidate Detection]------
  float whistleCandidateThreshold = 0.25f;
  float attentionMultiplier = 2.0f;
  unsigned int minWhistleFrames = 1;
  ConfidenceType confidenceType = ConfidenceType::neuralNetwork;
};

class AudioBuffer
{
private:
  //-------------[General]----------------
  unsigned int windowSize = 1024;
  unsigned int ringbufferSize = 42;
  unsigned int channels = 4;
  unsigned int ringPos = 0;
  unsigned int prevSamplesLeft = 0;
  unsigned int whistleLength = 18;
  unsigned int directSoundLength = 3;
  unsigned int numOfNewFrames = 0;
  std::vector<std::vector<float>> buffer;
  std::vector<std::vector<float>> tempDrr;
  std::vector<std::vector<float>> tempDirectSum;
  std::vector<std::vector<float>> tempReverbSum;
  std::vector<std::vector<float>> outNextLogSpectrogram;
  std::vector<std::vector<float>> outGetConfidences;
  std::vector<std::vector<kiss_fft_cpx>> helper;
  std::vector<std::vector<float>> helperAmp;
  std::vector<std::vector<float>> helperLogSpec;
  std::vector<std::vector<kiss_fft_cpx>> localFftBuffer;
  std::vector<RingBuffer<std::vector<float>, 42>> levels; // 42 frames are around 1 second of audio data
  std::vector<RingBuffer<std::vector<kiss_fft_cpx>, 42>> spectrograms;
  std::vector<RingBuffer<std::vector<float>, 42>> logSpectrograms;
  std::vector<RingBuffer<std::vector<float>, 42>> magnitudes;
  std::vector<RingBuffer<std::vector<float>, 42>> drrs;
  std::vector<std::vector<std::vector<float>>> output;
  void resize2D(auto& buffer, unsigned int size1, unsigned int size2);
  void resize3D(auto& vector, unsigned int size1, unsigned int size2, unsigned int size3);
  void kissFFT(kiss_fft_cpx in[], kiss_fft_cpx out[]);
  unsigned int checkNumOfWindows(unsigned int numOfWindows);
  void setMicStatus(std::vector<std::vector<float>> magnitudes);

  //-----------[Check Mics]---------------

  float noiseLimit = 0.25f;
  float micBrokenMagSumThreshold = 0.2f;
  unsigned int micBrokenThreshold = 10;
  std::vector<unsigned int> micsBrokenCount;
  std::vector<RingBufferWithSum<float, 200>> maxMagHist;
  float currentMeanMag[4] = {0.f, 0.f, 0.f, 0.f};
  float currentMinMag[4] = {0.f, 0.f, 0.f, 0.f};
  float currentMaxMag[4] = {0.f, 0.f, 0.f, 0.f};
  float relNoise[4] = {0.f, 0.f, 0.f, 0.f};
  std::vector<bool> micStatus;

  //----------[Whistle Detection]----------

  std::string oldModelPathPath = "";
  float nnWeight = 0.8f;
  float pmWeight = 0.6f;
  float noiseWeight = 0.4f;
  float overtoneBorderMultiplier = 1.0f;

  unsigned int samplerate = 22050;

  std::vector<std::vector<float>> gradients;

  RingBuffer<std::vector<float>, 42> confidences;

  std::unique_ptr<tflite::Interpreter> interpreter;
  std::unique_ptr<tflite::FlatBufferModel> model;
  tflite::ops::builtin::BuiltinOpResolver resolver;
  int inputTensor, outputTensor;

  void setupNeuralNetwork(std::string modelPath);
  std::vector<float> calculateConfidences();

  //----------[Whistle Calibration]----------
  unsigned int minFreq = 2500;
  unsigned int maxFreq = 3500;
  int currentMinFreq = 2500;
  int currentMaxFreq = 3500;
  int minFreqBorder = 450;
  int maxFreqBorder = 300;
  int permanentMinFreqBorder = 450;
  int permanentMaxFreqBorder = 300;

  RingBufferWithSum<int, 20> whistleFreqBuffer;
  std::array<unsigned int, 2> whistleMinMaxFreq = {2500, 3500};

  //------[Whistle Candidate Detection]------
  float whistleCandidateThreshold = 0.25f;
  float attentionMultiplier = 2.0f;
  unsigned int minWhistleFrames = 1;

  ConfidenceType confidenceType = ConfidenceType::neuralNetwork;


public:
  AudioBuffer();
  AudioBuffer(AudioBufferParameters parameters);
  int getCurrentMic();
  unsigned int getNumOfNewFrames();
  std::vector<std::vector<std::vector<float>>> getLevels(unsigned int minIDX = 0, unsigned int maxIDX = 17);
  std::vector<std::vector<float>> getConfidences(unsigned int numOfWindows = 1);
  std::vector<float> nextConfidences();
  std::vector<std::vector<std::vector<float>>> getLogSpectrograms(unsigned int minIDX = 0, unsigned int maxIDX = 17);
  std::vector<std::vector<std::vector<float>>> getPhases(unsigned int minIDX = 0, unsigned int maxIDX = 17);
  std::vector<std::vector<std::vector<float>>> getDrrs(unsigned int minIDX = 0, unsigned int maxIDX = 17);
  std::array<unsigned int, 2> calibrateFrequency(unsigned int detectedFrequency);
  std::vector<bool> getMicStatus();
  unsigned int getWindowSize();
  void setWindowSize(unsigned int windowSize);
  unsigned int getChannels();
  void setChannels(unsigned int channels);
  std::string getModelPath();
  void setModelPath(std::string modelPath);
  float getNoiseLimit();
  void setNoiseLimit(float noiseLimit);
  float getMicBrokenMagSumThreshold();
  void setMicBrokenMagSumThreshold(float micBrokenMagSumThreshold);
  unsigned int getMicBrokenThreshold();
  void setMicBrokenThreshold(unsigned int micBrokenThreshold);
  unsigned int getRingbufferSize();
  void setRingbufferSize(unsigned int ringbufferSize);
  unsigned int getSamplerate();
  void setSamplerate(unsigned int samplerate);
  unsigned int getWhistleLenght();
  void setWhistleLength(unsigned int whistleLength);
  float getNNWeight();
  void setNNWeight(float nnWeight);
  float getPMWeight();
  void setPMWeight(float pmWeight);
  float getNoiseWeight();
  void setNoiseWeight(float noiseWeight);
  float getWhistleCandidateThreshold();
  void setWhistleCandidateThreshold(float whistleCandidateThreshold);
  float getAttentionMultiplier();
  void setAttentionMultiplier(float attentionMultiplier);
  unsigned int getMinWhistleFrames();
  void setMinWhistleFrames(unsigned int minWhistleFrames);
  ConfidenceType getConfidenceType();
  void setConfidenceType(ConfidenceType confidenceType);
  int getMinFreq();
  void setMinFreq(unsigned int minFreq);
  int getMaxFreq();
  void setMaxFreq(unsigned int maxFreq);
  int getMinFreqBorder();
  void setMinFreqBorder(unsigned int minFreqBorder);
  int getMaxFreqBorder();
  void setMaxFreqBorder(unsigned int maxFreqBorder);
  float getOvertoneBorderMultiplier();
  void setOvertoneBorderMultiplier(float overtoneBorderMultiplier);
  void add(std::vector<float> samples);
  std::vector<std::vector<unsigned int>> getWhistleCandidateIdx();
  std::vector<std::vector<unsigned int>> getNewWhistleCandidateIdx();
};
