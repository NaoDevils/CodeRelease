#pragma once

#include <kissfft/kiss_fft.h>

#include "Tools/Module/Module.h"
#include "Representations/Modeling/WhistleDortmund.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include <cstdio>
//#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugImages.h"
#include "Platform/SystemCall.h"
#include <string>
#include <float.h>
#include "Tools/Math/Constants.h"
#include "Tools/RingBufferWithSum.h"
#include "Modules/Perception/TFlite.h"

#include "Tools/AudioBuffer.h"

MODULE(WhistleDetector,
  REQUIRES(FrameInfo),
  REQUIRES(AudioData),
  REQUIRES(JointSensorData),
  REQUIRES(RobotInfo),
  HAS_PREEXECUTION,
  PROVIDES(WhistleDortmund),
  LOADS_PARAMETERS(,
    //-------------[General]----------------
    (unsigned int) windowSize,
    (unsigned int) audioBufferSize,
    (std::string) whistleNetPath,
    (std::string) whistleDistanceNetPath,
    (std::string) whistleDirectionNetPath,
    //-----------[Check Mics]---------------
    (float) noiseLimit,
    (unsigned int) micBrokenThreshold,
    (float) micBrokenMagSumThreshold,
    //----------[Whistle Candidate Detection]----------
    (float) nnWeight,
    (float) pmWeight,
    (float) noiseWeight,
    (unsigned int) confidenceType,
    (unsigned int) minWhistleFrames,
    (float) attentionMultiplier,
    (float) overtoneBorderMultiplier,
    (float) whistleCandidateThreshold,
    //----------[Whistle Calibration]----------
    (int) minFreq, 
    (int) maxFreq,
    (int) minFreqBorder,
    (int) maxFreqBorder,
    (bool) freqCalibration,
    //----------[Whistle Detection]----------
    (float) threshold,
    (float) attackFactor,
    (unsigned int) whistleSequenceLengthThreshold,
    //----------[Whistle Direction Detection]----------
    (int) directionIdxOffset,
    (unsigned int) numOfDirectionDetectionFrames,
    //----------[Whistle Distance Detection]----------
    (unsigned int) whistleLength,
    (int) distanceIdxOffset,
    (std::vector<std::string>) readableDistance
  )
);

class WhistleDetector : public WhistleDetectorBase
{

public:
  WhistleDetector();
  void execute(tf::Subflow&);
  void update(WhistleDortmund& whistle);
  void detectWhistle(WhistleDortmund& whistle);
  void detectDirection(WhistleDortmund& whistle, unsigned int minIDX, unsigned int maxIDX);
  void detectDistance(WhistleDortmund& whistle, unsigned int minIDX, unsigned int maxIDX);
  void resizeBuffer(auto& buffer, unsigned int size1, unsigned int size2 = 0);
  void checkMics();

private:
  DECLARE_DEBUG_IMAGE(SPECTROGRAM);
  float spectrogramMin = FLT_MAX;
  float spectrogramMax = -FLT_MAX;
  std::vector<bool> markWhistleIdx();

  void setup();
  void init();
  bool initialize = false;
  bool speechToggle = true;

  AudioBuffer audioBuffer;
  unsigned int channels = 4;

  WhistleDortmund localWhistle;

  std::vector<bool> micStatus;
  int micIdx[4] = {0, 1, 2, 3};
  std::vector<bool> sayMicBroken;


  std::vector<bool> detectedWhistleCandidates;
  std::vector<std::vector<unsigned int>> whistleCandiates;
  std::vector<unsigned int> whistleSequenceLength;
  bool isWhistleLongEnough(int idxCandidate);

  std::string oldWhistleDistanceNetPath;
  std::string oldWhistleDirectionNetPath;
  unsigned int oldDirectionIdx;
  unsigned int oldDistanceIdx;

  unsigned lastAttackTime = 0;
  unsigned int releaseCount, attackCount;

  std::unique_ptr<tflite::Interpreter> directionInterpreter;
  std::unique_ptr<tflite::Interpreter> distanceInterpreter;

  std::unique_ptr<tflite::FlatBufferModel> directionModel;
  std::unique_ptr<tflite::FlatBufferModel> distanceModel;
  tflite::ops::builtin::BuiltinOpResolver directionResolver;
  tflite::ops::builtin::BuiltinOpResolver distanceResolver;
  int inputTensorSlice, inputTensorLevel, inputTensorDrr, inputTensorDistance, directionOutputTensor, distanceOutputTensor;

  Angle directionHeadAngleSnapshot;
  std::vector<float> distanceBuffer;
};
