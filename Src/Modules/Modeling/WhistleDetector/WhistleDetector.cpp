#include "WhistleDetector.h"
#include "Tools/Debugging/Annotation.h"

#include "Platform/File.h"

//  arecord -t raw -f U8 > audio
// 8 kHz, uint8_t

#define DEBUG_WIDTH 512
#define DEBUG_HEIGHT 300
#define GRID_DISTANCE_X 1000
#define GRID_DISTANCE_Y 25

#define CHROMA_WIDTH 500
#define CHROMA_MAX 50

WhistleDetector::WhistleDetector()
{
  setup();

  ringPos = 0;

  releaseCount = static_cast<unsigned int>(release);
  attackCount = 1;

  thresholdBuffer.reserve(adaptiveWindowSize);
  confidenceBuffer.reserve((size_t)std::ceil(adaptiveWindowSize / 2));
  prevAdaptiveWindowSize = adaptiveWindowSize;

  currentMinFreq = minFreq;
  currentMaxFreq = maxFreq;
  oldMinFreq = minFreq;
  oldMaxFreq = maxFreq;
  detectedWhistleFrequency = minFreq + ((maxFreq - minFreq) / 2);
}

void WhistleDetector::update(WhistleDortmund& whistle)
{
  INIT_DEBUG_IMAGE_BLACK(FFT, DEBUG_WIDTH, DEBUG_HEIGHT);
  DECLARE_PLOT("representation:Whistle:confidence");
  DECLARE_PLOT("representation:Whistle:nnConfidence");
  DECLARE_PLOT("representation:Whistle:pmConfidence");
  DECLARE_PLOT("representation:Whistle:relLimitCount");
  DECLARE_PLOT("representation:Whistle:nnConfidenceWeighted");
  DECLARE_PLOT("representation:Whistle:pmConfidenceWeighted");
  DECLARE_PLOT("representation:Whistle:relLimitCountWeighted");
  DECLARE_PLOT("representation:Whistle:detected");
  DECLARE_PLOT("representation:Whistle:threshold");
  DECLARE_PLOT("representation:Whistle:meanAmp");
  DECLARE_PLOT("representation:Whistle:minAmp");
  DECLARE_PLOT("representation:Whistle:maxAmp");

  DEBUG_RESPONSE("module:WhistleDetector:detectWhistle")
  {
    whistle.detectionState = WhistleDortmund::DetectionState::isDetected;
    whistle.lastDetectionTime = theFrameInfo.time;
    return;
  }

  //update local constants
  const int sampleRate = theAudioData.sampleRate;
  const int channels = theAudioData.channels;

  if (adaptiveWindowSize != prevAdaptiveWindowSize)
  {
    thresholdBuffer.reserve(adaptiveWindowSize);
    thresholdBuffer.clear();
    confidenceBuffer.reserve((size_t)std::ceil(adaptiveWindowSize / 2));
    confidenceBuffer.clear();
    prevAdaptiveWindowSize = adaptiveWindowSize;
  }

  if (oldMinFreq != minFreq)
  {
    oldMinFreq = minFreq;
    currentMinFreq = minFreq;
    whistleFreqBuffer.clear();
  }

  if (oldMaxFreq != maxFreq)
  {
    oldMaxFreq = maxFreq;
    currentMaxFreq = maxFreq;
    whistleFreqBuffer.clear();
  }

  if (oldWhistleNetPath != whistleNetPath)
    setup();

  if (!useAdaptiveThreshold)
  {
    thresholdBuffer.reserve(1);
    confidenceBuffer.reserve(1);
  }

  //declare physical model detection variables
  int peakPos = -1, overtonePeakPos = -1;
  float currentMinAmp = 0, currentMaxAmp = 0, currentMeanAmp = 0;

  if (theAudioData.isValid)
  {
    unsigned int audioDataPos = currentMic;

    while (audioDataPos < theAudioData.samples.size())
    {
      while (audioDataPos < theAudioData.samples.size() && samplesLeft > 0)
      {
        //S16 to float
        //buffer[ringPos] = static_cast<float>(theAudioData.samples[audioDataPos])
        //    / static_cast<float>(std::numeric_limits<short>::max());

        buffer[ringPos] = theAudioData.samples[audioDataPos];

        audioDataPos += channels;
        samplesLeft--;
        ringPos = (ringPos + 1) % windowSize;
      }

      if (samplesLeft == 0)
      {
        whistle.detectionState = WhistleDortmund::DetectionState::notDetected;
        samplesLeft = windowSize / 2;

        for (unsigned int i = 0; i < static_cast<unsigned int>(windowSize); i++)
        {
          in[i].r = buffer[(i + ringPos) % windowSize];
          in[i].i = 0.f;

          //apply Hann window
          if (useHannWindowing)
            in[i].r *= std::pow(std::sin(pi * i / windowSize), 2.f);
          else if (useNuttallWindowing)
            in[i].r *= 0.355768f - 0.487396f * std::sin(1 * pi * i / windowSize) + 0.144232f * std::sin(2 * pi * i / windowSize) - 0.012604f * std::sin(3 * pi * i / windowSize);
          else //apply Hamming window -> default
            in[i].r *= (0.54f - 0.46f * std::cos(2.f * pi * i / windowSize));
        }

        //do FFT analysis
        STOPWATCH("FFT")
        {
          kissFFT(in.data(), out.data());
        }

        //interpreter->typed_tensor<float>(input_tensor)
        float* output = nullptr;
        STOPWATCH("Whistle")
        {
          float prevAmp = 0;
          float ampSum = 0;
          float limitCount = 0;
          for (int i = 0; i < ampSize; i++)
          {
            float amp = std::sqrt((out[i].r * out[i].r) + (out[i].i * out[i].i));

            interpreter->typed_tensor<float>(input_tensor)[i] = 20 * std::log10(amp);
            amplitudes[i] = amp;
            gradients[i] = amp - prevAmp;
            prevAmp = amp;

            if (amp > currentMaxAmp)
              currentMaxAmp = amp;

            if (amp < currentMinAmp)
              currentMinAmp = amp;

            if (amp > limit)
              limitCount++;

            ampSum = ampSum + amp;
          }
          relLimitCount = limitCount / ampSize;
          maxAmpHist.push_front(currentMaxAmp);
          currentMeanAmp = ampSum / ampSize;

          //switch to another mic if the current mic is probably broken
          if (ampSum < 0.2f && currentMic < static_cast<unsigned int>(channels - 1))
          {
            micBrokenCount++;
            if (micBrokenCount >= micBrokenThreshold)
            {
              currentMic++;
              micBrokenCount = 0;
            }
          }
          else if (ampSum < 0.2f && currentMic == static_cast<unsigned int>(channels - 1) && !alert)
          {
#ifdef TARGET_ROBOT
            SystemCall::text2Speech("All micros are probably broken.");
            alert = true;
#endif
          }
          else
          {
            micBrokenCount = 0;
          }

          //do WHistleDetection PM
          int minPos = currentMinFreq * windowSize / sampleRate;
          int maxPos = currentMaxFreq * windowSize / sampleRate;

          //find whistle peak between min. freq. position and  max. freq. position
          peakPos = minPos;
          for (int i = minPos; i <= maxPos; i++)
          {
            if (amplitudes[i] > amplitudes[peakPos])
              peakPos = i;
          }

          float ampWeight = 1.f;
          if (useWeightedPMConfidence)
            ampWeight = (amplitudes[peakPos] / currentMaxAmp);

          //get min/max gradients around peakPos
          float maxGrad = gradients[peakPos];
          float minGrad = gradients[std::min(peakPos + 1, ampSize - 1)];
          maxGrad = std::abs(maxGrad / currentMaxAmp);
          minGrad = std::abs(minGrad / currentMaxAmp);

          if (amplitudes[peakPos] >= maxAmpHist.average())
          {
            minPos = static_cast<int>(minPos * 2);
            maxPos = static_cast<int>(maxPos * 2);

            //find first overtone peak between peak freq. position * overtoneMultiplierMin  and peak freq. position * overtoneMultiplierMax
            overtonePeakPos = minPos;
            for (int i = minPos; i <= maxPos; i++)
            {
              if (amplitudes[i] > amplitudes[overtonePeakPos])
                overtonePeakPos = i;
            }

            //detect whistle
            if (amplitudes[overtonePeakPos] >= currentMeanAmp)
            {
              pmConfidence = std::max(minGrad, maxGrad) * ampWeight;
            }
            else
            {
              pmConfidence = std::min(minGrad, maxGrad) * ampWeight;
            }
          }
          else
          {
            pmConfidence = std::min(minGrad, maxGrad) * ampWeight;
          }

          //do WhistleDetection NN
          output = interpreter->typed_tensor<float>(output_tensor);

          if (interpreter->Invoke() != kTfLiteOk)
          {
            OUTPUT_ERROR("Failed to invoke tflite!");
          }
        }
        nnConfidence = *output; //linear
        //confidence = std::max(0.f, std::min(1.f, (confidence + 1.f) / 2.f));

        //Merge NN, PM and limit information
        float confidence = ((nnWeight * nnConfidence + pmWeight * pmConfidence) / (nnWeight + pmWeight)) * (1 - (limitWeight * relLimitCount));
        confidenceBuffer.push_front(confidence);
        whistle.lastConfidence = confidenceBuffer.back();

        if (useAdaptiveThreshold)
          thresholdBuffer.push_front(threshold + ((1 - threshold) * (limitWeight * relLimitCount)));
        else
          thresholdBuffer.push_front(threshold);

        confidence = confidenceBuffer.back();
        if (confidenceBuffer.back() > thresholdBuffer.average()) // whistle detected this frame, min of #attack detections needed
        {
          detectedWhistleFrequency = peakPos * sampleRate / windowSize;

          //add attack to prevent short noises to be detected as whistles
          if (theFrameInfo.getTimeSince(lastAttackTime) < attackTimeout)
            attackCount++;
          else
            attackCount = 1;

          if (attackCount >= static_cast<unsigned int>(attack))
          {
            ANNOTATION("Whistle", "Whistle was detected.");
            whistle.detectionState = WhistleDortmund::DetectionState::isDetected;
            whistle.lastDetectionTime = theFrameInfo.time;
          }

          if (freqCalibration && detectedWhistleFrequency > minFreq && detectedWhistleFrequency < maxFreq)
          {
            if (confidenceBuffer.back() > thresholdBuffer.average() * 1.5f)
            {
              ANNOTATION("Whistle", "Update whistle frequency.");
              whistleFreqBuffer.push_front(detectedWhistleFrequency);
              int minDiff = std::abs(whistleFreqBuffer.average() - currentMinFreq);
              int maxDiff = std::abs(whistleFreqBuffer.average() - currentMaxFreq);

              currentMinFreq = std::min(whistleFreqBuffer.average() - minDiff / 2, whistleFreqBuffer.average() - 250);
              currentMaxFreq = std::max(whistleFreqBuffer.average() + maxDiff / 2, whistleFreqBuffer.average() + 250);
            }
          }
          whistle.minFrequency = currentMinFreq;
          whistle.maxFrequency = currentMaxFreq;
          whistle.detectedWhistleFrequency = detectedWhistleFrequency;
          lastAttackTime = theFrameInfo.time;
        }
      }
    }
  }
  else
  {
    whistle.detectionState = WhistleDortmund::DetectionState::dontKnow;
    attackCount = 1;
  }
  whistle.currentMic = static_cast<WhistleDortmund::Microphone>(currentMic);

  if (!eval)
    evaluationStarted = false;

  if (eval && !detectionProcessed && whistle.detectionState == WhistleDortmund::DetectionState::isDetected)
  {
    OutTextFile evalWhistleFile(std::string(File::getBHDir()) + "/Config/Sounds/Whistle/whistleEval.csv", evaluationStarted);
    if (evalWhistleFile.exists())
      evalWhistleFile << std::to_string(whistle.lastDetectionTime) + ";";
    detectionProcessed = true;
    evaluationStarted = true;
  }
  else if (whistle.detectionState != WhistleDortmund::DetectionState::isDetected)
  {
    detectionProcessed = false;
  }

  //add release to ensure the detection plot to be smooth
  if (whistle.detectionState == WhistleDortmund::DetectionState::isDetected)
    releaseCount = 0;
  else if (releaseCount < static_cast<unsigned int>(release))
  {
    whistle.detectionState = WhistleDortmund::DetectionState::isDetected;
    releaseCount++;
  }

  //debug draw FFT with detection rects and grids
  COMPLEX_IMAGE(FFT)
  {
    // transform sample rate to fft size to debug image size
    int min_x = currentMinFreq * windowSize / sampleRate * DEBUG_WIDTH / static_cast<int>(amplitudes.size());
    int max_x = currentMaxFreq * windowSize / sampleRate * DEBUG_WIDTH / static_cast<int>(amplitudes.size());

    // draw main detection rect
    for (int x = min_x; x <= max_x; x++)
      for (int y = static_cast<int>(0); y <= DEBUG_HEIGHT; y++)
        DEBUG_IMAGE_SET_PIXEL_YELLOW(FFT, x, DEBUG_HEIGHT - y);

    // draw horizontal grid
    for (int y = 0; y < DEBUG_HEIGHT; y += GRID_DISTANCE_Y)
    {
      DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 0, DEBUG_HEIGHT - y, DEBUG_WIDTH, DEBUG_HEIGHT - y, 80, 80, 80);
    }

    int grid = 0;
    for (int x = 0; x < DEBUG_WIDTH; x++)
    {
      // transform FFT size to debug image size
      int x_temp = static_cast<int>(x * amplitudes.size() / DEBUG_WIDTH);
      // remove FFT number errors to avoid possible crashes
      int y_temp = std::max(0, std::min(DEBUG_HEIGHT, (int)amplitudes[x_temp]));

      // draw vertical grid
      if ((x_temp * sampleRate / windowSize) >= grid)
      {
        DEBUG_IMAGE_DRAW_LINE_RGB(FFT, x, 0, x, DEBUG_HEIGHT, 80, 80, 80);
        grid += GRID_DISTANCE_X;
      }

      // draw FFT lines
      DEBUG_IMAGE_DRAW_LINE_RGB(FFT, x, DEBUG_HEIGHT - y_temp, x, DEBUG_HEIGHT, 255, 0, 0);
    }
  }

  // draw chroma view of FFTs in a short period of time
  COMPLEX_IMAGE(CHROMA)
  {
    // fill vertical line with FFT informations
    for (int i = 0; i < ampSize; i++)
    {
      int brightness = (255 * std::max(0, std::min(CHROMA_MAX, (int)amplitudes[i]))) / CHROMA_MAX;
      DEBUG_IMAGE_SET_PIXEL_RGB(CHROMA, chromaPos, ampSize - i - 1, 0, brightness, 0);
      DEBUG_IMAGE_SET_PIXEL_RGB(CHROMA, (chromaPos + 1), ampSize - i - 1, 255, 0, 0);
    }
    // set counter to next vertical line
    chromaPos++;
    chromaPos = chromaPos % CHROMA_WIDTH;
  }

  SEND_DEBUG_IMAGE(FFT);
  SEND_DEBUG_IMAGE(CHROMA);
  PLOT("representation:Whistle:confidence", confidenceBuffer.back());
  PLOT("representation:Whistle:nnConfidence", nnConfidence);
  PLOT("representation:Whistle:pmConfidence", pmConfidence);
  PLOT("representation:Whistle:relLimitCount", relLimitCount);
  PLOT("representation:Whistle:nnConfidenceWeighted", nnWeight * nnConfidence);
  PLOT("representation:Whistle:pmConfidenceWeighted", pmWeight * pmConfidence);
  PLOT("representation:Whistle:relLimitCountWeighted", limitWeight * relLimitCount);
  PLOT("representation:Whistle:detected", whistle.detectionState == WhistleDortmund::DetectionState::isDetected ? 1 : 0);
  PLOT("representation:Whistle:threshold", thresholdBuffer.average());
  PLOT("representation:Whistle:meanAmp", currentMeanAmp);
  PLOT("representation:Whistle:minAmp", currentMinAmp);
  PLOT("representation:Whistle:maxAmp", currentMaxAmp);
}

/**
*   Note that the arrays in and out must have the same size as given by param size
*/
void WhistleDetector::kissFFT(kiss_fft_cpx in[], kiss_fft_cpx out[])
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

void WhistleDetector::setup()
{
  // set up tflite
  std::string filename = std::string(File::getBHDir()) + whistleNetPath;
  oldWhistleNetPath = whistleNetPath;

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
    input_tensor = interpreter->inputs()[0];
    output_tensor = interpreter->outputs()[0];
    TfLiteIntArray* input_dims = interpreter->tensor(input_tensor)->dims;
    //int input_batch = input_dims->data[0];
    int input_size = input_dims->data[1];
    int channels = input_dims->data[2];


    std::vector<int> new_input;
    new_input.push_back(1);
    new_input.push_back(input_size);
    new_input.push_back(channels);
    interpreter->ResizeInputTensor(input_tensor, new_input);

    // Allocate memory fore the tensors
    interpreter->AllocateTensors();

    // Setup buffers for pre- and post-processing
    windowSize = (input_size * 2) - 2;
    ampSize = input_size;

    samplesLeft = windowSize / 2;
    buffer = std::vector<float>(windowSize);

    in = std::vector<kiss_fft_cpx>(windowSize);
    out = std::vector<kiss_fft_cpx>(windowSize);
    amplitudes = std::vector<float>(ampSize);
    gradients = std::vector<float>(ampSize);

    SET_DEBUG_IMAGE_SIZE(CHROMA, CHROMA_WIDTH, ampSize);
  }
}

MAKE_MODULE(WhistleDetector, modeling)
