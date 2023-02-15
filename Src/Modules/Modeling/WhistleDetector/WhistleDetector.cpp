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
  ringPos = 0;
  samplesLeft = windowSize / 2;
  buffer = std::vector<float>(windowSize); //TODO: new in set, free in play

  releaseCount = static_cast<unsigned int>(release);
  attackCount = 0;

  in = std::vector<kiss_fft_cpx>(windowSize);
  out = std::vector<kiss_fft_cpx>(windowSize);
  amplitudes = std::vector<float>(AMP_SIZE);

  SET_DEBUG_IMAGE_SIZE(CHROMA, CHROMA_WIDTH, AMP_SIZE);

  // set up tflite
  std::string filename = std::string(File::getBHDir()) + "/Config/WhistleDet_precision_0.9900_recall_0.6050_threshold_0.85.tflite"; //"/Config/WhistleDet_precision_0.9901_recall_0.7078_threshold_0.87.tflite"

  // Load the model
  model = tflite::FlatBufferModel::BuildFromFile(filename.c_str());

  // Build the interpreter
  tflite::InterpreterBuilder builder(*model, resolver);
  builder(&interpreter);

  interpreter->SetNumThreads(1);

  // Resize input tensors
  input_tensor = interpreter->inputs()[0];
  output_tensor = interpreter->outputs()[0];
  TfLiteIntArray* input_dims = interpreter->tensor(input_tensor)->dims;
  //int intput_batch = intput_dims->data[0];
  int input_size = input_dims->data[1];
  int channels = input_dims->data[2];

  std::vector<int> new_input;
  new_input.push_back(1);
  new_input.push_back(input_size);
  new_input.push_back(channels);
  interpreter->ResizeInputTensor(input_tensor, new_input);

  // Allocate memory fore the tensors
  interpreter->AllocateTensors();
}

void WhistleDetector::update(WhistleDortmund& whistle)
{
  INIT_DEBUG_IMAGE_BLACK(FFT, DEBUG_WIDTH, DEBUG_HEIGHT);
  DECLARE_PLOT("representation:Whistle:detected");
  DECLARE_PLOT("representation:Whistle:confidence");

  DEBUG_RESPONSE("module:WhistleDetector:detectWhistle")
  {
    whistle.detectionState = WhistleDortmund::DetectionState::isDetected;
    whistle.lastDetectionTime = theFrameInfo.time;
    return;
  }

  //update local constants
  const int sampleRate = theAudioData.sampleRate;
  const int channels = theAudioData.channels;

  //declare detection variables
  //int peakPos = -1, peak1Pos = -1, peak2Pos = -1;

  if (theAudioData.isValid)
  {
    unsigned int audioDataPos = 0;

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
        float* output;
        STOPWATCH("WhistleNN")
        {
          for (int i = 0; i < AMP_SIZE; i++)
          {
            interpreter->typed_tensor<float>(input_tensor)[i] = 20 * std::log10(std::sqrt((out[i].r * out[i].r) + (out[i].i * out[i].i))); //TODO: sqrt necessary?
          }

          //do WhistleDetection
          output = interpreter->typed_tensor<float>(output_tensor);

          if (interpreter->Invoke() != kTfLiteOk)
          {
            OUTPUT_ERROR("Failed to invoke tflite!");
          }
        }
        float confidence = *output; //linear
        //confidence = std::max(0.f, std::min(1.f, (confidence + 1.f) / 2.f));

        whistle.lastConfidence = confidence;
        if (confidence > threshold) // whistle detected this frame, min of #attack detections needed
        {
          if (theFrameInfo.getTimeSince(lastAttackTime) < attackTimeout)
          {
            //add attack to prevent short noises to be detected as whistles
            attackCount++;
            if (attackCount >= static_cast<unsigned int>(attack))
            {
              if (theMotionInfo.inStandUpMotion())
              {
                ANNOTATION("Whistle", "Whistle was detected and ignored due to stand up.");
              }
              else
              {
                ANNOTATION("Whistle", "Whistle was detected.");
                whistle.detectionState = WhistleDortmund::DetectionState::isDetected;
                whistle.lastDetectionTime = theFrameInfo.time;
              }
            }
          }
          else
          {
            attackCount = 0;
          }
          lastAttackTime = theFrameInfo.time;
        }
      }
    }
  }
  else
  {
    whistle.detectionState = WhistleDortmund::DetectionState::dontKnow;
    attackCount = 0;
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
    int min_x = minFreq * windowSize / sampleRate * DEBUG_WIDTH / static_cast<int>(amplitudes.size());
    int max_x = maxFreq * windowSize / sampleRate * DEBUG_WIDTH / static_cast<int>(amplitudes.size());

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
    for (int i = 0; i < AMP_SIZE; i++)
    {
      int brightness = (255 * std::max(0, std::min(CHROMA_MAX, (int)amplitudes[i]))) / CHROMA_MAX;
      DEBUG_IMAGE_SET_PIXEL_RGB(CHROMA, chromaPos, AMP_SIZE - i - 1, 0, brightness, 0);
      DEBUG_IMAGE_SET_PIXEL_RGB(CHROMA, (chromaPos + 1), AMP_SIZE - i - 1, 255, 0, 0);
    }
    // set counter to next vertical line
    chromaPos++;
    chromaPos = chromaPos % CHROMA_WIDTH;
  }

  SEND_DEBUG_IMAGE(FFT);
  SEND_DEBUG_IMAGE(CHROMA);
  PLOT("representation:Whistle:confidence", whistle.lastConfidence);
  PLOT("representation:Whistle:detected", (int)whistle.detectionState - 1);
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

MAKE_MODULE(WhistleDetector, modeling)
