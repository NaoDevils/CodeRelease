#include "WhistleDetectorMono2019.h"
#include "Tools/Settings.h"

#define DEBUG_WIDTH 512
#define DEBUG_HEIGHT 300
#define GRID_DISTANCE_X 1000
#define GRID_DISTANCE_Y 25

#define CHROMA_WIDTH 500
#define CHROMA_MAX 50

//#define BL 0 // back left mic
//#define BR 1 // back right mic
//#define FL 2 // front left mic
//#define FR 3 // front right mic

WhistleDetectorMono2019::WhistleDetectorMono2019()
{
    ringPos = 0;

    samplesLeft = params.WINDOW_SIZE / 2;
    buffer = std::vector<float>(params.WINDOW_SIZE); //TODO: new in set, free in play

    currMinAmp = params.MIN_AMP;
    channelRequest = params.USE_CHANNEL+1; // makes sure request is checked in initial update iteration
    currChannel = 0;

    releaseCount = static_cast<unsigned int>(params.RELEASE);
    attackCount = 0;

    in = std::vector<kiss_fft_cpx>(params.WINDOW_SIZE);
    out = std::vector<kiss_fft_cpx>(params.WINDOW_SIZE);
    amplitudes = std::vector<float>(AMP_SIZE);

    SET_DEBUG_IMAGE_SIZE(CHROMA, CHROMA_WIDTH, AMP_SIZE);
}

void WhistleDetectorMono2019::update(WhistleDortmund &whistle)
{ 
    MODIFY("module:WhistleDetector:params", params);

    INIT_DEBUG_IMAGE_BLACK(FFT, DEBUG_WIDTH, DEBUG_HEIGHT);
    DECLARE_PLOT("representation:Whistle:detected");

    //update local constants
    const int sampleRate = theAudioData.sampleRate;
    const unsigned channels = theAudioData.channels;

    if (channelRequest != params.USE_CHANNEL)
    {
      channelRequest = params.USE_CHANNEL;
      currChannel = channelRequest;
      if (channelRequest >= channels)
      {
          OUTPUT_WARNING("No channel " << channelRequest << " available. Using " << (channels-1) << " instead.");
          currChannel = channels-1;
      }
    }

    //declare detection variables
    int peak1Pos = -1, peak2Pos = -1;

    if (theAudioData.isValid)
    {
        unsigned int audioDataPos = params.USE_CHANNEL;

        while (audioDataPos < theAudioData.samples.size())
        {
            while (audioDataPos < theAudioData.samples.size() && samplesLeft > 0)
            {
                buffer[ringPos] = theAudioData.samples[audioDataPos];

                audioDataPos += channels; // skip all channels but one
                samplesLeft--;
                ringPos = (ringPos + 1) % params.WINDOW_SIZE;
            }

            if (samplesLeft == 0)
            {
                whistle.detectionState = WhistleDortmund::DetectionState::notDetected;
                samplesLeft = params.WINDOW_SIZE / 2;

                for (unsigned int i = 0; i < static_cast<unsigned int>(params.WINDOW_SIZE); i++)
                {
                    in[i].r = buffer[(i + ringPos) % params.WINDOW_SIZE];
                    in[i].i = 0.f;

                    //apply Hann window
                    if (params.USE_HANN_WINDOWING)
                       in[i].r *= std::pow(std::sin(pi * i / params.WINDOW_SIZE), 2.f);
                    else if (params.USE_NUTTALL_WINDOWING)
                       in[i].r *= 0.355768f
                           - 0.487396f * std::sin(1 * pi * i / params.WINDOW_SIZE)
                           + 0.144232f * std::sin(2 * pi * i / params.WINDOW_SIZE)
                           - 0.012604f * std::sin(3 * pi * i / params.WINDOW_SIZE);
                }

                //do FFT analysis
                kissFFT(in.data(), out.data());

                for (int i = 0; i < AMP_SIZE; i++)
                {
                    amplitudes[i] = std::sqrt((out[i].r * out[i].r) + (out[i].i * out[i].i)); //TODO: sqrt necessary?
                }

                // calculate min_amp regarding environment noise level
                currMinAmp = calc_noise_based_min_amp(amplitudes, sampleRate, params.NOISE_HISTORY_SIZE);

                //do peak detection
                int min_i = params.MIN_FREQ_WHISTLE * params.WINDOW_SIZE / sampleRate;
                int max_i = params.MAX_FREQ_WHISTLE * params.WINDOW_SIZE / sampleRate;

                int lastPeakPos = peakPos;
                // recalculate peakPos
                peakPos = min_i;
				//get maximum
                for (int i = min_i; i <= max_i; i++)
                {
                    if (amplitudes[i] > amplitudes[peakPos])
                        peakPos = i;
                }
                if (lastPeakPos == -1) { // case no peak has been there recently
                  lastPeakPos = peakPos;
                }
                if (amplitudes[peakPos] >= currMinAmp)// && std::abs(peakPos - lastPeakPos) < params.PEAK_DIFF)
                {
                    min_i = static_cast<int>(peakPos * params.OVERTONE_MULT_MIN_1);
                    max_i = static_cast<int>(peakPos * params.OVERTONE_MULT_MAX_1);

                    peak1Pos = min_i;
                    for (int i = min_i; i <= max_i; i++) //maximum for overtone 1
                    {
                        if (amplitudes[i] > amplitudes[peak1Pos])
                            peak1Pos = i;
                    }

                    if (amplitudes[peak1Pos] >= amplitudes[peakPos]*params.OVERTONE_MIN_AMP_1) //overtone 1 is "good enough"
                    {
                        min_i = static_cast<int>(peakPos * params.OVERTONE_MULT_MIN_2);
                        max_i = static_cast<int>(peakPos * params.OVERTONE_MULT_MAX_2);

                        peak2Pos = min_i;
                        for (int i = min_i; i <= max_i; i++)
                        {
                            if (amplitudes[i] > amplitudes[peak2Pos])
                                peak2Pos = i;
                        }

                        if (amplitudes[peak2Pos] >= amplitudes[peakPos]*params.OVERTONE_MIN_AMP_2)
                        {                           
                            if (theFrameInfo.getTimeSince(lastAttackTime) < params.ATTACK_TIMEOUT_MS)
                            {
                                //add attack to prevent short noises to be detected as whistles
                                attackCount++;
                                if (attackCount >= static_cast<unsigned int>(params.ATTACK))
                                {
                                    whistle.detectionState = WhistleDortmund::DetectionState::isDetected;
                                    whistle.lastDetectionTime = theFrameInfo.time;

                                    // print some peak relations for parameter definition
                                    //printf("peak0=%.3f, peak1=%.3f, peak1/peak0=%.3f\n", amplitudes[peakPos],amplitudes[peak1Pos],(amplitudes[peak1Pos]/amplitudes[peakPos]));
                                }
                            }
                            else
                            {
                                attackCount = 0;
                                peakPos = -1;
                            }
                            lastAttackTime = theFrameInfo.time;
                        }
                    }
                }
                //add release to ensure the detection plot to be smooth
                if (whistle.detectionState == WhistleDortmund::DetectionState::isDetected) {
                    releaseCount = 0;
                }
                else if (releaseCount < static_cast<unsigned int>(params.RELEASE))
                {
                    whistle.detectionState = WhistleDortmund::DetectionState::isDetected;
                    releaseCount += 1;
                }
            }
        }
    }
    else
    {
        whistle.detectionState = WhistleDortmund::DetectionState::dontKnow;
        attackCount = 0;
        peakPos = 0;
    }

    //debug draw FFT with detection rects and grids
    if (theAudioData.isValid)
    COMPLEX_IMAGE(FFT)
    {
        // transform sample rate to fft size to debug image size
        int min_x = params.MIN_FREQ_WHISTLE * params.WINDOW_SIZE / sampleRate * DEBUG_WIDTH / static_cast<int>(amplitudes.size());
        int max_x = params.MAX_FREQ_WHISTLE * params.WINDOW_SIZE / sampleRate * DEBUG_WIDTH / static_cast<int>(amplitudes.size());
        int debugPeakPos = peakPos * DEBUG_WIDTH / static_cast<int>(amplitudes.size());
        int debugPeak1Pos = peak1Pos * DEBUG_WIDTH / static_cast<int>(amplitudes.size());
        int debugPeak2Pos = peak2Pos * DEBUG_WIDTH / static_cast<int>(amplitudes.size());

        // draw main detection rect
        for (int x = min_x; x <= max_x; x++)
            for (int y = static_cast<int>(currMinAmp*params.DBG_IMG_Y_STRETCH); y <= DEBUG_HEIGHT; y++)
                DEBUG_IMAGE_SET_PIXEL_YELLOW(FFT, x, DEBUG_HEIGHT - y);

        // draw detection rect for first overtone only if peak1Pos has been calculate
        if (peak1Pos >= 0)
            for (int x = static_cast<int>(debugPeakPos * params.OVERTONE_MULT_MIN_1); x <= debugPeakPos * params.OVERTONE_MULT_MAX_1; x++)
                for (int y = static_cast<int>(amplitudes[peakPos]*params.OVERTONE_MIN_AMP_1*params.DBG_IMG_Y_STRETCH); y <= DEBUG_HEIGHT; y++)
                    DEBUG_IMAGE_SET_PIXEL_YELLOW(FFT, x, DEBUG_HEIGHT - y);

        // draw detection rect for second overtone only if peak2Pos has been calculate
        if (peak2Pos >= 0)
            for (int x = static_cast<int>(debugPeakPos * params.OVERTONE_MULT_MIN_2); x <= debugPeakPos * params.OVERTONE_MULT_MAX_2; x++)
                for (int y = static_cast<int>(amplitudes[peakPos]*params.OVERTONE_MIN_AMP_2*params.DBG_IMG_Y_STRETCH); y <= DEBUG_HEIGHT; y++)
                    DEBUG_IMAGE_SET_PIXEL_YELLOW(FFT, x, DEBUG_HEIGHT - y);

        // draw horizontal grid
        for (int y = 0; y < DEBUG_HEIGHT; y += GRID_DISTANCE_Y)
        {
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 0, DEBUG_HEIGHT - y, DEBUG_WIDTH, DEBUG_HEIGHT - y, 80, 80, 80);
        }

        // draw peaks as lines
        if (whistle.detectionState == WhistleDortmund::DetectionState::isDetected)
        {
            // draw peak 0
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, debugPeakPos - 1, 0, debugPeakPos - 1, DEBUG_HEIGHT, 0, 255, 0);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, debugPeakPos, 0, debugPeakPos, DEBUG_HEIGHT, 0, 255, 0);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, debugPeakPos + 1, 0, debugPeakPos + 1, DEBUG_HEIGHT, 0, 255, 0);

            // draw peak 1
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, debugPeak1Pos - 1, 0, debugPeak1Pos - 1, DEBUG_HEIGHT, 0, 255, 0);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, debugPeak1Pos, 0, debugPeak1Pos, DEBUG_HEIGHT, 0, 255, 0);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, debugPeak1Pos + 1, 0, debugPeak1Pos + 1, DEBUG_HEIGHT, 0, 255, 0);

            // draw peak 2
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, debugPeak2Pos - 1, 0, debugPeak2Pos - 1, DEBUG_HEIGHT, 0, 255, 0);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, debugPeak2Pos, 0, debugPeak2Pos, DEBUG_HEIGHT, 0, 255, 0);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, debugPeak2Pos + 1, 0, debugPeak2Pos + 1, DEBUG_HEIGHT, 0, 255, 0);
        }

        int grid = 0;
        int minPosBackNoise = params.MIN_FREQ_BACKGROUND_NOISE * params.WINDOW_SIZE / sampleRate;
        int maxPosBackNoise = params.MAX_FREQ_BACKGROUND_NOISE * params.WINDOW_SIZE / sampleRate;

        for (int x = 0; x<DEBUG_WIDTH; x++)
        {
            // transform FFT size to debug image size
            int x_temp = static_cast<int>(x*amplitudes.size() / DEBUG_WIDTH);
            // remove FFT number errors to avoid possible crashes
            int y_temp = std::max(0, std::min(DEBUG_HEIGHT, (int)(amplitudes[x_temp]*params.DBG_IMG_Y_STRETCH)));

            // draw vertical grid
            if ((x_temp * sampleRate / params.WINDOW_SIZE) >= grid)
            {
                DEBUG_IMAGE_DRAW_LINE_RGB(FFT, x, 0, x, DEBUG_HEIGHT, 80, 80, 80);
                grid += GRID_DISTANCE_X;
            }

            // draw FFT lines
            if (x >= minPosBackNoise && x <= maxPosBackNoise) // background noise
              DEBUG_IMAGE_DRAW_LINE_RGB(FFT, x, DEBUG_HEIGHT - y_temp, x, DEBUG_HEIGHT, 255, 150, 0);
            else
              DEBUG_IMAGE_DRAW_LINE_RGB(FFT, x, DEBUG_HEIGHT - y_temp, x, DEBUG_HEIGHT, 200, 0, 0);
        }
        // draw channel number to upper left corner
        if (currChannel == 0)
        {
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 1, 1, 1, 5, 255, 255, 255);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 3, 1, 3, 5, 255, 255, 255);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 1, 1, 3, 1, 255, 255, 255);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 1, 5, 3, 5, 255, 255, 255);
        }
        else if (currChannel == 1)
          DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 3, 1, 3, 5, 255, 255, 255);
        else if (currChannel == 2)
        {
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 1, 1, 3, 1, 255, 255, 255);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 3, 1, 3, 3, 255, 255, 255);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 1, 3, 3, 3, 255, 255, 255);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 1, 3, 1, 5, 255, 255, 255);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 1, 5, 3, 5, 255, 255, 255);
        }
        else if (currChannel == 3)
        {
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 3, 1, 3, 5, 255, 255, 255);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 1, 1, 3, 1, 255, 255, 255);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 1, 3, 3, 3, 255, 255, 255);
            DEBUG_IMAGE_DRAW_LINE_RGB(FFT, 1, 5, 3, 5, 255, 255, 255);
        }
    }

    // draw chroma view of FFTs in a short period of time
    if (theAudioData.isValid)
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
    PLOT("representation:Whistle:detected", (int)whistle.detectionState - 1);
}

/**
*   Note that the arrays in and out must have the same size as given by param size
*/
void WhistleDetectorMono2019::kissFFT(kiss_fft_cpx in[], kiss_fft_cpx out[])
{
    kiss_fft_cfg cfg;

    if ((cfg = kiss_fft_alloc(params.WINDOW_SIZE, 0/*is_inverse_fft*/, NULL, NULL)) != NULL)
    {
        kiss_fft(cfg, in, out);
        free(cfg);
    }
    else
    {
        OUTPUT_WARNING("Not enough memory for KissFFT?");
    }
}


/**
 * @brief calc_noise_based_min_amp
 * @param amplitudes vector
 * @return a meaningful minimal amplutude regarding the environment's noise level
 */
float WhistleDetectorMono2019::calc_noise_based_min_amp(std::vector<float> amplitudes, const int sampleRate, const size_t historySize)
{
  ASSERT(historySize > 0);

  //do peak detection
  int min_i = params.MIN_FREQ_BACKGROUND_NOISE * params.WINDOW_SIZE / sampleRate;
  int max_i = params.MAX_FREQ_BACKGROUND_NOISE * params.WINDOW_SIZE / sampleRate;

  float sum = 0;

  for (int i = min_i; i <= max_i; i++)
  {
    sum += amplitudes[i];
  }
  float mean = sum/(max_i - min_i);

  if (history.size() != historySize) {
    history = std::vector<float>(historySize, 5.f*mean);
  }

  if (historyPos >= history.size())
    historyPos = 0;
  history[historyPos] = mean;
  historyPos++;

  float histSum = 0.f;
  for (size_t i = 0; i < historySize; i++)
    histSum += history[i];
  float histMean = histSum / history.size();

  return histMean*params.MIN_AMP;
}

MAKE_MODULE(WhistleDetectorMono2019, modeling)
