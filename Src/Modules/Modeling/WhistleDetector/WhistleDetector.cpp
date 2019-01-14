#include "WhistleDetector.h"

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
    samplesLeft = WINDOW_SIZE / 2;
    buffer = std::vector<float>(WINDOW_SIZE); //TODO: new in set, free in play

    releaseCount = static_cast<unsigned int>(RELEASE);
    attackCount = 0;

    in = std::vector<kiss_fft_cpx>(WINDOW_SIZE);
    out = std::vector<kiss_fft_cpx>(WINDOW_SIZE);
    amplitudes = std::vector<float>(AMP_SIZE);

    SET_DEBUG_IMAGE_SIZE(CHROMA, CHROMA_WIDTH, AMP_SIZE);
}

void WhistleDetector::update(WhistleDortmund &whistle)
{ 
  MODIFY("module:WhistleDetector:MIN_FREQ", MIN_FREQ);
  MODIFY("module:WhistleDetector:MAX_FREQ", MAX_FREQ);
  MODIFY("module:WhistleDetector:MIN_AMP", MIN_AMP);
  MODIFY("module:WhistleDetector:OVERTONE_MULT_MIN_1", OVERTONE_MULT_MIN_1);
  MODIFY("module:WhistleDetector:OVERTONE_MULT_MAX_1", OVERTONE_MULT_MAX_1);
  MODIFY("module:WhistleDetector:OVERTONE_MIN_AMP_1", OVERTONE_MIN_AMP_1);
  MODIFY("module:WhistleDetector:OVERTONE_MULT_MIN_2", OVERTONE_MULT_MIN_2);
  MODIFY("module:WhistleDetector:OVERTONE_MULT_MAX_2", OVERTONE_MULT_MAX_2);
  MODIFY("module:WhistleDetector:OVERTONE_MIN_AMP_2", OVERTONE_MIN_AMP_2);
  MODIFY("module:WhistleDetector:RELEASE", RELEASE);
  MODIFY("module:WhistleDetector:ATTACK", ATTACK);
  MODIFY("module:WhistleDetector:USE_HANN_WINDOWING", USE_HANN_WINDOWING);
  MODIFY("module:WhistleDetector:USE_NUTTALL_WINDOWING", USE_NUTTALL_WINDOWING);

    INIT_DEBUG_IMAGE_BLACK(FFT, DEBUG_WIDTH, DEBUG_HEIGHT);
    DECLARE_PLOT("representation:Whistle:detected");

    //update local constants
    const int sampleRate = theAudioDataDortmund.sampleRate;
    const int channels = theAudioDataDortmund.channels;

    //declare detection variables
    int peakPos = -1, peak1Pos = -1, peak2Pos = -1;

    if (theAudioDataDortmund.isValid)
    {
        unsigned int audioDataPos = 0;

        while (audioDataPos < theAudioDataDortmund.samples.size())
        {
            while (audioDataPos < theAudioDataDortmund.samples.size() && samplesLeft > 0)
            {
                //S16 to float
                //buffer[ringPos] = static_cast<float>(theAudioDataDortmund.samples[audioDataPos])
                //    / static_cast<float>(std::numeric_limits<short>::max());

                buffer[ringPos] = theAudioDataDortmund.samples[audioDataPos];

                audioDataPos += channels;
                samplesLeft--;
                ringPos = (ringPos + 1) % WINDOW_SIZE;
            }

            if (samplesLeft == 0)
            {
                whistle.detectionState = WhistleDortmund::DetectionState::notDetected;
                whistle.detected = false;
                samplesLeft = WINDOW_SIZE / 2;

                for (unsigned int i = 0; i < static_cast<unsigned int>(WINDOW_SIZE); i++)
                {
                    in[i].r = buffer[(i + ringPos) % WINDOW_SIZE];
                    in[i].i = 0.f;

                    //apply Hann window
                    if (USE_HANN_WINDOWING)
                       in[i].r *= pow(sin(pi * i / WINDOW_SIZE), 2);
                    else if (USE_NUTTALL_WINDOWING)
                       in[i].r *= 0.355768
                           - 0.487396 * sin(1 * pi * i / WINDOW_SIZE)
                           + 0.144232 * sin(2 * pi * i / WINDOW_SIZE)
                           - 0.012604 * sin(3 * pi * i / WINDOW_SIZE);
                }

                //do FFT analysis
                kissFFT(in.data(), out.data());

                for (int i = 0; i < AMP_SIZE; i++)
                {
                    amplitudes[i] = std::sqrt((out[i].r * out[i].r) + (out[i].i * out[i].i)); //TODO: sqrt necessary?
                }

                //do WhistleDetection
                int min_i = MIN_FREQ * WINDOW_SIZE / sampleRate;
                int max_i = MAX_FREQ * WINDOW_SIZE / sampleRate;

                peakPos = min_i;
                for (int i = min_i; i <= max_i; i++)
                {
                    if (amplitudes[i] > amplitudes[peakPos])
                        peakPos = i;
                }
                if (amplitudes[peakPos] >= MIN_AMP)
                {
                    min_i = static_cast<int>(peakPos * OVERTONE_MULT_MIN_1);
                    max_i = static_cast<int>(peakPos * OVERTONE_MULT_MAX_1);

                    peak1Pos = min_i;
                    for (int i = min_i; i <= max_i; i++)
                    {
                        if (amplitudes[i] > amplitudes[peak1Pos])
                            peak1Pos = i;
                    }

                    if (amplitudes[peak1Pos] >= OVERTONE_MIN_AMP_1)
                    {
                        min_i = static_cast<int>(peakPos * OVERTONE_MULT_MIN_2);
                        max_i = static_cast<int>(peakPos * OVERTONE_MULT_MAX_2);

                        peak2Pos = min_i;
                        for (int i = min_i; i <= max_i; i++)
                        {
                            if (amplitudes[i] > amplitudes[peak2Pos])
                                peak2Pos = i;
                        }

                        if (amplitudes[peak2Pos] >= OVERTONE_MIN_AMP_2)
                        {                           
                            
                            if (theFrameInfo.getTimeSince(lastAttackTime) < ATTACK_TIMEOUT_MS)
                            {
                                //add attack to prevent short noises to be detected as whistles
                                attackCount++;
                                if (attackCount >= static_cast<unsigned int>(ATTACK))
                                {
                                    whistle.detectionState = WhistleDortmund::DetectionState::isDetected;
                                    whistle.detected = true;
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
        }
    }
    else
    {
        whistle.detectionState = WhistleDortmund::DetectionState::dontKnow;
        whistle.detected = false;
        attackCount = 0;
    }

    //add release to ensure the detection plot to be smooth
    if (whistle.detectionState == WhistleDortmund::DetectionState::isDetected)
        releaseCount = 0;
    else if (releaseCount < static_cast<unsigned int>(RELEASE))
    {
        whistle.detectionState = WhistleDortmund::DetectionState::isDetected;
        whistle.detected = true;
        releaseCount++;
    }

    //debug draw FFT with detection rects and grids
    COMPLEX_IMAGE(FFT)
    {
        // transform sample rate to fft size to debug image size
        int min_x = MIN_FREQ * WINDOW_SIZE / sampleRate * DEBUG_WIDTH / static_cast<int>(amplitudes.size());
        int max_x = MAX_FREQ * WINDOW_SIZE / sampleRate * DEBUG_WIDTH / static_cast<int>(amplitudes.size());
        int debugPeakPos = peakPos * DEBUG_WIDTH / static_cast<int>(amplitudes.size());
        int debugPeak1Pos = peak1Pos * DEBUG_WIDTH / static_cast<int>(amplitudes.size());
        int debugPeak2Pos = peak2Pos * DEBUG_WIDTH / static_cast<int>(amplitudes.size());

        // draw main detection rect
        for (int x = min_x; x <= max_x; x++)
            for (int y = static_cast<int>(MIN_AMP); y <= DEBUG_HEIGHT; y++)
                DEBUG_IMAGE_SET_PIXEL_YELLOW(FFT, x, DEBUG_HEIGHT - y);

        // draw detection rect for first overtone only if peak1Pos has been calculate
        if (peak1Pos >= 0)
            for (int x = static_cast<int>(debugPeakPos * OVERTONE_MULT_MIN_1); x <= debugPeakPos * OVERTONE_MULT_MAX_1; x++)
                for (int y = static_cast<int>(OVERTONE_MIN_AMP_1); y <= DEBUG_HEIGHT; y++)
                    DEBUG_IMAGE_SET_PIXEL_YELLOW(FFT, x, DEBUG_HEIGHT - y);

        // draw detection rect for second overtone only if peak2Pos has been calculate
        if (peak2Pos >= 0)
            for (int x = static_cast<int>(debugPeakPos * OVERTONE_MULT_MIN_2); x <= debugPeakPos * OVERTONE_MULT_MAX_2; x++)
                for (int y = static_cast<int>(OVERTONE_MIN_AMP_2); y <= DEBUG_HEIGHT; y++)
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
        for (int x = 0; x<DEBUG_WIDTH; x++)
        {
            // transform FFT size to debug image size
            int x_temp = static_cast<int>(x*amplitudes.size() / DEBUG_WIDTH);
            // remove FFT number errors to avoid possible crashes
            int y_temp = std::max(0, std::min(DEBUG_HEIGHT, (int)amplitudes[x_temp]));

            // draw vertical grid
            if ((x_temp * sampleRate / WINDOW_SIZE) >= grid)
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
    PLOT("representation:Whistle:detected", (int)whistle.detectionState - 1);
}

/**
*   Note that the arrays in and out must have the same size as given by param size
*/
void WhistleDetector::kissFFT(kiss_fft_cpx in[], kiss_fft_cpx out[])
{
    kiss_fft_cfg cfg;

    if ((cfg = kiss_fft_alloc(WINDOW_SIZE, 0/*is_inverse_fft*/, NULL, NULL)) != NULL)
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
