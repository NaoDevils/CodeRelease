#include "WhistleDetectorOld.h"
#include "Platform/SystemCall.h"
#include "Tools/Math/Fourier.h"
#include <string>
#include "Tools/Debugging/DebugDrawings.h"

//  arecord -t raw -f U8 > audio
// 8 kHz, uint8_t


WhistleDetectorOld::WhistleDetectorOld()
{
}

void WhistleDetectorOld::update(WhistleDortmund &whistle)
{
  whistle.detected = false;

  CFourier fft;
  fft.vector = new float[3];
  INIT_DEBUG_IMAGE_BLACK(FFTTT,320,240);
  PLOT("module:WhistleDetector:size", theAudioDataDortmund.samples.size());
  DECLARE_PLOT("module:WhistleDetector:fundamental_frequency");
  if (theAudioDataDortmund.samples.size() != 0)
  {

    fft.ComplexFFT(const_cast<float*>(theAudioDataDortmund.samples.data()), theAudioDataDortmund.samples.size(), theAudioDataDortmund.sampleRate,1);
    minAmp = 0;

    for (int x=0; x<320; x++)
    {
      unsigned int x_temp=((x*(theAudioDataDortmund.sampleRate/2))/320);
		  int y_temp=(int)((240*(pow(fft.vector[2*x_temp],2)+pow(fft.vector[2*x_temp+1],2)))/
			  ((double)pow(fft.vector[2*fft.fundamental_frequency],2)+pow(fft.vector[2*fft.fundamental_frequency+1],2)));

      // Search for the loudest frequence that is not the fundamental freq,
      // which should be the whistle. Please note that the amplitudes are
      // relative to the fundamental frequency, so very low if the robot hears
      // a loud fundamental frequency
      if (x_temp < (fft.fundamental_frequency - 50) || x_temp > (fft.fundamental_frequency + 50))
      {
        if (y_temp > minAmp)
          minAmp = y_temp;
      }
    }
    
    COMPLEX_IMAGE(FFTTT)
    {
      for (int x=0; x<320; x++)
      {
        int x_temp=((x*(theAudioDataDortmund.sampleRate/2))/320);
		    int y_temp=(int)((240*(pow(fft.vector[2*x_temp],2)+pow(fft.vector[2*x_temp+1],2)))/
				((double)pow(fft.vector[2*fft.fundamental_frequency],2)+pow(fft.vector[2*fft.fundamental_frequency+1],2)));

        DEBUG_IMAGE_SET_PIXEL_RGB(FFTTT,x,240 - y_temp,255,0,0);
        DEBUG_IMAGE_SET_PIXEL_RGB(FFTTT,x,240-minAmp,0,255,0);
      }
    }

  }
    
  SEND_DEBUG_IMAGE(FFTTT);
  float freq = fft.fundamental_frequency;
  if (theAudioDataDortmund.samples.size() != 0)
  {
    // If the loudest frequency that is not the fundamental frequency, below
    // threshold
    if (abs(minAmp) < threshold && fft.fundamental_frequency > minFreq)
      durCount++;
    else
      durCount = 0;

    if (durCount > minDur)
      whistle.detected = true;
    else
      whistle.detected = false;
    PLOT("module:WhistleDetector:fundamental_frequency", freq);
  }
  else
    freq = 0;
  PLOT("representation:Whistle:detected", (int)whistle.detected);

  
}

MAKE_MODULE(WhistleDetectorOld, modeling);