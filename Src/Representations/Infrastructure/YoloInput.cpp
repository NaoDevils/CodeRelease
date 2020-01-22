/**
* @file YoloInput.cpp
*
*/
#include "YoloInput.h"
#include "Tools/ColorModelConversions.h"

void YoloInput::toImage(Image& dest) const
{
  Image::Pixel pixel; 
  unsigned char r, g, b, y, cb, cr;
  int scale = dest.width / width;

  //dest.setResolution(width, height);
  dest.timeStamp = timeStamp + 1;
  Image::Pixel* pDest = dest[0];
  
  for (int i = 0; i < height; i++) {
    for (int h = 0; h < scale*2; h++) {
      for (int j = 0; j < width; j++) {
        if (channel == 3) {
          r = static_cast<unsigned char>(image[i * width * channel + j * channel + 0] * 255.f);
          g = static_cast<unsigned char>(image[i * width * channel + j * channel + 1] * 255.f);
          b = static_cast<unsigned char>(image[i * width * channel + j * channel + 2] * 255.f);
          ColorModelConversions::fromRGBToYCbCr(r, g, b, y, cb, cr);
          pixel.channels[0] = y;
          pixel.channels[1] = cb;
          pixel.channels[2] = y;
          pixel.channels[3] = cr;
        }
        else {
          pixel.channels[0] = static_cast<unsigned char>(image[i * width + j] * 255.f);
          pixel.channels[1] = 127;
          pixel.channels[2] = static_cast<unsigned char>(image[i * width + j] * 255.f);
          pixel.channels[3] = 127;
        }
        for (int w= 0; w < scale; w++) {
          *pDest = pixel;
          pDest++;
        }
        
      }

    }
  }
}


