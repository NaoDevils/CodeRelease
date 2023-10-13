/**
* @file YoloInput.cpp
*
*/
#include "YoloInput.h"
#include "Tools/ColorModelConversions.h"

void YoloInput::toImage(Image& dest) const
{
  if (!imageUpdated)
    return;

  Image::Pixel pixel;
  unsigned char r, g, b, y, cb, cr;
  int scale = dest.width / width;

  //dest.setResolution(width, height);
  dest.timeStamp = timeStamp + 1;
  Image::Pixel* pDest = dest[0];

  for (int i = 0; i < height; i++)
  {
    for (int h = 0; h < scale * 2; h++)
    {
      for (int j = 0; j < width; j++)
      {
        if (channel == 3)
        {
          r = static_cast<unsigned char>(image[i * width * channel + j * channel + 0] * 255.f);
          g = static_cast<unsigned char>(image[i * width * channel + j * channel + 1] * 255.f);
          b = static_cast<unsigned char>(image[i * width * channel + j * channel + 2] * 255.f);
          ColorModelConversions::fromRGBToYCbCr(r, g, b, y, cb, cr);
          pixel.yCbCrPadding = y;
          pixel.cb = cb;
          pixel.y = y;
          pixel.cr = cr;
        }
        else
        {
          pixel.yCbCrPadding = static_cast<unsigned char>(image[i * width + j] * 255.f);
          pixel.cb = 127;
          pixel.y = static_cast<unsigned char>(image[i * width + j] * 255.f);
          pixel.cr = 127;
        }
        for (int w = 0; w < scale; w++)
        {
          *pDest = pixel;
          pDest++;
        }
      }
    }
  }

  dest.imageSource = ImageSource::yoloInput;
}
