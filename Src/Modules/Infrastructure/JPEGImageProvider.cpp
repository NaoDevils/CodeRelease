/**
* @file JPEGImageProvider.cpp
* This file declares a module that provides JPEG images.
* @author Aaron Larisch
*/

#include "Modules/Infrastructure/JPEGImageProvider.h"

void JPEGImageProvider::update(JPEGImage& jpegImage)
{
  jpegImage.fromImage(theImage, quality);
}

void JPEGImageProvider::update(JPEGImageUpper& jpegImageUpper)
{
  jpegImageUpper.fromImage(theImageUpper, quality);
}

MAKE_MODULE(JPEGImageProvider, cognitionInfrastructure)
