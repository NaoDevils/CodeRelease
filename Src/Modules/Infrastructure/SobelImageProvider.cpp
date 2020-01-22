#include <algorithm>
#include <iostream>

#include "SobelImageProvider.h"
#include "Tools/Debugging/DebugDrawings.h" // PLOT
#include "Tools/ImageProcessing/SobelImplementations.h"

MAKE_MODULE(SobelImageProvider, cognitionInfrastructure)

SobelImageProvider::SobelImageProvider()
{
}

void SobelImageProvider::update(SobelImage &sobelImage)
{
  bool aligned_16_byte = (reinterpret_cast<uintptr_t>(theImage.image) % 16) == 0;

  // we have 3 bool values
  // horizontal -> compute g_x
  // vertical   -> compute g_y
  // magnitude  -> approximate sqrt(g_x^2 + g_y^2) with max(g_x, g_y) + 1/4 min(g_x, g_y)
  // represent them as one byte
  unsigned char computations = (unsigned char) (horizontal << 2) +
                               (unsigned char) (vertical   << 1) +
                               (unsigned char) magnitude;

  // set image width and height
  switch (reduction)
  {
    case Reduction::full:
    {
      sobelImage.width = 2 * theImage.width;
      sobelImage.height = 2 * theImage.height;
      break;
    }
    case Reduction::half:
    {
      sobelImage.width = theImage.width;
      sobelImage.height = theImage.height;
      break;
    }
    case Reduction::quarter:
    {
      sobelImage.width = theImage.width / 2;
      sobelImage.height = theImage.height / 2;
      break;
    }
  }

  // downsample debug image
  const bool downsample_debug_image = sobelImage.width > 640;

  const int debug_draw_width = downsample_debug_image ? sobelImage.width / 2 : sobelImage.width;
  const int debug_draw_height = downsample_debug_image ? sobelImage.height / 2 : sobelImage.height;

  // initialize debug drawing
  INIT_DEBUG_IMAGE_BLACK(SobelImageHorizontal, debug_draw_width, debug_draw_height);
  INIT_DEBUG_IMAGE_BLACK(SobelImageVertical, debug_draw_width, debug_draw_height);
  INIT_DEBUG_IMAGE_BLACK(SobelImageMagnitude, debug_draw_width, debug_draw_height);

  // call the implementation
  switch(reduction)
  {
    case Reduction::full:
    {
      if (aligned_16_byte)
      {
        switch(computations)
        {
          case 0: SobelImplementations::sse<1, 0, true>(theImage, sobelImage); break;
          case 1: SobelImplementations::sse<1, 1, true>(theImage, sobelImage); break;
          case 2: SobelImplementations::sse<1, 2, true>(theImage, sobelImage); break;
          case 3: SobelImplementations::sse<1, 3, true>(theImage, sobelImage); break;
          case 4: SobelImplementations::sse<1, 4, true>(theImage, sobelImage); break;
          case 5: SobelImplementations::sse<1, 5, true>(theImage, sobelImage); break;
          case 6: SobelImplementations::sse<1, 6, true>(theImage, sobelImage); break;
          case 7: SobelImplementations::sse<1, 7, true>(theImage, sobelImage); break;
          default: return;
        }
        break;
      }
      else
      {
        switch(computations)
        {
          case 0: SobelImplementations::sse<1, 0, false>(theImage, sobelImage); break;
          case 1: SobelImplementations::sse<1, 1, false>(theImage, sobelImage); break;
          case 2: SobelImplementations::sse<1, 2, false>(theImage, sobelImage); break;
          case 3: SobelImplementations::sse<1, 3, false>(theImage, sobelImage); break;
          case 4: SobelImplementations::sse<1, 4, false>(theImage, sobelImage); break;
          case 5: SobelImplementations::sse<1, 5, false>(theImage, sobelImage); break;
          case 6: SobelImplementations::sse<1, 6, false>(theImage, sobelImage); break;
          case 7: SobelImplementations::sse<1, 7, false>(theImage, sobelImage); break;
          default: return;
        }
        break;
      }
    }
    case Reduction::half:
    {
      if (aligned_16_byte)
      {
        switch(computations)
        {
          case 0: SobelImplementations::sse<2, 0, true>(theImage, sobelImage); break;
          case 1: SobelImplementations::sse<2, 1, true>(theImage, sobelImage); break;
          case 2: SobelImplementations::sse<2, 2, true>(theImage, sobelImage); break;
          case 3: SobelImplementations::sse<2, 3, true>(theImage, sobelImage); break;
          case 4: SobelImplementations::sse<2, 4, true>(theImage, sobelImage); break;
          case 5: SobelImplementations::sse<2, 5, true>(theImage, sobelImage); break;
          case 6: SobelImplementations::sse<2, 6, true>(theImage, sobelImage); break;
          case 7: SobelImplementations::sse<2, 7, true>(theImage, sobelImage); break;
          default: return;
        }
        break;
      }
      else
      {
        switch(computations)
        {
          case 0: SobelImplementations::sse<2, 0, false>(theImage, sobelImage); break;
          case 1: SobelImplementations::sse<2, 1, false>(theImage, sobelImage); break;
          case 2: SobelImplementations::sse<2, 2, false>(theImage, sobelImage); break;
          case 3: SobelImplementations::sse<2, 3, false>(theImage, sobelImage); break;
          case 4: SobelImplementations::sse<2, 4, false>(theImage, sobelImage); break;
          case 5: SobelImplementations::sse<2, 5, false>(theImage, sobelImage); break;
          case 6: SobelImplementations::sse<2, 6, false>(theImage, sobelImage); break;
          case 7: SobelImplementations::sse<2, 7, false>(theImage, sobelImage); break;
          default: return;
        }
        break;
      }
    }
    case Reduction::quarter:
    {
      if (aligned_16_byte)
      {
        switch(computations)
        {
          case 0: SobelImplementations::sse<4, 0, true>(theImage, sobelImage); break;
          case 1: SobelImplementations::sse<4, 1, true>(theImage, sobelImage); break;
          case 2: SobelImplementations::sse<4, 2, true>(theImage, sobelImage); break;
          case 3: SobelImplementations::sse<4, 3, true>(theImage, sobelImage); break;
          case 4: SobelImplementations::sse<4, 4, true>(theImage, sobelImage); break;
          case 5: SobelImplementations::sse<4, 5, true>(theImage, sobelImage); break;
          case 6: SobelImplementations::sse<4, 6, true>(theImage, sobelImage); break;
          case 7: SobelImplementations::sse<4, 7, true>(theImage, sobelImage); break;
          default: return;
        }
        break;
      }
      else
      {
        switch(computations)
        {
          case 0: SobelImplementations::sse<4, 0, false>(theImage, sobelImage); break;
          case 1: SobelImplementations::sse<4, 1, false>(theImage, sobelImage); break;
          case 2: SobelImplementations::sse<4, 2, false>(theImage, sobelImage); break;
          case 3: SobelImplementations::sse<4, 3, false>(theImage, sobelImage); break;
          case 4: SobelImplementations::sse<4, 4, false>(theImage, sobelImage); break;
          case 5: SobelImplementations::sse<4, 5, false>(theImage, sobelImage); break;
          case 6: SobelImplementations::sse<4, 6, false>(theImage, sobelImage); break;
          case 7: SobelImplementations::sse<4, 7, false>(theImage, sobelImage); break;
          default: return;
        }
        break;
      }
    }
    default:
      return;
  }

  // draw debug image
  if (horizontal)
  {
    COMPLEX_IMAGE(SobelImageHorizontal)
    {
      // TODO starting at 0, gives aliasing problems, might be a simulator problem only
      for (int i = 0; i < debug_draw_width; i++)
      {
        for (int j = 0; j < debug_draw_height; j++)
        {
          const int ii = downsample_debug_image ? 2 * i + 1 : i;
          const int jj = downsample_debug_image ? 2 * j + 1 : j;
          DEBUG_IMAGE_SET_PIXEL_YUV(SobelImageHorizontal, i, j, sobelImage.getHorizontal(jj, ii), 127, 127);
        }
      }
      SEND_DEBUG_IMAGE(SobelImageHorizontal);
    }
  }
  if (vertical)
  {
    COMPLEX_IMAGE(SobelImageVertical)
    {
      // TODO starting at 0, gives aliasing problems, might be a simulator problem only
      for (int i = 0; i < debug_draw_width; i++)
      {
        for (int j = 0; j < debug_draw_height; j++)
        {
          const int ii = downsample_debug_image ? 2 * i + 1 : i;
          const int jj = downsample_debug_image ? 2 * j + 1 : j;
          DEBUG_IMAGE_SET_PIXEL_YUV(SobelImageVertical, i, j, sobelImage.getVertical(jj, ii), 127, 127);
        }
      }
      SEND_DEBUG_IMAGE(SobelImageVertical);
    }
  }
  if (magnitude)
  {
    COMPLEX_IMAGE(SobelImageMagnitude)
    {
      // TODO starting at 0, gives aliasing problems, might be a simulator problem only
      for (int i = 0; i < debug_draw_width; i++)
      {
        for (int j = 0; j < debug_draw_height; j++)
        {
          const int ii = downsample_debug_image ? 2 * i + 1 : i;
          const int jj = downsample_debug_image ? 2 * j + 1 : j;
          DEBUG_IMAGE_SET_PIXEL_YUV(SobelImageMagnitude, i, j, sobelImage.getMagnitude(jj, ii), 127, 127);
        }
      }
      SEND_DEBUG_IMAGE(SobelImageMagnitude);
    }
  }
}

void SobelImageProvider::update(SobelImageUpper &sobelImageUpper)
{
  bool aligned_16_byte = (reinterpret_cast<uintptr_t>(theImageUpper.image) % 16) == 0;

  // we have 3 bool values
  // horizontal -> compute g_x
  // vertical   -> compute g_y
  // magnitude  -> approximate sqrt(g_x^2 + g_y^2) with max(g_x, g_y) + 1/4 min(g_x, g_y)
  // represent them as one byte
  unsigned char computations = (unsigned char) (horizontalUpper << 2) +
                               (unsigned char) (verticalUpper   << 1) +
                               (unsigned char) magnitudeUpper;

  // set image width and height
  switch (reductionUpper)
  {
    case Reduction::full:
    {
      sobelImageUpper.width = 2 * theImageUpper.width;
      sobelImageUpper.height = 2 * theImageUpper.height;
      break;
    }
    case Reduction::half:
    {
      sobelImageUpper.width = theImageUpper.width;
      sobelImageUpper.height = theImageUpper.height;
      break;
    }
    case Reduction::quarter:
    {
      sobelImageUpper.width = theImageUpper.width / 2;
      sobelImageUpper.height = theImageUpper.height / 2;
      break;
    }
  }

  // downsample debug image
  const bool downsample_debug_image = sobelImageUpper.width > 640;

  const int debug_draw_width = downsample_debug_image ? sobelImageUpper.width / 2 : sobelImageUpper.width;
  const int debug_draw_height = downsample_debug_image ? sobelImageUpper.height / 2 : sobelImageUpper.height;

  // initialize debug drawing
  INIT_DEBUG_IMAGE_BLACK(SobelImageUpperHorizontal, debug_draw_width, debug_draw_height);
  INIT_DEBUG_IMAGE_BLACK(SobelImageUpperVertical, debug_draw_width, debug_draw_height);
  INIT_DEBUG_IMAGE_BLACK(SobelImageUpperMagnitude, debug_draw_width, debug_draw_height);

  // call the implementation
  switch(reductionUpper)
  {
    case Reduction::full:
    {
      if (aligned_16_byte)
      {
        switch(computations)
        {
          case 0: SobelImplementations::sse<1, 0, true>(theImageUpper, sobelImageUpper); break;
          case 1: SobelImplementations::sse<1, 1, true>(theImageUpper, sobelImageUpper); break;
          case 2: SobelImplementations::sse<1, 2, true>(theImageUpper, sobelImageUpper); break;
          case 3: SobelImplementations::sse<1, 3, true>(theImageUpper, sobelImageUpper); break;
          case 4: SobelImplementations::sse<1, 4, true>(theImageUpper, sobelImageUpper); break;
          case 5: SobelImplementations::sse<1, 5, true>(theImageUpper, sobelImageUpper); break;
          case 6: SobelImplementations::sse<1, 6, true>(theImageUpper, sobelImageUpper); break;
          case 7: SobelImplementations::sse<1, 7, true>(theImageUpper, sobelImageUpper); break;
          default: return;
        }
        break;
      }
      else
      {
        switch(computations)
        {
          case 0: SobelImplementations::sse<1, 0, false>(theImageUpper, sobelImageUpper); break;
          case 1: SobelImplementations::sse<1, 1, false>(theImageUpper, sobelImageUpper); break;
          case 2: SobelImplementations::sse<1, 2, false>(theImageUpper, sobelImageUpper); break;
          case 3: SobelImplementations::sse<1, 3, false>(theImageUpper, sobelImageUpper); break;
          case 4: SobelImplementations::sse<1, 4, false>(theImageUpper, sobelImageUpper); break;
          case 5: SobelImplementations::sse<1, 5, false>(theImageUpper, sobelImageUpper); break;
          case 6: SobelImplementations::sse<1, 6, false>(theImageUpper, sobelImageUpper); break;
          case 7: SobelImplementations::sse<1, 7, false>(theImageUpper, sobelImageUpper); break;
          default: return;
        }
        break;
      }
    }
    case Reduction::half:
    {
      if (aligned_16_byte)
      {
        switch(computations)
        {
          case 0: SobelImplementations::sse<2, 0, true>(theImageUpper, sobelImageUpper); break;
          case 1: SobelImplementations::sse<2, 1, true>(theImageUpper, sobelImageUpper); break;
          case 2: SobelImplementations::sse<2, 2, true>(theImageUpper, sobelImageUpper); break;
          case 3: SobelImplementations::sse<2, 3, true>(theImageUpper, sobelImageUpper); break;
          case 4: SobelImplementations::sse<2, 4, true>(theImageUpper, sobelImageUpper); break;
          case 5: SobelImplementations::sse<2, 5, true>(theImageUpper, sobelImageUpper); break;
          case 6: SobelImplementations::sse<2, 6, true>(theImageUpper, sobelImageUpper); break;
          case 7: SobelImplementations::sse<2, 7, true>(theImageUpper, sobelImageUpper); break;
          default: return;
        }
        break;
      }
      else
      {
        switch(computations)
        {
          case 0: SobelImplementations::sse<2, 0, false>(theImageUpper, sobelImageUpper); break;
          case 1: SobelImplementations::sse<2, 1, false>(theImageUpper, sobelImageUpper); break;
          case 2: SobelImplementations::sse<2, 2, false>(theImageUpper, sobelImageUpper); break;
          case 3: SobelImplementations::sse<2, 3, false>(theImageUpper, sobelImageUpper); break;
          case 4: SobelImplementations::sse<2, 4, false>(theImageUpper, sobelImageUpper); break;
          case 5: SobelImplementations::sse<2, 5, false>(theImageUpper, sobelImageUpper); break;
          case 6: SobelImplementations::sse<2, 6, false>(theImageUpper, sobelImageUpper); break;
          case 7: SobelImplementations::sse<2, 7, false>(theImageUpper, sobelImageUpper); break;
          default: return;
        }
        break;
      }
    }
    case Reduction::quarter:
    {
      if (aligned_16_byte)
      {
        switch(computations)
        {
          case 0: SobelImplementations::sse<4, 0, true>(theImageUpper, sobelImageUpper); break;
          case 1: SobelImplementations::sse<4, 1, true>(theImageUpper, sobelImageUpper); break;
          case 2: SobelImplementations::sse<4, 2, true>(theImageUpper, sobelImageUpper); break;
          case 3: SobelImplementations::sse<4, 3, true>(theImageUpper, sobelImageUpper); break;
          case 4: SobelImplementations::sse<4, 4, true>(theImageUpper, sobelImageUpper); break;
          case 5: SobelImplementations::sse<4, 5, true>(theImageUpper, sobelImageUpper); break;
          case 6: SobelImplementations::sse<4, 6, true>(theImageUpper, sobelImageUpper); break;
          case 7: SobelImplementations::sse<4, 7, true>(theImageUpper, sobelImageUpper); break;
          default: return;
        }
        break;
      }
      else
      {
        switch(computations)
        {
          case 0: SobelImplementations::sse<4, 0, false>(theImageUpper, sobelImageUpper); break;
          case 1: SobelImplementations::sse<4, 1, false>(theImageUpper, sobelImageUpper); break;
          case 2: SobelImplementations::sse<4, 2, false>(theImageUpper, sobelImageUpper); break;
          case 3: SobelImplementations::sse<4, 3, false>(theImageUpper, sobelImageUpper); break;
          case 4: SobelImplementations::sse<4, 4, false>(theImageUpper, sobelImageUpper); break;
          case 5: SobelImplementations::sse<4, 5, false>(theImageUpper, sobelImageUpper); break;
          case 6: SobelImplementations::sse<4, 6, false>(theImageUpper, sobelImageUpper); break;
          case 7: SobelImplementations::sse<4, 7, false>(theImageUpper, sobelImageUpper); break;
          default: return;
        }
        break;
      }
    }
    default:
      return;

  }

  // draw debug image
  if (horizontalUpper)
  {
    COMPLEX_IMAGE(SobelImageUpperHorizontal)
    {
      // TODO starting at 0, gives aliasing problems, might be a simulator problem only
      for (int i = 0; i < debug_draw_width; i++)
      {
        for (int j = 0; j < debug_draw_height; j++)
        {
          const int ii = downsample_debug_image ? 2 * i + 1 : i;
          const int jj = downsample_debug_image ? 2 * j + 1 : j;
          DEBUG_IMAGE_SET_PIXEL_YUV(SobelImageUpperHorizontal, i, j, sobelImageUpper.getHorizontal(jj, ii), 127, 127);
        }
      }
      SEND_DEBUG_IMAGE(SobelImageUpperHorizontal);
    }
  }
  if (verticalUpper)
  {
    COMPLEX_IMAGE(SobelImageUpperVertical)
    {
      // TODO starting at 0, gives aliasing problems, might be a simulator problem only
      for (int i = 0; i < debug_draw_width; i++)
      {
        for (int j = 0; j < debug_draw_height; j++)
        {
          const int ii = downsample_debug_image ? 2 * i + 1 : i;
          const int jj = downsample_debug_image ? 2 * j + 1 : j;
          DEBUG_IMAGE_SET_PIXEL_YUV(SobelImageUpperVertical, i, j, sobelImageUpper.getVertical(jj, ii), 127, 127);
        }
      }
      SEND_DEBUG_IMAGE(SobelImageUpperVertical);
    }
  }
  if (magnitudeUpper)
  {
    COMPLEX_IMAGE(SobelImageUpperMagnitude)
    {
      // TODO starting at 0, gives aliasing problems, might be a simulator problem only
      for (int i = 0; i < debug_draw_width; i++)
      {
        for (int j = 0; j < debug_draw_height; j++)
        {
          const int ii = downsample_debug_image ? 2 * i + 1 : i;
          const int jj = downsample_debug_image ? 2 * j + 1 : j;
          DEBUG_IMAGE_SET_PIXEL_YUV(SobelImageUpperMagnitude, i, j, sobelImageUpper.getMagnitude(jj, ii), 127, 127);
        }
      }
      SEND_DEBUG_IMAGE(SobelImageUpperMagnitude);
    }
  }
}
