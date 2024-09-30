
#include "FieldColorMask.h"
#include "Tools/Debugging/DebugImages.h"

#include <typeinfo>

void FieldColorMask::draw() const
{
  if (typeid(*this) == typeid(FieldColorMask))
  {
    COMPLEX_IMAGE(FieldColorMaskLower)
    {
      DECLARE_LOCAL_DEBUG_IMAGE(FieldColorMaskLower);
      INIT_DEBUG_IMAGE_BLACK(FieldColorMaskLower, width, height);

      for (int y = 0; y < height; ++y)
      {
        for (int x = 0; x < width; ++x)
        {
          if ((*this)(x, y))
          {
            DEBUG_IMAGE_SET_PIXEL_WHITE(FieldColorMaskLower, x, y);
          }
        }
      }

      SEND_DEBUG_IMAGE(FieldColorMaskLower);
    }
  }
  else
  {
    COMPLEX_IMAGE(FieldColorMaskUpper)
    {
      DECLARE_LOCAL_DEBUG_IMAGE(FieldColorMaskUpper);
      INIT_DEBUG_IMAGE_BLACK(FieldColorMaskUpper, width, height);

      for (int y = 0; y < height; ++y)
      {
        for (int x = 0; x < width; ++x)
        {
          if ((*this)(x, y))
          {
            DEBUG_IMAGE_SET_PIXEL_WHITE(FieldColorMaskUpper, x, y);
          }
        }
      }

      SEND_DEBUG_IMAGE(FieldColorMaskUpper);
    }
  }
}
