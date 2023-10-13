/**
* @file PenaltyCrossHypotheses.cpp
* Declaration of a class that represents the penalty cross hypotheses found in an image.
* @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
*/

#include "PenaltyCrossHypotheses.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugImages.h"

void PenaltyCrossHypotheses::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:PenaltyCrossHypotheses:patches", "drawingOnImage");

  COMPLEX_IMAGE(PenaltyCrossPatches)
  {
    DECLARE_LOCAL_DEBUG_IMAGE(PenaltyCrossPatches);
    int n = std::max<int>(1, static_cast<int>(std::ceil(std::sqrt(static_cast<float>(penaltyCrosses.size() + penaltyCrossesUpper.size())))));
    INIT_DEBUG_IMAGE_BLACK(PenaltyCrossPatches, n * PENALTY_CROSS_SIZE + n, n * PENALTY_CROSS_SIZE + n);
    for (int m = 0; m < static_cast<int>(penaltyCrosses.size() + penaltyCrossesUpper.size()); m++)
    {
      int column = m / n;
      int row = m % n;
      PenaltyCross pc;
      if (m >= static_cast<int>(penaltyCrosses.size()))
        pc = penaltyCrossesUpper.at(m - penaltyCrosses.size());
      else
        pc = penaltyCrosses.at(m);

      if (!pc.patch.empty())
      {
        ASSERT(pc.patch.size() == (PENALTY_CROSS_SIZE * PENALTY_CROSS_SIZE * 3));
        for (int y = 0; y < PENALTY_CROSS_SIZE; y++)
        {
          for (int x = 0; x < PENALTY_CROSS_SIZE; x++)
          {
            float r = pc.patch[y * PENALTY_CROSS_SIZE * 3 + x * 3 + 0] * 255.f;
            float g = pc.patch[y * PENALTY_CROSS_SIZE * 3 + x * 3 + 1] * 255.f;
            float b = pc.patch[y * PENALTY_CROSS_SIZE * 3 + x * 3 + 2] * 255.f;
            DEBUG_IMAGE_SET_PIXEL_RGB(
                PenaltyCrossPatches, x + row * PENALTY_CROSS_SIZE + row, y + column * PENALTY_CROSS_SIZE + column, static_cast<unsigned char>(r), static_cast<unsigned char>(g), static_cast<unsigned char>(b));
          }
        }

        int x = static_cast<int>(PENALTY_CROSS_SIZE / 2.f - 6 + 0.5f) + row * PENALTY_CROSS_SIZE + row;
        int y = static_cast<int>(PENALTY_CROSS_SIZE - 3 + 0.5f) + column * PENALTY_CROSS_SIZE + column;
        DRAWTEXT("representation:PenaltyCrossHypotheses:patches", x, y, 3, ColorRGBA::white, pc.validity * 100.f);
      }
    }
    SEND_DEBUG_IMAGE(PenaltyCrossPatches);
  }
}
