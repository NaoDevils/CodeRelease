/**
* @file PenaltyCrossHypotheses.h
* Declaration of a class that represents the penalty cross hypotheses found in an image.
* @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
*/

//PenaltyCrossHypothesesYolo

#pragma once

#include "Tools/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/DebugDrawings.h"

STREAMABLE(PenaltyCross,
  PenaltyCross() = default;
  PenaltyCross(Vector2f pos) : positionInImage(pos) {};
  PenaltyCross(float x, float y) : positionInImage(x, y) {};

  bool operator < (const PenaltyCross& pc) const
  { return (validity > pc.validity);
  },

  (Vector2f)(Vector2f::Zero()) positionInImage,             /**< The position of the ball in the current image */
  (float)(0.f) validity,                                      /**< The validity of the ball percept in range [0,1]. */
  (bool)(false) fromUpper                                   /**< True, if ball was seen in upper image. Use with status. */
);

STREAMABLE(PenaltyCrossHypothesesYolo,
  PenaltyCrossHypothesesYolo()
  {
    penaltyCrosses.reserve(10);
    penaltyCrossesUpper.reserve(10);
  }

  /** The method draws all penalty cross hypotheses. */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:PenaltyCrossHypothesesYolo:Image:Lower", "drawingOnImage");
    DECLARE_DEBUG_DRAWING("representation:PenaltyCrossHypothesesYolo:Image:Upper", "drawingOnImage");
    char buffer[10];
    COMPLEX_DRAWING("representation:PenaltyCrossHypothesesYolo:Image:Lower")
    {
      for (std::vector<PenaltyCross>::const_iterator i = penaltyCrosses.begin(); i != penaltyCrosses.end(); ++i)
      {
        CROSS("representation:PenaltyCrossHypothesesYolo:Image:Lower",
          i->positionInImage.x(),
          i->positionInImage.y(),
          15, 3, Drawings::solidPen, ColorRGBA::orange);
        sprintf(buffer, "%.1f", i->validity * 100.f);
        DRAWTEXT("representation:PenaltyCrossHypothesesYolo:Image:Lower", i->positionInImage.x() - 15, i->positionInImage.y() - 15, 13, ColorRGBA::orange, buffer << "%");
      }
    }

    COMPLEX_DRAWING("representation:PenaltyCrossHypothesesYolo:Image:Upper")
    {
      for (std::vector<PenaltyCross>::const_iterator i = penaltyCrossesUpper.begin(); i != penaltyCrossesUpper.end(); ++i)
      {
        CROSS("representation:PenaltyCrossHypothesesYolo:Image:Upper",
          i->positionInImage.x(),
          i->positionInImage.y(),
          15, 3, Drawings::solidPen, ColorRGBA::orange);
        sprintf(buffer, "%.1f", i->validity * 100.f);
        DRAWTEXT("representation:PenaltyCrossHypothesesYolo:Image:Upper", i->positionInImage.x() - 15, i->positionInImage.y() - 15, 13, ColorRGBA::orange, buffer << "%");
      }
    }
  },

  (std::vector<PenaltyCross>) penaltyCrosses,
  (std::vector<PenaltyCross>) penaltyCrossesUpper
);