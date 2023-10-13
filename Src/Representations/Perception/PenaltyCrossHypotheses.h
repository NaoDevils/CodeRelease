/**
* @file PenaltyCrossHypotheses.h
* Declaration of a class that represents the penalty cross hypotheses found in an image.
* @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
*/

//PrePrePenaltyCrossHypothesesYolo

#pragma once

#include "Tools/Enum.h"
#include "Tools/ColorRGBA.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Infrastructure/Image.h"

#define PENALTY_CROSS_SIZE 32

STREAMABLE(PenaltyCross,
  PenaltyCross() = default;
  PenaltyCross(Vector2f pos) : positionInImage(pos) {};
  PenaltyCross(float x, float y) : positionInImage(x, y) {};

  bool operator < (const PenaltyCross& pc) const
  {
    if (positionInImage.y() == pc.positionInImage.y())
      return (validity > pc.validity);
    return positionInImage.y() > pc.positionInImage.y();
  },

  (Vector2f)(Vector2f::Zero()) positionInImage,             /**< The position of the penalty cross in the current image */
  (std::vector<float>) patch,
  (float)(0.f) validity,                                    /**< The validity of the penalty cross in range [0,1]. */
  (bool)(false) fromUpper                                   /**< True, if penalty cross was seen in upper image. */

);

STREAMABLE(PenaltyCrossHypotheses,
  PenaltyCrossHypotheses()
  {
    penaltyCrosses.reserve(10);
    penaltyCrossesUpper.reserve(10);
  };

  /** Draws the penaltyCross patches */
  void draw() const,

  (std::vector<PenaltyCross>) penaltyCrosses,
  (std::vector<PenaltyCross>) penaltyCrossesUpper
);

STREAMABLE_WITH_BASE(PrePenaltyCrossHypothesesYolo, PenaltyCrossHypotheses,
  /** The method draws all penalty cross hypotheses. */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:PenaltyCrossHypotheses:Image:Lower", "drawingOnImage");
    DECLARE_DEBUG_DRAWING("representation:PenaltyCrossHypotheses:Image:Upper", "drawingOnImage");
    char buffer[10];
    COMPLEX_DRAWING("representation:PenaltyCrossHypotheses:Image:Lower")
    {
      for (std::vector<PenaltyCross>::const_iterator i = penaltyCrosses.begin(); i != penaltyCrosses.end(); ++i)
      {
        CROSS("representation:PenaltyCrossHypotheses:Image:Lower",
          i->positionInImage.x(),
          i->positionInImage.y(),
          15, 3, Drawings::solidPen, ColorRGBA::orange);
        sprintf(buffer, "%.1f", i->validity * 100.f);
        DRAWTEXT("representation:PenaltyCrossHypotheses:Image:Lower", i->positionInImage.x() - 15, i->positionInImage.y() - 15, 13, ColorRGBA::magenta, buffer << "%");
      }
    }

    COMPLEX_DRAWING("representation:PenaltyCrossHypotheses:Image:Upper")
    {
      for (std::vector<PenaltyCross>::const_iterator i = penaltyCrossesUpper.begin(); i != penaltyCrossesUpper.end(); ++i)
      {
        CROSS("representation:PenaltyCrossHypotheses:Image:Upper",
          i->positionInImage.x(),
          i->positionInImage.y(),
          15, 3, Drawings::solidPen, ColorRGBA::orange);
        sprintf(buffer, "%.1f", i->validity * 100.f);
        DRAWTEXT("representation:PenaltyCrossHypotheses:Image:Upper", i->positionInImage.x() - 15, i->positionInImage.y() - 15, 13, ColorRGBA::magenta, buffer << "%");
      }
    }
  }
);

STREAMABLE_WITH_BASE(PrePenaltyCrossHypothesesScanlines, PenaltyCrossHypotheses,
  /** The method draws all penalty cross hypotheses. */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:PenaltyCrossHypotheses:Image:Lower", "drawingOnImage");
    DECLARE_DEBUG_DRAWING("representation:PenaltyCrossHypotheses:Image:Upper", "drawingOnImage");
    char buffer[10];
    COMPLEX_DRAWING("representation:PenaltyCrossHypotheses:Image:Lower")
    {
      for (std::vector<PenaltyCross>::const_iterator i = penaltyCrosses.begin(); i != penaltyCrosses.end(); ++i)
      {
        CROSS("representation:PenaltyCrossHypotheses:Image:Lower",
          i->positionInImage.x(),
          i->positionInImage.y(),
          15, 3, Drawings::solidPen, ColorRGBA::orange);
        sprintf(buffer, "%.1f", i->validity * 100.f);
        DRAWTEXT("representation:PenaltyCrossHypotheses:Image:Lower", i->positionInImage.x() - 15, i->positionInImage.y() - 15, 13, ColorRGBA::orange, buffer << "%");
      }
    }

    COMPLEX_DRAWING("representation:PenaltyCrossHypotheses:Image:Upper")
    {
      for (std::vector<PenaltyCross>::const_iterator i = penaltyCrossesUpper.begin(); i != penaltyCrossesUpper.end(); ++i)
      {
        CROSS("representation:PenaltyCrossHypotheses:Image:Upper",
          i->positionInImage.x(),
          i->positionInImage.y(),
          15, 3, Drawings::solidPen, ColorRGBA::orange);
        sprintf(buffer, "%.1f", i->validity * 100.f);
        DRAWTEXT("representation:PenaltyCrossHypotheses:Image:Upper", i->positionInImage.x() - 15, i->positionInImage.y() - 15, 13, ColorRGBA::orange, buffer << "%");
      }
    }
  }
);
