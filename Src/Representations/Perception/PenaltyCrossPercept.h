/**
* @file PenaltyCrossPercept.h
* Declaration of a class that represents the penalty cross found in an image.
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#ifndef __PenaltyCrossPercept_h_
#define __PenaltyCrossPercept_h_

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>

/**
* @class PenaltyCrossPercept
* A class that represents the penalty cross in an image.
*/
STREAMABLE(PenaltyCrossPercept,
  ENUM(DetectionType,
      scanlines,
      yolo
    );

  DetectionType detectionType = DetectionType::scanlines;

  /** Reset the CenterCirclePercept */
  void reset()
  {
    penaltyCrossWasSeen = false;
    detectionType = PenaltyCrossPercept::scanlines;
  }

  /**
  * The method draws the percept.
  */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:PenaltyCrossPercept:Image:Lower", "drawingOnImage");
    DECLARE_DEBUG_DRAWING("representation:PenaltyCrossPercept:Image:Upper", "drawingOnImage");
    DECLARE_DEBUG_DRAWING("representation:PenaltyCrossPercept:Field", "drawingOnField");
    if (penaltyCrossWasSeen)
    {
      if (fromUpper)
        if (detectionType == PenaltyCrossPercept::yolo)
          CROSS("representation:PenaltyCrossPercept:Image:Upper", pointInImage.x(), pointInImage.y(), 15, 3, Drawings::solidPen, ColorRGBA::blue);
        else
          CROSS("representation:PenaltyCrossPercept:Image:Upper", pointInImage.x(), pointInImage.y(), 15, 3, Drawings::solidPen, ColorRGBA::red);
      else
        if (detectionType == PenaltyCrossPercept::yolo)
          CROSS("representation:PenaltyCrossPercept:Image:Lower", pointInImage.x(), pointInImage.y(), 15, 3, Drawings::solidPen, ColorRGBA::blue);
        else
          CROSS("representation:PenaltyCrossPercept:Image:Lower", pointInImage.x(), pointInImage.y(), 15, 3, Drawings::solidPen, ColorRGBA::red);
        
      CROSS("representation:PenaltyCrossPercept:Field", 
        pointOnField.x(), pointOnField.y(), 50, 10, Drawings::solidPen, ColorRGBA::blue);
    }
  }
  ,
  (Vector2i)(Vector2i::Zero()) pointInImage,
  (Vector2i)(Vector2i::Zero()) pointOnField,
  (bool)(false) penaltyCrossWasSeen,
  (bool)(false) fromUpper
);

#endif //__PenaltyCrossPercept_h_
