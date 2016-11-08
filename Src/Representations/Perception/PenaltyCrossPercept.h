/**
* @file PenaltyCrossPercept.h
* Declaration of a class that represents the penalty cross found in an image.
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#ifndef __PenaltyCrossPercept_h_
#define __PenaltyCrossPercept_h_

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>

/**
* @class PenaltyCrossPercept
* A class that represents the penalty cross in an image.
*/
class PenaltyCrossPercept : public Streamable
{
private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(pointInImage);
    STREAM(pointOnField);
    STREAM(penaltyCrossWasSeen);
    STREAM(fromUpper);
    STREAM_REGISTER_FINISH;
  }

public:
  
  Vector2i pointInImage;
  Vector2i pointOnField;
  bool penaltyCrossWasSeen;
  bool fromUpper;

  /**
  * Default constructor.
  */
  PenaltyCrossPercept():penaltyCrossWasSeen(false),fromUpper(false) {}

  /** Reset the CenterCirclePercept */
  void reset()
  {
    penaltyCrossWasSeen = false;
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
        CROSS("representation:PenaltyCrossPercept:Image:Upper",
        pointInImage.x(), pointInImage.y(), 
        15, 3, Drawings::solidPen, ColorRGBA::blue);
      else
        CROSS("representation:PenaltyCrossPercept:Image:Lower",
        pointInImage.x(), pointInImage.y(), 
        15, 3, Drawings::solidPen, ColorRGBA::blue);

      CROSS("representation:PenaltyCrossPercept:Field", 
        pointOnField.x(), pointOnField.y(), 50, 10, Drawings::solidPen, ColorRGBA::blue);
    }
  }
};

#endif //__PenaltyCrossPercept_h_
