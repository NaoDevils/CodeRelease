/**
* @file CLIPFieldLinesPercept.h
* Declaration of a class that represents the field lines found in an image.
* @author <a href="mailto:stefan.czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
*/

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Eigen.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>

/**
* @class CLIPFieldLinesPercept
* A class that represents the lines found in an image.
*/
STREAMABLE(CLIPFieldLinesPercept,
  STREAMABLE(FieldLine,,
    (Vector2i) startInImage, /**< The start point in image coordinates. */
    (Vector2i) endInImage, /**< The end point in image coordinates. */
    (Vector2f) startOnField, /**< The start point in field coordinates. */
    (Vector2f) endOnField, /**< The end point in field coordinates. */
    (float)(0) lineWidthStart, /**< orthogonal to line */
    (float)(0) lineWidthEnd, /**< orthogonal to line */
    (float) validity, /**< validity of line */
    (bool) isPlausible, /**< determines whether line is plausible in percept. */
    (bool) fromUpper /**< Take a guess. */
  );

  CLIPFieldLinesPercept() {
  lines.reserve(50);}
  void reset()
  {
  lines.clear();
  }
  /**
  * The method draws the percept.
  */
  void draw() const
  {
    DEBUG_DRAWING("representation:CLIPFieldLinesPercept:Field", "drawingOnField")
    {
      for (std::vector<FieldLine>::const_iterator i = lines.begin(); i != lines.end(); ++i)
      {
        LINE("representation:CLIPFieldLinesPercept:Field", i->startOnField.x(), i->startOnField.y(),
          i->endOnField.x(), i->endOnField.y(), 5, Drawings::solidPen, ColorRGBA(0,255,255));
}
}
DEBUG_DRAWING("representation:CLIPFieldLinesPercept:Image:Lower", "drawingOnImage")
{
  for (std::vector<FieldLine>::const_reverse_iterator i = lines.rbegin(); i != lines.rend(); ++i)
  {
    if (!i->fromUpper)
      LINE("representation:CLIPFieldLinesPercept:Image:Lower", i->startInImage.x(), i->startInImage.y(), i->endInImage.x(), i->endInImage.y(), 2, Drawings::solidPen, ColorRGBA(0, 255, 255));
  }
}
DEBUG_DRAWING("representation:CLIPFieldLinesPercept:Image:Upper", "drawingOnImage")
{
  for (std::vector<FieldLine>::const_reverse_iterator i = lines.rbegin(); i != lines.rend(); ++i)
  {
    if (i->fromUpper)
      LINE("representation:CLIPFieldLinesPercept:Image:Upper", i->startInImage.x(), i->startInImage.y(), i->endInImage.x(), i->endInImage.y(), 2, Drawings::solidPen, ColorRGBA(0, 255, 255));
  }
}
},
  (std::vector<FieldLine>) lines /**< The lines in image and relative field coordinates. */
);
