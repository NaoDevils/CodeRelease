/**
* @file CLIPPointsPercept.h
* Declaration of a class that represents the points found in an image.
* @author <a href="mailto:ingmar.schwarz@udo.edu">Ingmar Schwarz</a>
*/

#pragma once
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>

/**
* @class CLIPPointsPercept
* A class that represents the points found in an image.
*/
STREAMABLE(CLIPPointsPercept,
  STREAMABLE(Point,,
    (Vector2f) inImage, /**< The point in image coordinates. */
    (Vector2f) onField, /**< The point in field coordinates. */
    (int) scanLineNoH, /**< The horizontal scanLine number in image */
    (int) scanLineNoV, /**< The vertical scanLine number in image */
    (bool) isVertical, /**< is scanLine vertical? */
    (float) lineSizeInImage /**< The line size in the image. */
  );

  /** Reset the CLIPPointsPercept */
  void reset()
  {
  points.clear();
  pointsUpper.clear();
  }

  /**
  * The method draws the percept.
  */
  void draw() const
  {
    DEBUG_DRAWING("representation:CLIPPointsPercept:Field:Lower", "drawingOnField")
    {
      for (std::vector<Point>::const_iterator i = points.begin(); i != points.end(); ++i)
        LARGE_DOT("representation:CLIPPointsPercept:Field:Lower", i->onField.x(), i->onField.y(),
          ColorRGBA::black, ColorRGBA::white);
}
DEBUG_DRAWING("representation:CLIPPointsPercept:Field:Upper", "drawingOnField")
{
  for (std::vector<Point>::const_iterator i = pointsUpper.begin(); i != pointsUpper.end(); ++i)
    LARGE_DOT("representation:CLIPPointsPercept:Field:Upper", i->onField.x(), i->onField.y(), ColorRGBA::black, ColorRGBA::white);
}
DEBUG_DRAWING("representation:CLIPPointsPercept:Image:Lower", "drawingOnImage")
{
  for (std::vector<Point>::const_iterator i = points.begin(); i != points.end(); ++i)
    DOT("representation:CLIPPointsPercept:Image:Lower", i->inImage.x(), i->inImage.y(), i->isVertical ? ColorRGBA::red : ColorRGBA::blue, i->isVertical ? ColorRGBA::red : ColorRGBA::blue);
}
DEBUG_DRAWING("representation:CLIPPointsPercept:Image:Upper", "drawingOnImage")
{
  for (std::vector<Point>::const_iterator i = pointsUpper.begin(); i != pointsUpper.end(); ++i)
    DOT("representation:CLIPPointsPercept:Image:Upper", i->inImage.x(), i->inImage.y(), i->isVertical ? ColorRGBA::red : ColorRGBA::blue, i->isVertical ? ColorRGBA::red : ColorRGBA::blue);
}
},

  (std::vector<Point>) points, /**< The points found in lower image. */
  (std::vector<Point>) pointsUpper /**< The points found in upper image. */
);
