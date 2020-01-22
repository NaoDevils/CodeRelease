/**
 * @file BallPercept.cpp
 *
 * Very simple representation of a seen ball
 *
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#include "BallPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void BallPercept::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:BallPercept:Image:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BallPercept:Image:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BallPercept:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:BallPercept", "robot");
  TRANSLATE3D("representation:BallPercept", 0, 0, -230);
  if(status == seen)
  {
    if (fromUpper)
      if (detectionType == BallPercept::yoloHypothesis)
      CIRCLE("representation:BallPercept:Image:Upper", positionInImage.x(), positionInImage.y(), radiusInImage, 1, // pen width
             Drawings::solidPen,  ColorRGBA::black, Drawings::solidBrush, ColorRGBA(0, 0, 255, 100));
      else if (detectionType == BallPercept::yoloFallback)
        CIRCLE("representation:BallPercept:Image:Upper", positionInImage.x(), positionInImage.y(), radiusInImage, 1, // pen width
               Drawings::solidPen,  ColorRGBA::black, Drawings::solidBrush, ColorRGBA(255, 0, 0, 100));
      else if (detectionType == BallPercept::yoloOnly)
        CIRCLE("representation:BallPercept:Image:Upper", positionInImage.x(), positionInImage.y(), radiusInImage, 1, // pen width
               Drawings::solidPen,  ColorRGBA::black, Drawings::solidBrush, ColorRGBA(0, 255, 0, 100));
      else
      CIRCLE("representation:BallPercept:Image:Upper", positionInImage.x(), positionInImage.y(), radiusInImage, 1, // pen width
             Drawings::solidPen,  ColorRGBA::black, Drawings::solidBrush, ColorRGBA(255, 128, 64, 100));
    else
      if (detectionType == BallPercept::yoloHypothesis)
        CIRCLE("representation:BallPercept:Image:Lower", positionInImage.x(), positionInImage.y(), radiusInImage, 1, // pen width
              Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA(0, 0, 255, 100));
      else if (detectionType == BallPercept::yoloFallback)
        CIRCLE("representation:BallPercept:Image:Lower", positionInImage.x(), positionInImage.y(), radiusInImage, 1, // pen width
               Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA(255, 0, 0, 100));
      else if (detectionType == BallPercept::yoloOnly)
        CIRCLE("representation:BallPercept:Image:Lower", positionInImage.x(), positionInImage.y(), radiusInImage, 1, // pen width
               Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA(0, 255, 0, 100));
      else
        CIRCLE("representation:BallPercept:Image:Lower", positionInImage.x(), positionInImage.y(), radiusInImage, 1, // pen width
              Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA(255, 128, 64, 100));

    CIRCLE("representation:BallPercept:Field", relativePositionOnField.x(), relativePositionOnField.y(), radiusOnField/2, 0, // pen width
           Drawings::solidPen, ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);
    CIRCLE("representation:BallPercept:Field", relativePositionOnField.x(), relativePositionOnField.y(), radiusOnField/4, 0, // pen width
      Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA::black);
    SPHERE3D("representation:BallPercept", relativePositionOnField.x(), relativePositionOnField.y(), radiusOnField, radiusOnField, ColorRGBA::orange);
  }
  else if(status != notSeen)
  {
    if (fromUpper)
      CROSS("representation:BallPercept:Image:Upper", positionInImage.x(), positionInImage.y(), 1, 1, Drawings::solidPen, ColorRGBA::black);
    else
      CROSS("representation:BallPercept:Image:Lower", positionInImage.x(), positionInImage.y(), 1, 1, Drawings::solidPen, ColorRGBA::black);
    if (fromUpper)
      DRAWTEXT("representation:BallPercept:Image:Upper", positionInImage.x() + 3, positionInImage.y() + 2, 10, ColorRGBA::black, BallPercept::getName(status));
    else
      DRAWTEXT("representation:BallPercept:Image:Lower", positionInImage.x() + 3, positionInImage.y() + 2, 10, ColorRGBA::black, BallPercept::getName(status));
  }
}
