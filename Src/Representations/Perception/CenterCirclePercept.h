/**
* @file CLIPCenterCirclePercept.h
* Declaration of a class that represents the center circle found in an image.
* @author <a href="mailto:stefan.czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
*/
#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include <vector>

/**
* @class CenterCirclePercept
* A class that represents the lines found in an image.
*/
STREAMABLE(CLIPCenterCirclePercept,
  STREAMABLE(CenterCircle,,
    (Vector2i) locationOnField,
    (Vector2i) locationInImage,
    (float) orientation, // angle of the center line (if detected) [in field coordinates relative to the robot]
    (bool) orientationKnown
  );

  float getDistanceToFieldLinesPercept(const CLIPFieldLinesPercept::FieldLine &line) const
  {
    Vector2f direction = line.endOnField-line.startOnField;
    if(direction.x() == 0 && direction.y() == 0)
      return (centerCircle.locationOnField.cast<float>() - line.endOnField).norm();

    Vector2f pos = centerCircle.locationOnField.cast<float>();
    Vector2f normal;
    normal.x() = direction.y();
    normal.y() = -direction.x();
    normal.normalize();

    float c = normal.dot(line.startOnField);

    return normal.dot(pos) - c;
  }


  /**
  * The method draws the percept.
  */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:CLIPCenterCirclePercept:Image:Lower", "drawingOnImage");
    DECLARE_DEBUG_DRAWING("representation:CLIPCenterCirclePercept:Image:Upper", "drawingOnImage");
    DECLARE_DEBUG_DRAWING("representation:CLIPCenterCirclePercept:Field", "drawingOnField");
    if (centerCircleWasSeen)
    {
      const float circleRadius = 750;

      if (fromUpper)
        CIRCLE("representation:CLIPCenterCirclePercept:Image:Upper",
          centerCircle.locationInImage.x(), centerCircle.locationInImage.y(),
          6, 2, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA());
      else
        CIRCLE("representation:CLIPCenterCirclePercept:Image:Lower",
          centerCircle.locationInImage.x(), centerCircle.locationInImage.y(),
          6, 2, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA());

      CIRCLE("representation:CLIPCenterCirclePercept:Field",
        centerCircle.locationOnField.x(), centerCircle.locationOnField.y(),
        circleRadius, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA());
      if (centerCircle.orientationKnown) // TODO: check this
      {
        LINE("representation:CLIPCenterCirclePercept:Field",
          (centerCircle.locationOnField.x() + 1.2*circleRadius*cos(centerCircle.orientation)),
          (centerCircle.locationOnField.y() + 1.2*circleRadius*sin(centerCircle.orientation)),
          (centerCircle.locationOnField.x() - 1.2*circleRadius*cos(centerCircle.orientation)),
          (centerCircle.locationOnField.y() - 1.2*circleRadius*sin(centerCircle.orientation)),
          30, Drawings::solidPen, ColorRGBA::blue);
      }
    }
  },

  (CenterCircle) centerCircle,
  (bool)(false) centerCircleWasSeen,
  (bool) fromUpper
);
