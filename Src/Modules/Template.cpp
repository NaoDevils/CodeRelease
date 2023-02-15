#include "Template.h"

// Helpful for debugging
#include "Tools/Debugging/DebugDrawings.h"


// This registers your module and specifies a category that assigns it to
// an appropriate cycle (motion or cognition). (see Category enum in Module.h)
MAKE_MODULE(Template, behaviorControl);


void Template::update(MotionRequest& motionRequest)
{
  // Declare debug drawings in each frame:
  // Field drawing in worldState window. Can be enabled using 'vfd worldState module:Template:myfielddrawing'.
  DECLARE_DEBUG_DRAWING("module:Template:myfielddrawing", "drawingOnField");
  // Image drawing in camera window. Can be enabled using 'vid [upper/lower] module:Template:myimagedrawing'.
  DECLARE_DEBUG_DRAWING("module:Template:myimagedrawing", "drawingOnImage");

  // Get current timestamp.
  unsigned timestamp = theFrameInfo.time;
  // Fill new representation.
  motionRequest.motion = MotionRequest::Motion::stand;

  // Draw a cross on the field.
  CROSS("module:Template:myfielddrawing", 123, 234, 30, 2, Drawings::solidPen, ColorRGBA::red);
  // Draw a line on the camera image.
  LINE("module:Template:myimagedrawing", 20, 30, 180, 200, 2, Drawings::solidPen, ColorRGBA::blue);
}
