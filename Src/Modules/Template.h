/**
 * @file Template.h
 * This file declares a module that ...
 * @author <a href="mailto:max.mustermann@tu-dortmund.de">Max Mustermann</a>
 */

#pragma once

#include "Tools/Module/Module.h"

// Include all referenced representations here.
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

// Module name of your choice.
// This equals the name of the .cpp and .h files.
// Change every occurence of "Template" to the name of your module in both files.
MODULE(Template,
  REQUIRES(FrameInfo),     // Representation that has to be updated before. Can be accessed using theFrameInfo.<property> .
  //USES(...),             // Representation (copy) that was updated in the last frame. (only to resolve cyclic dependencies)
  PROVIDES(MotionRequest), // Representation that will be filled by this module's update method.

  LOADS_PARAMETERS(,         // Loads a configuration file that has the same name as the module defined, but starts with lowercase letters.
                             // All attributes are streamed and can be accessed by requesting 'get parameters:Template'.
    (float)(0.42f) ratio,    // Has a parameter named ratio of type float. By default, it has the value 0.42f.
    (int)(42) size
  )
);

// Your class inherits from the <ModuleName>Base class generated by the MODULE() macro above.
class Template : public TemplateBase
{
private:
  // Add an update method for each PROVIDES representation here.
  void update(MotionRequest& motionRequest);
};