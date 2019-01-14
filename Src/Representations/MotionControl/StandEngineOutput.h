/**
 * @file Representations/MotionControl/StandEngineOutput.h
 * This file declares a struct that represents the output of the stand engine module.
 * @author <A href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</A>
 */

#pragma once

#include "Representations/Infrastructure/JointRequest.h"

/**
 * @struct StandEngineOutput
 * A struct that represents the output of the stand engine module.
 */
STREAMABLE_WITH_BASE(StandEngineOutput, JointRequest,
{ ,
  (float)(0.f) stiffnessTransition,
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
});
