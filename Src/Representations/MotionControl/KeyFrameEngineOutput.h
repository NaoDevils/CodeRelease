/**
 * @file Representations/MotionControl/KeyFrameEngineOutput.h
 * This file declares a struct that represents the output of the KeyFrameEngine module.
 * @author <A href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</A>
 */

#pragma once

#include "Representations/Infrastructure/JointRequest.h"

/**
 * @struct KeyFrameEngineOutput
 * A struct that represents the output of the stand engine module.
 */
STREAMABLE_WITH_BASE(KeyFrameEngineOutput, JointRequest,
{ ,
  (float)(0.f) stiffnessTransition,
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
});
