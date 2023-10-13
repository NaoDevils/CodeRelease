/**
 * @file JointRequestProvider.h
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 */

#pragma once

#include "Tools/Module/Module.h"

// Include all referenced representations here.
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/WalkCalibration.h"


MODULE(JointRequestProvider,
  REQUIRES(RawJointRequest),
  REQUIRES(WalkCalibration),
  PROVIDES(JointRequest)
);

class JointRequestProvider : public JointRequestProviderBase
{
private:
  // Add an update method for each PROVIDES representation here.
  void update(JointRequest& jointRequest);
};
