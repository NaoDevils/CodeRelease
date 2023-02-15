/**
* @file PSKickDetector.h
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Geometry.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/CLIPPointsPercept.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/BehaviorControl/PSGoalieTrigger.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/RingBufferWithSum.h"
#include <algorithm>

MODULE(PSKickDetector,
  REQUIRES(FallDownState),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(FieldColors),
  REQUIRES(FieldColorsUpper),
  REQUIRES(FieldDimensions),
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(BallSymbols),
  REQUIRES(BallModel),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  USES(BehaviorData),
  REQUIRES(RobotPose),
  PROVIDES(PSGoalieTrigger),
  DEFINES_PARAMETERS(,
    (int)(200) nonFCPixelThreshold,
    (float)(50) blockArea,
    (int)(3) filterWindowSize,
    (int)(100) ballPerceptDifference,
    (int)(25) velocityThreshold
  )
);

class PSKickDetector : public PSKickDetectorBase
{
public:
  /**
  * Default constructor.
  */
  PSKickDetector();

  /** Destructor */
  ~PSKickDetector();

private:
  void update(PSGoalieTrigger& thePSGoalieTrigger);
};
