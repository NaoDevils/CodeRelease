/**
 * @file VisualRefereeBehavior.h
 * 
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/VisualRefereeBehaviorSymbols.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/BehaviorControl/KeySymbols.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Perception/CameraMatrix.h"

#include "Tools/Module/ModuleManager.h"
#include <optional>

MODULE(VisualRefereeBehavior,
  REQUIRES(CameraCalibration),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(TorsoMatrix),
  REQUIRES(KeySymbols),
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions),
  PROVIDES(VisualRefereeBehaviorSymbols),
  LOADS_PARAMETERS(,
    (Vector2f)({1000.f, 2500.f}) refereeSize,
    (float)(750.f) robotPoseTransThreshold,
    (Angle)(45_deg) robotPoseRotThreshold,
    (Angle)(-pi_4) refereeTopInImage,
    (unsigned)(5000) localizeTime,
    (unsigned)(30000) localizeTimeout,
    (unsigned)(2000) lookTime,
    (float)(0.8f) minPoseValidity
  )
);

class VisualRefereeBehavior : public VisualRefereeBehaviorBase
{
  void update(VisualRefereeBehaviorSymbols& visualRefereeBehaviorSymbols);

private:
  std::optional<ModuleManager::Configuration> lastModuleConfig;
  Pose2f lastRobotPose;
};
