#pragma once

#include <Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/ShotParameters.h>
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/CurrentKickManager.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/Objectives/RecommendedKickObjective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/RoleProvider.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/BehaviorLogger.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ObjectivesManager/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ObjectivesManager/ObjectivesManager.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/PositionUtils.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/RecommendedKick/DirectionInfo.h"
#include "Representations/Modeling/RecommendedKick/PositionInfo/PositionInfo.h"
#include "Representations/Modeling/RecommendedKick/RecommendedKick.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Module/Module.h"
#include "Tools/Settings.h"

MODULE(RecommendedKickProvider,
  REQUIRES(BallSymbols),
  REQUIRES(DirectionInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(PositionInfo),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(TacticSymbols),
  REQUIRES(TeammateData),

  USES(PositioningAndKickSymbols),

  PROVIDES(RecommendedKick),

  LOADS_PARAMETERS(,
    (std::vector<std::string>) kickNames,

    // parameters //////////////////////////////////
    // kick parameters
    (float)(0.f) generalParameter,
    (float)(0.f) kickBlindParameter,
    // target parameters
    (float)(0.f) sidesHeatParameter,
    (float)(0.f) goalsHeatParameter,
    (float)(0.f) intoDirectionOfGoalsHeatParameter,
    (float)(0.f) intoGoalKickOutsideParameter,
    (float)(0.f) intoKickInOutsideParameter,
    (float)(0.f) intoCornerKickOutsideParameter,
    (float)(0.f) intoOpponentsGoalParameter,
    (float)(0.f) intoOwnGoalParameter,
    (float)(0.f) selfHeatParameter,
    (float)(0.f) selfToGoalHeatParameter,
    (float)(0.f) teammatesHeatParameter,
    (float)(0.f) teammatesToGoalHeatParameter,
    (float)(0.f) opponentsHeatParameter,
    (float)(0.f) opponentsToGoalHeatParameter,
    (float)(0.f) crowdedParameter,
    // pose parameters
    (float)(0.f) dontRuntIntoParameter,
    (float)(0.f) blockDefensiveConeParameter,
    (float)(0.f) blockOpponentParameter
  )
);

class RecommendedKickProvider : public RecommendedKickProviderBase
{

public:
  RecommendedKickProvider();
  void update(RecommendedKick& recommendedKick);

private:
  int skips = 0;
  std::vector<std::unique_ptr<Kick>> kicks;
  CurrentKickManager currentKickManager;
  ShotParameters configFileParameters;

  void noRecommendation(RecommendedKick& theRecommendedKick);
  void recommend(RecommendedKick& theRecommendedKick);
};
