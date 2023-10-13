#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Constants.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Selectables/KickPlan.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Selectables/SelectablePose.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Selectables/SelectableTarget.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/HysteresisUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/PathUtils.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/HeatMapCollection.h"
#include "Representations/Modeling/KickWheel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include <functional>
#include <optional>
#include <utility>
#include <vector>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Factors.h>

class ScoreFunctions
{

public:
  static void scoreKickPlan(
      KickPlan& kickPlan, const Factors& factors, const FieldDimensions& theFieldDimensions, const HeatMapCollection& theHeatMapCollection, const RobotMap& theRobotMap, const TacticSymbols& theTacticSymbols);

  static float scoreTarget(const SelectableTarget& selectableTarget,
      const float widthFactor,
      const float sidesHeatFactor,
      const float goalsHeatFactor,
      const float teammatesKickHeatFactor,
      const float teammatesGoalKickHeatFactor,
      const float opponentsKickHeatFactor,
      const float opponentsGoalKickHeatFactor,
      const FieldDimensions& theFieldDimensions,
      const HeatMapCollection& theHeatMapCollection,
      const RobotMap& theRobotMap);

  static float getWidthScore(const Vector2f& ballPosition, const Vector2f& targetPosition, const float widthFactor, const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap);

  static float scorePose(
      const SelectablePose& selectablePose, const float dontRuntIntoFactor, const float blockDefensiveCone, const float blockOpponentFactor, const float timeFactor, const TacticSymbols& theTacticSymbols);

  static float getDontRuntIntoScore(const SelectablePose& selectablePose, const float dontRuntIntoFactor, const TacticSymbols& theTacticSymbols);

  static float getBlockDefensiveConeScore(const SelectablePose& selectablePose, const float blockDefensiveConeFactor, const TacticSymbols& theTacticSymbols);

  static float getBlockOpponentScore(const SelectablePose& selectablePose, const float blockOpponentFactor, const TacticSymbols& theTacticSymbols);

  static float getTimeScore(const SelectablePose& selectablePose, const float timeFactor);

  static float scoreKick(const Kick* kick, float timeFactor, float inaccuracyFactor);

  static float applyHysteresis(const float score, const float distanceToKick);
};
