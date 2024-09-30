#pragma once

#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/Constants.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/ShotParameters.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/ExecutableShot.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/SelectablePose.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/SelectableShot.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/SelectableTarget.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/HysteresisUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/PathUtils.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RecommendedKick/DirectionInfo.h"
#include "Representations/Modeling/RecommendedKick/PositionInfo/PositionInfo.h"
#include "Representations/Modeling/RobotMap.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include <functional>
#include <optional>
#include <utility>
#include <vector>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Hysteresis.h>

class ScoreFunctions
{

public:
  class ProbabilityCone
  {

  public:
    ProbabilityCone(const Kick* kick, const Angle stepAngle)
        : stepAngle(stepAngle), stepsToEachDirection(calcStepsToEachDirection(kick, stepAngle)), divisor(calcDivisor(stepsToEachDirection))
    {
    }

    const Angle stepAngle;
    const int stepsToEachDirection;
    const float divisor;

  private:
    static int calcStepsToEachDirection(const Kick* kick, const Angle stepAngle) { return (int)((kick->horizontalInaccuracy + 0.0001_deg) / stepAngle); }

    static Angle calcDivisor(const int stepsToEachDirection)
    {
      float divisor = (float)stepsToEachDirection + 1.f;
      for (int step = (int)stepsToEachDirection; step > 0; step--)
      {
        divisor += (float)(2 * step);
      }
      return divisor;
    }
  };

  static float getNoSuccessScore(const ShotParameters::TargetParameters& factors);

  static float scoreKick(const Kick* kick, ShotParameters::KickParameters kickFactors);

  static float scoreTarget(
      float ballGoalsHeat, const SelectableTarget& selectableTarget, ShotParameters::TargetParameters targetFactors, const FieldDimensions& theFieldDimensions, const PositionInfo& thePositionInfo);
  static float getIntoDirectionOfGoalsHeatValue(const float ballGoalsHeat, const float goalsHeat, const ShotParameters::TargetParameters& targetFactors);

  static float scorePose(const SelectablePose& selectablePose, ShotParameters::PoseParameters poseFactors, const TacticSymbols& theTacticSymbols);

  static float getDontRuntIntoScore(const SelectablePose& selectablePose, float dontRuntIntoFactor, const TacticSymbols& theTacticSymbols);

  static float getBlockDefensiveConeScore(const SelectablePose& selectablePose, float blockDefensiveConeFactor, const TacticSymbols& theTacticSymbols);

  static float getBlockOpponentScore(const SelectablePose& selectablePose, float blockOpponentFactor, const TacticSymbols& theTacticSymbols);

  static float scoreSelectableShot(const SelectableShot& selectableShot, const ProbabilityCone& probabilityCone);

  static float scoreExecutableShot(const ExecutableShot& executableShot, const float successProbability, const float noSuccessScore);

  static float applyHysteresis(float score, float poseTime, const Hysteresis& hysteresis);
};
