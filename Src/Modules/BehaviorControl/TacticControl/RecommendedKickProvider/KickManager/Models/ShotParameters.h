#pragma once

#include <functional>
#include <optional>
#include <utility>
#include <vector>
#include <string>
#include "Tools/Debugging/Debugging.h"
#include "Platform/BHAssert.h"

class ExecutableShot;
class ShotParameters
{

public:
  class KickParameters
  {
  public:
    KickParameters() : generalParameter(0.f), kickBlindParameter(0.f) {}

    KickParameters(const float generalParameter, const float kickBlindParameter) : generalParameter(generalParameter), kickBlindParameter(kickBlindParameter)
    {
      ASSERT(generalParameter >= 0.f);
      ASSERT(kickBlindParameter <= 0.f);
    }

    float generalParameter;
    float kickBlindParameter;
  };

  class TargetParameters
  {
  public:
    TargetParameters()
        : sidesHeatParameter(0.f), goalsHeatParameter(0.f), intoGoalKickOutsideParameter(0.f), intoKickInOutsideParameter(0.f), intoCornerKickOutsideParameter(0.f),
          intoDirectionOfGoalsHeatParameter(0.f), intoOpponentsGoalParameter(0.f), intoOwnGoalParameter(0.f), selfHeatParameter(0.f), selfToGoalHeatParameter(0.f),
          teammatesHeatParameter(0.f), teammatesToGoalHeatParameter(0.f), opponentsHeatParameter(0.f), opponentsToGoalHeatParameter(0.f), crowdedParameter(0.f)
    {
    }

    TargetParameters(const float sidesHeatParameter,
        const float goalsHeatParameter,
        const float intoGoalKickOutsideParameter,
        const float intoKickInOutsideParameter,
        const float intoCornerKickOutsideParameter,
        const float intoDirectionOfGoalsHeatParameter,
        const float intoOpponentsGoalParameter,
        const float intoOwnGoalParameter,
        const float selfHeatParameter,
        const float selfToGoalHeatParameter,
        const float teammatesHeatParameter,
        const float teammatesToGoalHeatParameter,
        const float opponentsHeatParameter,
        const float opponentsToGoalHeatParameter,
        const float crowdedParameter)
        : sidesHeatParameter(sidesHeatParameter), goalsHeatParameter(goalsHeatParameter), intoGoalKickOutsideParameter(intoGoalKickOutsideParameter),
          intoKickInOutsideParameter(intoKickInOutsideParameter), intoCornerKickOutsideParameter(intoCornerKickOutsideParameter),
          intoDirectionOfGoalsHeatParameter(intoDirectionOfGoalsHeatParameter), intoOpponentsGoalParameter(intoOpponentsGoalParameter), intoOwnGoalParameter(intoOwnGoalParameter),
          selfHeatParameter(selfHeatParameter), selfToGoalHeatParameter(selfToGoalHeatParameter), teammatesHeatParameter(teammatesHeatParameter),
          teammatesToGoalHeatParameter(teammatesToGoalHeatParameter), opponentsHeatParameter(opponentsHeatParameter), opponentsToGoalHeatParameter(opponentsToGoalHeatParameter),
          crowdedParameter(crowdedParameter)
    {
      // sidesHeatParameter depends on situation
      ASSERT(goalsHeatParameter >= 0);
      ASSERT(intoDirectionOfGoalsHeatParameter >= 0);
      ASSERT(intoGoalKickOutsideParameter <= 0);
      ASSERT(intoKickInOutsideParameter <= 0);
      ASSERT(intoCornerKickOutsideParameter <= 0);
      ASSERT(intoOpponentsGoalParameter >= 0);
      ASSERT(intoOwnGoalParameter <= 0);
      ASSERT(selfHeatParameter >= 0);
      ASSERT(selfToGoalHeatParameter >= 0);
      ASSERT(teammatesHeatParameter >= 0);
      ASSERT(teammatesToGoalHeatParameter >= 0);
      ASSERT(opponentsHeatParameter <= 0);
      ASSERT(opponentsToGoalHeatParameter <= 0);
      ASSERT(crowdedParameter <= 0);

      const float opponentParameterSum = opponentsHeatParameter + opponentsToGoalHeatParameter;
      const std::string opponentMessage = " than the opponentsHeatParameter and opponentsToGoalHeatParameter combined.";
      if (intoOwnGoalParameter > opponentParameterSum)
      {
        OUTPUT_WARNING("The intoOwnGoalParameter is higher " + opponentMessage);
      }
      const std::string outsideSmallerOpponentMessage = "  is lower " + opponentMessage + " If the robot kicks into the outside, the opponent gets full control over the ball.";
      if (intoGoalKickOutsideParameter < opponentParameterSum)
      {
        OUTPUT_WARNING("The intoGoalKickOutsideParameter " + outsideSmallerOpponentMessage);
      }
      if (intoKickInOutsideParameter < opponentParameterSum)
      {
        OUTPUT_WARNING("The intoKickInOutsideParameter " + outsideSmallerOpponentMessage);
      }
      if (intoCornerKickOutsideParameter < opponentParameterSum)
      {
        OUTPUT_WARNING("The intoCornerKickOutsideParameter " + outsideSmallerOpponentMessage);
      }

      const float allPositiveOnFieldTargetParameterSum = goalsHeatParameter + intoDirectionOfGoalsHeatParameter + selfHeatParameter + selfToGoalHeatParameter
          + teammatesHeatParameter + teammatesToGoalHeatParameter;
      if (allPositiveOnFieldTargetParameterSum > intoOpponentsGoalParameter)
      {
        OUTPUT_WARNING("The intoOpponentsGoalParameter is lower than the highest possible score for targets on the field!");
      }
    }

    float sidesHeatParameter;
    float goalsHeatParameter;

    float intoGoalKickOutsideParameter;
    float intoKickInOutsideParameter;
    float intoCornerKickOutsideParameter;
    float intoDirectionOfGoalsHeatParameter;
    float intoOpponentsGoalParameter;
    float intoOwnGoalParameter;

    float selfHeatParameter;
    float selfToGoalHeatParameter;

    float teammatesHeatParameter;
    float teammatesToGoalHeatParameter;

    float opponentsHeatParameter;
    float opponentsToGoalHeatParameter;

    float crowdedParameter;
  };

  class PoseParameters
  {
  public:
    PoseParameters() : dontRuntIntoParameter(0.f), blockDefensiveConeParameter(0.f), blockOpponentParameter(0.f) {}

    PoseParameters(const float dontRuntIntoParameter, const float blockDefensiveConeParameter, const float blockOpponentParameter)
        : dontRuntIntoParameter(dontRuntIntoParameter), blockDefensiveConeParameter(blockDefensiveConeParameter), blockOpponentParameter(blockOpponentParameter)
    {
      ASSERT(dontRuntIntoParameter >= 0.f);
      ASSERT(blockDefensiveConeParameter >= 0.f);
      ASSERT(blockOpponentParameter >= 0.f);
    }

    float dontRuntIntoParameter;
    float blockDefensiveConeParameter;
    float blockOpponentParameter;
  };

  ShotParameters() : kickParameters({}), targetParameters({}), poseParameters({}) {}

  ShotParameters(const KickParameters kickParameters, const TargetParameters targetParameters, const PoseParameters poseParameters)
      : kickParameters(kickParameters), targetParameters(targetParameters), poseParameters(poseParameters)
  {
  }

  ShotParameters& addCustomScoreFunction(std::function<float(ExecutableShot&)>& customScoreFunction)
  {
    customScoreFunctions.push_back(customScoreFunction);
    return *this;
  }

  KickParameters kickParameters;
  TargetParameters targetParameters;
  PoseParameters poseParameters;

  std::vector<std::function<float(ExecutableShot&)>> customScoreFunctions = {};
};
