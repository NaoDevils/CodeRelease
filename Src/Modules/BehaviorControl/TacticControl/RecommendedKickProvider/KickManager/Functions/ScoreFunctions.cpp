#include "ScoreFunctions.h"

#include "ProbabilityFunctions.h"

float ScoreFunctions::getNoSuccessScore(const ShotParameters::TargetParameters& targetParameters)
{
  return targetParameters.opponentsHeatParameter + targetParameters.opponentsToGoalHeatParameter;
}

float ScoreFunctions::scoreKick(const Kick* kick, const ShotParameters::KickParameters kickParameters)
{
  float score = 0.f;
  score += kickParameters.generalParameter * kick->generalValue;
  if (kick->kickBlind)
  {
    score += kickParameters.kickBlindParameter;
  }
  return score;
}

float ScoreFunctions::scoreTarget(
    const float ballGoalsHeat, const SelectableTarget& selectableTarget, const ShotParameters::TargetParameters targetParameters, const FieldDimensions& theFieldDimensions, const PositionInfo& thePositionInfo)
{
  const int index = HeatMapFunctions::fieldToIndex(selectableTarget.target, theFieldDimensions);

  const float intoGoalKickOutsideScore = selectableTarget.distanceInfo.intoGoalKickOutsideProbability * targetParameters.intoGoalKickOutsideParameter;
  const float intoKickInOutsideScore = selectableTarget.distanceInfo.intoKickInOutsideProbability * targetParameters.intoKickInOutsideParameter;
  const float intoCornerKickOutsideScore = selectableTarget.distanceInfo.intoCornerKickOutsideProbability * targetParameters.intoCornerKickOutsideParameter;
  const float intoOpponentsGoalScore = selectableTarget.distanceInfo.intoOpponentsGoalProbability * targetParameters.intoOpponentsGoalParameter;
  const float intoOwnGoalScore = selectableTarget.distanceInfo.intoOwnGoalProbability * targetParameters.intoOwnGoalParameter;
  const float outsideScore = intoGoalKickOutsideScore + intoKickInOutsideScore + intoCornerKickOutsideScore + intoOpponentsGoalScore + intoOwnGoalScore;

  float positionScore = 0.f;

  positionScore += targetParameters.sidesHeatParameter * thePositionInfo.sidesHeatMap.getHeat(index);
  const float goalsHeat = thePositionInfo.goalsHeatMap.getHeat(index);
  positionScore += targetParameters.goalsHeatParameter * goalsHeat;

  float selfScore = 0.f;
  selfScore += targetParameters.selfHeatParameter * thePositionInfo.selfHeatMap.getHeat(index);
  selfScore += targetParameters.selfToGoalHeatParameter * thePositionInfo.selfToGoalHeatMap.getHeat(index, selectableTarget.target, theFieldDimensions);
  float teamScore = 0.f;
  teamScore += targetParameters.teammatesHeatParameter * thePositionInfo.teamHeatMap.getHeat(index);
  teamScore += targetParameters.teammatesToGoalHeatParameter * thePositionInfo.teamToGoalHeatMap.getHeat(index, selectableTarget.target, theFieldDimensions);
  positionScore += std::max(selfScore, teamScore); /* take max of values because only one can go to the ball. By taking the values instead of the heat the ShotParameters can be used
  to decide how much the border where one or the other should go the ball shifts */

  float opponentScore = 0.f;
  opponentScore += targetParameters.opponentsHeatParameter * thePositionInfo.opponentHeatMap.getHeat(index);
  opponentScore += targetParameters.opponentsToGoalHeatParameter * thePositionInfo.opponentToGoalHeatMap.getHeat(index, selectableTarget.target, theFieldDimensions);
  positionScore += opponentScore;

  const float crowdedScore = targetParameters.crowdedParameter * thePositionInfo.crowdedHeatMap.getHeat(index);
  positionScore += crowdedScore;

  const float intoFieldProbability = 1.f - selectableTarget.distanceInfo.intoGoalKickOutsideProbability - selectableTarget.distanceInfo.intoKickInOutsideProbability
      - selectableTarget.distanceInfo.intoCornerKickOutsideProbability - selectableTarget.distanceInfo.intoOpponentsGoalProbability - selectableTarget.distanceInfo.intoOwnGoalProbability;
  ASSERT(intoFieldProbability >= 0.f);
  ASSERT(intoFieldProbability <= 1.f);

  const float intoDirectionOfGoalsScore = getIntoDirectionOfGoalsHeatValue(ballGoalsHeat, goalsHeat, targetParameters);

  const float insideScore = intoFieldProbability * (positionScore + intoDirectionOfGoalsScore);

  return outsideScore + insideScore;
}

float ScoreFunctions::getIntoDirectionOfGoalsHeatValue(const float ballGoalsHeat, const float goalsHeat, const ShotParameters::TargetParameters& targetParameters)
{
  return goalsHeat >= ballGoalsHeat ? targetParameters.intoDirectionOfGoalsHeatParameter : 0.f;
}

float ScoreFunctions::scorePose(const SelectablePose& selectablePose, const ShotParameters::PoseParameters poseParameters, const TacticSymbols& theTacticSymbols)
{
  float score = 0;
  score += getDontRuntIntoScore(selectablePose, poseParameters.dontRuntIntoParameter, theTacticSymbols); // todo check. those are not tested well enough and might make problems
  score += getBlockDefensiveConeScore(selectablePose, poseParameters.blockDefensiveConeParameter, theTacticSymbols);
  score += getBlockOpponentScore(selectablePose, poseParameters.blockOpponentParameter, theTacticSymbols);
  return score;
}

float ScoreFunctions::getDontRuntIntoScore(const SelectablePose& selectablePose, const float dontRuntIntoFactor, const TacticSymbols& theTacticSymbols)
{
  ASSERT(dontRuntIntoFactor >= 0);

  if (!theTacticSymbols.hasClosestRobot())
  {
    return dontRuntIntoFactor;
  }

  const float MAX_VALUABLE_DISTANCE = 1500.f;

  const float robotToOtherRobotDistance = Geometry::distance(selectablePose.pose.translation, theTacticSymbols.closestToBallRobot.translation);
  if (robotToOtherRobotDistance >= MAX_VALUABLE_DISTANCE)
  {
    return dontRuntIntoFactor;
  }

  const float distanceFactor = robotToOtherRobotDistance / MAX_VALUABLE_DISTANCE;
  ASSERT(distanceFactor >= 0.f);
  return dontRuntIntoFactor * distanceFactor;
}

float ScoreFunctions::getBlockDefensiveConeScore(const SelectablePose& selectablePose, const float blockDefensiveConeFactor, const TacticSymbols& theTacticSymbols)
{
  ASSERT(blockDefensiveConeFactor >= 0.f);

  const float DONT_BLOCK_DEFENSIVE_CONE_DISTANCE = 1500.f;

  if (!theTacticSymbols.hasClosestOpponentRobot() || theTacticSymbols.ballToOpponentRobotDistance > DONT_BLOCK_DEFENSIVE_CONE_DISTANCE)
  {
    return blockDefensiveConeFactor;
  }

  const Vector2f& ballPosition = selectablePose.getBallPosition();
  const Angle ballToKickPoseAngle = (selectablePose.pose.translation - ballPosition).angle();
  return blockDefensiveConeFactor * MathUtils::getCloseToMiddlePercent(ballToKickPoseAngle, theTacticSymbols.defensiveCone.left, theTacticSymbols.defensiveCone.right);
}

/**
 * @return Higher score the closer the robot is to the opposite site of the opponent
 */
float ScoreFunctions::getBlockOpponentScore(const SelectablePose& selectablePose, const float blockOpponentFactor, const TacticSymbols& theTacticSymbols)
{
  ASSERT(blockOpponentFactor >= 0.f);

  const float MIN_DISTANCE_TO_BLOCK = 1500.f;

  if (!theTacticSymbols.hasClosestOpponentRobot() || theTacticSymbols.ballToOpponentRobotDistance > MIN_DISTANCE_TO_BLOCK)
  {
    return blockOpponentFactor;
  }

  const Vector2f& ballPosition = selectablePose.getBallPosition();

  const Angle ballToKickPoseAngle = (selectablePose.pose.translation - ballPosition).angle();
  const Angle ballToOpponentAngle = (theTacticSymbols.closestToBallOpponentRobot.translation - ballPosition).angle();

  const Angle diff = MathUtils::getAngleSmallestDiff(ballToKickPoseAngle, ballToOpponentAngle);
  const float value = diff / 180_deg;
  return blockOpponentFactor * value;
}

float ScoreFunctions::scoreSelectableShot(const SelectableShot& selectableShot, const ProbabilityCone& probabilityCone)
{
  const SelectableShot* left = selectableShot.leftSelectableShot;
  const SelectableShot* right = selectableShot.rightSelectableShot;

  float score = selectableShot.selectableTarget.getScore() * (float)(probabilityCone.stepsToEachDirection + 1);

  for (int step = probabilityCone.stepsToEachDirection; step > 0; step--)
  {
    if (left == nullptr)
    {
      break;
    }

    score += left->selectableTarget.getScore() * (float)step;

    ASSERT(left->leftSelectableShot != nullptr);
    left = left->leftSelectableShot;
  }

  for (int step = probabilityCone.stepsToEachDirection; step > 0; step--)
  {
    if (right == nullptr)
    {
      break;
    }

    score += right->selectableTarget.getScore() * (float)step;

    ASSERT(right->leftSelectableShot != nullptr);
    right = right->rightSelectableShot;
  }

  return score / probabilityCone.divisor;
}

float ScoreFunctions::scoreExecutableShot(const ExecutableShot& executableShot, const float successProbability, const float noSuccessScore)
{
  const float successAgnosticScore = executableShot.selectablePose.selectableShot.selectableTarget.selectableKick.getScore() + executableShot.selectablePose.getScore();
  const float successScore = executableShot.selectablePose.selectableShot.getScore();
  float executableShotScore = successAgnosticScore + successProbability * successScore + (1.f - successProbability) * noSuccessScore;
  if (executableShot.hysteresis)
  {
    executableShotScore = ScoreFunctions::applyHysteresis(executableShotScore, executableShot.selectablePose.getPoseTime(), executableShot.hysteresis);
  }
  return executableShotScore;
}

float ScoreFunctions::applyHysteresis(const float score, const float poseTime, const Hysteresis& hysteresis)
{
  if (hysteresis == Hysteresis::NO)
  {
    return score;
  }

  const float MAX_FOR_JUST_DO_IT_HYSTERESIS = 3.f;
  const float JUST_DO_IT_HYSTERESIS_ADD = 10.f;
  const float JUST_DO_IT_HYSTERESIS_MULTIPLIER = 10.f;

  const float MAX_FOR_HIGH_HYSTERESIS = 5.f;
  const float HIGH_HYSTERESIS_ADD = 3.f;
  const float HIGH_HYSTERESIS_MULTIPLIER = 3.f;

  const float MAX_FOR_LOW_HYSTERESIS = 7.f;
  const float LOW_HYSTERESIS_ADD = 0.5f;
  const float LOW_HYSTERESIS_MULTIPLIER = 1.1f;

  float hysteresisAdd;
  float hysteresisMultiplierOrDivisor;
  if (hysteresis == Hysteresis::FORCE_JUST_DO_IT || (hysteresis == Hysteresis::YES && poseTime < MAX_FOR_JUST_DO_IT_HYSTERESIS))
  {
    hysteresisAdd = JUST_DO_IT_HYSTERESIS_ADD;
    hysteresisMultiplierOrDivisor = JUST_DO_IT_HYSTERESIS_MULTIPLIER;
  }
  else if (hysteresis == Hysteresis::FORCE_HIGH || (hysteresis == Hysteresis::YES && poseTime < MAX_FOR_HIGH_HYSTERESIS))
  {
    hysteresisAdd = HIGH_HYSTERESIS_ADD;
    hysteresisMultiplierOrDivisor = HIGH_HYSTERESIS_MULTIPLIER;
  }
  else if (hysteresis == Hysteresis::FORCE_LOW || (hysteresis == Hysteresis::YES && poseTime < MAX_FOR_LOW_HYSTERESIS))
  {
    hysteresisAdd = LOW_HYSTERESIS_ADD;
    hysteresisMultiplierOrDivisor = LOW_HYSTERESIS_MULTIPLIER;
  }
  else
  {
    return score;
  }

  return HysteresisUtils::makeBigger(score, hysteresisAdd, hysteresisMultiplierOrDivisor);
}