#include "ScoreFunctions.h"

void ScoreFunctions::scoreKickPlan(
    KickPlan& kickPlan, const Factors& factors, const FieldDimensions& theFieldDimensions, const HeatMapCollection& theHeatMapCollection, const RobotMap& theRobotMap, const TacticSymbols& theTacticSymbols)
{
  kickPlan.selectablePose.selectableTarget.selectableKick.score = scoreKick(kickPlan.selectablePose.selectableTarget.selectableKick.kick, factors.timeFactor, factors.inaccuracyFactor);
  kickPlan.selectablePose.selectableTarget.score = scoreTarget(kickPlan.selectablePose.selectableTarget,
      factors.widthFactor,
      factors.sidesHeatFactor,
      factors.goalsHeatFactor,
      factors.teammatesKickHeatFactor,
      factors.teammatesGoalKickHeatFactor,
      factors.opponentsKickHeatFactor,
      factors.opponentsGoalKickHeatFactor,
      theFieldDimensions,
      theHeatMapCollection,
      theRobotMap);
  kickPlan.selectablePose.score = scorePose(kickPlan.selectablePose, factors.dontRuntIntoFactor, factors.blockDefensiveCone, factors.blockOpponentFactor, factors.timeFactor, theTacticSymbols);
  kickPlan.score = kickPlan.selectablePose.selectableTarget.selectableKick.score + kickPlan.selectablePose.selectableTarget.score + kickPlan.selectablePose.score;
  for (const auto& customScoreFunction : factors.customScoreFunctions)
  {
    kickPlan.score += customScoreFunction(kickPlan);
  }
}

float ScoreFunctions::scoreTarget(const SelectableTarget& selectableTarget,
    const float widthFactor,
    const float sidesHeatFactor,
    const float goalsHeatFactor,
    const float teammatesKickHeatFactor,
    const float teammatesGoalKickHeatFactor,
    const float opponentsKickHeatFactor,
    const float opponentsGoalKickHeatFactor,
    const FieldDimensions& theFieldDimensions,
    const HeatMapCollection& theHeatMapCollection,
    const RobotMap& theRobotMap)
{
  ASSERT(widthFactor >= 0);
  // sidesHeatFactor depends on situation
  ASSERT(goalsHeatFactor >= 0);
  ASSERT(teammatesKickHeatFactor >= 0);
  ASSERT(teammatesGoalKickHeatFactor >= 0);
  ASSERT(opponentsKickHeatFactor <= 0);
  ASSERT(opponentsGoalKickHeatFactor <= 0);

  float score = 0;
  if (!MathUtils::isEqual(0.f, widthFactor))
  {
    const float widthScore = getWidthScore(selectableTarget.ballPosition, selectableTarget.target, widthFactor, theFieldDimensions, theRobotMap);
    score += widthScore;
  }
  const int index = HeatMap::fieldToIndex(selectableTarget.target, theFieldDimensions);
  score += sidesHeatFactor * theHeatMapCollection.sidesHeatMap.getHeat(index);
  score += goalsHeatFactor * theHeatMapCollection.goalsHeatMap.getHeat(index);
  score += teammatesKickHeatFactor * theHeatMapCollection.teamKickHeatMap.getHeat(index);
  score += teammatesGoalKickHeatFactor * theHeatMapCollection.teamGoalKickHeatMap.getHeat(index);
  score += opponentsKickHeatFactor * theHeatMapCollection.opponentKickHeatMap.getHeat(index);
  score += opponentsGoalKickHeatFactor * theHeatMapCollection.opponentGoalKickHeatMap.getHeat(index);
  return score;
}

float ScoreFunctions::getWidthScore(const Vector2f& ballPosition, const Vector2f& targetPosition, const float widthFactor, const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap)
{
  const float MAX_WIDTH = 750.f;
  const float width = KickUtils::getMinKickToObstaclesDistance(ballPosition, targetPosition, theFieldDimensions, theRobotMap);
  const float widthValue = std::pow(MathUtils::clamp_f(width, 0.f, MAX_WIDTH) / MAX_WIDTH, 1 / 3.f); // TODO Constant
  return widthFactor * widthValue;
}

float ScoreFunctions::scorePose(
    const SelectablePose& selectablePose, const float dontRuntIntoFactor, const float blockDefensiveCone, const float blockOpponentFactor, const float timeFactor, const TacticSymbols& theTacticSymbols)
{
  float score = 0;
  score += getDontRuntIntoScore(selectablePose, dontRuntIntoFactor, theTacticSymbols);
  score += getBlockDefensiveConeScore(selectablePose, blockDefensiveCone, theTacticSymbols);
  score += getBlockOpponentScore(selectablePose, blockOpponentFactor, theTacticSymbols);
  score += getTimeScore(selectablePose, timeFactor);
  return score;
}

float ScoreFunctions::getDontRuntIntoScore(const SelectablePose& selectablePose, const float dontRuntIntoFactor, const TacticSymbols& theTacticSymbols)
{
  ASSERT(dontRuntIntoFactor >= 0);

  const Vector2f& ballPosition = selectablePose.selectableTarget.ballPosition;
  const Vector2f& targetPosition = selectablePose.selectableTarget.target;
  Kick* kick = selectablePose.selectableTarget.selectableKick.kick;

  float score = 0;

  if (theTacticSymbols.closeToBallRobotNumber > 0 && kick->getOptAngle() < 10_deg)
  {
    const Vector2f& robotPosition = theTacticSymbols.closeToBallRobot.translation;

    const Pose2f kickLinePose = Pose2f((targetPosition - ballPosition).angle(), ballPosition);
    const Angle kickLineToRobotAngle = Transformation::fieldToRobot(kickLinePose, robotPosition).angle();
    const Angle absKickLineToRobotAngle = std::abs(kickLineToRobotAngle);

    const Angle A_MIN = 50_deg;
    const Angle MAX_DYNAMIC_START = 130_deg;
    const Angle MAX_DYNAMIC_END = 170_deg;

    if (absKickLineToRobotAngle < A_MIN)
    {
      // Is ahead, give score for both feet
      score += dontRuntIntoFactor;
    }
    else
    {
      float value;
      if (absKickLineToRobotAngle < MAX_DYNAMIC_START)
      {
        value = 1.f;
      }
      else
      {
        value = std::max(0.f, 1 - (absKickLineToRobotAngle - MAX_DYNAMIC_START) / (MAX_DYNAMIC_END - MAX_DYNAMIC_START));
        ASSERT(value <= 1.f);
      }
      const float dontRunIntoScore = dontRuntIntoFactor * value;
      if (kickLineToRobotAngle > 0.f)
      {
        // Is left, give score for left foot
        score += selectablePose.kickWithLeft ? dontRunIntoScore : 0.f;
      }
      else
      {
        // Is right, give score for right foot
        score += selectablePose.kickWithLeft ? 0.f : dontRunIntoScore;
      }
    }
  }
  return score;
}

float ScoreFunctions::getBlockDefensiveConeScore(const SelectablePose& selectablePose, const float blockDefensiveConeFactor, const TacticSymbols& theTacticSymbols)
{
  if (MathUtils::isEqual(blockDefensiveConeFactor, 0.f))
  {
    return 0.f;
  }
  ASSERT(blockDefensiveConeFactor >= 0.f);

  if (theTacticSymbols.closeToBallOpponentRobotNumber == 0)
  {
    return 0.f;
  }
  const Vector2f& ballPosition = selectablePose.selectableTarget.ballPosition;
  const Angle ballToKickPoseAngle = (selectablePose.pose.translation - ballPosition).angle();
  return MathUtils::getCloseToMiddlePercent(ballToKickPoseAngle, theTacticSymbols.defensiveCone.left, theTacticSymbols.defensiveCone.right);
}

/**
 * @return Higher score the closer the robot is to the opposite site of the opponent
 */
float ScoreFunctions::getBlockOpponentScore(const SelectablePose& selectablePose, const float blockOpponentFactor, const TacticSymbols& theTacticSymbols)
{
  if (MathUtils::isEqual(blockOpponentFactor, 0.f))
  {
    return 0.f;
  }
  ASSERT(blockOpponentFactor >= 0.f);

  if (theTacticSymbols.closeToBallOpponentRobotNumber != 1)
  {
    return 0.f;
  }

  const Vector2f& ballPosition = selectablePose.selectableTarget.ballPosition;

  const Angle ballToKickPoseAngle = (selectablePose.pose.translation - ballPosition).angle();
  const Angle ballToOpponentAngle = (theTacticSymbols.closeToBallOpponentRobot.translation - ballPosition).angle();

  const Angle diff = MathUtils::getAngleSmallestDiff(ballToKickPoseAngle, ballToOpponentAngle);
  const float value = diff / 180_deg;
  return blockOpponentFactor * value;
}

float ScoreFunctions::getTimeScore(const SelectablePose& selectablePose, const float timeFactor)
{
  ASSERT(timeFactor <= 0.f);

  const Pose2f playerPose = selectablePose.playerPose;
  const Vector2f ballPosition = selectablePose.selectableTarget.ballPosition;

  const float MAX_TIME = 18.f;
  const float time = PathUtils::getPathTime(playerPose, selectablePose.pose, ballPosition);
  if (time > MAX_TIME)
  {
    OUTPUT_WARNING("The constant value MAX_TIME=" << MAX_TIME << " < time=" << time << "! Increase MAX_TIME!");
    return timeFactor;
  }
  return timeFactor * time / MAX_TIME;
}

float ScoreFunctions::scoreKick(const Kick* kick, float timeFactor, float inaccuracyFactor)
{
  ASSERT(timeFactor <= 0);
  ASSERT(inaccuracyFactor <= 0);

  float score = 0;

  const float MAX_TIME = 5.f;
  const float time = kick->getTime(false);
  if (time > MAX_TIME)
  {
    OUTPUT_WARNING("The constant value MAX_TIME=" << MAX_TIME << " < time=" << time << "! Increase MAX_TIME!");
    return timeFactor;
  }
  score += timeFactor * time / MAX_TIME;

  const float MAX_INACCURACY = 2000.f;
  const float inaccuracy = kick->getHorizontalInaccuracy();
  score += inaccuracyFactor * std::min(inaccuracy, MAX_INACCURACY) / MAX_INACCURACY;

  return score;
}

float ScoreFunctions::applyHysteresis(const float score, const float distanceToKick)
{
  const float MAX_FOR_HIGH_HYSTERESIS = MAX_DISTANCE_FOR_HIGH_HYSTERESIS;
  const float HIGH_HYSTERESIS_ADD = 2.f;
  const float HIGH_HYSTERESIS_MULTIPLIER = 3.f;

  const float MAX_FOR_MEDIUM_HYSTERESIS = 500.f;
  const float MEDIUM_HYSTERESIS_ADD = 0.5f;
  const float MEDIUM_HYSTERESIS_MULTIPLIER = 1.5f;

  const float LOW_HYSTERESIS_ADD = 0.01f;
  const float LOW_HYSTERESIS_MULTIPLIER = 1.01f;

  float hysteresisAdd;
  float hysteresisMultiplier;
  if (distanceToKick < MAX_FOR_HIGH_HYSTERESIS)
  {
    hysteresisAdd = HIGH_HYSTERESIS_ADD;
    hysteresisMultiplier = HIGH_HYSTERESIS_MULTIPLIER;
  }
  else if (distanceToKick < MAX_FOR_MEDIUM_HYSTERESIS)
  {
    hysteresisAdd = MEDIUM_HYSTERESIS_ADD;
    hysteresisMultiplier = MEDIUM_HYSTERESIS_MULTIPLIER;
  }
  else
  {
    hysteresisAdd = LOW_HYSTERESIS_ADD;
    hysteresisMultiplier = LOW_HYSTERESIS_MULTIPLIER;
  }

  return HysteresisUtils::makeBigger(score, hysteresisAdd, hysteresisMultiplier);
}