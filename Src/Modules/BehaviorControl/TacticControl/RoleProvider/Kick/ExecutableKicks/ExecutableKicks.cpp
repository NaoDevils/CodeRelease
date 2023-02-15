#include "ExecutableKicks.h"

#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/HysterUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Logs/KickDrawings.h>
#include <Representations/BehaviorControl/GoalSymbols.h>

void ExecutableKicks::insert(const ExecutableKicks& other)
{
  insert(other.executableKicks);
}

void ExecutableKicks::insert(const std::vector<ExecutableKick>& other)
{
  executableKicks.insert(end(executableKicks), begin(other), end(other));
}

ExecutableKicks ExecutableKicks::filterOutside(const FieldDimensions& theFieldDimensions)
{
  return filter(
      [this, &theFieldDimensions](const ExecutableKick& executableKick)
      {
        const Vector2f direction = (executableKick.target - ballPosition).normalized();
        return KickUtils::isKickToOutside(executableKick.kick, ballPosition, direction, executableKick.hysteresis, theFieldDimensions);
      });
}

ExecutableKicks ExecutableKicks::filterTooFarBack(float minX)
{
  return filter(
      [&minX](const ExecutableKick& executableKick)
      {
        return executableKick.target.x() < minX;
      });
}

ExecutableKicks ExecutableKicks::filterLeft(float maxY)
{
  return filter(
      [&maxY](const ExecutableKick& executableKick)
      {
        return executableKick.target.x() > maxY;
      });
}

ExecutableKicks ExecutableKicks::filterRight(float minY)
{
  return filter(
      [&minY](const ExecutableKick& executableKick)
      {
        return executableKick.target.x() < minY;
      });
}

ExecutableKicks ExecutableKicks::filterBlocked(const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap)
{
  for (ExecutableKick& executableKick : executableKicks)
  {
    ASSERT(executableKick.widthSet == false);
    const float width = KickUtils::getMinKickToObstaclesDistance(ballPosition, executableKick.target, theFieldDimensions, theRobotMap);
    executableKick.setWidth(width);
  }
  return filter(
      [&theFieldDimensions](const ExecutableKick& executableKick)
      {
        if (!executableKick.widthSet)
        {
          throw std::logic_error("Calculate width before check if blocked!");
        }
        const float ballRadius = theFieldDimensions.ballRadius;
        const float robotRadius = 140.f; // TODO Constants
        const float minWidth = (ballRadius + robotRadius) + (executableKick.hysteresis ? 0.f : 20.f);
        return executableKick.width < minWidth;
      });
}

ExecutableKicks ExecutableKicks::filterTooSharpForGoalKickAngles(const FieldDimensions& theFieldDimensions)
{
  const Angle MAX_ANGLE = 74_deg; // TODO Constant also used in GoalObjective

  const Geometry::Line leftFieldLine = {Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftSideline), Vector2f(-1.f, 0.f)};
  const Vector2f leftGoalPost = {theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal};
  const Geometry::Line leftAngleLine = {leftGoalPost, MathUtils::angleToVector(-MAX_ANGLE)};
  Vector2f leftOutsideIntersection;
  VERIFY(Geometry::getIntersectionOfLines(leftFieldLine, leftAngleLine, leftOutsideIntersection));

  const Geometry::Line rightFieldLine = {Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightSideline), Vector2f(-1.f, 0.f)};
  const Vector2f rightGoalPost = {theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal};
  const Geometry::Line rightAngleLine = {rightGoalPost, MathUtils::angleToVector(MAX_ANGLE)};
  Vector2f rightOutsideIntersection;
  VERIFY(Geometry::getIntersectionOfLines(rightFieldLine, rightAngleLine, rightOutsideIntersection));

  return filter(
      [&leftGoalPost, &leftOutsideIntersection, &rightGoalPost, &rightOutsideIntersection](const ExecutableKick& executableKick)
      {
        const bool left = executableKick.target.x() < 0;
        if (left)
        {
          return Geometry::isPointLeftOfLine(executableKick.target, leftOutsideIntersection, leftGoalPost);
        }
        else
        {
          return Geometry::isPointLeftOfLine(executableKick.target, rightGoalPost, rightOutsideIntersection);
        }
      });
}

ExecutableKicks ExecutableKicks::reduceToBest(const float timeFactor,
    const float inaccuracyFactor,
    const float widthFactor,
    const float sidesHeatFactor,
    const float opponentsGoalHeatFactor,
    const float alliesHeatFactor,
    const float alliesMaxHeatFactor,
    const float alliesGoalKickHeatFactor,
    const float opponentsHeatFactor,
    const float opponentsMaxHeatFactor,
    const float opponentsGoalKickHeatFactor,
    const bool leftFootClosestToBall,
    const FieldDimensions& theFieldDimensions,
    const HeatMapCollection& theHeatMapCollection,
    const RobotMap& theRobotMap)
{
  ASSERT(timeFactor <= 0);
  ASSERT(inaccuracyFactor <= 0);
  ASSERT(widthFactor >= 0);
  // sidesHeatFactor depends on situation
  ASSERT(opponentsGoalHeatFactor >= 0);
  ASSERT(alliesHeatFactor >= 0);
  ASSERT(alliesMaxHeatFactor >= 0);
  ASSERT(alliesGoalKickHeatFactor >= 0);
  ASSERT(opponentsHeatFactor <= 0);
  ASSERT(opponentsMaxHeatFactor <= 0);
  ASSERT(opponentsGoalKickHeatFactor <= 0);

  if (executableKicks.empty())
  {
    return *this;
  }

  const bool useWidth = !MathUtils::isEqual(0.f, widthFactor);

  const bool useSidesHeat = !MathUtils::isEqual(0.f, sidesHeatFactor);
  const bool useOpponentsGoalHeat = !MathUtils::isEqual(0.f, opponentsGoalHeatFactor);
  const bool useAlliesHeat = !MathUtils::isEqual(0.f, alliesHeatFactor);
  const bool useAlliesMaxHeat = !MathUtils::isEqual(0.f, alliesMaxHeatFactor);
  const bool useAlliesGoalKickHeat = !MathUtils::isEqual(0.f, alliesGoalKickHeatFactor);
  const bool useOpponentsHeat = !MathUtils::isEqual(0.f, opponentsHeatFactor);
  const bool useOpponentsMaxHeat = !MathUtils::isEqual(0.f, opponentsMaxHeatFactor);
  const bool useOpponentsGoalKickHeat = !MathUtils::isEqual(0.f, opponentsGoalKickHeatFactor);
  const bool useHeatMapScore = useSidesHeat || useOpponentsGoalHeat || useAlliesHeat || useAlliesMaxHeat || useAlliesGoalKickHeat || useOpponentsHeat || useOpponentsMaxHeat
      || useOpponentsGoalKickHeat; //TODO Or outside field?

  std::map<Kick*, int> kickToNrMap = {};
  int nr = 1;
  for (ExecutableKick& executableKick : executableKicks)
  {
    if (kickToNrMap.count(executableKick.kick) == 0)
    {
      kickToNrMap.insert({executableKick.kick, nr});
    }
  }

  std::map<int, float> indexHeatScoreMap = {};
  if (useHeatMapScore)
  {
    std::map<int, HeatMap::Area> indexAreaMap = {};
    for (ExecutableKick& executableKick : executableKicks)
    {
      const int index = HeatMap::fieldToIndex(executableKick.target, theFieldDimensions);
      const int key = index * kickToNrMap.at(executableKick.kick);
      if (indexAreaMap.count(key) == 0)
      {
        const HeatMap::Area area = HeatMap::getArea(executableKick.target, executableKick.kick->getInaccuracy(), theFieldDimensions);
        indexAreaMap.insert({key, area});
      }
    }

    for (auto [key, area] : indexAreaMap)
    {
      const auto sidesHeatScore = useSidesHeat ? (sidesHeatFactor * theHeatMapCollection.sidesHeatMap.getHeat(area, theFieldDimensions)) : 0;
      const auto opponentsGoalHeatScore = useOpponentsGoalHeat ? (opponentsGoalHeatFactor * theHeatMapCollection.opponentsGoalHeatMap.getHeat(area, theFieldDimensions)) : 0;
      const auto alliesHeatScore = useAlliesHeat ? (alliesHeatFactor * theHeatMapCollection.alliesHeatMap.getHeat(area, theFieldDimensions)) : 0;
      const auto alliesMaxHeatScore = useAlliesMaxHeat ? (alliesMaxHeatFactor * theHeatMapCollection.alliesMaxHeatMap.getHeat(area, theFieldDimensions)) : 0;
      const auto alliesGoalKickHeatScore = useAlliesGoalKickHeat ? (alliesGoalKickHeatFactor * theHeatMapCollection.alliesGoalKickHeatMap.getHeat(area, theFieldDimensions)) : 0;
      const auto opponentsHeatScore = useOpponentsHeat ? (opponentsHeatFactor * theHeatMapCollection.opponentsHeatMap.getHeat(area, theFieldDimensions)) : 0;
      const auto opponentsMaxHeatScore = useOpponentsMaxHeat ? (opponentsMaxHeatFactor * theHeatMapCollection.opponentsMaxHeatMap.getHeat(area, theFieldDimensions)) : 0;
      const auto opponentsGoalKickHeatScore = useOpponentsGoalKickHeat ? (opponentsGoalKickHeatFactor * theHeatMapCollection.opponentsGoalKickHeatMap.getHeat(area, theFieldDimensions)) : 0;

      float heatScore = sidesHeatScore + opponentsGoalHeatScore + alliesHeatScore + alliesMaxHeatScore + alliesGoalKickHeatScore + opponentsHeatScore + opponentsMaxHeatScore + opponentsGoalKickHeatScore;

      indexHeatScoreMap.insert({key, heatScore});
    }
  }

  const float distanceToKick = Geometry::distance(playerPose.translation, ballPosition);
  const bool useHysteresis = distanceToKick < 1500.f; // TODO Constant

  float maxScore = -std::numeric_limits<float>::infinity();
  ExecutableKick* maxScoreExecutableKick = nullptr;
  float maxScoreWidth = 0;

  std::vector<Vector2f> draw_targets;
  draw_targets.reserve(executableKicks.size());
  std::vector<float> draw_scores;
  draw_scores.reserve(executableKicks.size());

  for (ExecutableKick& executableKick : executableKicks)
  {
    float score = 0;

    const float time = KickUtils::getKickTime(*executableKick.kick, playerPose, ballPosition, executableKick.target, false, leftFootClosestToBall);
    const float maxTime = 20.f; // TODO Constant
    if (time > maxTime)
    {
      OUTPUT_WARNING("Assumed maxTime is lower than calculated time for kick!");
    }
    const float timeScore = timeFactor * std::min(time, maxTime) / maxTime;
    score += timeScore;

    const float inaccuracy = executableKick.kick->getHorizontalInaccuracy();
    const float maxInaccuracy = 2000.f; // TODO Constant
    const float inaccuracyScore = inaccuracyFactor * std::min(inaccuracy, maxInaccuracy) / maxInaccuracy;
    score += inaccuracyScore;

    float width = 0;
    if (useWidth)
    {
      if (executableKick.widthSet)
      {
        width = executableKick.width;
      }
      else
      {
        width = KickUtils::getMinKickToObstaclesDistance(ballPosition, executableKick.target, theFieldDimensions, theRobotMap);
        executableKick.setWidth(width);
      }
      const float maxWidth = 750.f; // TODO Constant
      const float widthValue = std::pow(MathUtils::clamp_f(width, 0.f, maxWidth) / maxWidth, 1 / 3.f); // TODO Constant
      const float widthScore = widthFactor * widthValue;
      score += widthScore;
    }

    if (useHeatMapScore)
    {
      const int index = HeatMap::fieldToIndex(executableKick.target, theFieldDimensions);
      const int key = index * kickToNrMap.at(executableKick.kick);
      score += indexHeatScoreMap.at(key);
    }

    const float scoreWithoutHysteresis = score;
    score = (useHysteresis && executableKick.hysteresis) ? HysterUtils::makeBigger(score, 1.f, 1.2f) : score;

    if (score > maxScore)
    {
      maxScore = score;
      maxScoreExecutableKick = &executableKick;
      maxScoreWidth = width;
    }

    draw_targets.push_back(executableKick.target);
    draw_scores.push_back(scoreWithoutHysteresis);
  }

  if (maxScoreExecutableKick == nullptr)
  {
    return {};
  }

  ExecutableKick maxScoreExecutableKickCopy = *maxScoreExecutableKick;

  drawInGrid(draw_targets, draw_scores, theFieldDimensions);
  drawFree(draw_targets, draw_scores);
  drawKickMinObstacleWidth(ballPosition, maxScoreExecutableKickCopy.target, maxScoreWidth);

  executableKicks.clear();
  executableKicks.push_back(maxScoreExecutableKickCopy);
  return *this;
}

bool ExecutableKicks::hasBest() const
{
  if (executableKicks.empty())
  {
    return false;
  }
  ASSERT(executableKicks.size() == 1);
  return true;
}

ExecutableKick ExecutableKicks::getBest() const
{
  ASSERT(executableKicks.size() == 1);
  return executableKicks.at(0);
}

ExecutableKicks ExecutableKicks::filter(const std::function<bool(ExecutableKick)>& isToRemove)
{
  executableKicks.erase(std::remove_if(executableKicks.begin(), executableKicks.end(), isToRemove), executableKicks.end());
  return *this;
}

void ExecutableKicks::drawInGrid(const std::vector<Vector2f>& draw_targets, const std::vector<float>& draw_scores, const FieldDimensions& theFieldDimensions)
{
  COMPLEX_DRAWING(DRAW_EXECUTABLE_KICKS_IN_GRID)
  {
    const bool STRETCH = false;

    std::vector<int> draw_indexes = {};
    for (const Vector2f& draw_target : draw_targets)
    {
      const int draw_index = HeatMap::fieldToIndex(draw_target, theFieldDimensions);
      draw_indexes.push_back(draw_index);
    }

    std::vector<int> unique_draw_indexes = {};
    std::vector<float> unique_draw_scores = {};
    for (size_t i = 0; i < draw_indexes.size(); i++)
    {
      const int index = draw_indexes.at(i);
      const float score = draw_scores.at(i);

      bool foundEqualTarget = false;
      for (size_t j = 0; j < unique_draw_indexes.size(); j++)
      {
        const int uniqueIndex = unique_draw_indexes.at(j);
        const float uniqueScore = unique_draw_scores.at(j);

        if (index == uniqueIndex)
        {
          if (score > uniqueScore)
          {
            unique_draw_scores.at(j) = score;
          }
          foundEqualTarget = true;
          break;
        }
      }
      if (!foundEqualTarget)
      {
        unique_draw_indexes.push_back(index);
        unique_draw_scores.push_back(score);
      }
    }

    if (STRETCH)
    {
      MathUtils::stretch(unique_draw_scores);
    }

    const size_t size = unique_draw_scores.size();
    for (size_t i = 0; i < size; ++i)
    {
      const Vector2f draw_target = HeatMap::indexToField(unique_draw_indexes[i], theFieldDimensions);
      float draw_score = unique_draw_scores[i];
      if (!STRETCH)
      {
        draw_score = (MathUtils::clamp_f(draw_score, -1.5f, 1.5f) + 1.5f) / 3.f;
      }
      CIRCLE(DRAW_EXECUTABLE_KICKS_IN_GRID, draw_target.x(), draw_target.y(), 50, 0, Drawings::noPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA((char)(255 * (1 - draw_score)), (char)(255 * draw_score), 0));
    }
  }
}

void ExecutableKicks::drawFree(const std::vector<Vector2f>& draw_targets, std::vector<float>& draw_scores)
{
  COMPLEX_DRAWING(DRAW_EXECUTABLE_KICKS_FREELY)
  {
    const bool STRETCH = false;

    if (STRETCH)
    {
      MathUtils::stretch(draw_scores);
    }

    const size_t size = draw_targets.size();
    for (size_t i = 0; i < size; ++i)
    {
      const Vector2f& draw_target = draw_targets.at(i);
      float draw_score = draw_scores.at(i);
      if (!STRETCH)
      {
        draw_score = (MathUtils::clamp_f(draw_score, -1.5f, 1.5f) + 1.5f) / 3.f;
      }
      CIRCLE(DRAW_EXECUTABLE_KICKS_FREELY, draw_target.x(), draw_target.y(), 30, 0, Drawings::noPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA((char)(255 * (1 - draw_score)), (char)(255 * draw_score), 0));
    }
  }
}

void ExecutableKicks::drawKickMinObstacleWidth(const Vector2f& ballPosition, const Vector2f& targetPosition, const float width)
{
  Vector2f w = Vector2f(targetPosition - ballPosition).normalized() * width;
  w = w.rotate(90_deg);

  const Vector2f p11 = ballPosition - w;
  const Vector2f p12 = ballPosition + w;

  const Vector2f p21 = targetPosition - w;
  const Vector2f p22 = targetPosition + w;

  LINE(DRAW_KICK_MIN_WIDTH, p11.x(), p11.y(), p21.x(), p21.y(), 10, Drawings::solidPen, ColorRGBA::orange);
  LINE(DRAW_KICK_MIN_WIDTH, p12.x(), p12.y(), p22.x(), p22.y(), 10, Drawings::solidPen, ColorRGBA::orange);
}
