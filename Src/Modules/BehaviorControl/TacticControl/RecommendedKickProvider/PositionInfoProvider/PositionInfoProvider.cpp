#include "PositionInfoProvider.h"

#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/Constants.h"
#include "PositionInfoUtils.h"
#include "Tools/Math/Transformation.h"
#include <taskflow/algorithm/for_each.hpp>

PositionInfoProvider::PositionInfoProvider() {}

void draw(std::vector<Vector2f> positionVector, std::vector<float> heatVector);

void PositionInfoProvider::execute(tf::Subflow& subflow)
{
  if (skips < SKIP_UPDATES)
  {
    skips += 1;
    return;
  }
  skips = 0;

  std::vector<Pose2f> selfPose = getSelfPose(theRobotPoseAfterPreview.rotation, theBallSymbols.ballPositionFieldPredicted); // Use Ball Position since it's the robots position when he will kick the ball
  const auto [teamRobots, opponentRobots] = PositionInfoUtils::getTeammateAndOtherRobots(theRobotMap);
  const std::vector<std::tuple<Pose2f, float>> crowdedPoses = PositionInfoUtils::getCrowdedRobots(theBallSymbols.ballPositionFieldPredicted, theRobotMap);

  // Update split cell
  updateSplitCells(localPositionInfo, selfPose, theFieldDimensions, theRobotMap);

  // Update regular cells
  subflow
      .for_each_index(0,
          Config::CELL_COUNT,
          1,
          [&, selfPose = selfPose, teamRobots = teamRobots, opponentRobots = opponentRobots, crowdedPoses = crowdedPoses](const int index)
          {
            updateRegularCell(localPositionInfo, index, selfPose, teamRobots, opponentRobots, crowdedPoses, theFieldDimensions);
          })
      .name("UpdateHeat [PositionInfoProvider]");
}

void PositionInfoProvider::update(PositionInfo& thePositionInfo)
{
  // Everything that has to be done once
  if (localPositionInfo.needInitialUpdate)
  {
    for (int index = 0; index < Config::CELL_COUNT; index++)
    {
      const Vector2f fieldPosition = HeatMapFunctions::indexToField(index, theFieldDimensions);
      localPositionInfo.goalsHeatMap.setHeat(PositionInfoUtils::getGoalsHeat(fieldPosition, theFieldDimensions), index, theFieldDimensions);
      localPositionInfo.sidesHeatMap.setHeat(PositionInfoUtils::getSidesHeat(fieldPosition, theFieldDimensions), index, theFieldDimensions);
    }
    localPositionInfo.needInitialUpdate = false;
  }

  // Copy from sub-flow
  thePositionInfo = localPositionInfo;

  // Draw
  DECLARE_DEBUG_DRAWING(DRAW_HEAT_MAP, "drawingOnField");
  COMPLEX_DRAWING(DRAW_HEAT_MAP)
  {
    thePositionInfo.draw(theFieldDimensions);
  }
}

PositionInfo PositionInfoProvider::create(
    const Pose2f& playerPose, const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions, const PositionInfo& thePositionInfo, const RobotMap& theRobotMap, const RobotPoseAfterPreview& theRobotPoseAfterPreview)
{
  PositionInfo newPositionInfo = thePositionInfo;

  const std::vector<Pose2f> selfPose = getSelfPose(playerPose.rotation, ballPosition); // Use Ball Position since it's the robots position when he will kick the ball
  const auto [teamRobots, opponentRobots] = PositionInfoUtils::getTeammateAndOtherRobots(theRobotMap);
  const std::vector<std::tuple<Pose2f, float>> crowdedPoses = PositionInfoUtils::getCrowdedRobots(ballPosition, theRobotMap);

  // Update split cell
  updateSplitCells(newPositionInfo, selfPose, theFieldDimensions, theRobotMap);

  for (int index = 0; index < Config::CELL_COUNT; index++)
  {
    updateRegularCell(newPositionInfo, index, selfPose, teamRobots, opponentRobots, crowdedPoses, theFieldDimensions);
  }

  return newPositionInfo;
}

void PositionInfoProvider::updateSplitCells(PositionInfo& thePositionInfo, const std::vector<Pose2f>& playerAtBallPose, const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap)
{
  /*
   * To be precise every split cell should take every robot into account, not just the one in the split cell.
   * But this are unnecessary calculations since you never want to play in the back of one robot anyway.
   */

  const auto [teamRobots, opponentRobots] = PositionInfoUtils::getTeammateAndOtherRobots(theRobotMap);

  const auto team_getPositionToHeatFunction = [&theFieldDimensions](const Pose2f& robotPose)
  {
    const auto positionToHeatFunction = [&robotPose, &theFieldDimensions](const Vector2f& splitCellPosition)
    {
      const auto [heat, toGoalHeat] = PositionInfoUtils::getRobotHeatForPosition(splitCellPosition, robotPose, FieldUtils::getOpponentGoalCenter(theFieldDimensions), theFieldDimensions);
      return toGoalHeat;
    };
    return positionToHeatFunction;
  };
  thePositionInfo.selfToGoalHeatMap.setSplitCellHeat(playerAtBallPose, theFieldDimensions, team_getPositionToHeatFunction);
  thePositionInfo.teamToGoalHeatMap.setSplitCellHeat(teamRobots, theFieldDimensions, team_getPositionToHeatFunction);

  const auto opponent_getPositionToHeatFunction = [&theFieldDimensions](const Pose2f& robotPose)
  {
    const auto positionToHeatFunction = [&robotPose, &theFieldDimensions](const Vector2f& splitCellPosition)
    {
      const auto [heat, toGoalHeat] = PositionInfoUtils::getRobotHeatForPosition(splitCellPosition, robotPose, FieldUtils::getOwnGoalCenter(theFieldDimensions), theFieldDimensions);
      return toGoalHeat;
    };
    return positionToHeatFunction;
  };
  thePositionInfo.opponentToGoalHeatMap.setSplitCellHeat(opponentRobots, theFieldDimensions, opponent_getPositionToHeatFunction);
}

void PositionInfoProvider::updateRegularCell(PositionInfo& thePositionInfo,
    const int index,
    const std::vector<Pose2f>& selfPose,
    const std::vector<Pose2f>& teamPoses,
    const std::vector<Pose2f>& opponentPoses,
    const std::vector<std::tuple<Pose2f, float>>& crowdedPoses,
    const FieldDimensions& theFieldDimensions)
{
  const Vector2f fieldPosition = HeatMapFunctions::indexToField(index, theFieldDimensions);

  // asser fieldPosition matches fieldPositionIndex
  const auto [selfHeat, selfToGoalHeat] = PositionInfoUtils::getRobotHeatForPosition(fieldPosition, selfPose, FieldUtils::getOpponentGoalCenter(theFieldDimensions), theFieldDimensions);
  thePositionInfo.selfHeatMap.setHeat(selfHeat, index, theFieldDimensions);
  thePositionInfo.selfToGoalHeatMap.setCellHeat(selfToGoalHeat, index, theFieldDimensions);

  const auto [teammatesHeat, teammatesToGoalHeat] = PositionInfoUtils::getRobotHeatForPosition(fieldPosition, teamPoses, FieldUtils::getOpponentGoalCenter(theFieldDimensions), theFieldDimensions);
  thePositionInfo.teamHeatMap.setHeat(teammatesHeat, index, theFieldDimensions);
  thePositionInfo.teamToGoalHeatMap.setCellHeat(teammatesToGoalHeat, index, theFieldDimensions);

  const auto [opponentsHeat, opponentsGoalHeat] = PositionInfoUtils::getRobotHeatForPosition(fieldPosition, opponentPoses, FieldUtils::getOwnGoalCenter(theFieldDimensions), theFieldDimensions);
  thePositionInfo.opponentHeatMap.setHeat(opponentsHeat, index, theFieldDimensions);
  thePositionInfo.opponentToGoalHeatMap.setCellHeat(opponentsGoalHeat, index, theFieldDimensions);

  const float crowdedHeat = PositionInfoUtils::getCrowdedHeat(fieldPosition, crowdedPoses);
  thePositionInfo.crowdedHeatMap.setHeat(crowdedHeat, index, theFieldDimensions);
}

std::vector<Pose2f> PositionInfoProvider::getSelfPose(const Angle rotation, const Vector2f& translation)
{
  std::vector<Pose2f> selfPose = {};
  selfPose.emplace_back(rotation, translation);
  return selfPose;
}

MAKE_MODULE(PositionInfoProvider, modeling)
