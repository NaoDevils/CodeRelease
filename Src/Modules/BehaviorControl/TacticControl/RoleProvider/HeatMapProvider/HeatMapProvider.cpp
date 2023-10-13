#include "HeatMapProvider.h"
#include "Tools/Math/Transformation.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/BallUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/HeatMapProvider/HeatMapUtils.h>
#include <taskflow/taskflow.hpp>
#include <taskflow/algorithm/for_each.hpp>

HeatMapProvider::HeatMapProvider() = default;

void draw(std::vector<Vector2f> positionVector, std::vector<float> heatVector);

void HeatMapProvider::execute(tf::Subflow& subflow)
{
  const auto [teammateRobots, opponentRobots] = HeatMapUtils::getTeammateAndOtherRobots(theRobotMap);

  subflow
      .for_each_index(0,
          HeatMap::CELL_COUNT,
          1,
          [&, teammateRobots = teammateRobots, opponentRobots = opponentRobots](const int index)
          {
            const Vector2f fieldPosition = HeatMap::indexToField(index, theFieldDimensions);

            if (firstUpdate)
            {
              localHeatMapCollection.sidesHeatMap.setHeat(std::pow(HeatMapUtils::getSidesHeat(fieldPosition, theFieldDimensions), 2.f), index, theFieldDimensions);
              localHeatMapCollection.goalsHeatMap.setHeat(HeatMapUtils::getGoalsHeat(fieldPosition, theFieldDimensions), index, theFieldDimensions);
            }

            const auto [teammatesKickHeat, teammatesGoalKickHeat] =
                HeatMapUtils::getRobotHeatForPosition(fieldPosition, teammateRobots, FieldUtils::getOpponentGoalCenter(theFieldDimensions), theFieldDimensions);
            const auto [opponentsKickHeat, opponentsGoalKickHeat] =
                HeatMapUtils::getRobotHeatForPosition(fieldPosition, opponentRobots, FieldUtils::getOwnGoalCenter(theFieldDimensions), theFieldDimensions);


            if (kickHeatTakeNewPercent > 0.99f)
            {
              teammatesKickHeatMap.setHeat(teammatesKickHeat, index, theFieldDimensions);
              localHeatMapCollection.opponentKickHeatMap.setHeat(opponentsKickHeat, index, theFieldDimensions);
            }
            else
            {
              teammatesKickHeatMap.updateHeat(kickHeatTakeNewPercent, teammatesKickHeat, index, theFieldDimensions);
              localHeatMapCollection.opponentKickHeatMap.updateHeat(kickHeatTakeNewPercent, opponentsKickHeat, index, theFieldDimensions);
            }

            if (goalKickHeatTakeNewPercent > 0.99f)
            {
              teammatesGoalKickHeatMap.setHeat(teammatesGoalKickHeat, index, theFieldDimensions);
              localHeatMapCollection.opponentGoalKickHeatMap.setHeat(opponentsGoalKickHeat, index, theFieldDimensions);
            }
            else
            {
              teammatesGoalKickHeatMap.updateHeat(goalKickHeatTakeNewPercent, teammatesGoalKickHeat, index, theFieldDimensions);
              localHeatMapCollection.opponentGoalKickHeatMap.updateHeat(goalKickHeatTakeNewPercent, opponentsGoalKickHeat, index, theFieldDimensions);
            }

            // Apply instant heat
            const std::vector<Pose2f> selfPose = {Pose2f(theRobotPose.rotation, theBallSymbols.ballPositionField)}; // Use Ball Position since it's the robots position when he will kick the ball
            const auto [selfKickHeat, selfGoalKickHeat] = HeatMapUtils::getRobotHeatForPosition(fieldPosition, selfPose, FieldUtils::getOpponentGoalCenter(theFieldDimensions), theFieldDimensions);
            const float teamKickHeat = std::max(teammatesKickHeatMap.getHeat(index), selfKickHeat);
            const float teamGoalKickHeat = std::max(teammatesGoalKickHeatMap.getHeat(index), selfGoalKickHeat);
            localHeatMapCollection.teamKickHeatMap.setHeat(teamKickHeat, index, theFieldDimensions);
            localHeatMapCollection.teamGoalKickHeatMap.setHeat(teamGoalKickHeat, index, theFieldDimensions);
          })
      .name("UpdateHeat [HeatMapProvider]");
}

void HeatMapProvider::update(HeatMapCollection& heatMapCollection)
{
  heatMapCollection = localHeatMapCollection;
  firstUpdate = false;

  DECLARE_DEBUG_DRAWING(DRAW_HEAT_MAP, "drawingOnField");
  COMPLEX_DRAWING(DRAW_HEAT_MAP)
  {
    HeatMapUtils::draw(heatMapCollection.teamGoalKickHeatMap, true, theFieldDimensions);
  }
}

MAKE_MODULE(HeatMapProvider, modeling)
