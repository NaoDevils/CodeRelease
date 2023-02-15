#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/HeatUtils.h>
#include "Tools/Math/Transformation.h"
#include "HeatMapProvider.h"

HeatMapProvider::HeatMapProvider() = default;

void draw(std::vector<Vector2f> positionVector, std::vector<float> heatVector);

void HeatMapProvider::update(HeatMapCollection& heatMapCollection)
{
  updateHeat(heatMapCollection);

  DECLARE_DEBUG_DRAWING("module:HeatMapProvider:HeatMap", "drawingOnField");
  HeatUtils::draw(heatMapCollection.opponentsGoalKickHeatMap, theFieldDimensions);
}

void HeatMapProvider::updateHeat(HeatMapCollection& heatMapCollection)
{
  for (int index = 0; index < HeatMap::CELL_COUNT; index++)
  {
    const Vector2f fieldPosition = HeatMap::indexToField(index, theFieldDimensions);

    if (firstUpdate)
    {
      sidesHeatMap.setHeat(std::pow(HeatUtils::getSidesHeat(fieldPosition, theFieldDimensions), 2.f), index, theFieldDimensions);
      opponentsGoalHeatMap.setHeat(HeatUtils::getOpponentsGoalHeat(fieldPosition, theFieldDimensions), index, theFieldDimensions);
    }

    heatMapCollection.sidesHeatMap = sidesHeatMap;
    heatMapCollection.opponentsGoalHeatMap = opponentsGoalHeatMap;

    auto [alliesHeat, alliesMaxHeat, alliesGoalKickHeat, opponentsHeat, opponentsMaxHeat, opponentsGoalKickHeat] = HeatUtils::getRobotHeat(fieldPosition, theFieldDimensions, theRobotMap, theRobotPose);

    if (takeNewPercent == 1.f)
    {
      heatMapCollection.alliesHeatMap.setHeat(alliesHeat, index, theFieldDimensions);
      heatMapCollection.alliesMaxHeatMap.setHeat(alliesMaxHeat, index, theFieldDimensions);
      heatMapCollection.opponentsHeatMap.setHeat(opponentsHeat, index, theFieldDimensions);
      heatMapCollection.opponentsMaxHeatMap.setHeat(opponentsMaxHeat, index, theFieldDimensions);
    }
    else
    {
      heatMapCollection.alliesHeatMap.updateHeat(takeNewPercent, alliesHeat, index, theFieldDimensions);
      heatMapCollection.alliesMaxHeatMap.updateHeat(takeNewPercent, alliesMaxHeat, index, theFieldDimensions);
      heatMapCollection.opponentsHeatMap.updateHeat(takeNewPercent, opponentsHeat, index, theFieldDimensions);
      heatMapCollection.opponentsMaxHeatMap.updateHeat(takeNewPercent, opponentsMaxHeat, index, theFieldDimensions);
    }

    if (goalKickHeatTakeNewPercent == 1.f)
    {
      heatMapCollection.alliesGoalKickHeatMap.setHeat(alliesGoalKickHeat, index, theFieldDimensions);
      heatMapCollection.opponentsGoalKickHeatMap.setHeat(opponentsGoalKickHeat, index, theFieldDimensions);
    }
    else
    {
      heatMapCollection.alliesGoalKickHeatMap.updateHeat(goalKickHeatTakeNewPercent, alliesGoalKickHeat, index, theFieldDimensions);
      heatMapCollection.opponentsGoalKickHeatMap.updateHeat(goalKickHeatTakeNewPercent, opponentsGoalKickHeat, index, theFieldDimensions);
    }
  }

  firstUpdate = false;
}

MAKE_MODULE(HeatMapProvider, modeling)
