#pragma once

#include "../../../Configuration/FieldDimensions.h"
#include "../../../../Tools/Debugging/DebugDrawings.h"
#include "../../../../Tools/Debugging/DebugDrawings3D.h"
#include "../../../../Tools/Streams/AutoStreamable.h"
#include "../../../../Tools/Math/Eigen.h"
#include <algorithm>
#include "../../../../Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "HeatMap.h"
#include "SplitHeatMap.h"

STREAMABLE(PositionInfo,

  PositionInfo()
  {

  }

  void draw(const FieldDimensions& theFieldDimensions)
  {
      HeatMapFunctions::drawArrows(goalsHeatMap.heatVector, theFieldDimensions);
      //goalsHeatMap.draw(false, theFieldDimensions);
  }

  bool needInitialUpdate = true;

  ,

  (HeatMap) goalsHeatMap,
  (HeatMap) sidesHeatMap,

  (HeatMap) selfHeatMap,
  (SplitHeatMap) selfToGoalHeatMap,
  (HeatMap) teamHeatMap,
  (SplitHeatMap) teamToGoalHeatMap,
  (HeatMap) opponentHeatMap,
  (SplitHeatMap) opponentToGoalHeatMap,

  (HeatMap) crowdedHeatMap
);
