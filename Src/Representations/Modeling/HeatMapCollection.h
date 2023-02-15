#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include <algorithm>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "HeatMap.h"

STREAMABLE(HeatMapCollection,

  HeatMapCollection()
  {

  }

  /** debug drawings, in world coordinate system. */
  void draw() const
  {
    /*
    DEBUG_DRAWING("representation:Ballchaser", "drawingOnField")
    {
      POSE_2D_SAMPLE("representation:Ballchaser", optPosition, ColorRGBA::blue);
      ELLIPSE("representation:Ballchaser", optPosition.translation,
              (thresholdXBack + thresholdXFront) / 2, thresholdY, optPosition.rotation, 5, Drawings::solidPen, ColorRGBA::red, Drawings::noBrush, ColorRGBA::red);
      ARROW("representation:Ballchaser", optPosition.translation.x(), optPosition.translation.y(), kickTarget.x(), kickTarget.y(), 5, Drawings::solidPen, ColorRGBA::red);
    }
    */
  }

  ,
  (HeatMap) sidesHeatMap,
  (HeatMap) opponentsGoalHeatMap,
  (HeatMap) alliesHeatMap,
  (HeatMap) alliesMaxHeatMap,
  (HeatMap) alliesGoalKickHeatMap,
  (HeatMap) opponentsHeatMap,
  (HeatMap) opponentsMaxHeatMap,
  (HeatMap) opponentsGoalKickHeatMap
);
