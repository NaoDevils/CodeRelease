
/**
* @file DangerMapProvider.h
* Declares a class that provides a map that contains information about the possible crowded/dangerous zones on the field
**
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/DangerMap.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/InStreams.h"
#include "Representations/Modeling/HeatMapCollection.h"

MODULE(HeatMapProvider,
  REQUIRES(BallSymbols),
  REQUIRES(BallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(RobotMap),
  REQUIRES(RobotPose),
  REQUIRES(TeammateData),
  HAS_PREEXECUTION,
  PROVIDES(HeatMapCollection),

  LOADS_PARAMETERS(,
    (float)(0.005f) kickHeatTakeNewPercent,
    (float)(0.025f) goalKickHeatTakeNewPercent
  )
);

class HeatMapProvider : public HeatMapProviderBase
{
public:
  HeatMapProvider();
  void execute(tf::Subflow&);
  void update(HeatMapCollection& heatMapCollection);

private:
  bool firstUpdate = true;
  HeatMap teammatesKickHeatMap;
  HeatMap teammatesGoalKickHeatMap;
  HeatMapCollection localHeatMapCollection;
};
