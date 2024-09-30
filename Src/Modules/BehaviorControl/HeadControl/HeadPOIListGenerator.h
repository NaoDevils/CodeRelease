/**
* @file HeadPOIListGenerator.h
*
* Declaration of class HeadPOIListGenerator, 
* calculates position to look at.
* 
* @author <a href="mailto:mahdokht.mohammadi@tu-dortmund.de">Mahdokht Mohammadi</a>
* @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
*
*/

#pragma once

#include "Tools/Module/Module.h"

// Include all referenced representations here.
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/BallchaserHeadPOIList.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/HeadPOIList.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/SpeedInfo.h"

MODULE(HeadPOIListGenerator,
  REQUIRES(BallSymbols),
  REQUIRES(BallchaserHeadPOIList),
  REQUIRES(BehaviorData),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(FallDownState),
  REQUIRES(HeadControlRequest),
  REQUIRES(RemoteBallModel),
  REQUIRES(RobotPose),
  REQUIRES(MotionRequest),
  REQUIRES(SpeedInfo),
  REQUIRES(GameSymbols),
  REQUIRES(RoleSymbols),

  USES(BallChaserDecision),

  PROVIDES(HeadPOIList),

  LOADS_PARAMETERS(,
    (std::array<Vector2a,2>)({Vector2a(-50_deg, 0_deg),Vector2a(50_deg,0_deg)}) localizeAngles,
    (std::array<Vector2a,2>)({Vector2a(-90_deg, 0_deg),Vector2a(90_deg,0_deg)}) goalieLocalizeAngles,
    (std::array<Vector2a,2>)({Vector2a(-45_deg, 0_deg),Vector2a(45_deg,0_deg)}) ballLostAngles,
    (std::array<Vector2a,2>)({Vector2a(-30_deg, 0_deg),Vector2a(30_deg,0_deg)}) ballSweepAngles
  )
);

class HeadPOIListGenerator : public HeadPOIListGeneratorBase
{
private:
  void update(HeadPOIList& headPOIList);

  void addSweep(HeadPOIList& headPOIList);
  void addBall(HeadPOIList& headPOIList);
  void addBallWithSweep(HeadPOIList& headPOIList);
  void addStraight(HeadPOIList& headPOIList);
  void addRemoteBall(HeadPOIList& headPOIList);
  void addPenaltyCross(HeadPOIList& headPOIList);
};
