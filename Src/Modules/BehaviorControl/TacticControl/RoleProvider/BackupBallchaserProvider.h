/**
* @file BackupBallchaserProvider.h
*
* Declaration of class BackupBallchaserProvider.
* Provides positions for the BackupBallchaser in PLAY and READY.
* All positions are in world coordinates with a positioning angle in degrees (for CABSL).
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/BackupBallchaser.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(BackupBallchaserProvider,
  REQUIRES(BallSymbols),
  REQUIRES(Ballchaser),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeammateData),
  PROVIDES(BackupBallchaser),
  LOADS_PARAMETERS(,
    (float)(1000.f) xDistanceToBallchaser
  )
);


/**
* @class BackupBallchaserProvider
* Symbols for new role behavior 2019
*/

class BackupBallchaserProvider : public BackupBallchaserProviderBase
{
public:
  /** Constructor */
  BackupBallchaserProvider() {}

  /** Updates some of the symbols */
  void update(BackupBallchaser& positioningSymbols);


private:
  /** Get fixed position during ready state. */
  void getReadyPosition(BackupBallchaser& positioningSymbols);

  /** Get position behind ballchaser. */
  void getFollowPosition(BackupBallchaser& positioningSymbols);

  /** If the ballchaser gets too close to own groundline move to the side to avoid being pushed of the field. */
  void avoidOwnGroundline(BackupBallchaser& positioningSymbols);

  /** Get position for Set Plays. */
  void getSetPlayPosition(BackupBallchaser& positioningSymbols);

  // 1 => avoid to the left; -1 => avoid to the right; 0 => no side chosen yet
  float sideChosenForAvoidingGroundline = 0.f;
  bool tooCloseToGroundline = false;
  bool ballchaserInFieldMiddle = true;
  bool ballchaserOnLeftFieldHalf = true;
  bool wasInSetPlay = false;
  bool setPlaySupportLeft = false;
};
