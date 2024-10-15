#pragma once

#include <optional>
#include <Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h>
#include <Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/CurrentKickManager.h>
#include <Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/KickManager.h>
#include "Tools/Module/Module.h"
#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/RemoteControl.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Settings.h"
#include "RoleProvider.h"
#include "Tools/ProcessFramework/CycleLocal.h"

MODULE(RemoteControlProvider,
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(RobotPose),
  PROVIDES(RemoteControl),

  LOADS_PARAMETERS(,
    (std::vector<std::string>) kickNames
  )
);

class RemoteControlProvider : public RemoteControlProviderBase, public RoleProvider<RemoteControl>
{

public:
  void update(RemoteControl& role) override;

private:
  void stateReady_kickOff_own(RemoteControl& role, const Vector2f& ballPosition) override;
  void stateReady_kickOff_opponent(RemoteControl& role, const Vector2f& ballPosition) override;
  float goalKick_own(RemoteControl& role, bool left) override;
  float goalKick_opponent(RemoteControl& role, bool left) override;
  float pushingFreeKick_own(RemoteControl& role) override;
  float pushingFreeKick_opponent(RemoteControl& role) override;
  float cornerKick_own(RemoteControl& role, const Vector2f& cornerKickPosition, bool left) override;
  float cornerKick_opponent(RemoteControl& role, const Vector2f& cornerKickPosition, bool left) override;
  float kickIn_own(RemoteControl& role, bool left) override;
  float kickIn_opponent(RemoteControl& role, bool left) override;
  float stateReady_penaltyKick_own(RemoteControl& role) override;
  float stateReady_penaltyKick_opponent(RemoteControl& role) override;
  void regularPlay(RemoteControl& role) override;

  RemoteControlRequest rcr;
  bool firstUpdate = true;
  bool enforceOffensive = false;
  bool enforceDefensive = false;
  bool passPref = false;
  bool kickPref = false;
  bool rcEnabled = false;

  static CycleLocal<RemoteControlProvider*> theInstance;

  std::vector<std::unique_ptr<Kick>> kicks;
  KickManager kickManager = {};

  bool handleMessage2(InMessage& message);

public:
  /*
  * Default constructor.
  */
  RemoteControlProvider()
  {
    theInstance = this;

    const auto kickEngineParameter = KicksProvider::loadKickEngineParameters();
    const auto customStepFiles = KicksProvider::loadCustomStepFiles();
    kicks = KicksProvider::createKicks(kickEngineParameter, customStepFiles, kickNames);
  };
  ~RemoteControlProvider() { theInstance.reset(); }

  static bool handleMessage(InMessage& message);
};
;
