#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h>
#include "RemoteControlProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Utils/PositionUtils.h"
#include "Utils/ThresholdUtils.h"
#include "Utils/BlockedUtils.h"
#include "Tools/MessageQueue/InMessage.h"

CycleLocal<RemoteControlProvider*> RemoteControlProvider::theInstance;

void RemoteControlProvider::update(RemoteControl& role)
{
  if (firstUpdate)
  {
    // init with random info to avoid crash
    kickManager.kickTo(role, theRobotPose, theBallSymbols.ballPositionFieldPredicted, {1000.f, 500.f}, true, KickUtils::unpack(kicks), theFrameInfo);
    firstUpdate = false;
  }

  ThresholdUtils::setThreshholdsLow(role);
  DECLARE_DEBUG_DRAWING("behavior:RemoteControlProvider", "drawingOnField");
  role.stopAtTarget = true;

  decide(role, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);


  role.enforceDefensiveRoles = enforceDefensive;
  role.enforceOffensiveRoles = enforceOffensive;
  role.kickPreference = kickPref;
  role.passPreference = passPref;
  role.enabled = rcEnabled;

  if (rcEnabled)
  {
    switch (rcr.command)
    {
    case RemoteControlRequest::Command::disable:
      rcEnabled = false;
      role.enabled = false;
      rcr.command = RemoteControlRequest::Command::stand;
      rcr.handled = true;
      break;
    case RemoteControlRequest::Command::stand:
      PositionUtils::setPosition(role, theRobotPose.translation.x(), theRobotPose.translation.y());
      if (!rcr.handled)
      {
        kickManager.deleteCurrentKick();
      }
      rcr.handled = true;
      break;
    case RemoteControlRequest::Command::walk:
      CROSS("behavior:RemoteControlProvider", rcr.target.translation.x(), rcr.target.translation.y(), 100, 50, Drawings::solidPen, ColorRGBA::blue);
      if (!rcr.handled)
      {
        kickManager.deleteCurrentKick();
      }
      role.optPosition = rcr.target;
      rcr.handled = true;
      break;
    case RemoteControlRequest::Command::pass:
      CROSS("behavior:RemoteControlProvider", rcr.target.translation.x(), rcr.target.translation.y(), 100, 50, Drawings::solidPen, ColorRGBA::green);
      LINE("behavior:RemoteControlProvider", theBallSymbols.ballPositionField.x(), theBallSymbols.ballPositionField.y(), rcr.target.translation.x(), rcr.target.translation.y(), 10, Drawings::solidPen, ColorRGBA::orange);
      if (!rcr.handled)
      {
        kickManager.deleteCurrentKick();
        rcr.handled = true;
      }
      kickManager.kickTo(role, theRobotPose, theBallSymbols.ballPositionFieldPredicted, rcr.target.translation, true, KickUtils::unpack(kicks), theFrameInfo);
      role.stopAtTarget = false;
      break;
    case RemoteControlRequest::Command::kick:
      CROSS("behavior:RemoteControlProvider", rcr.target.translation.x(), rcr.target.translation.y(), 100, 50, Drawings::solidPen, ColorRGBA::red);
      LINE("behavior:RemoteControlProvider", rcr.target.translation.x(), rcr.target.translation.y(), theBallSymbols.ballPositionField.x(), theBallSymbols.ballPositionField.y(), 10, Drawings::solidPen, ColorRGBA::orange);
      if (!rcr.handled)
      {
        kickManager.deleteCurrentKick();
        rcr.handled = true;
      }
      kickManager.kickTo(role, theRobotPose, theBallSymbols.ballPositionFieldPredicted, rcr.target.translation, false, KickUtils::unpack(kicks), theFrameInfo);
      role.stopAtTarget = false;
      break;
    }
  }
}

void RemoteControlProvider::stateReady_kickOff_own(RemoteControl& role, const Vector2f& ballPosition)
{
  regularPlay(role);
}

void RemoteControlProvider::stateReady_kickOff_opponent(RemoteControl& role, const Vector2f& ballPosition)
{
  regularPlay(role);
}

float RemoteControlProvider::goalKick_own(RemoteControl& role, bool left)
{
  regularPlay(role);
  return 0.f;
}

float RemoteControlProvider::goalKick_opponent(RemoteControl& role, bool left)
{
  regularPlay(role);
  return 0;
}

float RemoteControlProvider::pushingFreeKick_own(RemoteControl& role)
{
  regularPlay(role);
  return 0;
}

float RemoteControlProvider::pushingFreeKick_opponent(RemoteControl& role)
{
  regularPlay(role);
  return 0;
}

float RemoteControlProvider::cornerKick_own(RemoteControl& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float RemoteControlProvider::cornerKick_opponent(RemoteControl& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float RemoteControlProvider::kickIn_own(RemoteControl& role, bool left)
{
  regularPlay(role);
  return 0;
}

float RemoteControlProvider::kickIn_opponent(RemoteControl& role, bool left)
{
  regularPlay(role);
  return 0;
}

float RemoteControlProvider::stateReady_penaltyKick_own(RemoteControl& role)
{
  regularPlay(role);
  return 0;
}

float RemoteControlProvider::stateReady_penaltyKick_opponent(RemoteControl& role)
{
  regularPlay(role);
  return 0;
}

void RemoteControlProvider::regularPlay(RemoteControl& role)
{
  ThresholdUtils::setThreshholdsMedium(role);
}

bool RemoteControlProvider::handleMessage(InMessage& message)
{
  return *theInstance && (*theInstance)->handleMessage2(message);
}

bool RemoteControlProvider::handleMessage2(InMessage& message)
{
  if (message.getMessageID() == idRemoteControlRequest)
  {
    message.bin >> rcr;

    if (rcr.command == RemoteControlRequest::Command::enforceOffensiveRoles)
    {
      enforceDefensive = false;
      enforceOffensive = true;
    }
    if (rcr.command == RemoteControlRequest::Command::enforceDefensiveRoles)
    {
      enforceDefensive = true;
      enforceOffensive = false;
    }
    if (rcr.command == RemoteControlRequest::Command::kickPreference)
    {
      kickPref = true;
      passPref = false;
    }
    if (rcr.command == RemoteControlRequest::Command::passPreference)
    {
      kickPref = false;
      passPref = true;
    }

    if (!rcEnabled)
    {
      if (rcr.command == RemoteControlRequest::Command::enable)
      {
        rcEnabled = true;
        rcr.command = RemoteControlRequest::Command::stand;
      }
    }
    return true;
  }
  else
    return false;
}

MAKE_MODULE(RemoteControlProvider, behaviorControl)
