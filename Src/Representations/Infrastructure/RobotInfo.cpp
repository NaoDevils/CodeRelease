/**
 * @file RobotInfo.h
 * The file declares a struct that encapsulates the structure RobotInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * It also maps the robot's name on the robot's model.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <a href="mailto:aschreck@informatik.uni-bremen.de">André Schreck</a>
 */

#include "RobotInfo.h"
#include <cstring>
#include "Tools/Global.h"
#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/GameInfo.h"

RobotInfo::RobotInfo()
{
  memset((RoboCup::RobotInfo*)this, 0, sizeof(RoboCup::RobotInfo));
}

const char* RobotInfo::getPenaltyAsString() const
{
  switch (penalty)
  {
  case PENALTY_SPL_ILLEGAL_BALL_CONTACT:
    return "Illegal Ball Contact";
  case PENALTY_SPL_PLAYER_PUSHING:
    return "Pushing";
  case PENALTY_SPL_ILLEGAL_MOTION_IN_SET:
    return "Motion in Set";
  case PENALTY_SPL_INACTIVE_PLAYER:
    return "Fallen/Inactive Robot";
  case PENALTY_SPL_ILLEGAL_POSITION:
    return "Illegal Position";
  case PENALTY_SPL_LEAVING_THE_FIELD:
    return "Leaving the Field";
  case PENALTY_SPL_REQUEST_FOR_PICKUP:
    return "Request for Pickup";
  case PENALTY_SPL_LOCAL_GAME_STUCK:
    return "Local Game Stuck";
  case PENALTY_SPL_ILLEGAL_POSITION_IN_SET:
    return "Illegal Position in Set";
  case PENALTY_SPL_PLAYER_STANCE:
    return "Player Stance";
  case PENALTY_SUBSTITUTE:
    return "Substitute";
  case PENALTY_MANUAL:
    return "Manually Penalized";
  default:
    return "Unpenalized";
  }
}

Streamable& RobotInfo::operator=(const Streamable& other) noexcept
{
  return *this = dynamic_cast<const RobotInfo&>(other);
}

void RobotInfo::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(number); // robot number: 1..11
  STREAM(transitionToFramework);
  STREAM(penalty); // PENALTY_NONE, PENALTY_BALL_HOLDING, ...
  STREAM(secsTillUnpenalised); // estimate of time till unpenalised.
  STREAM_REGISTER_FINISH;
}
