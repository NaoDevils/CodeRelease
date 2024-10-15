/**
* @file BackupBallchaserProvider.cpp
*
* Implementation of class BackupBallchaserProvider.
*
*/

#include "BackupBallchaserProvider.h"

void BackupBallchaserProvider::update(BackupBallchaser& positioningSymbols)
{
  if (theGameInfo.state != STATE_PLAYING)
    getReadyPosition(positioningSymbols);
  else
  {
    if (theGameInfo.setPlay != SET_PLAY_NONE)
      getSetPlayPosition(positioningSymbols);
    else
    {
      // attention: current implementation of these functions only works if called right after each other
      getFollowPosition(positioningSymbols);
      avoidOwnGroundline(positioningSymbols);
    }
  }
}

/**
* \brief Get fixed position during ready state.
*/
void BackupBallchaserProvider::getReadyPosition(BackupBallchaser& positioningSymbols)
{
  // stand further back if oppenent team has kick off (ballchaser not in center circle)
  positioningSymbols.optPosition.translation.x() = -theFieldDimensions.centerCircleRadius - (theGameSymbols.ownKickOff ? 300.f : 1000.f);
  positioningSymbols.optPosition.translation.y() = 0.f;
  positioningSymbols.optPosition.rotation = 0_deg;
  positioningSymbols.stopAtTarget = true;
  positioningSymbols.previewArrival = true;
}

/**
* \brief Get position behind ballchaser.
*/
void BackupBallchaserProvider::getFollowPosition(BackupBallchaser& positioningSymbols)
{
  // assume ballchaser always positions in front of the ball
  Vector2f ballChaserPosition = theBallchaser.optPosition.translation;
  const Teammate* teammate = theTeammateData.getPlayer(static_cast<uint8_t>(theBallChaserDecision.playerNumberToBall));
  if (teammate)
    ballChaserPosition = teammate->robotPose.translation;

  positioningSymbols.optPosition.translation = ballChaserPosition;
  Angle toBallRotation = (theBallSymbols.ballPositionField - theRobotPose.translation).angle();
  positioningSymbols.optPosition.rotation = toBallRotation;
  // back a little
  positioningSymbols.optPosition.translation.x() -= xDistanceToBallchaser;
  // make sure not to get too close to the field lines
  positioningSymbols.optPosition.translation.y() = std::min(positioningSymbols.optPosition.translation.y(), 0.8f * theFieldDimensions.yPosLeftSideline);
  positioningSymbols.optPosition.translation.y() = std::max(positioningSymbols.optPosition.translation.y(), 0.8f * theFieldDimensions.yPosRightSideline);
  positioningSymbols.stopAtTarget = true;
  positioningSymbols.previewArrival = true;
}

/**
* \brief If the backupBallchaser position gets to close to the own groundline overwrite it with alternative.
*
* The x component of the optPositions gets limited to minimal value. Also the optPosition gets translated to the side
* to avoid the ballchaser. Which side to avoid to is decided based on the ballchasers optPosition. Once a side is chosen
* it will not be changed as long as backupBallchaser is too close to own groundline to avoid a jump of the optPosition 
* and possible collisions when repositioning.
*/
void BackupBallchaserProvider::avoidOwnGroundline(BackupBallchaser& positioningSymbols)
{
  // hysteresis for stable decision
  float factorGroundline = 0.5f - (tooCloseToGroundline ? 0.2f : 0.f);
  tooCloseToGroundline = positioningSymbols.optPosition.translation.x() < factorGroundline * theFieldDimensions.xPosOwnGroundline;
  if (tooCloseToGroundline)
  {
    // decide which side to move when avoiding the groundline only if you have not decided before
    if (sideChosenForAvoidingGroundline == 0.f)
    {
      // hysteresis for stable decision
      float factorMiddle = ballchaserInFieldMiddle ? 0.6f : 0.4f;
      ballchaserInFieldMiddle = std::abs(theBallchaser.optPosition.translation.y()) < factorMiddle * theFieldDimensions.yPosLeftSideline;
      // hysteresis for stable decision
      float factorLeftHalf = ballchaserOnLeftFieldHalf ? -0.2f : 0.2f;
      ballchaserOnLeftFieldHalf = theBallchaser.optPosition.translation.y() > factorLeftHalf * theFieldDimensions.yPosLeftSideline;
      bool moveToLeft = (ballchaserInFieldMiddle && ballchaserOnLeftFieldHalf) || (!ballchaserInFieldMiddle && !ballchaserOnLeftFieldHalf);
      sideChosenForAvoidingGroundline = moveToLeft ? 1.f : -1.f;
    }
    // fix x component to avoid running of the field
    positioningSymbols.optPosition.translation.x() = 0.5f * theFieldDimensions.xPosOwnGroundline;
    // values of sideChosenForAvoidingGroundline: 1 => moveLeft, -1 => moveRight, 0 => no decision made
    positioningSymbols.optPosition.translation.y() += sideChosenForAvoidingGroundline * 0.3f * theFieldDimensions.yPosLeftSideline;
    // limit y component to avoid leaving the field to far
    positioningSymbols.optPosition.translation.y() = std::max(1.1f * theFieldDimensions.yPosRightSideline, positioningSymbols.optPosition.translation.y());
    positioningSymbols.optPosition.translation.y() = std::min(1.1f * theFieldDimensions.yPosLeftSideline, positioningSymbols.optPosition.translation.y());
  }
  else
    // reset decision
    sideChosenForAvoidingGroundline = 0.f;
}

void BackupBallchaserProvider::getSetPlayPosition(BackupBallchaser& positioningSymbols)
{
  if (theGameSymbols.ownKickOff)
  {
    getFollowPosition(positioningSymbols);
    avoidOwnGroundline(positioningSymbols);
  }
  else
  {
    if (!wasInSetPlay)
      setPlaySupportLeft = theBallchaser.optPosition.translation.y() < theRobotPose.translation.y();
    positioningSymbols.optPosition.translation = theBallchaser.optPosition.translation;
    positioningSymbols.optPosition.translation.y() += setPlaySupportLeft ? 500.f : -500.f;
    // limit y component to avoid leaving the field to far
    positioningSymbols.optPosition.translation.y() = std::max(1.1f * theFieldDimensions.yPosRightSideline, positioningSymbols.optPosition.translation.y());
    positioningSymbols.optPosition.translation.y() = std::min(1.1f * theFieldDimensions.yPosLeftSideline, positioningSymbols.optPosition.translation.y());
    wasInSetPlay = true;
  }
}

MAKE_MODULE(BackupBallchaserProvider, behaviorControl)
