#include "Modules/Modeling/BallSearchAreaProvider/BallSearchAreaProvider.h"

void BallSearchAreaProvider::update(BallSearchArea &ballSearchArea) {
  localBallSearchArea = ballSearchArea;
  /*if (theBallModel.estimate.position.x() < theFieldDimensions.xPosOwnGroundline / 2)
  {
    localBallSearchArea.gameSituation = BallSearchArea::GameSituation::Defensive;
  }
  else
  {
    localBallSearchArea.gameSituation = BallSearchArea::GameSituation::Moderat;
  }*/
  localBallSearchArea.gameSituation = BallSearchArea::GameSituation::Moderat;
  checkActivityOfTeamMates();
  determineArea();
  ballSearchArea = localBallSearchArea;
}

void BallSearchAreaProvider::checkActivityOfTeamMates()
{
  defenderActive = false;
  defSuppActive = false;
  offSuppActive = false;
  strikerActive = false;
  for (Teammate currentTeammate: theTeammateData.teammates)
  {
    if (currentTeammate.behaviorData.role == BehaviorData::RoleAssignment::defender && (currentTeammate.status == Teammate::ACTIVE || currentTeammate.status == Teammate::FULLY_ACTIVE))
    {
      defenderActive = true;
    }
    else if (currentTeammate.behaviorData.role == BehaviorData::RoleAssignment::supporterDef && (currentTeammate.status == Teammate::ACTIVE || currentTeammate.status == Teammate::FULLY_ACTIVE))
    {
      defSuppActive = true;
    }
    else if (currentTeammate.behaviorData.role == BehaviorData::RoleAssignment::supporterOff && (currentTeammate.status == Teammate::ACTIVE || currentTeammate.status == Teammate::FULLY_ACTIVE))
    {
      offSuppActive = true;
    }
    else if (currentTeammate.behaviorData.role == BehaviorData::RoleAssignment::striker && (currentTeammate.status == Teammate::ACTIVE || currentTeammate.status == Teammate::FULLY_ACTIVE))
    {
      strikerActive = true;
    }
  }
}

//determines the bounds of the ball search area for this robot based on the game situation and the active players
void BallSearchAreaProvider::determineArea()
{
  
  switch (localBallSearchArea.gameSituation)
  {
  case BallSearchArea::GameSituation::Defensive:
    determineAreaDefensive();
    break;
  case BallSearchArea::GameSituation::Moderat:
    determineAreaModerat();
    break;
  case BallSearchArea::GameSituation::Offensive:
    break;
  default:
    break;
  }
}

void BallSearchAreaProvider::determineAreaModerat()
{
  switch (theRoleSymbols.role)
  {
  case BehaviorData::RoleAssignment::defender:
    determineAreaModeratDefender();
    break;
  case BehaviorData::RoleAssignment::supporterDef:
    determineAreaModeratDefSupp();
    break;
  case BehaviorData::RoleAssignment::supporterOff:
    determineAreaModeratOffSupp();
    break;
  case BehaviorData::RoleAssignment::striker:
    determineAreaModeratStriker();
    break;
  default:
    break;
  }
}

void BallSearchAreaProvider::determineAreaDefensive()
{
  switch (theRoleSymbols.role)
  {
  case BehaviorData::RoleAssignment::defender:
    determineAreaDefensiveDefender();
    break;
  case BehaviorData::RoleAssignment::supporterDef:
    determineAreaDefensiveDefSupp();
    break;
  case BehaviorData::RoleAssignment::supporterOff:
    determineAreaDefensiveOffSupp();
    break;
  case BehaviorData::RoleAssignment::striker:
    determineAreaDefensiveStriker();
    break;
  default:
    break;
  }
}

void BallSearchAreaProvider::determineAreaOffensive()
{
  switch (theRoleSymbols.role)
  {
  case BehaviorData::RoleAssignment::supporterDef:
    determineAreaOffensiveDefSupp();
    break;
  case BehaviorData::RoleAssignment::supporterOff:
    determineAreaOffensiveOffSupp();
    break;
  case BehaviorData::RoleAssignment::striker:
    determineAreaOffensiveStriker();
    break;
  default:
    break;
  }
}

void BallSearchAreaProvider::determineAreaModeratDefender()
{
  if (offSuppActive || defSuppActive || strikerActive)
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea;
    localBallSearchArea.upperX = theFieldDimensions.xPosOwnPenaltyArea + 2500;
    localBallSearchArea.lowerY = theFieldDimensions.yPosRightPenaltyArea;
    localBallSearchArea.upperY = theFieldDimensions.yPosLeftPenaltyArea;
  } 
  else
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea;
    localBallSearchArea.upperX = theFieldDimensions.xPosOpponentGroundline;
    localBallSearchArea.lowerY = theFieldDimensions.yPosRightSideline;
    localBallSearchArea.upperY = theFieldDimensions.yPosLeftSideline;
  }
}

void BallSearchAreaProvider::determineAreaModeratDefSupp()
{
  if (offSuppActive && strikerActive) 
  {
    localBallSearchArea.upperX = theFieldDimensions.xPosOwnPenaltyArea + (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOwnGroundline) / 3;
  }
  else if (offSuppActive || strikerActive)
  {
    localBallSearchArea.upperX = theFieldDimensions.xPosHalfWayLine;
  } 
  else
  {
    localBallSearchArea.upperX = theFieldDimensions.xPosOpponentGroundline;
  }
  localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea;
  localBallSearchArea.lowerY = theFieldDimensions.yPosRightSideline;
  localBallSearchArea.upperY = theFieldDimensions.yPosLeftSideline;
}

void BallSearchAreaProvider::determineAreaModeratOffSupp()
{
  bool defSuppActive = false;
  bool strikerActive = false;
  for (Teammate currentTeammate: theTeammateData.teammates)
  {
    if (currentTeammate.behaviorData.role == BehaviorData::RoleAssignment::supporterDef && (currentTeammate.status == Teammate::ACTIVE || currentTeammate.status == Teammate::FULLY_ACTIVE))
    {
      defSuppActive = true;
    }
    else if (currentTeammate.behaviorData.role == BehaviorData::RoleAssignment::striker && (currentTeammate.status == Teammate::ACTIVE || currentTeammate.status == Teammate::FULLY_ACTIVE))
    {
      strikerActive = true;
    }
  }
  if (defSuppActive && strikerActive)
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea + (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOwnGroundline) / 3;
    localBallSearchArea.upperX = localBallSearchArea.lowerX + (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOwnGroundline) / 3;
  }
  else if (defSuppActive)
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosHalfWayLine;
    localBallSearchArea.upperX = theFieldDimensions.xPosOpponentGroundline;
  }
  else if (strikerActive)
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea;
    localBallSearchArea.upperX = theFieldDimensions.xPosHalfWayLine;
  }
  else {
    localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea;
    localBallSearchArea.upperX = theFieldDimensions.xPosOpponentGroundline;
  }
  localBallSearchArea.lowerY = theFieldDimensions.yPosRightSideline;
  localBallSearchArea.upperY = theFieldDimensions.yPosLeftSideline;
}

void BallSearchAreaProvider::determineAreaModeratStriker()
{
  if (defSuppActive && offSuppActive)
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosOpponentGroundline - (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOwnGroundline) / 3;
  }
  else if (defSuppActive || offSuppActive)
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosHalfWayLine;
  }
  else
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea;
  }
  localBallSearchArea.upperX = theFieldDimensions.xPosOpponentGroundline;
  localBallSearchArea.lowerY = theFieldDimensions.yPosRightSideline;
  localBallSearchArea.upperY = theFieldDimensions.yPosLeftSideline;
}

void BallSearchAreaProvider::determineAreaDefensiveDefender()
{
  bool defSuppActive = false;
  bool offSuppActive = false;
  for (Teammate currentTeammate: theTeammateData.teammates)
  {
    if (currentTeammate.behaviorData.role == BehaviorData::RoleAssignment::supporterDef && (currentTeammate.status == Teammate::ACTIVE || currentTeammate.status == Teammate::FULLY_ACTIVE))
    {
      defSuppActive = true;
    }
    if (currentTeammate.behaviorData.role == BehaviorData::RoleAssignment::supporterOff && (currentTeammate.status == Teammate::ACTIVE || currentTeammate.status == Teammate::FULLY_ACTIVE))
    {
      offSuppActive = true;
    }
  }
  if (defSuppActive || offSuppActive) 
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea;
    localBallSearchArea.upperX = theFieldDimensions.xPosOwnPenaltyArea + 2500;
    localBallSearchArea.lowerY = theFieldDimensions.yPosRightPenaltyArea;
    localBallSearchArea.upperY = theFieldDimensions.yPosLeftPenaltyArea;
  }
  else
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea;
    localBallSearchArea.upperX = theFieldDimensions.xPosHalfWayLine;
    localBallSearchArea.lowerY = theFieldDimensions.yPosRightSideline;
    localBallSearchArea.upperY = theFieldDimensions.yPosLeftSideline;
  }
  
}

void BallSearchAreaProvider::determineAreaDefensiveDefSupp()
{
  if (offSuppActive && strikerActive)
  {
    localBallSearchArea.upperX = theFieldDimensions.xPosHalfWayLine;
    localBallSearchArea.lowerY = theFieldDimensions.yPosKickOffPoint;
  }
  else if(offSuppActive || strikerActive)
  {
    localBallSearchArea.upperX = theFieldDimensions.xPosHalfWayLine;
    localBallSearchArea.lowerY = theFieldDimensions.yPosRightSideline;
  }
  else 
  {
    localBallSearchArea.upperX = theFieldDimensions.xPosOpponentGroundline;
    localBallSearchArea.lowerY = theFieldDimensions.yPosRightSideline;
  }
  localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea;
  localBallSearchArea.upperY = theFieldDimensions.yPosLeftSideline;
  
}

void BallSearchAreaProvider::determineAreaDefensiveOffSupp()
{
  if (defSuppActive && strikerActive)
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea;
    localBallSearchArea.upperX = theFieldDimensions.xPosHalfWayLine;
    localBallSearchArea.upperY = theFieldDimensions.yPosKickOffPoint;
  }
  else if (defSuppActive)
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosHalfWayLine;
    localBallSearchArea.upperX = theFieldDimensions.xPosOpponentGroundline;
    localBallSearchArea.upperY = theFieldDimensions.yPosLeftSideline;
  }
  else if(strikerActive)
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea;
    localBallSearchArea.upperX = theFieldDimensions.xPosHalfWayLine;
    localBallSearchArea.upperY = theFieldDimensions.yPosLeftSideline;
  }
  else
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea;
    localBallSearchArea.upperX = theFieldDimensions.xPosOpponentGroundline;
    localBallSearchArea.lowerY = theFieldDimensions.yPosRightSideline;
  }
  localBallSearchArea.lowerY = theFieldDimensions.yPosRightSideline;
}

void BallSearchAreaProvider::determineAreaDefensiveStriker()
{
  if (defSuppActive || offSuppActive)
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosHalfWayLine;
    localBallSearchArea.upperX = theFieldDimensions.xPosOpponentGroundline;
  } 
  else
  {
    localBallSearchArea.lowerX = theFieldDimensions.xPosOwnPenaltyArea;
    localBallSearchArea.upperX = theFieldDimensions.xPosOpponentGroundline;
  }
  localBallSearchArea.lowerY = theFieldDimensions.yPosRightSideline;
  localBallSearchArea.upperY = theFieldDimensions.yPosLeftSideline;
}

void BallSearchAreaProvider::determineAreaOffensiveDefSupp()
{
}

void BallSearchAreaProvider::determineAreaOffensiveOffSupp()
{
}

void BallSearchAreaProvider::determineAreaOffensiveStriker()
{
}


MAKE_MODULE(BallSearchAreaProvider, modeling)
