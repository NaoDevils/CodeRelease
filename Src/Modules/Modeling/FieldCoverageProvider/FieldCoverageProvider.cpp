#include "FieldCoverageProvider.h"
#include "Tools/Math/Geometry.h"


FieldCoverageProvider::FieldCoverageProvider()
{

}

void FieldCoverageProvider::resetAllFieldCoverages()
{
  interestingCells.reset();
  seenCells.reset();
  localFieldCoverage.reset();
}

void FieldCoverageProvider::checkBallLostThisFrame()
{
  ballLostThisFrame =  (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)>timeThreshholdforBallLoss)
                            && (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeenByTeamMate)>timeThreshholdforBallLoss)
                            && (theBallModel.timeWhenLastSeen>timeWhenBallLost+timeThreshholdforBallLoss)
                            && (  (theBallModel.timeWhenLastSeenByTeamMate>timeWhenBallLost+timeThreshholdforBallLoss)
                                  || (theTeammateData.numberOfActiveTeammates == 0));

  if(ballLostThisFrame)
  {
    timeWhenBallLost = theFrameInfo.time;
  }
}

void FieldCoverageProvider::update(FieldCoverage &fieldCoverage)
{
  DECLARE_DEBUG_DRAWING("representation:FieldCoverage", "drawingOnField");

  if (theBehaviorConfiguration.behavior2015Parameters.useBallSearch2017)
  {

    checkBallLostThisFrame();

    /*bool ballWasSeenThisFrame = (theBallModel.timeWhenLastSeen == theFrameInfo.time)
                                || (theBallModel.timeWhenLastSeenByTeamMate == theFrameInfo.time);*/
    if (ballLostThisFrame)
    {
      resetAllFieldCoverages();
      localFieldCoverage.interestingCellInSearchArea = localFieldCoverage.getCellNumber(theTeamBallModel.position, theFieldDimensions);
    }
    else
    {
      findInterestingPoints();

      updateValidity();
    }

    combineCells();

    determineInterestingCellInSearchArea();

    fieldCoverage = localFieldCoverage;

  }

  DEBUG_RESPONSE("debug drawing:representation:FieldCoverage")
  {
    drawFieldCoverage();
  }
}

bool FieldCoverageProvider::isViewBlocked(const RobotMap &robotMap, const Vector2f &pCellOnField, const Pose2f &pose, const float &camAngle)
{
  Pose2f camPose(camAngle, pose.translation);
  Vector2f pRelative = Transformation::fieldToRobot(camPose, pCellOnField);
  float pAngle = pRelative.angle();
  float pDistance = pRelative.norm();
  if (std::abs(pAngle) > 30_deg) return false;

  for (auto &robot : robotMap.robots)
  {
    Vector2f rRelative = Transformation::fieldToRobot(camPose, robot.pose.translation);
    float rDistance = (rRelative).norm();
    float rAngle = rRelative.angle();
    if (rDistance < robotRadius)
      continue;
    if (pDistance > rDistance
      && std::abs(Angle::normalize(pAngle - rAngle)) < std::asin(robotRadius / rDistance))
      return true;
  }
  return false;
}


bool FieldCoverageProvider::containsFieldLines(const FieldCoverage &fieldCoverage, int cellNumber)
{
  Vector2f fieldCoords = getFieldCoordinates(cellNumber);
  const float maxDistToCellCenter = FieldCoverage::stepSize / 2 + 50;
  if (fieldCoords.x() + maxDistToCellCenter > theFieldDimensions.xPosOpponentGroundline // opp ground line
    || fieldCoords.x() - maxDistToCellCenter < theFieldDimensions.xPosOwnGroundline // own ground line
    || fieldCoords.y() + maxDistToCellCenter > theFieldDimensions.yPosLeftSideline // left side line
    || fieldCoords.y() - maxDistToCellCenter < theFieldDimensions.yPosRightSideline // right side line
    || std::abs(fieldCoords.x()) < maxDistToCellCenter // center line
    || std::abs(fieldCoords.norm() - theFieldDimensions.centerCircleRadius) < maxDistToCellCenter // center circle
    || (std::abs(std::abs(fieldCoords.y()) - theFieldDimensions.yPosLeftPenaltyArea) < maxDistToCellCenter
      && std::abs(fieldCoords.x()) >= theFieldDimensions.xPosOpponentPenaltyArea)
    || (std::abs(fieldCoords.y()) <= theFieldDimensions.yPosLeftPenaltyArea
      && std::abs(std::abs(fieldCoords.x()) - theFieldDimensions.xPosOpponentPenaltyArea) < maxDistToCellCenter))
    return true;

  return false;
}

void FieldCoverageProvider::determineOptimalArea() {
  centerOfOptimalArea = thePositioningSymbols.optPosition.translation;
}

void FieldCoverageProvider::makeOptimalAreaInteresting()
{
  for (int x = 0; x < FieldCoverage::numOfCellsX; x++)
  {
    for (int y = 0; y < FieldCoverage::numOfCellsY; y++)
    {
      int i = x*FieldCoverage::numOfCellsY + y;
      Vector2f fieldPositionOfCell = getFieldCoordinates(i);
      float distanceToOptimalArea = (centerOfOptimalArea - fieldPositionOfCell).norm();
      //TODO: 0.5 and 0.00025 to params
      float importance = (float)(0.8 - distanceToOptimalArea * 0.0004);
      setImportanceOfCell(fieldPositionOfCell, std::max(0.f, importance));
    }
  }
}

/**
 * @brief FieldCoverageProvider::findInterestingPoints
 * defines points of interest based on different criteria
 * * last percept
 */
void FieldCoverageProvider::findInterestingPoints(){

  if(theFrameInfo.getTimeSince(timeWhenBallLost) < ballPosRememberingTime)
  {
    addInterestingCell(Transformation::robotToField(theRobotPose,theBallModel.lastPerception),1.f);
    setProtection(getCellNumber(Transformation::robotToField(theRobotPose,theBallModel.lastPerception)),1.f);
  }
  // Ball out
  //bool ballOutByOwnTeam = (theFrameInfo.getTimeSince(theGameInfo.dropInTime) < 500) && (theGameInfo.dropInTeam == theOwnTeamInfo.teamNumber);
  //bool ballOutByOpponentTeam = (theFrameInfo.getTimeSince(theGameInfo.dropInTime) < 500) && (theGameInfo.dropInTeam != theOwnTeamInfo.teamNumber);
  bool ballOut =((theGameInfo.dropInTime < 2) && (theGameInfo.dropInTime > 0));
  if(ballOut)
  {
    for(float xPos = theFieldDimensions.xPosOwnDropInLine; xPos <= theFieldDimensions.xPosOpponentDropInLine; xPos+= FieldCoverage::stepSize/2.f)
    {
      addInterestingCell(Vector2f(xPos,theFieldDimensions.yPosRightDropInLine),1.f);
      addInterestingCell(Vector2f(xPos,theFieldDimensions.yPosLeftDropInLine),1.f);
    }
  }

  //add search points as interestin points
  addSearchPoints();

  // reset in set
  if(theGameInfo.state == STATE_SET)
  {
    localFieldCoverage.reset();
    interestingCells.reset();
    seenCells.reset();
  }
}

void FieldCoverageProvider::addSearchPoints()
{
  std::vector<Vector2f> searchPositions;
  searchPositions.push_back(Vector2f(-3725, 2250));
  searchPositions.push_back(Vector2f(-1000, 2250));
  searchPositions.push_back(Vector2f(1000, 2250));
  searchPositions.push_back(Vector2f(3275, 2250));
  searchPositions.push_back(Vector2f(-2725, 1225));
  searchPositions.push_back(Vector2f(2725, 1225));
  searchPositions.push_back(Vector2f(-3000, 0));
  searchPositions.push_back(Vector2f(-2000, 0));
  searchPositions.push_back(Vector2f(0, 0));
  searchPositions.push_back(Vector2f(2000, 0));
  searchPositions.push_back(Vector2f(3000, 0));
  searchPositions.push_back(Vector2f(-2725, -1260));
  searchPositions.push_back(Vector2f(2725, -1260));
  searchPositions.push_back(Vector2f(-3725, -2250));
  searchPositions.push_back(Vector2f(-1000, -2250));
  searchPositions.push_back(Vector2f(1000, -2250));
  searchPositions.push_back(Vector2f(3725, -2250));
  int stillImportantCells = 0;
  for (int i = 0; i < (int) searchPositions.size(); i++)
  {
    int cellNumber = getCellNumber(searchPositions[i]);
    bool overLowerX = searchPositions[i].x() >= theBallSearchArea.lowerX;
    bool underUpperX = searchPositions[i].x() <= theBallSearchArea.upperX;
    bool overLowerY = searchPositions[i].y() >= theBallSearchArea.lowerY;
    bool underUpperY = searchPositions[i].y() <= theBallSearchArea.upperY;
    if (interestingCells.cells[cellNumber].validity > 0.5f && overLowerX && underUpperX && overLowerY && underUpperY) //TODO magic number to config file
    {
      stillImportantCells++;
    }
  }
  if(stillImportantCells <= 1)
  {
    for (int i = 0; i < (int) searchPositions.size(); i++)
    {
      setImportanceOfCell(searchPositions[i], 1.f);
      setProtection(getCellNumber(searchPositions[i]), 0.99f);
    }
  }
  
}

void FieldCoverageProvider::updateValidity()
{
  for (int x = 0; x < FieldCoverage::numOfCellsX; x++)
  {
    for (int y = 0; y < FieldCoverage::numOfCellsY; y++)
    {
      int cellNo = x*FieldCoverage::numOfCellsY+y;
      // TODO: global/local decision to behavior?
      float visibility = getPointVisibility(getFieldCoordinates(cellNo),false);
      if (visibility > 0)
      {
        seenCells.cells[cellNo].validity -= validityUpdateLoss;
        if (containsFieldLines(seenCells, cellNo)) 
        {
          //Cells with lines have a lower validity gain so that they are looked at for a longer time
          seenCells.cells[cellNo].validity = std::min(1.f, seenCells.cells[cellNo].validity + visibility * (validityGainWhenSeen / 2));
        }
        else 
        {
          seenCells.cells[cellNo].validity = std::min(1.f, seenCells.cells[cellNo].validity + visibility * validityGainWhenSeen);
        }
        interestingCells.cells[cellNo].validity =
            std::max(0.f,interestingCells.cells[cellNo].validity*(1.f-seenCells.cells[cellNo].validity) - interestUpdateLoss*(1.f - getProtection(cellNo)));
      }
      else{
        seenCells.cells[cellNo].validity =
            std::max(0.f,seenCells.cells[cellNo].validity - validityUpdateLoss);
        interestingCells.cells[cellNo].validity =
            std::max(0.f,interestingCells.cells[cellNo].validity - interestUpdateLoss*(1.f - getProtection(cellNo)));
      }
    }
  }
}

// adds importance to a cell in interestingCells
void FieldCoverageProvider::addInterestingCell(const Vector2f &pos, float importance)
{
  int cellIndex = interestingCells.getCellNumber(pos, theFieldDimensions);
  interestingCells.cells[cellIndex].validity = std::min(1.f,interestingCells.cells[cellIndex].validity+importance);// TODO perhaps influence adjacend cells; optional: use fuzzy max-norm
}

// sets the importance of a cell to a given value if that value is higher than the current importance
void FieldCoverageProvider::setImportanceOfCell(const Vector2f &pos, float importance) {
  int cellIndex = interestingCells.getCellNumber(pos, theFieldDimensions);
  float currentImportance = interestingCells.cells[cellIndex].validity;
  if (currentImportance < importance)
  {
    if (importance >= -1.f) {
      interestingCells.cells[cellIndex].validity = std::min(1.f, importance);
    }
    
  }
}

// combines seenCells and interestingCells into localFieldCoverage in a way, that the higher the value the more the cell is interesting to look at
void FieldCoverageProvider::combineCells()
{
  for (int x = 0; x < FieldCoverage::numOfCellsX; x++)
  {
    for (int y = 0; y < FieldCoverage::numOfCellsY; y++)
    {
      int i = x*FieldCoverage::numOfCellsY+y;
      localFieldCoverage.cells[i].validity = interestingCells.cells[i].validity - seenCells.cells[i].validity;
    }
  }
}

// TODO: ..depending on intention (looking for ball or maybe for obstacles), different method!?
int FieldCoverageProvider::getInterestingCell(const RobotPose &pose)
{
  return localFieldCoverage.getInterestingCell(theRobotPose, theFieldDimensions, 0.f);
}

bool FieldCoverageProvider::isCellInSearchArea(int cellNumber) {
  Vector2f interestingCellCoordinates = localFieldCoverage.getFieldCoordinates(cellNumber, theFieldDimensions);
  if (interestingCellCoordinates.x() >= theBallSearchArea.lowerX
    && interestingCellCoordinates.x() <= theBallSearchArea.upperX
    && interestingCellCoordinates.y() >= theBallSearchArea.lowerY
    && interestingCellCoordinates.y() <= theBallSearchArea.upperY)
  {
    return true;
  }
  return false;
}

void FieldCoverageProvider::determineInterestingCellInSearchArea()
{
  
  int interestingCell = localFieldCoverage.interestingCellInSearchArea;
  int robotCell = localFieldCoverage.getCellNumber(theRobotPose.translation, theFieldDimensions);
  float distanceRobotToCell = localFieldCoverage.getCellDistance(interestingCell, robotCell, theFieldDimensions);
  float heightSearchArea = (theBallSearchArea.upperX - theBallSearchArea.lowerX);
  float widthSearchArea = (theBallSearchArea.upperY - theBallSearchArea.lowerY);
  float xPosSearchAreaCenter = theBallSearchArea.lowerX + heightSearchArea / 2.f;
  float yPosSearchAreaCenter = theBallSearchArea.lowerY + widthSearchArea / 2.f;
  Vector2f centerSearchArea(xPosSearchAreaCenter, yPosSearchAreaCenter);
  int newInterestingCell;

  if (interestingCell < 0 || interestingCell >= FieldCoverage::numOfCells)
  {
    newInterestingCell = localFieldCoverage.getInterestingCell(theRobotPose, theFieldDimensions, centerSearchArea, widthSearchArea, heightSearchArea, maxSafeDistance);
  }
  else if (distanceRobotToCell <= maxSafeDistance && localFieldCoverage.isSorroundingCovered(theRobotPose, theFieldDimensions, maxSafeDistance))
  {
    newInterestingCell = localFieldCoverage.getInterestingCell(theRobotPose, theFieldDimensions, centerSearchArea, widthSearchArea, heightSearchArea, maxSafeDistance);
  }
  else if (seenCells.hasCellBeenVisited(theRobotPose, theFieldDimensions, interestingCell) || !(isCellInSearchArea(interestingCell)))
  {
    newInterestingCell = localFieldCoverage.getInterestingCell(theRobotPose, theFieldDimensions, centerSearchArea, widthSearchArea, heightSearchArea, 0.f);
  }
  else
  {
    newInterestingCell = localFieldCoverage.interestingCellInSearchArea;
  }

  if (newInterestingCell >= 0 && newInterestingCell < FieldCoverage::numOfCells)
  {
    localFieldCoverage.interestingCellInSearchArea = newInterestingCell;
  } 
  else 
  {
    localFieldCoverage.interestingCellInSearchArea = localFieldCoverage.getCellNumber(centerSearchArea, theFieldDimensions);
  }
}

float FieldCoverageProvider::getPointVisibility(const Vector2f &pointOnField, bool local)
{
  const float openingAngle = theCameraInfo.openingAngleWidth*openingAngleRatio;
  float pAngle = Transformation::fieldToRobot(theRobotPose,pointOnField).angle();
  float camAngle = theJointAngles.angles[Joints::headYaw];

  float result = 0.f;
  const int cellNo = getCellNumber(pointOnField);

  // local update
  float angleDiff = Angle::normalize(pAngle-camAngle);
  if (angleDiff < openingAngle && angleDiff > -openingAngle)
  {
    bool viewBlocked = isViewBlocked(theRobotMap,pointOnField,theRobotPose,theJointAngles.angles[Joints::headYaw]);
    localFieldCoverage.cells[cellNo].viewBlocked = viewBlocked;
    if (!viewBlocked)
    {
      float distance = (theRobotPose.translation-pointOnField).norm();
      if (distance < maxSafeDistance)
        result = 1.f;
      else
        result = 1.f-sqr(std::min((distance-maxSafeDistance)/maxUnsureDistance,1.f));
    }
  }

  if (local || theRobotPose.validity < minLocalValidityForGlobalUpdate)
    return result;

  // global update
    
  for (auto &mate : theTeammateData.teammates)
  {
    const RobotPose &pose = mate.pose;
    if (pose.validity > minTeamMateValidity && 
      mate.status > Teammate::ACTIVE)
    {
      pAngle = Transformation::fieldToRobot(pose,pointOnField).angle();
      camAngle = Angle::normalize(pose.rotation+mate.headPan);

      // opening angle symmetric
      if (std::abs(Angle::normalize(camAngle - pAngle)) < (openingAngle))
      {
        bool viewBlocked = isViewBlocked(mate.robotMap,pointOnField,pose,camAngle);
        if (!viewBlocked)
        {
          float distance = (pose.translation-pointOnField).norm();
          if (distance < maxSafeDistance)
            result = 1.f;
          else
            result = std::max(result,
              1.f-sqr(std::min((distance-maxSafeDistance)/maxUnsureDistance,1.f)));
        }
      }
    }
  }

  return result;
}

void FieldCoverageProvider::setProtection(int cellNr, float protection)
{
  cellProtection.cells[cellNr].validity = protection;
}

float FieldCoverageProvider::getProtection(int cellNr)
{
  return cellProtection.cells[cellNr].validity;
}

MAKE_MODULE(FieldCoverageProvider, modeling)
