#include "FieldCoverage.h"

const float FieldCoverage::stepSize = 1000.f/3.f;

//helper functions

int FieldCoverage::getCellNumber(const Vector2f &posOnField, const FieldDimensions &fieldDimensions) const
{
  int xIndex = (int)((posOnField.x()+(float)fieldDimensions.xPosOpponentGroundline)/stepSize);
  int yIndex = (int)((posOnField.y()+(float)fieldDimensions.yPosLeftSideline)/stepSize);
  xIndex = std::min(std::max(0,xIndex),(int)numOfCellsX-1);
  yIndex = std::min(std::max(0,yIndex),(int)numOfCellsY-1);
  return xIndex*numOfCellsY+yIndex;
}

Vector2f FieldCoverage::getFieldCoordinates(int cellNo, const FieldDimensions &fieldDimensions) const
{
  return Vector2f(((cellNo/numOfCellsY)*stepSize-fieldDimensions.xPosOpponentGroundline+stepSize/2),
    (float)((cellNo%numOfCellsY)*stepSize-fieldDimensions.yPosLeftSideline+stepSize/2));
}

// copied here from FieldCoverageProvider

// distance between two cells on the field
float FieldCoverage::getCellDistance(int firstCell, int secondCell, const FieldDimensions &fieldDimensions) const
{
  return (getFieldCoordinates(firstCell, fieldDimensions)-getFieldCoordinates(secondCell, fieldDimensions)).norm();
}

// checks if a cell has been visited, i.e. the validity equals 1 or the robot is standing on that cell
bool FieldCoverage::hasCellBeenVisited(const RobotPose &pose, const FieldDimensions &fieldDimensions, int cellNumber) const
{
  if (cellNumber < 0 || cellNumber >= numOfCells || cells[cellNumber].validity == 1 || cellNumber == getCellNumber(pose.translation, fieldDimensions)) {
    return true;
  }
  else
  {
    return false;
  }
}

//this only works on the fieldcoverage that assigns negative values to covered cells, e.g. theFieldCoverage
bool FieldCoverage::isSorroundingCovered(const RobotPose &pose, const FieldDimensions &fieldDimensions, float radiusOfSorroundingArea) const
{
  float sorroundingSum = 0;
  float sorroundingCells = 0;
  for (int i = 0; i <= FieldCoverage::numOfCells - 1; i++)
  {
    int robotCell = getCellNumber(pose.translation, fieldDimensions);
    if (getCellDistance(i, robotCell, fieldDimensions) <= radiusOfSorroundingArea)
    {
      sorroundingSum += cells[i].validity;
      sorroundingCells++;
    }

  }
  float sorroundingAvg = sorroundingSum / sorroundingCells;
  if (sorroundingAvg <= -0.5)
  {
    return true;
  }
  return false;
}

// TODO: Do not return worst validty value, but an interesting cell.
// TODO: depending on distance to self and teammates
// TODO: ..depending on intention (looking for ball or maybe for obstacles), different method!?
// determines the most interesting cell on the field
int FieldCoverage::getInterestingCell(const RobotPose &pose, const FieldDimensions &fieldDimensions, float minDistanceToRobot) const
{
  Vector2f fieldCenter(fieldDimensions.xPosKickOffPoint, fieldDimensions.yPosKickOffPoint);
  return getInterestingCell(pose, fieldDimensions, fieldCenter, fieldDimensions.yPosLeftSideline * 2, fieldDimensions.xPosOpponentGroundline * 2, minDistanceToRobot);
}

/*determines the most interesting cell in a defined rectangle
@param center - center of the rectangle
@param width - size in y direction
@param height - size in x direction
*/
int FieldCoverage::getInterestingCell(const RobotPose &pose, const FieldDimensions &fieldDimensions, Vector2f center, float width, float height, float minDistanceToRobot) const
{
  return getInterestingCellExcluding(pose, fieldDimensions, center, width, height, -1, minDistanceToRobot);
}

// determines the most interesting cell on the field ignoring cellToExclude
int FieldCoverage::getInterestingCellExcluding(const RobotPose &pose, const FieldDimensions &fieldDimensions, int cellToExclude, float minDistanceToRobot) const
{
  Vector2f fieldCenter(fieldDimensions.xPosKickOffPoint, fieldDimensions.yPosKickOffPoint);
  return getInterestingCellExcluding(pose, fieldDimensions, fieldCenter, fieldDimensions.yPosLeftSideline * 2, fieldDimensions.xPosOpponentGroundline * 2, cellToExclude, minDistanceToRobot);
}

//determines the most interesting cell in a defined rectangle ignoring cellToExclude
int FieldCoverage::getInterestingCellExcluding(const RobotPose &pose, const FieldDimensions &fieldDimensions, Vector2f center, float width, float height, int cellToExclude, float minDistanceToRobot) const
{
  const int robotCell = getCellNumber(pose.translation, fieldDimensions);
  bool validCellFound = false;
  int mostInterestingCell = -1;

  for (int i = 0; i < FieldCoverage::numOfCells; i++)
  {
    if (getCellDistance(i, robotCell,fieldDimensions) >= minDistanceToRobot && i != cellToExclude && cells[i].validity >= -0.85) {
      Vector2f coordinatesOfCell = getFieldCoordinates(i, fieldDimensions);
      float minX = center.x() - height / 2;
      float maxX = center.x() + height / 2;
      float minY = center.y() - width / 2;
      float maxY = center.y() + width / 2;
      if (coordinatesOfCell.x() >= minX && coordinatesOfCell.x() <= maxX && coordinatesOfCell.y() >= minY && coordinatesOfCell.y() <= maxY)
      {
        if (!validCellFound)
        {
          mostInterestingCell = i;
          validCellFound = true;
        }
        Vector2f robotToMostInterestingCell = Transformation::fieldToRobot(pose, getFieldCoordinates(mostInterestingCell, fieldDimensions));
        Vector2f robotToI = Transformation::fieldToRobot(pose, getFieldCoordinates(i, fieldDimensions));
        float iDistance = std::max(1.f, std::abs(getCellDistance(robotCell, i, fieldDimensions)));                                                //min(1,...) don't divide by zero
        // unused variable
        //float robotToMostInterestingCellAngle = robotToMostInterestingCell.angle();
        float mostInterestingCellDistance = std::max(1.f, std::abs(getCellDistance(robotCell, mostInterestingCell, fieldDimensions)));
        float mostInterestingCellFactor = std::max(0.3f, std::cos(robotToMostInterestingCell.angle())) / mostInterestingCellDistance;             //min(0.3f,...) same weight for all cells with a large enough angle
        float iFactor = std::max(0.3f, std::cos(robotToI.angle())) / iDistance;
        if ((1.f + cells[mostInterestingCell].validity) * mostInterestingCellFactor <= (1.f + cells[i].validity) * iFactor)                       // (1+validity) -> don't actively avoid seen cells because they are near
        {
          mostInterestingCell = i;
          validCellFound = true;
        }
      }
    }
  }
  return mostInterestingCell;
}

//determines the most interesting cell in a defined circle area
int FieldCoverage::getInterestingCellInCircle(const RobotPose &pose, float maxDistance, const FieldDimensions &fieldDimensions, Vector2f center, float radius) const
{
    const int robotCell = getCellNumber(pose.translation, fieldDimensions);
    bool validCellFound = false;
    int mostInterestingCell = 0;

    for (int i = 0; i < FieldCoverage::numOfCells; i++)
    {
        Vector2f cellToCenter = center - getFieldCoordinates(i, fieldDimensions);
        float distanceCellToCenter = cellToCenter.norm();
        if (distanceCellToCenter < radius)
        {
            if (!validCellFound)
            {
                mostInterestingCell = i;
                validCellFound = true;
            }
            float cellDistance = getCellDistance(robotCell, i, fieldDimensions);
            float mostInterestingCellDistance = getCellDistance(robotCell, mostInterestingCell, fieldDimensions);
            if (cellDistance < maxDistance)
            {
                cellDistance = (cellDistance < 1.f ? 1.f : cellDistance); // don't divide by zero
                if ((1.f + cells[mostInterestingCell].validity) / mostInterestingCellDistance <= (1.f + cells[i].validity) / cellDistance) // (1+validity) -> don't actively avoid seen cells because they are near
                {
                    mostInterestingCell = i;
                    validCellFound = true;
                }
            }
        }
    }
    return mostInterestingCell;
}

int FieldCoverage::getInterestingCellInSearchArea()
{
  return interestingCellInSearchArea;
}
