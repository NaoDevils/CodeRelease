
#pragma once

#include "RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Streams/Streamable.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Tools/Math/Eigen.h"
#include <algorithm>



struct FieldCoverage : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    for (int i = 0; i < numOfCells; i++)
    {
      STREAM(cells[i].viewBlocked);
    }
    STREAM_REGISTER_FINISH;
  }

public:

  struct Cell : public Streamable
  {
    float validity; // globally updated [0..1]
    bool viewBlocked; // locally updated (for now, depends on robotMap)

    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(validity);
      STREAM(viewBlocked);
      STREAM_REGISTER_FINISH;
    }
  };

  enum
  {
    numOfCellsX = 27,
    numOfCellsY = 18,
    numOfCells = numOfCellsX*numOfCellsY
  };

  
  FieldCoverage() 
  {
    reset();
  }

  void reset()
  {
    for (int i = 0; i < numOfCells; i++)
    {
      cells[i].validity = 0.f;
      cells[i].viewBlocked = false;
      interestingCellInSearchArea = -1;
    }
  }
  
  FieldCoverage& operator=(const FieldCoverage &other)
  {
    if (this == &other)
      return *this;
    for (int i = 0; i < numOfCells; i++)
    {
      cells[i].validity = other.cells[i].validity;
      cells[i].viewBlocked = other.cells[i].viewBlocked;
    }
    interestingCellInSearchArea = other.interestingCellInSearchArea;
    return *this;
  }

  Cell cells[numOfCells];
  int interestingCellInSearchArea=-1;
  static const float stepSize;
//helper functions

int getCellNumber(const Vector2f &posOnField, const FieldDimensions &fieldDimensions) const;

Vector2f getFieldCoordinates(int cellNo, const FieldDimensions &fieldDimensions) const;

// copied here from FieldCoverageProvider

// distance between two cells on the field
float getCellDistance(int firstCell, int secondCell, const FieldDimensions &fieldDimensions) const;

// checks if a cell has been visited, i.e. the validity equals 1 or the robot is standing on that cell
bool hasCellBeenVisited(const RobotPose &pose, const FieldDimensions &fieldDimensions, int cellNumber) const;

//this only works on the fieldcoverage that assigns negative values to covered cells, e.g. theFieldCoverage
bool isSorroundingCovered(const RobotPose &pose, const FieldDimensions &fieldDimensions, float radiusOfSorroundingArea) const;

// TODO: Do not return worst validty value, but an interesting cell.
// TODO: depending on distance to self and teammates
// TODO: ..depending on intention (looking for ball or maybe for obstacles), different method!?
// determines the most interesting cell on the field
int getInterestingCell(const RobotPose &pose, const FieldDimensions &fieldDimensions, float minDistanceToRobot) const;

/*determines the most interesting cell in a defined rectangle
@param center - center of the rectangle
@param width - size in y direction
@param height - size in x direction
*/
int getInterestingCell(const RobotPose &pose, const FieldDimensions &fieldDimensions, Vector2f center, float width, float height, float minDistanceToRobot) const;

// determines the most interesting cell on the field ignoring cellToExclude
int getInterestingCellExcluding(const RobotPose &pose, const FieldDimensions &fieldDimensions, int cellToExclude, float minDistanceToRobot) const;

//determines the most interesting cell in a defined rectangle ignoring cellToExclude
int getInterestingCellExcluding(const RobotPose &pose, const FieldDimensions &fieldDimensions, Vector2f center, float width, float height, int cellToExclude, float minDistanceToRobot) const;

//determines the most interesting cell in a defined circle area
int getInterestingCellInCircle(const RobotPose &pose, float maxDistance, const FieldDimensions &fieldDimensions, Vector2f center, float radius) const;

int getInterestingCellInSearchArea();


};
