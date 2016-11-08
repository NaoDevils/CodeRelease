
#pragma once

#include "RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Streams/Streamable.h"
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
    numOfCellsX = 18,
    numOfCellsY = 12,
    numOfCells = numOfCellsX*numOfCellsY,
    stepSize = 500
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
    return *this;
  }

  Cell cells[numOfCells];

  //helper functions

  inline int getCellNumber(const Vector2f &posOnField, const FieldDimensions &fieldDimensions) const
  {
    int xIndex = (int)((posOnField.x()+(float)fieldDimensions.xPosOpponentGroundline)/(float)stepSize);
    int yIndex = (int)((posOnField.y()+(float)fieldDimensions.yPosLeftSideline)/(float)stepSize);
    xIndex = std::min(std::max(0,xIndex),(int)numOfCellsX-1);
    yIndex = std::min(std::max(0,yIndex),(int)numOfCellsY-1);
    return xIndex*numOfCellsY+yIndex;
  }

  inline Vector2f getFieldCoordinates(int cellNo, const FieldDimensions &fieldDimensions) const
  {
    return Vector2f((((float)cellNo/ (float)numOfCellsY)*stepSize-fieldDimensions.xPosOpponentGroundline+FieldCoverage::stepSize/2),
      (float)((cellNo%numOfCellsY)*stepSize-fieldDimensions.yPosLeftSideline+stepSize/2));
  }
};
