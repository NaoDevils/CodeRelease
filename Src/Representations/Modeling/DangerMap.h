/**
* @file DangerMap.h
* Declaration of a class that represents a map indicating the pressure through opponents on the field.
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include <algorithm>

struct DangerMap : public Streamable
{
  enum
  {
    numOfCellsX = 24,
    numOfCellsY = 16,
    numOfCells = numOfCellsX*numOfCellsY,
    stepSize = 375
  };

  DangerMap()
  {
    reset();
  }

  void reset()
  {
    for (int i = 0; i < numOfCells; i++)
    {
      danger[i] = 0.f;
    }
  }

  DangerMap& operator=(const DangerMap &other)
  {
    if (this == &other)
      return *this;
    for (int i = 0; i < numOfCells; i++)
    {
      danger[i] = other.danger[i];
    }
    return *this;
  }

  /** @return Maximum danger in selected zone (square defined by distance parameter) */
  inline float getDangerAt(const Vector2f &posOnField, const FieldDimensions &fieldDimensions, const float &distance) const
  {
    const int distanceInCells = (int)distance / stepSize;
    int xIndex = (int)((posOnField.x() + (float)fieldDimensions.xPosOpponentGroundline) / (float)stepSize);
    int yIndex = (int)((posOnField.y() + (float)fieldDimensions.yPosLeftSideline) / (float)stepSize);
    const int minX = std::max(0, xIndex - distanceInCells);
    const int maxX = std::min((int)numOfCellsX - 1, xIndex + distanceInCells);
    const int minY = std::max(0, yIndex - distanceInCells);
    const int maxY = std::min((int)numOfCellsY - 1, yIndex + distanceInCells);
    float result = 0.f;
    for (int x = minX; x <= maxX; x++)
    {
      for (int y = minY; y <= maxY; y++)
      {
        result = std::max(result, danger[x*numOfCellsY + y]);
      }
    }
    return result;
  }

  //helper functions
  inline int getCellNumber(const Vector2f &posOnField, const FieldDimensions &fieldDimensions) const
  {
    int xIndex = (int)((posOnField.x() + (float)fieldDimensions.xPosOpponentGroundline) / (float)stepSize);
    int yIndex = (int)((posOnField.y() + (float)fieldDimensions.yPosLeftSideline) / (float)stepSize);
    xIndex = std::min(std::max(0, xIndex), (int)numOfCellsX - 1);
    yIndex = std::min(std::max(0, yIndex), (int)numOfCellsY - 1);
    return xIndex*numOfCellsY + yIndex;
  }

  inline Vector2f getFieldCoordinates(int cellNo, const FieldDimensions &fieldDimensions) const
  {
    return Vector2f(((cellNo / numOfCellsY)*stepSize - fieldDimensions.xPosOpponentGroundline + stepSize / 2),
      (float)((cellNo%numOfCellsY)*stepSize - fieldDimensions.yPosLeftSideline + stepSize / 2));
  }

  float danger[numOfCells]; /**< indicating possible pressure of opponent on area [0..1] */

private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(danger);
    STREAM_REGISTER_FINISH;
  }
};
