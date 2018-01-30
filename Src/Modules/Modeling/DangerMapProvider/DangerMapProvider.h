
/**
* @file DangerMapProvider.h
* Declares a class that provides a map that contains information about the possible crowded/dangerous zones on the field
**
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/DangerMap.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/InStreams.h"

MODULE(DangerMapProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(RobotMap),
  REQUIRES(RobotPose),
  REQUIRES(TeammateData),
  PROVIDES(DangerMap),
  LOADS_PARAMETERS(
  {,
    (float) maxDistanceForUpdate,
    (float) minTeamMateValidity,
    (float) dangerUpdateLoss,
    (float) ballDangerUpdate,
    (float) robotDangerUpdate,
  }),
});

class DangerMapProvider : public DangerMapProviderBase
{
public:

  DangerMapProvider();

  void update(DangerMap &dangerMap);

private:

  DangerMap localDangerMap;

  // checks, if the view on a certain point on the field is blocked by another robot
  bool isViewBlocked(const RobotMap &robotMap, const Vector2f &pCellOnField, const Pose2f &pose, const float &camAngle);

  // main method
  void updateDanger();

  // cell has red border, if view to cell is blocked by a robot
  void drawDangerMap()
  {
    const int stepSizeHalf = DangerMap::stepSize / 2;
    for (int i = 0; i < DangerMap::numOfCells; i++)
    {
      Vector2f posField = getFieldCoordinates(i);
      int alpha = 255 - (int)(localDangerMap.danger[i] * 255);
      RECTANGLE2("module:DangerMapProvider:dangerMap",
        Vector2i((int)posField.x() - stepSizeHalf, (int)posField.y() - stepSizeHalf),
        DangerMap::stepSize,
        DangerMap::stepSize,
        0,
        10,
        Drawings::solidPen,
        ColorRGBA(static_cast<unsigned char>(255 - alpha), 
          static_cast<unsigned char>(alpha), 0),
        Drawings::solidBrush,
        ColorRGBA(static_cast<unsigned char>(255 - alpha), 
          static_cast<unsigned char>(alpha), 0, 
          static_cast<unsigned char>(50 + (255 - alpha)/2)));
    }
  }

  // TODO
  void drawDangerMap3D()
  {

  }

  // distance between two cells on the field
  inline float getCellDistance(int firstCell, int secondCell)
  {
    return DangerMap::stepSize*((getFieldCoordinates(firstCell) - getFieldCoordinates(secondCell)).norm());
  }

  inline Vector2f getFieldCoordinates(int cellNo)
  {
    return Vector2f(((cellNo / DangerMap::numOfCellsY)*DangerMap::stepSize - theFieldDimensions.xPosOpponentGroundline + DangerMap::stepSize / 2),
      (float)((cellNo%DangerMap::numOfCellsY)*DangerMap::stepSize - theFieldDimensions.yPosLeftSideline + DangerMap::stepSize / 2));
  }

  inline int getCellNumber(const Vector2f &posOnField)
  {
    int xIndex = (int)((posOnField.x() + (float)theFieldDimensions.xPosOpponentGroundline) / 500.f);
    int yIndex = (int)((posOnField.y() + (float)theFieldDimensions.yPosLeftSideline) / 500.f);
    return xIndex*DangerMap::numOfCellsY + yIndex;
  }

};

