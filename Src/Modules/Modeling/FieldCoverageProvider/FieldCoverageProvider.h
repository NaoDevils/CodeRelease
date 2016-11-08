
/**
* @file FieldCoverageProvider.h
* Declares a class that provides information about which part of the field was seen when.
**
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include <algorithm>

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Module/Module.h"

MODULE(FieldCoverageProvider,
{
  REQUIRES(BallModel),
  REQUIRES(CameraInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(JointAngles),
  REQUIRES(TeamBallModel),
  REQUIRES(RobotInfo),
  REQUIRES(RobotMap),
  REQUIRES(RobotPose),
  REQUIRES(TeammateData),
  PROVIDES(FieldCoverage),
  LOADS_PARAMETERS(
  {,
    (float) maxSafeDistance,
    (float) minTeamMateValidity,
    (float) validityUpdateLoss,
  }),
});

class FieldCoverageProvider : public FieldCoverageProviderBase
{
public:

  FieldCoverageProvider();

  void update(FieldCoverage &fieldCoverage);

private:

  FieldCoverage localFieldCoverage;

  // checks, if the view on a certain point on the field is blocked by another robot
  bool isViewBlocked(const RobotMap &robotMap, const Vector2f &pCellOnField, const Pose2f &pose, const float &camAngle);
  
  // main method
  void updateValidity();
  
  // cell has red border, if view to cell is blocked by a robot
  void drawFieldCoverage()
  {
    const int stepSizeHalf = FieldCoverage::stepSize/2;
    for (int i = 0; i < FieldCoverage::numOfCells; i++)
    {
      Vector2f posField = getFieldCoordinates(i);
      int alpha = 255 - (int)(localFieldCoverage.cells[i].validity*255);
      RECTANGLE2("representation:FieldCoverage",
        Vector2i((int)posField.x()-stepSizeHalf,(int)posField.y()-stepSizeHalf),
        FieldCoverage::stepSize,
        FieldCoverage::stepSize,
        0,
        10,
        Drawings::solidPen,
        localFieldCoverage.cells[i].viewBlocked ? ColorRGBA::red : ColorRGBA::green,
        Drawings::solidBrush,
        ColorRGBA(0,255,0,static_cast<unsigned char>(alpha)));
    }
  }

  // TODO
  void drawFieldCoverage3D()
  {
    
  }

  // TODO..
  int getInterestingCell(const RobotPose &pose, float maxDistance);
  
  // distance between two cells on the field
  inline float getCellDistance(int firstCell, int secondCell)
  {
    return FieldCoverage::stepSize*((getFieldCoordinates(firstCell)-getFieldCoordinates(secondCell)).norm());
  }

  inline Vector2f getFieldCoordinates(int cellNo)
  {
    return Vector2f((((float)cellNo/ (float)FieldCoverage::numOfCellsY)*FieldCoverage::stepSize-theFieldDimensions.xPosOpponentGroundline+FieldCoverage::stepSize/2),
      (float)((cellNo%FieldCoverage::numOfCellsY)*FieldCoverage::stepSize-theFieldDimensions.yPosLeftSideline+FieldCoverage::stepSize/2));
  }

  inline int getCellNumber(const Vector2f &posOnField)
  {
    int xIndex = (int)((posOnField.x()+(float)theFieldDimensions.xPosOpponentGroundline)/500.f);
    int yIndex = (int)((posOnField.y()+(float)theFieldDimensions.yPosLeftSideline)/500.f);
    return xIndex*FieldCoverage::numOfCellsY+yIndex;
  }

  /**
  * Between 0 (not visible) and 1 (visible).
  * @param local If no team mates should be taken into account, set this to true.
  */
  float getPointVisibility(const Vector2f &pointOnField, bool local);

};

