
/**
* @file FieldCoverageProvider.h
* Declares a class that provides information about which part of the field was seen when.
**
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include <algorithm>
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/PositioningSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Module/Module.h"
#include "Representations/Modeling/BallSearchArea.h"

MODULE(FieldCoverageProvider,
{ ,
  REQUIRES(BallModel),
  REQUIRES(BallSearchArea),
  USES(BehaviorData),
  REQUIRES(CameraInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(JointAngles),
  REQUIRES(OwnTeamInfo),
  REQUIRES(PositioningSymbols),
  REQUIRES(RobotInfo),
  REQUIRES(RobotMap),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(TeammateData),
  PROVIDES(FieldCoverage),
  LOADS_PARAMETERS(
  {,
    (float) maxSafeDistance,  /* distance at wich we are sure to see the ball */
    (float) maxUnsureDistance,  /* Additional distance with decreasing validity for distances exeeding maxSafeDistance */
    (float) minLocalValidityForGlobalUpdate,  /* threshold for the decision to use vision of the teammates */
    (float) minTeamMateValidity,  /* pose validity threshold for teammates */
    (float) openingAngleRatio,  /*  ratio of the cameras opening angle in wich we expect to find the ball */
    (float) validityGainWhenSeen, /* rate in which cells gain validity when seen by a robot */
    (float) validityUpdateLoss, /* rate of decay for seenCells (per Frame) */
    (float) interestUpdateLoss, /* rate of decay for interestingCells (per Frame) */
    (float) lineImportance, /* validity gain over time (per Frame) for cells containing field lines in intestingCells */
    (float) robotRadius,  /* radius for a circle at a robot position that blocks the view */
    (float) timeThreshholdforBallLoss,
    (int) ballPosRememberingTime, /* time for wich the last known ball position is added to interestingCells (in ms) */
  }),
});

class FieldCoverageProvider : public FieldCoverageProviderBase
{
public:

  FieldCoverageProvider();

  void update(FieldCoverage &fieldCoverage);

  bool containsFieldLines(const FieldCoverage &fieldCoverage, int cellNumber); //true when cell with number cellNumber in fieldCoverage contains field lines

  void makeOptimalAreaInteresting();

private:

  FieldCoverage localFieldCoverage;
  FieldCoverage seenCells;
  FieldCoverage interestingCells;
  FieldCoverage cellProtection;
  unsigned timeWhenBallLost = 0;
  Vector2f centerOfOptimalArea = Vector2f(theFieldDimensions.xPosKickOffPoint, theFieldDimensions.yPosKickOffPoint); // the center of the general area on the field where a robot should be located while searching for the ball
  bool ballLostThisFrame = true;

  // checks, if the view on a certain point on the field is blocked by another robot
  bool isViewBlocked(const RobotMap &robotMap, const Vector2f &pCellOnField, const Pose2f &pose, const float &camAngle);

  // adds importance to a cell in interestingCells
  void addInterestingCell(const Vector2f &pos, float importance);

  // sets the importance of a cell to a given value if that value is higher than the current importance
  void setImportanceOfCell(const Vector2f &pos, float importance);

  // combines seenCells and interestingCells into localFieldCoverage in a way, that the higher the value the more the cell is interesting to look at
  void combineCells();
  
  // adds points of interest converted to cells to interestingCells
  void findInterestingPoints();

  // adds the search points from LibPositioning as interesting points
  void addSearchPoints();

  // handles decay of interest and validity in cells
  void updateValidity();

  // determines the location of the general area in which the robot is supposed to stay while looking for the ball
  void determineOptimalArea();
  
  // cell has red border, if view to cell is blocked by a robot
  void drawFieldCoverage()
  {
    int interestingCell = localFieldCoverage.interestingCellInSearchArea;
    int robotCell = localFieldCoverage.getCellNumber(theRobotPose.translation, theFieldDimensions);
    float stepSizeHalf = FieldCoverage::stepSize/2;
    for (int i = 0; i < FieldCoverage::numOfCells; i++)
    {
      Vector2f posField = getFieldCoordinates(i);
      int alpha = (int)(localFieldCoverage.cells[i].validity*255);
      if (i == interestingCell)
      {
        RECTANGLE2("representation:FieldCoverage",
          Vector2f(posField.x() - stepSizeHalf, posField.y() - stepSizeHalf),
          FieldCoverage::stepSize,
          FieldCoverage::stepSize,
          0,
          10,
          Drawings::solidPen,
          localFieldCoverage.cells[i].viewBlocked ? ColorRGBA::red : ColorRGBA::green,
          Drawings::solidBrush,
          ColorRGBA::yellow);
      }
      else if (i == robotCell)
      {
        RECTANGLE2("representation:FieldCoverage",
          Vector2f(posField.x() - stepSizeHalf, posField.y() - stepSizeHalf),
          FieldCoverage::stepSize,
          FieldCoverage::stepSize,
          0,
          10,
          Drawings::solidPen,
          localFieldCoverage.cells[i].viewBlocked ? ColorRGBA::red : ColorRGBA::green,
          Drawings::solidBrush,
          ColorRGBA::red);
      }
      else
      {
        RECTANGLE2("representation:FieldCoverage",
          Vector2f(posField.x() - stepSizeHalf, posField.y() - stepSizeHalf),
          FieldCoverage::stepSize,
          FieldCoverage::stepSize,
          0,
          10,
          Drawings::solidPen,
          localFieldCoverage.cells[i].viewBlocked ? ColorRGBA::red : ColorRGBA::green,
          Drawings::solidBrush,
          alpha >= 0 ? ColorRGBA(0, 0, 255, static_cast<char>(alpha)) : ColorRGBA(0, 255, 0, static_cast<char>(-alpha)));
      }
    }
  }


  // TODO
  void drawFieldCoverage3D()
  {
    
  }

  int getInterestingCell(const RobotPose &pose);
  
  // distance between two cells on the field
  inline float getCellDistance(int firstCell, int secondCell)
  {
    return localFieldCoverage.getCellDistance(firstCell, secondCell, theFieldDimensions);
  }

  inline Vector2f getFieldCoordinates(int cellNo)
  {
    return localFieldCoverage.getFieldCoordinates(cellNo, theFieldDimensions);
  }

  inline int getCellNumber(const Vector2f &posOnField)
  {
    return localFieldCoverage.getCellNumber(posOnField, theFieldDimensions);
  }

  void determineInterestingCellInSearchArea();
  /**
  * Between 0 (not visible) and 1 (visible).
  * @param local If no team mates should be taken into account, set this to true.
  */
  float getPointVisibility(const Vector2f &pointOnField, bool local);

  //void setOptimalArea(Vector2f centerOfArea);

  void setProtection(int cellNr, float protection);

  float getProtection(int cellNr);

  void resetAllFieldCoverages();

  bool isCellInSearchArea(int cellNumber);

  void checkBallLostThisFrame();
};

