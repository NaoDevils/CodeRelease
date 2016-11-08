#include <algorithm>
#include "Tools/Math/Transformation.h"
#include "DangerMapProvider.h"

DangerMapProvider::DangerMapProvider()
{

}

void DangerMapProvider::update(DangerMap &dangerMap)
{
  DECLARE_DEBUG_DRAWING("module:DangerMapProvider:dangerMap", "drawingOnField");
  updateDanger();
  dangerMap = localDangerMap;
  drawDangerMap();
}

void DangerMapProvider::updateDanger()
{
  Vector2f ballPos = Transformation::robotToField(theRobotPose, theBallModel.estimate.position);
 
  for (int cellNo = 0; cellNo < DangerMap::numOfCells; cellNo++)
  {
    // danger naturally decreases
    localDangerMap.danger[cellNo] -= dangerUpdateLoss;

    // check if ball has been sighted and update..
    if (theBallModel.timeWhenLastSeen == theFrameInfo.time)
    {
      float distance = (getFieldCoordinates(cellNo) - ballPos).norm();
      localDangerMap.danger[cellNo] += ballDangerUpdate -
        std::min(ballDangerUpdate, ballDangerUpdate * (distance / maxDistanceForUpdate));
    }
    // TODO: team mate ball models

    // now check for robots
    // own robot map
    for (auto &robot : theRobotMap.robots)
    {
      if (robot.robotType == RobotEstimate::teammateRobot)
        continue;
      float distance = (getFieldCoordinates(cellNo) - robot.pose.translation).norm();
      localDangerMap.danger[cellNo] += robotDangerUpdate -
        std::min(robotDangerUpdate, robotDangerUpdate * (distance / maxDistanceForUpdate));
    }
    // from team mate data
    for (auto &mate : theTeammateData.teammates)
    {
      for (auto &robot : mate.robotMap.robots)
      {
        if (robot.robotType == RobotEstimate::teammateRobot)
          continue;
        float distance = (getFieldCoordinates(cellNo) - robot.pose.translation).norm();
        localDangerMap.danger[cellNo] += robotDangerUpdate -
          std::min(robotDangerUpdate, robotDangerUpdate * (distance / maxDistanceForUpdate));
      }
    }
    // finally, clip to [0..1]
    localDangerMap.danger[cellNo] = std::max(0.f, std::min(localDangerMap.danger[cellNo], 1.f));
  }
  
}

bool DangerMapProvider::isViewBlocked(
  const RobotMap &robotMap, 
  const Vector2f &pCellOnField, 
  const Pose2f &pose, 
  const float &camAngle)
{
  return false;
}

MAKE_MODULE(DangerMapProvider, modeling)