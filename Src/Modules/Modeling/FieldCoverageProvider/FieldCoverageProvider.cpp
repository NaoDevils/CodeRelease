#include "FieldCoverageProvider.h"
#include "Tools/Math/Geometry.h"

FieldCoverageProvider::FieldCoverageProvider()
{

}

void FieldCoverageProvider::update(FieldCoverage &fieldCoverage)
{
  DECLARE_DEBUG_DRAWING("representation:FieldCoverage", "drawingOnField");

  updateValidity();

  fieldCoverage = localFieldCoverage;

  DEBUG_RESPONSE("debug drawing:representation:FieldCoverage")
  {
    drawFieldCoverage();
  }
}

bool FieldCoverageProvider::isViewBlocked(const RobotMap &robotMap, const Vector2f &pCellOnField, const Pose2f &pose, const float &camAngle)
{
  // TODO: to params
  const float robotRadius = 200.f; 
  
  for (auto &robot : robotMap.robots)
  {
    float rDistance = (pose.translation-robot.pose.translation).norm();
    float rAngle = Transformation::fieldToRobot(pose,robot.pose.translation).angle();
    if (rAngle > pi_2 || robotRadius > rDistance)
      continue;
    float openingAngleForRobot = std::asin(robotRadius/rDistance);
    float pAngle = Transformation::fieldToRobot(pose,pCellOnField).angle();
    float pDistance = (float)((pCellOnField-pose.translation).norm());
    if (pDistance > rDistance 
      && std::abs(Angle::normalize(pAngle - rAngle)) < openingAngleForRobot)
      return true;
  }
  return false;
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
        localFieldCoverage.cells[cellNo].validity = 
          std::max(visibility,
          localFieldCoverage.cells[cellNo].validity - validityUpdateLoss);
      }
      else
        localFieldCoverage.cells[cellNo].validity = 
        std::max(0.f,localFieldCoverage.cells[cellNo].validity - validityUpdateLoss);
    }
  }
}

// TODO: Do not return worst validty value, but an interesting cell.
// TODO: ..depending on intention (looking for ball or maybe for obstacles), different method!?
int FieldCoverageProvider::getInterestingCell(const RobotPose &pose, float maxDistance)
{
  const int robotCell = getCellNumber(pose.translation);

  int worstCell = 0;

  for (int i = 0; i < FieldCoverage::numOfCells; i++)
  {
    const float cellDistance = getCellDistance(robotCell,i);
    if (cellDistance < maxDistance && localFieldCoverage.cells[worstCell].validity < localFieldCoverage.cells[i].validity)
      worstCell = i;
  }
  return worstCell;
}

float FieldCoverageProvider::getPointVisibility(const Vector2f &pointOnField, bool local)
{
  // TODO: opening angle to params
  const float openingAngle = theCameraInfo.openingAngleWidth*0.4f;
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
        result = 1.f-sqr(std::min((distance-maxSafeDistance)/5000.f,1.f));
    }
  }

  // TODO: minLocalValidityForGlobalUpdate to params
  if (local || theRobotPose.validity < 0.5f)
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
              1.f-sqr(std::min((distance-maxSafeDistance)/5000.f,1.f)));
        }
      }
    }
  }

  return result;
}

MAKE_MODULE(FieldCoverageProvider, modeling)