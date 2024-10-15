/**
 * @file RobotPose.cpp
 *
 * contains the implementation of the streaming operators
 * for the struct RobotPose
 */

#include "RobotPose.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Module/Blackboard.h"

void RobotPose::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:RobotPose", "drawingOnField");
  Vector2f bodyPoints[4] = {Vector2f(55, 90), Vector2f(-55, 90), Vector2f(-55, -90), Vector2f(55, -90)};
  for (int i = 0; i < 4; i++)
    bodyPoints[i] = *this * bodyPoints[i];

  Vector2f dirVec(200, 0);
  dirVec = *this * dirVec;
  const bool upright = Blackboard::getInstance().exists("FallDownState") ? Blackboard::get<FallDownState>().state == FallDownState::State::upright : false;
  const ColorRGBA ownTeamColorForDrawing = upright ? ColorRGBA::blue : ColorRGBA::red;
  LINE("representation:RobotPose", translation.x(), translation.y(), dirVec.x(), dirVec.y(), 20, Drawings::solidPen, ColorRGBA::white);
  POLYGON("representation:RobotPose", 4, bodyPoints, 20, Drawings::solidPen, ownTeamColorForDrawing, Drawings::solidBrush, ColorRGBA::white);
  CIRCLE("representation:RobotPose", translation.x(), translation.y(), 42, 0, Drawings::solidPen, ownTeamColorForDrawing, Drawings::solidBrush, ownTeamColorForDrawing);
  DRAWTEXT("representation:RobotPose", translation.x() - 80, translation.y() - 180, 80, ColorRGBA::blue, "pc: " << validity);
  DRAWTEXT("representation:RobotPose", translation.x() - 80, translation.y() - 260, 80, ColorRGBA::blue, "sc: " << sideConfidenceState);

  DECLARE_DEBUG_DRAWING("representation:RobotPose:deviation", "drawingOnField");
  DEBUG_DRAWING3D("representation:RobotPose", "field")
  {
    LINE3D("representation:RobotPose", translation.x(), translation.y(), 10, dirVec.x(), dirVec.y(), 10, 1, ownTeamColorForDrawing);
    for (int i = 0; i < 4; ++i)
    {
      const Vector2f p1 = bodyPoints[i];
      const Vector2f p2 = bodyPoints[(i + 1) & 3];
      LINE3D("representation:RobotPose", p1.x(), p1.y(), 10, p2.x(), p2.y(), 10, 1, ownTeamColorForDrawing);
    }
  }

  DECLARE_DEBUG_DRAWING("origin:RobotPose", "drawingOnField"); // Set the origin to the robot's current position
  DECLARE_DEBUG_DRAWING("origin:RobotPoseWithoutRotation", "drawingOnField");
  ORIGIN("origin:RobotPose", translation.x(), translation.y(), rotation);
  ORIGIN("origin:RobotPoseWithoutRotation", translation.x(), translation.y(), 0);
}

void GroundTruthRobotPose::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:GroundTruthRobotPose", "drawingOnField");
  const ColorRGBA transparentWhite(255, 255, 255, 128);
  Vector2f bodyPoints[4] = {Vector2f(55, 90), Vector2f(-55, 90), Vector2f(-55, -90), Vector2f(55, -90)};
  for (int i = 0; i < 4; i++)
    bodyPoints[i] = *this * bodyPoints[i];
  Vector2f dirVec(200, 0);
  dirVec = *this * dirVec;
  const ColorRGBA ownTeamColorForDrawing(0, 0, 0, 128);
  LINE("representation:GroundTruthRobotPose", translation.x(), translation.y(), dirVec.x(), dirVec.y(), 20, Drawings::solidPen, transparentWhite);
  POLYGON("representation:GroundTruthRobotPose", 4, bodyPoints, 20, Drawings::solidPen, ownTeamColorForDrawing, Drawings::solidBrush, transparentWhite);
  CIRCLE("representation:GroundTruthRobotPose", translation.x(), translation.y(), 42, 0, Drawings::solidPen, ownTeamColorForDrawing, Drawings::solidBrush, ownTeamColorForDrawing);

  DECLARE_DEBUG_DRAWING("origin:GroundTruthRobotPose", "drawingOnField"); // Set the origin to the robot's ground truth position
  DECLARE_DEBUG_DRAWING("origin:GroundTruthRobotPoseWithoutRotation", "drawingOnField");
  ORIGIN("origin:GroundTruthRobotPose", translation.x(), translation.y(), rotation);
  ORIGIN("origin:GroundTruthRobotPoseWithoutRotation", translation.x(), translation.y(), 0);
}

void RobotPoseAfterPreview::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:RobotPoseAfterPreview", "drawingOnField");
  const ColorRGBA transparentWhite(255, 255, 255, 128);
  Vector2f bodyPoints[4] = {Vector2f(55, 90), Vector2f(-55, 90), Vector2f(-55, -90), Vector2f(55, -90)};
  for (int i = 0; i < 4; i++)
    bodyPoints[i] = *this * bodyPoints[i];
  Vector2f dirVec(200, 0);
  dirVec = *this * dirVec;
  const ColorRGBA ownTeamColorForDrawing(0, 0, 0, 128);
  LINE("representation:RobotPoseAfterPreview", translation.x(), translation.y(), dirVec.x(), dirVec.y(), 20, Drawings::solidPen, ColorRGBA::magenta);
  POLYGON("representation:RobotPoseAfterPreview", 4, bodyPoints, 20, Drawings::solidPen, ownTeamColorForDrawing, Drawings::solidBrush, ColorRGBA::magenta);
  CIRCLE("representation:RobotPoseAfterPreview", translation.x(), translation.y(), 42, 0, Drawings::solidPen, ownTeamColorForDrawing, Drawings::solidBrush, ownTeamColorForDrawing);

  DECLARE_DEBUG_DRAWING("origin:RobotPoseAfterPreview", "drawingOnField"); // Set the origin to the robot's ground truth position
  ORIGIN("origin:RobotPoseAfterPreview", translation.x(), translation.y(), rotation);
}

RobotPoseCompressed::RobotPoseCompressed(const RobotPose& robotPose)
{
  translation = robotPose.translation;
  rotation = robotPose.rotation;
  validity = robotPose.validity;
  sideConfidenceState = robotPose.sideConfidenceState;
}

RobotPoseCompressed::operator RobotPose() const
{
  RobotPose robotPose;
  robotPose.translation = translation;
  robotPose.rotation = rotation;
  robotPose.validity = validity;
  robotPose.sideConfidenceState = sideConfidenceState;
  return robotPose;
}
