/**
 * @file RobotPose.cpp
 *
 * contains the implementation of the streaming operators
 * for the struct RobotPose
 */

#include "RobotPose.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Module/Blackboard.h"

namespace
{
  using compressedFloat_type = unsigned char;
  static constexpr float uCharMax = static_cast<float>(std::numeric_limits<compressedFloat_type>::max());
}

void RobotPose::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:RobotPose", "drawingOnField");
  Vector2f bodyPoints[4] =
  {
    Vector2f(55, 90),
    Vector2f(-55, 90),
    Vector2f(-55, -90),
    Vector2f(55, -90)
  };
  for(int i = 0; i < 4; i++)
    bodyPoints[i] = *this * bodyPoints[i];

  Vector2f dirVec(200, 0);
  dirVec = *this * dirVec;
  static const ColorRGBA colors[] =
  {
    ColorRGBA::blue,
    ColorRGBA::red,
    ColorRGBA::yellow,
    ColorRGBA::black
  };
  const ColorRGBA ownTeamColorForDrawing = colors[Blackboard::getInstance().exists("OwnTeamInfo") ?
    static_cast<const OwnTeamInfo&>(Blackboard::getInstance()["OwnTeamInfo"]).teamColour : TEAM_BLUE];
  LINE("representation:RobotPose", translation.x(), translation.y(), dirVec.x(), dirVec.y(),
       20, Drawings::solidPen, ColorRGBA::white);
  POLYGON("representation:RobotPose", 4, bodyPoints, 20, Drawings::solidPen,
          ownTeamColorForDrawing, Drawings::solidBrush, ColorRGBA::white);
  CIRCLE("representation:RobotPose", translation.x(), translation.y(), 42, 0,
         Drawings::solidPen, ownTeamColorForDrawing, Drawings::solidBrush, ownTeamColorForDrawing);
  DRAWTEXT("representation:RobotPose", translation.x()+100, translation.y()+200, 100, ColorRGBA::white, "validity: " << validity);
  DRAWTEXT("representation:RobotPose", translation.x()+100, translation.y()+100, 100, ColorRGBA::white, "symmetry: " << symmetry);

  DECLARE_DEBUG_DRAWING("representation:RobotPose:deviation", "drawingOnField");
  DEBUG_DRAWING3D("representation:RobotPose", "field")
  {
    LINE3D("representation:RobotPose", translation.x(), translation.y(), 10,
           dirVec.x(), dirVec.y(), 10, 1, ownTeamColorForDrawing);
    for(int i = 0; i < 4; ++i)
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
  const ColorRGBA transparentWhite(255,255, 255, 128);
  Vector2f bodyPoints[4] = {
    Vector2f(55, 90),
    Vector2f(-55, 90),
    Vector2f(-55, -90),
    Vector2f(55, -90)
  };
  for(int i = 0; i < 4; i++)
    bodyPoints[i] = *this * bodyPoints[i];
  Vector2f dirVec(200, 0);
  dirVec = *this * dirVec;
  const ColorRGBA ownTeamColorForDrawing(0, 0, 0, 128);
  LINE("representation:GroundTruthRobotPose", translation.x(), translation.y(), dirVec.x(), dirVec.y(),
       20, Drawings::solidPen, transparentWhite);
  POLYGON("representation:GroundTruthRobotPose", 4, bodyPoints, 20, Drawings::solidPen,
          ownTeamColorForDrawing, Drawings::solidBrush, transparentWhite);
  CIRCLE("representation:GroundTruthRobotPose", translation.x(), translation.y(), 42, 0,
         Drawings::solidPen, ownTeamColorForDrawing, Drawings::solidBrush, ownTeamColorForDrawing);

  DECLARE_DEBUG_DRAWING("origin:GroundTruthRobotPose", "drawingOnField"); // Set the origin to the robot's ground truth position
  DECLARE_DEBUG_DRAWING("origin:GroundTruthRobotPoseWithoutRotation", "drawingOnField");
  ORIGIN("origin:GroundTruthRobotPose", translation.x(), translation.y(), rotation);
  ORIGIN("origin:GroundTruthRobotPoseWithoutRotation", translation.x(), translation.y(), 0);
}

void MocapRobotPose::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:MocapRobotPose", "drawingOnField");
  const ColorRGBA transparentWhite(255, 255, 255, 128);
  Vector2f bodyPoints[4] = {
    Vector2f(55, 90),
    Vector2f(-55, 90),
    Vector2f(-55, -90),
    Vector2f(55, -90)
  };
  for (int i = 0; i < 4; i++)
    bodyPoints[i] = *this * bodyPoints[i];
  Vector2f dirVec(200, 0);
  dirVec = *this * dirVec;
  const ColorRGBA ownTeamColorForDrawing(0, 0, 0, 128);
  LINE("representation:MocapRobotPose", translation.x(), translation.y(), dirVec.x(), dirVec.y(),
    20, Drawings::solidPen, transparentWhite);
  POLYGON("representation:MocapRobotPose", 4, bodyPoints, 20, Drawings::solidPen,
    ownTeamColorForDrawing, Drawings::solidBrush, transparentWhite);
  CIRCLE("representation:MocapRobotPose", translation.x(), translation.y(), 42, 0,
    Drawings::solidPen, ownTeamColorForDrawing, Drawings::solidBrush, ownTeamColorForDrawing);

  DECLARE_DEBUG_DRAWING("origin:MocapRobotPose", "drawingOnField"); // Set the origin to the robot's ground truth position
  DECLARE_DEBUG_DRAWING("origin:MocapRobotPoseWithoutRotation", "drawingOnField");
  ORIGIN("origin:MocapRobotPose", translation.x(), translation.y(), rotation);
  ORIGIN("origin:MocapPoseWithoutRotation", translation.x(), translation.y(), 0);
}

void RobotPoseAfterPreview::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:RobotPoseAfterPreview", "drawingOnField");
  const ColorRGBA transparentWhite(255, 255, 255, 128);
  Vector2f bodyPoints[4] = {
    Vector2f(55, 90),
    Vector2f(-55, 90),
    Vector2f(-55, -90),
    Vector2f(55, -90)
  };
  for (int i = 0; i < 4; i++)
    bodyPoints[i] = *this * bodyPoints[i];
  Vector2f dirVec(200, 0);
  dirVec = *this * dirVec;
  const ColorRGBA ownTeamColorForDrawing(0, 0, 0, 128);
  LINE("representation:RobotPoseAfterPreview", translation.x(), translation.y(), dirVec.x(), dirVec.y(),
    20, Drawings::solidPen, ColorRGBA::magenta);
  POLYGON("representation:RobotPoseAfterPreview", 4, bodyPoints, 20, Drawings::solidPen,
    ownTeamColorForDrawing, Drawings::solidBrush, ColorRGBA::magenta);
  CIRCLE("representation:RobotPoseAfterPreview", translation.x(), translation.y(), 42, 0,
    Drawings::solidPen, ownTeamColorForDrawing, Drawings::solidBrush, ownTeamColorForDrawing);

  DECLARE_DEBUG_DRAWING("origin:RobotPoseAfterPreview", "drawingOnField"); // Set the origin to the robot's ground truth position
  ORIGIN("origin:RobotPoseAfterPreview", translation.x(), translation.y(), rotation);
}

RobotPoseCompressed::RobotPoseCompressed(const RobotPose& robotPose)
{
  translation = robotPose.translation.cast<short>();
  rotation = static_cast<short>(robotPose.rotation.toDegrees());
  validity = static_cast<compressedFloat_type>(robotPose.validity * uCharMax);
  symmetry = static_cast<compressedFloat_type>(robotPose.symmetry * uCharMax);
}

RobotPoseCompressed::operator RobotPose() const
{
  RobotPose robotPose;
  robotPose.translation = translation.cast<float>();
  robotPose.rotation = Angle::fromDegrees(rotation);
  robotPose.validity = validity / uCharMax;
  robotPose.symmetry = symmetry / uCharMax;
  return robotPose;
}
