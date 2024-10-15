/**
 * @file BallModel.cpp
 * Implementation of the BallModel's drawing functions
 */

#include "BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Platform/SystemCall.h"

Vector2f BallState::getVelocityInFieldCoordinates(const RobotPose& rp) const
{
  float c(std::cos(rp.rotation));
  float s(std::sin(rp.rotation));
  return Vector2f(velocity.x() * c - velocity.y() * s, velocity.x() * s + velocity.y() * c);
}

void BallState::setPositionAndVelocityFromFieldCoordinates(const Vector2f& positionOnField, const Vector2f& velocityOnField, const RobotPose& rp)
{
  position = Transformation::fieldToRobot(rp, positionOnField);
  float c(std::cos(rp.rotation));
  float s(std::sin(rp.rotation));
  velocity = Vector2f(velocityOnField.x() * c + velocityOnField.y() * s, -velocityOnField.x() * s + velocityOnField.y() * c);
}

void BallModel::draw() const
{
  // drawing of the ball model in the field view
  DEBUG_DRAWING("representation:BallModel", "drawingOnField")
  {
    const Vector2f& position(estimate.position);
    const Vector2f& velocity(estimate.velocity);
    CIRCLE("representation:BallModel",
        position.x(),
        position.y(),
        70,
        10, // pen width
        Drawings::solidPen,
        ColorRGBA::black,
        Drawings::solidBrush,
        ColorRGBA::white);
    ARROW("representation:BallModel", position.x(), position.y(), position.x() + velocity.x(), position.y() + velocity.y(), 5, 1, ColorRGBA::white);
    DRAWTEXT("representation:BallModel", position.x() + 50, position.y() + 50, 50, ColorRGBA::white, static_cast<int>(validity * 100.f + 0.5f) << "%");
  }

  // drawing of the end position
  DEBUG_DRAWING("representation:BallModel:endPosition", "drawingOnField")
  {
    Vector2f position = BallPhysics::getEndPosition(estimate.position, estimate.velocity, friction);
    CIRCLE("representation:BallModel:endPosition",
        position.x(),
        position.y(),
        70,
        10, // pen width
        Drawings::solidPen,
        ColorRGBA::black,
        Drawings::solidBrush,
        ColorRGBA::gray);
  }

  DEBUG_DRAWING3D("representation:BallModel", "robot")
  {
    TRANSLATE3D("representation:BallModel", 0, 0, -230);
    if (SystemCall::getTimeSince(timeWhenLastSeen) < 5000 && SystemCall::getTimeSince(timeWhenDisappeared) < 1000)
    {
      SPHERE3D("representation:BallModel", estimate.position.x(), estimate.position.y(), estimate.radius, estimate.radius, ColorRGBA::orange);
      LINE3D("representation:BallModel", 0, 0, 1.f, estimate.position.x(), estimate.position.y(), 1.f, 5.f, ColorRGBA::orange);
    }
  }
}

void GroundTruthBallModel::draw() const
{
  DEBUG_DRAWING("representation:GroundTruthBallModel", "drawingOnField")
  {
    const Vector2f& position(estimate.position);
    const Vector2f& velocity(estimate.velocity);
    const ColorRGBA colorForDrawing(0, 0, 0, 128);
    CIRCLE("representation:GroundTruthBallModel",
        position.x(),
        position.y(),
        45,
        0, // pen width
        Drawings::solidPen,
        colorForDrawing,
        Drawings::solidBrush,
        colorForDrawing);
    ARROW("representation:GroundTruthBallModel", position.x(), position.y(), position.x() + velocity.x(), position.y() + velocity.y(), 5, 1, colorForDrawing);
  }
}

BallModelCompressed::BallModelCompressed(const BallModel& ballModel)
    : position(ballModel.estimate.position), velocity(ballModel.estimate.velocity), timeWhenLastSeen(ballModel.timeWhenLastSeen), validity(ballModel.validity)
{
}

void MultipleBallModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:MultipleBallModel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:MultipleBallModel:endPosition", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:MultipleBallModel", "robot");

  for (size_t i = 0; i < ballModels.size(); i++)
  {
    // drawing of the ball model in the field view
    DEBUG_DRAWING("representation:MultipleBallModel", "drawingOnField")
    {
      const Vector2f& position(ballModels[i].estimate.position);
      const Vector2f& velocity(ballModels[i].estimate.velocity);
      const ColorRGBA colorForDrawing(255, 0, 0, 128);
      CIRCLE("representation:MultipleBallModel",
          position.x(),
          position.y(),
          45,
          0, // pen width
          Drawings::solidPen,
          colorForDrawing,
          Drawings::solidBrush,
          colorForDrawing);
      ARROW("representation:MultipleBallModel", position.x(), position.y(), position.x() + velocity.x(), position.y() + velocity.y(), 5, 1, colorForDrawing);
      DRAWTEXT("representation:MultipleBallModel", position.x() + 50, position.y() + 50, 50, colorForDrawing, static_cast<int>(ballModels[i].validity * 100.f + 0.5f) << "%");
    }

    // drawing of the end position
    DEBUG_DRAWING("representation:MultipleBallModel:endPosition", "drawingOnField")
    {
      Vector2f position = BallPhysics::getEndPosition(ballModels[i].estimate.position, ballModels[i].estimate.velocity, ballModels[i].friction);
      CIRCLE("representation:MultipleBallModel:endPosition",
          position.x(),
          position.y(),
          45,
          0, // pen width
          Drawings::solidPen,
          ColorRGBA::black,
          Drawings::solidBrush,
          ColorRGBA(168, 25, 99, 220));
    }

    DEBUG_DRAWING3D("representation:MultipleBallModel", "robot")
    {
      TRANSLATE3D("representation:MultipleBallModel", 0, 0, -230);
      if (SystemCall::getTimeSince(ballModels[i].timeWhenLastSeen) < 5000 && SystemCall::getTimeSince(ballModels[i].timeWhenDisappeared) < 1000)
      {
        SPHERE3D("representation:MultipleBallModel", ballModels[i].estimate.position.x(), ballModels[i].estimate.position.y(), ballModels[i].estimate.radius, ballModels[i].estimate.radius, ColorRGBA::orange);
        LINE3D("representation:MultipleBallModel", 0, 0, 1.f, ballModels[i].estimate.position.x(), ballModels[i].estimate.position.y(), 1.f, 5.f, ColorRGBA::orange);
      }
    }
  }
}

void GroundTruthMultipleBallModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:GroundTruthMultipleBallModel", "drawingOnField");

  for (size_t i = 0; i < ballModels.size(); i++)
  {
    DEBUG_DRAWING("representation:GroundTruthMultipleBallModel", "drawingOnField")
    {
      const Vector2f& position(ballModels[i].estimate.position);
      const Vector2f& velocity(ballModels[i].estimate.velocity);
      const ColorRGBA colorForDrawing(0, 0, 0, 128);
      CIRCLE("representation:GroundTruthMultipleBallModel",
          position.x(),
          position.y(),
          45,
          0, // pen width
          Drawings::solidPen,
          colorForDrawing,
          Drawings::solidBrush,
          colorForDrawing);
      ARROW("representation:GroundTruthMultipleBallModel", position.x(), position.y(), position.x() + velocity.x(), position.y() + velocity.y(), 5, 1, colorForDrawing);
    }
  }
}
