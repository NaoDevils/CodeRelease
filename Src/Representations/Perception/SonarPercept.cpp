#include "SonarPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Modeling/RobotPose.h"

#define SONAR_PERCEPT_DRAWING_FIELD "representation:SonarPercept:Field"

void SonarPercept::draw() const
{
  DECLARE_DEBUG_DRAWING(SONAR_PERCEPT_DRAWING_FIELD, "drawingOnField");
  // Field
  const RobotDimensions& theRobotDimensions = Blackboard::get<RobotDimensions>();
  const RobotPose& theRobotPose = Blackboard::get<RobotPose>();
  for (const SonarEstimate& sonarEstimate : sonarEstimates)
  {
    ColorRGBA color;
    Vector2f sonarSensorRelativePosition;
    if (sonarEstimate.sensorPosition == SonarEstimate::SensorPosition::left)
    {
      color = ColorRGBA::yellow;
      sonarSensorRelativePosition = theRobotDimensions.leftSonarInfo.translation;
    }
    else if (sonarEstimate.sensorPosition == SonarEstimate::SensorPosition::middle)
    {
      color = ColorRGBA::orange;
      sonarSensorRelativePosition = Vector2f(theRobotDimensions.leftSonarInfo.translation.x(), 0);
    }
    else if (sonarEstimate.sensorPosition == SonarEstimate::SensorPosition::right)
    {
      color = ColorRGBA::purple;
      sonarSensorRelativePosition = theRobotDimensions.rightSonarInfo.translation;
    }
    else
      OUTPUT_ERROR("Sonar estimate position must be either left, middle or right.");

    drawArc(sonarEstimate, theRobotPose, theRobotDimensions.sonarOpeningAngle, sonarSensorRelativePosition, color);
  }
}

void SonarPercept::drawArc(const SonarEstimate& sonarEstimate, const RobotPose& theRobotPose, const Angle& sonarDetectionCone, const Vector2f& sonarSensorRelativePosition, const ColorRGBA& arcColor) const
{
  Vector2f sonarSensorPositionOnField = Transformation::robotToField(theRobotPose, sonarSensorRelativePosition);
  Vector2f sonarEstimatePositionOnField = Transformation::robotToField(theRobotPose, sonarEstimate.relativePosition.translation);
  Vector2f distanceTextPositionOnField = Transformation::robotToField(theRobotPose, sonarEstimate.relativePosition.translation + sonarEstimate.relativePosition.translation.normalized() * 250);
  // CIRCLE
  CIRCLE(SONAR_PERCEPT_DRAWING_FIELD, sonarEstimatePositionOnField.x(), sonarEstimatePositionOnField.y(), 40, 2, Drawings::solidPen, arcColor, Drawings::solidBrush, arcColor);
  // ARC
  ARC(SONAR_PERCEPT_DRAWING_FIELD,
      sonarSensorPositionOnField.x(),
      sonarSensorPositionOnField.y(),
      sonarEstimate.distance - sonarSensorRelativePosition.norm(),
      Pose2f(theRobotPose).rotate(sonarEstimate.relativePosition.rotation - sonarDetectionCone / 2).rotation,
      sonarDetectionCone,
      20,
      Drawings::solidPen,
      arcColor,
      Drawings::solidBrush,
      arcColor);
  // TEXT
  DRAWTEXT(SONAR_PERCEPT_DRAWING_FIELD, distanceTextPositionOnField.x(), distanceTextPositionOnField.y(), 80, arcColor, std::round(sonarEstimate.distance / 10) << "cm");
}
