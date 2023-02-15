#include "Tools/Math/Geometry.h"
#include "Tools/Math/Pose2f.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/CenterCirclePercept.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Perception/PenaltyCrossPercept.h"

class PoseGenerator
{
public:
  /**
  * returns position as if opponent goal posts are observed!
  */
  static bool getPoseFromGoalObservation(const FieldDimensions& fd, const Vector2f& leftPercept, const Vector2f& rightPercept, Pose2f& ownPosition)
  {
    Vector2f goalLeft(fd.xPosOpponentGoalPost, fd.yPosLeftGoal);
    Vector2f goalRight(fd.xPosOpponentGoalPost, fd.yPosRightGoal);
    Vector2f posLeft(leftPercept);
    Vector2f posRight(rightPercept);
    // if behind goal posts.. check if this is already handled by percepts!!?!
    bool switched = false;
    if (posLeft.angle() < posRight.angle())
    {
      switched = true;
      Vector2f dummy(posRight);
      posRight = posLeft;
      posLeft = dummy;
    }
    float a = posRight.norm();
    float b = posLeft.norm();
    if (b == 0)
      return false;
    float c = 1500.f;
    float n = Angle::normalize((c * c + b * b - a * a) / (2 * b * c));
    if (std::abs(n) > 1)
      return false;
    //float gamma = posLeft.angle() - posRight.angle();
    float alpha = std::acos(n);

    Vector2f posLeftRotated(0.f, -1000.f);
    float rotationInWC = Angle::normalize(pi_2 - alpha - posLeft.angle());
    if (switched)
      posLeftRotated.rotate(alpha);
    else
      posLeftRotated.rotate(-alpha);
    posLeftRotated.normalize(b);
    Vector2f newPos = goalLeft + posLeftRotated;
    ownPosition.rotation = rotationInWC;
    ownPosition.translation = newPos;
    return true;
  }

  /**
  * returns position using goal(!) line(s) and penalty cross
  * assumes to see opponent penalty cross and goal line
  */
  static bool getPoseFromPenaltyCrossAndGoalLine(const FieldDimensions& fd, const CLIPFieldLinesPercept& flp, const PenaltyCrossPercept& pcp, Pose2f& position, float& perceptWeight)
  {
    perceptWeight = 0.f;

    if (!(pcp.penaltyCrossWasSeen && flp.lines.size() > 0))
      return false;
    Vector2f pcOnField = Vector2f(pcp.pointOnField.cast<float>());
    if (!pcp.penaltyCrossWasSeen || pcOnField.norm() > 2000)
      return false;
    Vector2f pcWC = Vector2f(fd.xPosOwnPenaltyMark, 0.f);
    std::vector<CLIPFieldLinesPercept::FieldLine>::const_iterator fl = flp.lines.begin();
    const float distPCToGoalArea = std::abs(fd.xPosOwnGoalArea - fd.xPosOwnPenaltyMark);
    for (; fl != flp.lines.end(); ++fl)
    {
      Geometry::Line line;
      line.base.x() = static_cast<float>(fl->startOnField.x());
      line.base.y() = static_cast<float>(fl->startOnField.y());
      line.direction.x() = static_cast<float>(fl->endOnField.x() - fl->startOnField.x());
      line.direction.y() = static_cast<float>(fl->endOnField.y() - fl->startOnField.y());
      float distToPC = Geometry::getDistanceToLine(line, pcOnField);
      if (std::abs(std::abs(distToPC) - distPCToGoalArea) < 100)
      {
        float myDistToLine = Geometry::getDistanceToLine(line, Vector2f::Zero());
        float myDistToPC = pcOnField.norm();
        if (myDistToPC == 0)
          return false;
        float div = (std::abs(distToPC - myDistToLine) / myDistToPC);
        if (std::abs(div) > 1)
          return false;
        float alpha = std::asin(div);

        if ((alpha + alpha) != (alpha + alpha))
          return false;

        float yAbs = std::abs(std::cos(alpha) * myDistToPC);
        position.translation.y() = yAbs;

        // case: between pc and center circle
        if (distToPC < myDistToLine && distToPC > 0)
        {
          position.translation.x() = fd.xPosOwnGoalArea + myDistToLine;
        }
        // case: between pc and line
        else
        {
          position.translation.x() = fd.xPosOwnGoalArea + myDistToLine;
          Vector2f temp = position.translation;
          temp.x() += 2 * myDistToLine;
          if (std::abs((position.translation - pcWC).norm() - myDistToPC) > std::abs((temp - pcWC).norm() - myDistToPC))
            position.translation.x() = temp.x();
        }

        position = getPoseFromLandmarkAndLine(pcWC, pcOnField, position, line.direction.angle());
        perceptWeight = static_cast<float>(fl->validity);
        return true;
      }
    }
    return false;
  }

  /**
  * returns position using penalty(!) line(s) and penalty cross
  * assumes to see opponent penalty cross and penalty area line
  */
  static bool getPoseFromPenaltyCrossAndPenaltyLine(const FieldDimensions& fd, const CLIPFieldLinesPercept& flp, const PenaltyCrossPercept& pcp, Pose2f& position, float& perceptWeight)
  {
    perceptWeight = 0.f;

    if (!(pcp.penaltyCrossWasSeen && flp.lines.size() > 0))
      return false;
    Vector2f pcOnField = Vector2f(pcp.pointOnField.cast<float>());
    if (!pcp.penaltyCrossWasSeen || pcOnField.norm() > 2000)
      return false;
    Vector2f pcWC = Vector2f(fd.xPosOwnPenaltyMark, 0.f);
    std::vector<CLIPFieldLinesPercept::FieldLine>::const_iterator fl = flp.lines.begin();
    const float distPCToPenaltyArea = std::abs(fd.xPosOwnPenaltyArea - fd.xPosOwnPenaltyMark);
    for (; fl != flp.lines.end(); ++fl)
    {
      Geometry::Line line;
      line.base.x() = static_cast<float>(fl->startOnField.x());
      line.base.y() = static_cast<float>(fl->startOnField.y());
      line.direction.x() = static_cast<float>(fl->endOnField.x() - fl->startOnField.x());
      line.direction.y() = static_cast<float>(fl->endOnField.y() - fl->startOnField.y());
      float distToPC = Geometry::getDistanceToLine(line, pcOnField);
      if (std::abs(std::abs(distToPC) - distPCToPenaltyArea) < 75)
      {
        float myDistToLine = Geometry::getDistanceToLine(line, Vector2f::Zero());
        float myDistToPC = pcOnField.norm();
        if (myDistToPC == 0)
          return false;
        float div = (std::abs(distToPC - myDistToLine) / myDistToPC);
        if (std::abs(div) > 1)
          return false;
        float alpha = std::asin(div);

        if ((alpha + alpha) != (alpha + alpha))
          return false;

        float yAbs = std::abs(std::cos(alpha) * myDistToPC);
        position.translation.y() = yAbs;

        // case: below pc
        if (distToPC < myDistToLine && distToPC > 0)
        {
          position.translation.x() = fd.xPosOwnPenaltyArea - myDistToLine;
        }
        // case: between pc and line
        else
        {
          position.translation.x() = fd.xPosOwnPenaltyArea - myDistToLine;
          Vector2f temp = position.translation;
          temp.x() += 2 * myDistToLine;
          if (std::abs((position.translation - pcWC).norm() - myDistToPC) > std::abs((temp - pcWC).norm() - myDistToPC))
            position.translation.x() = temp.x();
        }

        position = getPoseFromLandmarkAndLine(pcWC, pcOnField, position, line.direction.angle());
        perceptWeight = static_cast<float>(fl->validity);
        return true;
      }
    }
    return false;
  }

  /**
   * Calculates robot position (pose) using center circle and crossing line on own half.
   * \param [in] fieldLines The percept of the field lines.
   * \param [in] centerCircle The percept of the center-circle.
   * \param [out] pose The position calculated by this method.
   * \param [out] perceptWeight The weight of the percept calculated by this method.
   *                            Confidence of what is seen matches a center-circle-line [0..1].
   *                            Is set to 0 if not match has been found.
   * \return confidence of the calculated position [0..1].
   */
  static float getPoseFromCenterCircleAndCenterLine(const CLIPFieldLinesPercept& fieldLines, const CLIPCenterCirclePercept& centerCircle, Pose2f& pose, float& perceptWeight)
  {
    // Reset perceptWeight to the minimum.
    perceptWeight = 0.f;

    // Find the line in the middle of the center circle.
    int centerLineIndex = getCenterCircleCenterLineIndex(centerCircle, fieldLines);
    if (centerLineIndex < 0) // index = -1 if no center line was found.
    {
      // No pose found. Return confidence = 0.
      return 0;
    }


    // Get the corresponding line object.
    const CLIPFieldLinesPercept::FieldLine& centerLine = fieldLines.lines[centerLineIndex];
    perceptWeight = (float)centerLine.validity; // Return center line validity as perceptWeight.

    // Get rotation (relative to the robot) of the center line.
    Vector2f centerLineDirection = centerLine.endOnField - centerLine.startOnField;
    float centerLineAngle = Angle::normalize(centerLineDirection.angle());
    // Reduce line angle to ]0, pi]
    if (centerLineAngle <= 0)
      centerLineAngle += pi;

    // Calculate the rotation of the robot pose from the center line.
    // The robot angle is rotated by 90 degrees relative to the line angle.
    pose.rotation = Angle::normalize(pi_2 - centerLineAngle); // Angle in intervall [-pi/2, pi/2[

    // Get the distance to the center circle.
    // This is the distance from the robot pose to (0,0).
    float centerCircleDistance = centerCircle.centerCircle.locationOnField.cast<float>().norm();

    // Get the angle by which the robot has seen the center circle.
    float centerCircleAngle = centerCircle.centerCircle.locationOnField.cast<float>().angle();

    // Calculate the angle from (0,0) to the pose hypothesis.
    // This should be the result of pose.angle().
    float poseTranslationAngle = Angle::normalize(pose.rotation + centerCircleAngle + pi);
    // Mirror pose to the own half if neccessary.
    if (poseTranslationAngle < pi_2 && poseTranslationAngle >= -pi_2) // Angle in intervall [-pi , -pi/2[ or [pi/2, pi[
    {
      poseTranslationAngle = Angle::normalize(poseTranslationAngle + pi);
      pose.rotation = Angle::normalize(pose.rotation + pi);
    }

    // Calculate x and y coordinates from angle and distance to center circle.
    pose.translation.x() = centerCircleDistance * std::cos(poseTranslationAngle);
    pose.translation.y() = centerCircleDistance * std::sin(poseTranslationAngle);

    // Return high confidence because a pose was found.
    return 1;
  }

  static Pose2f getPoseFromLandmarkAndLine(const Vector2f& landmarkWC, const Vector2f& landmarkRC, const Pose2f& pose, const float& lineAngle)
  {
    Pose2f poseA(pose);
    Pose2f poseB(pose);
    poseB.translation.y() *= -1;
    float landMarkAngleWC = (landmarkWC - poseA.translation).angle();
    float landMarkAngleRC = landmarkRC.angle();
    poseA.rotation = Angle::normalize(landMarkAngleWC - landMarkAngleRC);

    landMarkAngleWC = (landmarkWC - poseB.translation).angle();
    poseB.rotation = Angle::normalize(landMarkAngleWC - landMarkAngleRC);
    float minDiffPoseA = std::min(std::abs(Angle::normalize(poseA.rotation + lineAngle - pi_2)), std::abs(Angle::normalize(poseA.rotation + lineAngle - pi - pi_2)));
    float minDiffPoseB = std::min(std::abs(Angle::normalize(poseB.rotation + lineAngle - pi_2)), std::abs(Angle::normalize(poseB.rotation + lineAngle - pi - pi_2)));
    if (minDiffPoseA > minDiffPoseB)
      poseA = poseB;

    return poseA;
  }

  /**************** HELPING METHODS ****************/
private:
  /**
   * Gets the index of the center-line of the center-circle in theFieldLines.lines vector.
   * \param [in] centerCircle The percept of the center-circle.
   * \param [in] fieldLines The percept of the field lines.
   * \return The resulting index of the center-line. This is set to -1 if no matching 
   *         line is found or if the center-circle is not seen.
   */
  static int getCenterCircleCenterLineIndex(const CLIPCenterCirclePercept& centerCircle, const CLIPFieldLinesPercept& fieldLines)
  {
    // Parameter:
    // The maximum distance between the line and the center of the center circle.
    float maxDistanceToCenter = 100; // Distance in millimeter


    // Set lineIndex to an invalid value.
    int lineIndex = -1;

    // Cannot find center line if the center circle was not seen.
    if (!centerCircle.centerCircleWasSeen)
      return -1; // Return invalid line index.

    // Search for the first line which is near enough to the center circle.
    for (size_t i = 0; i < fieldLines.lines.size(); i++)
    {
      float distanceToCenter = centerCircle.getDistanceToFieldLinesPercept(fieldLines.lines[i]);
      if (distanceToCenter < maxDistanceToCenter)
      {
        // Center line found.
        lineIndex = (int)i;
        break;
      }
    }

    // Return the index of the found line or -1 if no line was found.
    return lineIndex;
  }
};