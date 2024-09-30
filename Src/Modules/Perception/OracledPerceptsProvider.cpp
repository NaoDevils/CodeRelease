/**
* @file Modules/Infrastructure/OracledPerceptsProvider.h
*
* This file implements a module that provides precepts based on simulated data.
*
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
*/

#include "OracledPerceptsProvider.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Modeling/Obstacle.h"

OracledPerceptsProvider::OracledPerceptsProvider()
{
  // Four goal posts
  goalPosts.push_back(Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal));
  goalPosts.push_back(Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal));
  goalPosts.push_back(Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal));
  goalPosts.push_back(Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal));
  // Two penalty marks
  penaltyMarks.push_back(Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f));
  penaltyMarks.push_back(Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f));
  // Five points roughly approximating the center circle
  ccPoints.push_back(Vector2f::Zero());
  ccPoints.push_back(Vector2f(0.f, theFieldDimensions.centerCircleRadius));
  ccPoints.push_back(Vector2f(0.f, -theFieldDimensions.centerCircleRadius));
  ccPoints.push_back(Vector2f(theFieldDimensions.centerCircleRadius, 0.f));
  ccPoints.push_back(Vector2f(-theFieldDimensions.centerCircleRadius, 0.f));
  // The lines
  std::pair<Vector2f, Vector2f> line;
  // ground lines and center line: 1-3
  line.first = Vector2f(0, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2f(0, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  // side lines: 4-5
  line.first = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline);
  line.second = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  // opponent penalty area: 6-8
  line.first = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  // own penalty area: 9-11
  line.first = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  // opponent goal area: 12-14
  line.first = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftGoalArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosLeftGoalArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightGoalArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosLeftGoalArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea);
  lines.push_back(line);
  // own goal area: 15-17
  line.first = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoalArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoalArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea);
  lines.push_back(line);
  // opponent goal line: 18
  line.first = Vector2f(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosLeftGoal);
  line.second = Vector2f(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosRightGoal);
  lines.push_back(line);
  // own goal line: 19
  line.first = Vector2f(theFieldDimensions.xPosOwnGoal, theFieldDimensions.yPosLeftGoal);
  line.second = Vector2f(theFieldDimensions.xPosOwnGoal, theFieldDimensions.yPosRightGoal);
  lines.push_back(line);
}

void OracledPerceptsProvider::update(BallPercept& ballPercept)
{
  ballPercept.status = BallPercept::notSeen;
  if (!theCameraMatrix.isValid)
    return;
  if (theGroundTruthWorldState.balls.size() != 0)
  {
    updateBallPercept(ballPercept, 0);
  }
}

void OracledPerceptsProvider::update(MultipleBallPercept& multipleBallPercept)
{
  if (multipleBallPercept.balls.size() != theGroundTruthWorldState.balls.size())
    multipleBallPercept.balls.resize(theGroundTruthWorldState.balls.size());

  for (size_t i = 0; i < theGroundTruthWorldState.balls.size(); i++)
  {
    multipleBallPercept.balls[i].status = BallPercept::notSeen;
    if (!theCameraMatrix.isValid)
      return;

    updateBallPercept(multipleBallPercept.balls[i], i);
  }
}

void OracledPerceptsProvider::updateBallPercept(BallPercept& ballPercept, size_t index)
{
  const Vector2f ballOnField = theGroundTruthWorldState.balls[index];
  Vector2f ballOffset = theGroundTruthWorldState.ownPose.inverse() * ballOnField;
  float ballDistance = ballOffset.norm();
  if (ballDistance > ballMaxVisibleDistance)
    return;
  float recognitionRate = ballRecognitionRateMin + (ballRecognitionRateMax - ballRecognitionRateMin) * (1.f - (ballDistance / ballMaxVisibleDistance));
  if (randomFloat() > recognitionRate)
    return;
  Geometry::Circle circle, circleUpper;
  bool upper = false;
  bool inImage = false;
  if (Geometry::calculateBallInImage(ballOffset, theCameraMatrixUpper, theCameraInfoUpper, theFieldDimensions.ballRadius, circleUpper)
      && circleUpper.center.x() > circleUpper.radius && circleUpper.center.x() < theCameraInfoUpper.width - circleUpper.radius && circleUpper.center.y() > circleUpper.radius
      && circleUpper.center.y() < theCameraInfoUpper.height - circleUpper.radius)
  {
    inImage = true;
    upper = true;
  }
  if (Geometry::calculateBallInImage(ballOffset, theCameraMatrix, theCameraInfo, theFieldDimensions.ballRadius, circle) && circle.center.x() > circle.radius
      && circle.center.x() < theCameraInfo.width - circle.radius && circle.center.y() > circle.radius && circle.center.y() < theCameraInfo.height - circle.radius)
  {
    inImage = true;
    upper = false;
  }
  if (inImage)
  {
    const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
    const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
    if (upper)
      circle = circleUpper;
    if ((circle.center.x() >= -circle.radius / 1.5f) && (circle.center.x() < cameraInfo.width + circle.radius / 1.5f) && (circle.center.y() >= -circle.radius / 1.5f)
        && (circle.center.y() < cameraInfo.height + circle.radius / 1.5f))
    {
      ballPercept.status = BallPercept::seen;
      ballPercept.timestamp = theFrameInfo.time;
      ballPercept.positionInImage = circle.center;
      ballPercept.radiusInImage = circle.radius;
      ballPercept.relativePositionOnField = ballOffset;
      ballPercept.fromUpper = upper;
      ballPercept.validity = randomFloat(0.80f, 1.0f);
      // Add some noise:
      if (applyBallNoise)
      {
        applyNoise(ballCenterInImageStdDevInPixel, ballPercept.positionInImage);
        if (!Transformation::imageToRobotHorizontalPlane(ballPercept.positionInImage, theFieldDimensions.ballRadius, cameraMatrix, cameraInfo, ballPercept.relativePositionOnField))
        {
          ballPercept.status = BallPercept::notSeen;
          return;
        }
      }
    }
  }
}

void OracledPerceptsProvider::update(CLIPFieldLinesPercept& linePercept)
{
  linePercept.lines.clear();
  if (!theCameraMatrix.isValid)
    return;
  updateViewPolygon(false);
  updateViewPolygon(true);
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();

  // Find lines:
  for (unsigned int i = 0; i < lines.size(); i++)
  {
    Vector2f start, end;
    if (partOfLineIsVisible(lines[i], start, end, false))
    {
      addLinePercept(false, start, end, robotPoseInv, linePercept);
    }
    if (partOfLineIsVisible(lines[i], start, end, true))
    {
      addLinePercept(true, start, end, robotPoseInv, linePercept);
    }
  }
}

void OracledPerceptsProvider::addLinePercept(const bool upper, const Vector2f& start, const Vector2f& end, const Pose2f& robotPoseInv, CLIPFieldLinesPercept& linePercept)
{
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;

  CLIPFieldLinesPercept::FieldLine line;
  line.fromUpper = upper;
  line.startOnField = robotPoseInv * start;
  line.endOnField = robotPoseInv * end;
  if (line.startOnField.norm() > lineMaxVisibleDistance || line.endOnField.norm() > lineMaxVisibleDistance)
    return;
  float midLineDistance = (line.startOnField.norm() + line.endOnField.norm()) / 2.f;
  float recognitionRate = lineRecognitionRateMin + (lineRecognitionRateMax - lineRecognitionRateMin) * (1.f - (midLineDistance / lineMaxVisibleDistance));
  if (randomFloat() > recognitionRate)
    return;

  if (applyLineNoise)
  {
    applyNoise(linePosStdDevInmm, line.startOnField);
    applyNoise(linePosStdDevInmm, line.endOnField);
  }

  Vector2f pImg;
  if (Transformation::robotToImage(line.startOnField, cameraMatrix, cameraInfo, pImg))
  {
    line.startInImage = pImg.cast<int>();
    if (Transformation::robotToImage(line.endOnField, cameraMatrix, cameraInfo, pImg))
    {
      line.endInImage = pImg.cast<int>();
      bool success = true;
      Vector2i pField;
      success = Transformation::imageToRobot(line.startInImage.x(), line.startInImage.y(), cameraMatrix, cameraInfo, pField)
          && Transformation::imageToRobot(line.endInImage.x(), line.endInImage.y(), cameraMatrix, cameraInfo, pField);
      if (success)
      {
        line.isPlausible = true;
        Vector2i imageVector = (line.endInImage - line.startInImage);
        float length = imageVector.cast<float>().norm();
        line.validity = std::min(length / (cameraInfo.width / 2), 1.f);
        linePercept.lines.push_back(line);
      }
    }
  }
}

void OracledPerceptsProvider::update(PenaltyCrossPercept& penaltyMarkPercept)
{
  penaltyMarkPercept.penaltyCrossWasSeen = false;
  if (!theCameraMatrix.isValid)
    return;
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  for (auto& pos : penaltyMarks)
  {
    Vector2f relativeMarkPos = robotPoseInv * pos;
    float penaltyMarkDistance = relativeMarkPos.norm();
    if (penaltyMarkDistance > penaltyMarkMaxVisibleDistance)
      continue;
    float recognitionRate = penaltyMarkRecognitionRateMin + (penaltyMarkRecognitionRateMax - penaltyMarkRecognitionRateMin) * (1.f - (penaltyMarkDistance / penaltyMarkMaxVisibleDistance));
    if (randomFloat() > recognitionRate)
      continue;
    Vector2f penaltyMarkInImage;
    bool upper = false;
    if (pointIsInImage(relativeMarkPos, penaltyMarkInImage, upper))
    {
      bool success = true;
      if (applyPenaltyMarkNoise)
      {
        const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
        const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
        applyNoise(penaltyMarkPosInImageStdDevInPixel, penaltyMarkInImage);
        success = Transformation::imageToRobot(penaltyMarkInImage, cameraMatrix, cameraInfo, relativeMarkPos);
      }
      if (success)
      {
        penaltyMarkPercept.pointOnField = relativeMarkPos.cast<int>();
        penaltyMarkPercept.pointInImage = Vector2i(static_cast<int>(penaltyMarkInImage.x()), static_cast<int>(penaltyMarkInImage.y()));
        penaltyMarkPercept.fromUpper = upper;
        penaltyMarkPercept.penaltyCrossWasSeen = true;
      }
    }
  }
}

void OracledPerceptsProvider::update(RobotsPercept& playersPercept)
{
  playersPercept.robots.clear();
  if (!theCameraMatrix.isValid || !Global::hasSettings())
    return;

  for (const auto& bluePlayer : theGroundTruthWorldState.bluePlayers)
    createPlayerBox(bluePlayer, theOwnTeamInfo.teamNumber != 1, playersPercept);
  for (const auto& redPlayer : theGroundTruthWorldState.redPlayers)
    createPlayerBox(redPlayer, theOwnTeamInfo.teamNumber != 2, playersPercept);
}

void OracledPerceptsProvider::update(CLIPCenterCirclePercept& centerCirclePercept)
{
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  // Find center circle (at least one out of five center circle points must be inside the current image)
  int pointsFound = 0;
  bool upper = false;
  centerCirclePercept.centerCircleWasSeen = false;

  float centerCircleDistance = theGroundTruthWorldState.ownPose.translation.norm();
  if (centerCircleDistance > centerCircleMaxVisibleDistance)
    return;
  float recognitionRate = centerCircleRecognitionRateMin + (centerCircleRecognitionRateMax - centerCircleRecognitionRateMin) * (1.f - (centerCircleDistance / centerCircleMaxVisibleDistance));
  float randomValue = randomFloat();
  if (randomValue > recognitionRate)
    return;

  for (unsigned int i = 0; i < ccPoints.size(); ++i)
  {
    const Vector2f relPos = robotPoseInv * ccPoints[i];
    Vector2f posInImage;
    if (pointIsInImage(relPos, posInImage, upper))
    {
      pointsFound++;
      break;
    }
  }

  if (pointsFound > 0)
  {
    recognitionRate = 0.5f * recognitionRate + 0.5f * recognitionRate * (pointsFound / 5);
    if (randomValue > recognitionRate)
      return;

    const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
    const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
    centerCirclePercept.centerCircle.locationOnField = (robotPoseInv * Vector2f::Zero()).cast<int>();
    centerCirclePercept.centerCircleWasSeen = true;
    centerCirclePercept.fromUpper = upper;

    // Add some noise:
    if (applyCenterCircleNoise)
    {
      Vector2f nPImg;
      if (Transformation::robotToImage(Vector2f(centerCirclePercept.centerCircle.locationOnField.cast<float>()), cameraMatrix, cameraInfo, nPImg))
      {
        applyNoise(centerCircleCenterInImageStdDevInPixel, nPImg);
        centerCirclePercept.centerCircle.locationInImage = nPImg.cast<int>();
        Vector2f pField;
        if (!Transformation::imageToRobot(nPImg, cameraMatrix, cameraInfo, pField))
        {
          centerCirclePercept.centerCircle.locationOnField = pField.cast<int>();
          centerCirclePercept.centerCircleWasSeen = false;
        }
      }
    }
  }
}

void OracledPerceptsProvider::createPlayerBox(const GroundTruthWorldState::GroundTruthPlayer& player, bool isOpponent, RobotsPercept& playersPercept)
{
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  Vector2f relativePlayerPos = robotPoseInv * player.pose.translation;
  float playerDistance = relativePlayerPos.norm();
  if (playerDistance > playerMaxVisibleDistance)
    return;
  float recognitionRate = playerRecognitionRateMin + (playerRecognitionRateMax - playerRecognitionRateMin) * (1.f - (playerDistance / playerMaxVisibleDistance));
  if (randomFloat() > recognitionRate)
    return;
  Vector2f playerInImage;
  bool upper;
  if (pointIsInImage(relativePlayerPos, playerInImage, upper))
  {
    const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
    const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
    bool success = true;
    if (applyPlayerNoise)
    {
      applyNoise(playerPosInImageStdDevInPixel, playerInImage);
      success = Transformation::imageToRobot(playerInImage, cameraMatrix, cameraInfo, relativePlayerPos);
    }
    if (success)
    {
      RobotEstimate re;
      float distance = relativePlayerPos.norm();
      int width = static_cast<int>(Geometry::getSizeByDistance(cameraInfo, 200, distance));
      re.fromUpperImage = upper;
      re.distance = distance;
      re.validity = 1.f;
      // TODO: noise for robot type
      float jerseyValue = randomFloat();
      if (jerseyValue <= teamRecognitionRateMin) // Correct
        re.robotType = isOpponent ? RobotEstimate::RobotType::opponentRobot : RobotEstimate::RobotType::teammateRobot;
      else if (jerseyValue <= teamRecognitionRateMax) // Unknown
        re.robotType = RobotEstimate::RobotType::unknownRobot;
      else // FP
        re.robotType = isOpponent ? RobotEstimate::RobotType::teammateRobot : RobotEstimate::RobotType::opponentRobot;
      //re.robotType = isOpponent ? RobotEstimate::RobotType::opponentRobot : RobotEstimate::RobotType::teammateRobot;
      re.imageLowerRight = playerInImage.cast<int>();
      re.imageLowerRight.x() += width / 2;
      re.imageUpperLeft = playerInImage.cast<int>();
      re.imageUpperLeft.x() -= width / 2;
      re.imageUpperLeft.y() = std::max(4, re.imageUpperLeft.y() - static_cast<int>(Geometry::getSizeByDistance(cameraInfo, 500, distance)));
      re.locationOnField = Pose2f(relativePlayerPos);
      playersPercept.robots.push_back(re);
    }
  }
}

bool OracledPerceptsProvider::pointIsInImage(const Vector2f& p, Vector2f& pImg, bool& upper) const
{
  upper = false;
  if (Transformation::robotToImage(p, theCameraMatrix, theCameraInfo, pImg))
  {
    if ((pImg.x() >= 0) && (pImg.x() < theCameraInfo.width) && (pImg.y() >= 0) && (pImg.y() < theCameraInfo.height))
    {
      return true;
    }
  }
  if (Transformation::robotToImage(p, theCameraMatrixUpper, theCameraInfoUpper, pImg))
  {
    if ((pImg.x() >= 0) && (pImg.x() < theCameraInfoUpper.width) && (pImg.y() >= 0) && (pImg.y() < theCameraInfoUpper.height))
    {
      upper = true;
      return true;
    }
  }
  return false;
}

void OracledPerceptsProvider::updateViewPolygon(bool upper)
{
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  Vector2f* vP = upper ? viewPolygonUpper : viewPolygon;
  // code is copied from FieldCoverageProvider::drawFieldView()
  const Vector3f vectorToCenter(1, 0, 0);

  RotationMatrix r = cameraMatrix.rotation;
  r.rotateY(cameraInfo.openingAngleHeight / 2);
  r.rotateZ(cameraInfo.openingAngleWidth / 2);
  Vector3f vectorToCenterWorld = r * vectorToCenter;

  const float a1 = cameraMatrix.translation.x(), a2 = cameraMatrix.translation.y(), a3 = cameraMatrix.translation.z();
  float b1 = vectorToCenterWorld.x(), b2 = vectorToCenterWorld.y(), b3 = vectorToCenterWorld.z(), f = a3 / b3;
  Vector2f pof = Vector2f(a1 - f * b1, a2 - f * b2);

  if (f > 0.f)
    vP[0] = theGroundTruthWorldState.ownPose.translation;
  else
    vP[0] = theGroundTruthWorldState.ownPose * pof;

  r = cameraMatrix.rotation;
  r.rotateY(cameraInfo.openingAngleHeight / 2);
  r.rotateZ(-(cameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x();
  b2 = vectorToCenterWorld.y();
  b3 = vectorToCenterWorld.z();
  f = a3 / b3;
  pof = Vector2f(a1 - f * b1, a2 - f * b2);

  if (f > 0.f)
    vP[1] = theGroundTruthWorldState.ownPose.translation;
  else
    vP[1] = theGroundTruthWorldState.ownPose * pof;

  r = cameraMatrix.rotation;
  r.rotateY(-(cameraInfo.openingAngleHeight / 2));
  r.rotateZ(-(cameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x();
  b2 = vectorToCenterWorld.y();
  b3 = vectorToCenterWorld.z();
  f = a3 / b3;
  pof = Vector2f(a1 - f * b1, a2 - f * b2);

  const float maxDist = std::sqrt(4.f * theFieldDimensions.xPosOpponentFieldBorder * theFieldDimensions.xPosOpponentFieldBorder
      + 4.f * theFieldDimensions.yPosLeftFieldBorder * theFieldDimensions.yPosLeftFieldBorder);
  if (f > 0.f)
    vP[2] = theGroundTruthWorldState.ownPose.translation
        + Vector2f(maxDist, 0).rotate(theGroundTruthWorldState.ownPose.rotation + (-cameraInfo.openingAngleWidth / 2) + cameraMatrix.rotation.getZAngle());
  else
    vP[2] = theGroundTruthWorldState.ownPose * pof;

  r = cameraMatrix.rotation;
  r.rotateY(-(cameraInfo.openingAngleHeight / 2));
  r.rotateZ(cameraInfo.openingAngleWidth / 2);
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x();
  b2 = vectorToCenterWorld.y();
  b3 = vectorToCenterWorld.z();
  f = a3 / b3;
  pof = Vector2f(a1 - f * b1, a2 - f * b2);

  if (f > 0.f)
    vP[3] = theGroundTruthWorldState.ownPose.translation
        + Vector2f(maxDist, 0).rotate(theGroundTruthWorldState.ownPose.rotation + (cameraInfo.openingAngleWidth / 2) + cameraMatrix.rotation.getZAngle());
  else
    vP[3] = theGroundTruthWorldState.ownPose * pof;
}

bool OracledPerceptsProvider::partOfLineIsVisible(const std::pair<Vector2f, Vector2f>& line, Vector2f& start, Vector2f& end, bool upper) const
{
  const Vector2f* vP = upper ? viewPolygonUpper : viewPolygon;
  // First case: both points are inside:
  if (Geometry::isPointInsideConvexPolygon(vP, 4, line.first) && Geometry::isPointInsideConvexPolygon(vP, 4, line.second))
  {
    start = line.first;
    end = line.second;
    return true;
  }
  // Second case: start is inside but end is outside
  if (Geometry::isPointInsideConvexPolygon(vP, 4, line.first) && !Geometry::isPointInsideConvexPolygon(vP, 4, line.second))
  {
    start = line.first;
    for (int i = 0; i < 4; i++)
    {
      if (Geometry::checkIntersectionOfLines(line.first, line.second, vP[i], vP[(i + 1) % 4]))
      {
        if (Geometry::getIntersectionOfLines(Geometry::Line(line.first, line.second - line.first), Geometry::Line(vP[i], vP[(i + 1) % 4] - vP[i]), end))
        {
          return true;
        }
      }
    }
    return false; // should not happen ...
  }
  // Third case: end is inside but start is outside
  if (!Geometry::isPointInsideConvexPolygon(vP, 4, line.first) && Geometry::isPointInsideConvexPolygon(vP, 4, line.second))
  {
    start = line.second;
    for (int i = 0; i < 4; i++)
    {
      if (Geometry::checkIntersectionOfLines(line.first, line.second, vP[i], vP[(i + 1) % 4]))
      {
        if (Geometry::getIntersectionOfLines(Geometry::Line(line.first, line.second - line.first), Geometry::Line(vP[i], vP[(i + 1) % 4] - vP[i]), end))
        {
          return true;
        }
      }
    }
    return false; // should not happen ...
  }
  // Fourth case: both points are outside the polygon but maybe intersect it:
  std::vector<Vector2f> intersectionPoints;
  for (int i = 0; i < 4; i++)
  {
    if (Geometry::checkIntersectionOfLines(line.first, line.second, vP[i], vP[(i + 1) % 4]))
    {
      Vector2f point;
      if (Geometry::getIntersectionOfLines(Geometry::Line(line.first, line.second - line.first), Geometry::Line(vP[i], vP[(i + 1) % 4] - vP[i]), point))
      {
        intersectionPoints.push_back(point);
      }
    }
  }
  // There are some more special cases that could be treated but in general, there should be two intersections that are not at the same place.
  // Other cases are ignored here
  if (intersectionPoints.size() == 2 && (intersectionPoints[0] - intersectionPoints[1]).norm() > 100.f)
  {
    start = intersectionPoints[0];
    end = intersectionPoints[1];
    return true;
  }
  // Sorry, nothing found ...
  return false;
}

void OracledPerceptsProvider::applyNoise(float standardDeviation, Vector2f& p) const
{
  p.x() += sampleNormalDistribution(standardDeviation);
  p.y() += sampleNormalDistribution(standardDeviation);
}

void OracledPerceptsProvider::applyNoise(float standardDeviation, Vector2i& p) const
{
  const float errorX = sampleNormalDistribution(standardDeviation);
  const float errorY = sampleNormalDistribution(standardDeviation);
  p.x() += static_cast<int>(floor(errorX + 0.5f));
  p.y() += static_cast<int>(floor(errorY + 0.5f));
}

void OracledPerceptsProvider::update(CLIPGoalPercept& goalPercept)
{
  goalPercept.goalPosts.clear();
  if (!theCameraMatrix.isValid)
    return;
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  for (unsigned int i = 0; i < goalPosts.size(); i++)
  {
    const Vector2f relativePostPos = robotPoseInv * goalPosts[i];
    float goalPostDistance = relativePostPos.norm();
    if (goalPostDistance > goalPostMaxVisibleDistance)
      continue;
    float recognitionRate = goalPostRecognitionRateMin + (goalPostRecognitionRateMax - goalPostRecognitionRateMin) * (1.f - (goalPostDistance / goalPostMaxVisibleDistance));
    if (randomFloat() > recognitionRate)
      continue;
    Vector2f postInImage;
    bool upper = false;
    if (pointIsInImage(relativePostPos, postInImage, upper))
    {
      const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
      const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
      CLIPGoalPercept::GoalPost newPost; // GoalPost::IS_UNKNOWN is the default side information
      newPost.bottomInImage.x() = static_cast<int>(std::floor(postInImage.x() + 0.5f));
      newPost.bottomInImage.y() = static_cast<int>(std::floor(postInImage.y() + 0.5f));
      newPost.topInImage.x() = newPost.bottomInImage.x();
      newPost.topInImage.y() = std::max(4, newPost.bottomInImage.y() - 50); // some default value
      newPost.bottomWidth = Geometry::getSizeByDistance(cameraInfo, theFieldDimensions.goalPostRadius, relativePostPos.norm());
      newPost.topWidth = newPost.bottomWidth; // not correct but enough for drawing
      newPost.fromUpper = upper;
      newPost.validity = 1.f;
      newPost.bottomFound = true;
      newPost.foundLineAtBottom = false;
      newPost.locationOnField = relativePostPos;
      // Add some noise:
      if (applyGoalPostNoise)
      {
        applyNoise(ballCenterInImageStdDevInPixel, newPost.bottomInImage); // TODO: why ballCenter..?
        if (!Transformation::imageToRobot(newPost.bottomInImage.x(), newPost.bottomInImage.y(), cameraMatrix, cameraInfo, newPost.locationOnField))
        {
          continue;
        }
      }
      // If a part of the goal bar might be in the image, there is also a side information:
      const Vector3f relativeBarPos(relativePostPos.x(), relativePostPos.y(), theFieldDimensions.goalHeight);
      Vector2f barInImage;
      if (Transformation::robotToImage(relativeBarPos, cameraMatrix, cameraInfo, barInImage))
      {
        if ((barInImage.x() >= 0.f) && (barInImage.x() < static_cast<float>(cameraInfo.width)) && (barInImage.y() >= 0.f) && (barInImage.y() < static_cast<float>(cameraInfo.height)))
        {
          newPost.topInImage = barInImage.cast<int>();
          if ((goalPosts[i].x() > 0 && goalPosts[i].y() > 0) || (goalPosts[i].x() < 0 && goalPosts[i].y() < 0))
            newPost.goalPostSide = CLIPGoalPercept::GoalPost::leftPost;
          else
            newPost.goalPostSide = CLIPGoalPercept::GoalPost::rightPost;
        }
      }
      goalPercept.goalPosts.push_back(newPost);
    }
  }
}


MAKE_MODULE(OracledPerceptsProvider, cognitionInfrastructure)
