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
  // ground lines and center line:
  line.first = Vector2f(0, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2f(0, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  // side lines:
  line.first = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline);
  line.second = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline);
  line.second = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline);
  lines.push_back(line);
  // opponent penalty area:
  line.first = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  // own penalty area:
  line.first = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  line.second = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  lines.push_back(line);
  // The field boundary
  line.first = Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  line.second = Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  fieldBoundaryLines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  line.second = Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  fieldBoundaryLines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  line.second = Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  fieldBoundaryLines.push_back(line);
  line.first = Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightFieldBorder);
  line.second = Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  fieldBoundaryLines.push_back(line);
}

void OracledPerceptsProvider::update(BallPercept& ballPercept)
{
  ballPercept.status = BallPercept::notSeen;
  if (!theCameraMatrix.isValid)
    return;
  if (theGroundTruthWorldState.balls.size() != 0)
  {
    const Vector2f ballOnField = theGroundTruthWorldState.balls[0];
    Vector2f ballOffset = theGroundTruthWorldState.ownPose.inverse() * ballOnField;
    if (ballOffset.norm() > ballMaxVisibleDistance)
      return;
    if (randomFloat() > ballRecognitionRate)
      return;
    Geometry::Circle circle, circleUpper;
    bool upper = false;
    bool inImage = false;
    if (Geometry::calculateBallInImage(ballOffset, theCameraMatrixUpper, theCameraInfoUpper, theFieldDimensions.ballRadius, circleUpper)
      && circleUpper.center.x() > circleUpper.radius && circleUpper.center.x() < theCameraInfoUpper.width - circleUpper.radius
      && circleUpper.center.y() > circleUpper.radius && circleUpper.center.y() < theCameraInfoUpper.height - circleUpper.radius)
    {
      inImage = true;
      upper = true;
    }
    if (Geometry::calculateBallInImage(ballOffset, theCameraMatrix, theCameraInfo, theFieldDimensions.ballRadius, circle)
      && circle.center.x() > circle.radius && circle.center.x() < theCameraInfo.width - circle.radius
      && circle.center.y() > circle.radius && circle.center.y() < theCameraInfo.height - circle.radius)
    {
      inImage = true;
      upper = false;
    }
    if (inImage)
    {
      const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
      const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
      if (upper) circle = circleUpper;
      if ((circle.center.x() >= -circle.radius / 1.5f) &&
        (circle.center.x() < cameraInfo.width + circle.radius / 1.5f) &&
        (circle.center.y() >= -circle.radius / 1.5f) &&
        (circle.center.y() < cameraInfo.height + circle.radius / 1.5f))
      {
        ballPercept.status = BallPercept::seen;
        ballPercept.positionInImage = circle.center;
        ballPercept.radiusInImage = circle.radius;
        ballPercept.relativePositionOnField = ballOffset;
        ballPercept.fromUpper = upper;
        ballPercept.validity = 1.f;
        // Add some noise
        if (applyBallNoise)
        {
          applyNoise(ballCenterInImageStdDev, ballPercept.positionInImage);
          if (!Transformation::imageToRobotHorizontalPlane(ballPercept.positionInImage, theFieldDimensions.ballRadius, cameraMatrix, cameraInfo, ballPercept.relativePositionOnField))
          {
            ballPercept.status = BallPercept::notSeen;
            return;
          }
        }
      }
    }
  }
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
    if (relativePostPos.norm() > goalPostMaxVisibleDistance)
      continue;
    if (randomFloat() > goalPostRecognitionRate)
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
        applyNoise(ballCenterInImageStdDev, newPost.bottomInImage); // TODO: why ballCenter..?
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
        if ((barInImage.x() >= 0.f) && (barInImage.x() < static_cast<float>(cameraInfo.width)) &&
          (barInImage.y() >= 0.f) && (barInImage.y() < static_cast<float>(cameraInfo.height)))
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
    if (randomFloat() > lineRecognitionRate)
      continue;
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
  Vector2f pImg;
  if (Transformation::robotToImage(line.startOnField, cameraMatrix, cameraInfo, pImg))
  {
    line.startInImage = pImg.cast<int>();
    if (Transformation::robotToImage(line.endOnField, cameraMatrix, cameraInfo, pImg))
    {
      line.endInImage = pImg.cast<int>();
      bool success = true;
      if (applyLineNoise)
      {
        applyNoise(linePosInImageStdDev, line.startInImage);
        applyNoise(linePosInImageStdDev, line.endInImage);
        Vector2i pField;
        success = Transformation::imageToRobot(line.startInImage.x(), line.startInImage.y(), cameraMatrix, cameraInfo, pField) &&
          Transformation::imageToRobot(line.endInImage.x(), line.endInImage.y(), cameraMatrix, cameraInfo, pField);
      }
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
    if (relativeMarkPos.norm() > penaltyMarkMaxVisibleDistance)
      continue;
    if (randomFloat() > penaltyMarkRecognitionRate)
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
        applyNoise(penaltyMarkPosInImageStdDev, penaltyMarkInImage);
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
  if (!theCameraMatrix.isValid || !Global::settingsExist())
    return;

  // Simulation scene should only use blue and red for now
  ASSERT(Global::getSettings().teamColor == Settings::blue || Global::getSettings().teamColor == Settings::red);

  const bool isBlue = Global::getSettings().teamColor == Settings::blue;
  for (unsigned int i = 0; i < theGroundTruthWorldState.bluePlayers.size(); ++i)
    createPlayerBox(theGroundTruthWorldState.bluePlayers[i], !isBlue, playersPercept, false);
  for (unsigned int i = 0; i < theGroundTruthWorldState.redPlayers.size(); ++i)
    createPlayerBox(theGroundTruthWorldState.redPlayers[i], isBlue, playersPercept, false);
}

void OracledPerceptsProvider::update(RobotsPerceptUpper& playersPercept)
{
  playersPercept.robots.clear();
  if (!theCameraMatrixUpper.isValid || !Global::settingsExist())
    return;

  // Simulation scene should only use blue and red for now
  ASSERT(Global::getSettings().teamColor == Settings::blue || Global::getSettings().teamColor == Settings::red);

  const bool isBlue = Global::getSettings().teamColor == Settings::blue;
  for (unsigned int i = 0; i < theGroundTruthWorldState.bluePlayers.size(); ++i)
    createPlayerBox(theGroundTruthWorldState.bluePlayers[i], !isBlue, (RobotsPercept&)playersPercept, true);
  for (unsigned int i = 0; i < theGroundTruthWorldState.redPlayers.size(); ++i)
    createPlayerBox(theGroundTruthWorldState.redPlayers[i], isBlue, (RobotsPercept&)playersPercept, true);
}

void OracledPerceptsProvider::update(CLIPCenterCirclePercept& centerCirclePercept)
{
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  // Find center circle (at least one out of five center circle points must be inside the current image)
  bool pointFound = false;
  bool upper = false;
  centerCirclePercept.centerCircleWasSeen = false;
  if ((theGroundTruthWorldState.ownPose.translation.norm() <= centerCircleMaxVisibleDistance) &&
    (randomFloat() < centerCircleRecognitionRate))
  {
    for (unsigned int i = 0; i < ccPoints.size(); ++i)
    {
      const Vector2f relPos = robotPoseInv * ccPoints[i];
      Vector2f posInImage;
      if (pointIsInImage(relPos, posInImage, upper))
      {
        pointFound = true;
        break;
      }
    }
  }
  if (pointFound)
  {
    const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
    const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
    centerCirclePercept.centerCircle.locationOnField = (robotPoseInv * Vector2f::Zero()).cast<int>();
    // Add some noise:
    centerCirclePercept.centerCircleWasSeen = true;
    centerCirclePercept.fromUpper = upper;

    if (applyCenterCircleNoise)
    {
      Vector2f nPImg;
      if (Transformation::robotToImage(Vector2f(centerCirclePercept.centerCircle.locationOnField.cast<float>()), cameraMatrix, cameraInfo, nPImg))
      {
        applyNoise(centerCircleCenterInImageStdDev, nPImg);
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

void OracledPerceptsProvider::createPlayerBox(const GroundTruthWorldState::GroundTruthPlayer& player, bool isOpponent, RobotsPercept& playersPercept, const bool &upper)
{
  const Pose2f robotPoseInv = theGroundTruthWorldState.ownPose.inverse();
  Vector2f relativePlayerPos = robotPoseInv * player.pose.translation;
  if (relativePlayerPos.norm() > playerMaxVisibleDistance)
    return;
  if (randomFloat() > playerRecognitionRate)
    return;
  Vector2f playerInImage;
  bool localUpper = upper;
  if (pointIsInImage(relativePlayerPos, playerInImage, localUpper) && upper == localUpper)
  {
    const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
    const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
    bool success = true;
    if (applyPlayerNoise)
    {  
      applyNoise(playerPosInImageStdDev, playerInImage);
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
      re.robotType = isOpponent ? RobotEstimate::RobotType::opponentRobot : RobotEstimate::RobotType::teammateRobot;
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

bool OracledPerceptsProvider::pointIsInImage(const Vector2f& p, Vector2f& pImg, bool &upper) const
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

  const float a1 = cameraMatrix.translation.x(),
    a2 = cameraMatrix.translation.y(),
    a3 = cameraMatrix.translation.z();
  float b1 = vectorToCenterWorld.x(),
    b2 = vectorToCenterWorld.y(),
    b3 = vectorToCenterWorld.z(),
    f = a3 / b3;
  Vector2f pof = Vector2f(a1 - f * b1, a2 - f * b2);

  if (f > 0.f)
    vP[0] = theGroundTruthWorldState.ownPose.translation;
  else
    vP[0] = (theGroundTruthWorldState.ownPose + pof).translation;

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
    vP[1] = (theGroundTruthWorldState.ownPose + pof).translation;

  r = cameraMatrix.rotation;
  r.rotateY(-(cameraInfo.openingAngleHeight / 2));
  r.rotateZ(-(cameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x();
  b2 = vectorToCenterWorld.y();
  b3 = vectorToCenterWorld.z();
  f = a3 / b3;
  pof = Vector2f(a1 - f * b1, a2 - f * b2);

  const float maxDist = std::sqrt(4.f * theFieldDimensions.xPosOpponentFieldBorder * theFieldDimensions.xPosOpponentFieldBorder +
    4.f * theFieldDimensions.yPosLeftFieldBorder * theFieldDimensions.yPosLeftFieldBorder);
  if (f > 0.f)
    vP[2] = theGroundTruthWorldState.ownPose.translation + Vector2f(maxDist, 0).rotate(theGroundTruthWorldState.ownPose.rotation + (-cameraInfo.openingAngleWidth / 2) + cameraMatrix.rotation.getZAngle());
  else
    vP[2] = (theGroundTruthWorldState.ownPose + pof).translation;

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
    vP[3] = theGroundTruthWorldState.ownPose.translation + Vector2f(maxDist, 0).rotate(theGroundTruthWorldState.ownPose.rotation + (cameraInfo.openingAngleWidth / 2) + cameraMatrix.rotation.getZAngle());
  else
    vP[3] = (theGroundTruthWorldState.ownPose + pof).translation;
}

bool OracledPerceptsProvider::partOfLineIsVisible(const std::pair<Vector2f, Vector2f>& line, Vector2f& start, Vector2f& end, bool upper) const
{
  const Vector2f* vP = upper ? viewPolygonUpper : viewPolygon;
  // First case: both points are inside:
  if (Geometry::isPointInsideConvexPolygon(vP, 4, line.first) &&
    Geometry::isPointInsideConvexPolygon(vP, 4, line.second))
  {
    start = line.first;
    end = line.second;
    return true;
  }
  // Second case: start is inside but end is outside
  if (Geometry::isPointInsideConvexPolygon(vP, 4, line.first) &&
    !Geometry::isPointInsideConvexPolygon(vP, 4, line.second))
  {
    start = line.first;
    for (int i = 0; i < 4; i++)
    {
      if (Geometry::checkIntersectionOfLines(line.first, line.second, vP[i], vP[(i + 1) % 4]))
      {
        if (Geometry::getIntersectionOfLines(Geometry::Line(line.first, line.second - line.first),
          Geometry::Line(vP[i], vP[(i + 1) % 4] - vP[i]), end))
        {
          return true;
        }
      }
    }
    return false; // should not happen ...
  }
  // Third case: end is inside but start is outside
  if (!Geometry::isPointInsideConvexPolygon(vP, 4, line.first) &&
    Geometry::isPointInsideConvexPolygon(vP, 4, line.second))
  {
    start = line.second;
    for (int i = 0; i < 4; i++)
    {
      if (Geometry::checkIntersectionOfLines(line.first, line.second, vP[i], vP[(i + 1) % 4]))
      {
        if (Geometry::getIntersectionOfLines(Geometry::Line(line.first, line.second - line.first),
          Geometry::Line(vP[i], vP[(i + 1) % 4] - vP[i]), end))
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
      if (Geometry::getIntersectionOfLines(Geometry::Line(line.first, line.second - line.first),
        Geometry::Line(vP[i], vP[(i + 1) % 4] - vP[i]), point))
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

MAKE_MODULE(OracledPerceptsProvider, cognitionInfrastructure)
