/**
 * @file CameraMatrix.cpp
 *
 * Implementation of struct CameraMatrix.
 */

#include "CameraMatrix.h"
#include "Tools/Boundary.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Module/Blackboard.h"

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Configuration/FieldDimensions.h"

CameraMatrix::CameraMatrix(const Pose3f& pose) : Pose3f(pose), isValid(true) {}

void CameraMatrix::drawFieldLines(bool upper) const
{
  // get copy representation here because RobotPose is provided after camera matrix
  const auto& theRobotPose = Blackboard::get<RobotPose>(true);
  const auto& theCameraInfo = Blackboard::get<CameraInfo>();
  const auto& theCameraInfoUpper = Blackboard::get<CameraInfoUpper>();
  const auto& theFieldDimensions = Blackboard::get<FieldDimensions>();

  const auto intersectLineWithCullPlane = [](const Vector3f& lineBase, const Vector3f& lineDir, Vector3f& point)
  {
    if (lineDir.x() == 0.)
      return false;
    point = lineBase + lineDir * ((lineBase.x() - 200.0f) / -lineDir.x());
    return true;
  };

  const auto camera2image = [&](const Vector3f& camera, Vector2f& image, bool upper)
  {
    const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
    const float& scale(cameraInfo.focalLength / camera.x());
    image.x() = float(cameraInfo.opticalCenter.x() - scale * camera.y());
    image.y() = float(cameraInfo.opticalCenter.y() - scale * camera.z());
  };

  Vector3f start0C, start1C, end0C, end1C;
  Vector2f start0I, start1I, end0I, end1I;

  const Pose2f& robotPoseInv(theRobotPose.inverse());
  const Pose3f& cameraMatrixInv(inverse());
  float halfFieldLinesWidth = theFieldDimensions.fieldLinesWidth / 2;

  for (unsigned int i = 0; i < theFieldDimensions.fieldLines.lines.size(); ++i)
  {
    const FieldDimensions::LinesTable::Line& line = theFieldDimensions.fieldLines.lines[i];
    Pose2f corner((line.to - line.from).angle(), line.from);
    float length = (line.to - line.from).norm();
    Pose2f relativeLine(robotPoseInv);
    relativeLine.conc(corner);
    const Vector2f start0(Pose2f(relativeLine).translate(0, -halfFieldLinesWidth).translation);
    const Vector2f end1(Pose2f(relativeLine).translate(length, halfFieldLinesWidth).translation);

    start0C = cameraMatrixInv * Vector3f(start0.x(), start0.y(), 0.); // field2camera
    end1C = cameraMatrixInv * Vector3f(end1.x(), end1.y(), 0.); // field2camera

    if (start0C.x() <= 200 && end1C.x() <= 200)
      continue;

    const Vector2f& start1(Pose2f(relativeLine).translate(0, halfFieldLinesWidth).translation);
    const Vector2f& end0(Pose2f(relativeLine).translate(length, -halfFieldLinesWidth).translation);

    start1C = cameraMatrixInv * Vector3f(start1.x(), start1.y(), 0.); // field2camera
    end0C = cameraMatrixInv * Vector3f(end0.x(), end0.y(), 0.); // field2camera

    if (start0C.x() <= 200)
      intersectLineWithCullPlane(start0C, end0C - start0C, start0C);
    else if (end0C.x() <= 200)
      intersectLineWithCullPlane(start0C, end0C - start0C, end0C);
    if (start1C.x() <= 200)
      intersectLineWithCullPlane(start1C, end1C - start1C, start1C);
    else if (end1C.x() <= 200)
      intersectLineWithCullPlane(start1C, end1C - start1C, end1C);

    camera2image(start0C, start0I, upper);
    camera2image(end0C, end0I, upper);
    camera2image(start1C, start1I, upper);
    camera2image(end1C, end1I, upper);

    if (upper)
    {
      LINE("representation:CameraMatrixUpper:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA(0, 0, 0));
      LINE("representation:CameraMatrixUpper:calibrationHelper", start1I.x(), start1I.y(), end1I.x(), end1I.y(), 0, Drawings::solidPen, ColorRGBA(0, 0, 0));
    }
    else
    {
      LINE("representation:CameraMatrix:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA(0, 0, 0));
      LINE("representation:CameraMatrix:calibrationHelper", start1I.x(), start1I.y(), end1I.x(), end1I.y(), 0, Drawings::solidPen, ColorRGBA(0, 0, 0));
    }
  }

  start0C = cameraMatrixInv * Vector3f(100, -11, 0.); // field2camera
  end0C = cameraMatrixInv * Vector3f(0, -11, 0.); // field2camera
  camera2image(start0C, start0I, upper);
  camera2image(end0C, end0I, upper);

  if (upper)
    LINE("representation:CameraMatrixUpper:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::blue);
  else
    LINE("representation:CameraMatrix:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::blue);

  start0C = cameraMatrixInv * Vector3f(100, 11, 0.); // field2camera
  end0C = cameraMatrixInv * Vector3f(0, 11, 0.); // field2camera
  camera2image(start0C, start0I, upper);
  camera2image(end0C, end0I, upper);

  if (upper)
    LINE("representation:CameraMatrixUpper:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::blue);
  else
    LINE("representation:CameraMatrix:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::blue);

  start0C = cameraMatrixInv * Vector3f(110, 1000, 0.); // field2camera
  end0C = cameraMatrixInv * Vector3f(110, -1000, 0.); // field2camera
  camera2image(start0C, start0I, upper);
  camera2image(end0C, end0I, upper);

  if (upper)
    LINE("representation:CameraMatrixUpper:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::blue);
  else
    LINE("representation:CameraMatrix:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::blue);
}

void CameraMatrix::drawFieldLines() const
{
  DEBUG_DRAWING("representation:CameraMatrix:calibrationHelper", "drawingOnImage") drawFieldLines(false);
}

void CameraMatrixUpper::drawFieldLines() const
{
  DEBUG_DRAWING("representation:CameraMatrixUpper:calibrationHelper", "drawingOnImage") CameraMatrix::drawFieldLines(true);
}

void CameraMatrix::draw() const
{
  drawFieldLines();

  DECLARE_DEBUG_DRAWING("representation:CameraMatrix:Image", "drawingOnImage"); // Shows the robot coordinate system
  DECLARE_DEBUG_DRAWING("representation:CameraMatrix:Field", "drawingOnField"); // Shows the robot coordinate system

  COMPLEX_DRAWING("representation:CameraMatrix:Field")
  {
    CameraInfo cameraInfo = Blackboard::get<CameraInfo>();
    Vector2i pointOnField[6];
    bool isValid[6];
    // calculate the projection of the four image corners to the ground
    isValid[0] = Transformation::imageToRobot(0, 0, *this, cameraInfo, pointOnField[0]);
    isValid[1] = Transformation::imageToRobot(cameraInfo.width, 0, *this, cameraInfo, pointOnField[1]);
    isValid[2] = Transformation::imageToRobot(cameraInfo.width, cameraInfo.height, *this, cameraInfo, pointOnField[2]);
    isValid[3] = Transformation::imageToRobot(0, cameraInfo.height, *this, cameraInfo, pointOnField[3]);

    // calculate a line 15 pixels below the horizon in the image
    const Geometry::Line horizon = Geometry::calculateHorizon(*this, cameraInfo);
    Geometry::Line lineBelowHorizon;
    const Vector2f vertLineDirection(-horizon.direction.y(), horizon.direction.x());
    lineBelowHorizon.direction = horizon.direction;
    lineBelowHorizon.base = horizon.base;
    lineBelowHorizon.base += vertLineDirection * 15.0f;

    // calculate the projection to the ground of the intersection points of the line parallel to the horizon and the image borders
    Vector2i beginPoint;
    Vector2i endPoint;
    if (Geometry::getIntersectionPointsOfLineAndRectangle(Vector2i::Zero(), Vector2i(cameraInfo.width - 1, cameraInfo.height - 1), lineBelowHorizon, beginPoint, endPoint))
    {
      isValid[4] = Transformation::imageToRobot(beginPoint.x(), beginPoint.y(), *this, cameraInfo, pointOnField[4]);
      isValid[5] = Transformation::imageToRobot(endPoint.x(), endPoint.y(), *this, cameraInfo, pointOnField[5]);
      if (isValid[4] && isValid[5])
        LINE("representation:CameraMatrix:Field", pointOnField[4].x(), pointOnField[4].y(), pointOnField[5].x(), pointOnField[5].y(), 30, Drawings::solidPen, ColorRGBA::yellow);
    }
    else
      isValid[4] = isValid[5] = false;

    // determine the boundary of all the points that were projected to the ground
    Boundaryi boundary(-10000, +10000);
    for (int i = 0; i < 6; ++i)
      if (isValid[i])
      {
        boundary.add(pointOnField[i]);
        const ColorRGBA& color = i < 4 ? ColorRGBA::white : ColorRGBA::yellow;
        CIRCLE("representation:CameraMatrix:Field", pointOnField[i].x(), pointOnField[i].y(), 100, 50, Drawings::solidPen, color, Drawings::noBrush, color);
      }

    LINE("representation:CameraMatrix:Field", boundary.x.min, boundary.y.min, boundary.x.max, boundary.y.min, 30, Drawings::solidPen, ColorRGBA::red);
    LINE("representation:CameraMatrix:Field", boundary.x.max, boundary.y.min, boundary.x.max, boundary.y.max, 30, Drawings::solidPen, ColorRGBA::yellow);
    LINE("representation:CameraMatrix:Field", boundary.x.max, boundary.y.max, boundary.x.min, boundary.y.max, 30, Drawings::solidPen, ColorRGBA::blue);
    LINE("representation:CameraMatrix:Field", boundary.x.min, boundary.y.max, boundary.x.min, boundary.y.min, 30, Drawings::solidPen, ColorRGBA::white);

    // fill the bounding rectangle with coordinate system lines (and reproject it to the image)
    const int spacing = 100;
    for (int xx = boundary.x.min - boundary.x.min % spacing + spacing; xx <= boundary.x.max; xx += spacing)
    {
      LINE("representation:CameraMatrix:Field", xx, boundary.y.min, xx, boundary.y.max, 5, Drawings::solidPen, ColorRGBA::white);
    }
    for (int yy = boundary.y.min - boundary.y.min % spacing + spacing; yy <= boundary.y.max; yy += spacing)
    {
      LINE("representation:CameraMatrix:Field", boundary.x.min, yy, boundary.x.max, yy, 5, Drawings::solidPen, ColorRGBA::white);
    }
  } // end complex drawing

  COMPLEX_DRAWING("representation:CameraMatrix:Image")
  {
    if (Blackboard::getInstance().exists("CameraInfo"))
    {
      const CameraInfo& cameraInfo = Blackboard::get<CameraInfo>();
      Vector2i pointOnField[6];
      bool isValid[6];
      // calculate the projection of the four image corners to the ground
      isValid[0] = Transformation::imageToRobot(0, 0, *this, cameraInfo, pointOnField[0]);
      isValid[1] = Transformation::imageToRobot(cameraInfo.width, 0, *this, cameraInfo, pointOnField[1]);
      isValid[2] = Transformation::imageToRobot(cameraInfo.width, cameraInfo.height, *this, cameraInfo, pointOnField[2]);
      isValid[3] = Transformation::imageToRobot(0, cameraInfo.height, *this, cameraInfo, pointOnField[3]);

      // calculate a line 15 pixels below the horizon in the image
      const Geometry::Line horizon = Geometry::calculateHorizon(*this, cameraInfo);
      Geometry::Line lineBelowHorizon;
      const Vector2f vertLineDirection(-horizon.direction.y(), horizon.direction.x());
      lineBelowHorizon.direction = horizon.direction;
      lineBelowHorizon.base = horizon.base;
      lineBelowHorizon.base += vertLineDirection * 15.0f;

      // calculate the projection to the ground of the intersection points of the line parallel to the horizon and the image borders
      Vector2f beginPoint = Vector2f::Zero();
      Vector2f endPoint = Vector2f::Zero();
      const Vector2f topRight = Vector2f(cameraInfo.width - 1.f, cameraInfo.height - 1.f);
      if (Geometry::getIntersectionPointsOfLineAndRectangle(Vector2f::Zero(), topRight, lineBelowHorizon, beginPoint, endPoint))
      {
        LINE("representation:CameraMatrix:Image", beginPoint.x(), beginPoint.y(), endPoint.x(), endPoint.y(), 3, Drawings::dashedPen, ColorRGBA::white);
        Vector2f pointOnFieldFloat = Vector2f::Zero();
        isValid[4] = Transformation::imageToRobot(beginPoint, *this, cameraInfo, pointOnFieldFloat);
        pointOnField[4].x() = static_cast<int>(std::floor(pointOnFieldFloat.x() + 0.5));
        pointOnField[4].y() = static_cast<int>(std::floor(pointOnFieldFloat.y() + 0.5));
        isValid[5] = Transformation::imageToRobot(endPoint, *this, cameraInfo, pointOnFieldFloat);
        pointOnField[5].x() = static_cast<int>(std::floor(pointOnFieldFloat.x() + 0.5));
        pointOnField[5].y() = static_cast<int>(std::floor(pointOnFieldFloat.y() + 0.5));
      }
      else
        isValid[4] = isValid[5] = false;

      // determine the boundary of all the points that were projected to the ground
      Boundaryi boundary(-10000, +10000);
      for (int i = 0; i < 6; ++i)
        if (isValid[i])
          boundary.add(pointOnField[i]);

      // fill the bounding rectangle with coordinate system lines (and reproject it to the image)
      int spacing = 100;
      for (int xx = boundary.x.min - boundary.x.min % spacing + spacing; xx <= boundary.x.max; xx += spacing)
        if (Transformation::robotToImage(Vector2f(static_cast<float>(xx), static_cast<float>(boundary.y.min)), *this, cameraInfo, beginPoint)
            && Transformation::robotToImage(Vector2f(static_cast<float>(xx), static_cast<float>(boundary.y.max)), *this, cameraInfo, endPoint))
          LINE("representation:CameraMatrix:Image", beginPoint.x(), beginPoint.y(), endPoint.x(), endPoint.y(), xx == 0 ? 3 : 0, Drawings::solidPen, ColorRGBA::white);
      for (int yy = boundary.y.min - boundary.y.min % spacing + spacing; yy <= boundary.y.max; yy += spacing)
        if (Transformation::robotToImage(Vector2f(static_cast<float>(boundary.x.min), static_cast<float>(yy)), *this, cameraInfo, beginPoint)
            && Transformation::robotToImage(Vector2f(static_cast<float>(boundary.x.max), static_cast<float>(yy)), *this, cameraInfo, endPoint))
          LINE("representation:CameraMatrix:Image", beginPoint.x(), beginPoint.y(), endPoint.x(), endPoint.y(), yy == 0 ? 3 : 0, Drawings::solidPen, ColorRGBA::white);
    }
  } // end complex drawing
}

void RobotCameraMatrix::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:RobotCameraMatrix:Image", "drawingOnImage"); // Shows the robot coordinate system
  DECLARE_DEBUG_DRAWING("representation:RobotCameraMatrix:Field", "drawingOnField"); // Shows the robot coordinate system

  COMPLEX_DRAWING("representation:RobotCameraMatrix:Field")
  {
    if (Blackboard::getInstance().exists("CameraInfo"))
    {
      const CameraInfo& cameraInfo = Blackboard::get<CameraInfo>();
      Vector2i pointOnField[6];
      bool isValid[6];
      // calculate the projection of the four image corners to the ground
      isValid[0] = Transformation::imageToRobot(0, 0, *this, cameraInfo, pointOnField[0]);
      isValid[1] = Transformation::imageToRobot(cameraInfo.width, 0, *this, cameraInfo, pointOnField[1]);
      isValid[2] = Transformation::imageToRobot(cameraInfo.width, cameraInfo.height, *this, cameraInfo, pointOnField[2]);
      isValid[3] = Transformation::imageToRobot(0, cameraInfo.height, *this, cameraInfo, pointOnField[3]);

      // calculate a line 15 pixels below the horizon in the image
      const Geometry::Line horizon = Geometry::calculateHorizon(*this, cameraInfo);
      Geometry::Line lineBelowHorizon;
      const Vector2f vertLineDirection(-horizon.direction.y(), horizon.direction.x());
      lineBelowHorizon.direction = horizon.direction;
      lineBelowHorizon.base = horizon.base;
      lineBelowHorizon.base += vertLineDirection * 15.f;

      // calculate the projection to the ground of the intersection points of the line parallel to the horizon and the image borders
      Vector2i beginPoint;
      Vector2i endPoint;
      if (Geometry::getIntersectionPointsOfLineAndRectangle(Vector2i::Zero(), Vector2i(cameraInfo.width - 1, cameraInfo.height - 1), lineBelowHorizon, beginPoint, endPoint))
      {
        isValid[4] = Transformation::imageToRobot(beginPoint.x(), beginPoint.y(), *this, cameraInfo, pointOnField[4]);
        isValid[5] = Transformation::imageToRobot(endPoint.x(), endPoint.y(), *this, cameraInfo, pointOnField[5]);
        if (isValid[4] && isValid[5])
          LINE("representation:CameraMatrix:Field", pointOnField[4].x(), pointOnField[4].y(), pointOnField[5].x(), pointOnField[5].y(), 30, Drawings::solidPen, ColorRGBA::yellow);
      }

      // determine the boundary of all the points that were projected to the ground
      Boundaryi boundary(-10000, +10000);
      for (int i = 0; i < 6; ++i)
        if (isValid[i])
        {
          boundary.add(pointOnField[i]);
          const ColorRGBA& color = i < 4 ? ColorRGBA::white : ColorRGBA::yellow;
          CIRCLE("representation:RobotCameraMatrix:Field", pointOnField[i].x(), pointOnField[i].y(), 100, 50, Drawings::solidPen, color, Drawings::noBrush, color);
        }

      LINE("representation:RobotCameraMatrix:Field", boundary.x.min, boundary.y.min, boundary.x.max, boundary.y.min, 30, Drawings::solidPen, ColorRGBA::red);
      LINE("representation:RobotCameraMatrix:Field", boundary.x.max, boundary.y.min, boundary.x.max, boundary.y.max, 30, Drawings::solidPen, ColorRGBA::yellow);
      LINE("representation:RobotCameraMatrix:Field", boundary.x.max, boundary.y.max, boundary.x.min, boundary.y.max, 30, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:RobotCameraMatrix:Field", boundary.x.min, boundary.y.max, boundary.x.min, boundary.y.min, 30, Drawings::solidPen, ColorRGBA::white);

      // fill the bounding rectangle with coordinate system lines (and reproject it to the image)
      const int spacing = 100;
      for (int xx = boundary.x.min - boundary.x.min % spacing + spacing; xx <= boundary.x.max; xx += spacing)
      {
        LINE("representation:RobotCameraMatrix:Field", xx, boundary.y.min, xx, boundary.y.max, 5, Drawings::solidPen, ColorRGBA::white);
      }
      for (int yy = boundary.y.min - boundary.y.min % spacing + spacing; yy <= boundary.y.max; yy += spacing)
      {
        LINE("representation:RobotCameraMatrix:Field", boundary.x.min, yy, boundary.x.max, yy, 5, Drawings::solidPen, ColorRGBA::white);
      }
    }
  } // end complex drawing

  COMPLEX_DRAWING("representation:RobotCameraMatrix:Image")
  {
    if (Blackboard::getInstance().exists("CameraInfo"))
    {
      const CameraInfo& cameraInfo = Blackboard::get<CameraInfo>();
      Vector2i pointOnField[6];
      bool isValid[6];
      // calculate the projection of the four image corners to the ground
      isValid[0] = Transformation::imageToRobot(0, 0, *this, cameraInfo, pointOnField[0]);
      isValid[1] = Transformation::imageToRobot(cameraInfo.width, 0, *this, cameraInfo, pointOnField[1]);
      isValid[2] = Transformation::imageToRobot(cameraInfo.width, cameraInfo.height, *this, cameraInfo, pointOnField[2]);
      isValid[3] = Transformation::imageToRobot(0, cameraInfo.height, *this, cameraInfo, pointOnField[3]);

      // calculate a line 15 pixels below the horizon in the image
      const Geometry::Line horizon = Geometry::calculateHorizon(*this, cameraInfo);
      Geometry::Line lineBelowHorizon;
      const Vector2f vertLineDirection(-horizon.direction.y(), horizon.direction.x());
      lineBelowHorizon.direction = horizon.direction;
      lineBelowHorizon.base = horizon.base;
      lineBelowHorizon.base += vertLineDirection * 15.f;

      // calculate the projection to the ground of the intersection points of the line parallel to the horizon and the image borders
      Vector2f beginPoint;
      Vector2f endPoint;
      if (Geometry::getIntersectionPointsOfLineAndRectangle(
              Vector2f::Zero(), Vector2f(static_cast<float>(cameraInfo.width - 1), static_cast<float>(cameraInfo.height - 1)), lineBelowHorizon, beginPoint, endPoint))
      {
        LINE("representation:RobotCameraMatrix:Image", beginPoint.x(), beginPoint.y(), endPoint.x(), endPoint.y(), 3, Drawings::dashedPen, ColorRGBA::white);
        Vector2f pointOnFieldFloat;
        isValid[4] = Transformation::imageToRobot(beginPoint, *this, cameraInfo, pointOnFieldFloat);
        pointOnField[4].x() = static_cast<int>(std::floor(pointOnFieldFloat.x() + 0.5));
        pointOnField[4].y() = static_cast<int>(std::floor(pointOnFieldFloat.y() + 0.5));
        isValid[5] = Transformation::imageToRobot(endPoint, *this, cameraInfo, pointOnFieldFloat);
        pointOnField[5].x() = static_cast<int>(std::floor(pointOnFieldFloat.x() + 0.5));
        pointOnField[5].y() = static_cast<int>(std::floor(pointOnFieldFloat.y() + 0.5));
      }
      else
        isValid[4] = isValid[5] = false;

      // determine the boundary of all the points that were projected to the ground
      Boundaryi boundary(-10000, +10000);
      for (int i = 0; i < 6; ++i)
        if (isValid[i])
          boundary.add(pointOnField[i]);

      // fill the bounding rectangle with coordinate system lines (and reproject it to the image)
      const int spacing = 100;
      for (int xx = boundary.x.min - boundary.x.min % spacing + spacing; xx <= boundary.x.max; xx += spacing)
        if (Transformation::robotToImage(Vector2f(static_cast<float>(xx), static_cast<float>(boundary.y.min)), *this, CameraInfo(), beginPoint)
            && Transformation::robotToImage(Vector2f(static_cast<float>(xx), static_cast<float>(boundary.y.max)), *this, CameraInfo(), endPoint))
          LINE("representation:RobotCameraMatrix:Image", beginPoint.x(), beginPoint.y(), endPoint.x(), endPoint.y(), xx == 0 ? 3 : 0, Drawings::solidPen, ColorRGBA::yellow);
      for (int yy = boundary.y.min - boundary.y.min % spacing + spacing; yy <= boundary.y.max; yy += spacing)
        if (Transformation::robotToImage(Vector2f(static_cast<float>(boundary.x.min), static_cast<float>(yy)), *this, CameraInfo(), beginPoint)
            && Transformation::robotToImage(Vector2f(static_cast<float>(boundary.x.max), static_cast<float>(yy)), *this, CameraInfo(), endPoint))
          LINE("representation:RobotCameraMatrix:Image", beginPoint.x(), beginPoint.y(), endPoint.x(), endPoint.y(), yy == 0 ? 3 : 0, Drawings::solidPen, ColorRGBA::yellow);
    }
  } // end complex drawing
}

RobotCameraMatrix::RobotCameraMatrix(const RobotDimensions& robotDimensions, float headYaw, float headPitch, const CameraCalibration& cameraCalibration, bool upperCamera)
{
  computeRobotCameraMatrix(robotDimensions, headYaw, headPitch, cameraCalibration, upperCamera);
}

void RobotCameraMatrix::computeRobotCameraMatrix(const RobotDimensions& robotDimensions, float headYaw, float headPitch, const CameraCalibration& cameraCalibration, bool upperCamera)
{
  *this = RobotCameraMatrix();

  translate(0., 0., robotDimensions.hipToNeckLength);
  rotateZ(headYaw);
  rotateY(headPitch);
  if (upperCamera)
  {
    translate(robotDimensions.xOffsetNeckToUpperCamera, 0.f, robotDimensions.zOffsetNeckToUpperCamera);
    rotateY(robotDimensions.tiltNeckToUpperCamera + cameraCalibration.upperCameraRotationCorrection.y());
    rotateX(cameraCalibration.upperCameraRotationCorrection.x());
    rotateZ(cameraCalibration.upperCameraRotationCorrection.z());
  }
  else
  {
    translate(robotDimensions.xOffsetNeckToLowerCamera, 0.f, robotDimensions.zOffsetNeckToLowerCamera);
    rotateY(robotDimensions.tiltNeckToLowerCamera + cameraCalibration.lowerCameraRotationCorrection.y());
    rotateX(cameraCalibration.lowerCameraRotationCorrection.x());
    rotateZ(cameraCalibration.lowerCameraRotationCorrection.z());
  }
}

Pose3f RobotCameraMatrix::getNeckCameraMatrix(const RobotDimensions& robotDimensions, const CameraCalibration& cameraCalibration, bool upperCamera)
{
  Pose3f ret;
  if (upperCamera)
  {
    ret.translate(robotDimensions.xOffsetNeckToUpperCamera, 0.f, robotDimensions.zOffsetNeckToUpperCamera);
    ret.rotateY(robotDimensions.tiltNeckToUpperCamera + cameraCalibration.upperCameraRotationCorrection.y());
    ret.rotateX(cameraCalibration.upperCameraRotationCorrection.x());
    ret.rotateZ(cameraCalibration.upperCameraRotationCorrection.z());
  }
  else
  {
    ret.translate(robotDimensions.xOffsetNeckToLowerCamera, 0.f, robotDimensions.zOffsetNeckToLowerCamera);
    ret.rotateY(robotDimensions.tiltNeckToLowerCamera + cameraCalibration.lowerCameraRotationCorrection.y());
    ret.rotateX(cameraCalibration.lowerCameraRotationCorrection.x());
    ret.rotateZ(cameraCalibration.lowerCameraRotationCorrection.z());
  }
  return ret;
}


CameraMatrix::CameraMatrix(const Pose3f& torsoMatrix, const Pose3f& robotCameraMatrix, const CameraCalibration& cameraCalibration)
{
  computeCameraMatrix(torsoMatrix, robotCameraMatrix, cameraCalibration);
}

void CameraMatrix::computeCameraMatrix(const Pose3f& torsoMatrix, const Pose3f& robotCameraMatrix, const CameraCalibration& cameraCalibration)
{
  static_cast<Pose3f&>(*this) = torsoMatrix;
  rotateY(cameraCalibration.bodyRotationCorrection.y());
  rotateX(cameraCalibration.bodyRotationCorrection.x());
  conc(robotCameraMatrix);
}

void CameraMatrix::computeCameraMatrix(const Pose3f& torsoMatrix, const Pose3f& torsoNeckMatrix, const Pose3f& neckCameraMatrix, float headYaw, float headPitch)
{
  static_cast<Pose3f&>(*this) = torsoMatrix;
  conc(torsoNeckMatrix);
  rotateZ(headYaw);
  rotateY(headPitch);
  conc(neckCameraMatrix);
}

Pose3f CameraMatrix::getTorsoNeckMatrix(const RobotDimensions& robotDimensions, const CameraCalibration& cameraCalibration)
{
  Pose3f ret;
  ret.rotateY(cameraCalibration.bodyRotationCorrection.y());
  ret.rotateX(cameraCalibration.bodyRotationCorrection.x());
  ret.translate(0., 0., robotDimensions.hipToNeckLength);
  return ret;
}
