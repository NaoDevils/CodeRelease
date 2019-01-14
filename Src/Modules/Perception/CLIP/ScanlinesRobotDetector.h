/**
  * @file ScanlinesRobotDetector.h
  * This file declares a module that performs obstacle scans to find hypothesis for robots.
  * It provides the robot rectangles representation, that contains only the rectangles 
  * for the hypothesis. In this module no images are copied, it only creates the rectangles.
  * @original author <a href="mailto:fabian.rensen@tu-dortmund.de">Fabian Rensen</a>
  * @modified <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 **/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Tools/Math/Eigen.h"
#include "Representations/Perception/FieldColor.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/RobotParameters.h"
#include "Tools/Math/Transformation.h"

MODULE(ScanlinesRobotDetector,
{,
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(ObstacleBasePoints),
  REQUIRES(FieldColors),
  REQUIRES(FieldColorsUpper),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  PROVIDES(RobotsHypotheses),
  LOADS_PARAMETERS(
  {,
   (float) robotHeight,                       // robot height in mm
   (float) robotWidth,                        // robot width in mm
   (int) directionScanBufferLength,           // how many pixels are buffered in obstacleScans, with these the decision is made if an end was found
   (int) directionScanBufferMajority,         // how many values need to be object or field color to decide if an end was found
   (int) directionScanMaxDistForFurtherWhite, // how much farther scanning is performed to see if the obstacle continues?
   (float) robotRatio,                        // height / width of a robot
   (int) sideScanSteps,                       // How many Side Scans will be performed per obstacle point
   (int) downScanSteps,                       // How many Down Scans will be performed per obstacle point
   (float) allowedDeviationFromRealSize,      // Allowed deviation from real size, in percentage, e.g. 0.5 means that an object that appears to be 50cm wide is discarded if it is more than 50% larger in the image, ie 75cm and above (or 50% smaller)
   (float) overlappingRatio,                  // How much two rectangles have to overlap so that the smaller one gets removed
   (bool) fitHypotheses,                      // Wether to fit the hypotheses to desired ratio before resizing
 }),
});


namespace RobotDetector
{
  /**
   * @brief The SideScanResult represents a small datastructure, that is used as return type for side scans.
   */
  struct SideScanResult
  {
    /**
     * @brief startedOnGreen If the scan started on the obstacle or not
     */
    bool startedOnGreen;

    /**
     * @brief right Deprecated
     * @deprecated
     */
    bool right;
    /**
     * @brief jumpsToGreen Both possible jumps to green, not necessarily both filled
     */
    Vector2i jumpsToGreen[2];
    /**
     * @brief jumpToWhite The location of the jump to white (if started not on obstacle)
     */
    Vector2i jumpToWhite;
    /**
     * @brief obstacleEnd The end of the object
     */
    Vector2i obstacleEnd;

    /**
     * @brief SideScanResult Sets all location variables to (-1,-1)
     */
    SideScanResult()
    {
      jumpToWhite = {-1,-1};
      jumpsToGreen[0] = {-1,-1};
      jumpsToGreen[1] = {-1,-1};
      obstacleEnd = {-1,-1};
    }
  };
}

class ScanlinesRobotDetector : public ScanlinesRobotDetectorBase
{
public:
  ScanlinesRobotDetector();
  void update(RobotsHypotheses& theRobotsHypotheses);

private:

  enum SideScanState
  {
    scanForGreen,
    scanMaxDistForFurtherWhite,
    scanForSecondGreen,
    obstacleEndsAtFirstGreen,
    obstacleEndsAtSecondGreen,
    obstacleEndsAtImageBorder
  };

  enum RejectionReason
  {
    firstCameraMatrixCheck,
    secondCameraMatrixCheck,
    tooFewValidSideScans,
    tooMuchSizeDeviation,
    unknown,
    notRejected
  };

  const Image* image;
  RejectionReason rejectionReason;
  void execute(RobotsHypotheses& theRobotsHypotheses);
  bool checkCameraMatrixRobotSize(Vector2f& realRobotSize, const Vector2f& basePointInImage, const bool& upper);
  void fitHypothesesToWidthHeigtRatio(RobotsHypotheses& robotsHypotheses, float desiredRatio, const bool& upper);

  RobotDetector::SideScanResult scanToDirection(const Vector2i& start, Vector2f direction, const Vector2f& robotsize, const bool& upper);
  void drawRejectionReason(const ObstacleBasePoints::ObstacleBasePoint& base);
  void drawSideScan(const RobotDetector::SideScanResult& result, const Vector2i& start, const bool right, const bool upper);

  std::vector<RobotsHypotheses::RobotHypothesis> removeOverlappingRectangles(const std::vector<RobotsHypotheses::RobotHypothesis>& input);

  int overlap(RobotsHypotheses::RobotHypothesis r1, RobotsHypotheses::RobotHypothesis r2);
};

