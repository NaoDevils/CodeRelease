/**
* @file PSKickDetector.cpp
*/

#include "PSKickDetector.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#include "Tools/Math/Random.h"

PSKickDetector::PSKickDetector() {}

PSKickDetector::~PSKickDetector() {}

void PSKickDetector::update(PSGoalieTrigger& trigger)
{
  trigger.numOfNonFieldColorPixels = -1;
  trigger.hasKicked = false;

  MODIFY("module:PSKickDetector:filterWindowSize", filterWindowSize);

  DECLARE_DEBUG_DRAWING("module:PSKickDetector:Upper:ScanLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PSKickDetector:Upper:AveragePosition", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PSKickDetector:Upper:PenaltyMark", "drawingOnImage");

  // only run this module in penalty shootout
  if (Global::getSettings().gameMode != Settings::penaltyShootout || theBehaviorData.role != BehaviorData::keeper || theGameInfo.state != STATE_PLAYING || theRobotInfo.penalty != PENALTY_NONE)
    return;

  // ##################
  // ##### Using the BallModel is not suggested for the moment
  // ## Reacting too slow to trigger dive motion

  // decide from BallSymbols only if possible
  if (theBallModel.estimate.velocity.x() < -velocityThreshold || theBallModel.lastPerception.x() < theBallSymbols.ballPositionRelative.x() - ballPerceptDifference)
  {
    trigger.hasKicked = true;
    if (std::abs(theBallModel.lastPerception.y()) < blockArea)
      trigger.kickDirection = PSGoalieTrigger::KickDirection::CENTER;
    else if (theBallModel.lastPerception.y() > 0.f)
      trigger.kickDirection = PSGoalieTrigger::KickDirection::LEFT;
    else
      trigger.kickDirection = PSGoalieTrigger::KickDirection::RIGHT;
  }
  else
  {
    trigger.hasKicked = false;
  }

  if (!trigger.hasKicked)
  {
    // ################
    // ### primitive fieldcolor test (reacts faster, maybe not that robust yet)

    // only upper image for now
    bool upper = true;
    const Image& image = upper ? (Image&)theImageUpper : (Image&)theImage;
    const FieldColors& fieldColor = upper ? (FieldColors&)theFieldColorsUpper : theFieldColors;
    const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : (CameraMatrix&)theCameraMatrix;
    const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

    // find scanLine y position, also find x position (do it with ballModel, its safe in slow)
    // based on robotPose
    Vector2f pPenaltyMark = Vector2f::Zero();
    Vector2f pHighestScanLine = Vector2f::Zero();

    if (theBallSymbols.timeSinceLastSeen < 3000 && Transformation::robotToImage(theBallSymbols.ballPositionRelative, cameraMatrix, cameraInfo, pPenaltyMark))
    {
      CROSS("module:PSKickDetector:Upper:PenaltyMark", pPenaltyMark.x(), pPenaltyMark.y(), 4, 2, Drawings::solidPen, ColorRGBA::green);
      pHighestScanLine = Vector2f(pPenaltyMark.x(), pPenaltyMark.y() + 10);
    }
    else if (Transformation::robotToImage(Vector2f(theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyMark, 0.f), cameraMatrix, cameraInfo, pPenaltyMark))
    {
      CROSS("module:PSKickDetector:Upper:PenaltyMark", pPenaltyMark.x(), pPenaltyMark.y(), 4, 2, Drawings::solidPen, ColorRGBA::orange);
      pHighestScanLine = Vector2f(pPenaltyMark.x(), pPenaltyMark.y() + 5);

      int numOfMarkPixels = 0;
      for (int y = (int)pPenaltyMark.y() + 50; y < 0; y--)
      {
        numOfMarkPixels = 0;
        for (int x = (int)pPenaltyMark.x() - 20; x <= (int)pPenaltyMark.x() + 20; x++)
        {
          bool hasFieldColor = false;
          for (int i = -(filterWindowSize * 2); i <= filterWindowSize * 2; i++)
          {
            // clip to image range
            const int xiPos = std::max(0, std::min(x + i, image.width - 1));
            const int yPos = std::max(0, std::min(y, image.height - 1));
#ifdef USE_FULL_RESOLUTION
            Image::YUVPixel p;
            image.getPixel(xiPos, yPos, &p);
#else
            Image::Pixel p = image[yPos][xiPos];
#endif
            hasFieldColor |= (fieldColor.isPixelFieldColor(p.y, p.cb, p.cr) || p.y < fieldColor.fieldColorArray[0].fieldColorOptY);
          }
          if (!hasFieldColor)
            numOfMarkPixels++;
        }
        if (numOfMarkPixels > 0)
        {
          pHighestScanLine.y() = y + 5.f;
          break;
        }
      }
    }
    else
    { // nothing in cam, stop calculating
      return;
    }

    // scan green in front of penaltyMark
    // can be considered free of obstacles by rule
    int halfWidthOfTrapezTop = image.width / 6;
    int scanLineNum = 0;
    int maxNumberOfScanLines = 75;
    int numOfNonFCPixels = 0;
    int xSum = 0;
    int ySum = 0;
    for (int y = (int)pHighestScanLine.y(); y < cameraInfo.height && y > 0 && scanLineNum < maxNumberOfScanLines; y++)
    {
      for (int x = (int)pHighestScanLine.x() - halfWidthOfTrapezTop; x <= (int)pHighestScanLine.x() + halfWidthOfTrapezTop; x++)
      {
        bool hasFieldColor = false;
        for (int i = -filterWindowSize; i <= filterWindowSize; i++)
          for (int j = -filterWindowSize; j <= filterWindowSize; j++)
          {
            // clip to image range
            const int xiPos = std::max(0, std::min(x + i, image.width - 1));
            const int yjPos = std::max(0, std::min(y + j, image.height - 1));
#ifdef USE_FULL_RESOLUTION
            Image::YUVPixel p;
            image.getPixel(xiPos, yjPos, &p);
#else
            Image::Pixel p = image[yjPos][xiPos];
#endif
            hasFieldColor |= (fieldColor.isPixelFieldColor(p.y, p.cb, p.cr) || p.y < fieldColor.fieldColorArray[0].fieldColorOptY);
          }
        if (hasFieldColor)
        {
          LINE("module:PSKickDetector:Upper:ScanLines", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::black);
        }
        else
        {
          LINE("module:PSKickDetector:Upper:ScanLines", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::red);
          xSum += x;
          ySum += y;
          numOfNonFCPixels++;
        }
      }
      scanLineNum++;
    }

    trigger.numOfNonFieldColorPixels = numOfNonFCPixels;

    // estimate position of the ball
    Vector2f averageLocation = Vector2f((float)xSum / numOfNonFCPixels, (float)ySum / numOfNonFCPixels);
    CROSS("module:PSKickDetector:Upper:AveragePosition", averageLocation.x(), averageLocation.y(), 5, 2, Drawings::solidPen, ColorRGBA::blue);

    // transform estimated position of ball to robot coordinates
    Vector2f averagePos = Vector2f::Zero();

    if (numOfNonFCPixels > nonFCPixelThreshold)
    {
      trigger.hasKicked = true;
      if (averagePos.y() > 0)
        trigger.kickDirection = PSGoalieTrigger::KickDirection::LEFT;
      else
        trigger.kickDirection = PSGoalieTrigger::KickDirection::RIGHT;
    }
    else
    {
      trigger.hasKicked = false;
    }
  }
}

MAKE_MODULE(PSKickDetector, perception)
