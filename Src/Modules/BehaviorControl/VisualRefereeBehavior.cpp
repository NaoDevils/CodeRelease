/**
 * @file VisualRefereeBehavior.cpp
 * 
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#include "VisualRefereeBehavior.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/Annotation.h"

MAKE_MODULE(VisualRefereeBehavior, perception);

void VisualRefereeBehavior::update(VisualRefereeBehaviorSymbols& vrbs)
{

  // transitions
  switch (vrbs.state)
  {
  case VisualRefereeBehaviorSymbols::State::idle:
    if (theGameInfo.inPreGame() && theRobotInfo.transitionToFramework == 1.f && theFrameInfo.getTimeSince(theKeySymbols.lastTimeNotPressed[KeyStates::Key::headFront]) > 200
        && theFrameInfo.getTimeSince(theKeySymbols.lastTimeNotPressed[KeyStates::Key::headRear]) > 200)
    {
      vrbs.state = VisualRefereeBehaviorSymbols::State::localize;
      vrbs.timestampStarted = theFrameInfo.time;
    }
    break;
  case VisualRefereeBehaviorSymbols::State::localize:
    if ((theFrameInfo.getTimeSince(vrbs.timestampStarted) > static_cast<int>(localizeTime) && theRobotPoseAfterPreview.validity > minPoseValidity)
        || theFrameInfo.getTimeSince(vrbs.timestampStarted) > static_cast<int>(localizeTimeout))
    {
      vrbs.state = VisualRefereeBehaviorSymbols::State::look;
      vrbs.timestampStarted = theFrameInfo.time;
    }
    break;
  case VisualRefereeBehaviorSymbols::State::look:
    if (theFrameInfo.getTimeSince(vrbs.timestampStarted) > static_cast<int>(lookTime) && theGameInfo.state == STATE_STANDBY)
    {
      vrbs.state = VisualRefereeBehaviorSymbols::State::capture;
    }
    break;
  case VisualRefereeBehaviorSymbols::State::capture:
    break;
  }

  if (!theGameInfo.inPreGame() || theRobotInfo.transitionToFramework < 1.f)
    vrbs.state = VisualRefereeBehaviorSymbols::State::idle;

  // go back to localize when being moved
  if (vrbs.state != VisualRefereeBehaviorSymbols::State::idle && theFallDownState.state == FallDownState::State::flying)
  {
    vrbs.state = VisualRefereeBehaviorSymbols::State::localize;
    vrbs.timestampStarted = theFrameInfo.time;
  }

  if (!lastModuleConfig.has_value() && vrbs.state != VisualRefereeBehaviorSymbols::State::idle)
  {
    lastModuleConfig = ModuleManager::sendModuleRequest({// YoloRobotDetector
        {"YoloInputUpper", "default"},
        {"YoloInput", "default"},
        {"RobotsHypothesesYolo", "default"},
        {"RobotsHypothesesYoloUpper", "default"},
        {"BallHypothesesYolo", "default"},
        {"PrePenaltyCrossHypothesesYolo", "default"},

        // CLIPBallPerceptor
        {"PreBallPercept", "default"},
        {"PreMultipleBallPercept", "default"},
        {"PreProcessedBallPatches", "default"},
        {"PreProcessedBallPatches", "default"},

        // WhistleDetector
        {"WhistleDortmund", "default"},

        // RobotClassifier
        {"RobotsPerceptClassified", "default"},
        {"ProcessedRobotsHypotheses", "default"},

        // Behavior Modules
        {"PositionInfo", "default"},
        {"DirectionInfo", "default"}});
  }
  else if (lastModuleConfig.has_value() && vrbs.state == VisualRefereeBehaviorSymbols::State::idle)
  {
    ModuleManager::sendModuleRequest(*lastModuleConfig);
    lastModuleConfig.reset();
  }

  // actions
  switch (vrbs.state)
  {
  case VisualRefereeBehaviorSymbols::State::idle:
  {
    vrbs.targetHeadAngle.setConstant(0_deg);
    vrbs.refereePositionInImage.fill(Vector2i::Zero());
  }
  break;
  case VisualRefereeBehaviorSymbols::State::localize:
  {
    lastRobotPose = theRobotPoseAfterPreview;

    const Pose2f idealPose = theOwnTeamInfo.onLeftSide ? Pose2f(90_deg, -750, theFieldDimensions.yPosRightSideline) : Pose2f(-90_deg, -750, theFieldDimensions.yPosLeftSideline);
    const Pose2f poseOffset = idealPose - theRobotPoseAfterPreview;

    // assume default position if localization is wrong
    if (poseOffset.translation.norm() > robotPoseTransThreshold || std::abs(poseOffset.rotation) > robotPoseRotThreshold)
      lastRobotPose = idealPose;
  }
    [[fallthrough]];
  case VisualRefereeBehaviorSymbols::State::look:
  case VisualRefereeBehaviorSymbols::State::capture:
  {
    const Vector2f refereeField(theFieldDimensions.xPosHalfWayLine, theOwnTeamInfo.onLeftSide ? theFieldDimensions.yPosLeftSideline : theFieldDimensions.yPosRightSideline);

    float refereeXOffsetLeft = theOwnTeamInfo.onLeftSide ? (+refereeSize.x() / 2.f) : (-refereeSize.x() / 2.f);
    float refereeXOffsetRight = theOwnTeamInfo.onLeftSide ? (-refereeSize.x() / 2.f) : (+refereeSize.x() / 2.f);

    Vector2f refereeTField, refereeTLField, refereeTRField, refereeBField, refereeBLField, refereeBRField;
    refereeTField << refereeField;
    refereeTLField << (refereeField.x() + refereeXOffsetLeft), refereeField.y();
    refereeTRField << (refereeField.x() + refereeXOffsetRight), refereeField.y();
    refereeBField << refereeField;
    refereeBLField << (refereeField.x() + refereeXOffsetLeft), refereeField.y();
    refereeBRField << (refereeField.x() + refereeXOffsetRight), refereeField.y();

    Vector3f refereeTRobot, refereeTLRobot, refereeTRRobot, refereeBRobot, refereeBLRobot, refereeBRRobot;
    refereeTRobot << Transformation::fieldToRobot(lastRobotPose, refereeTField), refereeSize.y();
    refereeTLRobot << Transformation::fieldToRobot(lastRobotPose, refereeTLField), refereeSize.y();
    refereeTRRobot << Transformation::fieldToRobot(lastRobotPose, refereeTRField), refereeSize.y();
    refereeBRobot << Transformation::fieldToRobot(lastRobotPose, refereeBField), 0.f;
    refereeBLRobot << Transformation::fieldToRobot(lastRobotPose, refereeBLField), 0.f;
    refereeBRRobot << Transformation::fieldToRobot(lastRobotPose, refereeBRField), 0.f;

    vrbs.targetHeadAngle = InverseKinematic::calcHeadJoints(theTorsoMatrix.inverse() * refereeTRobot, theRobotDimensions, false, theCameraCalibration, refereeTopInImage, 0_deg);

    Vector2f refereeTLImage, refereeTRImage, refereeBLImage, refereeBRImage;
    if (Transformation::robotToImage(refereeTLRobot, theCameraMatrixUpper, theCameraInfoUpper, refereeTLImage)
        && Transformation::robotToImage(refereeTRRobot, theCameraMatrixUpper, theCameraInfoUpper, refereeTRImage)
        && Transformation::robotToImage(refereeBLRobot, theCameraMatrixUpper, theCameraInfoUpper, refereeBLImage)
        && Transformation::robotToImage(refereeBRRobot, theCameraMatrixUpper, theCameraInfoUpper, refereeBRImage))
    {
      vrbs.refereePositionInImage = {refereeTLImage.cast<int>(), refereeTRImage.cast<int>(), refereeBRImage.cast<int>(), refereeBLImage.cast<int>()};
    }
    else
    {
      vrbs.refereePositionInImage.fill(Vector2i::Zero());
    }
  }
  break;
  }
}
