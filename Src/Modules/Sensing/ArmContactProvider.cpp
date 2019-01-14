/**
 * @file ArmContactProvider.h
 * Provider for arm contact.
 * @author <a href="mailto:dino.menges@tu-dortmund.de> Dino Menges</a>
 */

#include "ArmContactProvider.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Settings.h"

ArmContactProvider::ArmContactProvider()
{
  localArmContact.armContactStateLeft = ArmContact::None;
  localArmContact.armContactStateRight = ArmContact::None;
  localArmContact.timeStampLeft = 0;
  localArmContact.timeStampRight = 0;
  resetBuffers();
  lastPitchLeft = lastPitchRight = 0;
  lastRequest.timestamp = 0;
  falling = false;
  timeWhenWalkStarted = 0;
  lastMotionType = MotionRequest::specialAction;
}

ArmContactProvider::~ArmContactProvider()
{
}

void ArmContactProvider::update(ArmContact& armContact)
{
  // Check if lastRequest has been set
  if (lastRequest.timestamp == 0)
    lastRequest = theJointRequest;

  // just check if it is in walk or not
  if (lastMotionType != MotionRequest::walk && theMotionSelection.ratios[MotionRequest::walk] == 1.f)
  {
    lastMotionType = MotionRequest::walk;
    timeWhenWalkStarted = theFrameInfo.time;
  }
  else if (theMotionSelection.ratios[MotionRequest::walk] != 1.f)
    lastMotionType = MotionRequest::specialAction; // whatever

  // Only when in standard walk (not in kicking, not in special actions etc)
  if (enableAvoidArmContact && (theGameInfo.state == STATE_READY || theGameInfo.state == STATE_PLAYING) &&
    lastMotionType == MotionRequest::walk &&
    theFrameInfo.getTimeSince(timeWhenWalkStarted) > 2000 && // a little time to get started here to not move arms immediately on walk start
    theFallDownState.state == FallDownState::upright &&
      Global::getSettings().gameMode != Settings::penaltyShootout) // no robots to contact in penalty shootout
  {
    localArmContact.armContactStateLeft = ArmContact::ArmContactState::None;
    localArmContact.armContactStateRight = ArmContact::ArmContactState::None;
    if (!theArmMovement.armsInContactAvoidance)
    {
      // desired arm direction known -> check if something stops the desired movement
      Angle angleDiff = lastRequest.angles[Joints::lShoulderPitch] - theJointSensorData.angles[Joints::lShoulderPitch];
      if (std::abs(angleDiff) > minAngleDiff)
        bufferLeft.push_front(angleDiff);
      else
        bufferLeft.push_front(0);

      angleDiff = lastRequest.angles[Joints::rShoulderPitch] - theJointSensorData.angles[Joints::rShoulderPitch];
      if (std::abs(angleDiff) > minAngleDiff)
        bufferRight.push_front(angleDiff);
      else
        bufferRight.push_front(0);
    }
    else
    {
      bufferLeft.push_front(0);
      bufferRight.push_front(0);
    }

    bool obstacleLeft = false;
    bool obstacleRight = false;

    // avoid contact with goal posts
    Vector2f goalPostRight(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
    Vector2f goalPostLeft(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
    Vector2f goalPostRightRelative = Transformation::fieldToRobot(theRobotPose, goalPostRight);
    Vector2f goalPostLeftRelative = Transformation::fieldToRobot(theRobotPose, goalPostLeft);
    if (goalPostRightRelative.norm() < 300 && goalPostRightRelative.angle() < 0)
      obstacleRight = true;
    if (goalPostLeftRelative.norm() < 300 && goalPostLeftRelative.angle() > 0)
      obstacleLeft = true;

    goalPostRight = Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal);
    goalPostLeft = Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal);
    goalPostRightRelative = Transformation::fieldToRobot(theRobotPose, goalPostRight);
    goalPostLeftRelative = Transformation::fieldToRobot(theRobotPose, goalPostLeft);
    if (goalPostRightRelative.norm() < 300 && goalPostRightRelative.angle() < 0)
      obstacleRight = true;
    if (goalPostLeftRelative.norm() < 300 && goalPostLeftRelative.angle() > 0)
      obstacleLeft = true;

    // avoid contact via robot map

    if (useRobotMap)
    {
      for (auto &robot : theRobotMap.robots)
      {
        Vector2f robotRelative = Transformation::fieldToRobot(theRobotPose, robot.pose.translation);
        float robotDist = robotRelative.norm();
        //float ballDist = theBallModelAfterPreview.estimate.position.norm();
        if (std::abs(toDegrees(robotRelative.angle())) < 90 &&
          robotDist < maxRobotDist)
        {
          obstacleLeft = obstacleLeft || robotRelative.angle() > 0;
          obstacleRight = obstacleRight || robotRelative.angle() < 0;
        }
      }
    }

    if (useArmPitchDiff)
    {
      // set arm contact state and time stamp of arm contact using pitch differences
      if (bufferLeft.sum() > maxSum)
        localArmContact.armContactStateLeft = ArmContact::Front;
      else if (bufferLeft.sum() < -maxSum)
        localArmContact.armContactStateLeft = ArmContact::Back;

      if (bufferRight.sum() > maxSum)
        localArmContact.armContactStateRight = ArmContact::Front;
      else if (bufferRight.sum() < -maxSum)
        localArmContact.armContactStateRight = ArmContact::Back;
    }
    if (obstacleLeft)
      localArmContact.armContactStateLeft = ArmContact::Front;
    if (obstacleRight)
      localArmContact.armContactStateRight = ArmContact::Front;
  }
  else // No walk motion
  {
    localArmContact.armContactStateLeft = ArmContact::None;
    localArmContact.armContactStateRight = ArmContact::None;
    localArmContact.timeStampLeft = 0;
    localArmContact.timeStampRight = 0;

    bufferLeft.push_front(0);
    bufferRight.push_front(0);
  }

  // if robot might fall down, put arm back to side
  float fallDownAngle = falling ? 0.25f : 0.35f;
  falling = (std::abs(theInertialSensorData.angle.y()) > fallDownAngle);

  if (falling)
  {
    localArmContact.armContactStateLeft = ArmContact::None;
    localArmContact.armContactStateRight = ArmContact::None;
  }
  else if (bothArmsBack)
  {
    if (localArmContact.armContactStateLeft == ArmContact::None && localArmContact.armContactStateRight != ArmContact::None) 
    {
      localArmContact.armContactStateLeft = localArmContact.armContactStateRight;
    }
    else if (localArmContact.armContactStateLeft != ArmContact::None && localArmContact.armContactStateRight == ArmContact::None)
    {
      localArmContact.armContactStateRight = localArmContact.armContactStateLeft;
    }
  }

  PLOT("module:ArmContactProvider:bufferLeft", bufferLeft.sum());
  PLOT("module:ArmContactProvider:bufferRight", bufferRight.sum());

  lastPitchLeft = theJointRequest.angles[Joints::lShoulderPitch];
  lastPitchRight = theJointRequest.angles[Joints::rShoulderPitch];

  ArmContact::ArmContactState leftState = armContact.armContactStateLeft;
  ArmContact::ArmContactState rightState = armContact.armContactStateRight;
  MODIFY("module:ArmContactProvider:leftState", leftState);
  MODIFY("module:ArmContactProvider:rightState", rightState);
  if (leftState != localArmContact.armContactStateLeft)
  {
    if (localArmContact.armContactStateLeft != ArmContact::None)
    {
      localArmContact.timeStampLeft = theFrameInfo.time;
      resetBuffers();
    }
  }
  if (rightState != localArmContact.armContactStateRight)
  {
    if (localArmContact.armContactStateRight != ArmContact::None)
    {
      localArmContact.timeStampRight = theFrameInfo.time;
      resetBuffers();
    }
  }

  lastRequest = theJointRequest;
  armContact = localArmContact;
}

void ArmContactProvider::resetBuffers()
{
  bufferLeft.clear();
  bufferRight.clear();
}

MAKE_MODULE(ArmContactProvider, motionControl);
