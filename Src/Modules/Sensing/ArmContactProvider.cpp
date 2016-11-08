/**
 * @file ArmContactProvider.h
 * Provider for arm contact.
 * @author <a href="mailto:dino.menges@tu-dortmund.de> Dino Menges</a>
 */

#include "ArmContactProvider.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/DebugDrawings.h"

ArmContactProvider::ArmContactProvider()
{
  localArmContact.armContactStateLeft = ArmContact::None;
  localArmContact.armContactStateRight = ArmContact::None;
  localArmContact.timeStampLeft = 0;
  localArmContact.timeStampRight = 0;
  bufferLeftBack.clear();
  bufferLeftFront.clear();
  bufferRightBack.clear();
  bufferRightFront.clear();
  directionLeft = 0;
  directionRight = 0;
  bufferDirectionLeft.clear();
  bufferDirectionRight.clear();
  lastPitchLeft = lastPitchRight = 0;
  lastRequest.timestamp = 0;
}

ArmContactProvider::~ArmContactProvider()
{
}

void ArmContactProvider::update(ArmContact& armContact)
{
  // Check if lastRequest has been set
  if (lastRequest.timestamp == 0)
    lastRequest = theJointRequest;
  
  // Only when in standard walk (not in kicking, not in special actions etc)
  if (enableAvoidArmContact && theMotionRequest.motion == MotionRequest::walk && theFallDownState.state == FallDownState::upright)
  {
    // fill buffer to check if arm wants to move forward/backward constantly
    bufferDirectionLeft.push_front((theJointRequest.angles[Joints::lShoulderPitch] - lastPitchLeft) > 0);
    bufferDirectionRight.push_front((theJointRequest.angles[Joints::rShoulderPitch] - lastPitchRight) > 0);

    int leftSum = bufferDirectionLeft.sum();
    int rightSum = bufferDirectionRight.sum();
    switch (leftSum)
    {
    case 4:
      directionLeft = 1;
      break;
    case -4:
      directionLeft = -1;
      break;
    default:
      directionLeft = 0;
    }

    switch (rightSum)
    {
    case 4:
      directionRight = 1;
      break;
    case -4:
      directionRight = -1;
      break;
    default:
      directionRight = 0;
    }

    // desired arm direction known -> check if something stops the desired movement
    float angleDiff = lastRequest.angles[Joints::lShoulderPitch]-theJointSensorData.angles[Joints::lShoulderPitch];
    switch (directionLeft)
    {
    case 1:
      if (minAngleDiff < angleDiff)
        bufferLeftFront.push_front(angleDiff);
      else
        bufferLeftFront.push_front(0);
      break;
    case -1:
      if (angleDiff < -minAngleDiff)
        bufferLeftBack.push_front(-angleDiff);
      else
        bufferLeftBack.push_front(0);
      break;
    default:
      bufferLeftFront.push_front(0);
      bufferLeftBack.push_front(0);
      break;
    }
    angleDiff = lastRequest.angles[Joints::rShoulderPitch]-theJointSensorData.angles[Joints::rShoulderPitch];
    switch (directionRight)
    {
    case 1:
      if (minAngleDiff < angleDiff)
        bufferRightFront.push_front(angleDiff);
      else
        bufferLeftFront.push_front(0);
      break;
    case -1:
      if (angleDiff < -minAngleDiff)
        bufferRightBack.push_front(-angleDiff);
      else
        bufferRightBack.push_front(0);
      break;
    default:
      bufferRightFront.push_front(0);
      bufferRightBack.push_front(0);
      break;
    }

    
    if (localArmContact.armContactStateLeft != ArmContact::None
      && (unsigned int)theFrameInfo.getTimeSince(localArmContact.timeStampLeft) > timeToHoldArmBack)
      localArmContact.armContactStateLeft = ArmContact::None;

    if (localArmContact.armContactStateRight != ArmContact::None
      && (unsigned int)theFrameInfo.getTimeSince(localArmContact.timeStampRight) > timeToHoldArmBack)
      localArmContact.armContactStateRight = ArmContact::None;

    bool obstacleLeft = false;
    bool obstacleRight = false;
    
    // avoid contact with goal posts
    Vector2f goalPostRight(theFieldDimensions.xPosOpponentGoalPost,theFieldDimensions.yPosRightGoal);
    Vector2f goalPostLeft(theFieldDimensions.xPosOpponentGoalPost,theFieldDimensions.yPosLeftGoal);
    Vector2f goalPostRightRelative = Transformation::fieldToRobot(theRobotPose,goalPostRight);
    Vector2f goalPostLeftRelative = Transformation::fieldToRobot(theRobotPose,goalPostLeft);
    if (goalPostRightRelative.norm() < 300 && goalPostRightRelative.angle() < 0)
      obstacleRight = true;
    if (goalPostLeftRelative.norm() < 300 && goalPostLeftRelative.angle() > 0)
      obstacleLeft = true;

    goalPostRight = Vector2f(theFieldDimensions.xPosOwnGoalPost,theFieldDimensions.yPosRightGoal);
    goalPostLeft = Vector2f(theFieldDimensions.xPosOwnGoalPost,theFieldDimensions.yPosLeftGoal);
    goalPostRightRelative = Transformation::fieldToRobot(theRobotPose,goalPostRight);
    goalPostLeftRelative = Transformation::fieldToRobot(theRobotPose,goalPostLeft);
    if (goalPostRightRelative.norm() < 300 && goalPostRightRelative.angle() < 0)
      obstacleRight = true;
    if (goalPostLeftRelative.norm() < 300 && goalPostLeftRelative.angle() > 0)
      obstacleLeft = true;

    // avoid contact via robot map
    
    if (useRobotMap)
    {
      for (auto &robot : theRobotMap.robots)
      {
        Vector2f robotRelative = Transformation::fieldToRobot(theRobotPose,robot.pose.translation);
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
      if (bufferLeftFront.sum() > maxSum)
      {
        if (localArmContact.armContactStateLeft != ArmContact::Front)
        {
          localArmContact.timeStampLeft = theFrameInfo.time;
          localArmContact.armContactStateLeft = ArmContact::Front;
        }
      }
      else if (bufferLeftBack.sum() > maxSum)
      {
        if (localArmContact.armContactStateLeft != ArmContact::Back)
        {
          localArmContact.timeStampLeft = theFrameInfo.time;
          localArmContact.armContactStateLeft = ArmContact::Back;
        }
      }

      if (bufferRightFront.sum() > maxSum)
      {
        if (localArmContact.armContactStateRight != ArmContact::Front)
        {
          localArmContact.timeStampRight = theFrameInfo.time;
          localArmContact.armContactStateRight = ArmContact::Front;
        }
      }
      else if (bufferRightBack.sum() > maxSum)
      {
        if (localArmContact.armContactStateRight != ArmContact::Back)
        {
          localArmContact.timeStampRight = theFrameInfo.time;
          localArmContact.armContactStateRight = ArmContact::Back;
        }
      }
    }
    if (obstacleLeft)
    {
      localArmContact.armContactStateLeft = ArmContact::Front;
      localArmContact.timeStampLeft = theFrameInfo.time;
    }
    if (obstacleRight)
    {
      localArmContact.armContactStateRight = ArmContact::Front;
      localArmContact.timeStampRight = theFrameInfo.time;
    }
  }
  else // No walk motion
  {
    localArmContact.armContactStateLeft = ArmContact::None;
    localArmContact.armContactStateRight = ArmContact::None;
    localArmContact.timeStampLeft = 0;
    localArmContact.timeStampRight = 0;

    bufferLeftFront.push_front(0);
    bufferLeftBack.push_front(0);
    bufferRightFront.push_front(0);
    bufferRightBack.push_front(0);
  }

  // if robot might fall down, put arm back to side
  float fallDownAngle = falling ? 0.25f : 0.35f;
  falling = (std::abs(theInertialSensorData.angle.y()) > fallDownAngle);
  
  if (falling)
  {
    localArmContact.armContactStateLeft = ArmContact::None;
    localArmContact.armContactStateRight = ArmContact::None;
  }

  PLOT("module:ArmContactProvider:bufferLF",bufferLeftFront.sum());
  PLOT("module:ArmContactProvider:bufferLB",bufferLeftBack.sum());
  PLOT("module:ArmContactProvider:bufferRF",bufferRightFront.sum());
  PLOT("module:ArmContactProvider:bufferRB",bufferRightBack.sum());

  lastPitchLeft = theJointRequest.angles[Joints::lShoulderPitch];
  lastPitchRight = theJointRequest.angles[Joints::rShoulderPitch];

  int leftState = localArmContact.armContactStateLeft;
  int rightState = localArmContact.armContactStateRight;
  MODIFY("module:ArmContactProvider:leftState", leftState);
  MODIFY("module:ArmContactProvider:rightState", rightState);
  if (leftState != localArmContact.armContactStateLeft)
  {
    localArmContact.armContactStateLeft = (ArmContact::ArmContactState)leftState;
    if (localArmContact.armContactStateLeft != ArmContact::None)
      localArmContact.timeStampLeft = theFrameInfo.time;
  }
  if (rightState != localArmContact.armContactStateRight)
  {
    localArmContact.armContactStateRight = (ArmContact::ArmContactState)rightState;
    if (localArmContact.armContactStateRight != ArmContact::None)
      localArmContact.timeStampRight = theFrameInfo.time;
  }
  
  lastRequest = theJointRequest;
	armContact = localArmContact;
}

MAKE_MODULE(ArmContactProvider, motionControl);
