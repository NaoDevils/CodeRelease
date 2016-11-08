#include "ArmAnimator.h"
#include "Tools/Settings.h"

ArmAnimator::ArmAnimator()
{
  lastContactStateLeft = ArmContact::None;
  lastContactStateRight = ArmContact::None;
}


ArmAnimator::~ArmAnimator()
{

}

void ArmAnimator::update(ArmMovement& armMovement)
{
  WalkingEngineParams curparams = (WalkingEngineParams &)theWalkingEngineParams;
  if (theWalkingInfo.kickPhase != freeLegNA)
	  curparams = theFreeLegPhaseParams;

  armMovement.angles[Joints::lShoulderPitch]=0;
  armMovement.angles[Joints::lShoulderRoll]=Angle::fromDegrees(curparams.arms1);
  armMovement.angles[Joints::lElbowRoll]=0;
  armMovement.angles[Joints::lElbowYaw]=pi_2;
  //BEMBLECUP Changed Wrist Direction from wristOffset to -wristOffset
  armMovement.angles[Joints::lWristYaw] = -wristOffset;
  armMovement.angles[Joints::lHand] = handOffset;

  armMovement.angles[Joints::rShoulderPitch]=0;
  armMovement.angles[Joints::rShoulderRoll]= Angle::fromDegrees(-curparams.arms1);
  armMovement.angles[Joints::rElbowRoll]=0;
  armMovement.angles[Joints::rElbowYaw]=-pi_2;
  armMovement.angles[Joints::rWristYaw] = wristOffset;
  armMovement.angles[Joints::rHand] = handOffset;


  
float xOffset=(theKinematicRequest.leftFoot[0]+theKinematicRequest.rightFoot[0])/2;
  float leftArm,rightArm;

  leftArm= Angle::fromDegrees(90) + curparams.armFactor*(theKinematicRequest.leftFoot[0]-xOffset);
  rightArm= Angle::fromDegrees(90) + curparams.armFactor*(theKinematicRequest.rightFoot[0]-xOffset);
  leftArm=leftArm<0?0:(leftArm>pi?pi:leftArm);
  rightArm=rightArm<0?0:(rightArm>pi?pi:rightArm);

  armMovement.angles[Joints::lShoulderPitch] = leftArm - theBodyTilt.y;
  armMovement.angles[Joints::rShoulderPitch] = rightArm - theBodyTilt.y;

  armMovement.usearms=true;
  armMovement.armsInContactAvoidance = false;

  // arm contact stuff
  {
    // Compute times
    const unsigned time4_5 = timeToPullPitch - timeToPullPitch/5;
    const unsigned timeToMoveArm = timeToPullPitch + timeToPullArmIn;

    if (lastContactStateLeft != theArmContact.armContactStateLeft)
    {
      leftPitchWhenContactStateChanged = theJointSensorData.angles[Joints::lShoulderPitch];
      leftTimeWhenContactStateChanged = theArmContact.timeStampLeft;
      lastContactStateLeft = theArmContact.armContactStateLeft;
    }
    if (lastContactStateRight != theArmContact.armContactStateRight)
    {
      rightPitchWhenContactStateChanged = theJointSensorData.angles[Joints::rShoulderPitch];
      rightTimeWhenContactStateChanged = theArmContact.timeStampRight;
      lastContactStateRight = theArmContact.armContactStateRight;
    }

    // Left Arm
    unsigned timeSinceContact = theFrameInfo.getTimeSince(leftTimeWhenContactStateChanged);
    {
      // back to original state
      if (timeSinceContact < timeToMoveArm && theArmContact.armContactStateLeft == ArmContact::None)
      {
        float alpha = std::min((float)(timeSinceContact-timeToHoldArmBack-timeToMoveArm)/timeToPullArmIn,1.0f);
        armMovement.angles[Joints::lElbowRoll] = (1-alpha)*armBackElbowRoll;
        armMovement.angles[Joints::lShoulderRoll] = Angle::normalize((alpha)*Angle::fromDegrees(curparams.arms1) + (1-alpha) * -0.3f);
        alpha = (timeSinceContact > timeToHoldArmBack+timeToMoveArm+timeToPullArmIn) ?
          (float)(timeSinceContact-(timeToHoldArmBack+timeToMoveArm+timeToPullArmIn))/timeToPullPitch : 0;
        alpha = std::min(alpha,1.f);
        armMovement.angles[Joints::lShoulderPitch] = Angle::normalize((alpha)*leftPitchWhenContactStateChanged
          + (1-alpha)*armBackPitch);
        armMovement.armsInContactAvoidance = true;
      }
      // pull in
      else if (theArmContact.armContactStateLeft != ArmContact::None)
      {
        float alpha = std::min((float)timeSinceContact/timeToPullPitch,1.0f);
        armMovement.angles[Joints::lShoulderPitch] = Angle::normalize((1-alpha)*leftPitchWhenContactStateChanged
          + alpha*armBackPitch);
        alpha = (timeSinceContact > time4_5) ? (float)(timeSinceContact-time4_5)/timeToPullArmIn : 0;
        alpha = std::min(alpha,1.f);
        armMovement.angles[Joints::lElbowRoll] = alpha*armBackElbowRoll;
        armMovement.angles[Joints::lShoulderRoll] = Angle::normalize((1-alpha)*Angle::fromDegrees(curparams.arms1) + (alpha) * -0.3f);
        armMovement.armsInContactAvoidance = true;
      }
    }
    
    
    // Right Arm
    timeSinceContact = theFrameInfo.getTimeSince(rightTimeWhenContactStateChanged);
    {
      // back to original state
      if (timeSinceContact < timeToMoveArm && theArmContact.armContactStateRight == ArmContact::None)
      {
        float alpha = std::min((float)(timeSinceContact-timeToHoldArmBack-timeToMoveArm)/timeToPullArmIn,1.0f);
        armMovement.angles[Joints::rElbowRoll] = (1-alpha) * (-armBackElbowRoll);
        armMovement.angles[Joints::rShoulderRoll] = Angle::normalize((alpha)*Angle::fromDegrees(-curparams.arms1) + (1-alpha) * 0.3f);
        alpha = (timeSinceContact > timeToHoldArmBack+timeToMoveArm+timeToPullArmIn) ?
          (float)(timeSinceContact-(timeToHoldArmBack+timeToMoveArm+timeToPullArmIn))/timeToPullPitch : 0;
        alpha = std::min(alpha,1.f);
        armMovement.angles[Joints::rShoulderPitch] = Angle::normalize((alpha)*rightPitchWhenContactStateChanged
          + (1-alpha)*armBackPitch);
        armMovement.armsInContactAvoidance = true;
      }
      // pull in
      else if (theArmContact.armContactStateRight != ArmContact::None)
      {
        float alpha = std::min((float)timeSinceContact/timeToPullPitch,1.0f);
        armMovement.angles[Joints::rShoulderPitch] = Angle::normalize((1-alpha)*rightPitchWhenContactStateChanged
          + alpha*armBackPitch);
        alpha = (timeSinceContact > time4_5) ? (float)(timeSinceContact-time4_5)/timeToPullArmIn : 0;
        alpha = std::min(alpha,1.f);
        armMovement.angles[Joints::rElbowRoll] = alpha * (-armBackElbowRoll);
        armMovement.angles[Joints::rShoulderRoll] = Angle::normalize((1-alpha)*Angle::fromDegrees(-curparams.arms1) + (alpha) * 0.3f);
        armMovement.armsInContactAvoidance = true;
      }
    }
  }
}

MAKE_MODULE(ArmAnimator, dortmundWalkingEngine)
