#include "OrientationArmAnimator.h"

OrientationArmAnimator::OrientationArmAnimator()
{

}


OrientationArmAnimator::~OrientationArmAnimator()
{

}

void OrientationArmAnimator::update(ArmMovement& armMovement)
{
	armMovement.angles[Joints::lShoulderPitch]=0;
	armMovement.angles[Joints::lShoulderRoll]=(float)(theWalkingEngineParams.arms1*3.1415/180);
	armMovement.angles[Joints::lElbowYaw]=0;
	armMovement.angles[Joints::lElbowRoll]=0;

	armMovement.angles[Joints::rShoulderPitch]=0;
	armMovement.angles[Joints::rShoulderRoll]=(float)(theWalkingEngineParams.arms1*3.1415/180);
	armMovement.angles[Joints::lElbowYaw]=0;
	armMovement.angles[Joints::lElbowRoll]=0;

	float leftArm,rightArm;

  rightArm=leftArm=-90*3.1415f/180 - theWalkingEngineParams.armFactor*((float)theInertialSensorData.angle.y());
	armMovement.angles[Joints::lShoulderPitch] = leftArm;
	armMovement.angles[Joints::rShoulderPitch]= rightArm;
	armMovement.usearms=true;
}

MAKE_MODULE(OrientationArmAnimator, dortmundWalkingEngine)
