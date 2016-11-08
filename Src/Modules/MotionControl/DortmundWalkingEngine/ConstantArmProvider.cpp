#include "ConstantArmProvider.h"

ConstantArmProvider::ConstantArmProvider()
{
	intfac=0;
}


ConstantArmProvider::~ConstantArmProvider()
{

}

void ConstantArmProvider::update(ArmMovement& armMovement)
{
	if (intfac==0)
	{

	}

	armMovement.angles[Joints::lShoulderRoll]=-2.09f;
	armMovement.angles[Joints::lShoulderPitch]=0;
	armMovement.angles[Joints::lElbowRoll]=-1.7f;
	armMovement.angles[Joints::lElbowYaw]=-1.55f;

	armMovement.angles[Joints::rShoulderRoll]=-2.09f;
	armMovement.angles[Joints::rShoulderPitch]=0;
	armMovement.angles[Joints::rElbowRoll]=-1.7f;
	armMovement.angles[Joints::rElbowYaw]=-1.55f;


	armMovement.usearms=true;
}

MAKE_MODULE(ConstantArmProvider, dortmundWalkingEngine)
