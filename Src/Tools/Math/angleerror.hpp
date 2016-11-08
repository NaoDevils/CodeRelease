/**
 
 */
#include "Representations/Infrastructure/JointAngles.h"
class AngleError
{
public:
	AngleError () { err=0; added=0; };
	float err;
	int added;
	bool calc(const JointAngles &target, const JointAngles &measured, int sensorDelay, int steplen)
	{
		bool newvalue=false;
		if (added>=steplen)
		{
			added=0;
			err=0;
			newvalue=true;
		}
		delayBuffer.push_front(target);
    JointAngles &delayedTarget=delayBuffer[sensorDelay];
		for (int i=Joints::lHipYawPitch; i<Joints::numOfJoints; i++)
		{
			err+=(float)((delayedTarget.angles[i]-measured.angles[i])*(delayedTarget.angles[i]-measured.angles[i]));
		}
		added++;
		return newvalue;
	}
private:
	RingBuffer<JointAngles, 100> delayBuffer;
};