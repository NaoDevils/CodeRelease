/** 
* @file NaoKinematic.cpp
* This file implements the inverse kinematic.
* @author <A href="mailto:Stefan.Czarnetzki@uni-dortmund.de">Stefan Czarnetzki</A>
* @author <a href="mailto:oliver.urbann@tu-dortmund.de"> Oliver Urbann</a> 
*/

//#define LOGGING
#include "DortmundWalkingEngine/StepData.h"
#include "NaoKinematic.h"
#include "Tools/Motion/InverseKinematic.h"
#ifndef WALKING_SIMULATOR
//#include "Platform/GTAssert.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#else
#include "math/Pose3D_D.h"
#include "math/Common.h"
#include "csvlogger.h"
#endif
NaoKinematic::NaoKinematic() 
{
}

NaoKinematic::~NaoKinematic() {

}



Vector3f NaoKinematic::checkConstraints(Vector3f lf, float lfr, Vector3f rf, float rfr, bool correctleft)
{
	Vector2f checkPoints[2], constraintPoint, innerPoint;
	Vector3f cf3D;

	checkPoints[0].x()=theRobotDimensions.footFront;
	checkPoints[1].x()=theRobotDimensions.footBack;
	constraintPoint.x()=theRobotDimensions.footFront;
	innerPoint.x()=theRobotDimensions.footFront;
  float footInner = theRobotDimensions.footInner - 10;
  float footOuter = theRobotDimensions.footOuter - 10;
	if (correctleft)
	{
		checkPoints[0].y()=-footInner;
		checkPoints[0].rotate(lfr);
		checkPoints[0].x()+=lf.x();
		checkPoints[0].y()+=lf.y();
		
		checkPoints[1].y()=-footInner;
		checkPoints[1].rotate(lfr);
		checkPoints[1].x()+=lf.x();
		checkPoints[1].y()+=lf.y();
		
		constraintPoint.y()=footInner;
		constraintPoint.rotate(rfr);
		constraintPoint.x()+=rf.x();
		constraintPoint.y()+=rf.y();

		innerPoint.y()=-footOuter;
		innerPoint.rotate(rfr);
		innerPoint.x()+=rf.x();
		innerPoint.y()+=rf.y();

		cf3D=lf;
	}
	else
	{
		// right foot will be checked and moved

		checkPoints[0].y()=footInner;
		checkPoints[0].rotate(rfr);
		checkPoints[0].x()+=rf.x();
		checkPoints[0].y()+=rf.y();
		
		checkPoints[1].y()=footInner;
		checkPoints[1].rotate(rfr);
		checkPoints[1].x()+=rf.x();
		checkPoints[1].y()+=rf.y();
		
		constraintPoint.y()=-footInner;
		constraintPoint.rotate(lfr);
		constraintPoint.x()+=lf.x();
		constraintPoint.y()+=lf.y();

		innerPoint.y()=footOuter;
		innerPoint.rotate(lfr);
		innerPoint.x()+=lf.x();
		innerPoint.y()+=lf.y();

		cf3D=rf;
	}

	Vector2f a=innerPoint-constraintPoint;
	Vector2f b[2];
	b[0]=checkPoints[0]-constraintPoint;
	b[1]=checkPoints[1]-constraintPoint;

	a.normalize();

  float dot[2];
	dot[0]=a.x()*b[0].x()+a.y()*b[0].y();
	dot[1]=a.x()*b[1].x()+a.y()*b[1].y();

	int largestIndex;
	if (dot[0]>dot[1])
		largestIndex=0;
	else
		largestIndex=1;

	Vector2f cf(cf3D.x(), cf3D.y());
	
	if (dot[largestIndex]>0)
			cf-=a*dot[largestIndex];
	
	cf3D.x()=cf.x();
	cf3D.y()=cf.y();
	return cf3D; 
}


bool NaoKinematic::calcLegJoints(Joints::Joint whichSideJoint0, const Vector3f& position, const Vector3f& rotation, float t0, JointRequest& jointRequest, const RobotDimensions &robotDimensions)
{
  bool result = true;

  float footHeight = robotDimensions.footHeight;
  float tibiaLength = robotDimensions.lowerLegLength;
  float thighLength = robotDimensions.upperLegLength;
  float hipOffsetY = robotDimensions.yHipOffset;
  float hipOffsetZ = 0;//85;

#ifdef WALKING_SIMULATOR
	//hipOffsetZ = 85;
#endif

  float right = static_cast<float>((whichSideJoint0 == Joints::lHipYawPitch) ? -1 : 1);


  float tiltAngleOfFirstJointAxis =- pi / 2 + right * pi / 4;

  // Rotate around X to create HipYawPitch axis
	Matrix3f M1 = Matrix3f::Identity();
	M1(1,1) = std::cos(tiltAngleOfFirstJointAxis);
  M1(2,1) = std::sin(tiltAngleOfFirstJointAxis);
	M1(1,2) = -std::sin(tiltAngleOfFirstJointAxis);
	M1(2,2) = std::cos(tiltAngleOfFirstJointAxis);

  // When created rotate back
	Matrix3f M2 = Matrix3f::Identity();
	M2(1,1) = std::cos(tiltAngleOfFirstJointAxis);
	M2(2,1) = -std::sin(tiltAngleOfFirstJointAxis);
	M2(1,2) = std::sin(tiltAngleOfFirstJointAxis);
	M2(2,2) = std::cos(tiltAngleOfFirstJointAxis);

  float rx=rotation.x();

	// BEGIN	footRotX=rotMatrixAroundX(rx);
	//			footRot=rotMatrixAroundY(ry);
	//			footHeightVec=footRot*footRotX*[0;0;46];
	RotationMatrix footRot, footRotX;
	Vector3f footHeightVec(0, 0, footHeight);
	footRotX.rotateX(rx);
	footRot.rotateY(rotation.y());
	footHeightVec = footRot * footRotX * footHeightVec;

	// END

	// BEGIN p = M1 * rotMatrixAroundZ(-t0)*M2*([x;y;z]+[0;0;46]+[0;right*50;85]);
	RotationMatrix rotZ;
	rotZ.rotateZ(-t0);

	Vector3f p(position.x(), position.y() + right * hipOffsetY, position.z() + hipOffsetZ);
	p = p + footHeightVec;
	p = M1 * rotZ * M2 * p;
	// END p = M1 * rotMatrixAroundZ(-t0)*M2*([x;y;z]+[0;0;46]+[0;right*50;85]);

  float ysign = 1;

  float r = p.norm();
#if 0
  ASSERT(r <= tibiaLength + thighLength);
#else
	if (r>tibiaLength + thighLength)
	{
		//OUTPUT(idText, text, "Bein zu kurz!\r\n");
		p.normalize(tibiaLength + thighLength);
		r=tibiaLength + thighLength;
		//Vector3f(0, 0, -hipOffsetZ) + M1 * rotZ * M2 * Vector3f(
    result = false;
	}
#endif
  // Kosinussatz fuer das Knie
  float t3 = std::acos((sqr(tibiaLength)+sqr(thighLength)-sqr(r))/(2*tibiaLength*thighLength));
  float t1 = std::atan(p[1]/p[2]);

	Vector3f p1(p);
	p1[0] = 0;
  float t2a = std::acos((sqr(thighLength)+sqr(r)-sqr(tibiaLength))/(2*thighLength*r));

  float soll;
	if (p[0]!=0)
		soll= std::atan(p[0]/p1.norm());
	else
		soll=0;

  float t2=t2a+soll;


	tiltAngleOfFirstJointAxis = - pi / 2 +  pi / 4;
  
  M1(1,1) = std::cos(tiltAngleOfFirstJointAxis);
  M1(2,1) = std::sin(tiltAngleOfFirstJointAxis);
  M1(1,2) = -std::sin(tiltAngleOfFirstJointAxis);
  M1(2,2) = std::cos(tiltAngleOfFirstJointAxis);
  
  M2(1,1) = std::cos(tiltAngleOfFirstJointAxis);
  M2(2,1) = -std::sin(tiltAngleOfFirstJointAxis);
  M2(1,2) = std::sin(tiltAngleOfFirstJointAxis);
  M2(2,2) = std::cos(tiltAngleOfFirstJointAxis);
  
  // BEGIN R=M1*rotMatrixAroundZ(t0)*M2; R=R*rotMatrixAroundX(ysign*t1)*rotMatrixAroundY(-t2)*rotMatrixAroundY(pi-t3);
	Matrix3f R = Matrix3f::Identity();
	RotationMatrix rotZ2, rotY4, rotX, rotY5;
	rotZ2.rotateZ(t0);
	rotY4.rotateY(-t2);
	rotX.rotateX(ysign * t1);
	rotY5.rotateY(pi - t3);
	R = M1 * rotZ2 * M2 * rotX * rotY4 * rotY5;
	// END R=M1*rotMatrixAroundZ(t0)*M2; R=R*rotMatrixAroundX(ysign*t1)*rotMatrixAroundY(-t2)*rotMatrixAroundY(pi-t3);

	// BEGIN schnitt=cross(R*[0;1;0], [0;0;1]);
	Vector3f v3(0, 0, 1);
	Vector3f v4(0, 1, 0);
	v4 = R * v4;
	Vector3f schnitt(v4.cross(footRot * v3));
	// END schnitt=cross(R*[0;1;0], [0;0;1]);


	Vector3f vek(R * v3);

  float t4 = -std::asin((schnitt.dot(vek)) / (schnitt.norm() * vek.norm()));

	// BEGIN R=R*rotMatrixAroundY(t4);
	RotationMatrix rotY6;
	rotY6.rotateY(t4);
	R = R * rotY6;
	// END R=R*rotMatrixAroundY(t4);

	// BEGIN schnitt=cross(R*[1;0;0], [0;0;1]);
	Vector3f v5(1, 0, 0);
	schnitt = (R * v5).cross(footRot * v3);
	// END schnitt=cross(R*[1;0;0], [0;0;1]);

	vek = R * v3;

	// BEGIN t5=-asin((schnitt'*vek)/(norm(schnitt)*norm(vek)))+rx;
  float t5 = -std::asin((schnitt.dot(vek)) / (schnitt.norm() * vek.norm()))+right*rx;
	// END

	jointRequest.angles[whichSideJoint0 + 0] = (float)t0; // [-90 ;  0 ] = [...
	jointRequest.angles[whichSideJoint0 + 1] = (float)(-t1); // [-100; 25 ] = [...
	jointRequest.angles[whichSideJoint0 + 2] = (float)-t2; // [-45 ; 25 ]
	jointRequest.angles[whichSideJoint0 + 3] = pi - (float)t3; // [ 0  ; 130]
	jointRequest.angles[whichSideJoint0 + 4] = (float)t4; // [-75 ; 45 ]
	jointRequest.angles[whichSideJoint0 + 5] = (float)-t5; // [-25 ; 45 ]
  return result;
}

bool NaoKinematic::calcLegJoints(Joints::Joint whichSideJoint0, const Vector3f& position, const Vector3f& rotation, JointRequest& jointRequest, const RobotDimensions &robotDimensions)
{
	// to provide clearification about the constants in the robot dimensions:
	// (referring to the figure provided by aldebaran)
	// http://www.aldebaran-robotics.com/robocup/doc/docref/general/robocup/media/RobocupmDHDefintion.jpg
	//double footHeight =46;//= theRobotDimensions.heightLeg5Joint;
	//double tibiaLength =100;//= theRobotDimensions.lowerLegLength;
	//double thighLength =100;//= theRobotDimensions.upperLegLength;
	//double hipOffsetY =50;//= 0.5 * theRobotDimensions.lengthBetweenLegs;
	//// HipOffsetZ = ?
	//double hipOffsetZ = 0;//85;
  bool result = true;
  
  float footHeight = robotDimensions.footHeight;
  float tibiaLength = robotDimensions.lowerLegLength;
  float thighLength = robotDimensions.upperLegLength;
  float hipOffsetY = robotDimensions.yHipOffset;
  float hipOffsetZ = 0;//85;

#ifdef WALKING_SIMULATOR
	//hipOffsetZ = 85;
#endif

  float sign = static_cast<float>((whichSideJoint0 == Joints::lHipYawPitch) ? -1 : 1);

	RotationMatrix rot;
	rot.rotateX(rotation.x());
	rot.rotateY(rotation.y());
	rot.rotateZ(rotation.z());

	Pose3f T(rot,position);

	//    T = T(Waist->J0) T0to5 T(J5->Foot)
	// => T0to5 = T(Waist->J0)^-1 T T(J5->Foot)^-1
	// with M1 = T(Waist->J0)^-1
	//      M2 = T(J5->Foot)^-1

	Pose3f M1(0, - sign * hipOffsetY, - hipOffsetZ);
	M1.invert();
	Pose3f M2(0,0, - footHeight);
	M2.invert();

	Pose3f T0to5 = M1 * T * M2;


	// now let's calculate the angles
  float t0,t1,t2,t3,t4,t5;
  float angle1,angle2; // will be needed later

  float length = T0to5.translation.norm(); // sqrt( T0to5(1,4) *  T0to5(1,4) + T0to5(2,4) * T0to5(2,4) + T0to5(3,4) * T0to5(3,4) );

	if (length > thighLength + tibiaLength)
	{
		t3 = 0;
		angle1 = 0;
		angle2 = 0;
	}
	else
	{
		// first the knee angle by law of cosines
    float kneeAngle = std::acos( (tibiaLength*tibiaLength + thighLength*thighLength - length*length) / (2*thighLength*tibiaLength) );
		t3 = pi - kneeAngle; // Will be a positive number. t3 might theoretically also be negative, but the knee joint can only be bent in one direction, so this is the correct one.

		angle1 = std::asin( thighLength * std::sin(kneeAngle) / length); // law of sines
		angle2 = pi - kneeAngle - angle1;
	}

	Pose3f T0to5Inv = T0to5.inverse();
	t5 = std::atan2( T0to5Inv.translation.y(), T0to5Inv.translation.z() ); // atan2(T0to5Inv(2,4),T0to5Inv(3,4));
	t4 = std::atan2( -T0to5Inv.translation.x(), std::sqrt(T0to5Inv.translation.y()*T0to5Inv.translation.y() + T0to5Inv.translation.z()*T0to5Inv.translation.z()) ) - angle1;


	//    T0to5 = T0to2 * T2to3 * T3to4 * T4to5
	// => T0to2 = T0to5 * T4to5^-1 * T3to4^-1 * T2to3^-1

	//RotationMatrix dummy; // somehow the static call doesn't work; has to be investigated
	Pose3f T4to5Inv(RotationMatrix().rotateX(-t5));
	Pose3f T3to4Inv(RotationMatrix().rotateY(-t4));
	Pose3f T2to3Inv(RotationMatrix().rotateY(-t3));

	Pose3f T0to2 = ((T0to5 * T4to5Inv) * T3to4Inv) * T2to3Inv;


  float c0,s0,c1,s1,c2,s2;



  float x = 1 / std::sqrt(2.f);

  // Matrixzugriff: (Zeile, Spalte)
  float X1 = T0to2.rotation(0,0); // = T0to2(1,1);
	//double X2 = T0to2(1,2);
  float X3 = T0to2.rotation(0,2); // = T0to2(1,3);
  float X4 = T0to2.rotation(1,0); // = T0to2(2,1);
  float X5 = T0to2.rotation(1,1); // = T0to2(2,2);
  float X6 = T0to2.rotation(1,2); // = T0to2(2,3);
  float X7 = T0to2.rotation(2,0); // = T0to2(3,1);
  float X8 = T0to2.rotation(2,1); // = T0to2(3,2);
  float X9 = T0to2.rotation(2,2); // = T0to2(3,3);

  float X10 = sign * X6 + X9; // = c2(c1-s1)
  float X11 = sign * X4 + X7; // = s2(s1-c1)
  float X12 = X5 - sign * X8; // = c0(c1-s1)

	// X10 might be zero in the following cases:
	// 1.: c2 might be 0. This cannot be, because t2 is in [-100,25] and therefore can neither be -180 nor 180.
	// 2.: (c1-s1) might be 0. This can happen if t1 is 45 (or -135, but this is not possible).
	if (X10 == 0)
	{
		// ERROR HANDLING NEEDED!
		OUTPUT(idText,text,"something wrong in NaoKinematic");
    t0 = t1 = t2 = 0;
    result = false;
	}
	else // X10 != 0   => c2 != 0  && (c1-s1) != 0
	{
		t2 = std::atan( - X11 / X10 );
		s2 = std::sin(t2);
		c2 = std::cos(t2);

		c0 = c2 * X1 + s2 * X3;
		s0 = -(c2 * X7 + s2 * X9) / x;
		t0 = std::atan2(s0,c0);

		if (c0 == 0)
		{
			c1 = X5 + X10/(2*c2); // c1 != 0, see above
			s1 = X5 - X10/(2*c2);
		}
		else if (s2 == 0) // && c0 != 0)
		{
      float X14 = X5 - sign * X6; // = s1 + c0c1
			s1 = (X14 - X12) / (1 + c0); // c0 won't be -1, because t0 can neither be -180 nor 180
			c1 = (X14 + X12/c0) / (1 + c0);
		}
		else // (c0 != 0) && (s0 != 0)
		{
      float X13 = - X7/s2 - sign* X6/c2 + x*s0*s2/c2 - x*s0*c2/s2; // = c0(c1+s1)
			c1 = (X12 + X13)/(2*c0); // c0 != 0, because "else"...
			s1 = (-X12 + X13)/(2*c0);
		}
		t1 = std::atan2(s1,c1);
	}





	// sign's are necessary because bredo-framework uses a different joint calibration than the rotation directions shown in the above mentioned figure provided by aldebaran.
	jointRequest.angles[whichSideJoint0 + 0] = (float)t0; // [-90 ;  0 ] = [...
  jointRequest.angles[whichSideJoint0 + 1] = (float)sign*t1; // [-100; 25 ] = [...
  jointRequest.angles[whichSideJoint0 + 2] = (float)t2; // [-45 ; 25 ]
  jointRequest.angles[whichSideJoint0 + 3] = (float)t3; // [ 0  ; 130]
  jointRequest.angles[whichSideJoint0 + 4] = (float)t4; // [-75 ; 45 ]
  jointRequest.angles[whichSideJoint0 + 5] = (float)t5; // [-25 ; 45 ]

  return result;
}


void NaoKinematic::update(KinematicOutput& kinematicOutput)
{
#if 0
	MODIFY("NaoKinematic:rcxpKinematicRequest", rcxpKinematic);

	rcxpKinematic.calculated = false;
	DEBUG_RESPONSE("NaoKinematic:rcxpKinematic", 	
		Vector3f leftFootRCXP(rcxpKinematic.kinematicRequest[0], rcxpKinematic.kinematicRequest[1] , rcxpKinematic.kinematicRequest[2]);
	Vector3f leftRotRCXP(rcxpKinematic.kinematicRequest[3], rcxpKinematic.kinematicRequest[4] , rcxpKinematic.kinematicRequest[5]);
	Vector3f rightFootRCXP(rcxpKinematic.kinematicRequest[6], rcxpKinematic.kinematicRequest[7], rcxpKinematic.kinematicRequest[8]);
	Vector3f rightRotRCXP(rcxpKinematic.kinematicRequest[9], rcxpKinematic.kinematicRequest[10], rcxpKinematic.kinematicRequest[11]);
	calcLegJoints(Joints::lHipYawPitch, leftFootRCXP, leftRotRCXP, rcxpKinematic.jointRequest,theRobotDimensions);
	calcLegJoints(Joints::rHipYawPitch, rightFootRCXP, rightRotRCXP, rcxpKinematic.jointRequest,theRobotDimensions);

	rcxpKinematic.calculated = true;
	);
#endif
  
	Vector3f leftFoot(theKinematicRequest.leftFoot[0], theKinematicRequest.leftFoot[1], theKinematicRequest.leftFoot[2]);
	Vector3f rightFoot(theKinematicRequest.rightFoot[0], theKinematicRequest.rightFoot[1], theKinematicRequest.rightFoot[2]);
	Vector3f leftFootRot(theKinematicRequest.leftFoot[3], theKinematicRequest.leftFoot[4], theKinematicRequest.leftFoot[5]);
	Vector3f rightFootRot(theKinematicRequest.rightFoot[3], theKinematicRequest.rightFoot[4], theKinematicRequest.rightFoot[5]);

	//double mean=0;

	float distLeft=std::sqrt(sqr(leftFoot[0])+sqr(leftFoot[1])+sqr(leftFoot[2]));
	float distRight=std::sqrt(sqr(rightFoot[0])+sqr(rightFoot[1])+sqr(rightFoot[2]));

	switch (theKinematicRequest.kinematicType) 
	{
  case KinematicRequest::feet :
		// Stand on the foot, which is nearer to the center
		// This is foot has more pressure when the robot moves slowly
    if (!useBHKinematics)
    {
      if (distLeft < distRight)
      {
        calcLegJoints(Joints::lHipYawPitch, leftFoot, leftFootRot, kinematicOutput, theRobotDimensions);
        calcLegJoints(Joints::rHipYawPitch, rightFoot, rightFootRot, kinematicOutput.angles[Joints::lHipYawPitch], kinematicOutput, theRobotDimensions);
      }
      else
      {
        calcLegJoints(Joints::rHipYawPitch, rightFoot, rightFootRot, kinematicOutput, theRobotDimensions);
        calcLegJoints(Joints::lHipYawPitch, leftFoot, leftFootRot, kinematicOutput.angles[Joints::rHipYawPitch], kinematicOutput, theRobotDimensions);

      }
    }
    else
      //TODO Return value ignored
      InverseKinematic::calcLegJoints(
        Pose3f(RotationMatrix(leftFootRot.x(), leftFootRot.y(), leftFootRot.z()), leftFoot + Vector3f(0, 0, 40)),
        Pose3f(RotationMatrix(rightFootRot.x(), rightFootRot.y(), rightFootRot.z()), rightFoot + Vector3f(0, 0, 40)),
        kinematicOutput, theRobotDimensions);
		break;

	case KinematicRequest::bodyAndLeftFoot :
		rightFoot=checkConstraints(leftFoot, leftFootRot.z(), rightFoot, rightFootRot.z(), false);
    if (!useBHKinematics)
    {
      calcLegJoints(Joints::lHipYawPitch, leftFoot, leftFootRot, kinematicOutput, theRobotDimensions);
      calcLegJoints(Joints::rHipYawPitch, rightFoot, rightFootRot, kinematicOutput.angles[Joints::lHipYawPitch], kinematicOutput, theRobotDimensions); 
    }
    else
      //TODO Return value ignored
      InverseKinematic::calcLegJoints(
        Pose3f(RotationMatrix(leftFootRot.x(), leftFootRot.y(), leftFootRot.z()), leftFoot + Vector3f(0, 0, 40)),
        Pose3f(RotationMatrix(rightFootRot.x(), rightFootRot.y(), rightFootRot.z()), rightFoot + Vector3f(0, 0, 40)),
        kinematicOutput, theRobotDimensions);
		//kinematicOutput.angles[Joints::lHipRoll]+=0.1;
		break;

	case KinematicRequest::bodyAndRightFoot :	  
		leftFoot=checkConstraints(leftFoot, leftFootRot.z(), rightFoot, rightFootRot.z(), true);
    if (!useBHKinematics)
    {
      calcLegJoints(Joints::rHipYawPitch, rightFoot, rightFootRot, kinematicOutput,theRobotDimensions);
		  calcLegJoints(Joints::lHipYawPitch, leftFoot, leftFootRot, kinematicOutput.angles[Joints::rHipYawPitch], kinematicOutput,theRobotDimensions);
    }
    else
      //TODO Return value ignored
      InverseKinematic::calcLegJoints(
      Pose3f(RotationMatrix(leftFootRot.x(), leftFootRot.y(), leftFootRot.z()), leftFoot + Vector3f(0,0,40)),
      Pose3f(RotationMatrix(rightFootRot.x(), rightFootRot.y(), rightFootRot.z()), rightFoot + Vector3f(0, 0, 40)),
      kinematicOutput, theRobotDimensions);
		//kinematicOutput.angles[Joints::rHipRoll]-=0.1;
		break;
	default:
		break;

	}
#ifdef WALKING_SIMULATOR
	for (int i=(int)Joints::lHipYawPitch; i<12; i++)
	{
		kinematicOutput.angles[i]+=theKinematicRequest.offsets.angles[i];
	}
#endif
#ifdef TARGET_SIM
  // If this happens, go back on stack to ModuleManager and check
  // all Modules in "providers"
  for (int i = 0; i < Joints::numOfJoints; i++)
    ASSERT(kinematicOutput.angles[i] == kinematicOutput.angles[i]);
#endif
  
	kinematicOutput.angles[Joints::lShoulderPitch] = JointAngles::ignore;
	kinematicOutput.angles[Joints::lShoulderRoll] = JointAngles::ignore;
	kinematicOutput.angles[Joints::lElbowYaw] = JointAngles::ignore;
	kinematicOutput.angles[Joints::lElbowRoll] = JointAngles::ignore;

	kinematicOutput.angles[Joints::rShoulderPitch] = JointAngles::ignore;
	kinematicOutput.angles[Joints::rShoulderRoll] = JointAngles::ignore;
	kinematicOutput.angles[Joints::rElbowYaw] = JointAngles::ignore;
	kinematicOutput.angles[Joints::rElbowRoll] = JointAngles::ignore;
	kinematicOutput.angles[Joints::headYaw] = JointAngles::ignore;
	kinematicOutput.angles[Joints::headPitch] = JointAngles::ignore;

	if ((kinematicOutput.angles[Joints::lHipYawPitch] == JointAngles::ignore)
		||(kinematicOutput.angles[Joints::lHipYawPitch] == JointAngles::off))
		kinematicOutput.angles[Joints::lHipYawPitch] = theKinematicRequest.offsets.angles[Joints::lHipYawPitch];
	else
		kinematicOutput.angles[Joints::lHipYawPitch] += theKinematicRequest.offsets.angles[Joints::lHipYawPitch];

	if ((kinematicOutput.angles[Joints::lHipRoll] == JointAngles::ignore)
		||(kinematicOutput.angles[Joints::lHipRoll] == JointAngles::off))
		kinematicOutput.angles[Joints::lHipRoll] = theKinematicRequest.offsets.angles[Joints::lHipRoll];
	else
		kinematicOutput.angles[Joints::lHipRoll] += theKinematicRequest.offsets.angles[Joints::lHipRoll];

	if ((kinematicOutput.angles[Joints::lHipPitch] == JointAngles::ignore)
		||(kinematicOutput.angles[Joints::lHipPitch] == JointAngles::off))
		kinematicOutput.angles[Joints::lHipPitch] = theKinematicRequest.offsets.angles[Joints::lHipPitch];
	else
		kinematicOutput.angles[Joints::lHipPitch] += theKinematicRequest.offsets.angles[Joints::lHipPitch];

	if ((kinematicOutput.angles[Joints::lKneePitch] == JointAngles::ignore)
		||(kinematicOutput.angles[Joints::lKneePitch] == JointAngles::off))
		kinematicOutput.angles[Joints::lKneePitch] = theKinematicRequest.offsets.angles[Joints::lKneePitch];
	else
		kinematicOutput.angles[Joints::lKneePitch] += theKinematicRequest.offsets.angles[Joints::lKneePitch];

	if ((kinematicOutput.angles[Joints::lAnklePitch] == JointAngles::ignore)
		||(kinematicOutput.angles[Joints::lAnklePitch] == JointAngles::off))
		kinematicOutput.angles[Joints::lAnklePitch] = theKinematicRequest.offsets.angles[Joints::lAnklePitch];
	else
		kinematicOutput.angles[Joints::lAnklePitch] += theKinematicRequest.offsets.angles[Joints::lAnklePitch];

	if ((kinematicOutput.angles[Joints::lAnkleRoll] == JointAngles::ignore)
		||(kinematicOutput.angles[Joints::lAnkleRoll] == JointAngles::off))
		kinematicOutput.angles[Joints::lAnkleRoll] = theKinematicRequest.offsets.angles[Joints::lAnkleRoll];
	else
		kinematicOutput.angles[Joints::lAnkleRoll] += theKinematicRequest.offsets.angles[Joints::lAnkleRoll];


	if ((kinematicOutput.angles[Joints::rHipYawPitch] == JointAngles::ignore)
		||(kinematicOutput.angles[Joints::rHipYawPitch] == JointAngles::off))
		kinematicOutput.angles[Joints::rHipYawPitch] = theKinematicRequest.offsets.angles[Joints::rHipYawPitch];
	else
		kinematicOutput.angles[Joints::rHipYawPitch] += theKinematicRequest.offsets.angles[Joints::rHipYawPitch];

	if ((kinematicOutput.angles[Joints::rHipRoll] == JointAngles::ignore)
		||(kinematicOutput.angles[Joints::rHipRoll] == JointAngles::off))
		kinematicOutput.angles[Joints::rHipRoll] = theKinematicRequest.offsets.angles[Joints::rHipRoll];
	else
		kinematicOutput.angles[Joints::rHipRoll] += theKinematicRequest.offsets.angles[Joints::rHipRoll];

	if ((kinematicOutput.angles[Joints::rHipPitch] == JointAngles::ignore)
		||(kinematicOutput.angles[Joints::rHipPitch] == JointAngles::off))
		kinematicOutput.angles[Joints::rHipPitch] = theKinematicRequest.offsets.angles[Joints::rHipPitch];
	else
		kinematicOutput.angles[Joints::rHipPitch] += theKinematicRequest.offsets.angles[Joints::rHipPitch];

	if ((kinematicOutput.angles[Joints::rKneePitch] == JointAngles::ignore)
		||(kinematicOutput.angles[Joints::rKneePitch] == JointAngles::off))
		kinematicOutput.angles[Joints::rKneePitch] = theKinematicRequest.offsets.angles[Joints::rKneePitch];
	else
		kinematicOutput.angles[Joints::rKneePitch] += theKinematicRequest.offsets.angles[Joints::rKneePitch];

	if ((kinematicOutput.angles[Joints::rAnklePitch] == JointAngles::ignore)
		||(kinematicOutput.angles[Joints::rAnklePitch] == JointAngles::off))
		kinematicOutput.angles[Joints::rAnklePitch] = theKinematicRequest.offsets.angles[Joints::rAnklePitch];
	else
		kinematicOutput.angles[Joints::rAnklePitch] += theKinematicRequest.offsets.angles[Joints::rAnklePitch];

	if ((kinematicOutput.angles[Joints::rAnkleRoll] == JointAngles::ignore)
		||(kinematicOutput.angles[Joints::rAnkleRoll] == JointAngles::off))
		kinematicOutput.angles[Joints::rAnkleRoll] = theKinematicRequest.offsets.angles[Joints::rAnkleRoll];
	else
		kinematicOutput.angles[Joints::rAnkleRoll] += theKinematicRequest.offsets.angles[Joints::rAnkleRoll];

#ifdef WARN_ANGLES
	if (kinematicOutput.angles[Joints::lHipYawPitch]<-1.58 || kinematicOutput.angles[Joints::lHipYawPitch]>0.01)
		OUTPUT(idText, text, "Angle out of Range (LHipYawPitch): " << kinematicOutput.angles[Joints::lHipYawPitch]);
	if (kinematicOutput.angles[Joints::lHipRoll]<-0.79 || kinematicOutput.angles[Joints::lHipRoll]>0.44)
		OUTPUT(idText, text, "Angle out of Range (LHipRoll): " << kinematicOutput.angles[Joints::lHipRoll]);
	if (kinematicOutput.angles[Joints::lHipPitch]<-1.75 || kinematicOutput.angles[Joints::lHipPitch]>0.44)
		OUTPUT(idText, text, "Angle out of Range (LHipPitch): " << kinematicOutput.angles[Joints::lHipPitch]);
	if (kinematicOutput.angles[Joints::lKneePitch]<-0.01 || kinematicOutput.angles[Joints::lKneePitch]>2.27)
		OUTPUT(idText, text, "Angle out of Range (lKneePitch): " << kinematicOutput.angles[Joints::lKneePitch]);
	if (kinematicOutput.angles[Joints::lAnklePitch]<-1.31 || kinematicOutput.angles[Joints::lAnklePitch]>0.79)
		OUTPUT(idText, text, "Angle out of Range (lAnklePitch): " << kinematicOutput.angles[Joints::lAnklePitch]);
	if (kinematicOutput.angles[Joints::lAnkleRoll]<-0.44 || kinematicOutput.angles[Joints::lAnkleRoll]>0.79)
		OUTPUT(idText, text, "Angle out of Range (lAnkleRoll): " << kinematicOutput.angles[Joints::lAnkleRoll]);
	if (kinematicOutput.angles[Joints::rHipYawPitch]<-1.58 || kinematicOutput.angles[Joints::rHipYawPitch]>0.01)
		OUTPUT(idText, text, "Angle out of Range (rHipYawPitch): " << kinematicOutput.angles[Joints::rHipYawPitch]);
	if (kinematicOutput.angles[Joints::rHipRoll]<-0.79 || kinematicOutput.angles[Joints::rHipRoll]>0.44)
		OUTPUT(idText, text, "Angle out of Range (rHipRoll): " << kinematicOutput.angles[Joints::rHipRoll]);
	if (kinematicOutput.angles[Joints::rHipPitch]<-1.75 || kinematicOutput.angles[Joints::rHipPitch]>0.44)
		OUTPUT(idText, text, "Angle out of Range (rHipPitch): " << kinematicOutput.angles[Joints::rHipPitch]);
	if (kinematicOutput.angles[Joints::rKneePitch]<-0.01 || kinematicOutput.angles[Joints::rKneePitch]>2.27)
		OUTPUT(idText, text, "Angle out of Range (rKneePitch): " << kinematicOutput.angles[Joints::rKneePitch]);
	if (kinematicOutput.angles[Joints::rAnklePitch]<-1.31 || kinematicOutput.angles[Joints::rAnklePitch]>0.79)
		OUTPUT(idText, text, "Angle out of Range (rAnklePitch): " << kinematicOutput.angles[Joints::rAnklePitch]);
	if (kinematicOutput.angles[Joints::rAnkleRoll]<-0.44 || kinematicOutput.angles[Joints::rAnkleRoll]>0.79)
		OUTPUT(idText, text, "Angle out of Range (rAnkleRoll): " << kinematicOutput.angles[Joints::rAnkleRoll]);
#endif
	/*LOG("ExternalSimulator.csv", "LHipRoll", kinematicOutput.angles[Joints::lHipRoll]);
	LOG("ExternalSimulator.csv", "LHipPitch", kinematicOutput.angles[Joints::lHipPitch]);
	LOG("ExternalSimulator.csv", "lKneePitch", kinematicOutput.angles[Joints::lKneePitch]);
	LOG("ExternalSimulator.csv", "lAnklePitch", kinematicOutput.angles[Joints::lAnklePitch]);
	LOG("ExternalSimulator.csv", "lAnkleRoll", kinematicOutput.angles[Joints::lAnkleRoll]);

	LOG("ExternalSimulator.csv", "rHipRoll", kinematicOutput.angles[Joints::rHipRoll]);
	LOG("ExternalSimulator.csv", "rHipPitch", kinematicOutput.angles[Joints::rHipPitch]);
	LOG("ExternalSimulator.csv", "rKneePitch", kinematicOutput.angles[Joints::rKneePitch]);
	LOG("ExternalSimulator.csv", "rAnklePitch", kinematicOutput.angles[Joints::rAnklePitch]);
	LOG("ExternalSimulator.csv", "rAnkleRoll", kinematicOutput.angles[Joints::rAnkleRoll]); */
}

MAKE_MODULE(NaoKinematic, dortmundWalkingEngine)
