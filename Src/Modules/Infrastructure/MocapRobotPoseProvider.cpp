/**
* @file MocapRobotPoseProvider.cpp
* This file declares a module that provides ground truth data based on the mocap data
* @author Janine Hemmers
*/

#include "MocapRobotPoseProvider.h"
#include <float.h>

PROCESS_LOCAL MocapRobotPoseProvider* MocapRobotPoseProvider::theInstance = 0;

MocapRobotPoseProvider::MocapRobotPoseProvider()
{
  theInstance = this;
}

MocapRobotPoseProvider::~MocapRobotPoseProvider()
{
  theInstance = 0;
}


void MocapRobotPoseProvider::update(RobotPose & robotPose)
{
  if (theInstance) {
    if (robotID < 0) { getRobotID(); }
    else {
      robotPose.translation = calcTranslation(robotID);
      robotPose.rotation = calcRotation();
      robotPose.validity = calcValidity(robotID);
    }
  }
}

void MocapRobotPoseProvider::update(MocapRobotPose& mocapRobotPose)
{
  
  if (theInstance) {
    if (robotID < 0) { getRobotID(); }
    else{
    mocapRobotPose.translation = calcTranslation(robotID);
    mocapRobotPose.rotation = calcRotation();
    mocapRobotPose.validity = calcValidity(robotID);
    mocapRobotPose.mocapFrameNumber = theMocapData.frameNumber;
    mocapRobotPose.timestamp = theFrameInfo.time;
    }
  }
}
 
void MocapRobotPoseProvider::update(MocapBallModel & mocapBallModel)
{
  if (theInstance) {
    if (ballID < 0) { 
      getBallID(); 
    }
    else {
      mocapBallModel.translation = calcTranslation(ballID);
      mocapBallModel.validity = calcValidity(ballID);
      mocapBallModel.mocapFrameNumber = theMocapData.frameNumber;
      mocapBallModel.timestamp = theFrameInfo.time;
    }
  }
}

Vector2f MocapRobotPoseProvider::calcTranslation(int ID)
{
  std::vector<MocapRigidbody> rigidBodies = theMocapData.rigidBodies;
  MocapRigidbody theRigidbody;
  for (std::vector<MocapRigidbody>::size_type i = 0; i < rigidBodies.size(); i++) {
    if (rigidBodies[i].id == ID) {
      theRigidbody = rigidBodies[i];
    }
  }

  if (theRigidbody.id > -1) {
    return Vector2f(theRigidbody.z*1000, theRigidbody.x*1000);    
  }
  return Vector2f(0,0);
}

Angle MocapRobotPoseProvider::calcRotation()
{
  Quat q;

  std::vector<MocapRigidbody> rigidBodies = theMocapData.rigidBodies;
  MocapRigidbody theRigidbody;
  for (std::vector<MocapRigidbody>::size_type i = 0; i < rigidBodies.size(); i++) {
    if (rigidBodies[i].id == robotID) {
      theRigidbody = rigidBodies[i];
    }
  }
 
  if (theRigidbody.id > -1) {
    q.x = theRigidbody.qx;
    q.y = theRigidbody.qy;
    q.z = theRigidbody.qz;
    q.w = theRigidbody.qw;

    //OUTPUT_TEXT("X: " << q.x << ", Y: " << q.y << ", Z: " << q.z << ", W: " << q.w);
    q.w = -q.w;
    Vector3f Euler = FromQ2(q);
    /* Return euler angle y because it is the same than our z */
    if(Euler[1]<=180)
    {
      return Angle((-Euler[1]) * pi / 180);
    }
    else {
      return Angle((360-Euler[1]) * pi / 180);
    }
  }
  return Angle(0);
}

float MocapRobotPoseProvider::calcValidity(int ID)
{
  std::vector<MocapRigidbody> rigidBodies = theMocapData.rigidBodies;
  MocapRigidbody theRigidbody;
  for (std::vector<MocapRigidbody>::size_type i = 0; i < rigidBodies.size(); i++) {
    if (rigidBodies[i].id == ID) {
      theRigidbody = rigidBodies[i];
    }
  }

  if (theRigidbody.id > -1 && theRigidbody.trackingValid) {
    return 1.0f;
  }
  return 0.0f;
}


/*** HELPER FUNCTIONS ***/

void MocapRobotPoseProvider::getBallID()
{
  std::vector<MocapRigidbody> rigidBodies = theMocapData.rigidBodies;
  MocapMarkerSet markerSet = theMocapData.markerSetBall;
  //Needs to be lower than -1. Otherwise ist could validate an invalid rigidbody 
  //(all invalid rigidbodies have id = -1)
  int retval = -2;
  for (std::vector<MocapRigidbody>::size_type i = 0; i < rigidBodies.size(); i++) {
    float x = rigidBodies[i].firstMarker.x();
    float y = rigidBodies[i].firstMarker.y();
    float z = rigidBodies[i].firstMarker.z();
    for (std::vector<Vector3f>::size_type l = 0; l < markerSet.marker.size(); l++)
    {
      float mx = markerSet.marker[l][0];
      float my = markerSet.marker[l][1];
      float mz = markerSet.marker[l][2];

      if (x == mx && y == my && z == mz) {

        retval = rigidBodies[i].id;
        OUTPUT_TEXT("Mocap System: Ball" << " mapped to ID " << retval);
      }
    }

    //printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k], markerData[k * 3], markerData[k * 3 + 1], markerData[k * 3 + 2]);

  }
  ballID = retval;
  
}


void MocapRobotPoseProvider::getRobotID()
{
  std::vector<MocapRigidbody> rigidBodies = theMocapData.rigidBodies;
  MocapMarkerSet markerSet = theMocapData.markerSet;
  //Needs to be lower than -1. Otherwise ist could validate an invalid rigidbody 
  //(all invalid rigidbodies have id = -1)
  int retval = -1;
  for (std::vector<MocapRigidbody>::size_type i = 0; i < rigidBodies.size(); i++) {
    float x = rigidBodies[i].firstMarker.x();
    float y = rigidBodies[i].firstMarker.y();
    float z = rigidBodies[i].firstMarker.z();
    for (std::vector<Vector3f>::size_type l = 0; l < markerSet.marker.size(); l++)
    {
      float mx = markerSet.marker[l][0];
      float my = markerSet.marker[l][1];
      float mz = markerSet.marker[l][2];
      //| a - b | <epsilon
      if ( (std::abs(x-mx) < FLT_EPSILON) && (std::abs(y - my) < FLT_EPSILON)  && (std::abs(z - mz) < FLT_EPSILON)) {

        retval = rigidBodies[i].id;
        OUTPUT_TEXT("Mocap System: " << markerSet.name << " mapped to ID " << retval);
      }
    }

    //printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k], markerData[k * 3], markerData[k * 3 + 1], markerData[k * 3 + 2]);

  }
  robotID = retval;
  
}

Vector3f MocapRobotPoseProvider::FromQ2(Quat q1)
{
  float sqw = q1.w * q1.w;
  float sqx = q1.x * q1.x;
  float sqy = q1.y * q1.y;
  float sqz = q1.z * q1.z;
  float unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
  float test = q1.x * q1.w - q1.y * q1.z;
  Vector3f v;

  if (test > 0.4995f * unit)
  { // singularity at north pole
    v[1] = 2 * static_cast<float>(atan2(q1.y, q1.x));
    v[0] = pi / 2;
    v[2] = 0;
    return NormalizeAngles(v * 180 / pi);
  }
  if (test < -0.4995f * unit)
  { // singularity at south pole
    v[1] = -2 * static_cast<float>(atan2(q1.y, q1.x));
    v[0] = -pi / 2;
    v[2] = 0;
    return NormalizeAngles(v * 180 / pi);
  }
  Quat q;
  q.x = q1.w;
  q.y = q1.z;
  q.z = q1.x;
  q.w = q1.y;
  v[1] = (float)atan2(2 * q.x * q.w + 2 * q.y * q.z, 1 - 2 * (q.z * q.z + q.w * q.w));     // Yaw
  v[0] = (float)asin(2 * (q.x * q.z - q.w * q.y));                                         // Pitch
  v[2] = (float)atan2(2 * q.x * q.y + 2 * q.z * q.w, 1 - 2 * (q.y * q.y + q.z * q.z));     // Roll
  return NormalizeAngles(v * 180 / pi);
}

Vector3f MocapRobotPoseProvider::NormalizeAngles(Vector3f angles)
{
  angles[0] = NormalizeAngle(angles[0]);
  angles[1] = NormalizeAngle(angles[1]);
  angles[2] = NormalizeAngle(angles[2]);
  return angles;
}

float MocapRobotPoseProvider::NormalizeAngle(float angle)
{
  while (angle > 360)
    angle -= 360;
  while (angle < 0)
    angle += 360;
  return angle;
}

MAKE_MODULE(MocapRobotPoseProvider, cognitionInfrastructure)