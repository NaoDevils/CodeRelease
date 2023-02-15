/**
* @file OdometryOnlySelfLocator.cpp
*
* Declares a class that performs self-localization by adding odometry offsets.
* This is not for real self-localization but for testing and debugging.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "OdometryOnlySelfLocator.h"

void OdometryOnlySelfLocator::update(RobotPose& robotPose)
{
  Pose2f offset = theOdometryData - referenceOdometry;
  distance += (theOdometryData - lastOdometryData).translation.norm();
  rotation += (theOdometryData - lastOdometryData).rotation;
  robotPose = base + offset;
  robotPose.validity = 1.f;
  robotPose.sideConfidenceState = SideConfidence::ConfidenceState::CONFIDENT;

  MODIFY("module:OdometryOnlySelfLocator:basePose", base);
  DEBUG_RESPONSE_ONCE("module:OdometrOnlySelfLocator:resetReferenceOdometry")
  {
    referenceOdometry = theOdometryData;
    distance = 0.f;
  }

  DEBUG_RESPONSE_ONCE("module:OdometrOnlySelfLocator:getDistance")
  {
    OUTPUT_TEXT("Distance: " << distance << "mm");
    OUTPUT_TEXT("Distance(euklidisch): " << offset.translation.norm() << "mm");
    OUTPUT_TEXT("Rotation: " << rotation << "deg");
    OUTPUT_TEXT("Rotation(pose): " << offset.rotation.toDegrees() << "deg");
  }

  if (theKeySymbols.pressed_and_released[KeyStates::headMiddle] && !lastPressedAndRelease)
  {
    OUTPUT(idConsole, text, "set representation:MotionRequest motion = walk; specialActionRequest = { specialAction = sitDown; mirror = false; }; walkRequest = { requestType = speed; rotationType = irrelevant; request = { rotation = 0deg; translation = { x = 0; y = 0; }; }; stepRequest = none; }; kickRequest = { kickMotionType = none; mirror = false; dynamical = false; armsBackFix = true; dynPoints = []; kickTarget = { x = 1000; y = 0; }; };");
    OUTPUT(idConsole, text, "dr module:OdometrOnlySelfLocator:getDistance");
  }

  lastPressedAndRelease = theKeySymbols.pressed_and_released[KeyStates::headMiddle];
  lastOdometryData = theOdometryData;
}

MAKE_MODULE(OdometryOnlySelfLocator, modeling)
