/**
 * @file Controller/LocalRobot.h
 *
 * Declaration of LocalRobot.
 *
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Controller/RobotConsole.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/SonarSensorData.h"
#include "Representations/MotionControl/OdometryData.h"
#include "SimulatedRobot.h"
struct Settings;

/**
 * @class LocalRobot
 *
 * A process that is instantiated to either directly control a physical robot,
 * a simulated one, or to replay a logfile.
 */
class LocalRobot : public RobotConsole
{
private:
  DEBUGGING;
  Image image; /**< The simulated image sent to the robot code. */
  ImageUpper imageUpper; /**< The simulated image sent to the robot code. */
  CameraInfo cameraInfo; /**< The information about the camera that took the image sent to the robot code. */
  CameraInfoUpper cameraInfoUpper; /**< The information about the camera that took the image sent to the robot code. */
  FsrSensorData fsrSensorData; /**< The simulated inertia sensor data sent to the robot code. */
  JointSensorData jointSensorData; /**< The simulated joint measurements sent to the robot code. */
  InertialSensorData inertialSensorData; /**< The simulated inertia sensor data sent to the robot code. */
  SonarSensorData sonarSensorData; /**< The simulated sonar sensor data sent to the robot code. */
  Pose2f robotPose = Pose2f(); /**< The robot's pose, used for some internal computations. */
  GroundTruthWorldState worldState; /**< The current world state of the simulation scene, sent to the robot code. */
  GroundTruthOdometryData odometryData; /**< The simulated odometry data sent to the robot code. */
  unsigned nextImageTimeStamp, /**< The theoretical timestamp of the next image to be calculated. */
      imageLastTimeStampSent, /**< The timestamp of the last sent image. */
      jointLastTimeStampSent; /**< The timestamp of the last sent joint data. */
  SimulatedRobot simulatedRobot; /**< The interface to simulated objects. */
  Semaphore updateSignal; /**< A signal used for synchronizing main() and update(). */
  Semaphore updatedSignal; /**< A signal used for yielding processing time to main(). */
  SimRobotCore2::Body* puppet; /**< A pointer to the puppet when there is one during logfile replay. Otherwise 0. */
  unsigned ping = 0;
  unsigned pong = 0;

public:
  /**
   * Constructor.
   */
  LocalRobot(Settings& settings);

  /**
   * The function is called from the framework once in every frame
   */
  virtual bool main();

  /**
   * The function must be called to exchange data with SimRobot.
   * It sends the motor commands to SimRobot and acquires new sensor data.
   */
  void update();

  virtual bool handleMessage(InMessage& message);
};
