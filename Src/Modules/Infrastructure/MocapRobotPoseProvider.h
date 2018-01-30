/**
* @file MocapRobotPoseProvider.h
* This file declares a module that provides ground truth data based on the mocap data
* @author Janine Hemmers
*/

#pragma once 


#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/MocapData.h"
#include "Representations/Infrastructure/MocapRigidbody.h"
#include "Tools/ProcessFramework/MocapHandler.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/MocapBallModel.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Angle.h"
#include "Tools/Settings.h"

MODULE(MocapRobotPoseProvider,
{ ,
REQUIRES(FrameInfo),
REQUIRES(MocapData),
PROVIDES(MocapRobotPose),
PROVIDES(MocapBallModel),
PROVIDES(RobotPose),
});

typedef struct { float x, y, z, w; } Quat; /* Quaternion */

class MocapRobotPoseProvider : public MocapRobotPoseProviderBase
{
private:
  static PROCESS_LOCAL MocapRobotPoseProvider* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
  int robotID= -1;
  int ballID = -1;
  Settings settings;

  /** The main function, called every cycle
  * @param mocapRobotPose The data struct to be filled
  */
  void update(RobotPose& robotPose);

  /** The main function, called every cycle
  * @param mocapRobotPose The data struct to be filled
  */
  void update(MocapRobotPose& mocapRobotPose);


  /** The main function, called every cycle
  * @param mocapBallModel The data struct to be filled
  */
  void update(MocapBallModel& mocapBallModel);

  /** Calculates the translation of the robots rigidbody 
  * @return (x,y) tranlsation of robot 
  */
  Vector2f calcTranslation(int ID);

  /** Calculates the rotation of the robots rigidbody 
  * @return rotation around z-axis of robot
  */
  Angle calcRotation();

  /** Calculates the validity of the measured robots rigidbody
  * @return validity
  */
  float calcValidity(int ID);

  /*** HELPER FUNCTIONS ***/
  void getRobotID();
  void getBallID();
  Vector3f FromQ2(Quat q1);
  Vector3f NormalizeAngles(Vector3f angles);
  float NormalizeAngle(float angle);

public:
  /** Constructor */
  MocapRobotPoseProvider();

  /** Destructor */
  ~MocapRobotPoseProvider();
};