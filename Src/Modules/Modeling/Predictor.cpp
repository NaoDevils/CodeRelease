/**
* @file Predictor.cpp
* Implements a class that provides the robot pose as it will be after execution of the walking preview.
**
* @author <a href="mailto:Stefan.Czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
*/

#include "Predictor.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/DebugDrawings.h"

/** 
* The method provides the robot pose after preview.
* @param robotPoseAfterPreview The robot pose representation that is updated by this module.
*/
void Predictor::update(RobotPoseAfterPreview& robotPoseAfterPreview)
{
  robotPoseAfterPreview = this->robotPoseAfterPreview;
  ASSERT(robotPoseAfterPreview.translation.x() == robotPoseAfterPreview.translation.x());
  ASSERT(robotPoseAfterPreview.translation.y() == robotPoseAfterPreview.translation.y());
  ASSERT(robotPoseAfterPreview.rotation == robotPoseAfterPreview.rotation);
}

/** 
* The method provides the ball model after preview.
* @param ballModelAfterPreview The ball model representation that is updated by this module.
*/
void Predictor::update(BallModelAfterPreview& ballModelAfterPreview)
{
  static_cast<BallModel&>(ballModelAfterPreview) = theBallModel;

  Vector2f position, velocity;

  position = Transformation::robotToField(theRobotPose, theBallModel.estimate.position);
  velocity = theBallModel.estimate.getVelocityInFieldCoordinates(theRobotPose);
  ballModelAfterPreview.estimate.setPositionAndVelocityFromFieldCoordinates(position, velocity, (RobotPose&)robotPoseAfterPreview);

  position = Transformation::robotToField(theRobotPose, theBallModel.lastPerception);
  ballModelAfterPreview.lastPerception = Transformation::fieldToRobot(robotPoseAfterPreview, position);
}

/**
* The method provides the multiple ball model after preview.
* @param multipleBallModelAfterPreview The multiple ball model representation that is updated by this module.
*/
void Predictor::update(MultipleBallModelAfterPreview& multipleBallModelAfterPreview)
{
  multipleBallModelAfterPreview.ballModels.clear();

  for (const BallModel& bm : theMultipleBallModel.ballModels)
  {
    BallModel lbm = bm;
    Vector2f position, velocity;

    position = Transformation::robotToField(theRobotPose, bm.estimate.position);
    velocity = bm.estimate.getVelocityInFieldCoordinates(theRobotPose);
    lbm.estimate.setPositionAndVelocityFromFieldCoordinates(position, velocity, (RobotPose&)robotPoseAfterPreview);

    position = Transformation::robotToField(theRobotPose, bm.lastPerception);
    lbm.lastPerception = Transformation::fieldToRobot(robotPoseAfterPreview, position);
    multipleBallModelAfterPreview.ballModels.push_back(lbm);
  }
}


void Predictor::execute(tf::Subflow& subflow)
{
  correctedOdometryOffset = theMotionInfo.offsetToRobotPoseAfterPreview;
  static_cast<RobotPose&>(robotPoseAfterPreview) = theRobotPose;
  static_cast<Pose2f&>(robotPoseAfterPreview) += correctedOdometryOffset;

  PLOT("module:Predictor:robotPose.x", theRobotPose.translation.x());
  PLOT("module:Predictor:robotPose.y", theRobotPose.translation.y());
  PLOT("module:Predictor:robotPose.r", theRobotPose.rotation);
  PLOT("module:Predictor:robotPoseAfterPreview.x", robotPoseAfterPreview.translation.x());
  PLOT("module:Predictor:robotPoseAfterPreview.y", robotPoseAfterPreview.translation.y());
  PLOT("module:Predictor:robotPoseAfterPreview.r", robotPoseAfterPreview.rotation);
}


MAKE_MODULE(Predictor, modeling)
