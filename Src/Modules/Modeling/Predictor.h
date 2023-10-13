/**
* @file Predictor.h
* Declares a class that predicts RobotPose etc. based on the knowledge
* how we have moved after execution of the current motion (ie. preview while walking).
**
* @author <a href="mailto:Stefan.Czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
*/

#ifndef _Predictor_H_
#define _Predictor_H_

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/MotionInfo.h"

MODULE(Predictor,
  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(MultipleBallModel),
  PROVIDES(RobotPoseAfterPreview),
  PROVIDES(BallModelAfterPreview),
  PROVIDES(MultipleBallModelAfterPreview),
  HAS_PREEXECUTION
);


/**
* @class PosePredictor
*/
class Predictor : public PredictorBase
{
public:
  /** 
  * The method provides the robot pose after preview.
  * @param robotPoseAfterPreview The robot pose representation that is updated by this module.
  */
  void update(RobotPoseAfterPreview& robotPoseAfterPreview);

  /** 
  * The method provides the ball model after preview.
  * @param ballModelAfterPreview The ball model representation that is updated by this module.
  */
  void update(BallModelAfterPreview& ballModelAfterPreview);

  /**
  * The method provides the multiple ball model after preview.
  * @param multipleBallModelAfterPreview The multiple ball model representation that is updated by this module.
  */
  void update(MultipleBallModelAfterPreview& multipleBallModelAfterPreview);

  /*
  * calculate the corrected odometry offset for preview only once
  */
  void execute(tf::Subflow&);

  Pose2f correctedOdometryOffset;
  RobotPoseAfterPreview robotPoseAfterPreview;
};

#endif
