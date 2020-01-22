/**
 * @file BehaviorControl2015.h
 * Declaration of the base class of the C-based state machine behavior control module.
 * @author Thomas Röfer
 * @author Tim Laue
 */

#pragma once

#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/CLIPGoalPercept.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Module/Module.h"

#include <algorithm>
#include <limits>
#include <sstream>

MODULE(BehaviorControl2015,
{,
  REQUIRES(BallModel),
  REQUIRES(BallModelAfterPreview),
  REQUIRES(BehaviorData),
  REQUIRES(CameraCalibration),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(HeadJointRequest),
  REQUIRES(HeadLimits),
  REQUIRES(Image),
  REQUIRES(JointAngles),
  REQUIRES(JointRequest),
  REQUIRES(KeyStates),
  REQUIRES(KickEngineOutput),
  REQUIRES(MotionInfo),
  REQUIRES(MotionSelection),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotMap),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(SideConfidence),
  REQUIRES(TeamBallModel),
  REQUIRES(TeammateData),
  REQUIRES(TorsoMatrix), // Required for kicks
  REQUIRES(WalkingEngineOutput),
  REQUIRES(ActivationGraph),
  PROVIDES(ActivationGraph),
  PROVIDES(BehaviorControlOutput),
  PROVIDES(BehaviorLEDRequest),
  PROVIDES(HeadControlRequest),
  PROVIDES(MotionRequest),
});

namespace Behavior2015
{
  /**
   * Container for references to representations modified by the behavior.
   */
  class BehaviorOutput
  {
  public:
    ActivationGraph& theActivationGraph;
    BehaviorControlOutput& theBehaviorControlOutput;
    BehaviorLEDRequest& theBehaviorLEDRequest;
    HeadControlRequest& theHeadControlRequest;
    MotionRequest& theMotionRequest;

    BehaviorOutput(ActivationGraph& theActivationGraph,
                 BehaviorControlOutput& theBehaviorControlOutput,
                 BehaviorLEDRequest& theBehaviorLEDRequest,
                 HeadControlRequest& theHeadControlRequest,
                 MotionRequest& theMotionRequest)
    : theActivationGraph(theActivationGraph),
      theBehaviorControlOutput(theBehaviorControlOutput),
      theBehaviorLEDRequest(theBehaviorLEDRequest),
      theHeadControlRequest(theHeadControlRequest),
      theMotionRequest(theMotionRequest){}
  };

  /**
   * Common base class for behavior options and libraries.
   */
  class BehaviorBase : public BehaviorControl2015Base, public BehaviorOutput
  {
  private:
    void update(ActivationGraph&) {}
    void update(BehaviorControlOutput&) {}
    void update(BehaviorLEDRequest&) {}
    void update(HeadControlRequest&) {}
    void update(MotionRequest&) {}

  public:
    using BehaviorOutput::theActivationGraph; /**< Use the non-const version. */
    bool firstReady;

    /**
     * Constructor.
     * Note that the constructor uses the default constructor of the base class.
     * @param base The behavior base class some attributes are copied from.
     * @param behaviorData The data modified by the behavior.
     */
    BehaviorBase(const BehaviorControl2015Base& base,
      BehaviorOutput& behaviorOutput)
    : BehaviorOutput(behaviorOutput) {}

    /**
     * Copy constructor.
     * Note that the constructor uses the default constructor of the base class, because
     * the attributes of the base class should not be copied.
     * @param other The object that is copied.
     */
    BehaviorBase(const BehaviorBase& other)
    : BehaviorOutput(other) {}
  };
}
