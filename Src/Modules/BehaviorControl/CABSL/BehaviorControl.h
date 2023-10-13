/**
 * @file BehaviorControl.h
 * Declaration of the base class of the C-based state machine behavior control module.
 * @author Thomas Röfer
 * @author Tim Laue
 */

#pragma once

#include "Libraries/HelperFunctions.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/BehaviorControl/BallSearch.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/GoalSymbols.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/BehaviorControl/KeySymbols.h"
#include "Representations/BehaviorControl/PSGoalieTrigger.h"
#include "Representations/BehaviorControl/RoleSelection.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/BackupBallchaser.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/BehaviorControl/RoleSymbols/BallchaserKeeper.h"
#include "Representations/BehaviorControl/RoleSymbols/Center.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderLeft.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderRight.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderSingle.h"
#include "Representations/BehaviorControl/RoleSymbols/Keeper.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningAndKickSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/Receiver.h"
#include "Representations/BehaviorControl/RoleSymbols/LeftWing.h"
#include "Representations/BehaviorControl/RoleSymbols/RightWing.h"
#include "Representations/BehaviorControl/RoleSymbols/ReplacementKeeper.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/MotionSettings.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CMCorrectorStatus.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/NetworkStatus.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/TeamCommSenderOutput.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/USBSettings.h"
#include "Representations/Infrastructure/USBStatus.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/DangerMap.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/WhistleDortmund.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/MotionState.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Perception/CLIPGoalPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Sensing/ArmContact.h"
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

MODULE(BehaviorControl,
  REQUIRES(BallChaserDecision),
  REQUIRES(BallModel),
  REQUIRES(BallModelAfterPreview),
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorConfiguration),
  REQUIRES(DangerMap),
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GoalSymbols),
  REQUIRES(GameSymbols),
  REQUIRES(KeyStates),
  REQUIRES(KeySymbols),
  REQUIRES(Ballchaser),
  REQUIRES(BallchaserKeeper),
  REQUIRES(Keeper),
  REQUIRES(ReplacementKeeper),
  REQUIRES(BackupBallchaser),
  REQUIRES(Center),
  REQUIRES(DefenderLeft),
  REQUIRES(DefenderRight),
  REQUIRES(DefenderSingle),
  REQUIRES(Receiver),
  REQUIRES(LeftWing),
  REQUIRES(RightWing),
  REQUIRES(KickEngineOutput),
  REQUIRES(MotionInfo),
  REQUIRES(MotionState),
  REQUIRES(MotionSelection),
  REQUIRES(MotionSettings),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(PositioningSymbols),
  REQUIRES(PositioningAndKickSymbols),
  REQUIRES(RawGameInfo),
  REQUIRES(RemoteBallModel),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotMap),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(RoleSymbols),
  REQUIRES(RoleSelection),
  REQUIRES(SideConfidence),
  REQUIRES(TacticSymbols),
  REQUIRES(TeammateData),
  REQUIRES(SpeedInfo),
  REQUIRES(SpecialActionsOutput),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(WalkingEngineParams),
  REQUIRES(WalkCalibration),
  REQUIRES(ArmContact),
  REQUIRES(PSGoalieTrigger),
  REQUIRES(BallSearch),
  REQUIRES(WhistleDortmund),
  REQUIRES(CMCorrectorStatus),
  REQUIRES(ActivationGraph),
  REQUIRES(USBSettings),
  REQUIRES(USBStatus),
  REQUIRES(NetworkStatus),
  USES(TeamCommSenderOutput),
  PROVIDES(ActivationGraph),
  PROVIDES(BehaviorData),
  PROVIDES(BehaviorLEDRequest),
  PROVIDES(HeadControlRequest),
  PROVIDES(MotionRequest)
);

namespace NDBehavior
{
  /**
   * Container for references to representations modified by the behavior.
   */
  class BehaviorOutput
  {
  public:
    ActivationGraph& theActivationGraph;
    BehaviorData& theBehaviorData;
    BehaviorLEDRequest& theBehaviorLEDRequest;
    HeadControlRequest& theHeadControlRequest;
    MotionRequest& theMotionRequest;

    BehaviorOutput(ActivationGraph& theActivationGraph, BehaviorData& theBehaviorData, BehaviorLEDRequest& theBehaviorLEDRequest, HeadControlRequest& theHeadControlRequest, MotionRequest& theMotionRequest)
        : theActivationGraph(theActivationGraph), theBehaviorData(theBehaviorData), theBehaviorLEDRequest(theBehaviorLEDRequest), theHeadControlRequest(theHeadControlRequest),
          theMotionRequest(theMotionRequest)
    {
    }
  };

  /**
   * Common base class for behavior options and libraries.
   */
  class BehaviorBase : public BehaviorControlBase, public BehaviorOutput
  {
  private:
    void update(ActivationGraph&) {}
    void update(BehaviorData&) {}
    void update(BehaviorLEDRequest&) {}
    void update(HeadControlRequest&) {}
    void update(MotionRequest&) {}

  public:
    using BehaviorOutput::theActivationGraph; /**< Use the non-const version. */

    /**
     * Constructor.
     * Note that the constructor uses the default constructor of the base class.
     * @param base The behavior base class some attributes are copied from.
     * @param behaviorOutput The data modified by the behavior.
     */
    BehaviorBase(const BehaviorControlBase& base, BehaviorOutput& behaviorOutput) : BehaviorOutput(behaviorOutput) {}

    /**
     * Copy constructor.
     * Note that the constructor uses the default constructor of the base class, because
     * the attributes of the base class should not be copied.
     * @param other The object that is copied.
     */
    BehaviorBase(const BehaviorBase& other) : BehaviorOutput(other) {}

    /**
     * Assignment operator, because the standard operator is not accepted by the compiler.
     * @param other The instance that is cloned.
     */
    void operator=(const BehaviorBase& other);
  };
} // namespace NDBehavior
