/**
 * @file BehaviorControl.cpp
 * Implementation of a C-based state machine behavior control module.
 * @author Thomas Röfer
 * @author Tim Laue
 */

#include "BehaviorControl.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/Annotation.h"
#include "Modules/BehaviorControl/BehaviorHelper.h"

#ifdef __INTELLISENSE__
#define INTELLISENSE_PREFIX NDBehavior::Behavior::
#endif
#include "Tools/Cabsl.h" // include last, because macros might mix up other header files

namespace NDBehavior
{
  /**
   * The wrapper for the behavior options.
   */
  class Behavior : public Cabsl<Behavior>, public BehaviorBase
  {
#include "Options.h"
  public:
    bool didCheeringOnce = false;
    bool mirrorInPenaltyShootout = false;
    unsigned timeStampLastWalkKickExecution = 0;
    bool gotoFieldCoordinatesFinished = false;
    unsigned timeStampLastLongKickExecution = 0;
    unsigned timeStampLastDribbleExecution = 0;
    uint8_t lastGameState = 0;
    /**
     * Constructor.
     * @param base The blackboard configured for this module.
     */
    Behavior(const BehaviorControlBase& base, BehaviorOutput& behaviorOutput) : Cabsl<Behavior>(&behaviorOutput.theActivationGraph), BehaviorBase(base, behaviorOutput) {}

    /**
     * Executes one behavior cycle.
     * @param roots A set of root options. They must be parameterless.
     */
    void execute(const std::vector<OptionInfos::Option>& roots)
    {
      beginFrame(theFrameInfo.time);

      for (std::vector<Behavior::OptionInfos::Option>::const_iterator i = roots.begin(); i != roots.end(); ++i)
        Cabsl<Behavior>::execute(*i);

      endFrame();
    }

    void Positioning(bool preciseArrival = true, bool ballchaser = false)
    {
      if (ballchaser)
      {
        GoToFieldCoordinates(thePositioningAndKickSymbols.optPosition,
            thePositioningAndKickSymbols.thresholdXFront,
            thePositioningAndKickSymbols.thresholdXBack,
            thePositioningAndKickSymbols.thresholdY,
            thePositioningAndKickSymbols.thresholdRotation,
            thePositioningAndKickSymbols.stopAtTarget,
            thePositioningAndKickSymbols.previewArrival,
            preciseArrival);
      }
      else
      {
        GoToFieldCoordinates(thePositioningSymbols.optPosition,
            thePositioningSymbols.thresholdXFront,
            thePositioningSymbols.thresholdXBack,
            thePositioningSymbols.thresholdY,
            thePositioningSymbols.thresholdRotation,
            thePositioningSymbols.stopAtTarget,
            thePositioningSymbols.previewArrival,
            preciseArrival);
      }
    }
  };

  void BehaviorBase::operator=(const BehaviorBase& other)
  {
    memcpy((void*)this, (void*)&other, sizeof(*this));
  }
} // namespace NDBehavior

using namespace NDBehavior;

/**
 * @class BehaviorControl
 * A C-based state machine behavior control module.
 */
class BehaviorControl : public BehaviorControlBase
{
  STREAMABLE(Parameters,
    /** Helper for streaming a vector of enums that are defined in another class. */
    struct OptionInfos : Behavior::OptionInfos { typedef std::vector<Option> Options;},

    ((OptionInfos) Options) roots /**< All options that function as behavior roots. */
  );

  void update(ActivationGraph& activationGraph)
  {
    Parameters p(parameters); // make a copy, to make "unchanged" work
    MODIFY("parameters:BehaviorControl", p);
    if (theFrameInfo.time)
    {
      theBehavior->execute(p.roots);
    }
  }

  /** Update the behavior output by copying it from the behavior */
  void update(BehaviorData& behaviorData)
  {
    behaviorData = theBehaviorData;
    // fill part of behaviorData coming from other providers
    behaviorData.role = theRoleSymbols.role;
    behaviorData.lastRole = theRoleSymbols.lastRole;
    behaviorData.roleSuggestions = theRoleSymbols.roleSuggestions;
    behaviorData.playerNumberToBall = static_cast<unsigned char>(theBallChaserDecision.playerNumberToBall);
    behaviorData.ownTimeToBall = theBallChaserDecision.ownTimeToBall;
    behaviorData.ballPositionField = theBallSymbols.ballPositionField;
    behaviorData.ballPositionFieldPredicted = theBallSymbols.ballPositionFieldPredicted;
    behaviorData.ballPositionRelative = theBallSymbols.ballPositionRelativeWOPreview;
    behaviorData.timeSinceBallWasSeen = theBallSymbols.timeSinceLastSeen;
    behaviorData.kickTarget = theMotionRequest.kickRequest.kickTarget;

    if (behaviorData.role != BehaviorData::RoleAssignment::noRole && roleFromLastFrame != BehaviorData::RoleAssignment::noRole && roleFromLastFrame != behaviorData.role)
      ANNOTATION("RoleSymbols", "Role changed from " << BehaviorData::getName(roleFromLastFrame) << " to " << BehaviorData::getName(behaviorData.role));
    roleFromLastFrame = behaviorData.role;

    if (behaviorData.behaviorState >= BehaviorData::BehaviorState::firstCalibrationState && behaviorData.behaviorState != lastBehaviorState)
      ANNOTATION("GameInfo", "Calibration (" << BehaviorData::getName(behaviorData.behaviorState) << ") state.");
    lastBehaviorState = behaviorData.behaviorState;

    if (behaviorData.soccerState != lastSoccerState)
      ANNOTATION("BehaviorData", "SoccerState changed to " << BehaviorData::getName(behaviorData.soccerState));
    lastSoccerState = behaviorData.soccerState;
  }

  /** Updates the motion request by copying it from the behavior */
  void update(MotionRequest& motionRequest) { motionRequest = theMotionRequest; }

  /** Updates the arm motion request by copying it from the behavior */
  void update(HeadControlRequest& headControlRequest) { headControlRequest = theHeadControlRequest; }

  Parameters parameters; /**< The root options. */
  BehaviorData theBehaviorData;
  HeadControlRequest theHeadControlRequest;
  MotionRequest theMotionRequest;
  BehaviorOutput behaviorOutput; /**< References to the representations above. */
  Behavior* theBehavior; /**< The behavior with all options and libraries. */

  BehaviorData::BehaviorState lastBehaviorState;
  BehaviorData::SoccerState lastSoccerState;
  BehaviorData::RoleAssignment roleFromLastFrame = BehaviorData::RoleAssignment::noRole;

public:
  BehaviorControl()
      : behaviorOutput(const_cast<ActivationGraph&>(theActivationGraph), theBehaviorData, theHeadControlRequest, theMotionRequest),
        theBehavior(new Behavior(*this, behaviorOutput)), lastBehaviorState(BehaviorData::BehaviorState::frameworkInactive), lastSoccerState(BehaviorData::SoccerState::penalized)
  {
    InMapFile stream("behaviorControl.cfg");
    ASSERT(stream.exists());
    stream >> parameters;
  }

  ~BehaviorControl() { delete theBehavior; }
};

MAKE_MODULE(BehaviorControl, behaviorControl)
