/**
 * @file BehaviorControl2015.cpp
 * Implementation of a C-based state machine behavior control module.
 * @author Thomas Röfer
 * @author Tim Laue
 */

#include "Libraries.h"
#include "Tools/Streams/InStreams.h"
#ifdef __INTELLISENSE__
#define INTELLISENSE_PREFIX Behavior2015::Behavior::
#endif
#include "Tools/Cabsl.h" // include last, because macros might mix up other header files

namespace Behavior2015
{
  /**
   * The wrapper for the behavior options.
   */
  class Behavior : public Cabsl<Behavior>, public Libraries
  {
#include "Options.h"
  public:
    /**
     * Constructor.
     * @param base The blackboard configured for this module.
     */
    Behavior(const BehaviorControl2015Base& base, BehaviorOutput& behaviorOutput)
    : Cabsl<Behavior>(&behaviorOutput.theActivationGraph),
      Libraries(base, behaviorOutput) {}

    /**
     * Executes one behavior cycle.
     * @param roots A set of root options. They must be parameterless.
     */
    void execute(const std::vector<OptionInfos::Option>& roots)
    {
      beginFrame(theFrameInfo.time);
      preProcessLibraries();

      for(std::vector<Behavior::OptionInfos::Option>::const_iterator i = roots.begin(); i != roots.end(); ++i)
      Cabsl<Behavior>::execute(*i);

      postProcessLibraries();
      endFrame();
    }
  };
}

using namespace Behavior2015;

/**
 * @class BehaviorControl2015
 * A C-based state machine behavior control module.
 */
class BehaviorControl2015 : public BehaviorControl2015Base
{
  STREAMABLE(Parameters,
  {
    /** Helper for streaming a vector of enums that are defined in another class. */
    struct OptionInfos : Behavior::OptionInfos {typedef std::vector<Option> Options;},

    ((OptionInfos) Options) roots, /**< All options that function as behavior roots. */
  });

  void update(ActivationGraph& activationGraph)
  {
    Parameters p(parameters); // make a copy, to make "unchanged" work
    MODIFY("parameters:BehaviorControl2015", p);
    if(theFrameInfo.time)
    {
      theBehavior->execute(p.roots);
    }
  }

  /** Update the behavior output by copying it from the behavior */
  void update(BehaviorControlOutput& behaviorControlOutput) { behaviorControlOutput = theBehaviorControlOutput; }

  /** Update the behavior led request by copying it from the behavior */
  void update(BehaviorLEDRequest& behaviorLEDRequest) {behaviorLEDRequest = theBehaviorLEDRequest;}

  /** Updates the motion request by copying it from the behavior */
  void update(MotionRequest& motionRequest) { motionRequest = theMotionRequest;}

  /** Updates the arm motion request by copying it from the behavior */
  void update(HeadControlRequest& headControlRequest) { headControlRequest = theHeadControlRequest; }

  Parameters parameters; /**< The root options. */
  BehaviorControlOutput theBehaviorControlOutput;
  BehaviorLEDRequest theBehaviorLEDRequest;
  HeadControlRequest theHeadControlRequest;
  MotionRequest theMotionRequest;
  BehaviorOutput behaviorOutput; /**< References to the representations above. */
  Behavior* theBehavior; /**< The behavior with all options and libraries. */

public:
  BehaviorControl2015()
  : behaviorOutput(const_cast<ActivationGraph&>(theActivationGraph),
                 theBehaviorControlOutput,
                 theBehaviorLEDRequest,
                 theHeadControlRequest,
                 theMotionRequest),
    theBehavior(new Behavior(*this, behaviorOutput))
  {
    InMapFile stream("behaviorControl2015.cfg");
    ASSERT(stream.exists());
    stream >> parameters;
  }

  ~BehaviorControl2015() {delete theBehavior;}
};

MAKE_MODULE(BehaviorControl2015, behaviorControl)
