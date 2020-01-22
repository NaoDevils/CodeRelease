/**
 * \file JoystickControl.h
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 *
 * This file declares a module that creates MotionRequests from a joystick input.
 */
#pragma once

#include <climits>
#include <vector>

#include "Tools/Module/Module.h"
#include "Tools/Math/Eigen.h"
#include "Representations/MotionControl/SpecialActionRequest.h"
// REQUIRES
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/KickEngineOutput.h"
// PROVIDES
#include "Representations/Sensing/ArmContact.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/MotionControl/MotionRequest.h"

#include "JoystickControlParameters.h"
#include "JoystickLib.h"
#include "JoystickState.h"


// Send debug messages for received joystick events.
//#define TEXT_OUTPUT


MODULE(JoystickControl,
{,
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(KeyStates),
  USES(KickEngineOutput),

  PROVIDES(ArmContact), // Provide ArmContact to prevent moving arms in front of obstacles.
  PROVIDES(HeadControlRequest),
  PROVIDES(MotionRequest),
});


class JoystickControl : public JoystickControlBase
{
public:
  /**
   * Default constructor.
   * Invokes method \c initialize().
   */
  JoystickControl();

  /**
   * Destructor.
   */
  ~JoystickControl();

  /**
   * The update method to provide the representation ArmContact. This is done 
   * to prevent moving arms in front of obstacles.
   */
  void update(ArmContact& armContact);

  /**
  * The update method to send a head control request.
  */
  void update(HeadControlRequest& headControlRequest);

  /**
   * The update method to send a motion request.
   */
  void update(MotionRequest& motionRequest);

private:
  /**
   * Initializes this module.
   * Starts listening for joystick actions.
   */
  void initialize();

  /**
   * Main entry point for execution, runs all neccessary methods to update the provided representations.
   */
  void execute();

  /**
   * Reads all pending joystick events.
   */
  void parseJoystickEvents();

  /**
   * Process the state of the chest button. Stand up / sit down when
   * chest button is pressed.
   */
  void processChestButton();

  /**
  * Process current joystick events and set \c localHeadControlRequest.
  */
  void generateHeadControlRequest();

  /**
   * Process current joystick events and set \c localMotionRequest.
   */
  void generateMotionRequest();
  
  /**
   * Process joystick button events to run special actions or kicks.
   */
  void generateActionRequest();

  /**
   * Checks whether the robot currently stands. The robot is only able to do all 
   * the other actions, if it is standing.
   * If it is fallen, a stand up action is started.
   */
  bool standing();
  /**
   * Checks whether a started action (see \c generateActionRequest()) 
   * is currently running. While this is true, no other action or walk 
   * request should be sent.
   * @return true if any action is running yet.
   * \see generateActionRequest()
   */
  bool actionRunning();

  /**
   * Process joystick axes to let the robot walk.
   */
  void generateWalkRequest();

  /**
   * Process arrow keys to trigger a choosen CustomStep Execution
   */
  void generateCustomStep();


  // --- Actions ---

  /**
   * Set localMotionRequest to walk at speed zero.
   */
  void stopMotion();

  /**
   * Run the given special action.
   * @param id Choose the special action.
   * @param mirror Should the action be mirrored?
   * @param duration The duration time of the given special action.
   */
  void specialAction(SpecialActionRequest::SpecialActionID id, bool mirror, unsigned duration);

  /**
   * Run the given special action.
   * @param specialAction Choose the special action including option to mirror.
   * @param duration The duration time of the given special action.
   */
  void specialAction(SpecialActionRequest &specialAction, unsigned duration);

  /**
   * Run the special action 'playDead'.
   */
  void playDead();

  /**
   * Run the special action 'stand'.
   */
  void stand();

  /**
   * Run the special action 'sitDown'.
   */
  void sitDown();

  /**
   * The robot stands up from lying or sitting.
   */
  void standUpFallen();

  /**
   * Execute a kick action.
   * @param kickLeft Kick with left foot if true, else kick with right foot.
   */
  void kick(bool kickLeft);

private:
  /// Parameters loaded from locations config file "joystickControl.cfg".
  JoystickControlParameters parameters;

  /// Joystick object is used to receive joystick events.
  JoystickLib joystick;
  /// The current state (after execution of \c parseJoystickEvents) of 
  /// all buttons and axes of the joystick.
  JoystickState state;

  /// The \c HeadControlequest is saved in this class member.
  HeadControlRequest localHeadControlRequest;

  /// The \c MotionRequest is saved in this class member.
  MotionRequest localMotionRequest;

  /// Time stamp of last execution of the method \c executeCommonCode().
  /// Used to execute it only once per frame.
  unsigned lastExecuteTimeStamp;

  /// true, if the robot stands.
  /// Only standing the robot is able to do all the other actions.
  bool m_standing;

  // True while the execution of a button event endures.
  bool m_actionRunning;

  // True while walkin on spot for testing CustomStep
  bool m_inCustomStepTest;

  /// Time stamp when the execution of a button event will be finished.
  unsigned m_actionEndTime;

  /// TODO
  unsigned int startOfCustomStep = 0;

  /// TODO
  WalkRequest::StepRequest currentExecutedCustomStep = WalkRequest::StepRequest::none;

  /// TODO
  bool currentExecutedCustomStepMirror = false;
};
