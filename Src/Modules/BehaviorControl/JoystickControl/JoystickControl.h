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
// REQUIRES
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/BehaviorControl/JoystickControl/JoystickState.h"
#include "Representations/BehaviorControl/JoystickControl/JoystickControlParameters.h"
#include "Representations/BehaviorControl/JoystickControl/JoystickDeviceParameters.h"

// PROVIDES
#include "Representations/Sensing/ArmContact.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/MotionControl/MotionRequest.h"


// Send debug messages for received joystick events.
//#define TEXT_OUTPUT


MODULE(JoystickControl,
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(RobotInfo),
  USES(KickEngineOutput),

  REQUIRES(JoystickState),
  REQUIRES(JoystickControlParameters),
  REQUIRES(JoystickDeviceParameters),

  PROVIDES(ArmContact), // Provide ArmContact to prevent moving arms in front of obstacles.
  PROVIDES(HeadControlRequest),
  PROVIDES(MotionRequest),
  HAS_PREEXECUTION,
  LOADS_PARAMETERS(
    ,
    (unsigned)(10000) continueWalkingTime,
    (unsigned)(3500) standTime,
    (unsigned)(2500) sitDownTime,
    (unsigned)(5000) standUpBackTime,
    (unsigned)(2800) standUpFrontTime,
    (unsigned)(2000) kickTime
  )
);


class JoystickControl : public JoystickControlBase
{
public:
  /**
   * Default constructor.
   * Invokes method \c initialize().
   */
  JoystickControl();

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
  void execute(tf::Subflow&);

  /**
   * Process the state of the chest button. Stand up / sit down when
   * chest button is pressed.
   */
  void processStartButton();

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

  /**
   * Checks if a button or axis is set in the config
   */
  void checkButtonOrAxisAvailability();

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
  void specialAction(const SpecialActionRequest& specialAction, unsigned duration);

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
  /// The \c HeadControlequest is saved in this class member.
  HeadControlRequest localHeadControlRequest;

  /// The \c MotionRequest is saved in this class member.
  MotionRequest localMotionRequest;

  /// true, if the robot stands.
  /// Only standing the robot is able to do all the other actions.
  bool m_standing;

  /// True while the execution of a button event endures.
  bool m_actionRunning;

  /// True while walkin on spot for testing
  bool m_walkOnSpot;

  /// Time stamp when the execution of a button event will be finished.
  unsigned m_actionEndTime;

  /// Time stamp when last movement happend.
  unsigned int m_timestampOfLastWalkMovement = 0;

  /// TODO
  unsigned int startOfCustomStep = 0;

  /// TODO
  WalkRequest::StepRequest currentExecutedCustomStep = WalkRequest::StepRequest::none;

  /// TODO
  bool currentExecutedCustomStepMirror;

  bool startButtonAvailable, headPanAxisAvailable, headTiltAxisAvailable, backButtonAvailable, rightKickButtonAvailable, leftKickButtonAvailable;
  std::vector<bool> specialActionsButtonAvailable;
  bool walkXAxisAvailable, additionalWalkXAxisAvailable, walkYAxisAvailable, walkRotAxisAvailable, customStepXAxisAvailable, customStepYAxisAvailable;
};
