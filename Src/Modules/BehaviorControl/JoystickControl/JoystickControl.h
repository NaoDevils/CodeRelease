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


/**
 * \class JoystickState
 * 
 * This class stores pressed buttons until they are processed and 
 * the current state of all axes.
 */
class JoystickState {
public:
  /**
   * Creates a new \c JoystickState object with no buttons and no axes.
   */
  JoystickState() :
    pressedButtons(std::vector<bool>(0)),
    axes(std::vector<float>(0)) {}

  /**
   * Creates a new \c JoystickState object with the given button / axes count.
   */
  JoystickState(unsigned int buttonsCount, unsigned int axesCount) :
    pressedButtons(std::vector<bool>(buttonsCount)),
    axes(std::vector<float>(axesCount))
  {
    // Initialize empty arrays.
    for (unsigned int i = 0; i < pressedButtons.size(); i++)
      pressedButtons[i] = false;
    for (unsigned int i = 0; i < axes.size(); i++)
      axes[i] = 0.f;
  }

  /**
   * Checks whether at least one button was recently pressed (\c pressedButtons
   * contains at least one entry with value \c true).
   */
  bool pressedButtonAvailable()
  {
    for (size_t b = 0; b < pressedButtons.size(); b++)
    {
      if (pressedButtons[b]) return true;
    }
    return false;
  }
  /**
   * Delete all pressed button events and resets all axis values to 0.
   */
  void deleteEvents(const JoystickControlParameters &parameters)
  {
    // Reinitialize all buttons with false.
    for (size_t b = 0; b < pressedButtons.size(); b++)
      pressedButtons[b] = false;

    // Reinitialize all axes with 0.
    for (size_t a = 0; a < axes.size(); a++)
      axes[a] = 0.f;
    // Reinitialize split axes with -1.
    if (parameters.device.walkRotAxisSplit())
    {
      axes[parameters.device.walkRotLeftAxis] = -1.f;
      axes[parameters.device.walkRotRightAxis] = -1.f;
    }
  }

public:
  /// Contains a bool value for each button.
  /// pressedButtons(i) = true: Event 'Button i pressed' was not processed yet.
  std::vector<bool> pressedButtons;
  /// Stores the values of all axes (range [-1.f, 1.f]).
  std::vector<float> axes;
};


class JoystickControl : public JoystickControlBase
{
public:
  /**
   * Default constructor.
   * Invokes method \c initialize().
   */
  JoystickControl() : 
    lastExecuteTimeStamp(0)
  {
    initialize(); 
  }
  /**
   * Destructor.
   */
  ~JoystickControl() {}

  /**
   * The update method to provide the representation ArmContact. This is done 
   * to prevent moving arms in front of obstacles.
   */
  void update(ArmContact& armContact)
  {
    armContact.armContactStateLeft = ArmContact::None;
    armContact.armContactStateRight = ArmContact::None;
    armContact.timeStampLeft = 0;
    armContact.timeStampRight = 0;
  }

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
   * Runs all neccessary methods to update the provided representations.
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
   * \return true if any action is running yet.
   * \see generateActionRequest()
   */
  bool actionRunning();

  /**
   * Process joystick axes to let the robot walk.
   */
  void generateWalkRequest();


  // --- Actions ---

  /**
   * Set localMotionRequest to walk at speed zero.
   */
  void stopMotion();

  /**
   * Run the given special action.
   * \param id Choose the special action.
   * \param mirror Should the action be mirrored?
   * \param duration The duration time of the given special action.
   */
  void specialAction(SpecialActionRequest::SpecialActionID id, bool mirror, uint64_t duration);
  /**
   * Run the given special action.
   * \param specialAction Choose the special action including option to mirror.
   * \param duration The duration time of the given special action.
   */
  void specialAction(SpecialActionRequest &specialAction, uint64_t duration);
  /**
   * Run the special action 'playDead'.
   */
  void playDead();
  /**
   * Run the special action 'stand'.
   */
  void stand() { specialAction(SpecialActionRequest::stand, false, 5000); }
  /**
   * The robot stands up from lying or sitting.
   */
  void standUpFallen();
  /**
   * Run the special action 'sitDown'.
   */
  void sitDown() { specialAction(SpecialActionRequest::sitDown, false, 3000); }

  /**
   * Execute a kick action.
   * \param kickLeft Kick with left foot if true, else kick with right foot.
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
  uint64_t lastExecuteTimeStamp;


  /// true, if the robot stands.
  /// Only standing the robot is able to do all the other actions.
  bool m_standing;

  // True while the execution of a button event endures.
  bool m_actionRunning;
  /// Time stamp when the execution of a button event will be finished.
  uint64_t m_actionEndTime;
};
