#pragma once

#include <string>

#ifdef LINUX
#include <linux/joystick.h>

/**
 * Encapsulates all data relevant to a sampled joystick event.
 */
class JoystickEvent : public js_event
{
public:
  /**
   * Returns true if this event is the result of a button press.
   */
  bool isButton() { return (type & JS_EVENT_BUTTON) != 0; }

  /**
   * Returns true if this event is the result of an axis movement.
   */
  bool isAxis() { return (type & JS_EVENT_AXIS) != 0; }

  /**
   * Returns true if this event is part of the initial state obtained when
   * the joystick is first connected to.
   */
  bool isInitialState() { return (type & JS_EVENT_INIT) != 0; }
};
#else
class JoystickEvent;
#endif

/**
 * Represents a joystick device. Allows data to be sampled from it.
 */
class JoystickLib
{
public:
  ~JoystickLib();

  /**
   * Creates new joystick object without connection.
   */
  JoystickLib();

  /**
   * Initialises an instance for the joystick with the specified,
   * zero-indexed number.
   */
  JoystickLib(int joystickNumber);

  /**
   * Initialises an instance for the joystick device specified.
   */
  JoystickLib(std::string devicePath);

  /**
   * Establishes a connection for the first joystick: /dev/input/js0
   * If already a joystick is connected, the old connection is closed.
   * \returns true, if the connection was successfully established.
   */
  bool connect();

  /**
   * Closes the connection.
   */
  void closeJoystick();

  /**
   * Returns true if the joystick was found and may be used, otherwise false.
   */
  bool isFound();

  /**
   * Attempts to populate the provided JoystickEvent instance with data
   * from the joystick. Returns true if data is available, otherwise false.
   */
  bool sample(JoystickEvent* event);

private:
  /**
   * Open connection to the given file descriptor.
   */
  void openPath(std::string devicePath);

#ifdef LINUX
  /// File descriptor
  int m_fd = 0;
#endif
};
