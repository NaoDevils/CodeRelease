/**
 * @file Global.h
 * Declaration of a class that contains pointers to global data.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

// Only declare prototypes. Don't include anything here, because this
// file is included in many other files.
class AnnotationManager;
class OutMessage;
struct Settings;
class DebugRequestTable;
class DebugDataTable;
class StreamHandler;
class DrawingManager;
class DrawingManager3D;
class TimingManager;
class Blackboard;
class GlobalGuard;
class GlobalKeeper;
class RobotConsole;

#if defined(_DEBUG) && !defined(ASSERT)
#include <cassert>
#define ASSERT assert
#define ASSERT_DEFINED
#endif

/**
 * @class Global
 * A class that contains pointers to global data.
 */
struct Global
{
  AnnotationManager* annotationManager = nullptr;
  OutMessage* debugOut = nullptr;
  Settings* settings = nullptr;
  DebugRequestTable* debugRequestTable = nullptr;
  DebugDataTable* debugDataTable = nullptr;
  StreamHandler* streamHandler = nullptr;
  DrawingManager* drawingManager = nullptr;
  DrawingManager3D* drawingManager3D = nullptr;
  TimingManager* timingManager = nullptr;
  Blackboard* blackboard = nullptr;
#ifdef _DEBUG
  bool verify = true;
#endif

  bool operator==(const Global&) const = default;

private:
  static thread_local Global theInstance;

public:
  /**
   * The method returns if there is a valid reference to the process wide instance.
   * @return If there is an instance of the annotation manager in this process.
   */
  static bool hasAnnotationManager()
  {
    return theInstance.annotationManager;
  }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the annotation manager in this process.
   */
  static AnnotationManager& getAnnotationManager()
  {
    return *theInstance.annotationManager;
  }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the outgoing debug message queue in this process.
   */
  static OutMessage& getDebugOut()
  {
    return *theInstance.debugOut;
  }

  static bool hasDebugOut()
  {
    return theInstance.debugOut;
  }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the settings in this process.
   */
  static const Settings& getSettings()
  {
    return *theInstance.settings;
  }

  /**
   * The method returns whether the settings have already been instantiated.
   * @return Is it safe to use getSettings()?
   */
  static bool hasSettings()
  {
    return theInstance.settings;
  }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the debug request table in this process.
   */
  static DebugRequestTable& getDebugRequestTable()
  {
    return *theInstance.debugRequestTable;
  }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the debug data table in this process.
   */
  static DebugDataTable& getDebugDataTable()
  {
    return *theInstance.debugDataTable;
  }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the stream handler in this process.
   */
  static StreamHandler& getStreamHandler()
  {
    return *theInstance.streamHandler;
  }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the drawing manager in this process.
   */
  static DrawingManager& getDrawingManager()
  {
    return *theInstance.drawingManager;
  }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the 3-D drawing manager in this process.
   */
  static DrawingManager3D& getDrawingManager3D()
  {
    return *theInstance.drawingManager3D;
  }

  /**
   * The method returns a reference to the process wide instance.
   * @return the instance of the timing manager in this process.
   */
  static TimingManager& getTimingManager()
  {
    return *theInstance.timingManager;
  }

  /**
   * The method returns a reference to the process wide instance.
   * @return the instance of the blackboard in this process.
   */
  static Blackboard& getBlackboard()
  {
    return *theInstance.blackboard;
  }

  friend class GlobalGuard;
  friend class GlobalKeeper;
  friend class RobotConsole; // modifies settings received from robot in handleMessage()
};

/**
 * @brief This class sets global variables using the RAII pattern and saves previous values on the stack.
 */
class GlobalGuard
{
public:
  GlobalGuard(const Global& nextGlobal)
  {
    prevGlobal = Global::theInstance;

    const auto apply = [](auto& p, auto n)
    {
      if (n)
        p = n;
    };

    apply(Global::theInstance.annotationManager, nextGlobal.annotationManager);
    apply(Global::theInstance.debugOut, nextGlobal.debugOut);
    apply(Global::theInstance.settings, nextGlobal.settings);
    apply(Global::theInstance.debugRequestTable, nextGlobal.debugRequestTable);
    apply(Global::theInstance.debugDataTable, nextGlobal.debugDataTable);
    apply(Global::theInstance.streamHandler, nextGlobal.streamHandler);
    apply(Global::theInstance.drawingManager, nextGlobal.drawingManager);
    apply(Global::theInstance.drawingManager3D, nextGlobal.drawingManager3D);
    apply(Global::theInstance.timingManager, nextGlobal.timingManager);
    apply(Global::theInstance.blackboard, nextGlobal.blackboard);

#ifdef _DEBUG
    this->thread = &Global::theInstance;
    this->nextGlobal = Global::theInstance;
#endif
  }

  ~GlobalGuard()
  {
    // Ensures that this class is destroyed in the same thread (and order) in which it was previously created.
#ifdef _DEBUG
    ASSERT(prevGlobal.verify == Global::theInstance.verify);
    ASSERT(!prevGlobal.verify || Global::theInstance == nextGlobal);
    ASSERT(this->thread == &Global::theInstance);
#endif

    Global::theInstance = prevGlobal;
  }

private:
  Global prevGlobal;

#ifdef _DEBUG
  void* thread;
  Global nextGlobal;
#endif
};

/**
 * @brief This class keeps globals the same when constructing (without destructing) GlobalGuards.
 *
 * This is useful when managing Robot objects in main thread, because each Robot constructs a ProcessList
 * with multiple GlobalGuards that should not affect globals of the main thread.
 */
class GlobalKeeper
{
public:
  GlobalKeeper()
  {
    prevGlobal = Global::theInstance;
#ifdef _DEBUG
    ASSERT(Global::theInstance.verify);
    Global::theInstance.verify = false;
#endif
  }
  ~GlobalKeeper()
  {
#ifdef _DEBUG
    ASSERT(!Global::theInstance.verify);
    Global::theInstance.verify = true;
#endif
    Global::theInstance = prevGlobal;
  }

private:
  Global prevGlobal;
};

#ifdef ASSERT_DEFINED
#undef ASSERT
#endif
