#pragma once

#include <vector>
#include <stack>
#include <typeinfo>
#include <unordered_map>
#include "Tools/Streams/OutStreams.h"

class StreamHandler;

In& operator>>(In& in, StreamHandler& streamHandler);
Out& operator<<(Out& out, const StreamHandler& streamHandler);
StreamHandler& operator<<(StreamHandler& a, const StreamHandler& b);

class ConsoleRoboCupCtrl;
class RobotConsole;
class DebugDataStreamer;
class Framework;
/**
 * singleton stream handler class
 */
class StreamHandler
{
private:
  /**
   * Default constructor.
   * No other instance of this class is allowed except the one accessible via getStreamHandler
   * therefore the constructor is private.
   */
  StreamHandler();

  /**
   * Copy constructor.
   * Copying instances of this class is not allowed
   * therefore the copy constructor is private.
   */
  StreamHandler(const StreamHandler&) {}

  /**
   * Only a process is allowed to create the instance.
   */
  friend class Process;
  friend class SubThread;

  struct RegisteringAttributes
  {
    short baseClass;
    bool registering;
    bool externalOperator;
  };

  typedef std::unordered_map<const char*, const char*> BasicTypeSpecification;
  BasicTypeSpecification basicTypeSpecification;

  typedef std::pair<std::string, const char*> TypeNamePair;
  typedef std::unordered_map<const char*, std::vector<TypeNamePair>> Specification;
  Specification specification;

  typedef std::unordered_map<const char*, std::vector<const char*>> EnumSpecification;
  EnumSpecification enumSpecification;

  typedef std::unordered_map<std::string, int> StringTable;
  StringTable stringTable;

  typedef std::pair<Specification::iterator, RegisteringAttributes> RegisteringEntry;
  typedef std::stack<RegisteringEntry> RegisteringEntryStack;
  RegisteringEntryStack registeringEntryStack;

  bool registering;
  bool registeringBase;

  const char* getString(const std::string& string);

public:
  void clear();
  void startRegistration(const char* name, bool registerWithExternalOperator);
  void registerBase() { registeringBase = true; }
  void finishRegistration();
  void registerWithSpecification(const char* name, const std::type_info& ti);
  void registerEnum(const std::type_info& ti, const char* (*fp)(int));

  /**
   * Check whether the specifications of two types are structurally identical,
   * so that the first type could read the data written by the second type.
   * The only deviation allowed is that the first type can define more enumeration
   * constants, because it would still be able to read all constants the second type
   * defined.
   * This only works if both specifications involved have been created from a stream,
   * i.e. all type names were demangled.
   * @param other The specification data used for the second type.
   * @param rawType The type the specification which will be searched in this object.
   * @param otherType The type the specification which will be searched in the other object.
   * @return Is the other type compatible with this type?
   */
  bool areSpecificationsForTypesCompatible(StreamHandler& other, std::string rawType, std::string otherType);

  OutBinarySize dummyStream;

private:
  friend In& operator>>(In&, StreamHandler&);
  friend Out& operator<<(Out&, const StreamHandler&);
  friend StreamHandler& operator<<(StreamHandler& a, const StreamHandler& b);
  friend class ConsoleRoboCupCtrl; // constructs a StreamHandler used when outside process contexts.
  friend class RobotConsole; // constructs a StreamHandler storing the information received.
  friend class Framework;
  friend class DebugDataStreamer; // needs access to internal data types.
  friend class LogPlayer; // constructs a StreamHandler to represent specification of logged data.
  friend class LogDataProvider; // constructs a StreamHandler to represent specification of logged data.
  friend int main(int, char*[]); // construct a default StreamHandler in main function
};
