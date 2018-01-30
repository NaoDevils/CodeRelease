/**
* @file Tools/ProcessFramework/MocapHandler.h
* The file declares a class for communicating mocap data to the robot
* @author Janine Hemmers
*/

#pragma once

#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Network/UdpComm.h"
#include <string>

#include "Representations/Infrastructure/MocapMessage.h"

//Necessary?
#define MAX_NUM_OF_MOCAP_MESSAGES  100
#define NAT_FRAMEOFDATA            7




// Wrapper class for SPL standard messages.
class MocapDataIn
{
public:
  MocapDataIn() { /*clear(); queue.setSize(MAX_NUM_OF_MOCAP_MESSAGES * sizeof(MocapMessage));*/messages.reserve(MAX_NUM_OF_MOCAP_MESSAGES);  }
  std::vector<MocapMessage> messages;
  //MessageQueue queue;
  //int currentPosition;
  void clear() { messages.clear(); }
  //bool isEmpty() { return messages.empty(); }
  //void handleAllMessages(MessageHandler& handler) { queue.handleAllMessages(handler); }
};

#define MOCAP_COMM \
  MocapDataIn theMocapReceiver; \
  MocapHandler theMocapHandler;

#define INIT_MOCAP_COMM \
  theMocapHandler(theMocapReceiver)

#define START_MOCAP_COMM \
  theMocapHandler.start(1511, "10.0.0.25");

#define RECEIVE_MOCAP_COMM \
  (void) theMocapHandler.receive()

//#define USE_MOCAP

/**
* @class MocapHandler
* A class for receiving mocap data.
*/
class MocapHandler
{
private:
  MocapDataIn& in; /**< Incoming debug data is stored here. */
  int port = 0; /**< The UDP port this handler is listening to. */
  UdpComm socket; /**< The socket used to communicate. */

public:
  /**
  * Constructor.
  * @param in Incoming mocap data is stored here.
  */
  MocapHandler(MocapDataIn& in);

  /**
  * The method starts the actual communication on the given port.
  * @param port The UDP port this handler is listening to.
  * @param subnet The subnet the handler is broadcasting to.
  */
  void start(int port, const char* subnet);

  /**
  * The method receives packages if available.
  * @return The number of bytes received.
  */
  unsigned receive();

  bool checkMessage(MocapMessage& msg);


};
