/**
* @file Tools/ProcessFramework/MocapHandler.cpp
* The file declares a class for communicating mocap data to the robot
* @author Janine Hemmers
*/

#include "MocapHandler.h"

#include "Platform/SystemCall.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Settings.h"


MocapHandler::MocapHandler(MocapDataIn& in) :
  in(in)
{}

void MocapHandler::start(int port, const char* subnet)
{
  ASSERT(!this->port);
  this->port = port;

  socket.setBlocking(false);
  //VERIFY(socket.setBroadcast(true));
  VERIFY(socket.bind("0.0.0.0", port));
  
  if (socket.joinMulticast("239.255.42.99")) {
    //OUTPUT(idText, text, "Joined multicast group successfully.");
  }
  socket.setLoopback(false);
  OUTPUT_TEXT("Mocap Comm successfully started!");
}



unsigned MocapHandler::receive()
{
  if (!port)
    return 0; // not started yet

  MocapMessage inMsg;
  int size;
  unsigned receivedSize = 0;

  do
  {
    size = socket.read((char*)&inMsg, sizeof(MocapMessage));
    if (size > 0 && size <= static_cast<int>(sizeof(MocapMessage)))
    {
      receivedSize = static_cast<unsigned>(size);
      in.messages.push_back(inMsg);
      //OUTPUT(idText, text, "Mocap UDP package received!");
      
    }
  } 
  while (size > 0);

  return receivedSize;
}


bool MocapHandler::checkMessage(MocapMessage& msg)
{
  /*
  if (msg.iMessage == 7)
  {
    return true;
  }*/
  return false;
}
