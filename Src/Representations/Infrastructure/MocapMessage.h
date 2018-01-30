/**
 * @file MocapMessage.h
 * The file declares a struct that encapsulates the structure MocapMessage
 * @author Janine Hemmers
 */

#pragma once


#define MAX_MOCAP_PACKETSIZE 100000	// max size of packet (actual packet size is dynamic)
#define MAX_NAMELENGTH              256

struct sSender
{
  char szName[MAX_NAMELENGTH];            // sending app's name
  unsigned char Version[4];               // sending app's version [major.minor.build.revision]
  unsigned char NatNetVersion[4];         // sending app's NatNet version [major.minor.build.revision]

};

struct MocapMessage 
{
private:

  //MessageQueue theOutMsgData;

public:
  unsigned short iMessage;                // message ID (e.g. NAT_FRAMEOFDATA)
  unsigned short nDataBytes;              // Num bytes in payload
  //union Data
  //{
    unsigned char  cData[MAX_MOCAP_PACKETSIZE];
    char           szData[MAX_MOCAP_PACKETSIZE];
    unsigned long  lData[MAX_MOCAP_PACKETSIZE / 4];
    float          fData[MAX_MOCAP_PACKETSIZE / 4];
    sSender        Sender;
  //};

  MocapMessage() {}
};