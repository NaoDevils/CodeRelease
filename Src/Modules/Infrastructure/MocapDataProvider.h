/**
 * @file MocapDataProvider.h
 * This file declares a module that provides data from the motion capturing system
 * @author Janine Hemmers
 */
 
#pragma once 

#include "Representations/Infrastructure/MocapData.h"
#include "Representations/Configuration/MocapCalibration.h"
#include "Representations/Infrastructure/MocapRigidbody.h"
#include "Representations/Infrastructure/MocapMarkerSet.h"
#include "Tools/ProcessFramework/MocapHandler.h"
#include "Tools/Module/Module.h"
#include "Tools/Settings.h"

MODULE(MocapDataProvider,
{,
  PROVIDES(MocapData),
});

class MocapDataProvider : public MocapDataProviderBase
{
  private:
    static PROCESS_LOCAL MocapDataProvider* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
    MocapData* mocapDataPtr;  /**< A pointer to have access to the mocap data struct from all methods */
    Settings settings;

    /** The main function, called every cycle
    * @param data The data struct to be filled
    */
    void update(MocapData& data);

    /** This method unpacks the framenumber of the mocap UDP package
    * @param pData The UDP package data
    */
    int getFramenumber(char* pData);

    /** This method unpacks the mocap UDP package
    * @param pData The UDP package data
    */
    void unpack(char* pData);

  public:
    /** Default constructor */
    MocapDataProvider();

    /** Destructor */
    ~MocapDataProvider();

    /**
    * The method is called to handle all incoming mocap messages.
    * @param mocapReceiver The message queue containing all mocap messages received.
    */
    static void handleMessages(MocapDataIn& mocapReceiver);
};