/**
* @file CognitionConfigurationDataProvider.cpp
* This file implements a module that provides data loaded from configuration files.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include <cstdio>

#include "CognitionConfigurationDataProvider.h"
#include "Platform/File.h"

PROCESS_LOCAL CognitionConfigurationDataProvider* CognitionConfigurationDataProvider::theInstance = 0;

CognitionConfigurationDataProvider::CognitionConfigurationDataProvider()
{
  theInstance = this;

  readFieldDimensions();
  readCameraCalibration();
  readColorCalibration();
  readRobotDimensions();
  readHeadLimits();
  readOdometryCorrectionTables();
}

CognitionConfigurationDataProvider::~CognitionConfigurationDataProvider()
{
  if(theFieldDimensions)
    delete theFieldDimensions;
  if(theCameraCalibration)
    delete theCameraCalibration;
  if(theColorCalibration)
    delete theColorCalibration;
  if(theRobotDimensions)
    delete theRobotDimensions;
  if(theHeadLimits)
    delete theHeadLimits;
  if(theOdometryCorrectionTables)
    delete theOdometryCorrectionTables;
  theInstance = 0;
}

void CognitionConfigurationDataProvider::update(FieldDimensions& fieldDimensions)
{
  if(theFieldDimensions)
  {
    fieldDimensions = *theFieldDimensions;
    delete theFieldDimensions;
    theFieldDimensions = 0;
  }
  fieldDimensions.drawPolygons(theOwnTeamInfo.teamColour);
}

void CognitionConfigurationDataProvider::update(CameraCalibration& cameraCalibration)
{
  if(theCameraCalibration)
  {
    cameraCalibration = *theCameraCalibration;
    delete theCameraCalibration;
    theCameraCalibration = 0;
  }
  else
  {
    if(theCameraCalibrationNext.hasNext())
    {
      cameraCalibration = const_cast<CameraCalibrationNext&>(theCameraCalibrationNext).getNext();
    }
  }
}

void CognitionConfigurationDataProvider::update(ColorTable& colorTable)
{
  if(theColorCalibration)
  {
    colorTable.fromColorCalibration(*theColorCalibration, colorCalibration);
    delete theColorCalibration;
    theColorCalibration = 0;
  }

  DEBUG_RESPONSE_ONCE("representation:ColorCalibration:once")
  {
    OUTPUT(idColorCalibration, bin, colorCalibration);
  }
}

void CognitionConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  if(theRobotDimensions)
  {
    robotDimensions = *theRobotDimensions;
    delete theRobotDimensions;
    theRobotDimensions = 0;
  }
}

void CognitionConfigurationDataProvider::update(HeadLimits& headLimits)
{
  if(theHeadLimits)
  {
    headLimits = *theHeadLimits;
    delete theHeadLimits;
    theHeadLimits = 0;
  }
}

void CognitionConfigurationDataProvider::update(OdometryCorrectionTables& odometryCorrectionTables)
{
  if (theOdometryCorrectionTables)
  {
    odometryCorrectionTables = *theOdometryCorrectionTables;
    delete theOdometryCorrectionTables;
    theOdometryCorrectionTables = 0;
  }
}

void CognitionConfigurationDataProvider::readFieldDimensions()
{
  ASSERT(!theFieldDimensions);

  theFieldDimensions = new FieldDimensions;
  theFieldDimensions->load();
}

void CognitionConfigurationDataProvider::readCameraCalibration()
{
  ASSERT(!theCameraCalibration);

  InMapFile stream("cameraCalibration.cfg");
  if(stream.exists())
  {
    theCameraCalibration = new CameraCalibration;
    stream >> *theCameraCalibration;
  }
}

void CognitionConfigurationDataProvider::readColorCalibration()
{
  ASSERT(!theColorCalibration);

  InMapFile stream("colorCalibration.cfg");
  if(stream.exists())
  {
    theColorCalibration = new ColorCalibration;
    stream >> *theColorCalibration;
  }
}

void CognitionConfigurationDataProvider::readRobotDimensions()
{
  ASSERT(!theRobotDimensions);

  InMapFile stream("robotDimensions.cfg");
  if(stream.exists())
  {
    theRobotDimensions = new RobotDimensions;
    stream >> *theRobotDimensions;
  }
}

void CognitionConfigurationDataProvider::readHeadLimits()
{
  ASSERT(!theHeadLimits);

  InMapFile stream("headLimits.cfg");
  if(stream.exists())
  {
    theHeadLimits = new HeadLimits;
    stream >> *theHeadLimits;
  }
}

void CognitionConfigurationDataProvider::readOdometryCorrectionTables()
{
  ASSERT(!theOdometryCorrectionTables);

  InMapFile stream("odometryCorrectionTables.cfg");
  if (stream.exists())
  {
    theOdometryCorrectionTables = new OdometryCorrectionTables;
    stream >> *theOdometryCorrectionTables;
  }
}

bool CognitionConfigurationDataProvider::handleMessage(InMessage& message)
{
  if(theInstance && message.getMessageID() == idColorCalibration)
  {
    if(!theInstance->theColorCalibration)
      theInstance->theColorCalibration = new ColorCalibration;
    message.bin >> *theInstance->theColorCalibration;
    return true;
  }
  else
    return false;
}

MAKE_MODULE(CognitionConfigurationDataProvider, cognitionInfrastructure)
