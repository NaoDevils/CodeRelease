/**
* @file Controller/RobotConsole.cpp
*
* Implementation of RobotConsole.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "RobotConsole.h"
#include "SimulatedRobot.h"

#include "LogPlayer.h"

#include <algorithm>
#include <fstream>
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Representations/Infrastructure/YoloInput.h"
#include "Representations/Infrastructure/TeamCommSenderOutput.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDataStreamer.h"
#include "Tools/Debugging/QueueFillRequest.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Module/Module.h"
#include "Modules/MotionControl/KeyFrameEngine/KeyFrameEngine.h"

#include "ConsoleRoboCupCtrl.h"
#include "Platform/File.h"
#include "Views/AnnotationView/AnnotationView.h"
#include "Views/ColorSpaceView.h"
#include "Views/FieldView.h"
#include "Views/ImageView.h"
#include "Views/JointView.h"
#include "Views/KickView/KickView.h"
#include "Views/ModuleGraphView.h"
#include "Views/PlotView.h"
#include "Views/SensorView.h"
#include "Views/CABSLBehaviorView.h"
#include "Views/TimeView/TimeView.h"
#include "Views/DataView/DataView.h"
#include <nlohmann/json.hpp>

#include "Visualization/DebugDrawing.h"
#include "Visualization/DebugDrawing3D.h"


#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/RoleSymbols/RemoteControl.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/SensorData/SonarSensorData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Perception/GoalPercept.h"

#include "Representations/AnnotationInfo.h"
#include "Representations/TimeInfo.h"
#include "Representations/ModuleInfo.h"

#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <filesystem>

using namespace std;

#define PREREQUISITE(p) \
  pollingFor = #p;      \
  if (!poll(p))         \
    return false;

struct RobotConsole::Pimpl
{
  JointRequest jointRequest; /**< The joint angles request received from the robot code. */
  JointSensorData jointSensorData; /**< The most current set of joint angles received from the robot code. */
  FsrSensorData fsrSensorData; /**< The most current set of fsr sensor data received from the robot code. */
  InertialSensorData inertialSensorData; /**< The most current set of inertial sensor data received from the robot code. */
  KeyStates keyStates; /**< The most current set of key states received from the robot code. */
  SystemSensorData systemSensorData; /**< The most current set of system sensor data received from the robot code. */
  SonarSensorData sonarSensorData; /**< The most current set of sonar sensor data received from the robot code. */
  RobotPose robotPose; /**< Robot pose from team communication. */
  BallModel ballModel; /**< Ball model from team communication. */
  TeamBallModel teamBallModel; /**< combined ball information from team communication */
  GoalPercept goalPercept; /**< Goal percept from team communication. */
  BehaviorData behaviorData; /**< Behavior data from team communication. */
  RobotHealth robotHealth; /**< Robot Health from team communication. */
  MotionRequest motionRequest; /**< Motion Request from team communication. */
  JointCalibration jointCalibration; /**< The joint calibration received from the robot code. */
  RobotDimensions robotDimensions; /**< The robotDimensions received from the robot code. */
  ActivationGraph activationGraph; /**< Graph of active options and states. */
  RemoteControlRequest remoteControlRequest;

  std::unordered_map<char, AnnotationInfo> annotationInfos;
  Drawings camImageDrawings, motionImageDrawings; /**< Buffers for image drawings from the debug queue. */
  Drawings camFieldDrawings, motionFieldDrawings; /**< Buffers for field drawings from the debug queue. */
  Drawings3D camDrawings3D, motionDrawings3D; /**< Buffers for 3d drawings from the debug queue. */
  TimeInfos timeInfos; /**< Information about the timing of modules per process. */

  Drawings incompleteImageDrawings; /**< Buffers incomplete image drawings from the debug queue. */
  Drawings incompleteFieldDrawings; /**< Buffers incomplete field drawings from the debug queue. */
  Drawings3D incompleteDrawings3D; /**< Buffers incomplete 3d drawings from the debug queue. */
};

bool RobotConsole::MapWriter::handleMessage(InMessage& message)
{
  ASSERT(message.getMessageID() == idDebugDataResponse);
  std::string name, type;
  message.bin >> name >> type;
  DebugDataStreamer streamer(streamHandler, message.bin, type);
  stream << streamer;
  return true;
}

bool RobotConsole::Printer::handleMessage(InMessage& message)
{
  ASSERT(message.getMessageID() == idText);
  ctrl->printLn(message.text.readAll());
  return true;
}

RobotConsole::RobotConsole(MessageQueue& in, MessageQueue& out, Settings& settings)
    : Process(in, out, settings), data(std::make_unique<Pimpl>()), logPlayerPtr(std::make_unique<LogPlayer>(out)), debugOut(out), annotationInfos(data->annotationInfos),
      camImageDrawings(data->camImageDrawings), motionImageDrawings(data->motionImageDrawings), camFieldDrawings(data->camFieldDrawings), motionFieldDrawings(data->motionFieldDrawings),
      camDrawings3D(data->camDrawings3D), motionDrawings3D(data->motionDrawings3D), moduleInfoPtr(std::make_unique<ModuleInfo>()), repViewWriter(&representationViews),
      incompleteImageDrawings(data->incompleteImageDrawings), incompleteFieldDrawings(data->incompleteFieldDrawings), incompleteDrawings3D(data->incompleteDrawings3D), timeInfos(data->timeInfos)
{
  // this is a hack: call global functions to get parameters
  ctrl = (ConsoleRoboCupCtrl*)RoboCupCtrl::controller;
  robotFullName = RoboCupCtrl::getRobotFullName();
  robotName = robotFullName.mid(robotFullName.lastIndexOf('.') + 1);
  for (int i = 0; i < numOfMessageIDs; ++i)
  {
    waitingFor[i] = 0;
    polled[i] = false;
  }
  for (int i = 0; i < Joystick::numOfAxes; ++i)
  {
    joystickAxisMaxSpeeds[i] = joystickAxisThresholds[i] = joystickAxisCenters[i] = 0.f;
    joystickAxisMappings[i] = 0;
  }
  logPlayer.setSize(size_t{64} * 1024 * 1024 * 1024); // max. 64 GB

  currentImageDrawings = &camImageDrawings;
  currentFieldDrawings = &camFieldDrawings;
  currentDrawings3D = &camDrawings3D;
  timeInfos['c'] = TimeInfo("Cognition");
  timeInfos['m'] = TimeInfo("Motion");
  annotationInfos['c'];
  annotationInfos['m'];
}

RobotConsole::~RobotConsole()
{
  SYNC;
  streamHandler.clear();
  if (logMessages)
    delete[] logMessages;
  for (DebugDataInfos::iterator i = debugDataInfos.begin(); i != debugDataInfos.end(); ++i)
    delete i->second.second;
  destructed = true;
}

void RobotConsole::init()
{
  if (mode == SystemCall::remoteRobot)
    poll(idRobotname);
  if (mode != SystemCall::teamRobot)
  {
    joystick.init();
  }
}

void RobotConsole::addViews()
{
  SimRobot::Object* category = ctrl->addCategory(robotName, 0, ":/Icons/SimRobot.png");
  if (mode != SystemCall::teamRobot)
  {
    SimRobot::Object* annotationCategory = ctrl->addCategory("annotations", category);
    ctrl->addView(new AnnotationView(robotName + ".annotations.cognition", annotationInfos['c'], logPlayer, *this, ctrl->application), annotationCategory);
    ctrl->addView(new AnnotationView(robotName + ".annotations.motion", annotationInfos['m'], logPlayer, *this, ctrl->application), annotationCategory);

    ctrl->addView(new CABSLBehaviorView(robotName + ".behavior", *this, data->activationGraph, activationGraphReceived), category);

    ctrl->addCategory("colorSpace", ctrl->application->resolveObject(robotName));
    ctrl->addCategory("data", ctrl->application->resolveObject(robotName));
    ctrl->addCategory("field", ctrl->application->resolveObject(robotName));
    ctrl->addCategory("image", ctrl->application->resolveObject(robotName));
    ctrl->addView(new JointView(robotName + ".jointData", *this, data->jointSensorData, data->jointRequest), category);
    SimRobot::Object* modulesCategory = ctrl->addCategory("modules", category);
    SimRobot::Object* cognitionCategory = ctrl->addCategory("cognition", modulesCategory);
    ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.cognition.all", *this, 'c'), cognitionCategory);
    ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.cognition.behaviorControl", *this, 'c', ModuleBase::behaviorControl), cognitionCategory);
    ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.cognition.infrastructure", *this, 'c', ModuleBase::cognitionInfrastructure), cognitionCategory);
    ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.cognition.modeling", *this, 'c', ModuleBase::modeling), cognitionCategory);
    ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.cognition.pathPlanning", *this, 'c', ModuleBase::pathPlanning), cognitionCategory);
    ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.cognition.perception", *this, 'c', ModuleBase::perception), cognitionCategory);
    SimRobot::Object* motionCategory = ctrl->addCategory("motion", modulesCategory);
    ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.motion.all", *this, 'm'), motionCategory);
    ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.motion.infrastructure", *this, 'm', ModuleBase::motionInfrastructure), motionCategory);
    ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.motion.motionControl", *this, 'm', ModuleBase::motionControl), motionCategory);
    ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.motion.sensing", *this, 'm', ModuleBase::sensing), motionCategory);
    ctrl->addView(new ModuleGraphViewObject(robotName + ".modules.motion.dortmundWalkingEngine", *this, 'm', ModuleBase::dortmundWalkingEngine), motionCategory);
    ctrl->addCategory("plot", ctrl->application->resolveObject(robotName));
  }

  ctrl->addView(new SensorView(robotName + ".sensorData", *this, data->fsrSensorData, data->inertialSensorData, data->keyStates, data->systemSensorData), category);

  if (mode != SystemCall::teamRobot)
  {
    SimRobot::Object* timingCategory = ctrl->addCategory("timing", category);
    ctrl->addView(new TimeView(robotName + ".timing.cognition", *this, timeInfos.at('c')), timingCategory);
    ctrl->addView(new TimeView(robotName + ".timing.motion", *this, timeInfos.at('m')), timingCategory);
  }
}

void RobotConsole::addColorSpaceViews(const std::string& id, const std::string& name, bool user, bool upperCam)
{
  SimRobot::Object* colorSpaceCategory = ctrl->application->resolveObject(robotName + ".colorSpace");
  SimRobot::Object* nameCategory = ctrl->addCategory(name.c_str(), colorSpaceCategory);

  for (int cm = 0; cm < ColorSpaceView::numOfColorModels - (user ? 0 : 1); ++cm)
  {
    SimRobot::Object* modelCategory = ctrl->addCategory(ColorSpaceView::getName(ColorSpaceView::ColorModel(cm)), nameCategory);
    const QString& modelCategoryName = modelCategory->getFullName();

    if (ColorSpaceView::ColorModel(cm) == ColorSpaceView::user)
    {
      for (int channel = 1; channel < 4; ++channel)
      {
        ctrl->addView(
            new ColorSpaceView(modelCategoryName + "." + ColorSpaceView::getChannelNameForColorModel(ColorSpaceView::ColorModel(cm), channel), *this, id, ColorSpaceView::YCbCr, channel + 3, background, upperCam),
            modelCategory);
      }
    }
    else
    {
      for (int channel = 0; channel < 4; ++channel)
      {
        ctrl->addView(
            new ColorSpaceView(modelCategoryName + "." + ColorSpaceView::getChannelNameForColorModel(ColorSpaceView::ColorModel(cm), channel),
                *this,
                id,
                ColorSpaceView::ColorModel(cm),
                channel + (cm == ColorSpaceView::YCbCr && channel ? 1 : 0),
                background,
                upperCam),
            modelCategory);
      }
    }
  }
}

void RobotConsole::handleAllMessages(MessageQueue& messageQueue)
{
  SYNC; // Only one thread can access *this now.
  messageQueue.handleAllMessages(*this);
}

bool RobotConsole::handleMessage(InMessage& message)
{
  if (!handleMessages)
    return true;

  if (destructed) // if object is already destroyed, abort here
    return true; // avoid further processing of this message

  if (message.getMessageID() < numOfDataMessageIDs)
  {
    if (logImagesAsJPEGs && message.getMessageID() == idImage)
    {
      Image image;
      message.bin >> image;
      MessageQueue queue;
      queue.out.bin << JPEGImage(image);
      queue.out.finishMessage(idJPEGImage);
      logPlayer.handleMessage(queue.in);
    }
    else if (logImagesAsJPEGs && message.getMessageID() == idImageUpper)
    {
      ImageUpper image;
      message.bin >> image;
      MessageQueue queue;
      queue.out.bin << JPEGImage(image);
      queue.out.finishMessage(idJPEGImageUpper);
      logPlayer.handleMessage(queue.in);
    }
    else
      logPlayer.handleMessage(message);
    message.resetReadPosition();
  }

  switch (message.getMessageID())
  {
  case idText:
  {
    std::string buffer(message.text.readAll());
    if (printMessages)
      ctrl->printLn(buffer);
    if (logMessages)
      *logMessages << buffer << std::endl;
    return true;
  }
  case idConsole:
    commands.push_back(message.text.readAll());
    return true;
  case idImage:
    if (!incompleteImages["raw image"])
      incompleteImages["raw image"] = std::make_unique<Image>(false);
    message.bin >> *incompleteImages["raw image"].get();
    return true;
  case idImageUpper:
    if (!incompleteImages["raw imageUpper"])
      incompleteImages["raw imageUpper"] = std::make_unique<ImageUpper>(false);
    message.bin >> *incompleteImages["raw imageUpper"].get();
    return true;
  case idJPEGImage:
  {
    if (!incompleteImages["raw image"])
      incompleteImages["raw image"] = std::make_unique<Image>(false);
    JPEGImage jpi;
    message.bin >> jpi;
    jpi.toImage(*incompleteImages["raw image"].get());
    return true;
  }
  case idJPEGImageUpper:
  {
    if (!incompleteImages["raw imageUpper"])
      incompleteImages["raw imageUpper"] = std::make_unique<ImageUpper>(false);
    JPEGImage jpi;
    message.bin >> jpi;
    jpi.toImage(*incompleteImages["raw imageUpper"].get());
    return true;
  }
  case idThumbnail:
  {
    if (!incompleteImages["raw image"])
      incompleteImages["raw image"] = std::make_unique<Image>(false);
    Thumbnail thumbnail;
    message.bin >> thumbnail;
    thumbnail.toImage(*incompleteImages["raw image"].get());
    incompleteImages["raw image"]->timeStamp = SystemCall::getCurrentSystemTime();
    return true;
  }
  case idThumbnailUpper:
  {
    if (!incompleteImages["raw imageUpper"])
      incompleteImages["raw imageUpper"] = std::make_unique<ImageUpper>(false);
    ThumbnailUpper thumbnail;
    message.bin >> thumbnail;
    thumbnail.toImage(*incompleteImages["raw imageUpper"].get());
    incompleteImages["raw imageUpper"]->timeStamp = SystemCall::getCurrentSystemTime();
    return true;
  }
  case idYoloInput:
  {
    if (!incompleteImages["raw image"])
      incompleteImages["raw image"] = std::make_unique<Image>(false);
    YoloInput yoloInput;
    message.bin >> yoloInput;
    yoloInput.toImage(*incompleteImages["raw image"]);
    incompleteImages["raw image"]->timeStamp = SystemCall::getCurrentSystemTime();
    return true;
  }
  case idYoloInputUpper:
  {
    if (!incompleteImages["raw imageUpper"])
      incompleteImages["raw imageUpper"] = std::make_unique<ImageUpper>(false);
    YoloInputUpper yoloInputUpper;
    message.bin >> yoloInputUpper;
    yoloInputUpper.toImage(*incompleteImages["raw imageUpper"].get());
    incompleteImages["raw imageUpper"]->timeStamp = SystemCall::getCurrentSystemTime();
    return true;
  }
  case idDebugImage:
  {
    std::string id;
    message.bin >> id;
    if (!incompleteImages[id])
      incompleteImages[id] = std::make_unique<Image>(false);
    message.bin >> *incompleteImages[id].get();
    incompleteImages[id]->timeStamp = SystemCall::getCurrentSystemTime();
    break;
  }
  case idDebugJPEGImage:
  {
    std::string id;
    message.bin >> id;
    if (!incompleteImages[id])
      incompleteImages[id] = std::make_unique<Image>(false);
    JPEGImage jpi;
    message.bin >> jpi;
    jpi.toImage(*incompleteImages[id].get());
    incompleteImages[id]->timeStamp = SystemCall::getCurrentSystemTime();
    break;
  }
  case idFsrSensorData:
  {
    message.bin >> data->fsrSensorData;
    return true;
  }
  case idInertialSensorData:
  {
    message.bin >> data->inertialSensorData;
    return true;
  }
  case idJointRequest:
  {
    message.bin >> data->jointRequest;
    data->jointRequest.angles[Joints::rHipYawPitch] = data->jointRequest.angles[Joints::lHipYawPitch];
    return true;
  }
  case idJointSensorData:
  {
    message.bin >> data->jointSensorData;
    return true;
  }
  case idKeyStates:
  {
    message.bin >> data->keyStates;
    return true;
  }
  case idSystemSensorData:
  {
    message.bin >> data->systemSensorData;
    return true;
  }
  case idSonarSensorData:
  {
    message.bin >> data->sonarSensorData;
    return true;
  }
  case idAudioData:
  {
    AudioData audioData;
    message.bin >> audioData;
    audioPlayer.play(audioData);
    return true;
  }
  case idDebugDrawing:
  {
    if (polled[idDrawingManager] && !waitingFor[idDrawingManager]) // drawing manager not up-to-date
    {
      char shapeType, id;
      message.bin >> shapeType >> id;
      const char* name = drawingManager.getDrawingName(id); // const char* is required here
      std::string type = drawingManager.getDrawingType(name);

      if (type == "drawingOnImage")
        incompleteImageDrawings[name].addShapeFromQueue(message, (::Drawings::ShapeType)shapeType);
      else if (type == "drawingOnField")
        incompleteFieldDrawings[name].addShapeFromQueue(message, (::Drawings::ShapeType)shapeType);
    }
    return true;
  }
  case idDebugDrawing3D:
  {
    if (polled[idDrawingManager3D] && !waitingFor[idDrawingManager3D])
    {
      char shapeType, id;
      message.bin >> shapeType >> id;
      incompleteDrawings3D[drawingManager3D.getDrawingName(id)].addShapeFromQueue(message, (::Drawings3D::ShapeType)shapeType, processIdentifier);
    }
    return true;
  }
  case idPlot:
  {
    std::string id;
    float value;
    message.bin >> id >> value;
    Plot& plot = plots[ctrl->translate(id)];
    plot.points.push_back(value);
    while (plot.points.size() > maxPlotSize)
      plot.points.pop_front();
    plot.timeStamp = SystemCall::getCurrentSystemTime();
    return true;
  }
  case idProcessBegin:
  {
    message.bin >> processIdentifier;
    drawingManager.setProcess(processIdentifier == 'd' ? 'c' : processIdentifier);
    drawingManager3D.setProcess(processIdentifier == 'd' ? 'c' : processIdentifier);
    return true;
  }
  case idProcessFinished:
  {
    char c;
    message.bin >> c;
    ++frame;
    ASSERT(processIdentifier == c);

    if (processIdentifier != 'm')
    {
      currentImageDrawings = &camImageDrawings;
      currentFieldDrawings = &camFieldDrawings;
      currentDrawings3D = &camDrawings3D;
      camImages.clear();
      for (Images::iterator i = incompleteImages.begin(); i != incompleteImages.end(); ++i)
      {
        auto& imagePtr = (camImages)[i->first];
        imagePtr = std::move(i->second);
      }
    }
    else //processIdentifier == 'm'
    {
      currentImageDrawings = &motionImageDrawings;
      currentFieldDrawings = &motionFieldDrawings;
      currentDrawings3D = &motionDrawings3D;
    }

    // Add new Field and Image drawings
    currentImageDrawings->clear();
    currentFieldDrawings->clear();
    if (processIdentifier == 'm' || drawingsViaProcess == 'b' || drawingsViaProcess == processIdentifier)
    {
      for (Drawings::const_iterator i = incompleteImageDrawings.begin(); i != incompleteImageDrawings.end(); ++i)
      {
        DebugDrawing& debugDrawing = (*currentImageDrawings)[i->first];
        debugDrawing = i->second;
        debugDrawing.processIdentifier = processIdentifier;
      }

      for (Drawings::const_iterator i = incompleteFieldDrawings.begin(); i != incompleteFieldDrawings.end(); ++i)
      {
        DebugDrawing& debugDrawing = (*currentFieldDrawings)[i->first];
        debugDrawing = i->second;
        debugDrawing.processIdentifier = processIdentifier;
      }
    }

    // 3D Drawings
    if (polled[idDrawingManager3D] && !waitingFor[idDrawingManager3D])
    {
      // reset all 3d drawings originated from current process
      for (Drawings3D::iterator i = currentDrawings3D->begin(), end = currentDrawings3D->end(); i != end; ++i)
      {
        ASSERT(i->second.processIdentifier == processIdentifier);
        DebugDrawing3D& debugDrawing3D = i->second;
        debugDrawing3D.reset();
      }

      // copy and register newly received 3d debug drawings
      if (processIdentifier == 'm' || drawingsViaProcess == 'b' || drawingsViaProcess == processIdentifier)
      {
        for (Drawings3D::iterator i = incompleteDrawings3D.begin(); i != incompleteDrawings3D.end(); ++i)
        {
          std::string type = drawingManager3D.getDrawingType(drawingManager3D.getString(i->first));
          std::string name = type == "camera" ? i->first + processIdentifier : i->first;
          DebugDrawing3D& debugDrawing3D = (*currentDrawings3D)[name];
          bool drawn = debugDrawing3D.drawn;
          bool flip = debugDrawing3D.flip;
          i->second.robotConsole = this;
          debugDrawing3D = i->second;
          debugDrawing3D.drawn = drawn;
          debugDrawing3D.flip = flip;
          if (!drawn)
          {
            if (type != "unknown")
            {
              QVector<QString> parts;
              parts.append(robotName);
              if (type == "field")
              {
                QString robotNumberString(robotName);
                robotNumberString.remove(0, 5);
                debugDrawing3D.flip = robotNumberString.toInt() <= MAX_NUM_PLAYERS;
                parts[0] = "RoboCup";
              }
              else if (type == "robot")
                parts.append("origin");
              else if (type == "camera")
                parts.append(processIdentifier == 'c' ? "CameraTop" : "CameraBottom");
              else
                parts.append(type.c_str());
              SYNC_WITH(*ctrl);
              SimRobotCore2::PhysicalObject* object = (SimRobotCore2::PhysicalObject*)ctrl->application->resolveObject(parts);
              object->registerDrawing(debugDrawing3D);
              debugDrawing3D.drawn = true;
            }
          }
        }
      }
    }

    incompleteImages.clear();
    incompleteImageDrawings.clear();
    incompleteFieldDrawings.clear();
    incompleteDrawings3D.clear();

    return true;
  }
  case idActivationGraph:
    message.bin >> data->activationGraph;
    activationGraphReceived = SystemCall::getCurrentSystemTime();
    return true;
  case idAnnotation:
    ASSERT(annotationInfos.find(processIdentifier == 'd' ? 'c' : processIdentifier) != annotationInfos.end());
    annotationInfos[processIdentifier == 'd' ? 'c' : processIdentifier].handleMessage(message, frame);
    return true;
  case idStopwatch:
    ASSERT(timeInfos.find(processIdentifier == 'd' ? 'c' : processIdentifier) != timeInfos.end());
    timeInfos.at(processIdentifier == 'd' ? 'c' : processIdentifier).handleMessage(message);
    return true;
  case idDebugResponse:
  {
    SYNC_WITH(*ctrl);
    std::string description;
    bool enable;
    message.text >> description >> enable;
    if (description != "pollingFinished")
      debugRequestTable.addRequest(DebugRequest(description, enable), true);
    else if (--waitingFor[idDebugResponse] <= 0)
    {
      ctrl->setDebugRequestTable(debugRequestTable);
      updateCompletion = true;
    }
    return true;
  }
  case idModuleTable:
  {
    SYNC_WITH(*ctrl);

    moduleInfo.handleMessage(message, processIdentifier == 'd' ? 'c' : processIdentifier);
    if (--waitingFor[idModuleTable] <= 0)
    {
      ctrl->setModuleInfo(moduleInfo);
      updateCompletion = true;
      polled[idDebugResponse] = polled[idDrawingManager] = polled[idDrawingManager3D] = false;
    }
    return true;
  }
  case idDrawingManager:
  {
    SYNC_WITH(*ctrl);
    message.bin >> drawingManager;
    if (--waitingFor[idDrawingManager] <= 0)
    {
      ctrl->setDrawingManager(drawingManager);
      updateCompletion = true;
    }
    return true;
  }
  case idDrawingManager3D:
  {
    SYNC_WITH(*ctrl);
    message.bin >> drawingManager3D;
    if (--waitingFor[idDrawingManager3D] <= 0)
    {
      ctrl->setDrawingManager3D(drawingManager3D);
      updateCompletion = true;
    }
    return true;
  }
  case idStreamSpecification:
  {
    message.bin >> streamHandler;
    --waitingFor[idStreamSpecification];
    return true;
  }
  case idDebugDataResponse:
  {
    std::string name, type;
    message.bin >> name >> type;
    if (debugDataInfos.find(name) == debugDataInfos.end())
      debugDataInfos[name] = DebugDataInfoPair(type, new MessageQueue);
    debugDataInfos[name].second->clear();
    message >> *debugDataInfos[name].second;
    const char* t = streamHandler.getString(type);
    if (streamHandler.basicTypeSpecification.find(t) != streamHandler.basicTypeSpecification.end() || streamHandler.specification.find(t) != streamHandler.specification.end()
        || streamHandler.enumSpecification.find(t) != streamHandler.enumSpecification.end())
    {
      repViewWriter.handleMessage(message, type, name);
    }
    else if (polled[idStreamSpecification] || !waitingFor[idStreamSpecification])
    {
      polled[idStreamSpecification] = false;
      if (getOrSetWaitsFor != name)
        commands.push_back("_vd");
    }

    if (getOrSetWaitsFor == name) // console command requested this one?
      waitingFor[idDebugDataResponse] = 0;

    return true;
  }
  case idLogResponse:
    logAcknowledged = true;
    return true;
  case idTeamBallModel:
    message.bin >> data->teamBallModel;
    teamBallModelReceived = SystemCall::getCurrentSystemTime();
    return true;
  case idTeammateHasGroundContact:
    message.bin >> hasGroundContact;
    hasGroundContactReceived = SystemCall::getCurrentSystemTime();
    return true;
  case idTeammateIsUpright:
    message.bin >> isUpright;
    isUprightReceived = SystemCall::getCurrentSystemTime();
    return true;
  case idBehaviorData:
    message.bin >> data->behaviorData;
    return true;
  case idRobotHealth:
    message.bin >> data->robotHealth;
    robotHealthReceived = SystemCall::getCurrentSystemTime();
    return true;
  case idMotionRequest:
    message.bin >> data->motionRequest;
    motionRequestReceived = SystemCall::getCurrentSystemTime();
    return true;
  case idRobotname:
  {
    Global::theInstance.settings->read(message.bin);
    --waitingFor[idRobotname];
    return true;
  }
  case idRobotDimensions:
    message.bin >> data->robotDimensions;
    return true;
  case idJointCalibration:
    message.bin >> data->jointCalibration;
    return true;
  case idExecutorObservings:
  {
    SYNC_WITH(*ctrl);

    std::vector<uint8_t> data;
    STREAM_EXT(message.bin, data);

    if (data.size() == 0)
    {
      observerDataFinished[processIdentifier] = true;
    }
    else
    {
      observerDataFinished[processIdentifier] = false;
      std::move(data.begin(), data.end(), std::back_inserter(observerMsgpackData[processIdentifier]));
    }

    if (observerDataFinished['m'] && observerDataFinished['c'])
    {
      using json = nlohmann::json;

      json j = json::array();
      json motion = json::from_msgpack(observerMsgpackData['m']);
      j.insert(j.begin(), motion.begin(), motion.end());
      json cognition = json::from_msgpack(observerMsgpackData['c']);
      j.insert(j.begin(), cognition.begin(), cognition.end());

      auto now = std::chrono::system_clock::now();
      auto in_time_t = std::chrono::system_clock::to_time_t(now);

      std::stringstream ss;
      ss << "Logs/executorObservings_" << Global::getSettings().robotName << std::put_time(std::localtime(&in_time_t), "_%Y%m%d_%H%M%S") << ".json";
      OutTextRawFile file(ss.str());
      file << j.dump() << "\n";

      observerMsgpackData.clear();
      observerDataFinished.clear();
    }
    return true;
  }
  case idTeamCommSenderOutput:
  {
    TeamCommSenderOutput teamCommSenderOutput;
    message.bin >> teamCommSenderOutput;
    if (teamCommSenderOutput.dataSent)
    {
      QString robotNumberString(robotName);
      robotNumberString.remove(0, 5);
      ctrl->gameController.decreaseMessageBudget(robotNumberString.toInt());
    }
    return true;
  }
  default:
    return mode == SystemCall::teamRobot ? Process::handleMessage(message) : false;
  }

  return false;
}

void RobotConsole::update()
{
  GlobalGuard g(getGlobals()); // this is called in GUI thread -> set globals for this process
  handleJoystick();

  while (!lines.empty())
  {
    std::list<std::string> temp = lines;
    lines.clear();
    if (handleConsoleLine(temp.front()))
    {
      if (sleepTimer == 0)
      {
        temp.pop_front();
        lines.splice(lines.end(), temp);
      }
      else
      {
        lines = temp;
        break;
      }
    }
    else
    {
      lines = temp;
      break;
    }
  }

  if (!commands.empty())
  {
    std::list<std::string> commands;
    {
      SYNC;
      commands.swap(this->commands);
    }
    for (std::list<std::string>::const_iterator iter = commands.begin(), end = commands.end(); iter != end; ++iter)
      ctrl->executeConsoleCommand(*iter, this);
  }

  pollForDirectMode();

  if (updateCompletion)
  {
    SYNC;
    ctrl->updateCommandCompletion();
    updateCompletion = false;
  }
}

void RobotConsole::handleConsole(const std::string& line, bool fromCall)
{
  GlobalGuard g(getGlobals()); // this is called in GUI thread -> set globals for this process
  lines.push_back(line);
  while (!lines.empty())
  {
    std::list<std::string> temp = lines;
    lines.clear();
    if (handleConsoleLine(temp.front(), fromCall))
    {
      temp.pop_front();
      lines.splice(lines.end(), temp);
    }
    else
    {
      lines = temp;
      break;
    }
  }

  pollForDirectMode();
}

void RobotConsole::triggerProcesses()
{
  if (mode == SystemCall::logfileReplay)
  {
    SYNC;
    debugOut.out.bin << 'c';
    debugOut.out.finishMessage(idProcessBegin);
    debugOut.out.bin << 'c';
    debugOut.out.finishMessage(idProcessFinished);
    debugOut.out.bin << 'm';
    debugOut.out.finishMessage(idProcessBegin);
    debugOut.out.bin << 'm';
    debugOut.out.finishMessage(idProcessFinished);
  }
}

JointRequest& RobotConsole::getJointRequest()
{
  return data->jointRequest;
}

bool RobotConsole::poll(MessageID id)
{
  if (waitingFor[id] > 0)
  {
    // When in replay log file mode, force replay while polling to keep Cognition running
    triggerProcesses();
    return false;
  }
  else if (polled[id])
    return true;
  else
  {
    polled[id] = true;
    switch (id)
    {
    case idDebugResponse:
    {
      SYNC;
      debugRequestTable.removeAllRequests();
      debugOut.out.bin << DebugRequest("poll");
      debugOut.out.finishMessage(idDebugRequest);
      if (mode == SystemCall::teamRobot)
        waitingFor[id] = 1; // team robot module should answer
      else
        waitingFor[id] = 3; // Cognition + Motion + Debug will answer
      break;
    }
    case idModuleTable:
    {
      SYNC;
      moduleInfo.clear();
      debugOut.out.bin << DebugRequest("automated requests:ModuleTable", true);
      debugOut.out.finishMessage(idDebugRequest);
      waitingFor[id] = 2; // Cognition + Motion will answer
      break;
    }
    case idStreamSpecification:
    {
      SYNC;
      debugOut.out.bin << DebugRequest("automated requests:StreamSpecification", true);
      debugOut.out.finishMessage(idDebugRequest);
      if (mode == SystemCall::teamRobot)
        waitingFor[id] = 1; // team robot module should answer
      else
        waitingFor[id] = 2; // Cognition + Motion will answer
      break;
    }
    case idDrawingManager:
    {
      SYNC;
      drawingManager.clear();
      debugOut.out.bin << DebugRequest("automated requests:DrawingManager", true);
      debugOut.out.finishMessage(idDebugRequest);
      if (mode == SystemCall::teamRobot)
        waitingFor[id] = 1; // team robot module should answer
      else
        waitingFor[id] = 2; // Cognition + Motion will answer
      break;
    }
    case idDrawingManager3D:
    {
      SYNC;
      drawingManager3D.clear();
      debugOut.out.bin << DebugRequest("automated requests:DrawingManager3D", true);
      debugOut.out.finishMessage(idDebugRequest);
      if (mode == SystemCall::teamRobot)
        waitingFor[id] = 1; // team robot module should answer
      else
        waitingFor[id] = 2; // Cognition + Motion will answer
      break;
    }
    case idRobotname:
    {
      SYNC;
      debugOut.out.bin << DebugRequest("module:NaoProvider:robotName", true);
      debugOut.out.finishMessage(idDebugRequest);
      waitingFor[id] = 1; // Motion will answer
      break;
    }
    default:
      ASSERT(false);
    }
    return false;
  }
}

void RobotConsole::pollForDirectMode()
{
  if (directMode)
  {
    poll(idDebugResponse);
    poll(idDrawingManager);
    poll(idDrawingManager3D);
    poll(idModuleTable);
  }
}

bool RobotConsole::handleConsoleLine(const std::string& line, bool fromCall)
{
  InConfigMemory stream(line.c_str(), line.size());
  std::string command;
  stream >> command;
  bool result = false;
  if (command == "") // comment
    result = true;
  else if (command == "endOfStartScript")
  {
    directMode = true;
    result = true;
  }
  else if (command == "ac")
    result = acceptCamera(stream);
  else if (command == "bc")
    result = backgroundColor(stream);
  else if (command == "kick")
  {
    result = kickView();
  }
  else if (command == "cls")
  {
    ctrl->printLn("_cls");
    result = true;
  }
  else if (command == "dr")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager3D);
    result = debugRequest(stream);
  }
  else if (command == "echo")
  {
    ctrl->echo(stream);
    result = true;
  }
  else if (command == "get")
  {
    PREREQUISITE(idDebugResponse);
    result = get(stream, true, true);
  }
  else if (command == "_get") // get, part 2
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idStreamSpecification);
    result = get(stream, false, true);
  }
  else if (command == "_get2") // get, part 1 without printing
  {
    PREREQUISITE(idDebugResponse);
    result = get(stream, true, false);
  }
  else if (command == "_get3") // get, part 2 without printing
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idStreamSpecification);
    result = get(stream, false, false);
  }
  else if (command == "jc")
    result = joystickCommand(stream);
  else if (command == "js")
    result = joystickSpeeds(stream);
  else if (command == "jm")
    result = joystickMaps(stream);
  else if (command == "kfm")
    result = sendKfm(stream);
  else if (command == "log")
  {
    PREREQUISITE(idModuleTable);
    result = log(stream);
  }
  else if (command == "_log")
  {
    PREREQUISITE(idStreamSpecification);
    result = log(stream, false);
  }
  else if (command == "mr")
  {
    PREREQUISITE(idModuleTable);
    result = moduleRequest(stream);
  }
  else if (command == "msg")
  {
    result = msg(stream);
  }
  else if (command == "mv")
  {
    if (moveOp != noMove)
      return false;
    result = moveRobot(stream);
  }
  else if (command == "mvb")
  {
    if (moveOp != noMove)
      return false;
    result = moveBall(stream);
  }
  else if (command == "kiba")
  {
    result = kickBall(stream);
  }
  else if (command == "rc")
  {
    result = remoteControl(stream);
  }
  else if (command == "sleep")
  {
    result = sleepConsole(stream);
  }
  else if (command == "poll")
    result = repoll(stream);
  else if (command == "pr" && mode == SystemCall::simulatedRobot)
    result = ctrl->gameController.handleRobotConsole(robotName.mid(5).toInt() - 1, stream);
  else if (command == "qfr")
    result = queueFillRequest(stream);
  else if (command == "save")
  {
    PREREQUISITE(idDebugResponse);
    result = saveRequest(stream, true);
  }
  else if (command == "_save")
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idStreamSpecification);
    result = saveRequest(stream, false);
  }
  else if (command == "set")
  {
    PREREQUISITE(idDebugResponse);
    result = set(stream);
  }
  else if (command == "_set") // set, part 2
  {
    PREREQUISITE(idDebugDataResponse);
    PREREQUISITE(idStreamSpecification);
    result = set(stream);
  }
  else if (command == "si")
  {
    result = saveImage(stream);
  }
  else if (command == "vf")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewField(stream);
  }
  else if (command == "vfd")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewDrawing(stream, fieldViews, "drawingOnField");
  }
  else if (command == "vd") //view data part 1
  {
    PREREQUISITE(idDebugResponse);
    result = viewData(stream);
  }
  else if (command == "_vd") //view data part 2
  {
    // This dummy command polls the stream specification if necessary
    PREREQUISITE(idStreamSpecification);
    result = true;
  }
  else if (command == "vp")
  {
    PREREQUISITE(idDebugResponse);
    result = viewPlot(stream);
  }
  else if (command == "vpd")
  {
    PREREQUISITE(idDebugResponse);
    result = viewPlotDrawing(stream);
  }
  else if (mode == SystemCall::teamRobot) // stop
    result = false;
  else if (command == "vid")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewDrawing(stream, imageViews, "drawingOnImage");
  }
  else if (command == "vi")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = viewImage(stream);
  }
  else if (command == "v3")
  {
    PREREQUISITE(idDebugResponse);
    PREREQUISITE(idDrawingManager);
    result = view3D(stream);
  }
  else if (command == "wek")
    result = sendWek(stream);

  pollingFor = 0;
  if (!result)
  {
    if (directMode && !fromCall)
    {
      ctrl->printLn("Syntax Error");
    }
    else
    {
      ctrl->printLn((std::string("Syntax Error: ") + line).c_str());
    }
  }
  return true;
}

bool RobotConsole::msg(In& stream)
{
  std::string state;
  stream >> state;
  if (state == "off")
  {
    printMessages = false;
    return true;
  }
  else if (state == "on")
  {
    printMessages = true;
    return true;
  }
  else if (state == "log")
  {
    if (logMessages)
      delete logMessages;
    stream >> state;
    std::string name(state);
    if (name.size() == 0)
      return false;
    else
    {
      if ((int)name.rfind('.') <= (int)name.find_last_of("\\/"))
        name = name + ".txt";
      char buf[FILENAME_MAX];
      if (name[0] != '/' && name[0] != '\\' && name[0] != '.' && (name[0] == 0 || name[1] != ':'))
        sprintf(buf, "%s/Config/Logs/", File::getBHDir());
      else
        buf[0] = 0;
      ASSERT(strlen(buf) + strlen(name.c_str()) < FILENAME_MAX);
      strcat(buf, name.c_str());
      logMessages = new std::fstream(buf, std::ios_base::out);
      return true;
    }
  }
  else if (state == "disable")
  {
    handleMessages = false;
    return true;
  }
  else if (state == "enable")
  {
    handleMessages = true;
    return true;
  }
  return false;
}

bool RobotConsole::backgroundColor(In& stream)
{
  stream >> background.x() >> background.y() >> background.z();
  background *= 0.01F;
  return true;
}

bool RobotConsole::debugRequest(In& stream)
{
  std::string debugRequestString, state;
  stream >> debugRequestString >> state;

  if (debugRequestString == "?")
  {
    for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      ctrl->list(ctrl->translate(debugRequestTable.debugRequests[i].description).c_str(), state);
    ctrl->printLn("");
    return true;
  }
  else
  {
    if (debugRequestString == "off")
    {
      SYNC;
      debugOut.out.bin << DebugRequest("disableAll");
      debugOut.out.finishMessage(idDebugRequest);
      return true;
    }
    else
      for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
        if (ctrl->translate(debugRequestTable.debugRequests[i].description) == debugRequestString)
        {
          DebugRequest& d = debugRequestTable.debugRequests[i];
          if (state == "off")
            d.enable = false;
          else if (state == "on" || state == "")
            d.enable = true;
          else
            return false;

          SYNC;
          debugOut.out.bin << d;
          debugOut.out.finishMessage(idDebugRequest);
          return true;
        }
  }
  return false;
}

bool RobotConsole::log(In& stream, bool first)
{
  std::string command;
  stream >> command;
  if (command == "start")
  {
    SYNC;
    if (logFile != "")
      logPlayer.play();
    else if (logPlayer.state != LogPlayer::recording)
      logPlayer.recordStart();
    return true;
  }
  else if (command == "stop")
  {
    SYNC;
    if (logPlayer.state == LogPlayer::recording)
      logPlayer.recordStop();
    else
      logPlayer.stop();
    return true;
  }
  else if (command == "clear")
  {
    SYNC;
    logPlayer.init();
    return true;
  }
  else if (command == "save")
  {
    std::string name;
    stream >> name;
    if (name.size() == 0)
      return false;
    else if (first && logFile == "") // poll specification if created new log file
    {
      polled[idStreamSpecification] = false;
      handleConsole("_log save " + name);
      return true;
    }
    else
    {
      if ((int)name.rfind('.') <= (int)name.find_last_of("\\/"))
        name = name + ".log";
      if (name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
        name = std::string("Logs\\") + name;
      SYNC;
      return logPlayer.save(name.c_str(), logFile == "" ? &streamHandler : nullptr);
    }
  }
  else if (command == "saveAudio")
  {
    SYNC;
    std::string name;
    stream >> name;
    if (name.size() == 0)
      return false;
    else
    {
      if ((int)name.rfind('.') <= (int)name.find_last_of("\\/"))
        name = name + ".wav";
      if (name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
        name = std::string("Sounds\\") + name;
      return logPlayer.saveAudioFile(name.c_str());
    }
  }
  else if (command == "saveTrueWhistleAudio")
  {
    SYNC;
    bool split = false;
    std::string name;
    stream >> name >> split;
    if (name.size() == 0)
      return false;
    else
    {
      if ((int)name.rfind('.') <= (int)name.find_last_of("\\/"))
        name = name + ".wav";
      if (name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
        name = std::string("Sounds\\Whistle\\") + name;
      int ret = logPlayer.saveTrueWhistleAudioFile(name.c_str(), split);
      if (ret == 1)
        ctrl->printLn("Potentially no true whistles in log");
      if (ret == 2)
        ctrl->printLn("Unable to create file");
      return true;
    }
  }
  else if (command == "saveFalseWhistleAudio")
  {
    SYNC;
    bool split = false;
    std::string name;
    stream >> name >> split;
    if (name.size() == 0)
      return false;
    else
    {
      if ((int)name.rfind('.') <= (int)name.find_last_of("\\/"))
        name = name + ".wav";
      if (name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
        name = std::string("Sounds\\Whistle\\") + name;
      int ret = logPlayer.saveFalseWhistleAudioFile(name.c_str(), split);
      if (ret == 1)
        ctrl->printLn("Potentially no false whistles in log");
      if (ret == 2)
        ctrl->printLn("Unable to create file");
      return true;
    }
  }
  else if (command == "saveImages")
  {
    SYNC;
    stream >> command;
    std::string name(command);
    bool raw = false;
    if (name == "raw")
    {
      raw = true;
      stream >> name;
    }
    if (name.size() == 0)
      name = "Images\\image";
    return logPlayer.saveImages(raw, name.c_str());
  }
  else if (command == "saveTiming")
  {
    SYNC;
    std::string name;
    stream >> name;
    if (name.size() == 0)
      return false;
    else
    {
      if ((int)name.rfind('.') <= (int)name.find_last_of("\\/"))
        name = name + ".csv";
      return logPlayer.writeTimingData(name);
    }
  }
  else if (command == "keep" || command == "remove")
  {
    SYNC;
    std::string buf;
    stream >> buf;
    std::vector<MessageID> messageIDs;
    while (buf != "")
    {
      int i;
      for (i = 1; i < numOfMessageIDs; ++i)
        if (buf == ::getName(MessageID(i)))
        {
          messageIDs.push_back(MessageID(i));
          break;
        }

      if (i == numOfMessageIDs)
        return false;
      stream >> buf;
    }
    if (messageIDs.size())
    {
      messageIDs.push_back(undefined);
      if (command == "keep")
        logPlayer.keep(&messageIDs[0]);
      else
        logPlayer.remove(&messageIDs[0]);
      return true;
    }
  }
  else if (command == "keepFrames" || command == "removeFrames")
  {
    SYNC;
    int startFrame = -1, endFrame = -1;
    stream >> startFrame >> endFrame;
    if (command == "keepFrames")
      logPlayer.keep(startFrame - 1, endFrame - 1);
    else
      logPlayer.remove(startFrame - 1, endFrame - 1);

    return true;
  }
  else if (command == "full")
  {
    logImagesAsJPEGs = false;
    return true;
  }
  else if (command == "jpeg")
  {
    logImagesAsJPEGs = true;
    return true;
  }
  else if (command == "?")
  {
    SYNC;
    std::string option;
    stream >> option;
    int frequencies[numOfMessageIDs];
    unsigned sizes[numOfMessageIDs];
    logPlayer.statistics(frequencies, sizes);

    float size = 0;
    for (int i = 0; i < numOfDataMessageIDs; ++i)
      size += static_cast<float>(sizes[i]);

    char buf[100];
    for (int i = 0; i < numOfDataMessageIDs; ++i)
      if (frequencies[i])
      {
        sprintf(buf, "%u\t%.2f%%", frequencies[i], static_cast<float>(sizes[i]) * 100.f / size);
        ctrl->list(std::string(buf) + "\t" + ::getName(MessageID(i)), option, true);
      }
    sprintf(buf, "%u", logPlayer.getNumberOfMessages());
    ctrl->printLn(std::string(buf) + "\ttotal");
    return true;
  }
  else if (command == "mr") //log mr
  {
    SYNC;
    std::string param;
    stream >> param;

    int upperFrequencies[numOfDataMessageIDs];
    int lowerFrequencies[numOfDataMessageIDs];
    int motionFrequencies[numOfDataMessageIDs];
    logPlayer.statistics(upperFrequencies, nullptr, 'c');
    logPlayer.statistics(lowerFrequencies, nullptr, 'd');
    logPlayer.statistics(motionFrequencies, nullptr, 'm');

    std::list<std::string> commands;
    const auto cLog = std::find(moduleInfo.modules.begin(), moduleInfo.modules.end(), "CognitionLogDataProvider");
    const auto mLog = std::find(moduleInfo.modules.begin(), moduleInfo.modules.end(), "MotionLogDataProvider");
    for (int i = idProcessFinished + 1; i < numOfDataMessageIDs; ++i)
    {
      if (upperFrequencies[i] || lowerFrequencies[i] || motionFrequencies[i])
      {
        std::string representation = std::string(::getName(MessageID(i))).substr(2);
        if (representation == "JPEGImage" || representation == "LowFrameRateImage" || representation == "Thumbnail" || representation == "YoloInput") // || representation == "SequenceImage"
          representation = "Image";
        if (representation == "JPEGImageUpper" || representation == "LowFrameRateImageUpper" || representation == "ThumbnailUpper" || representation == "YoloInputUpper") // || representation == "SequenceImageUpper"
          representation = "ImageUpper";
        bool inCognition = cLog != moduleInfo.modules.end() && std::find(cLog->representations.begin(), cLog->representations.end(), representation) != cLog->representations.end()
            && (upperFrequencies[i] || lowerFrequencies[i]);
        bool inMotion = mLog != moduleInfo.modules.end() && std::find(mLog->representations.begin(), mLog->representations.end(), representation) != mLog->representations.end()
            && motionFrequencies[i];
        if (inCognition || inMotion)
          commands.push_back(representation + " default");
        if (inCognition)
          commands.push_back(representation + " " + cLog->name);
        if (inMotion)
          commands.push_back(representation + " " + mLog->name);
      }
    }

    bool success = true;
    for (std::list<std::string>::const_iterator i = commands.begin(); i != commands.end(); ++i)
      if (param == "list")
        ctrl->printLn("mr " + *i);
      else
      {
        InTextMemory strMem(i->c_str(), i->size());
        success &= moduleRequest(strMem);
      }
    return success;
  }
  else if (logFile != "")
  {
    SYNC;
    if (command == "load")
    {
      std::string name;
      stream >> name;
      if (name.size() == 0)
        return false;
      else
      {
        if ((int)name.rfind('.') <= (int)name.find_last_of("\\/"))
          name = name + ".log";
        if (name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
          name = std::string("Logs\\") + name;
        logFile = name;
        LogPlayer::LogPlayerState state = logPlayer.state;
        bool result = logPlayer.open(name.c_str());
        if (result)
          logPlayer.handleAllMessages(annotationInfos['c']);
        if (result && state == LogPlayer::playing)
          logPlayer.play();
        return result;
      }
    }
    else if (command == "cycle")
    {
      logPlayer.setLoop(true);
      return true;
    }
    else if (command == "once")
    {
      logPlayer.setLoop(false);
      return true;
    }
    else if (command == "pause")
    {
      logPlayer.pause();
      return true;
    }
    else if (command == "forward")
    {
      std::string opt;
      stream >> opt;
      if (opt == "image")
        logPlayer.stepImageForward();
      else
        logPlayer.stepForward();
      return true;
    }
    else if (command == "backward")
    {
      std::string opt;
      stream >> opt;
      if (opt == "image")
        logPlayer.stepImageBackward();
      else
        logPlayer.stepBackward();
      return true;
    }
    else if (command == "repeat")
    {
      logPlayer.stepRepeat();
      return true;
    }
    else if (command == "goto")
    {
      LogPlayer::LogPlayerState state = logPlayer.state;

      int frame;
      stream >> frame;
      logPlayer.gotoFrame(std::max<>(std::min<>(frame - 1, logPlayer.numberOfFrames - 1), 0));
      if (state == LogPlayer::playing)
        logPlayer.play();
      return true;
    }
    else if (command == "fast_forward")
    {
      std::string opt;
      stream >> opt;

      //backup state, gotoFrame will change the state.
      LogPlayer::LogPlayerState state = logPlayer.state;
      int frame = logPlayer.currentFrameNumber + 100;

      logPlayer.gotoFrame(std::max<>(std::min<>(frame, logPlayer.numberOfFrames - 1), 0));
      if (opt == "image")
        logPlayer.stepImageForward();
      if (state == LogPlayer::playing)
      {
        //if the state was playing before, continue playing
        logPlayer.play();
      }
      return true;
    }
    else if (command == "fast_rewind")
    {
      std::string opt;
      stream >> opt;

      //backup state, gotoFrame will change the state.
      LogPlayer::LogPlayerState state = logPlayer.state;
      int frame = logPlayer.currentFrameNumber - 100;

      logPlayer.gotoFrame(std::max<>(std::min<>(frame, logPlayer.numberOfFrames - 1), 0));
      if (opt == "image")
        logPlayer.stepImageBackward();
      if (state == LogPlayer::playing)
      {
        //if the state was playing before, continue playing
        logPlayer.play();
      }
      return true;
    }
    else if (command == "export")
    {
      std::string name, par;
      std::list<std::string> ids;
      stream >> name;

      while (!stream.eof())
      {
        stream >> par;
        ids.push_back(par);
      }


      if (name.size() == 0)
        return false;
      else
      {
        logPlayer.export_data(name, ids);
        return true;
      }
    }
  }
  return false;
}

bool RobotConsole::get(In& stream, bool first, bool print)
{
  std::string request, option;
  stream >> request >> option;
  if (request == "?")
  {
    for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if (debugRequestTable.debugRequests[i].description.substr(0, 11) == "debug data:")
        ctrl->list(ctrl->translate(debugRequestTable.debugRequests[i].description.substr(11)), option);
    ctrl->printLn("");
    return true;
  }
  else
    for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if (std::string("debugData:") + request == ctrl->translate(debugRequestTable.debugRequests[i].description))
      {
        if (first)
        {
          // request up-to-date data
          if (!debugRequestTable.debugRequests[i].enable)
          {
            SYNC;
            debugOut.out.bin << DebugRequest(debugRequestTable.debugRequests[i].description, true);
            debugOut.out.finishMessage(idDebugRequest);
            waitingFor[idDebugDataResponse] = 1;
            getOrSetWaitsFor = debugRequestTable.debugRequests[i].description.substr(11);
          }
          polled[idDebugDataResponse] = true; // no automatic repolling
          handleConsole(std::string(print ? "_get " : "_get3 ") + request + " " + option);
          return true;
        }
        else
        {
          getOrSetWaitsFor = "";
          DebugDataInfos::const_iterator j = debugDataInfos.find(debugRequestTable.debugRequests[i].description.substr(11));
          ASSERT(j != debugDataInfos.end());
          if (option == "?")
          {
            printType(j->second.first.c_str());
            ctrl->printLn("");
            return true;
          }
          else if (option == "")
          {
            SYNC;
            OutMapSize size(true);
            MapWriter sizeWriter(streamHandler, size);
            j->second.second->handleAllMessages(sizeWriter);
            char* buf = new char[size.getSize()];
            OutMapMemory memory(buf, true);
            MapWriter memoryWriter(streamHandler, memory);
            j->second.second->handleAllMessages(memoryWriter);
            buf[size.getSize() - 1] = 0; // overwrite final space
            std::string buffer = "set " + request + " " + buf;
            delete[] buf;
            if (print)
              ctrl->printLn(buffer);
            else
              printBuffer = buffer;
            return true;
          }
        }
        break;
      }
  return false;
}

bool RobotConsole::RepViewWriter::handleMessage(InMessage& message)
{
  std::string name, type;
  message.bin >> name >> type;

  return handleMessage(message, type, name);
}

bool RobotConsole::RepViewWriter::handleMessage(InMessage& message, const std::string& type, const std::string& name)
{
  std::map<std::string, DataView*>::const_iterator view = pRepViews->find(name);
  ASSERT(message.getMessageID() == idDebugDataResponse);
  return view != pRepViews->end() && view->second->handleMessage(message, type, name);
}

bool RobotConsole::set(In& stream)
{
  std::string request, option;
  stream >> request >> option;
  if (request == "?")
  {
    for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if (debugRequestTable.debugRequests[i].description.substr(0, 11) == "debug data:")
        ctrl->list(ctrl->translate(debugRequestTable.debugRequests[i].description.substr(11)), option);
    ctrl->printLn("");
    return true;
  }
  else
    for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if (std::string("debugData:") + request == ctrl->translate(debugRequestTable.debugRequests[i].description))
      {
        if (option == "unchanged")
        {
          SYNC;
          debugOut.out.bin << debugRequestTable.debugRequests[i].description.substr(11) << char(0);
          debugOut.out.finishMessage(idDebugDataChangeRequest);
          return true;
        }
        else
        {
          char buf[10000];
          OutTextMemory temp(buf);
          temp << option;
          bool singleValue = true;
          while (!stream.eof())
          {
            singleValue = false;
            std::string text;
            stream >> text;
            temp << text;
          }
          ASSERT(temp.getLength() < (int)sizeof(buf) - 1);
          buf[temp.getLength()] = 0;
          std::string line(buf);
          DebugDataInfos::const_iterator j = debugDataInfos.find(debugRequestTable.debugRequests[i].description.substr(11)); //the substr(11) removes "debug data:" from the descrption string
          if (j == debugDataInfos.end())
          {
            // request type specification
            {
              SYNC;
              debugOut.out.bin << DebugRequest(debugRequestTable.debugRequests[i].description, true);
              debugOut.out.finishMessage(idDebugRequest);
              waitingFor[idDebugDataResponse] = 1;
              polled[idDebugDataResponse] = true; // no automatic repolling
              getOrSetWaitsFor = debugRequestTable.debugRequests[i].description.substr(11);
            }
            handleConsole(std::string("_set ") + request + " " + line);
            return true;
          }
          else
          {
            getOrSetWaitsFor = "";
            if (option == "?")
            {
              printType(j->second.first.c_str());
              ctrl->printLn("");
              return true;
            }
            else
            {
              SYNC;
              if (singleValue)
                line = "value = " + line + ";";
              MessageQueue errors;
              GlobalGuard g({.debugOut = &errors.out});
              InMapMemory lineStream(line.c_str(), line.size());
              if (!lineStream.eof())
              {
                debugOut.out.bin << debugRequestTable.debugRequests[i].description.substr(11) << char(1);
                DebugDataStreamer streamer(streamHandler, debugOut.out.bin, j->second.first, singleValue ? "value" : 0);
                lineStream >> streamer;
                if (errors.isEmpty())
                {
                  debugOut.out.finishMessage(idDebugDataChangeRequest);
                  return true;
                }
                else
                  debugOut.out.cancelMessage();
              }
              Printer printer(ctrl);
              errors.handleAllMessages(printer);
              return !errors.isEmpty(); // return true if error was already printed
            }
          }
        }
        break;
      }
  return false;
}

void RobotConsole::printType(std::string type, const char* field)
{
  if (type.size() > 8 && type.substr(type.size() - 8) == " __ptr64")
    type = type.substr(0, type.size() - 8);

  if (type[type.size() - 1] == ']')
  {
    size_t index = type.size();
    while (type[index - 1] != '[')
      --index;
    size_t index2 = type[index - 2] == ' ' ? index - 2 : index - 1;
    if (type[index2 - 1] == ')')
    {
      index2 -= type[index2 - 2] == '4' ? 8 : 0; // " __ptr64"
      index2 -= type[index2 - 4] == ' ' ? 4 : 3;
    }
    printType(type.substr(0, index2), (std::string(field) + type.substr(index - 1)).c_str());
  }
  else if (type[type.size() - 1] == '*')
    printType(std::string(type).substr(0, type.size() - (type.size() > 1 && type[type.size() - 2] == ' ' ? 2 : 1)).c_str(), (std::string(field) + "[]").c_str());
  else
  {
    if (type.size() > 6 && type.substr(type.size() - 6) == " const")
      type = type.substr(0, type.size() - 6);
    const char* t = streamHandler.getString(type.c_str());
    StreamHandler::Specification::const_iterator i = streamHandler.specification.find(t);
    StreamHandler::BasicTypeSpecification::const_iterator b = streamHandler.basicTypeSpecification.find(t);
    StreamHandler::EnumSpecification::const_iterator e = streamHandler.enumSpecification.find(t);
    if (i != streamHandler.specification.end())
    {
      if (*field)
        ctrl->print(std::string(field) + " : {");
      for (std::vector<StreamHandler::TypeNamePair>::const_iterator j = i->second.begin(); j != i->second.end();)
      {
        printType(j->second, j->first.c_str());
        if (++j != i->second.end())
          ctrl->print("; ");
        else
          ctrl->print(";");
      }
      if (*field)
        ctrl->print("}");
    }
    else if (b != streamHandler.basicTypeSpecification.end() || e != streamHandler.enumSpecification.end())
      ctrl->print((*field ? std::string(field) + " : " : "") + t);
    else
      ctrl->print((*field ? std::string(field) + " : " : "") + "UNKNOWN");
  }
}

bool RobotConsole::sendKfm(In& stream)
{
  std::string parameter;
  stream >> parameter;
  if (parameter != "")
    return false;

  std::vector<KeyFrameMotion> kfms = KeyFrameEngine::loadKeyFrameMotions();

  STREAM_EXT(debugOut.out.bin, kfms);
  debugOut.out.finishMessage(idKeyFrameMotions);

  return true;
}

bool RobotConsole::sendWek(In& stream)
{
  std::string parameter;
  stream >> parameter;
  if (parameter != "")
    return false;

  /*for(int i = 1; i < WalkRequest::numOfKickTypes; i += 2)
  {
    char filePath[256];
    sprintf(filePath, "Kicks/%s.cfg", WalkRequest::getName(WalkRequest::KickType(i)));
    File file(filePath, "rb");
    if(!file.exists())
      continue;
    size_t size = file.getSize();
    char* buffer = new char[size];
    file.read(buffer, size);
    {
      SYNC;
      debugOut.out.bin << i << (unsigned) size;
      debugOut.out.bin.write(buffer, size);
      debugOut.out.finishMessage(idWalkingEngineKick);
    }
    delete[] buffer;
  }
  */
  return true;
}

bool RobotConsole::repoll(In& stream)
{
  polled[idDebugResponse] = polled[idDrawingManager] = polled[idDrawingManager3D] = false;
  return true;
}

bool RobotConsole::queueFillRequest(In& stream)
{
  std::string request;
  stream >> request;
  QueueFillRequest qfr;
  if (request == "queue")
  {
    qfr.behavior = QueueFillRequest::sendImmediately;
    qfr.filter = QueueFillRequest::sendEverything;
    qfr.target = QueueFillRequest::sendViaNetwork;
  }
  else if (request == "replace")
  {
    qfr.behavior = QueueFillRequest::sendImmediately;
    qfr.filter = QueueFillRequest::latestOnly;
    qfr.target = QueueFillRequest::sendViaNetwork;
  }
  else if (request == "reject")
  {
    qfr.behavior = QueueFillRequest::discardAll;
  }
  else if (request == "collect")
  {
    qfr.behavior = QueueFillRequest::sendAfter;
    qfr.filter = QueueFillRequest::sendEverything;
    qfr.target = QueueFillRequest::sendViaNetwork;

    stream >> qfr.timingMilliseconds;
    qfr.timingMilliseconds *= 1000;
    if (!qfr.timingMilliseconds)
      return false;
  }
  else if (request == "save")
  {
    qfr.filter = QueueFillRequest::sendEverything;
    qfr.target = QueueFillRequest::writeToStick;

    stream >> qfr.timingMilliseconds;
    qfr.timingMilliseconds *= 1000;
    if (!qfr.timingMilliseconds)
      qfr.behavior = QueueFillRequest::sendImmediately;
    else
      qfr.behavior = QueueFillRequest::sendAfter;
  }
  else
    return false;
  SYNC;
  debugOut.out.bin << qfr;
  debugOut.out.finishMessage(idQueueFillRequest);
  return true;
}

bool RobotConsole::moduleRequest(In& stream)
{
  SYNC;
  std::string representation, module, pattern;
  stream >> representation >> module >> pattern;
  if (representation == "modules")
  {
    for (const auto& m : moduleInfo.modules)
    {
      if (m.name == module)
      {
        std::string text = m.name + " (" + ModuleBase::getName(m.category) + "):\n";
        for (const auto& r : m.requirements)
        {
          for (const auto& rp : moduleInfo.config.representationProviders)
            if (rp.representation == r)
            {
              text += "     " + r + " (" + rp.provider + ")\n";
              break;
            }
        }

        text += "-> ";
        for (const auto& r : m.representations)
        {
          bool selected = false;
          for (const auto& rp : moduleInfo.config.representationProviders)
            selected |= rp.provider == m.name;
          text += r + (selected ? "* " : " ");
        }
        ctrl->list(text, pattern);
      }
    }

    return true;
  }
  else if (representation == "?")
  {
    for (const auto& rp : moduleInfo.config.representationProviders)
    {
      std::string process = "both";
      for (const auto& m : moduleInfo.modules)
        if (m.name == rp.provider)
          process = m.processIdentifier != 'c' ? "Motion" : "Cognition";
      std::string text = rp.representation + " (" + process + "): ";
      for (const auto& m : moduleInfo.modules)
        if (std::find(m.representations.begin(), m.representations.end(), rp.representation) != m.representations.end())
          text += m.name + (m.name == rp.provider ? "* " : " ");
      text += rp.provider == "default" ? "default*" : "default";
      ctrl->list(text, module, true);
    }
    return true;
  }
  else if (representation == "save")
  {
    OutMapFile modulesStream("modules.cfg");
    moduleInfo.sendRequest(modulesStream, true);
    return true;
  }
  else if (representation == "off" || representation == "default")
  {
    if (std::find(moduleInfo.modules.begin(), moduleInfo.modules.end(), module) == moduleInfo.modules.end())
      return false;

    for (auto i = moduleInfo.config.representationProviders.begin(); i != moduleInfo.config.representationProviders.end();)
    {
      if (i->provider == module)
      {
        if (representation == "off")
        {
          i = moduleInfo.config.representationProviders.erase(i);
          continue;
        }
        else
          i->provider = representation;
      }
      ++i;
    }

    moduleInfo.timeStamp = SystemCall::getCurrentSystemTime() + ++mrCounter;
    debugOut.out.bin << moduleInfo.timeStamp;
    moduleInfo.sendRequest(debugOut.out.bin);
    debugOut.out.finishMessage(idModuleRequest);
    polled[idDebugResponse] = polled[idDrawingManager] = polled[idDrawingManager3D] = false;
    logPlayer.streamSpecificationReplayed = false;
    return true;
  }
  else
  {
    if (std::find(moduleInfo.representations.begin(), moduleInfo.representations.end(), representation) == moduleInfo.representations.end())
      return false;
    else if (module == "?")
    {
      std::string provider;
      for (const auto& rp : moduleInfo.config.representationProviders)
        if (rp.representation == representation)
          provider = rp.provider;
      for (const auto& m : moduleInfo.modules)
        if (std::find(m.representations.begin(), m.representations.end(), representation) != m.representations.end())
          ctrl->list(m.name + (m.name == provider ? "*" : ""), pattern);
      ctrl->list(std::string("default") + ("default" == provider ? "*" : ""), pattern);
      ctrl->printLn("");
      return true;
    }
    else
    {
      char process = 0;
      if (module != "off" && module != "default")
        for (const auto& m : moduleInfo.modules)
          if (m.name == module)
          {
            process = m.processIdentifier;
            break;
          }

      for (auto i = moduleInfo.config.representationProviders.begin(); i != moduleInfo.config.representationProviders.end();)
      {
        if (i->representation == representation)
        {
          char processOld = 0;
          if (process)
            for (const auto& m : moduleInfo.modules)
              if (m.name == i->provider)
              {
                processOld = m.processIdentifier;
                break;
              }
          if (!process || !processOld || process == processOld)
          {
            i = moduleInfo.config.representationProviders.erase(i);
            continue;
          }
        }
        ++i;
      }

      if (module != "off")
        moduleInfo.config.representationProviders.push_back(ModuleManager::Configuration::RepresentationProvider(representation, module));

      moduleInfo.timeStamp = SystemCall::getCurrentSystemTime() + ++mrCounter;
      debugOut.out.bin << moduleInfo.timeStamp;
      moduleInfo.sendRequest(debugOut.out.bin);
      debugOut.out.finishMessage(idModuleRequest);
      polled[idDebugResponse] = polled[idDrawingManager] = polled[idDrawingManager3D] = false;
      logPlayer.streamSpecificationReplayed = false;
      return true;
    }
  }
  return false;
}

bool RobotConsole::moveRobot(In& stream)
{
  SYNC;
  stream >> movePos.x() >> movePos.y() >> movePos.z();
  if (stream.eof())
    moveOp = movePosition;
  else
  {
    stream >> moveRot.x() >> moveRot.y() >> moveRot.z();
    moveOp = moveBoth;
  }
  return true;
}

bool RobotConsole::moveBall(In& stream)
{
  SYNC;
  stream >> movePos.x() >> movePos.y() >> movePos.z();
  moveOp = moveBallPosition;
  return true;
}

bool RobotConsole::kickBall(In& stream)
{
  SYNC;
  float arg1, arg2;
  stream >> arg1 >> arg2;
  if (stream.eof())
  {
    //2 parameters variant (float kickAngleDeg, float kickVelocity)
    float kickAngleDeg = arg1;
    kickVelocity = arg2;
    kickAngle = Angle::fromDegrees(kickAngleDeg);
    moveOp = kickBallVelocity;
  }
  else
  {
    //3 parameters variant (float targetX, float targetY, float kickVelocity)
    Vector2f targetPos = Vector2f(arg1, arg2);
    stream >> kickVelocity;
    Vector2f ballPos;
    SimulatedRobot::getAbsoluteBallPosition(ballPos);
    Vector2f diff = targetPos - ballPos;
    if (diff.norm() > 0.001)
    {
      kickAngle = diff.angle();
      moveOp = kickBallVelocity;
    }
  }
  return true;
}

RemoteControlRequest& RobotConsole::getRemoteControlRequest()
{
  return data->remoteControlRequest;
}

void RobotConsole::sendRemoteControlRequest()
{
  debugOut.out.bin << data->remoteControlRequest;
  debugOut.out.finishMessage(idRemoteControlRequest);
}

bool RobotConsole::remoteControl(In& stream)
{
  SYNC;
  std::string command;
  stream >> command;

  if (command == "on")
  {
    data->remoteControlRequest.command = RemoteControlRequest::Command::enable;
    data->remoteControlRequest.handled = false;
  }
  else if (command == "off")
  {
    data->remoteControlRequest.command = RemoteControlRequest::Command::disable;
    data->remoteControlRequest.handled = false;
  }
  else if (command == "walk")
  {
    stream >> data->remoteControlRequest.target.translation.x() >> data->remoteControlRequest.target.translation.y() >> data->remoteControlRequest.target.rotation;
    data->remoteControlRequest.command = RemoteControlRequest::Command::walk;
    data->remoteControlRequest.handled = false;
  }
  else if (command == "pass")
  {
    stream >> data->remoteControlRequest.target.translation.x() >> data->remoteControlRequest.target.translation.y() >> data->remoteControlRequest.target.rotation;
    data->remoteControlRequest.command = RemoteControlRequest::Command::pass;
    data->remoteControlRequest.handled = false;
  }
  else if (command == "kick")
  {
    stream >> data->remoteControlRequest.target.translation.x() >> data->remoteControlRequest.target.translation.y() >> data->remoteControlRequest.target.rotation;
    data->remoteControlRequest.command = RemoteControlRequest::Command::kick;
    data->remoteControlRequest.handled = false;
  }
  else if (command == "offensive")
  {
    data->remoteControlRequest.command = RemoteControlRequest::Command::enforceOffensiveRoles;
    data->remoteControlRequest.handled = false;
  }
  else if (command == "defensive")
  {
    data->remoteControlRequest.command = RemoteControlRequest::Command::enforceDefensiveRoles;
    data->remoteControlRequest.handled = false;
  }
  else if (command == "kickPref")
  {
    data->remoteControlRequest.command = RemoteControlRequest::Command::kickPreference;
    data->remoteControlRequest.handled = false;
  }
  else if (command == "passPref")
  {
    data->remoteControlRequest.command = RemoteControlRequest::Command::passPreference;
    data->remoteControlRequest.handled = false;
  }
  else
  {
    return false;
  }

  debugOut.out.bin << data->remoteControlRequest;
  debugOut.out.finishMessage(idRemoteControlRequest);
  return true;
}

bool RobotConsole::sleepConsole(In& stream)
{
  SYNC;
  unsigned sleepFrames;
  stream >> sleepFrames;
  if (sleepTimer == 0)
  {
    sleepTimer = sleepFrames;
  }
  else
  {
    sleepTimer--;
  }
  return true;
}

void RobotConsole::printLn(const std::string& line)
{
  if (nullptr != ctrl)
  {
    ctrl->printLn(line);
  }
}

bool RobotConsole::view3D(In& stream)
{
  std::string buffer;
  stream >> buffer;
  if (buffer == "?")
  {
    stream >> buffer;
    ctrl->list("image", buffer);
    for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if (debugRequestTable.debugRequests[i].description.substr(0, 13) == "debug images:")
        ctrl->list(ctrl->translate(debugRequestTable.debugRequests[i].description.substr(13)), buffer);
    ctrl->printLn("");
    return true;
  }
  else
  {
    std::string buffer2;
    bool jpeg = false;
    bool upperCam = false;
    for (;;)
    {
      stream >> buffer2;
      if (!jpeg && buffer2 == "jpeg")
        jpeg = true;
      else if (!upperCam && buffer2 == "upper")
        upperCam = true;
      else
        break;
    }

    if (buffer == "image" || buffer == "imageUpper")
    {
      upperCam = (buffer != "image");
      std::string name = buffer2 != "" ? buffer2 : std::string("image");
      if (imageViews3D.find(name) != imageViews3D.end())
      {
        ctrl->printLn("View already exists. Specify a (different) name.");
        return true;
      }
      imageViews3D[name];
      addColorSpaceViews("raw " + buffer, name, false, upperCam);
      if (jpeg)
        upperCam ? handleConsole("dr representation:JPEGImageUpper on") : handleConsole("dr representation:JPEGImage on");
      else
        upperCam ? handleConsole("dr representation:ImageUpper on") : handleConsole("dr representation:Image on");
      return true;
    }
    else if (!jpeg)
      for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
        if (debugRequestTable.debugRequests[i].description.substr(0, 13) == "debug images:" && ctrl->translate(debugRequestTable.debugRequests[i].description.substr(13)) == buffer)
        {
          std::string name = buffer2 != "" ? buffer2 : std::string(buffer);
          if (imageViews3D.find(name) != imageViews3D.end())
          {
            ctrl->printLn("View already exists. Specify a (different) name.");
            return true;
          }
          addColorSpaceViews(debugRequestTable.debugRequests[i].description.substr(13), name, true, upperCam);
          handleConsole(std::string("dr ") + ctrl->translate(debugRequestTable.debugRequests[i].description) + " on");
          return true;
        }
  }
  return false;
}

bool RobotConsole::viewField(In& stream)
{
  std::string name;
  stream >> name;
  if (fieldViews.find(name) != fieldViews.end())
    ctrl->printLn("View already exists. Specify a different name.");
  else
  {
    fieldViews[name];
    ctrl->setFieldViews(fieldViews);
    ctrl->updateCommandCompletion();
    ctrl->addView(new FieldView(robotName + ".field." + name.c_str(), *this, name), robotName + ".field", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
  }
  return true;
}

bool RobotConsole::viewData(In& stream)
{
  std::string name, option;
  stream >> name >> option;
  for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
    if (std::string("visualizeData:") + name == ctrl->translate(debugRequestTable.debugRequests[i].description))
    {
      // enable the debug request if it is not already enabled
      DebugRequest& d = debugRequestTable.debugRequests[i];
      if (option == "off")
      {
        d.enable = false;
        return true;
      }
      else if (option == "on" || option == "")
      {
        if (representationViews.find(name) == representationViews.end())
        {
          representationViews[name] = new DataView(robotName + ".data." + name.c_str(), name, *this, streamHandler);
          ctrl->addView(representationViews[name], robotName + ".data", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
        }

        d.enable = true;

        {
          SYNC;
          debugOut.out.bin << d;
          debugOut.out.finishMessage(idDebugRequest);
        }

        return true;
      }
      else
        return false;
    }
  return false;
}

bool RobotConsole::viewDrawing(In& stream, RobotConsole::Views& views, const char* type)
{
  std::string buffer;
  stream >> buffer;
  if (buffer == "?")
  {
    stream >> buffer;
    for (Views::const_iterator i = views.begin(); i != views.end(); ++i)
      ctrl->list(i->first, buffer);
    ctrl->printLn("");
    return true;
  }
  else
    for (Views::const_iterator i = views.begin(); i != views.end(); ++i)
      if (i->first == buffer)
      {
        stream >> buffer;
        if (buffer == "?")
        {
          stream >> buffer;
          for (std::unordered_map<const char*, DrawingManager::Drawing>::const_iterator j = drawingManager.drawings.begin(); j != drawingManager.drawings.end(); ++j)
            if (!strcmp(drawingManager.getDrawingType(j->first), type))
              ctrl->list(ctrl->translate(j->first), buffer);
          ctrl->printLn("");
          return true;
        }
        else
        {
          for (std::unordered_map<const char*, DrawingManager::Drawing>::const_iterator j = drawingManager.drawings.begin(); j != drawingManager.drawings.end(); ++j)
            if (ctrl->translate(j->first) == buffer && !strcmp(drawingManager.getDrawingType(j->first), type))
            {
              std::string buffer2;
              stream >> buffer2;
              if (buffer2 == "on" || buffer2 == "")
              {
                if (std::string(j->first).substr(0, 7) != "origin:")
                  views[i->first].remove(j->first);
                views[i->first].push_back(j->first);
                handleConsole(std::string("dr debugDrawing:") + buffer + " on");
                return true;
              }
              else if (buffer2 == "off")
              {
                views[i->first].remove(j->first);
                handleConsole(std::string("dr debugDrawing:") + buffer + " off");
                return true;
              }
              else
                return false;
            }
        }
      }
  return false;
}

bool RobotConsole::viewImage(In& stream)
{
  std::string buffer;
  stream >> buffer;
  if (buffer == "?")
  {
    stream >> buffer;
    ctrl->list("none", buffer);
    ctrl->list("image", buffer);
    ctrl->list("imageUpper", buffer);
    for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if (debugRequestTable.debugRequests[i].description.substr(0, 13) == "debug images:")
        ctrl->list(ctrl->translate(debugRequestTable.debugRequests[i].description.substr(13)), buffer);
    ctrl->printLn("");
    return true;
  }
  else if (buffer == "none")
  {
    bool upperCam = false;
    stream >> buffer;
    if (buffer == "upper")
    {
      upperCam = true;
      stream >> buffer;
    }

    std::string name = buffer != "" ? buffer : "none";
    if (imageViews.find(name) != imageViews.end())
    {
      ctrl->printLn("View already exists. Specify a (different) name.");
      return true;
    }
    imageViews[name];
    ctrl->setImageViews(imageViews);
    ctrl->updateCommandCompletion();
    ctrl->addView(new ImageView(robotName + ".image." + name.c_str(), *this, "none", name, false, upperCam), robotName + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
    return true;
  }
  else
  {
    std::string buffer2;
    bool jpeg = false, segmented = false, upperCam = false;
    for (;;)
    {
      stream >> buffer2;
      if (!jpeg && buffer2 == "jpeg")
        jpeg = true;
      else if (!segmented && buffer2 == "segmented")
        segmented = true;
      else
        break;
    }

    if (buffer == "image" || buffer == "imageUpper")
    {
      std::string name = buffer2 != "" ? buffer2 : buffer + (segmented ? "Segmented" : "");
      if (imageViews.find(name) != imageViews.end())
      {
        ctrl->printLn("View already exists. Specify a (different) name.");
        return true;
      }
      upperCam = (buffer != "image");
      imageViews[name];
      ctrl->setImageViews(imageViews);
      ctrl->updateCommandCompletion();
      std::string enableGain;
      stream >> enableGain;
      float gain = 1.0f;
      if (enableGain == "gain")
      {
        stream >> gain;
      }
      actualImageViews[name] = new ImageView(robotName + ".image." + name.c_str(), *this, ("raw " + buffer).c_str(), name, segmented, upperCam, gain);
      ctrl->addView(actualImageViews[name], robotName + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
      if (segmented)
        segmentedImageViews.emplace_back(actualImageViews[name]);
      if (jpeg)
        handleConsole("dr representation:JPEGI" + buffer.substr(1) + " on");
      else
        handleConsole("dr representation:I" + buffer.substr(1) + " on");

      return true;
    }
    else if (!jpeg)
      for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
        if (debugRequestTable.debugRequests[i].description.substr(0, 13) == "debug images:" && ctrl->translate(debugRequestTable.debugRequests[i].description.substr(13)) == buffer)
        {
          std::string name = buffer2 != "" ? buffer2 : std::string(buffer) + (segmented ? "Segmented" : "");
          if (imageViews.find(name) != imageViews.end())
          {
            ctrl->printLn("View already exists. Specify a (different) name.");
            return true;
          }
          imageViews[name];
          ctrl->setImageViews(imageViews);
          ctrl->updateCommandCompletion();
          actualImageViews[name] = new ImageView(robotName + ".image." + name.c_str(), *this, debugRequestTable.debugRequests[i].description.substr(13), name, false, upperCam);
          ctrl->addView(actualImageViews[name], robotName + ".image", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
          handleConsole(std::string("dr ") + ctrl->translate(debugRequestTable.debugRequests[i].description) + " on");
          return true;
        }
  }
  return false;
}

bool RobotConsole::viewPlot(In& stream)
{
  std::string name;
  int plotSize;
  float minValue;
  float maxValue;
  std::string yUnit;
  std::string xUnit;
  float xScale;
  stream >> name >> plotSize >> minValue >> maxValue >> yUnit >> xUnit >> xScale;
  if (plotSize < 2 || minValue >= maxValue)
    return false;
  if (xScale == 0.)
    xScale = 1.;
  QString fullName = robotName + ".plot." + name.c_str();
  if ((unsigned int)plotSize > maxPlotSize)
    maxPlotSize = plotSize;
  if (plotViews.find(name) != plotViews.end())
  {
    PlotView* plotView = (PlotView*)ctrl->application->resolveObject(fullName);
    ASSERT(plotView);
    plotView->setParameters((unsigned int)plotSize, minValue, maxValue, yUnit, xUnit, xScale);
    return true;
  }
  plotViews[name];
  ctrl->setPlotViews(plotViews);
  ctrl->updateCommandCompletion();
  ctrl->addView(new PlotView(fullName, *this, name, (unsigned int)plotSize, minValue, maxValue, yUnit, xUnit, xScale), robotName + ".plot", SimRobot::Flag::copy | SimRobot::Flag::exportAsImage);
  return true;
}

bool RobotConsole::kickView()
{
  if (kickViewSet)
    ctrl->printLn("View already exists.");
  else
  {
    if (mode == SystemCall::simulatedRobot)
    {
      kickViewSet = true;
      ctrl->addView(
          new KickView(
              robotName + ".KickView", *this, data->motionRequest, data->jointSensorData, data->jointCalibration, data->robotDimensions, (SimRobotCore2::Body*)ctrl->application->resolveObject(robotFullName, SimRobotCore2::body)),
          robotName);
      return true;
    }
    if (mode == SystemCall::remoteRobot)
    {
      kickViewSet = true;
      QString puppetName("RoboCup.puppets." + robotName);

      ctrl->addView(
          new KickView(
              robotName + ".KickView", *this, data->motionRequest, data->jointSensorData, data->jointCalibration, data->robotDimensions, (SimRobotCore2::Body*)ctrl->application->resolveObject(puppetName, SimRobotCore2::body)),
          robotName);
      return true;
    }
  }
  return false;
}

void RobotConsole::sendDebugMessage(InMessage& msg)
{
  SYNC;
  msg >> debugOut;
}

std::string RobotConsole::getDebugRequest(const std::string& name)
{
  SYNC;
  for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
  {
    if (std::string("debugData:") + name == ctrl->translate(debugRequestTable.debugRequests[i].description))
    {
      return debugRequestTable.debugRequests[i].description.substr(11);
    }
  }
  printLn("Error: RobotConsole: DebugRequest not found.");
  return "";
}

bool RobotConsole::viewPlotDrawing(In& stream)
{
  std::string buffer;
  stream >> buffer;
  if (buffer == "?")
  {
    stream >> buffer;
    for (PlotViews::const_iterator i = plotViews.begin(); i != plotViews.end(); ++i)
      ctrl->list(i->first.c_str(), buffer);
    ctrl->printLn("");
    return true;
  }
  else
    for (PlotViews::const_iterator i = plotViews.begin(); i != plotViews.end(); ++i)
      if (i->first == buffer)
      {
        stream >> buffer;
        if (buffer == "?")
        {
          stream >> buffer;
          for (int j = 0; j < debugRequestTable.currentNumberOfDebugRequests; ++j)
            if (debugRequestTable.debugRequests[j].description.substr(0, 5) == "plot:")
              ctrl->list(ctrl->translate(debugRequestTable.debugRequests[j].description.substr(5)).c_str(), buffer);
          ctrl->printLn("");
          return true;
        }
        else
        {
          for (int j = 0; j < debugRequestTable.currentNumberOfDebugRequests; ++j)
            if (ctrl->translate(debugRequestTable.debugRequests[j].description) == std::string("plot:") + buffer)
            {
              Layer layer;
              layer.layer = buffer;
              stream >> buffer;
              if (buffer == "?")
              {
                stream >> buffer;
                for (int color = 0; color < numOfColors; ++color)
                  ctrl->list(ctrl->translate(getName((Color)color)).c_str(), buffer);
                ctrl->printLn("");
                return true;
              }
              if (buffer == "off")
              {
                {
                  SYNC;
                  plotViews[i->first].remove(layer);
                }
                handleConsole(std::string("dr plot:") + layer.layer + " off");
                return true;
              }
              else
              {
                if (buffer == "black")
                  layer.color = ColorRGBA::black;
                else if (buffer == "red")
                  layer.color = ColorRGBA::red;
                else if (buffer == "green")
                  layer.color = ColorRGBA::green;
                else if (buffer == "blue")
                  layer.color = ColorRGBA::blue;
                else if (buffer == "yellow")
                  layer.color = ColorRGBA::yellow;
                else if (buffer == "cyan")
                  layer.color = ColorRGBA::cyan;
                else if (buffer == "magenta")
                  layer.color = ColorRGBA::magenta;
                else if (buffer == "orange")
                  layer.color = ColorRGBA::orange;
                else if (buffer == "violet")
                  layer.color = ColorRGBA::violet;
                else if (buffer == "gray")
                  layer.color = ColorRGBA::gray;
                else
                {
                  int c = 0;
                  sscanf(buffer.c_str(), "%x", &c);
                  layer.color = ColorRGBA((c >> 16) & 0xff, (c >> 8) & 0xff, c & 0xff);
                }
                stream >> layer.description;
                if (layer.description.empty())
                {
                  const char* name = strrchr(layer.layer.c_str(), ':');
                  if (name)
                    layer.description = name + 1;
                  else
                    layer.description = layer.layer;
                }
                {
                  SYNC;
                  plotViews[i->first].remove(layer);
                  plotViews[i->first].push_back(layer);
                }
                handleConsole(std::string("dr plot:") + layer.layer + " on");
                return true;
              }
              return false;
            }
        }
      }
  return false;
}

bool RobotConsole::joystickExecCommand(const std::string& cmd)
{
  if (cmd == "")
    return false;

  ctrl->executeConsoleCommand(cmd, this);
  if (joystickTrace)
    ctrl->printLn(cmd);

  return true;
}

void RobotConsole::handleJoystick()
{
  if (!joystick.update())
    return; //return if no joystick was found

  // handle joystick events
  unsigned int buttonId;
  bool pressed;
  bool buttonCommandExecuted(false);
  while (joystick.getNextEvent(buttonId, pressed))
  {
    ASSERT(buttonId < Joystick::numOfButtons);
    buttonCommandExecuted |= joystickExecCommand(pressed ? joystickButtonPressCommand[buttonId] : joystickButtonReleaseCommand[buttonId]);
    if (!pressed)
      for (int j = 0; j < joystickNumOfMotionCommands; ++j)
        joystickMotionCommands[j].lastCommand = "";
  }

  // walk and move head only when there is no button command
  if (buttonCommandExecuted)
    return;

  unsigned int timeNow = SystemCall::getCurrentSystemTime();
  if (lines.empty() && timeNow - joystickLastTime >= 100) // don't generate too many commands
  {
    joystickLastTime = timeNow;
    float speeds[Joystick::numOfAxes];
    bool preparedSpeeds = false;
    for (int j = 0; j < joystickNumOfMotionCommands; ++j)
    {
      JoystickMotionCommand& cmd(joystickMotionCommands[j]);
      if (!cmd.command.empty())
      {
        if (!preparedSpeeds)
        {
          for (int i = 0; i < Joystick::numOfAxes; ++i)
          {
            float d = joystick.getAxisState(i);
            float threshold = joystickAxisThresholds[i];
            if (d < -threshold)
              speeds[i] = (d + threshold) / (1 - threshold);
            else if (d > threshold)
              speeds[i] = (d - threshold) / (1 - threshold);
            else
              speeds[i] = 0;
            if (joystickAxisMappings[i])
            {
              bool pressed1 = joystick.isButtonPressed(joystickAxisMappings[i] & 0xffff);
              bool pressed2 = joystick.isButtonPressed(joystickAxisMappings[i] >> 16);
              if (pressed1 != pressed2)
                speeds[i] = pressed1 ? 1.f : -1.f;
            }
            speeds[i] *= joystickAxisMaxSpeeds[i];
            speeds[i] += joystickAxisCenters[i];
          }
          preparedSpeeds = true;
        }
        if (joystickCommandBuffer.size() < cmd.command.length() + 258)
          joystickCommandBuffer.resize(cmd.command.length() + 258);
        ASSERT(Joystick::numOfAxes == 8);
        sprintf(&joystickCommandBuffer[0],
            cmd.command.c_str(),
            speeds[cmd.indices[0]],
            speeds[cmd.indices[1]],
            speeds[cmd.indices[2]],
            speeds[cmd.indices[3]],
            speeds[cmd.indices[4]],
            speeds[cmd.indices[5]],
            speeds[cmd.indices[6]],
            speeds[cmd.indices[7]]);
        if (strcmp(cmd.lastCommand.c_str(), &joystickCommandBuffer[0]))
        {
          joystickExecCommand(&joystickCommandBuffer[0]);
          cmd.lastCommand = &joystickCommandBuffer[0];
        }
      }
    }
  }
}

bool RobotConsole::joystickCommand(In& stream)
{
  std::string command;
  stream >> command;
  if (command == "show")
  {
    joystickTrace = true;
    return true;
  }
  else if (command == "hide")
  {
    joystickTrace = false;
    return true;
  }

  int number;
  stream >> number;
  //rest of line into one string (std::stringstream-like .str() would be nice :-/):
  std::string line;
  stream >> line;
  while (!stream.eof())
  {
    std::string text;
    stream >> text;
    line += ' ';
    line += text;
  }

  if (command == "press" || command == "release")
  {
    if (number > 0 && number <= Joystick::numOfButtons)
    {
      if (command == "release")
        joystickButtonReleaseCommand[number - 1].swap(line);
      else
        joystickButtonPressCommand[number - 1].swap(line);
      return true;
    }
    return false;
  }
  else if (command == "motion")
  {
    if (number > 0 && number <= joystickNumOfMotionCommands)
    {
      JoystickMotionCommand& cmd(joystickMotionCommands[number - 1]);
      for (int i = 0; i < Joystick::numOfAxes; ++i)
        cmd.indices[i] = 0;
      std::string::size_type pos = line.find("$");
      int i = 0;
      while (i < Joystick::numOfAxes && pos != std::string::npos)
      {
        int id = line[pos + 1] - '1';
        if (id >= 0 && id < Joystick::numOfAxes)
        {
          cmd.indices[i++] = id;
          line.replace(pos, 2, "%lf");
          pos = line.find("$");
        }
        else
          return false;
      }
      cmd.command.swap(line);
      cmd.lastCommand.clear();
      return true;
    }
    return false;
  }
  return false;
}

bool RobotConsole::joystickSpeeds(In& stream)
{
  int id;
  stream >> id;
  if (id > 0 && id <= Joystick::numOfAxes)
  {
    stream >> joystickAxisMaxSpeeds[id - 1] >> joystickAxisThresholds[id - 1] >> joystickAxisCenters[id - 1];
    return true;
  }
  return false;
}

bool RobotConsole::joystickMaps(In& stream)
{
  int axis, button1, button2;
  stream >> axis >> button1 >> button2;
  if (axis > 0 && axis <= Joystick::numOfAxes && button1 >= 0 && button1 <= Joystick::numOfButtons && button2 > 0 && button2 <= Joystick::numOfButtons)
  {
    joystickAxisMappings[axis - 1] = button1 == 0 ? 0 : ((button1 - 1) | ((button2 - 1) << 16));
    return true;
  }
  return false;
}

bool RobotConsole::saveRequest(In& stream, bool first)
{
  std::string buffer;
  std::string path;
  stream >> buffer >> path;

  if (buffer == "?")
  {
    for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if (debugRequestTable.debugRequests[i].description.substr(0, 11) == "debug data:")
        ctrl->list(ctrl->translate(debugRequestTable.debugRequests[i].description.substr(11)), path);
    ctrl->printLn("");
    return true;
  }
  else
  {
    for (int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
      if (std::string("debugData:") + buffer == ctrl->translate(debugRequestTable.debugRequests[i].description))
      {
        if (first) // request current Values
        {
          SYNC;
          debugOut.out.bin << DebugRequest(debugRequestTable.debugRequests[i].description, true);
          debugOut.out.finishMessage(idDebugRequest);

          waitingFor[idDebugDataResponse] = 1;
          polled[idDebugDataResponse] = true; // no automatic repolling
          getOrSetWaitsFor = debugRequestTable.debugRequests[i].description.substr(11);
          handleConsole(std::string("_save ") + buffer + " " + path);
          return true;
        }
        else
        {
          getOrSetWaitsFor = "";
          const std::string representation = debugRequestTable.debugRequests[i].description.substr(11);
          DebugDataInfos::const_iterator j = debugDataInfos.find(representation);
          ASSERT(j != debugDataInfos.end());

          const std::string moduleConfig = getConfigFileForRepresentation(representation);
          std::string filename = path;
          if (path == "") // no path specified, use default location
          {
            filename = getPathForRepresentation(representation);
          }
          else if (path.back() == '/') // directory given
          {
            const std::string prefix = std::string(File::getBHDir()) + "/Config/";
            filename = prefix + path + moduleConfig;
          }

          std::filesystem::create_directories(std::filesystem::path(filename).parent_path());

          {
            OutMapFile file(filename);
            if (!file.exists())
            {
              ctrl->printLn("Unable to open " + filename + " for write.");
              return true;
            }
            MapWriter writer(streamHandler, file);
            j->second.second->handleAllMessages(writer);
          }

          std::list<std::string> names = File::getFullNamesHierarchy(moduleConfig);

          // keep full config file if first one in hierarchy
          if (names.front() == filename)
            return true;
          // keep full config file if not part of hierarchy
          if (std::find(names.begin(), names.end(), filename) == names.end())
            return true;

          std::shared_ptr<SimpleMap> map_with;
          {
            InMapFile infile_with(names);
            map_with = infile_with.getMap();
          }

          names.remove(filename);

          std::shared_ptr<SimpleMap> map_without;
          {
            InMapFile infile_without(names);
            map_without = infile_without.getMap();
          }

          *map_with -= *map_without;

          OutMapFile outfile(filename);
          outfile << *map_with;

          return true;
        }
      }
    return false;
  }
}

bool RobotConsole::saveImage(In& stream)
{
  std::string cam;
  stream >> cam;
  if (cam == "reset")
  {
    imageSaveNumber = 0;
    return true;
  }
  else
  {
    bool useUpperCam;
    if (cam == "upper")
      useUpperCam = true;
    else if (cam == "lower")
      useUpperCam = false;
    else
      return false;

    std::string filename;
    stream >> filename;
    int number = -1;
    if (filename == "number")
    {
      number = imageSaveNumber++;
      stream >> filename;
    }
    if (filename == "")
      filename = "raw_image_" + cam + ".bmp";

    SYNC;
    Image* srcImage;
    if (useUpperCam)
      srcImage = camImages["raw imageUpper"].get();
    else
      srcImage = camImages["raw image"].get();

    return srcImage && LogPlayer::saveImage(*srcImage, filename.c_str(), number);
  }
}

void RobotConsole::handleKeyEvent(int key, bool pressed)
{
  if (joystickTrace && pressed)
  {
    char buf[33];
    sprintf(buf, "%u", key + 1);
    ctrl->printLn(std::string("shortcut: ") + buf);
  }
  std::string* joystickButtonCommand(pressed ? joystickButtonPressCommand : joystickButtonReleaseCommand);
  if (key >= 0 && key < Joystick::numOfButtons && joystickButtonCommand[key] != "")
    ctrl->executeConsoleCommand(joystickButtonCommand[key], this);
}

std::string RobotConsole::getPathForRepresentation(const std::string& representation)
{
  // Check where the file has to be placed. Search order:
  // Robots/[robotname]
  // Robots/Default
  // the config directory, default return value

  std::string configName = getConfigFileForRepresentation(representation);
  std::list<std::string> names = File::getFullNames(configName);
  for (std::list<std::string>::const_iterator i = names.begin(); i != names.end(); ++i)
  {
    File path(*i, "r", false);
    if (path.exists())
      return *i;
  }

  // if file is not anywhere else, return config directory as default directory
  return names.back();
}

std::string RobotConsole::getConfigFileForRepresentation(const std::string& representation)
{
  std::string config;

  // check custom config names first
  auto file = ctrl->representationToFile.find(representation);
  if (file != ctrl->representationToFile.end())
  {
    config = file->second;
  }
  else
  {
    config = representation.substr(representation.find(":") + 1);
    config = getConfigName(config.c_str());
  }

  return config;
}

bool RobotConsole::acceptCamera(In& stream)
{
  std::string command;
  stream >> command;

  if (command == "?")
  {
    stream >> command;
    ctrl->list(drawingsViaProcess == 'b' ? "both*" : "both", command);
    ctrl->list(drawingsViaProcess == 'd' ? "lower*" : "lower", command);
    ctrl->list(drawingsViaProcess == 'c' ? "upper*" : "upper", command);
    ctrl->printLn("");
  }
  else
  {
    SYNC;
    if (command == "upper")
      drawingsViaProcess = 'c';
    else if (command == "lower")
      drawingsViaProcess = 'd';
    else if (command == "both")
      drawingsViaProcess = 'b';
    else
      return false;
  }

  return true;
}
