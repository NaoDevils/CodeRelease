/**
 * @file CognitionLogDataProvider.h
 * This file declares a module that provides data replayed from a log file.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "LogDataProvider.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h"
#include "Representations/BehaviorControl/VisualRefereeBehaviorSymbols.h"
#include "Representations/BehaviorControl/JoystickControl/JoystickState.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SequenceImage.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/RobotPoseHypotheses.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Perception/CLIPGoalPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/CenterCirclePercept.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/PenaltyCrossPercept.h"
#include "Representations/Perception/RefereeKeypoints.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Module/Module.h"
#include "Tools/ProcessFramework/CycleLocal.h"

#include <memory>

MODULE(CognitionLogDataProvider,
  USES(CameraInfo),
  USES(CameraInfoUpper),
  PROVIDES_CONCURRENT(ActivationGraph),
  PROVIDES_CONCURRENT(AudioData),
  PROVIDES_CONCURRENT(BallModel),
  PROVIDES_CONCURRENT(BallPercept),
  PROVIDES_CONCURRENT(MultipleBallPercept),
  PROVIDES_CONCURRENT(Ballchaser),
  PROVIDES_CONCURRENT(BallSymbols),
  PROVIDES_CONCURRENT(BehaviorData),
  PROVIDES_CONCURRENT(BodyContour),
  PROVIDES_CONCURRENT(CameraCalibration),
  PROVIDES_CONCURRENT(CameraInfo),
  PROVIDES_CONCURRENT(CameraInfoUpper),
  PROVIDES_CONCURRENT(CameraMatrix),
  PROVIDES_CONCURRENT(CameraMatrixUpper),
  PROVIDES_CONCURRENT(CLIPCenterCirclePercept),
  PROVIDES_CONCURRENT(CLIPFieldLinesPercept),
  PROVIDES_CONCURRENT(CLIPGoalPercept),
  PROVIDES_CONCURRENT(FrameInfo),
  PROVIDES_CONCURRENT(GameInfo),
  PROVIDES_CONCURRENT(GroundContactState),
  PROVIDES_CONCURRENT(GroundTruthWorldState),
  PROVIDES_CONCURRENT(HeadAngleRequest),
  PROVIDES_CONCURRENT(HeadControlRequest),
  PROVIDES_CONCURRENT_WITHOUT_MODIFY(Image),
  PROVIDES_CONCURRENT_WITHOUT_MODIFY(ImageUpper),
  PROVIDES_CONCURRENT_WITHOUT_MODIFY(SequenceImage),
  PROVIDES_CONCURRENT_WITHOUT_MODIFY(SequenceImageUpper),
  PROVIDES_CONCURRENT(ImageCoordinateSystem),
  PROVIDES_CONCURRENT(ImageCoordinateSystemUpper),
  PROVIDES_CONCURRENT(JointSensorData),
  PROVIDES_CONCURRENT(JoystickState),
  PROVIDES_CONCURRENT(MotionInfo),
  PROVIDES_CONCURRENT(MotionRequest),
  PROVIDES_CONCURRENT(OdometryData),
  PROVIDES_CONCURRENT(OpponentTeamInfo),
  PROVIDES_CONCURRENT(OwnTeamInfo),
  PROVIDES_CONCURRENT(PenaltyCrossPercept),
  PROVIDES_CONCURRENT(ProcessedBallPatches),
  PROVIDES_CONCURRENT(ProcessedRobotsHypotheses),
  PROVIDES_CONCURRENT(PositioningSymbols),
  PROVIDES_CONCURRENT(RawGameInfo),
  PROVIDES_CONCURRENT(RefereeKeypoints),
  PROVIDES_CONCURRENT(RemoteBallModel),
  PROVIDES_CONCURRENT(RobotHealth),
  PROVIDES_CONCURRENT(RobotInfo),
  PROVIDES_CONCURRENT(RobotMap),
  PROVIDES_CONCURRENT(RobotsPercept),
  PROVIDES_CONCURRENT(RobotPose),
  PROVIDES_CONCURRENT_WITHOUT_MODIFY(RobotPoseHypotheses),
  PROVIDES_CONCURRENT(SideConfidence),
  PROVIDES_CONCURRENT(TeamBallModel),
  PROVIDES_CONCURRENT(TeammateData),
  PROVIDES_CONCURRENT(VisualRefereeBehaviorSymbols)
);


struct LowFrameRateImage;
struct LowFrameRateImageUpper;
class CognitionLogDataProvider : public CognitionLogDataProviderBase, public LogDataProvider
{
private:
  static CycleLocal<CognitionLogDataProvider*> theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
  bool frameDataComplete; /**< Were all messages of the current frame received? */
  LowFrameRateImage* lowFrameRateImage; /**< This will be allocated when a low frame rate image was received. */
  LowFrameRateImageUpper* lowFrameRateImageUpper; /**< This will be allocated when a low frame rate image was received. */
  SequenceImage* sequenceImage; /**< This will be allocated when a low frame rate image was received. */
  SequenceImageUpper* sequenceImageUpper; /**< This will be allocated when a low frame rate image was received. */
  Image lastImages; /**< Stores images per camera received as low frame rate images. */
  ImageUpper lastImagesUpper; /**< Stores images per camera received as low frame rate images. */
  std::unique_ptr<RobotPoseHypothesesCompressed> robotPoseHypothesesCompressed; /**< Stores robot pose hypotheses .*/

  DECLARE_DEBUG_IMAGE(corrected);
  DECLARE_DEBUG_IMAGE(correctedUpper);

  // No-op update stubs
  void update(ActivationGraph&) override {}
  void update(AudioData&) override {}
  void update(Ballchaser&) override {}
  void update(BallModel&) override {}
  void update(BallPercept&) override {}
  void update(MultipleBallPercept&) override {}
  void update(BallSymbols&) override {}
  void update(BehaviorData&) override {}
  void update(BodyContour&) override {}
  void update(CameraCalibration&) override {}
  void update(CameraInfo& cameraInfo) override {}
  void update(CameraInfoUpper& cameraInfoUpper) override {}
  void update(CameraMatrix&) override {}
  void update(CameraMatrixUpper&) override {}
  void update(CLIPCenterCirclePercept&) override {}
  void update(CLIPFieldLinesPercept&) override {}
  void update(CLIPGoalPercept&) override {}
  void update(FrameInfo&) override {}
  void update(GameInfo&) override {}
  void update(GroundContactState&) override {}
  void update(GroundTruthWorldState&) override {}
  void update(HeadAngleRequest&) override {}
  void update(HeadControlRequest&) override {}
  void update(JointSensorData&) override {}
  void update(JoystickState&) override {}
  void update(MotionInfo&) override {}
  void update(MotionRequest&) override {}
  void update(OdometryData&) override {}
  void update(OpponentTeamInfo&) override {}
  void update(OwnTeamInfo&) override {}
  void update(PenaltyCrossPercept&) override {}
  void update(ProcessedBallPatches&) override {}
  void update(ProcessedRobotsHypotheses&) override {}
  void update(PositioningSymbols&) override {}
  void update(RawGameInfo&) override {}
  void update(RefereeKeypoints&) override {}
  void update(RemoteBallModel&) override {}
  void update(RobotHealth&) override {}
  void update(RobotInfo&) override {}
  void update(RobotMap&) override {}
  void update(RobotPose&) override {}
  void update(RobotsPercept&) override {}
  void update(SideConfidence&) override {}
  void update(TeamBallModel&) override {}
  void update(TeammateData&) override {}
  void update(VisualRefereeBehaviorSymbols&) override {}

  // Updates with data
  void update(Image& image) override;
  void update(ImageUpper& imageUpper) override;
  void update(SequenceImage& image) override;
  void update(SequenceImageUpper& imageUpper) override;
  void update(ImageCoordinateSystem& imageCoordinateSystem) override;
  void update(ImageCoordinateSystemUpper& imageCoordinateSystem) override;
  void update(RobotPoseHypotheses& robotPoseHypotheses) override;
  /**
  * The method is called for every incoming debug message by handleMessage.
  * @param message An interface to read the message from the queue.
  * @return Was the message handled?
  */
  bool handleMessage2(InMessage& message);

public:
  /**
  * Default constructor.
  */
  CognitionLogDataProvider();

  /**
  * Destructor.
  */
  ~CognitionLogDataProvider();

  /**
  * The method is called for every incoming debug message.
  * @param message An interface to read the message from the queue.
  * @return Was the message handled?
  */
  static bool handleMessage(InMessage& message);

  /**
  * The method returns whether idProcessFinished was received.
  * @return Were all messages of the current frame received?
  */
  static bool isFrameDataComplete();
};
