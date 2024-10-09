/**
 * @file MessageIDs.h
 *
 * Declaration of ids for debug messages.
 *
 * @author Martin Lötzsch
 */

#pragma once

#include "Tools/Enum.h"

/**
 * IDs for debug messages
 *
 * To distinguish debug messages, they all have an id.
 *
 * !!! WARNING: Only add new IDs at the end.
 *              If not all logs will become unreplayable !!!
 */
BIGENUM(MessageID,
  (
    undefined,
    idProcessBegin,
    idProcessFinished,

    idActivationGraph,
    idAnnotation,
    idAudioData,
    idBallModel,
    idBallPercept,
    idMultipleBallPercept, // added by NDevils
    idBehaviorData, // added by NDevils
    idBodyContour,
    idCameraInfo,
    idCameraInfoUpper, // added by NDevils
    idCameraMatrix,
    idCameraMatrixUpper, // added by NDevils
    idCLIPCenterCirclePercept, // added by NDevils
    idCLIPFieldLinesPercept, // added by NDevils
    idCLIPGoalPercept, // added by NDevils
    idCLIPPointsPercept, // added by NDevils
    idColorCalibration,
    idFilteredJointData,
    idFsrSensorData,
    idFrameInfo,
    idGameInfo,
    idGroundContactState,
    idGroundTruthOdometryData,
    idGroundTruthWorldState,
    idImage,
    idImageUpper, // added by NDevils
    idImageCoordinateSystem,
    idInertialData,
    idInertialSensorData,
    idJointAngles,
    idJointDataPrediction,
    idJointSensorData,
    idJointVelocities,
    idJPEGImage,
    idJPEGImageUpper,
    idKeyStates,
    idRemoteRobotMap, // added by NDevils
    idLowFrameRateImage,
    idMotionInfo,
    idMotionRequest,
    //idOdometer, Replaced by LocalRobotMap
    idLocalRobotMap, // added by NDevils
    idOdometryData,
    idOpponentTeamInfo,
    idOwnTeamInfo,
    idPenaltyCrossPercept, // added by NDevils
    idRobotHealth,
    idRobotInfo,
    idRobotMap, // added by NDevils
    idRobotPose,
    idRobotsPercept,
    idSideConfidence,
    idKickSymbols, // added by NDevils
    idStopwatch,
    idSystemSensorData,
    idTeamBallModel,
    idTeammateData,
    idRoleSymbols, // added by NDevils
    idThumbnail,
    idSonarSensorData,
    // added by NDevils
    idSpeedRequest,
    idBallModelAfterPreview,
    idBallSymbols,
    idGameSymbols,
    idPositioningSymbols,
    idFallDownState,
    idLowFrameRateImageUpper,
    idThumbnailUpper,
    idRemoteBallModel,
    idBodyContourUpper,
    idRobotPoseHypothesesCompressed,
    idYoloInput,
    idYoloInputUpper,
    idSequenceImage,
    idSequenceImageUpper,
    idProcessedRobotsHypotheses,
    idJointRequest,
    idFootpositions,
    idMotionSelection,
    idMotionState,
    idProcessedBallPatches,
    idKinematicRequest,
    idHeadControlRequest,
    idRawGameInfo,
    idWalkCalibration,
    idCustomStepSelection,
    idTeamCommEvents,
    idBallchaser,
    idHeadAngleRequest,
    idCameraCalibration,
    idVisualRefereeBehaviorSymbols,
    idRefereeKeypoints,
    idJoystickState,

    numOfDataMessageIDs /**< everything below this does not belong into log files */
  ),(
    // ids used in team communication
    idTimeSynchronization = numOfDataMessageIDs,
    idPlaceholder1,
    idPlaceholder2,
    idPlaceholder3,
    idRobot,
    idSimpleRobotsDistributed,
    idTeam,
    idTeammateHasGroundContact,
    idPlaceholder4,
    idTeammateIsUpright,
    idTeammateTimeOfLastGroundContact,
    //idWhistle, // Added to whistleDortmund; Replaced by SpeedInfo
    idWhistleDortmund, // added by NDevils
    idSpeedInfo,

    // infrastructure
    idConsole,
    idDebugDataChangeRequest,
    idDebugDataResponse,
    idDebugDrawing,
    idDebugDrawing3D,
    idDebugImage,
    idDebugJPEGImage,
    idDebugRequest,
    idDebugResponse,
    idDrawingManager,
    idDrawingManager3D,
    idJointCalibration,
    idLogResponse,
    idModuleRequest,
    idModuleTable,
    idMotionNet,
    idPlot,
    idQueueFillRequest,
    idRobotDimensions,
    idRobotname,
    idStreamSpecification,
    idText,
    idKeyFrameMotions,
    idWalkingEngineKick,
    idPathDebugMessage,
    idExecutorObservings,
    idPingpong,
    idTeamCommSenderOutput,
    idRemoteControlRequest
  )
);
