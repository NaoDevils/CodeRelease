/**
 * @file CameraMatrixProvider.h
 * This file declares a class to calculate the position of the camera for the Nao.
 * @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

MODULE(CameraMatrixProvider,
  REQUIRES(CameraCalibration),
  REQUIRES(CameraInfo), // for debug drawing
  REQUIRES(CameraInfoUpper), // for debug drawing
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(JointSensorData),
  REQUIRES(MotionInfo),
  REQUIRES(RobotCameraMatrix),
  REQUIRES(RobotCameraMatrixUpper),
  REQUIRES(RobotModel),
  REQUIRES(RobotInfo),
  REQUIRES(TorsoMatrix),
  PROVIDES(CameraMatrix),
  PROVIDES(CameraMatrixUpper)
);

class CameraMatrixProvider : public CameraMatrixProviderBase
{
private:
  void update(CameraMatrix& cameraMatrix);
  void update(CameraMatrixUpper& cameraMatrix);

  void camera2image(const Vector3f& camera, Vector2f& image, bool upper) const;

  STREAMABLE(ModelPoints,,
    (std::vector<float>) thighPoints,
    (std::vector<int>) thighIndex,
    (std::vector<float>) shinePoints,
    (std::vector<int>) shineIndex,
    (std::vector<float>) footPoints,
    (std::vector<int>) footIndex
  );

  ModelPoints p;

  void drawRobotParts();
};
