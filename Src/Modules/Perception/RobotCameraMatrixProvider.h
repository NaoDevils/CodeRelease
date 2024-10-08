/**
 * @file RobotCameraMatrixProvider.h
 * This file declares a class to calculate the position of the camera relative to the body for the Nao.
 * @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Perception/CameraMatrix.h"

MODULE(RobotCameraMatrixProvider,
  REQUIRES(CameraCalibration),
  REQUIRES(JointSensorData),
  REQUIRES(RobotDimensions),
  PROVIDES(RobotCameraMatrix),
  PROVIDES(RobotCameraMatrixUpper)
);

class RobotCameraMatrixProvider : public RobotCameraMatrixProviderBase
{
private:
  void update(RobotCameraMatrix& robotCameraMatrix);
  void update(RobotCameraMatrixUpper& robotCameraMatrixUpper);
};
