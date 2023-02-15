/**
 * @file JoinedIMUDataProvider.cpp
 *
 *
 *
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 */

#include "JoinedIMUDataProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"

MAKE_MODULE(JoinedIMUDataProvider, sensing)

void JoinedIMUDataProvider::update(JoinedIMUData& joinedIMUData)
{
  joinedIMUData.imuData[JoinedIMUData::inertialSensorData] = theInertialSensorData;

  joinedIMUData.imuData[JoinedIMUData::inertialData] = theInertialData;

  joinedIMUData.imuData[JoinedIMUData::imuModel].acc = theIMUModel.acceleration;
  joinedIMUData.imuData[JoinedIMUData::imuModel].gyro = theIMUModel.rotational_velocity.cast<Angle>();
  //joinedIMUData.imuData[JoinedIMUData::imuModel].gyro.z() *= -1.f;
  joinedIMUData.imuData[JoinedIMUData::imuModel].angle = theIMUModel.orientation;

  if (theWalkCalibration.bodyAngleCalibrated)
  {
    joinedIMUData.imuData[JoinedIMUData::inertialSensorData].angle -= theWalkCalibration.imuAngleOffsets[JoinedIMUData::inertialSensorData];
    joinedIMUData.imuData[JoinedIMUData::inertialData].angle -= theWalkCalibration.imuAngleOffsets[JoinedIMUData::inertialData];
    joinedIMUData.imuData[JoinedIMUData::imuModel].angle -= theWalkCalibration.imuAngleOffsets[JoinedIMUData::imuModel];
  }

  imuAngleBuffer.push_front(joinedIMUData.imuData[JoinedIMUData::imuModel].angle);
  drawPlots(joinedIMUData);
}

void JoinedIMUDataProvider::drawPlots(JoinedIMUData& joinedIMUData)
{
  PLOT("module:JoinedIMUDataProvider:gyro:x:InertialSensorData", joinedIMUData.imuData[JoinedIMUData::inertialSensorData].gyro.x().toDegrees());
  PLOT("module:JoinedIMUDataProvider:gyro:y:InertialSensorData", joinedIMUData.imuData[JoinedIMUData::inertialSensorData].gyro.y().toDegrees());
  PLOT("module:JoinedIMUDataProvider:gyro:z:InertialSensorData", joinedIMUData.imuData[JoinedIMUData::inertialSensorData].gyro.z().toDegrees());
  PLOT("module:JoinedIMUDataProvider:acc:x:InertialSensorData", joinedIMUData.imuData[JoinedIMUData::inertialSensorData].acc.x());
  PLOT("module:JoinedIMUDataProvider:acc:y:InertialSensorData", joinedIMUData.imuData[JoinedIMUData::inertialSensorData].acc.y());
  PLOT("module:JoinedIMUDataProvider:acc:z:InertialSensorData", joinedIMUData.imuData[JoinedIMUData::inertialSensorData].acc.z() - theWalkCalibration.gravity);
  PLOT("module:JoinedIMUDataProvider:angle:x:InertialSensorData", joinedIMUData.imuData[JoinedIMUData::inertialSensorData].angle.x().toDegrees());
  PLOT("module:JoinedIMUDataProvider:angle:y:InertialSensorData", joinedIMUData.imuData[JoinedIMUData::inertialSensorData].angle.y().toDegrees());

  PLOT("module:JoinedIMUDataProvider:gyro:x:InertialData", joinedIMUData.imuData[JoinedIMUData::inertialData].gyro.x().toDegrees());
  PLOT("module:JoinedIMUDataProvider:gyro:y:InertialData", joinedIMUData.imuData[JoinedIMUData::inertialData].gyro.y().toDegrees());
  PLOT("module:JoinedIMUDataProvider:gyro:z:InertialData", joinedIMUData.imuData[JoinedIMUData::inertialData].gyro.z().toDegrees());
  PLOT("module:JoinedIMUDataProvider:acc:x:InertialData", joinedIMUData.imuData[JoinedIMUData::inertialData].acc.x());
  PLOT("module:JoinedIMUDataProvider:acc:y:InertialData", joinedIMUData.imuData[JoinedIMUData::inertialData].acc.y());
  PLOT("module:JoinedIMUDataProvider:acc:z:InertialData", joinedIMUData.imuData[JoinedIMUData::inertialData].acc.z() - theWalkCalibration.gravity);
  PLOT("module:JoinedIMUDataProvider:angle:x:InertialData", joinedIMUData.imuData[JoinedIMUData::inertialData].angle.x().toDegrees());
  PLOT("module:JoinedIMUDataProvider:angle:y:InertialData", joinedIMUData.imuData[JoinedIMUData::inertialData].angle.y().toDegrees());

  PLOT("module:JoinedIMUDataProvider:gyro:x:IMUModel", joinedIMUData.imuData[JoinedIMUData::imuModel].gyro.x().toDegrees());
  PLOT("module:JoinedIMUDataProvider:gyro:y:IMUModel", joinedIMUData.imuData[JoinedIMUData::imuModel].gyro.y().toDegrees());
  PLOT("module:JoinedIMUDataProvider:gyro:z:IMUModel", joinedIMUData.imuData[JoinedIMUData::imuModel].gyro.z().toDegrees());
  PLOT("module:JoinedIMUDataProvider:acc:x:IMUModel", joinedIMUData.imuData[JoinedIMUData::imuModel].acc.x());
  PLOT("module:JoinedIMUDataProvider:acc:y:IMUModel", joinedIMUData.imuData[JoinedIMUData::imuModel].acc.y());
  PLOT("module:JoinedIMUDataProvider:acc:z:IMUModel", joinedIMUData.imuData[JoinedIMUData::imuModel].acc.z());
  PLOT("module:JoinedIMUDataProvider:angle:x:IMUModel", joinedIMUData.imuData[JoinedIMUData::imuModel].angle.x().toDegrees());
  PLOT("module:JoinedIMUDataProvider:angle:y:IMUModel", joinedIMUData.imuData[JoinedIMUData::imuModel].angle.y().toDegrees());
  PLOT("module:JoinedIMUDataProvider:angle:x:IMUModelDelayed", imuAngleBuffer[theWalkingEngineParams.imuSensorDelayFrames - 1].x().toDegrees());
  PLOT("module:JoinedIMUDataProvider:angle:y:IMUModelDelayed", imuAngleBuffer[theWalkingEngineParams.imuSensorDelayFrames - 1].y().toDegrees());

  Vector3a gyroDiffIMUModel = joinedIMUData.imuData[JoinedIMUData::inertialSensorData].gyro - joinedIMUData.imuData[JoinedIMUData::imuModel].gyro;
  Vector3f accDiffIMUModel = joinedIMUData.imuData[JoinedIMUData::inertialSensorData].acc - joinedIMUData.imuData[JoinedIMUData::imuModel].acc;
  Vector2a angleDiffIMUModel = joinedIMUData.imuData[JoinedIMUData::inertialSensorData].angle - joinedIMUData.imuData[JoinedIMUData::imuModel].angle;
  Vector2a angleDiffIMUModelDelayed = joinedIMUData.imuData[JoinedIMUData::inertialSensorData].angle - imuAngleBuffer[theWalkingEngineParams.imuSensorDelayFrames - 1];

  PLOT("module:JoinedIMUDataProvider:gyroDiffIMUModel:x", gyroDiffIMUModel.x().toDegrees());
  PLOT("module:JoinedIMUDataProvider:gyroDiffIMUModel:y", gyroDiffIMUModel.y().toDegrees());
  PLOT("module:JoinedIMUDataProvider:gyroDiffIMUModel:z", gyroDiffIMUModel.z().toDegrees());
  PLOT("module:JoinedIMUDataProvider:accDiffIMUModel:x", accDiffIMUModel.x());
  PLOT("module:JoinedIMUDataProvider:accDiffIMUModel:y", accDiffIMUModel.y());
  PLOT("module:JoinedIMUDataProvider:accDiffIMUModel:z", accDiffIMUModel.z() - theWalkCalibration.gravity);
  PLOT("module:JoinedIMUDataProvider:angleDiffIMUModel:x", angleDiffIMUModel.x().toDegrees());
  PLOT("module:JoinedIMUDataProvider:angleDiffIMUModel:y", angleDiffIMUModel.y().toDegrees());
  PLOT("module:JoinedIMUDataProvider:angleDiffIMUModelDelayed:x", angleDiffIMUModelDelayed.x().toDegrees());
  PLOT("module:JoinedIMUDataProvider:angleDiffIMUModelDelayed:y", angleDiffIMUModelDelayed.y().toDegrees());

  Vector3a gyroDiffInertialData = joinedIMUData.imuData[JoinedIMUData::inertialSensorData].gyro - joinedIMUData.imuData[JoinedIMUData::inertialData].gyro;
  Vector3f accDiffInertialData = joinedIMUData.imuData[JoinedIMUData::inertialSensorData].acc - joinedIMUData.imuData[JoinedIMUData::inertialData].acc;
  Vector2a angleDiffInertialData = joinedIMUData.imuData[JoinedIMUData::inertialSensorData].angle - joinedIMUData.imuData[JoinedIMUData::inertialData].angle;

  PLOT("module:JoinedIMUDataProvider:gyroDiffInertialData:x", gyroDiffInertialData.x().toDegrees());
  PLOT("module:JoinedIMUDataProvider:gyroDiffInertialData:y", gyroDiffInertialData.y().toDegrees());
  PLOT("module:JoinedIMUDataProvider:gyroDiffInertialData:z", gyroDiffInertialData.z().toDegrees());
  PLOT("module:JoinedIMUDataProvider:accDiffInertialData:x", accDiffInertialData.x());
  PLOT("module:JoinedIMUDataProvider:accDiffInertialData:y", accDiffInertialData.y());
  PLOT("module:JoinedIMUDataProvider:accDiffInertialData:z", accDiffInertialData.z() - theWalkCalibration.gravity);
  PLOT("module:JoinedIMUDataProvider:angleDiffInertialData:x", angleDiffInertialData.x().toDegrees());
  PLOT("module:JoinedIMUDataProvider:angleDiffInertialData:y", angleDiffInertialData.y().toDegrees());
}
