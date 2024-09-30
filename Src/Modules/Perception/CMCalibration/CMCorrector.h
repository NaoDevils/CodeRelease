#pragma once

#include "Tools/Module/Module.h"

#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/CMCorrectorStatus.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"

#include "Tools/Module/ModuleManager.h"

#include <optional>
#include <tuple>

MODULE(CMCorrector,
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(FrameInfo),
  REQUIRES(RobotDimensions),
  REQUIRES(KeyStates),
  REQUIRES(FieldDimensions),
  REQUIRES(FallDownState),
  REQUIRES(CameraCalibration),
  REQUIRES(RobotInfo),
  USES(JointSensorData), // consistency with USES(CLIPFieldLinesPercept)
  USES(TorsoMatrix), // consistency with USES(CLIPFieldLinesPercept)
  USES(CLIPFieldLinesPercept),
  USES(RobotPose),
  USES(BehaviorData),
  PROVIDES(CameraCalibration),
  PROVIDES(HeadAngleRequest),
  PROVIDES(MotionRequest),
  PROVIDES(CMCorrectorStatus),
  HAS_PREEXECUTION,
  LOADS_PARAMETERS(,
    (std::array<Angle,2>)({2_deg}) firstStageErrorLimits,
    (std::array<Angle,2>)({2_deg}) secondStageErrorLimits,
    (std::array<Vector2i,2>)({Vector2i{10,5},Vector2i{10,5}}) minLineLength,
    (std::array<size_t,2>)({400,400}) minLines,
    (std::vector<HeadAngleRequest>) upperHeadAngleRequests,
    (std::vector<HeadAngleRequest>) lowerHeadAngleRequests,
    (std::vector<HeadAngleRequest>) localizeHeadAngleRequests,
    (Pose2f)(180_deg, {-1000.f, 0.f}) calibrationPose,
    (unsigned)(3000) waitTimeAfterWalking,
    (unsigned)(270000) calibrationTimeout
  )
);

class CMCorrector : public CMCorrectorBase
{
public:
  CMCorrector();

private:
  struct Sample
  {
    const std::vector<Geometry::Line> lines;
    const Vector2a headPosition;
    const TorsoMatrix torsoMatrix;
    const bool upper;
  };

  void execute(tf::Subflow&);
  void update(CameraCalibration& cameraCalibration);
  void update(HeadAngleRequest& headControlRequest);
  void update(MotionRequest& motionRequest);
  void update(CMCorrectorStatus& cmCorrectorStatus);

  bool optimizeUpper();
  bool optimizeLower();

  void captureSamples();
  std::optional<Sample> captureSample(bool upper) const;
  static size_t countLines(const std::vector<Sample>& samples);
  size_t getNumberOfSamples() const;

  CameraMatrix getCameraMatrix(const CameraCalibration& cc, const TorsoMatrix& tm, const Vector2a& hp, bool upper) const;
  static std::optional<Geometry::Line> transformImageToRobot(const Geometry::Line& line, const CameraMatrix& cm, const CameraInfo& ci);
  std::vector<std::optional<Geometry::Line>> transformSampleToRobot(const Sample& sample, const CameraCalibration& cc) const;
  std::vector<std::optional<Geometry::Line>> transformSampleToRobot(const Sample& sample, const Pose3f& torsoNeckMatrix, const Pose3f& neckCameraMatrix) const;
  std::vector<std::optional<Geometry::Line>> transformSamplesToRobot(const std::vector<Sample>& samples, const CameraCalibration& cc) const;

  static Angle getTotalError(const std::vector<std::optional<Geometry::Line>>& robotLines);
  static Angle getTotalErrorApprox(const std::vector<std::optional<Geometry::Line>>& robotLines);
  static std::tuple<std::array<unsigned int, 180>, unsigned int> getAngleDistribution(const std::vector<std::optional<Geometry::Line>>& robotLines);
  Angle calcError(const std::vector<Sample>& samples, const CameraCalibration& cc, bool approx = false) const;

  static std::tuple<std::vector<Sample>, std::vector<Sample>> separateSamples(const std::vector<Sample>& samples);

  template <typename T> static std::vector<T> join(std::vector<std::vector<T>>& vec);

  template <typename T, int dim>
  std::pair<float, Eigen::Matrix<T, dim, 1>> optimizeFunction(
      const Eigen::Matrix<T, dim, 1>& min, const Eigen::Matrix<T, dim, 1>& max, const std::vector<Eigen::Matrix<T, dim, 1>>& stepSizes, std::function<float(const Eigen::Matrix<T, dim, 1>&)> func);
  template <typename T, int dim>
  std::pair<float, Eigen::Matrix<T, dim, 1>> optimizeFunction(
      const Eigen::Matrix<T, dim, 1>& min, const Eigen::Matrix<T, dim, 1>& max, const Eigen::Matrix<T, dim, 1>& stepSize, std::function<float(const Eigen::Matrix<T, dim, 1>&)> func);

  void start();
  void gotoState(CMCorrectorStatus::CalibrationState state, unsigned char stage = 0);
  void stop();
  void load();
  void save() const;
  void reset();
  void printStatistics(const std::vector<Sample>& samples, const CameraCalibration& cc) const;
  void annotateCalibration() const;

  void debug();

  CMCorrectorStatus cmCorrectorStatus;

  std::vector<Sample> samples;

  unsigned startTimestamp = 0;
  unsigned stateBeginTimestamp = 0;
  unsigned char nearestCalibrationPose = 0;

  Vector2a lastHeadSensor = Vector2a::Zero();
  unsigned char captureHeadPosition = 0;

  bool saidPleaseCalibrateMe = false;
  std::optional<ModuleManager::Configuration> lastModuleConfig;

  CameraCalibration localCalibration;
};
