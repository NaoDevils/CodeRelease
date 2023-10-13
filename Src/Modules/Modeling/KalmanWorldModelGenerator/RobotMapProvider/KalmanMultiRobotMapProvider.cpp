/**
 * \file KalmanMultiRobotMapProvider.cpp
 * \author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
 */

#include "KalmanMultiRobotMapProvider.h"

#include <vector>

#define DRAW_MAP(id, map)                                                                                                   \
  DEBUG_DRAWING(id, "drawingOnField")                                                                                       \
  {                                                                                                                         \
    for (unsigned i = 0; i < map.size(); i++)                                                                               \
    {                                                                                                                       \
      const auto& hypothesis = map[i];                                                                                      \
      ColorRGBA color = (hypothesis.lastValidRobotType() == RobotEstimate::teammateRobot) ? teammateColor                   \
          : (hypothesis.lastValidRobotType() == RobotEstimate::opponentRobot)                                               \
          ? opponentColor                                                                                                   \
          : unknownColor;                                                                                                   \
      float x = hypothesis.kalman.positionX();                                                                              \
      float y = hypothesis.kalman.positionY();                                                                              \
      float rot = hypothesis.kalman.velocity().angle();                                                                     \
      FILLED_TRIANGLE(id, x, y, rot, 10, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, color);                \
      ARROW(id, x, y, x + hypothesis.kalman.velocityX(), y + hypothesis.kalman.velocityY(), 10, Drawings::solidPen, color); \
      std::stringstream ss;                                                                                                 \
      ss << roundf(hypothesis.validity * 100.f) / 100.f;                                                                    \
      DRAWTEXT(id, x + 70, y, 75, color, ss.str().c_str());                                                                 \
      COVARIANCE2D(id, hypothesis.kalman.covarianceMatrix.cast<float>(), hypothesis.kalman.position());                     \
    }                                                                                                                       \
  }

// This anonymous namespace contains a named-parameter object for the performSensorUpdate function
namespace
{
  static const Vector2f defaultPosition;
  static const Vector2a defaultAngle;

  template <typename RobotMap> struct SensorUpdateType
  {
    RobotMap& robotMap_;
    const Vector2f* measuredPositionRelative_ = &defaultPosition;
    float measuredDistance_ = 0;
    const Vector2f* measuredVelocity_ = nullptr;
    RobotEstimate::RobotType desiredRobotType_ = RobotEstimate::RobotType::invalid;
    unsigned timestamp_ = 0;
    float perceptValidity_ = 0;
    const Vector2a* maxAngleDiffToMerge_ = &defaultAngle;
    float maxDistanceForDistanceBasedMerging_ = 0;
    float maxDistanceToMerge_ = 0;
    float initialValidityForNewHypothesis_ = 0;
    const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices_;

    SensorUpdateType(RobotMap& rm, const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& noise) : robotMap_(rm), kalmanNoiseMatrices_(noise) {}
    RobotMap& robotMap() const { return robotMap_; }

    const Vector2f& measuredPositionRelative() const { return *measuredPositionRelative_; }
    SensorUpdateType& measuredPositionRelative(const Vector2f& mp)
    {
      measuredPositionRelative_ = &mp;
      return *this;
    }

    float measuredDistance() const { return measuredDistance_; }
    SensorUpdateType& measuredDistance(float md)
    {
      measuredDistance_ = md;
      return *this;
    }

    const Vector2f* measuredVelocity() const { return measuredVelocity_; }
    SensorUpdateType& measuredVelocity(const Vector2f* velocity)
    {
      measuredVelocity_ = velocity;
      return *this;
    }

    RobotEstimate::RobotType desiredRobotType() const { return desiredRobotType_; }
    SensorUpdateType& desiredRobotType(RobotEstimate::RobotType type)
    {
      desiredRobotType_ = type;
      return *this;
    }

    unsigned timestamp() const { return timestamp_; }
    SensorUpdateType& timestamp(unsigned time)
    {
      timestamp_ = time;
      return *this;
    }

    float perceptValidity() const { return perceptValidity_; }
    SensorUpdateType& perceptValidity(float validity)
    {
      perceptValidity_ = validity;
      return *this;
    }

    const Vector2a& maxAngleDiffToMerge() const { return *maxAngleDiffToMerge_; }
    SensorUpdateType& maxAngleDiffToMerge(const Vector2a& angleDiff)
    {
      maxAngleDiffToMerge_ = &angleDiff;
      return *this;
    }

    float maxDistanceForDistanceBasedMerging() const { return maxDistanceForDistanceBasedMerging_; }
    SensorUpdateType& maxDistanceForDistanceBasedMerging(float maxDistance)
    {
      maxDistanceForDistanceBasedMerging_ = maxDistance;
      return *this;
    }

    float maxDistanceToMerge() const { return maxDistanceToMerge_; }
    SensorUpdateType& maxDistanceToMerge(float maxDistance)
    {
      maxDistanceToMerge_ = maxDistance;
      return *this;
    }

    float initialValidityForNewHypothesis() const { return initialValidityForNewHypothesis_; }
    SensorUpdateType& initialValidityForNewHypothesis(float validity)
    {
      initialValidityForNewHypothesis_ = validity;
      return *this;
    }

    const auto& kalmanNoiseMatrices() const { return kalmanNoiseMatrices_; }
    template <typename Matrix> SensorUpdateType& kalmanNoiseMatrices(const Matrix& noise)
    {
      kalmanNoiseMatrices_ = noise;
      return *this;
    }
  };
  template <typename RobotMap, typename Matrix> SensorUpdateType<RobotMap> sensorUpdateArguments(RobotMap& map, const Matrix& noise)
  {
    return SensorUpdateType<RobotMap>(map, noise);
  }
} // namespace

// ==========================
// UPDATE METHODS
// ==========================

void KalmanMultiRobotMapProvider::update(RobotMap& robotMap)
{
  robotMap = m_robotMap;
}

void KalmanMultiRobotMapProvider::update(LocalRobotMap& localRobotMap)
{
  localRobotMap = m_localRobotMap;
}

void KalmanMultiRobotMapProvider::update(RemoteRobotMap& remoteRobotMap)
{
  remoteRobotMap = m_remoteRobotMap;
}


// ==========================
// INITIALIZE
// ==========================

void KalmanMultiRobotMapProvider::reset()
{
  // Reset robot maps
  m_robotMap.reset();
  m_localKalmanRobotMap.clear();
  m_localRobotMap.reset();
  m_remoteKalmanRobotMap.clear();
  m_remoteRobotMap.reset();
  m_mergedKalmanRobotMap.clear();
}


// ==========================
// EXCUTE CODE
// ==========================

void KalmanMultiRobotMapProvider::execute(tf::Subflow&)
{
  // Modify internal params
  modifyInternalParameters();

  // Init debugging
  initDebugging();

  // Motion update (kalman maps)
  motionUpdate();

  // Sensor update (kalman maps)
  sensorUpdate();

  // Prune hypotheses
  pruneHypotheses();

  // Update robot types
  updateRobotTypes();

  // Generate robot maps from kalman maps
  generateMaps();

  // Debugging
  doGlobalDebugging();

  // --- Save current state for next iteration ---
  m_lastOdometryData = theOdometryData;
}

void KalmanMultiRobotMapProvider::motionUpdate()
{
  // Calculate odometry change since last iteration.
  Pose2f odometryOffset = theOdometryData - m_lastOdometryData;

  // Apply odometry offset to all local robot hypotheses.
  m_localKalmanRobotMap.removeOdometry(odometryOffset);
  m_remoteKalmanRobotMap.removeOdometry(odometryOffset);
  m_mergedKalmanRobotMap.removeOdometry(odometryOffset);

  m_localKalmanRobotMap.setRobotPoseCameraMatrixAndFriction(theRobotPose, theCameraMatrixUpper, parameters.friction);
  m_remoteKalmanRobotMap.setRobotPoseCameraMatrixAndFriction(theRobotPose, theCameraMatrixUpper, parameters.friction);
  m_mergedKalmanRobotMap.setRobotPoseCameraMatrixAndFriction(theRobotPose, theCameraMatrixUpper, parameters.friction);


  m_localKalmanRobotMap.motionUpdate(theFrameInfo.time);
  m_remoteKalmanRobotMap.motionUpdate(theFrameInfo.time);
  m_mergedKalmanRobotMap.motionUpdate(theFrameInfo.time);
}

void KalmanMultiRobotMapProvider::sensorUpdate()
{
  std::vector<RobotEstimate> perceptsInThisFrame;

  // Update the perceptBuffer with the latest odometry
  Pose2f odometryOffset = theOdometryData - m_lastOdometryData;
  for (std::vector<RobotEstimate>& rev : perceptBuffer)
  {
    for (RobotEstimate& re : rev)
    {
      //Vector2f& reOnField = Transformation::fieldToRobot(theRobotPose, re.locationOnField.translation);
      Vector2f& reOnField = re.locationOnField.translation;
      reOnField -= odometryOffset.translation;
      reOnField.rotate(-odometryOffset.rotation);
    }
  }

  // ===============================
  // Update local percepts
  // ===============================
  for (const RobotEstimate& robot : theRobotsPercept.robots)
  {
    perceptsInThisFrame.push_back(robot);
    // Update sensor
    performSensorUpdate(
        sensorUpdateArguments(m_localKalmanRobotMap, kalmanNoiseMatrices)
            .measuredPositionRelative(robot.locationOnField.translation)
            .measuredDistance(robot.distance)
            .desiredRobotType(robot.robotType)
            .timestamp(theFrameInfo.time)
            .perceptValidity(robot.validity)
            .maxAngleDiffToMerge(parameters.localMapParameters.maxAngleDiffToMerge)
            .maxDistanceForDistanceBasedMerging(parameters.localMapParameters.maxDistanceForDistanceBasedMerging)
            .maxDistanceToMerge(parameters.localMapParameters.maxDistanceToMerge)
            .initialValidityForNewHypothesis(parameters.localPercept.hypotheses_initialValidityForNewHypotheses));
    // Update merged
    performSensorUpdate(
        sensorUpdateArguments(m_mergedKalmanRobotMap, kalmanNoiseMatrices)
            .measuredPositionRelative(robot.locationOnField.translation)
            .measuredDistance(robot.distance)
            .desiredRobotType(robot.robotType)
            .timestamp(theFrameInfo.time)
            .perceptValidity(robot.validity * parameters.mergedMapParameters.localPerceptInfluence)
            .maxAngleDiffToMerge(parameters.mergedMapParameters.maxAngleDiffToMerge)
            .maxDistanceForDistanceBasedMerging(parameters.mergedMapParameters.maxDistanceForDistanceBasedMerging)
            .maxDistanceToMerge(parameters.mergedMapParameters.maxDistanceToMerge)
            .initialValidityForNewHypothesis(parameters.localPercept.hypotheses_initialValidityForNewHypotheses),
        theRobotInfo.number,
        RemoteKalmanPositionHypothesis::TeammateInfo(robot.validity, theFrameInfo.time));
  }
  perceptBuffer.push_front(perceptsInThisFrame);

  DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:perceptBuffer", "drawingOnField")
  {
    if (perceptBuffer.size() > 0)
    {
      for (const std::vector<RobotEstimate>& rev : perceptBuffer)
      {
        for (const RobotEstimate& re : rev)
        {
          CIRCLE("module:KalmanMultiRobotMapProvider:perceptBuffer",
              re.locationOnField.translation.x(),
              re.locationOnField.translation.y(),
              25 /*distancePrecisionLineDetection / 2.f*/,
              2,
              Drawings::solidPen,
              ColorRGBA::white,
              Drawings::solidBrush,
              ColorRGBA(255, 255, 255, 100));
        }
      }
    }
  }

  // ===============================
  // Update from teammate positions
  // ===============================
  for (const TeammateReceived& teammate : theTeammateData.teammates)
  {
    // Loop over all players which have sent data and are active
    if (teammate.status == TeammateReceived::Status::FULLY_ACTIVE)
    {
      // Calculate distance
      auto relativePosition = Transformation::fieldToRobot(theRobotPose, teammate.robotPose.translation);
      float distance = relativePosition.norm();

      // Calculate speed
      const Pose2f& speed = teammate.speedInfo.speed;
      auto speed2d = Vector2f(speed.translation.x() * cos(speed.rotation), speed.translation.y() * sin(speed.rotation));
      auto relativeSpeed = Transformation::robotToFieldVelocity(teammate.robotPose, speed2d);

      // Update remote
      performSensorUpdate(
          sensorUpdateArguments(m_remoteKalmanRobotMap, kalmanNoiseMatrices)
              .measuredPositionRelative(relativePosition)
              .measuredDistance(distance)
              .measuredVelocity(&relativeSpeed)
              .desiredRobotType(RobotEstimate::RobotType::teammateRobot)
              .timestamp(teammate.sendTimestamp)
              .perceptValidity(teammate.robotPose.validity * parameters.remoteMapParameters.teammatePositionInfluence)
              .maxAngleDiffToMerge(parameters.remoteMapParameters.maxAngleDiffToMerge)
              .maxDistanceForDistanceBasedMerging(parameters.remoteMapParameters.maxDistanceForDistanceBasedMerging)
              .maxDistanceToMerge(parameters.remoteMapParameters.maxDistanceToMerge)
              .initialValidityForNewHypothesis(parameters.remoteModel.hypotheses_initialValidityForNewHypotheses),
          teammate.playerNumber,
          RemoteKalmanPositionHypothesis::TeammateInfo(teammate.robotPose.validity, teammate.sendTimestamp));
      // Update merged
      performSensorUpdate(
          sensorUpdateArguments(m_mergedKalmanRobotMap, kalmanNoiseMatrices)
              .measuredPositionRelative(relativePosition)
              .measuredDistance(distance)
              .measuredVelocity(&relativeSpeed)
              .desiredRobotType(RobotEstimate::RobotType::teammateRobot)
              .timestamp(teammate.sendTimestamp)
              .perceptValidity(teammate.robotPose.validity * parameters.mergedMapParameters.teammatePositionInfluence)
              .maxAngleDiffToMerge(parameters.mergedMapParameters.maxAngleDiffToMerge)
              .maxDistanceForDistanceBasedMerging(parameters.mergedMapParameters.maxDistanceForDistanceBasedMerging)
              .maxDistanceToMerge(parameters.mergedMapParameters.maxDistanceToMerge)
              .initialValidityForNewHypothesis(parameters.remoteModel.hypotheses_initialValidityForNewHypotheses),
          teammate.playerNumber,
          RemoteKalmanPositionHypothesis::TeammateInfo(teammate.robotPose.validity, teammate.sendTimestamp));
    }
  }

  // ===============================
  // Update sonar percepts
  // ===============================
  for (const SonarEstimate& sonarEstimate : theSonarPercept.sonarEstimates)
  {
    // Calculate speed
    Vector2f sonarEstimateRelativeSpeed = Vector2f(0, 0);

    performSensorUpdate(
        sensorUpdateArguments(m_localKalmanRobotMap, kalmanNoiseMatrices)
            .measuredPositionRelative(sonarEstimate.relativePosition.translation)
            .measuredDistance(sonarEstimate.distance)
            .measuredVelocity(&sonarEstimateRelativeSpeed)
            .desiredRobotType(RobotEstimate::RobotType::unknownRobot)
            .timestamp(sonarEstimate.timestamp)
            .perceptValidity(sonarEstimate.validity)
            .maxAngleDiffToMerge(parameters.localMapParameters.maxAngleDiffToMerge)
            .maxDistanceForDistanceBasedMerging(parameters.localMapParameters.maxDistanceForDistanceBasedMerging)
            .maxDistanceToMerge(parameters.localMapParameters.maxDistanceToMerge)
            .initialValidityForNewHypothesis(parameters.localPercept.hypotheses_initialValidityForNewHypotheses));

    DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:sonarPercept", "drawingOnField")
    {
      Vector2f sonarEstimatePositionOnField = Transformation::robotToField(theRobotPose, sonarEstimate.relativePosition.translation);
      CIRCLE("module:KalmanMultiRobotMapProvider:sonarPercept", sonarEstimatePositionOnField.x(), sonarEstimatePositionOnField.y(), 40, 2, Drawings::solidPen, ColorRGBA::yellow, Drawings::solidBrush, ColorRGBA::yellow);
    }
  }

  // ===============================
  // Update from teammate data
  // ===============================
  for (const TeammateReceived& teammate : theTeammateData.teammates)
  {
    // Loop over all players which have sent data and are active
    if (teammate.status == TeammateReceived::Status::FULLY_ACTIVE)
    {
      // ===== Update models =====
      for (const auto& robotModel : teammate.localRobotMap.robots)
      {
        // Check that is not me
        if ((robotModel.pose.translation - theRobotPose.translation).norm() > 200)
        {
          // Calculate distance
          // Received, locale map of the teammate are in posOnField coordians
          auto relativePosition = Transformation::fieldToRobot(theRobotPose, robotModel.pose.translation);
          float distance = relativePosition.norm();

          // Update remote
          Vector2f percept = teammate.robotPose.translation;
          performSensorUpdate(
              sensorUpdateArguments(m_remoteKalmanRobotMap, kalmanNoiseMatrices)
                  .measuredPositionRelative(relativePosition)
                  .measuredDistance(distance)
                  // TC: Removed, because not transmitted .measuredVelocity(&robotModel.velocity)
                  .desiredRobotType(robotModel.robotType)
                  .timestamp(teammate.sendTimestamp)
                  .perceptValidity(robotModel.validity * parameters.remoteMapParameters.teammateModelInfluence)
                  .maxAngleDiffToMerge(parameters.remoteMapParameters.maxAngleDiffToMerge)
                  .maxDistanceForDistanceBasedMerging(parameters.remoteMapParameters.maxDistanceForDistanceBasedMerging)
                  .maxDistanceToMerge(parameters.remoteMapParameters.maxDistanceToMerge)
                  .initialValidityForNewHypothesis(parameters.remoteModel.hypotheses_initialValidityForNewHypotheses),
              teammate.playerNumber,
              RemoteKalmanPositionHypothesis::TeammateInfo(robotModel.validity, teammate.sendTimestamp));
          // Update merged
          performSensorUpdate(
              sensorUpdateArguments(m_mergedKalmanRobotMap, kalmanNoiseMatrices)
                  .measuredPositionRelative(relativePosition)
                  .measuredDistance(distance)
                  // TC: Removed, because not transmitted .measuredVelocity(&robotModel.velocity)
                  .desiredRobotType(robotModel.robotType)
                  .timestamp(teammate.sendTimestamp)
                  .perceptValidity(robotModel.validity * parameters.mergedMapParameters.teammateModelInfluence)
                  .maxAngleDiffToMerge(parameters.mergedMapParameters.maxAngleDiffToMerge)
                  .maxDistanceForDistanceBasedMerging(parameters.mergedMapParameters.maxDistanceForDistanceBasedMerging)
                  .maxDistanceToMerge(parameters.mergedMapParameters.maxDistanceToMerge)
                  .initialValidityForNewHypothesis(parameters.remoteModel.hypotheses_initialValidityForNewHypotheses),
              teammate.playerNumber,
              RemoteKalmanPositionHypothesis::TeammateInfo(robotModel.validity, teammate.sendTimestamp));
        }
      }
      //Percepts are currently (as of November 2022) no longer sent around with
      /*
      // ===== Update percepts =====
      // Update from lower percepts
      for (const auto& percept : teammate.robotsPercept.robots)
      {
        // Check that is not me
        if ((percept.locationOnField.translation - theRobotPose.translation).norm() > 400)
        {
          // Received, locale map of the teammate are in posOnField coordians
          auto relativePosition = Transformation::fieldToRobot(theRobotPose, percept.locationOnField.translation);
          // Update remote
          performSensorUpdate(sensorUpdateArguments(m_remoteKalmanRobotMap, kalmanNoiseMatrices)
            .measuredPositionRelative(relativePosition)
            .measuredDistance(percept.distance)
            .desiredRobotType(percept.robotType)
            .timestamp(percept.timestampFromImage)
            .perceptValidity(percept.validity* parameters.remoteMapParameters.teammatePerceptInfluence)
            .maxAngleDiffToMerge(parameters.remoteMapParameters.maxAngleDiffToMerge)
            .maxDistanceForDistanceBasedMerging(parameters.remoteMapParameters.maxDistanceForDistanceBasedMerging)
            .maxDistanceToMerge(parameters.remoteMapParameters.maxDistanceToMerge)
            .initialValidityForNewHypothesis(parameters.remoteModel.hypotheses_initialValidityForNewHypotheses),
            teammate.number,
            RemoteKalmanPositionHypothesis::TeammateInfo(percept.validity, percept.timestampFromImage));
          // Update merged
          performSensorUpdate(sensorUpdateArguments(m_mergedKalmanRobotMap, kalmanNoiseMatrices)
            .measuredPositionRelative(relativePosition)
            .measuredDistance(percept.distance)
            .desiredRobotType(percept.robotType)
            .timestamp(percept.timestampFromImage)
            .perceptValidity(percept.validity* parameters.mergedMapParameters.teammatePerceptInfluence)
            .maxAngleDiffToMerge(parameters.mergedMapParameters.maxAngleDiffToMerge)
            .maxDistanceForDistanceBasedMerging(parameters.mergedMapParameters.maxDistanceForDistanceBasedMerging)
            .maxDistanceToMerge(parameters.mergedMapParameters.maxDistanceToMerge)
            .initialValidityForNewHypothesis(parameters.remoteModel.hypotheses_initialValidityForNewHypotheses),
            teammate.number,
            RemoteKalmanPositionHypothesis::TeammateInfo(percept.validity, percept.timestampFromImage));
        }
      }
      // Update from upper percepts
      for (const auto& percept : teammate.robotsPerceptUpper.robots)
      {
        auto relativePosition = Transformation::fieldToRobot(theRobotPose, percept.locationOnField.translation);

        // Check that is not me
        if ((percept.locationOnField.translation - theRobotPose.translation).norm() > 400)
        {
          // Update remote
          performSensorUpdate(sensorUpdateArguments(m_remoteKalmanRobotMap, kalmanNoiseMatrices)
            .measuredPositionRelative(relativePosition)
            .measuredDistance(percept.distance)
            .desiredRobotType(percept.robotType)
            .timestamp(percept.timestampFromImage)
            .perceptValidity(percept.validity * parameters.remoteMapParameters.teammatePerceptInfluence)
            .maxAngleDiffToMerge(parameters.remoteMapParameters.maxAngleDiffToMerge)
            .maxDistanceForDistanceBasedMerging(parameters.remoteMapParameters.maxDistanceForDistanceBasedMerging)
            .maxDistanceToMerge(parameters.remoteMapParameters.maxDistanceToMerge)
            .initialValidityForNewHypothesis(parameters.remoteModel.hypotheses_initialValidityForNewHypotheses),
            teammate.number,
            RemoteKalmanPositionHypothesis::TeammateInfo(percept.validity, percept.timestampFromImage));
          // Update merged
          performSensorUpdate(sensorUpdateArguments(m_mergedKalmanRobotMap, kalmanNoiseMatrices)
            .measuredPositionRelative(relativePosition)
            .measuredDistance(percept.distance)
            .desiredRobotType(percept.robotType)
            .timestamp(percept.timestampFromImage)
            .perceptValidity(percept.validity * parameters.mergedMapParameters.teammatePerceptInfluence)
            .maxAngleDiffToMerge(parameters.mergedMapParameters.maxAngleDiffToMerge)
            .maxDistanceForDistanceBasedMerging(parameters.mergedMapParameters.maxDistanceForDistanceBasedMerging)
            .maxDistanceToMerge(parameters.mergedMapParameters.maxDistanceToMerge)
            .initialValidityForNewHypothesis(parameters.remoteModel.hypotheses_initialValidityForNewHypotheses),
            teammate.number,
            RemoteKalmanPositionHypothesis::TeammateInfo(percept.validity, percept.timestampFromImage));
        }
      }
      */
    }
  }

  // Update validities after sensor update
  updateValidities();
}

void KalmanMultiRobotMapProvider::pruneHypotheses()
{
  // Merge hypotheses
  mergeHypotheses(m_localKalmanRobotMap, parameters.localMapParameters.maxAngleDiffToMerge, parameters.localMapParameters.maxDistanceToMerge);
  mergeHypotheses(m_remoteKalmanRobotMap, parameters.remoteMapParameters.maxAngleDiffToMerge, parameters.remoteMapParameters.maxDistanceToMerge);
  mergeHypotheses(m_mergedKalmanRobotMap, parameters.mergedMapParameters.maxAngleDiffToMerge, parameters.mergedMapParameters.maxDistanceToMerge);

  // Prune because of validity
  m_localKalmanRobotMap.cleanUpHypothesesLowValidity(parameters.localPercept.cleanUpHypotheses_belowValidity, false);
  m_remoteKalmanRobotMap.cleanUpHypothesesLowValidity(parameters.remoteModel.cleanUpHypotheses_belowValidity, false);
  m_mergedKalmanRobotMap.cleanUpHypothesesLowValidity(parameters.remoteModel.cleanUpHypotheses_belowValidity, false);

  // Clean outside of field
  if (parameters.cleanUpHypotheses_outsideField)
  {
    m_localKalmanRobotMap.cleanUpHypothesesOutsideField(theFieldDimensions, theRobotPose, parameters.cleanUpHypotheses_fieldBorderThreshold);

    m_remoteKalmanRobotMap.cleanUpHypothesesOutsideField(theFieldDimensions, theRobotPose, parameters.cleanUpHypotheses_fieldBorderThreshold);

    m_mergedKalmanRobotMap.cleanUpHypothesesOutsideField(theFieldDimensions, theRobotPose, parameters.cleanUpHypotheses_fieldBorderThreshold);
  }
}

void KalmanMultiRobotMapProvider::updateRobotTypes()
{
  // Update local robot type
  for (unsigned i = 0; i < m_localKalmanRobotMap.size(); i++)
  {
    m_localKalmanRobotMap[i].updateRobotType(parameters.localPercept.validity_minPerceptsPerSecond, parameters.localPercept.validity_maxPerceptsPerSecond, theFrameInfo.time);
  }
  // Update remote robot type
  for (unsigned i = 0; i < m_remoteKalmanRobotMap.size(); i++)
  {
    m_remoteKalmanRobotMap[i].updateRobotType(parameters.remoteModel.validity_minPerceptsPerSecond, parameters.remoteModel.validity_maxPerceptsPerSecond, theFrameInfo.time);
  }
  // Update combined robot type
  for (unsigned i = 0; i < m_mergedKalmanRobotMap.size(); i++)
  {
    m_mergedKalmanRobotMap[i].updateRobotType(parameters.localPercept.validity_minPerceptsPerSecond, parameters.localPercept.validity_maxPerceptsPerSecond, theFrameInfo.time);
  }
}

void KalmanMultiRobotMapProvider::generateMaps()
{
  // Size of local robot map
  auto lSize = m_localKalmanRobotMap.size();
  // Clear old map
  m_localRobotMap.robots.clear();
  // Reserve size
  m_localRobotMap.robots.reserve(lSize);
  // Generate maps
  for (unsigned i = 0; i < lSize; i++)
  {
    // Add to map
    // Convert positionRelative to positionOnField prior
    auto mapEntry = m_localKalmanRobotMap[i].getRobotMapEntry();
    mapEntry.pose.translation = Transformation::robotToField(theRobotPose, mapEntry.pose.translation);

    m_localRobotMap.robots.push_back(mapEntry);
  }

  // Size of remote robot map
  auto rSize = m_remoteKalmanRobotMap.size();
  // Clear old map
  m_remoteRobotMap.robots.clear();
  // Reserve size
  m_remoteRobotMap.robots.reserve(rSize);
  // Generate maps
  for (unsigned i = 0; i < rSize; i++)
  {
    // Add to map
    // Convert positionRelative to positionOnField prior
    auto mapEntry = m_remoteKalmanRobotMap[i].getRobotMapEntry();
    mapEntry.pose.translation = Transformation::robotToField(theRobotPose, mapEntry.pose.translation);

    m_remoteRobotMap.robots.push_back(mapEntry);
  }

  // Clear old map
  m_robotMap.robots.clear();
  // Fill map
  // Size of merged robot map
  auto mSize = m_mergedKalmanRobotMap.size();
  // Reserve size
  m_robotMap.robots.reserve(mSize);
  // Generate maps
  for (unsigned i = 0; i < mSize; i++)
  {
    // Add to map
    // Convert positionRelative to positionOnField prior
    auto mapEntry = m_mergedKalmanRobotMap[i].getRobotMapEntry();
    mapEntry.pose.translation = Transformation::robotToField(theRobotPose, mapEntry.pose.translation);

    m_robotMap.robots.push_back(mapEntry);
  }

  //perceptBuffer.clear();
}


// ==========================
// DEBUGGING
// ==========================

void KalmanMultiRobotMapProvider::modifyInternalParameters() {}

void KalmanMultiRobotMapProvider::initDebugging()
{
  DECLARE_DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:localHypotheses", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:remoteHypotheses", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:mergedHypotheses", "drawingOnField");

  DECLARE_DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:perceptBuffer", "drawingOnField");

  DECLARE_DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:perceptMerge", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:hypothesisMerge", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:perceptCreate", "drawingOnField");

  //POC
  DECLARE_DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:sonarPerceptLeft", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:sonarPerceptRight", "drawingOnField");

  DEBUG_RESPONSE_ONCE("module:KalmanMultiRobotMapProvider:reset") reset();
}

void KalmanMultiRobotMapProvider::doGlobalDebugging()
{
  static const ColorRGBA teammateColor = ColorRGBA(150, 255, 0); // light green
  static const ColorRGBA opponentColor = ColorRGBA::red;
  static const ColorRGBA unknownColor = ColorRGBA::black;
  static const float size = 50.f;

  DRAW_MAP("module:KalmanMultiRobotMapProvider:localHypotheses", m_localKalmanRobotMap);
  DRAW_MAP("module:KalmanMultiRobotMapProvider:remoteHypotheses", m_remoteKalmanRobotMap);
  DRAW_MAP("module:KalmanMultiRobotMapProvider:mergedHypotheses", m_mergedKalmanRobotMap);
}


// ==========================
// HELPERS
// ==========================

void KalmanMultiRobotMapProvider::updateValidities()
{
  // Update local validities
  m_localKalmanRobotMap.updateValidity(parameters.localPercept.validity_maxPerceptsPerSecond,
      parameters.localPercept.goodValidity(),
      parameters.localPercept.validity_weightOfPreviousValidity,
      parameters.localPercept.validity_weightOfPreviousValidity_goodHypotheses);
  // Update local validities
  m_remoteKalmanRobotMap.updateValidity(parameters.remoteModel.validity_maxPerceptsPerSecond,
      parameters.remoteModel.goodValidity(),
      parameters.remoteModel.validity_weightOfPreviousValidity,
      parameters.remoteModel.validity_weightOfPreviousValidity_goodHypotheses);
  // Update merged validities
  m_mergedKalmanRobotMap.updateValidity(parameters.localPercept.validity_maxPerceptsPerSecond,
      parameters.localPercept.goodValidity(),
      parameters.localPercept.validity_weightOfPreviousValidity,
      parameters.localPercept.validity_weightOfPreviousValidity_goodHypotheses);
}

void KalmanMultiRobotMapProvider::getAngles(Vector2a& angles, const Vector2f& relativePosition)
{
  const CameraMatrix& cameraMatrix = theCameraMatrixUpper;
  angles.x() = std::atan2(relativePosition.y(), relativePosition.x());
  angles.y() = std::atan2(cameraMatrix.translation.z(), relativePosition.norm());
}

static Angle normalizeAngle(Angle a)
{
  return a.normalize();
}

template <typename RobotMap> void KalmanMultiRobotMapProvider::mergeHypotheses(RobotMap& robotMap, const Vector2a& mergeAngleDiff, const float mergeDistance)
{
  // Iterate over hypotheses
  for (unsigned i = 0; i < robotMap.size(); i++)
  {
    auto& iRobot = robotMap[i];
    // Skip hypotheses which are to be removed
    if (iRobot.validity == 0)
      continue;
    Vector2a iRobotPositionRelativeAngles, hypothesisPositionRelativeAngles;
    getAngles(iRobotPositionRelativeAngles, iRobot.kalman.position()); // iRobot.kalman.position());
    for (unsigned j = i + 1; j < robotMap.size(); j++)
    {
      auto& jRobot = robotMap[j];
      // Skip hypotheses which are to be removed
      if (jRobot.validity == 0)
        continue;

      getAngles(hypothesisPositionRelativeAngles, jRobot.kalman.position()); // jRobot.kalman.position());
      // Calculate head angles between the perception and the current hypothesis.
      Angle yDiff = std::abs(normalizeAngle(iRobotPositionRelativeAngles.y() - hypothesisPositionRelativeAngles.y()));
      Angle xDiff = std::abs(normalizeAngle(iRobotPositionRelativeAngles.x() - hypothesisPositionRelativeAngles.x()));

      // Get distance between measurements
      float distance = Geometry::distance(iRobot.kalman.position(), jRobot.kalman.position());
      // Get differene in robot type
      float robotTypeDifference = std::abs(iRobot.robotTypeValidity() - jRobot.robotTypeValidity());

      // Check whether hypotheses close enough for merging
      if (((yDiff < mergeAngleDiff.y() && xDiff < mergeAngleDiff.x()) || distance < mergeDistance) && robotTypeDifference < parameters.localMapParameters.maxRobotTypeDifferenceToMerge)
      {
        DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:hypothesisMerge", "drawingOnField")
        {

          bool isLocalMap = static_cast<void*>(&robotMap) == static_cast<void*>(&m_localKalmanRobotMap);
          if (isLocalMap)
          {
            ARROW("module:KalmanMultiRobotMapProvider:hypothesisMerge",
                jRobot.kalman.position().x(),
                jRobot.kalman.position().y(),
                iRobot.kalman.position().x(),
                iRobot.kalman.position().y(),
                15,
                Drawings::solidPen,
                ColorRGBA::red);
          }
        }
        // Merge
        iRobot.merge(jRobot);
        // Set validity of the other to 0 to prune it later
        jRobot.validity = 0;
      }
    }
  }
}

template <typename RobotMap>
typename RobotMap::HypothesisType* KalmanMultiRobotMapProvider::findNearestHypothesis(
    RobotMap& robotMap, Vector2a& angleDiff, const Vector2f& measuredPositionRelative, float& distance, RobotEstimate::RobotType desiredRobotType)
{
  typename RobotMap::HypothesisType* nearestHypothesisAngle = nullptr;
  Angle minAngleDiff = -1;
  Angle minAngleDiffX = -1;
  Angle minAngleDiffY = -1;
  float minDistance = -1;

  Vector2a measuredPositionRelativeAngles, hypothesisPositionRelativeAngles;
  getAngles(measuredPositionRelativeAngles, measuredPositionRelative);

  for (size_t i = 0; i < robotMap.size(); i++)
  {
    auto& actualHypothesis = robotMap[i];
    Vector2f positionRelative = actualHypothesis.kalman.position();
    //position = BallPhysics::propagateBallPosition(position, actualHypothesis.kalman.velocity(), 30.f / 1000.f, friction);
    // Only consider hypotheses with the correct robot type.
    if (!actualHypothesis.matchRobotType(desiredRobotType))
      continue;

    getAngles(hypothesisPositionRelativeAngles, positionRelative);
    // Calculate distance from the perception to the current hypothesis.
    float currentDistance = Geometry::distance(measuredPositionRelative, actualHypothesis.kalman.position());

    // Calculate head angles between the perception and the current hypothesis.
    Angle yDiff = std::abs(normalizeAngle(measuredPositionRelativeAngles.y() - hypothesisPositionRelativeAngles.y()));
    Angle xDiff = std::abs(normalizeAngle(measuredPositionRelativeAngles.x() - hypothesisPositionRelativeAngles.x()));

    // Check whether the angle diff sum is smaller than the minimum of all previous hypotheses.
    if (minAngleDiff < 0 || (xDiff + yDiff) < minAngleDiff)
    {
      nearestHypothesisAngle = &actualHypothesis;
      minAngleDiff = (xDiff + yDiff);
      minAngleDiffY = yDiff;
      minAngleDiffX = xDiff;
      minDistance = currentDistance;
    }
  }

  angleDiff.y() = minAngleDiffY;
  angleDiff.x() = minAngleDiffX;
  distance = minDistance;

  return nearestHypothesisAngle;
}

template <template <typename T> class SensorUpdateArgs, typename RobotMap>
typename RobotMap::HypothesisType* KalmanMultiRobotMapProvider::performSensorUpdate(const SensorUpdateArgs<RobotMap>& args)
{
  if (args.perceptValidity() <= 0.f)
    return nullptr; // Check whether percept validity is positive before performing update

  // Need to re-implement this method as we want to take robot type into account when searching for nearest hypothesis to update

  Vector2a angleDiff;
  float distanceToNearestHypothesis;

  typename RobotMap::HypothesisType* nearestHypothesis = findNearestHypothesis(args.robotMap(), angleDiff, args.measuredPositionRelative(), distanceToNearestHypothesis, args.desiredRobotType());
  float measurementNoiseFactor = powf(1.25, args.measuredDistance() / 1000.f < 1.f ? 1.f : args.measuredDistance() / 1000.f); // convert distance to m
  float angleDistanceFactor = 9.f / (args.measuredDistance() / 1000.f);

  Vector2a localMergeAngleDiff(args.maxAngleDiffToMerge().x() * angleDistanceFactor, args.maxAngleDiffToMerge().y() * angleDistanceFactor);

  if (nearestHypothesis != nullptr)
  {
    localMergeAngleDiff.x() = localMergeAngleDiff.x() * nearestHypothesis->mergeFactor;
    localMergeAngleDiff.y() = localMergeAngleDiff.y() * nearestHypothesis->mergeFactor;
  }


  if (nearestHypothesis != nullptr
      && ((args.measuredDistance() < args.maxDistanceForDistanceBasedMerging() && distanceToNearestHypothesis < args.maxDistanceToMerge()) // distancebase-merging
          || (angleDiff.y() < localMergeAngleDiff.y() && angleDiff.x() < localMergeAngleDiff.x()))) // anglebased-merging
  {
    // TODO: make parameter
    // Set measurementNoiseMatrix according to the direction from robot to measurement position.
    // In this direction (^= distance) the noise is set to a higher value than in the orthogonal direction (^= angle).
    float max = static_cast<float>(nearestHypothesis->kalman.matrices.noise.maxMeasurementNoise);
    max *= measurementNoiseFactor;
    nearestHypothesis->kalman.matrices.noise.measurementNoiseMatrix =
        Covariance::create((Vector2f() << max / 5.f, max / 12.5f).finished(), args.measuredPositionRelative().angle()).template cast<double>();

    // Correct kalman filter with the perception.
    if (args.measuredVelocity() == nullptr)
      nearestHypothesis->sensorUpdate(args.measuredPositionRelative(), args.timestamp(), args.perceptValidity());
    else
      nearestHypothesis->sensorUpdate(args.measuredPositionRelative(), *args.measuredVelocity(), args.timestamp(), args.perceptValidity());

    DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:perceptMerge", "drawingOnField")
    {
      bool isLocalMap = static_cast<void*>(&args.robotMap()) == static_cast<void*>(&m_localKalmanRobotMap);
      if (isLocalMap)
      {
        ARROW("module:KalmanMultiRobotMapProvider:perceptMerge",
            args.measuredPositionRelative().x(),
            args.measuredPositionRelative().y(),
            nearestHypothesis->kalman.position().x(),
            nearestHypothesis->kalman.position().y(),
            15,
            Drawings::solidPen,
            ColorRGBA::white);
      }
    }
  }
  else
  {
    bool isLocalMap = static_cast<void*>(&args.robotMap()) == static_cast<void*>(&m_localKalmanRobotMap);
    // Add new hypothesis if all existing ones are too far.
    typename RobotMap::HypothesisType newHypothesis(
        args.kalmanNoiseMatrices(), args.initialValidityForNewHypothesis(), args.timestamp(), args.perceptValidity(), args.measuredPositionRelative(), isLocalMap ? LOCAL_PERCEPT_DURATION : REMOTE_PERCEPT_DURATION);

    DEBUG_DRAWING("module:KalmanMultiRobotMapProvider:perceptCreate", "drawingOnField")
    {
      if (isLocalMap)
      {
        CIRCLE("module:KalmanMultiRobotMapProvider:perceptCreate",
            args.measuredPositionRelative().x(),
            args.measuredPositionRelative().y(),
            100,
            30,
            Drawings::solidPen,
            ColorRGBA::yellow,
            Drawings::solidBrush,
            ColorRGBA(255, 255, 255, 0));
      }
    }

    args.robotMap().addHypothesis(std::move(newHypothesis));
    nearestHypothesis = &args.robotMap().back();
  }

  // Update robot type sensor
  nearestHypothesis->addPerceptRobotType(args.desiredRobotType(), args.timestamp());

  return nearestHypothesis;
}

template <template <typename T> class SensorUpdateArgs, typename RobotMap>
void KalmanMultiRobotMapProvider::performSensorUpdate(const SensorUpdateArgs<RobotMap>& args, int playerNumber, const typename RemoteRobotMapHypothesis::TeammateInfo& teammate)
{
  typename RobotMap::HypothesisType* nearestHypothesis = performSensorUpdate(args);

  // Add a teammate label to the hypothesis.
  if (nearestHypothesis) // Check that there is a hypothesis (handle perceptValidity <= 0 case)
    nearestHypothesis->addTeammateInfo(playerNumber, teammate);
}

MAKE_MODULE(KalmanMultiRobotMapProvider, modeling)
