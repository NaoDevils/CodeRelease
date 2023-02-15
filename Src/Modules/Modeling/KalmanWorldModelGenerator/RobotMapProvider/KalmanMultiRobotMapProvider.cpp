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

// ==========================
// UPDATE METHODS
// ==========================

void KalmanMultiRobotMapProvider::update(RobotMap& robotMap)
{
  execute();
  robotMap = m_robotMap;
}

void KalmanMultiRobotMapProvider::update(LocalRobotMap& localRobotMap)
{
  execute();
  localRobotMap = m_localRobotMap;
}

void KalmanMultiRobotMapProvider::update(RemoteRobotMap& remoteRobotMap)
{
  execute();
  remoteRobotMap = m_remoteRobotMap;
}


// ==========================
// INITIALIZE
// ==========================

void KalmanMultiRobotMapProvider::initialize()
{
  // Reset robot maps
  m_robotMap.reset();
  m_localRobotMap.reset();
  m_remoteRobotMap.reset();
}


// ==========================
// EXCUTE CODE
// ==========================

void KalmanMultiRobotMapProvider::execute()
{
  if (theFrameInfo.time == m_lastTimeStamp)
    return;

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


  // Save last execution time
  m_lastTimeStamp = theFrameInfo.time;
}

void KalmanMultiRobotMapProvider::motionUpdate()
{
  m_localKalmanRobotMap.motionUpdate(theFrameInfo.time, parameters.friction);
  m_remoteKalmanRobotMap.motionUpdate(theFrameInfo.time, parameters.friction);
  m_mergedKalmanRobotMap.motionUpdate(theFrameInfo.time, parameters.friction);
}

void KalmanMultiRobotMapProvider::sensorUpdate()
{
  // ===============================
  // Update local percepts
  // ===============================
  for (const RobotEstimate& robot : theRobotsPerceptUpper.robots)
  {
    // Transform to field coordinates
    Vector2f posOnField = Transformation::robotToField(theRobotPose, robot.locationOnField.translation);
    // Update sensor
    performSensorUpdate(m_localKalmanRobotMap,
        posOnField,
        robot.distance,
        nullptr,
        robot.robotType,
        theFrameInfo.time,
        robot.validity,
        parameters.localPercept.hypotheses_minDistanceForNewHypothesis,
        parameters.localPercept.hypotheses_initialValidityForNewHypotheses,
        kalmanNoiseMatrices);
    // Update merged
    performSensorUpdate(m_mergedKalmanRobotMap,
        posOnField,
        robot.distance,
        nullptr,
        robot.robotType,
        theFrameInfo.time,
        robot.validity * parameters.mergedMapParameters.localPerceptInfluence,
        parameters.localPercept.hypotheses_minDistanceForNewHypothesis,
        parameters.localPercept.hypotheses_initialValidityForNewHypotheses,
        kalmanNoiseMatrices,
        theRobotInfo.number,
        RemoteKalmanPositionHypothesis::TeammateInfo(robot.validity, theFrameInfo.time));
  }
  for (const RobotEstimate& robot : theRobotsPercept.robots)
  {
    // Transform to field coordinates
    Vector2f posOnField = Transformation::robotToField(theRobotPose, robot.locationOnField.translation);
    // Update sensor
    performSensorUpdate(m_localKalmanRobotMap,
        posOnField,
        robot.distance,
        nullptr,
        robot.robotType,
        theFrameInfo.time,
        robot.validity * parameters.mergedMapParameters.localPerceptInfluence,
        parameters.localPercept.hypotheses_minDistanceForNewHypothesis,
        parameters.localPercept.hypotheses_initialValidityForNewHypotheses,
        kalmanNoiseMatrices);
    // Update merged
    performSensorUpdate(m_mergedKalmanRobotMap,
        posOnField,
        robot.distance,
        nullptr,
        robot.robotType,
        theFrameInfo.time,
        robot.validity,
        parameters.localPercept.hypotheses_minDistanceForNewHypothesis,
        parameters.localPercept.hypotheses_initialValidityForNewHypotheses,
        kalmanNoiseMatrices,
        theRobotInfo.number,
        RemoteKalmanPositionHypothesis::TeammateInfo(robot.validity, theFrameInfo.time));
  }


  // ===============================
  // Update from teammate positions
  // ===============================
  for (const Teammate& teammate : theTeammateData.teammates)
  {
    // Loop over all players which have sent data and are active
    if (teammate.status == Teammate::FULLY_ACTIVE)
    {
      // Calculate distance
      auto relativePosition = Transformation::fieldToRobot(theRobotPose, teammate.pose.translation);
      float distance = relativePosition.norm();

      // Calculate speed
      const Pose2f& speed = teammate.speedInfo.speed;
      auto speed2d = Vector2f(speed.translation.x() * cos(speed.rotation), speed.translation.y() * sin(speed.rotation));
      auto relativeSpeed = Transformation::robotToFieldVelocity(teammate.pose, speed2d);

      // Update remote
      performSensorUpdate(m_remoteKalmanRobotMap,
          teammate.pose.translation,
          distance,
          &relativeSpeed,
          RobotEstimate::RobotType::teammateRobot,
          teammate.timeWhenSent,
          teammate.pose.validity * parameters.remoteMapParameters.teammatePositionInfluence,
          parameters.remoteModel.hypotheses_minDistanceForNewHypothesis,
          parameters.remoteModel.hypotheses_initialValidityForNewHypotheses,
          kalmanNoiseMatrices,
          teammate.number,
          RemoteKalmanPositionHypothesis::TeammateInfo(teammate.pose.validity, teammate.timeWhenSent));
      // Update merged
      performSensorUpdate(m_mergedKalmanRobotMap,
          teammate.pose.translation,
          distance,
          &relativeSpeed,
          RobotEstimate::RobotType::teammateRobot,
          teammate.timeWhenSent,
          teammate.pose.validity * parameters.mergedMapParameters.teammatePositionInfluence,
          parameters.remoteModel.hypotheses_minDistanceForNewHypothesis,
          parameters.remoteModel.hypotheses_initialValidityForNewHypotheses,
          kalmanNoiseMatrices,
          teammate.number,
          RemoteKalmanPositionHypothesis::TeammateInfo(teammate.pose.validity, teammate.timeWhenSent));
    }
  }


  // ===============================
  // Update from teammate data
  // ===============================
  for (const Teammate& teammate : theTeammateData.teammates)
  {
    // Loop over all players which have sent data and are active
    if (teammate.status == Teammate::FULLY_ACTIVE)
    {
      // ===== Update models =====
      for (const auto& robotModel : teammate.localRobotMap.robots)
      {
        // Check that is not me
        if ((robotModel.pose.translation - theRobotPose.translation).norm() > 200)
        {
          // Calculate distance
          auto relativePosition = Transformation::fieldToRobot(theRobotPose, robotModel.pose.translation);
          float distance = relativePosition.norm();

          // Update remote
          Vector2f percept = teammate.pose.translation;
          performSensorUpdate(m_remoteKalmanRobotMap,
              robotModel.pose.translation,
              distance,
              &robotModel.velocity,
              robotModel.robotType,
              teammate.timeWhenSent,
              robotModel.validity * parameters.remoteMapParameters.teammateModelInfluence,
              parameters.remoteModel.hypotheses_minDistanceForNewHypothesis,
              parameters.remoteModel.hypotheses_initialValidityForNewHypotheses,
              kalmanNoiseMatrices,
              teammate.number,
              RemoteKalmanPositionHypothesis::TeammateInfo(robotModel.validity, teammate.timeWhenSent));
          // Update merged
          performSensorUpdate(m_mergedKalmanRobotMap,
              robotModel.pose.translation,
              distance,
              &robotModel.velocity,
              robotModel.robotType,
              teammate.timeWhenSent,
              robotModel.validity * parameters.mergedMapParameters.teammateModelInfluence,
              parameters.remoteModel.hypotheses_minDistanceForNewHypothesis,
              parameters.remoteModel.hypotheses_initialValidityForNewHypotheses,
              kalmanNoiseMatrices,
              teammate.number,
              RemoteKalmanPositionHypothesis::TeammateInfo(robotModel.validity, teammate.timeWhenSent));
        }
      }

      // ===== Update percepts =====
      // Update from lower percepts
      for (const auto& percept : teammate.robotsPercept.robots)
      {
        // Check that is not me
        if ((percept.locationOnField.translation - theRobotPose.translation).norm() > 400)
        {
          // Update remote
          performSensorUpdate(m_remoteKalmanRobotMap,
              percept.locationOnField.translation,
              percept.distance,
              nullptr,
              percept.robotType,
              percept.timestampFromImage,
              percept.validity * parameters.remoteMapParameters.teammatePerceptInfluence,
              parameters.remoteModel.hypotheses_minDistanceForNewHypothesis,
              parameters.remoteModel.hypotheses_initialValidityForNewHypotheses,
              kalmanNoiseMatrices,
              teammate.number,
              RemoteKalmanPositionHypothesis::TeammateInfo(percept.validity, percept.timestampFromImage));
          // Update merged
          performSensorUpdate(m_mergedKalmanRobotMap,
              percept.locationOnField.translation,
              percept.distance,
              nullptr,
              percept.robotType,
              percept.timestampFromImage,
              percept.validity * parameters.remoteMapParameters.teammatePerceptInfluence,
              parameters.remoteModel.hypotheses_minDistanceForNewHypothesis,
              parameters.remoteModel.hypotheses_initialValidityForNewHypotheses,
              kalmanNoiseMatrices,
              teammate.number,
              RemoteKalmanPositionHypothesis::TeammateInfo(percept.validity, percept.timestampFromImage));
        }
      }
      // Update from upper percepts
      for (const auto& percept : teammate.robotsPerceptUpper.robots)
      {
        // Check that is not me
        if ((percept.locationOnField.translation - theRobotPose.translation).norm() > 400)
        {
          // Update remote
          performSensorUpdate(m_remoteKalmanRobotMap,
              percept.locationOnField.translation,
              percept.distance,
              nullptr,
              percept.robotType,
              percept.timestampFromImage,
              percept.validity * parameters.remoteMapParameters.teammatePerceptInfluence,
              parameters.remoteModel.hypotheses_minDistanceForNewHypothesis,
              parameters.remoteModel.hypotheses_initialValidityForNewHypotheses,
              kalmanNoiseMatrices,
              teammate.number,
              RemoteKalmanPositionHypothesis::TeammateInfo(percept.validity, percept.timestampFromImage));
          // Update merged
          performSensorUpdate(m_mergedKalmanRobotMap,
              percept.locationOnField.translation,
              percept.distance,
              nullptr,
              percept.robotType,
              percept.timestampFromImage,
              percept.validity * parameters.remoteMapParameters.teammatePerceptInfluence,
              parameters.remoteModel.hypotheses_minDistanceForNewHypothesis,
              parameters.remoteModel.hypotheses_initialValidityForNewHypotheses,
              kalmanNoiseMatrices,
              teammate.number,
              RemoteKalmanPositionHypothesis::TeammateInfo(percept.validity, percept.timestampFromImage));
        }
      }
    }
  }

  // Update validities after sensor update
  updateValidities();
}

void KalmanMultiRobotMapProvider::pruneHypotheses()
{
  // Merge hypotheses
  mergeHypotheses(m_localKalmanRobotMap);
  mergeHypotheses(m_remoteKalmanRobotMap);
  mergeHypotheses(m_mergedKalmanRobotMap);

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
    m_localRobotMap.robots.push_back(m_localKalmanRobotMap[i].getRobotMapEntry());
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
    m_remoteRobotMap.robots.push_back(m_remoteKalmanRobotMap[i].getRobotMapEntry());
  }

  // Clear old map
  m_robotMap.robots.clear();
  // Fill map
  if (parameters.useMergedMap)
  {
    // Size of merged robot map
    auto mSize = m_mergedKalmanRobotMap.size();
    // Reserve size
    m_robotMap.robots.reserve(mSize);
    // Generate maps
    for (unsigned i = 0; i < mSize; i++)
    {
      // Add to map
      m_robotMap.robots.push_back(m_mergedKalmanRobotMap[i].getRobotMapEntry());
    }
  }
  else
  {
    // Remember which hypotheses of the remote map have already been inserted because they're similar to a local model
    std::vector<const RemoteRobotMapHypothesis*> usedRemoteHypotheses;

    // Distance to closest
    float distance = -1;
    for (unsigned i = 0; i < m_localKalmanRobotMap.size(); i++)
    {
      // Save the hypothesis we're working on
      const auto& hypothesis = m_localKalmanRobotMap[i];

      // Find nearest pose in remote map
      const auto* nearestHypothesis = findNearestHypothesis(m_remoteKalmanRobotMap, hypothesis.kalman.position(), distance, hypothesis.robotType());

      // This can be treated as one robot?
      if (nearestHypothesis && distance < parameters.mergedMapParameters.maxDistanceToMerge)
      {
        // Mark this hypothesis as "in map"
        usedRemoteHypotheses.push_back(nearestHypothesis);

        // Get local and remote entry
        RobotMapEntry localModel = hypothesis.getRobotMapEntry();
        RobotMapEntry remoteModel = nearestHypothesis->getRobotMapEntry();

        // Combined entry
        RobotMapEntry entry;

        // ====== "Merge" entries =====
        // Translation mean
        entry.pose.translation = (localModel.pose.translation + remoteModel.pose.translation) / 2;
        // Shift both rotations to [0, 2pi], divide by 2 (mean) and shift back to [-pi, pi]. Then normalize (should already be the case)
        entry.pose.rotation = Angle::normalize((localModel.pose.rotation + remoteModel.pose.rotation + pi2) / 2 - pi);
        // Detect robot type
        auto localType = localModel.robotType;
        // Use localType if not unknown (robot types have already been matched in findNearestHypothesis (only chance their not the same is localType is unknown)
        entry.robotType = localType != RobotEstimate::RobotType::unknownRobot ? localType : remoteModel.robotType;
        // Mean validity
        entry.validity = (localModel.validity + remoteModel.validity) / 2;
        // Mean velocity
        entry.velocity = (localModel.velocity + remoteModel.velocity) / 2;

        // Add to map
        m_robotMap.robots.push_back(std::move(entry));
      }
      else
      {
        // Add to map
        m_robotMap.robots.push_back(hypothesis.getRobotMapEntry());
      }
    }

    // Remote map
    for (unsigned i = 0; i < m_remoteKalmanRobotMap.size(); i++)
    {
      // Save the hypothesis we're working on
      const auto& hypothesis = m_remoteKalmanRobotMap[i];

      bool alreadyAdded = false;
      // Check whether this robot has already been added
      for (auto testHypo : usedRemoteHypotheses)
      {
        alreadyAdded = (testHypo == &hypothesis);

        if (alreadyAdded)
          break;
      }

      if (!alreadyAdded)
      {
        m_robotMap.robots.push_back(hypothesis.getRobotMapEntry());
      }
    }
  }
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

MAKE_MODULE(KalmanMultiRobotMapProvider, modeling)
