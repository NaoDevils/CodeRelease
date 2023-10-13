/**
 * \file KalmanRobotMapProvider.cpp
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 */

#include "KalmanRobotMapProvider.h"

MAKE_MODULE(KalmanRobotMapProvider, modeling)


// MARK: Update methods for representations

void KalmanRobotMapProvider::update(RobotMap& robotMap)
{
  robotMap = m_robotMap;
}

// MARK: KalmanRobotMapProvider methods

void KalmanRobotMapProvider::execute(tf::Subflow&)
{
  // --- Debug methods ---

  // Allow modification of some parameters in SimRobot.
  initModify();
  // Init debug drawings.
  initDebugDrawing();


  // --- Filter ---

  // Perform prediction step.
  motionUpdate();
  // Perform correction step.
  sensorUpdate();


  // Update robot type of each hypothesis.
  m_multiKalmanRobotMap.updateRobotTypes(parameters.validity.Validity_minPerceptsPerSecond, parameters.validity.Validity_maxPerceptsPerSecond, theFrameInfo.time);

  // Remove hypotheses with too small validities and those outside the field.
  m_multiKalmanRobotMap.cleanUpHypothesesLowValidity(parameters.CleanUpHypotheses_belowValidity, false);
  if (parameters.CleanUpHypotheses_outsideField)
  {
    m_multiKalmanRobotMap.cleanUpHypothesesOutsideField(theFieldDimensions, theRobotPose, parameters.CleanUpHypotheses_fieldBorderThreshold);
  }

  // Create standard robot map object from multiple kalman model.
  generateRemoteRobotMap();

  // --- Debug methods ---

  // Do debug drawings.
  draw();
  plot();
}

void KalmanRobotMapProvider::motionUpdate()
{
  // Perform prediction step on all hypotheses
  m_multiKalmanRobotMap.motionUpdate(theFrameInfo.time, parameters.friction); // friction <= 0
}

void KalmanRobotMapProvider::sensorUpdate()
{
  sensorUpdateTeammates();
  sensorUpdateLocal();
  //sensorUpdateRemote();

  // Update validity of all hypotheses.
  m_multiKalmanRobotMap.updateValidity(
      parameters.validity.Validity_maxPerceptsPerSecond, parameters.validity.goodValidity(), parameters.validity.Validity_weightOfPreviousValidity, parameters.validity.Validity_weightOfPreviousValidity_goodHypotheses);
}

void KalmanRobotMapProvider::sensorUpdateTeammates()
{
  // Collect robot pose of all team mates.
  for (const TeammateReceived& teammate : theTeammateData.teammates)
  {
    // Loop over all players which have sent data and are active.
    if (teammate.status == TeammateReceived::Status::FULLY_ACTIVE)
    {
      // Transform local robot coordinates to global field coordinates
      Vector2f percept = teammate.robotPose.translation;
      sensorUpdateSingle(percept, 0.f, RobotEstimate::RobotType::teammateRobot, teammate.sendTimestamp, teammate.robotPose.validity, teammate.playerNumber);
    }
  }
}

void KalmanRobotMapProvider::sensorUpdateLocal()
{
  for (const RobotEstimate& robot : theRobotsPercept.robots)
  {
    // Transform local robot coordinates to global field coordinates
    Vector2f percept = Transformation::robotToField(theRobotPose, robot.locationOnField.translation);
    sensorUpdateSingle(percept, robot.distance, robot.robotType, theFrameInfo.time, robot.validity, theRobotInfo.number);
  }
}

void KalmanRobotMapProvider::sensorUpdateRemote()
{
  // Not available as long as RobotsPercepts are not transmitted

  //// Collect robot map information from all team mates.
  //for (const TeammateReceived& teammate : theTeammateData.teammates)
  //{
  //  // Loop over all players which have sent data and are active.
  //  if (teammate.status == TeammateReceived::Status::FULLY_ACTIVE)
  //  {
  //    for (const RobotEstimate& robot : teammate.robotsPercept.robots)
  //    {
  //      // Transform local robot coordinates to global field coordinates.
  //      Vector2f percept = Transformation::robotToField(teammate.robotPose, robot.locationOnField.translation);

  //      // Ignore remote percepts which could be this robot itself.
  //      float dist = Geometry::distance(percept, theRobotPose.translation);
  //      // Estimate horizontal angle difference from teammate to percept and *this robot.
  //      float horizontalAnglePercept = Geometry::angleTo(teammate.robotPose, percept);
  //      float horizontalAngleThisRobot = Geometry::angleTo(teammate.robotPose, theRobotPose.translation);
  //      float horizontalAngleDiff = Geometry::absDifferenceBetweenTwoAngles(horizontalAnglePercept, horizontalAngleThisRobot);
  //      // Estimate vertical angle difference from teammates camera to percept and *this robot.
  //      // (Robot height estimated by 550 mm)
  //      float verticalAnglePercept = atan2f(robot.distance, 550.f);
  //      float verticalAngleThisRobot = atan2f(Geometry::distance(teammate.robotPose.translation, theRobotPose.translation), 550.f);
  //      float verticalAngleDiff = Geometry::absDifferenceBetweenTwoAngles(verticalAnglePercept, verticalAngleThisRobot);
  //      // TODO: make params?
  //      if (dist >= 500.f && (horizontalAngleDiff >= 0.5f || verticalAngleDiff >= 0.25f))
  //      {
  //        sensorUpdateSingle(percept, robot.distance, robot.robotType, teammate.sendTimestamp, robot.validity, teammate.playerNumber);
  //      }
  //    }
  //  }
  //}
}

void KalmanRobotMapProvider::sensorUpdateSingle(const Vector2f& position, float distanceToRobot, RobotEstimate::RobotType robotType, unsigned timeStamp, float perceptValidity, int playerNumber)
{
  // Teammate information.
  RemoteRobotMapHypothesis::TeammateInfo teammate(perceptValidity, timeStamp);

  // Run sensor update with global ball position.
  m_multiKalmanRobotMap.sensorUpdate(position,
      distanceToRobot,
      robotType, // Restrict matching hypotheses to those matching to the given robot type.
      timeStamp,
      perceptValidity,
      parameters.validity.Hypotheses_minDistanceForNewHypothesis,
      parameters.validity.Hypotheses_initialValidityForNewHypotheses,
      kalmanNoiseMatrices,
      playerNumber,
      teammate);
}

void KalmanRobotMapProvider::generateRemoteRobotMap()
{
  m_robotMap.robots.resize(m_multiKalmanRobotMap.size());
  for (size_t i = 0; i < m_multiKalmanRobotMap.size(); i++)
  {
    // Only include robots with enough validity.
    if (m_multiKalmanRobotMap[i].validity >= parameters.validity.minValidity())
    {
      m_multiKalmanRobotMap[i].updateRobotMapEntry(m_robotMap.robots[i]);
    }
  }
}


// ---------- Debug methods ----------

void KalmanRobotMapProvider::initModify()
{
  MODIFY("module:KalmanRobotMapProvider:parameters", parameters);
  MODIFY("module:KalmanRobotMapProvider:kalmanMatrices", kalmanNoiseMatrices);
  MODIFY("module:KalmanRobotMapProvider:hypotheses", m_multiKalmanRobotMap);
}

void KalmanRobotMapProvider::initDebugDrawing() const
{
  DECLARE_DEBUG_DRAWING("module:KalmanRobotMapProvider:hypotheses", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:KalmanRobotMapProvider:validity", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:KalmanRobotMapProvider:covariances", "drawingOnField");
}

void KalmanRobotMapProvider::draw() const
{
  DEBUG_DRAWING("module:KalmanRobotMapProvider:hypotheses", "drawingOnField")
  {
    m_multiKalmanRobotMap.draw();
  }
}

void KalmanRobotMapProvider::plot() const
{
#ifndef TARGET_ROBOT // Compute plot values only in SimRobot
  /*
   vp kalmanRobotModelValidity 300 0 1 validity s 0.033
   vpd kalmanRobotModelValidity module:KalmanRobotMapProvider:validityMin gray
   vpd kalmanRobotModelValidity module:KalmanRobotMapProvider:validityGood black
   vpd kalmanRobotModelValidity module:KalmanRobotMapProvider:meanValidity red

   vp kalmanRobotModelPPS 300 0 30 PPS s 0.033
   vpd kalmanRobotModelPPS module:KalmanRobotMapProvider:ppsMin gray
   vpd kalmanRobotModelPPS module:KalmanRobotMapProvider:ppsMax black
   vpd kalmanRobotModelPPS module:KalmanRobotMapProvider:ppsGood green
   vpd kalmanRobotModelPPS module:KalmanRobotMapProvider:meanPPS red
   */

  // Number of hypotheses:
  DECLARE_PLOT("module:KalmanRobotMapProvider:numberOfHypotheses");
  PLOT("module:KalmanRobotMapProvider:numberOfHypotheses", m_multiKalmanRobotMap.size());

  // Validities:
  float meanValidity = 0.f;
  float meanPPS = 0.f;
  float meanRobotTypeValidity = 0.f;
  for (size_t i = 0; i < m_multiKalmanRobotMap.size(); i++)
  {
    meanValidity += m_multiKalmanRobotMap[i].validity;
    meanPPS += m_multiKalmanRobotMap[i].perceptsPerSecond();
    meanRobotTypeValidity += m_multiKalmanRobotMap[i].robotTypeValidity();
  }
  meanValidity = meanValidity / static_cast<float>(m_multiKalmanRobotMap.size());
  meanPPS = meanPPS / static_cast<float>(m_multiKalmanRobotMap.size());

  DECLARE_PLOT("module:KalmanRobotMapProvider:validityMin");
  DECLARE_PLOT("module:KalmanRobotMapProvider:validityGood");
  PLOT("module:KalmanRobotMapProvider:validityMin", parameters.validity.minValidity());
  PLOT("module:KalmanRobotMapProvider:validityGood", parameters.validity.goodValidity());
  DECLARE_PLOT("module:KalmanRobotMapProvider:meanValidity");
  PLOT("module:KalmanRobotMapProvider:meanValidity", meanValidity);

  // Percepts per second:
  DECLARE_PLOT("module:KalmanRobotMapProvider:ppsMin");
  DECLARE_PLOT("module:KalmanRobotMapProvider:ppsMax");
  DECLARE_PLOT("module:KalmanRobotMapProvider:ppsGood");
  PLOT("module:KalmanRobotMapProvider:ppsMin", parameters.validity.Validity_minPerceptsPerSecond);
  PLOT("module:KalmanRobotMapProvider:ppsMax", parameters.validity.Validity_maxPerceptsPerSecond);
  PLOT("module:KalmanRobotMapProvider:ppsGood", parameters.validity.Validity_goodPerceptsPerSecond);
  DECLARE_PLOT("module:KalmanRobotMapProvider:meanPPS");
  PLOT("module:KalmanRobotMapProvider:meanPPS", meanPPS);

  // Robot type validities:
  DECLARE_PLOT("module:KalmanRobotMapProvider:RobotType_opponent");
  DECLARE_PLOT("module:KalmanRobotMapProvider:RobotType_teammate");
  PLOT("module:KalmanRobotMapProvider:RobotType_opponent", RemoteRobotMapHypothesis::ROBOT_TYPE_VALIDITY_OPPONENT);
  PLOT("module:KalmanRobotMapProvider:RobotType_teammate", RemoteRobotMapHypothesis::ROBOT_TYPE_VALIDITY_TEAMMATE);
  DECLARE_PLOT("module:KalmanRobotMapProvider:meanRobotType");
  PLOT("module:KalmanRobotMapProvider:meanRobotType", meanRobotTypeValidity);
#endif
}
