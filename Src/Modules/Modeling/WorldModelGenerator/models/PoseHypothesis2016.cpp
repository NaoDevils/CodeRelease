/**
 * @file PoseHypothesis2016.cpp
 *
 * Implementation of a class which represents a single world map in form of an EKF
 *
 * @author <a href="mailto:stefan.tasse@tu-dortmund.de">Stefan Tasse</a>
 * @author <a href="mailto:daniel.hauschildt@tu-dortmund.de">Daniel Hauschildt</a>
 * @author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>

 * @issue check the copy constructors
 *
 */



#include "PoseHypothesis2016.h"
#include "Tools/Modeling/PoseGenerator.h"

namespace
{

  static uint64_t lastUniqueId;
  static uint64_t getNextUniqueId()
  {
    return ++lastUniqueId;
  }

  static Matrix3d   poseCovarianceForLineMatching;
}

tls_type Eigen::Matrix<double, 2, PoseHypothesis2016::totalDimension>* PoseHypothesis2016::angleObservationMeasurementModelJacobian_H = 0;
tls_type Eigen::Matrix<double, 3, PoseHypothesis2016::totalDimension>* PoseHypothesis2016::infiniteLineMeasurementModelJacobian_H = 0;

tls_type PoseHypothesis2016::KalmanStateUpdate* PoseHypothesis2016::stateUpdate = 0;
tls_type PoseHypothesis2016::KalmanInfiniteLineStateUpdate* PoseHypothesis2016::stateUpdateInfiniteLine = 0;

PoseHypothesis2016::PoseHypothesis2016(const Pose2f & newPose, float _positionConfidence, double _symmetryConfidence, const unsigned &timeStamp, const SelfLocator2016Parameters& parameters) :
positionConfidence(_positionConfidence),
normalizedPositionConfidence(_positionConfidence),
symmetryConfidence(_symmetryConfidence),
timeOfLastPrediction(0),
initialized(false)
{
  covariance = Eigen::Matrix<double, totalDimension,totalDimension>::Identity();
  covariance *= 100000.0;
  _uniqueId = getNextUniqueId();
  creationTime = timeStamp;
  init(newPose, parameters);
}
PoseHypothesis2016::PoseHypothesis2016(const PoseHypothesis2016& other, const unsigned &timeStamp)
{
  *this = other;
  creationTime = timeStamp;
}

PoseHypothesis2016& PoseHypothesis2016::operator=(const PoseHypothesis2016& other)
{
  _uniqueId = getNextUniqueId();
  if (this == &other)
  {
    return *this;
  }
  correspondencesInGlobalCoordsWithLocalFeaturePerceptionsInfiniteLines.clear();
  correspondencesInGlobalCoordsWithLocalFeaturePerceptionsSpherical.clear();
  covariance = other.covariance;
  debugLineMatching = other.debugLineMatching;
  initialized = other.initialized;
  normalizedPositionConfidence = other.normalizedPositionConfidence;
  observationAnglesWithLocalFeaturePerceptionsSpherical.clear();
  observationsAsNormalsWithLocalFeaturePerceptionsInfiniteLines.clear();
  positionConfidence = other.positionConfidence;
  singleMeasurementCovariance_2x2 = other.singleMeasurementCovariance_2x2;
  state = other.state;
  symmetryConfidence = other.symmetryConfidence;
  timeOfLastPrediction = other.timeOfLastPrediction;
  temporaryLocalSymmetryLikelihood = other.temporaryLocalSymmetryLikelihood;
  creationTime = other.creationTime;
  return *this;
}

void PoseHypothesis2016::cleanup()
{
  if (angleObservationMeasurementModelJacobian_H != 0)
  {
    delete angleObservationMeasurementModelJacobian_H;
    angleObservationMeasurementModelJacobian_H = 0;
  }
  if (infiniteLineMeasurementModelJacobian_H != 0)
  {
    delete infiniteLineMeasurementModelJacobian_H;
    infiniteLineMeasurementModelJacobian_H = 0;
  }
  if (stateUpdate != 0)
  {
    delete stateUpdate;
    stateUpdate = 0;
  }
  if (stateUpdateInfiniteLine != 0)
  {
    delete stateUpdateInfiniteLine;
    stateUpdateInfiniteLine = 0;
  }
}

void PoseHypothesis2016::init(const Pose2f& newPose, const SelfLocator2016Parameters& parameters)
{
  initialized = true;


  // set the new state
  state[0] = newPose.translation.x();
  state[1] = newPose.translation.y();
  state[2] = newPose.rotation;

  // set initial noise from "measurement noise"
  covariance = Eigen::Matrix<double, totalDimension, totalDimension>::Identity();
  covariance *= 100000;

  Matrix2d singleMeasurementCovariance = Matrix2d::Zero();
  singleMeasurementCovariance(0, 0) = parameters.sensorUpdate_verticalAngleVariance;
  singleMeasurementCovariance(1, 1) = parameters.sensorUpdate_horizontalAngleVariance;

  singleMeasurementCovariance_2x2(0,0) = parameters.sensorUpdate_verticalAngleVariance;
  singleMeasurementCovariance_2x2(1,1) = parameters.sensorUpdate_horizontalAngleVariance;
  singleMeasurementCovariance_2x2(1,0) = singleMeasurementCovariance_2x2(0,1) = 0;

  Matrix3d singleInfiniteLineMeasurementCovariance = Matrix3d::Zero();
  singleInfiniteLineMeasurementCovariance(0, 0) = parameters.sensorUpdate_projectiveNormalVariance;
  singleInfiniteLineMeasurementCovariance(1, 1) = parameters.sensorUpdate_projectiveNormalVariance;
  singleInfiniteLineMeasurementCovariance(2, 2) = parameters.sensorUpdate_projectiveNormalVariance;

  if (stateUpdate == 0)
    stateUpdate = new PoseHypothesis2016::KalmanStateUpdate();
  stateUpdate->initMeasurementCovariance(singleMeasurementCovariance, parameters.sensorUpdate_correlationFactorBetweenMeasurements);

  if (stateUpdateInfiniteLine == 0)
    stateUpdateInfiniteLine = new PoseHypothesis2016::KalmanInfiniteLineStateUpdate();
  stateUpdateInfiniteLine->initMeasurementCovarianceInfiniteLine(singleInfiniteLineMeasurementCovariance, parameters.sensorUpdate_correlationFactorBetweenMeasurements);

  poseCovarianceForLineMatching = Matrix3d::Zero();
  poseCovarianceForLineMatching(0,0) = parameters.lineMatching_positionVariance;
  poseCovarianceForLineMatching(1,1) = parameters.lineMatching_positionVariance;
  poseCovarianceForLineMatching(2,2) = parameters.lineMatching_orientationVariance;
}

void PoseHypothesis2016::predict(const Pose2f & odometryDelta, const SelfLocator2016Parameters& parameters)
{
  Pose2f tempPose;
  getRobotPose(tempPose);
  tempPose += odometryDelta;
  state[0] = tempPose.translation.x();
  state[1] = tempPose.translation.y();
  state[2] = Angle::normalize(tempPose.rotation);

  // add robot process noise
  const float minPositionVariance = 100 * parameters.processUpdate_positionVariance;
  const float minRotationVariance = 100 * parameters.processUpdate_rotationVariance;
  const float& minPositionChange = parameters.processUpdate_minPositionChageForCovarianceUpdate;
  const float& minRotationChange = parameters.processUpdate_minRotationChageForCovarianceUpdate;
  if (!parameters.processUpdate_odometryBasedVarianceUpdate || std::abs(odometryDelta.translation.x()) < minPositionChange || covariance(0, 0) < minPositionVariance)
    covariance(0, 0) += parameters.processUpdate_positionVariance;
  if (!parameters.processUpdate_odometryBasedVarianceUpdate || std::abs(odometryDelta.translation.y()) < minPositionChange || covariance(1, 1) < minPositionVariance)
    covariance(1, 1) += parameters.processUpdate_positionVariance;
  if (!parameters.processUpdate_odometryBasedVarianceUpdate || std::abs(odometryDelta.rotation) < minRotationChange || covariance(2, 2) < minRotationVariance)
    covariance(2, 2) += parameters.processUpdate_rotationVariance;
}

void PoseHypothesis2016::fillCorrectionMatrices(
  const LineMatchingResult & theLineMatchingResult,
  const CLIPCenterCirclePercept &theCenterCirclePercept,
  const CLIPGoalPercept &theGoalPercept,
  const PenaltyCrossPercept &thePenaltyCrossPercept,
  const FieldDimensions & theFieldDimensions,
  const CameraMatrix & theCameraMatrix,
  const CameraMatrixUpper & theCameraMatrixUpper,
  const SelfLocator2016Parameters &parameters)
{
  Pose2f  robotPose;
  getRobotPose(robotPose);

  // Update with feature percepts (angles)
  std::vector<AngleObservation>& observationsAsAngles = observationAnglesWithLocalFeaturePerceptionsSpherical;
  std::vector< ExpectedMeasurementLandmark<totalDimension> >& correspondencesInGlobalCoordsAngles = correspondencesInGlobalCoordsWithLocalFeaturePerceptionsSpherical;

  observationsAsAngles.clear();
  correspondencesInGlobalCoordsAngles.clear();

  // update with center circle
  if (theCenterCirclePercept.centerCircleWasSeen
    && theCenterCirclePercept.centerCircle.locationOnField.x() > 50)
    // the center might also be under or even behind the robot;
    // don't use this, too much trouble with periodicities and singularities etc.
  {
    const float &cameraHeight = theCenterCirclePercept.fromUpper ? theCameraMatrixUpper.translation.z() : theCameraMatrix.translation.z();
    ExpectedMeasurementLandmark<totalDimension> measurement = calculateMeasurementModelForAngularLandmarkObservation(Vector2d(0, 0), cameraHeight, parameters);
    Vector2d relativePosition(theCenterCirclePercept.centerCircle.locationOnField.cast<double>());
    AngleObservation realMeasurement = getSphericalRepresentationForCartesianObservation(relativePosition, cameraHeight);
    if (calculateMeasurementLikelihoodSpherical(measurement.measurement, realMeasurement.observation, parameters) > 0)
    {
      //Vector2f expectedMeasurementCartesian = Transformation::fieldToRobot(robotPose, Vector2f(0, 0));
      observationsAsAngles.push_back(realMeasurement);
      correspondencesInGlobalCoordsAngles.push_back(measurement);

      Vector2f obsInFieldCoords = robotPose * Vector2f(relativePosition.cast<float>());
      LINE("module:SelfLocator2016:correspondences",
        robotPose.translation.x(), robotPose.translation.y(), obsInFieldCoords.x(), obsInFieldCoords.y(), 30, Drawings::dashedPen, ColorRGBA(200, 200, 200));
      ARROW("module:SelfLocator2016:correspondences",
        obsInFieldCoords.x(), obsInFieldCoords.y(), 0, 0, 30, Drawings::solidPen, ColorRGBA(200, 200, 200));
    }
  }

  // update with penalty cross
  if (thePenaltyCrossPercept.penaltyCrossWasSeen
    && thePenaltyCrossPercept.pointOnField.x() > 50)
    // don't update with too close percept
    // too much trouble with periodicities and singularities etc.
  {
    const float &cameraHeight = thePenaltyCrossPercept.fromUpper ? theCameraMatrixUpper.translation.z() : theCameraMatrix.translation.z();
    Vector2d penaltyMarkOpp(theFieldDimensions.xPosOpponentPenaltyMark, 0);
    Vector2d penaltyMarkOwn(theFieldDimensions.xPosOwnPenaltyMark, 0);
    ExpectedMeasurementLandmark<totalDimension> measurementOpp = calculateMeasurementModelForAngularLandmarkObservation(penaltyMarkOpp, cameraHeight, parameters);
    ExpectedMeasurementLandmark<totalDimension> measurementOwn = calculateMeasurementModelForAngularLandmarkObservation(penaltyMarkOwn, cameraHeight, parameters);
    const Vector2d &expectedMeasurementOpp = measurementOpp.measurement;
    const Vector2d &expectedMeasurementOwn = measurementOwn.measurement;
    Vector2d relativePosition(thePenaltyCrossPercept.pointOnField.x(), thePenaltyCrossPercept.pointOnField.y());
    AngleObservation realMeasurement = getSphericalRepresentationForCartesianObservation(relativePosition, cameraHeight);
    Vector2f obsInFieldCoords = robotPose * Vector2f(relativePosition.cast<float>());
    bool useOwnMark =
      (obsInFieldCoords - Vector2f(penaltyMarkOwn.cast<float>())).norm()
      < (obsInFieldCoords - Vector2f(penaltyMarkOpp.cast<float>())).norm();
    // unused variable
    //double likelihoodPenaltyCrosspercept = 1.0;
    if (!useOwnMark && calculateMeasurementLikelihoodSpherical(expectedMeasurementOpp, realMeasurement.observation, parameters) > 0)
    {
      //Vector2f expectedMeasurementCartesian = Transformation::fieldToRobot(robotPose, Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0));
      observationsAsAngles.push_back(realMeasurement);
      correspondencesInGlobalCoordsAngles.push_back(measurementOpp);

      LINE("module:SelfLocator2016:correspondences",
        robotPose.translation.x(), robotPose.translation.y(), obsInFieldCoords.x(), obsInFieldCoords.y(), 30, Drawings::dashedPen, ColorRGBA(200, 200, 200));
      ARROW("module:SelfLocator2016:correspondences",
        obsInFieldCoords.x(), obsInFieldCoords.y(), penaltyMarkOpp.x(), penaltyMarkOpp.y(), 30, Drawings::solidPen, ColorRGBA(200, 200, 200));
    }
    else if (useOwnMark && calculateMeasurementLikelihoodSpherical(expectedMeasurementOwn, realMeasurement.observation, parameters) > 0)
    {
      //Vector2f expectedMeasurementCartesian = Transformation::fieldToRobot(robotPose, Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0));
      observationsAsAngles.push_back(realMeasurement);
      correspondencesInGlobalCoordsAngles.push_back(measurementOwn);

      LINE("module:SelfLocator2016:correspondences",
        robotPose.translation.x(), robotPose.translation.y(), obsInFieldCoords.x(), obsInFieldCoords.y(), 30, Drawings::dashedPen, ColorRGBA(200, 200, 200));
      ARROW("module:SelfLocator2016:correspondences",
        obsInFieldCoords.x(), obsInFieldCoords.y(), penaltyMarkOwn.x(), penaltyMarkOwn.y(), 30, Drawings::solidPen, ColorRGBA(200, 200, 200));
    }
  }

  // update with goal posts
  if (theGoalPercept.numberOfGoalPosts > 0
    && theGoalPercept.numberOfGoalPosts < 3) // one of them is a false positive anyway and using any of them is risky
  {
    for (auto goalPost : theGoalPercept.goalPosts)
    {
      AngleObservation realMeasurement;
      ExpectedMeasurementLandmark<totalDimension> bestCorrespondenceInGlobalCoords;
      if (findGoalPostMatch(goalPost, theCameraMatrix, theCameraMatrixUpper, realMeasurement, bestCorrespondenceInGlobalCoords, theFieldDimensions, parameters))
      {
        observationsAsAngles.push_back(realMeasurement);
        correspondencesInGlobalCoordsAngles.push_back(bestCorrespondenceInGlobalCoords);

        Vector2f obsInFieldCoords = robotPose * Vector2f(goalPost.locationOnField.cast<float>());
        LINE("module:SelfLocator2016:correspondences",
          robotPose.translation.x(), robotPose.translation.y(), obsInFieldCoords.x(), obsInFieldCoords.y(), 30, Drawings::dashedPen, ColorRGBA(200, 200, 0));
        ARROW("module:SelfLocator2016:correspondences",
          obsInFieldCoords.x(), obsInFieldCoords.y(),
          bestCorrespondenceInGlobalCoords.measurement.x(), bestCorrespondenceInGlobalCoords.measurement.y(),
          30, Drawings::solidPen, ColorRGBA(200, 200, 0));
      }
    }
  }

  // Fieldlines
  std::vector<LineMatchingResult::FieldLine> correspondencesForObservations;
  correspondencesForObservations.clear();
  double likelihoodThreshold = 0.1;

  // Update with infinite lines (normals)
  std::vector<NormalObservation>& observationsAsNormals = observationsAsNormalsWithLocalFeaturePerceptionsInfiniteLines;
  std::vector<Vector2d>& correspondencesInGlobalCoordsNormals = correspondencesInGlobalCoordsWithLocalFeaturePerceptionsInfiniteLines;

  observationsAsNormals.clear();
  correspondencesInGlobalCoordsNormals.clear();

  bool foundCorrespondenceInLineMatch = false;
  if (theLineMatchingResult.containsNonUniqueMatches())
  {
    foundCorrespondenceInLineMatch = theLineMatchingResult.getCorrespondencesForLocalizationHypothesis(robotPose, poseCovarianceForLineMatching, likelihoodThreshold, singleMeasurementCovariance_2x2, correspondencesForObservations);
  }

  // filter "critical" correspondences in combination with onlyObservedOneFieldLine:
  // * looking out of the field and only seeing something corresponding to the own penalty line
  //   -> this is likely the goal line, as that one would also be seen, otherwise
  // * a single segment of the center circle generated a line observation
  if (foundCorrespondenceInLineMatch && theLineMatchingResult.onlyObservedOneFieldLine)
  {
    // penalty area situation
    bool isPenaltyAreaSituation = false;
    const LineMatchingResult::FieldLine & firstObservation = theLineMatchingResult.observations[0];
    const LineMatchingResult::FieldLine & firstCorrespondence = correspondencesForObservations[0];
    double observationDirection = robotPose.rotation + (firstObservation.start + firstObservation.end).angle();// angle of pose to middle of observed line in abs coords
    if (fabs(observationDirection) < pi_2)
    {
      // looking to opponent goal direction
      isPenaltyAreaSituation = (firstCorrespondence.end.x() == theFieldDimensions.xPosOpponentPenaltyArea
        &&firstCorrespondence.start.x() == theFieldDimensions.xPosOpponentPenaltyArea);
    }
    else
    {
      // looking to own goal direction
      isPenaltyAreaSituation = (firstCorrespondence.end.x() == theFieldDimensions.xPosOwnPenaltyArea
        &&firstCorrespondence.start.x() == theFieldDimensions.xPosOwnPenaltyArea);
    }

    // center circle situation
    bool lineFragmentNearCenterCircle = false;
    if (!theCenterCirclePercept.centerCircleWasSeen)
    {
      for (unsigned int i = 0; i < theLineMatchingResult.observations.size(); i++)
      {
        const LineMatchingResult::FieldLine & correspondingFieldLineSegment = correspondencesForObservations[i];
        if (correspondingFieldLineSegment.start.norm() < theFieldDimensions.centerCircleRadius * 2.0
          && correspondingFieldLineSegment.end.norm() < theFieldDimensions.centerCircleRadius * 2.0)
        {
          lineFragmentNearCenterCircle = true;
        }
      }
    }

    if (lineFragmentNearCenterCircle || isPenaltyAreaSituation)
    {
      foundCorrespondenceInLineMatch = false; // do not accept the match and do not use it to update the pose
    }
  }

  if (foundCorrespondenceInLineMatch)
  {
    if (!debugLineMatching)
    {
      theLineMatchingResult.drawRequestedCorrespondences(robotPose, poseCovarianceForLineMatching, likelihoodThreshold, singleMeasurementCovariance_2x2);
    }
    for (unsigned int i = 0; i < theLineMatchingResult.observations.size(); i++)
    {
      const LineMatchingResult::FieldLine & observedLine = theLineMatchingResult.observations[i];
      const LineMatchingResult::FieldLine & correspondingFieldLineSegment = correspondencesForObservations[i];

      Vector3d observedProjectiveGeometryNormal = Vector3d(observedLine.start.x(),
        observedLine.start.y(),
        -observedLine.cameraHeight).cross(
        Vector3d(observedLine.end.x(),
        observedLine.end.y(),
        -observedLine.cameraHeight));
      observedProjectiveGeometryNormal.normalize();

      observationsAsNormals.push_back(NormalObservation(observedProjectiveGeometryNormal, static_cast<float>(observedLine.cameraHeight)));
      correspondencesInGlobalCoordsNormals.push_back(correspondingFieldLineSegment.start);
      correspondencesInGlobalCoordsNormals.push_back(correspondingFieldLineSegment.end);
    }
  }
}

void PoseHypothesis2016::updateStateRotationWithLocalFieldLines(
  const CLIPFieldLinesPercept &theFieldLinesPercept,
  const SelfLocator2016Parameters &parameters)
{
  // Adjust hypotheses to best fitting angle
  double bestOrientationFromPercepts = 0.0;
  if (findBestAngle(theFieldLinesPercept, bestOrientationFromPercepts))
  {
    double tempRotation = bestOrientationFromPercepts;
    while (std::abs(Angle::normalize(state[2] - tempRotation)) > pi_4)
      tempRotation = Angle::normalize(tempRotation + pi_2);
    //if (theSensorCalibration.gyroZGain == 0) // NaoV4 should use this fully
    //  newPositionConfidence = 1;
    const float &weight = parameters.sensorUpdate_adjustRotationToBestFittingAngle;
    double diff = Angle::normalize(tempRotation - state[2]);
    state[2] = Angle::normalize(state[2] + weight*diff);
  }
}

bool PoseHypothesis2016::updatePositionConfidenceWithLocalFeaturePerceptionsWeighted(
  const CLIPFieldLinesPercept &theFieldLinesPercept,
  const CLIPCenterCirclePercept &theCenterCirclePercept,
  const CLIPGoalPercept &theGoalPercept,
  const PenaltyCrossPercept &thePenaltyCrossPercept,
  const FieldDimensions & theFieldDimensions,
  const SelfLocator2016Parameters & parameters)
{
  bool update = false;
  Pose2f  robotPose;
  getRobotPose(robotPose);
  static int lh[5] = { 0, 2, 4, 6, 9 };
  static int lv[6] = { 1, 3, 5, 7, 8, 10 };
  static std::vector<int> linesHorizontal(&lh[0], &lh[0] + 5);
  static std::vector<int> linesVertical(&lv[0], &lv[0] + 6);
  // LEVEL 1
  if (parameters.sensorUpdate_use1stLevelUpdate)
    update |= updatePositionConfidenceWithSingleLines(theFieldLinesPercept, theFieldDimensions, parameters,
    robotPose, linesHorizontal, linesVertical);
  // LEVEL 2
  if (parameters.sensorUpdate_use2ndLevelUpdate)
    update |= updatePositionConfidenceWithLineCrossings(theFieldLinesPercept, theFieldDimensions, parameters,
    robotPose, linesHorizontal, linesVertical);
  // LEVEL 3
  if (parameters.sensorUpdate_use3rdLevelUpdate)
    update |= updatePositionConfidenceWithLineAndLandmark(
    theFieldLinesPercept, theCenterCirclePercept, theGoalPercept,
    thePenaltyCrossPercept, theFieldDimensions, parameters,
    robotPose, linesHorizontal, linesVertical);

  return update;
}

// LEVEL 1
bool PoseHypothesis2016::updatePositionConfidenceWithSingleLines(
  const CLIPFieldLinesPercept & theFieldLinesPercept,
  const FieldDimensions & theFieldDimensions,
  const SelfLocator2016Parameters & parameters,
  const Pose2f &robotPose,
  const std::vector<int> &linesHorizontal,
  const std::vector<int> &linesVertical)
{
  float correspondence = 0.f;

  std::vector<CLIPFieldLinesPercept::FieldLine>::const_iterator percept = theFieldLinesPercept.lines.begin();
  std::vector<CLIPFieldLinesPercept::FieldLine>::const_iterator perceptEnd = theFieldLinesPercept.lines.end();
  bool foundMatch = false;
  bool update = false;
  for (; percept != perceptEnd; ++percept)
  {
    foundMatch = false;
    correspondence = 0.0;
    Vector2f perceptFieldStart = Transformation::robotToField(robotPose, (*percept).startOnField);
    Vector2f perceptFieldEnd = Transformation::robotToField(robotPose, (*percept).endOnField);
    //double perceptDistanceRelative = (((*percept).startOnField + (*percept).endOnField)*0.5).abs();
    float angle = (perceptFieldEnd - perceptFieldStart).angle();
    float length = (perceptFieldEnd - perceptFieldStart).norm();
    float horizontalAngle = std::min<float>(std::abs(Angle::normalize(angle + pi_2)),
      std::abs(Angle::normalize(angle - pi_2)));
    float verticalAngle = std::min<float>(std::abs(angle),
      std::abs(Angle::normalize(angle + pi)));
    bool isHorizontal = horizontalAngle < parameters.sensorUpdate_worstAngleDifference; // TODO
    bool isVertical = verticalAngle < parameters.sensorUpdate_worstAngleDifference; // TODO
    int minDistanceID = -1;

    if (percept->isPlausible
      && length > parameters.sensorUpdate_minLineLengthOnField)
    {
      float minDistance = 100000.f;
      Vector2f perceptCenterField = ((perceptFieldEnd + perceptFieldStart)*0.5f);

      if (isHorizontal)
      {
        for (std::vector<int>::const_iterator line = linesHorizontal.begin(); line != linesHorizontal.end(); ++line)
        {
          const FieldDimensions::LinesTable::Line &fieldLine = theFieldDimensions.fieldLines.lines[(*line)];
          float allowedOffset = fieldLine.length / 10;
          float minY = std::min(fieldLine.from.y(), fieldLine.to.y()) - allowedOffset;
          float maxY = std::max(fieldLine.from.y(), fieldLine.to.y()) + allowedOffset;
          float perceptDistance = std::abs(fieldLine.from.x() - perceptCenterField.x());
          if (perceptDistance < minDistance
            && perceptFieldEnd.y() > minY
            && perceptFieldEnd.y() < maxY
            && perceptFieldStart.y() > minY
            && perceptFieldStart.y() < maxY)
          {
            minDistance = perceptDistance;
            minDistanceID = *line;
          }
        }
      }
      else if (isVertical)
      {
        for (std::vector<int>::const_iterator line = linesVertical.begin(); line != linesVertical.end(); ++line)
        {
          const FieldDimensions::LinesTable::Line &fieldLine = theFieldDimensions.fieldLines.lines[(*line)];
          float allowedOffset = fieldLine.length / 10;
          float minX = std::min(fieldLine.from.x(), fieldLine.to.x()) - allowedOffset;
          float maxX = std::max(fieldLine.from.x(), fieldLine.to.x()) + allowedOffset;
          float perceptDistance = std::abs(fieldLine.from.y() - perceptCenterField.y());
          if (perceptDistance < minDistance
            && perceptFieldEnd.x() > minX
            && perceptFieldEnd.x() < maxX
            && perceptFieldStart.x() > minX
            && perceptFieldStart.x() < maxX)
          {
            minDistance = perceptDistance;
            minDistanceID = *line;
          }
        }
      }
      if (minDistanceID >= 0 &&
        length / theFieldDimensions.fieldLines.lines[minDistanceID].length < 1.2)
      {
        // TODO: use field dimensions methods?
        float distanceToFieldLine = isHorizontal ? std::abs(theFieldDimensions.fieldLines.lines[minDistanceID].from.x() - robotPose.translation.x())
          : std::abs(theFieldDimensions.fieldLines.lines[minDistanceID].from.y() - robotPose.translation.y());
        angle = isHorizontal ? horizontalAngle : verticalAngle;
        correspondence = (1.f - std::min<float>(1.f, (minDistance / (distanceToFieldLine + 1)) / parameters.sensorUpdate_maxDistanceError))
          * (1.f - angle / parameters.sensorUpdate_worstAngleDifference);
        foundMatch = (correspondence > 0.01f);
      }

      if (foundMatch)
        updatePositionConfidence(true, parameters.sensorUpdate_influenceOfNewLineMeasurementOnPositionConfidence, percept->validity*correspondence);
      else
        updatePositionConfidence(false, parameters.sensorUpdate_influenceOfNewLineMeasurementOnPositionConfidence, percept->validity);

      update |= foundMatch;
    }
  }
  return update;
}

// LEVEL 2
bool PoseHypothesis2016::updatePositionConfidenceWithLineCrossings(
  const CLIPFieldLinesPercept & theFieldLinesPercept,
  const FieldDimensions & theFieldDimensions,
  const SelfLocator2016Parameters & parameters,
  const Pose2f &robotPose,
  const std::vector<int> &linesHorizontal,
  const std::vector<int> &linesVertical)
{
  return false;
}

// LEVEL 3
bool PoseHypothesis2016::updatePositionConfidenceWithLineAndLandmark(
  const CLIPFieldLinesPercept & theFieldLinesPercept,
  const CLIPCenterCirclePercept &theCenterCirclePercept,
  const CLIPGoalPercept &theGoalPercept,
  const PenaltyCrossPercept &thePenaltyCrossPercept,
  const FieldDimensions & theFieldDimensions,
  const SelfLocator2016Parameters & parameters,
  const Pose2f &robotPose,
  const std::vector<int> &linesHorizontal,
  const std::vector<int> &linesVertical)
{
  Pose2f pose;
  float weight = 0.f;
  bool foundMatch = false;
  bool update = false;

  // Update from Center Circle and Center Line
  float perceptWeight = 0.f;
  if (PoseGenerator::getPoseFromCenterCircleAndCenterLine(theFieldLinesPercept, theCenterCirclePercept, pose, perceptWeight) > 0)
  {
    Pose2f symmetricPose(Angle::normalize(pose.rotation + pi), pose.translation * -1);
    //Pose2f &closestPose = robotPose.translation.x() < 0 ? pose : symmetricPose;
    const Pose2f &closestPose = getClosestPose(robotPose, pose, symmetricPose);
    weight = perceptWeight*calcWeightForPoseDifference(closestPose - robotPose, parameters);
    update |= (foundMatch = weight > 0.f);
    updatePositionConfidence(foundMatch, parameters.sensorUpdate_influenceOfNewCenterCircleMeasurementOnPositionConfidence, weight);
    DRAW_ROBOT_POSE("module:SelfLocator2016:poseFromCenterCircle", closestPose, ColorRGBA(255 - (unsigned char)(perceptWeight * 255.f), (unsigned char)(perceptWeight * 255.f), 0));
  }

  // Update from Penalty Cross and Line
  foundMatch = false;
  if (PoseGenerator::getPoseFromPenaltyCrossAndLine(theFieldDimensions, theFieldLinesPercept, thePenaltyCrossPercept, pose, perceptWeight))
  {
    Pose2f symmetricPose(Angle::normalize(pose.rotation + pi), pose.translation * -1);
    //Pose2f &closestPose = robotPose.translation.x() > 0 ? pose : symmetricPose;
    const Pose2f &closestPose = getClosestPose(robotPose, pose, symmetricPose);
    weight = perceptWeight*calcWeightForPoseDifference(closestPose - robotPose, parameters);
    update |= (foundMatch = weight > 0.f);
    updatePositionConfidence(foundMatch, parameters.sensorUpdate_influenceOfNewPenaltyCrossMeasurementOnPositionConfidence, weight);
    DRAW_ROBOT_POSE("module:SelfLocator2016:poseFromPenaltyCross", closestPose, ColorRGBA(0, 0, 255));
  }
  return update;
}

bool PoseHypothesis2016::updatePositionConfidenceWithLocalFeaturePerceptionsSpherical(
  const LineMatchingResult &theLineMatchingResult,
  const CLIPCenterCirclePercept &theCenterCirclePercept,
  const CLIPGoalPercept &theGoalPercept,
  const PenaltyCrossPercept &thePenaltyCrossPercept,
  const FieldDimensions & theFieldDimensions,
  const CameraMatrix & theCameraMatrix,
  const CameraMatrixUpper & theCameraMatrixUpper,
  const SelfLocator2016Parameters & parameters)
{
  Pose2f  robotPose;
  getRobotPose(robotPose);

  std::vector<LineMatchingResult::FieldLine> correspondencesForObservations;
  correspondencesForObservations.clear();
  double likelihoodThreshold = 0.1;
  bool update = false;


  Pose2f requestCorrespondencesForThisPose;
  
  MODIFY("module:SelfLocator2016:requestLineCorrespondencesForThisPose", requestCorrespondencesForThisPose);
  MODIFY("module:SelfLocator2016:lineCorrespondenceLikelihoodThreshold", likelihoodThreshold);
  
  debugLineMatching = requestCorrespondencesForThisPose.translation.x() != 0
    || requestCorrespondencesForThisPose.translation.y() != 0
    || requestCorrespondencesForThisPose.rotation != 0;
  if (debugLineMatching)
  {
    theLineMatchingResult.drawRequestedCorrespondences(requestCorrespondencesForThisPose, poseCovarianceForLineMatching, likelihoodThreshold, singleMeasurementCovariance_2x2);
  }


  // update with field lines
  if (theLineMatchingResult.containsUniqueMatches())
  {
    bool foundCorrespondenceInLineMatch = theLineMatchingResult.getCorrespondencesForLocalizationHypothesis(robotPose, poseCovarianceForLineMatching, likelihoodThreshold, singleMeasurementCovariance_2x2, correspondencesForObservations);
    updatePositionConfidence(foundCorrespondenceInLineMatch, parameters.sensorUpdate_influenceOfNewLineMeasurementOnPositionConfidence, 1.f);
    update |= foundCorrespondenceInLineMatch;
  }

  // update with center circle
  if (theCenterCirclePercept.centerCircleWasSeen
    && theCenterCirclePercept.centerCircle.locationOnField.x() > 50)
    // the center might also be under or even behind the robot;
    // don't use this, too much trouble with periodicities and singularities etc.
  {
    const float &cameraHeight = theCenterCirclePercept.fromUpper ? theCameraMatrixUpper.translation.z() : theCameraMatrix.translation.z();
    ExpectedMeasurementLandmark<totalDimension> measurement = calculateMeasurementModelForAngularLandmarkObservation(Vector2d(0, 0), cameraHeight, parameters);
    Vector2d expectedMeasurement = Vector2d(measurement.measurement.x(), measurement.measurement.y());
    Vector2d relativePosition(theCenterCirclePercept.centerCircle.locationOnField.cast<double>());
    AngleObservation realMeasurement = getSphericalRepresentationForCartesianObservation(relativePosition, cameraHeight);
    if (calculateMeasurementLikelihoodSpherical(expectedMeasurement, realMeasurement.observation, parameters) > 0)
    {
      Vector2d expectedMeasurementCartesian = Transformation::fieldToRobot(robotPose, Vector2f::Zero()).cast<double>();
      float distanceErrorFactor = 1.f - std::min(1.f, static_cast<float>((expectedMeasurementCartesian - relativePosition).norm()) / 2000);
      updatePositionConfidence(true, parameters.sensorUpdate_influenceOfNewCenterCircleMeasurementOnPositionConfidence, distanceErrorFactor);
    }
    else
    {
      updatePositionConfidence(false, parameters.sensorUpdate_influenceOfNewCenterCircleMeasurementOnPositionConfidence, 1.f);
    }
    update = true;
  }

  // update with penalty cross
  if (thePenaltyCrossPercept.penaltyCrossWasSeen
    && thePenaltyCrossPercept.pointOnField.x() > 50)
    // dont update with too close percept
    // too much trouble with periodicities and singularities etc.
  {
    const float &cameraHeight = thePenaltyCrossPercept.fromUpper ? theCameraMatrixUpper.translation.z() : theCameraMatrix.translation.z();
    Vector2d penaltyMarkOpp(theFieldDimensions.xPosOpponentPenaltyMark, 0);
    Vector2d penaltyMarkOwn(theFieldDimensions.xPosOwnPenaltyMark, 0);
    ExpectedMeasurementLandmark<totalDimension> measurementOpp = calculateMeasurementModelForAngularLandmarkObservation(penaltyMarkOpp, cameraHeight, parameters);
    ExpectedMeasurementLandmark<totalDimension> measurementOwn = calculateMeasurementModelForAngularLandmarkObservation(penaltyMarkOwn, cameraHeight, parameters);
    Vector2d expectedMeasurementOpp = measurementOpp.measurement;
    Vector2d expectedMeasurementOwn = measurementOwn.measurement;
    Vector2d relativePosition = thePenaltyCrossPercept.pointOnField.cast<double>();
    AngleObservation realMeasurement = getSphericalRepresentationForCartesianObservation(relativePosition, cameraHeight);
    Vector2d obsInFieldCoords = Transformation::robotToField(robotPose, relativePosition.cast<float>()).cast<double>();
    bool useOwnMark =
      (obsInFieldCoords - penaltyMarkOwn).norm() 
      < (obsInFieldCoords - penaltyMarkOpp).norm();
    // unused variable
    //double likelihoodPenaltyCrosspercept = 1.0;
    if (!useOwnMark && calculateMeasurementLikelihoodSpherical(expectedMeasurementOpp, realMeasurement.observation, parameters) > 0)
    {
      Vector2d expectedMeasurementCartesian = Transformation::fieldToRobot(robotPose, Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0)).cast<double>();
      float distanceErrorFactor = parameters.sensorUpdate_maxInfluenceOnPositionConfidencePenaltyCrossOnly -
        std::min(parameters.sensorUpdate_maxInfluenceOnPositionConfidencePenaltyCrossOnly,
        static_cast<float>((expectedMeasurementCartesian - relativePosition).norm() / 2000));
      updatePositionConfidence(true, parameters.sensorUpdate_influenceOfNewPenaltyCrossMeasurementOnPositionConfidence, distanceErrorFactor);
    }
    else if (useOwnMark && calculateMeasurementLikelihoodSpherical(expectedMeasurementOwn, realMeasurement.observation, parameters) > 0)
    {
      Vector2d expectedMeasurementCartesian = Transformation::fieldToRobot(robotPose, Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0)).cast<double>();
      float distanceErrorFactor = parameters.sensorUpdate_maxInfluenceOnPositionConfidencePenaltyCrossOnly -
        std::min(parameters.sensorUpdate_maxInfluenceOnPositionConfidencePenaltyCrossOnly,
        static_cast<float>((expectedMeasurementCartesian - relativePosition).norm() / 2000));
      updatePositionConfidence(true, parameters.sensorUpdate_influenceOfNewPenaltyCrossMeasurementOnPositionConfidence, distanceErrorFactor);
    }
    else
    {
      updatePositionConfidence(false, parameters.sensorUpdate_influenceOfNewPenaltyCrossMeasurementOnPositionConfidence, 1.f);
    }
    update = true;
  }

  // update with goal posts
  if (theGoalPercept.numberOfGoalPosts > 0
    && theGoalPercept.numberOfGoalPosts < 3) // one of them is a false positive anyway and using any of them is risky
  {
    for (int i = 0; i < theGoalPercept.numberOfGoalPosts; i++)
    {
      AngleObservation realMeasurement;
      ExpectedMeasurementLandmark<totalDimension> bestCorrespondenceInGlobalCoords;
      if (findGoalPostMatch(theGoalPercept.goalPosts[i], theCameraMatrix, theCameraMatrixUpper, realMeasurement, bestCorrespondenceInGlobalCoords, theFieldDimensions, parameters))
      {
        updatePositionConfidence(true, parameters.sensorUpdate_influenceOfNewGoalMeasurementOnPositionConfidence, 1.f);
      }
      else
      {
        updatePositionConfidence(false, parameters.sensorUpdate_influenceOfNewGoalMeasurementOnPositionConfidence, 1.f);
      }
      update = true;
    }
  }
  return update;
}
void PoseHypothesis2016::updateStateWithLocalFeaturePerceptionsSpherical(const SelfLocator2016Parameters & parameters)
{
  //will be filled in fillCorrectionMatrices
  std::vector<AngleObservation>& observationsAsAngles = observationAnglesWithLocalFeaturePerceptionsSpherical;
  std::vector< ExpectedMeasurementLandmark<totalDimension> >& correspondencesInGlobalCoords = correspondencesInGlobalCoordsWithLocalFeaturePerceptionsSpherical;

  if (observationsAsAngles.size() == 0) return;
  if (correspondencesInGlobalCoords.size() == 0) return;

  int dimensionOfMeasurementVector = static_cast<int>(observationsAsAngles.size() * 2); // each observation has 2 angles

  // ok, since we only have limited sized matrizes for the update, we restrict the measurement dimension...
  const int maxMeasurementDimension = 2 * MAX_OBSERVATIONS;
  if (parameters.displayWarnings && dimensionOfMeasurementVector > maxMeasurementDimension)
  {
    OUTPUT_TEXT("PoseHypothesis2016::updateWithLocalFeaturePerceptions(...): Consider increasing maxMeasurementDimension!");
  }
  while (dimensionOfMeasurementVector > maxMeasurementDimension)
  {
    observationsAsAngles.pop_back();
    correspondencesInGlobalCoords.pop_back();
    dimensionOfMeasurementVector = static_cast<int>(observationsAsAngles.size() * 2);
  }

  if (stateUpdate == 0)
    stateUpdate = new PoseHypothesis2016::KalmanStateUpdate();

  Eigen::Matrix<double, totalDimension, 1> correction = stateUpdate->updateWithLocalObservations(observationsAsAngles, correspondencesInGlobalCoords, covariance);
  if (correction.isMuchSmallerThan(0.1))
    OUTPUT_TEXT("Minimal state correction neglected");
  else
    state += correction;

  observationsAsAngles.clear(); //clear so we (k)now whats up
  correspondencesInGlobalCoords.clear();
}

bool PoseHypothesis2016::updatePositionConfidenceWithLocalFeaturePerceptionsInfiniteLines(
  const LineMatchingResult & theLineMatchingResult,
  const CLIPCenterCirclePercept &theCenterCirclePercept,
  const FieldDimensions & theFieldDimensions,
  const SelfLocator2016Parameters & parameters)
{
  Pose2f  robotPose;
  getRobotPose(robotPose);

  double likelihoodThreshold = 0.1;
  bool update = false;

  // update with field lines
  bool foundCorrespondenceInLineMatch = false;
  if (theLineMatchingResult.containsNonUniqueMatches())
  {
    std::vector<LineMatchingResult::FieldLine> correspondencesForObservations;
    foundCorrespondenceInLineMatch = theLineMatchingResult.getCorrespondencesForLocalizationHypothesis(robotPose, poseCovarianceForLineMatching, likelihoodThreshold, singleMeasurementCovariance_2x2, correspondencesForObservations);

    updatePositionConfidence(foundCorrespondenceInLineMatch, parameters.sensorUpdate_influenceOfNewInfiniteLineMeasurementOnPositionConfidence, 1.f);
    update = true;
  }
  return update;
}
void PoseHypothesis2016::updateStateWithLocalFeaturePerceptionsInfiniteLines(const SelfLocator2016Parameters & parameters)
{
  std::vector<NormalObservation>& observationsAsNormals = observationsAsNormalsWithLocalFeaturePerceptionsInfiniteLines;
  std::vector<Vector2d>&
    correspondencesInGlobalCoords = correspondencesInGlobalCoordsWithLocalFeaturePerceptionsInfiniteLines;

  if (observationsAsNormals.size() == 0) return;
  if (correspondencesInGlobalCoords.size() == 0) return;

  int dimensionOfMeasurementVector = static_cast<int>(observationsAsNormals.size() * 3); // each observation has 3 normals

  // ok, since we only have limited sized matrizes for the update, we restrict the measurement dimension...
  const int maxMeasurementDimension = 3 * MAX_OBSERVATIONS_INFINITELINE;
  if (parameters.displayWarnings && dimensionOfMeasurementVector > maxMeasurementDimension)
  {
    OUTPUT_TEXT("PoseHypothesis2016::updateStateWithLocalFeaturePerceptionsInfiniteLines(...): Consider increasing maxMeasurementDimension!");
  }
  while (dimensionOfMeasurementVector > maxMeasurementDimension)
  {
    observationsAsNormals.pop_back();
    correspondencesInGlobalCoords.pop_back();
    correspondencesInGlobalCoords.pop_back();
    dimensionOfMeasurementVector = static_cast<int>(observationsAsNormals.size() * 3);
  }

  std::vector<ExpectedMeasurementInfiniteLine<totalDimension> > measurements;
  for (size_t landmarkIndex = 0; landmarkIndex < observationsAsNormals.size(); landmarkIndex++)
    measurements.push_back(calculateMeasurementModelForInfiniteLineObservation(correspondencesInGlobalCoords[landmarkIndex * 2],
    correspondencesInGlobalCoords[landmarkIndex * 2 + 1], observationsAsNormals[landmarkIndex].cameraHeight));

  if (stateUpdateInfiniteLine == 0)
    stateUpdateInfiniteLine = new PoseHypothesis2016::KalmanInfiniteLineStateUpdate();

  Eigen::Matrix<double, totalDimension, 1> correction = stateUpdateInfiniteLine->updateWithLocalInfiniteLineObservations(observationsAsNormals, measurements, covariance);
  state += correction;
}

void PoseHypothesis2016::updateWithLocalFeaturePerceptionsCartesian(
  const CLIPCenterCirclePercept &theCenterCirclePercept,
  const PenaltyCrossPercept &thPenaltyCrossPercept,
  const SelfLocator2016Parameters & parameters)
{
  OUTPUT_ERROR("Not implemented yet");
}

bool PoseHypothesis2016::findGoalPostMatch(
  const CLIPGoalPercept::GoalPost & goalPost,
  const CameraMatrix & theCameraMatrix,
  const CameraMatrixUpper & theCameraMatrixUpper,
  AngleObservation & realMeasurement,
  ExpectedMeasurementLandmark<totalDimension> & bestCorrespondenceInGlobalCoords,
  const FieldDimensions & theFieldDimensions,
  const SelfLocator2016Parameters& parameters)
{
  const float& cameraHeight = goalPost.fromUpper ? theCameraMatrixUpper.translation.z() : theCameraMatrix.translation.z();
  realMeasurement = getSphericalRepresentationForCartesianObservation(Vector2d(goalPost.locationOnField.cast<double>()), cameraHeight);
  double bestLikelihood = 0;
  double likelihood;
  Vector2d correspondenceInGlobalCoords;
  ExpectedMeasurementLandmark<totalDimension> measurement;

  if (goalPost.goalPostSide != CLIPGoalPercept::GoalPost::rightPost) // unknown or left
  {
    // left own goal post ("left" as seen from the middle of the field)
    correspondenceInGlobalCoords.x() = theFieldDimensions.xPosOwnGroundline;
    correspondenceInGlobalCoords.y() = theFieldDimensions.yPosRightGoal;
    measurement = calculateMeasurementModelForAngularLandmarkObservation(correspondenceInGlobalCoords, cameraHeight, parameters);
    likelihood = calculateMeasurementLikelihoodSpherical(realMeasurement.observation, measurement.measurement, parameters);
    if (likelihood > bestLikelihood)
    {
      bestLikelihood = likelihood;
      bestCorrespondenceInGlobalCoords = measurement;
    }

    // left opponent goal post ("left" as seen from the middle of the field)
    correspondenceInGlobalCoords *= -1;
    measurement = calculateMeasurementModelForAngularLandmarkObservation(correspondenceInGlobalCoords, cameraHeight, parameters);
    likelihood = calculateMeasurementLikelihoodSpherical(realMeasurement.observation, measurement.measurement, parameters);
    if (likelihood > bestLikelihood)
    {
      bestLikelihood = likelihood;
      bestCorrespondenceInGlobalCoords = measurement;
    }
  }

  if (goalPost.goalPostSide != CLIPGoalPercept::GoalPost::leftPost) // unknown or right
  {
    // right own goal post ("right" as seen from the middle of the field)
    correspondenceInGlobalCoords.x() = theFieldDimensions.xPosOwnGroundline;
    correspondenceInGlobalCoords.y() = theFieldDimensions.yPosLeftGoal;
    measurement = calculateMeasurementModelForAngularLandmarkObservation(correspondenceInGlobalCoords, cameraHeight, parameters);
    likelihood = calculateMeasurementLikelihoodSpherical(realMeasurement.observation, measurement.measurement, parameters);
    if (likelihood > bestLikelihood)
    {
      bestLikelihood = likelihood;
      bestCorrespondenceInGlobalCoords = measurement;
    }

    // left opponent goal post ("left" as seen from the middle of the field)
    correspondenceInGlobalCoords *= -1;
    measurement = calculateMeasurementModelForAngularLandmarkObservation(correspondenceInGlobalCoords, cameraHeight, parameters);
    likelihood = calculateMeasurementLikelihoodSpherical(realMeasurement.observation, measurement.measurement, parameters);
    if (likelihood > bestLikelihood)
    {
      bestLikelihood = likelihood;
      bestCorrespondenceInGlobalCoords = measurement;
    }
  }

  return bestLikelihood > 0;
}

ExpectedMeasurementLandmark<PoseHypothesis2016::totalDimension> PoseHypothesis2016::calculateMeasurementModelForAngularLandmarkObservation(
  const Vector2d & correspondenceInGlobalCoords, const float &cameraHeight, const SelfLocator2016Parameters & parameters)
{
  if (angleObservationMeasurementModelJacobian_H == 0)
    angleObservationMeasurementModelJacobian_H = new Eigen::Matrix<double, 2, PoseHypothesis2016::totalDimension>();
  *angleObservationMeasurementModelJacobian_H = Eigen::Matrix<double, 2, PoseHypothesis2016::totalDimension>::Zero();

  ExpectedMeasurementLandmark<totalDimension> expectedAngularMeasurement;

  // some helpful variables
  const double p2l_x = correspondenceInGlobalCoords.x() - state[0]; // pose to landmark, x
  const double p2l_y = correspondenceInGlobalCoords.y() - state[1]; // pose to landmark, y
  const double d2 = p2l_x*p2l_x + p2l_y*p2l_y;
  const double d = sqrt(d2);
  const double cameraHeight2 = cameraHeight*cameraHeight;

  // calculate expectedAngularMeasurement
  expectedAngularMeasurement.measurement.x() = atan2(cameraHeight, d); // vertical angle
  expectedAngularMeasurement.measurement.y() = Angle::normalize(atan2(p2l_y, p2l_x) - state[2]); // horizontal angle

  // Weight of the observation (far away observations will might have more error due to camera matrix, etc...)
  double fullWeightAngle = parameters.sensorUpdate_maxVerticalAngleFullObservationWeight;
  expectedAngularMeasurement.weight = expectedAngularMeasurement.measurement.x() / (pi_2 - fullWeightAngle);
  expectedAngularMeasurement.weight = std::min(expectedAngularMeasurement.weight, 1.);

  // calculate angleObservationMeasurementModelJacobian_H
  (*angleObservationMeasurementModelJacobian_H)(0, 0) = cameraHeight*p2l_x / (d*(cameraHeight2 + d2));
  (*angleObservationMeasurementModelJacobian_H)(0, 1) = cameraHeight*p2l_y / (d*(cameraHeight2 + d2));
  (*angleObservationMeasurementModelJacobian_H)(0, 2) = 0;
  (*angleObservationMeasurementModelJacobian_H)(1, 0) = p2l_y / d2;
  (*angleObservationMeasurementModelJacobian_H)(1, 1) = -p2l_x / d2;
  (*angleObservationMeasurementModelJacobian_H)(1, 2) = -1;

  expectedAngularMeasurement.measurementModelJacobian = angleObservationMeasurementModelJacobian_H;

  return expectedAngularMeasurement;
}

ExpectedMeasurementInfiniteLine<PoseHypothesis2016::totalDimension> PoseHypothesis2016::calculateMeasurementModelForInfiniteLineObservation(const Vector2d & startPointCorrespondenceInGlobalCoords,
  const Vector2d & endPointCorrespondenceInGlobalCoords, const float &cameraHeight)
{
  if (infiniteLineMeasurementModelJacobian_H == 0)
    infiniteLineMeasurementModelJacobian_H = new Eigen::Matrix<double, 3, PoseHypothesis2016::totalDimension>();
  *infiniteLineMeasurementModelJacobian_H = Eigen::Matrix<double, 3, PoseHypothesis2016::totalDimension>::Zero();

  ExpectedMeasurementInfiniteLine<totalDimension> expectedMeasurement;

  // some helpful variables
  const double sin_p = sin(state[2]);
  const double cos_p = cos(state[2]);
  const double l_x_1 = startPointCorrespondenceInGlobalCoords.x();
  const double l_y_1 = startPointCorrespondenceInGlobalCoords.y();
  const double l_x_2 = endPointCorrespondenceInGlobalCoords.x();
  const double l_y_2 = endPointCorrespondenceInGlobalCoords.y();
  const double p_x = state[0];
  const double p_y = state[1];
  const double cameraHeight2 = cameraHeight*cameraHeight;
  const double t1 = l_x_1*l_y_2 - l_x_2*l_y_1 - l_x_1*p_y + l_y_1*p_x + l_x_2*p_y - l_y_2*p_x;
  const double t2 = cameraHeight2 * sqr(l_x_1 - l_x_2) + cameraHeight2 * sqr(l_y_1 - l_y_2) + t1*t1;
  const double t3 = 1 / sqrt(t2);
  const double t4 = t3*t3*t3;

  // calculate expectedMeasurement
  expectedMeasurement.measurement.x() = -cameraHeight * (cos_p*(l_y_1 - l_y_2) - sin_p*(l_x_1 - l_x_2));
  expectedMeasurement.measurement.y() = cameraHeight * (cos_p*(l_x_1 - l_x_2) + sin_p*(l_y_1 - l_y_2));
  expectedMeasurement.measurement.z() = t1;
  expectedMeasurement.measurement *= t3;

  // Normals will always add up to 1.0! (So nearly usesless; Maybe we find something else)
  expectedMeasurement.weight = std::min(expectedMeasurement.measurement.norm(), 1.);

  // calculate infiniteLineMeasurementModelJacobian_H

  // d_nx / d_px:
  (*infiniteLineMeasurementModelJacobian_H)(0, 0) = cameraHeight *(l_y_1 - l_y_2) *(cos_p*(l_y_1 - l_y_2) - sin_p*(l_x_1 - l_x_2)) *t1 *t4;
  // d_nx / d_py:
  (*infiniteLineMeasurementModelJacobian_H)(0, 1) = -cameraHeight *(l_x_1 - l_x_2) *(cos_p*(l_y_1 - l_y_2) - sin_p*(l_x_1 - l_x_2)) *t1 *t4;
  // d_nx / d_pteta:
  (*infiniteLineMeasurementModelJacobian_H)(0, 2) = cameraHeight * (cos_p*(l_x_1 - l_x_2) + sin_p*(l_y_1 - l_y_2)) * t3;
  // d_ny / d_px:
  (*infiniteLineMeasurementModelJacobian_H)(1, 0) = -cameraHeight *(l_y_1 - l_y_2) *(cos_p*(l_x_1 - l_x_2) + sin_p*(l_y_1 - l_y_2)) *t1 *t4;
  // d_ny / d_py:
  (*infiniteLineMeasurementModelJacobian_H)(1, 1) = cameraHeight *(l_x_1 - l_x_2) *(cos_p*(l_x_1 - l_x_2) + sin_p*(l_y_1 - l_y_2)) *t1 *t4;
  // d_ny / d_pteta:
  (*infiniteLineMeasurementModelJacobian_H)(1, 2) = cameraHeight * (cos_p*(l_y_1 - l_y_2) - sin_p*(l_x_1 - l_x_2)) * t3;
  // d_nz / d_px:
  (*infiniteLineMeasurementModelJacobian_H)(2, 0) = cameraHeight2 * (l_y_1 - l_y_2) * (sqr(l_x_1 - l_x_2) + sqr(l_y_1 - l_y_2)) * t4;
  // d_nz / d_py:
  (*infiniteLineMeasurementModelJacobian_H)(2, 1) = -cameraHeight2 * (l_x_1 - l_x_2) * (sqr(l_x_1 - l_x_2) + sqr(l_y_1 - l_y_2)) * t4;
  // d_nz / d_pteta:
  (*infiniteLineMeasurementModelJacobian_H)(2, 2) = 0;

  expectedMeasurement.measurementModelJacobian = infiniteLineMeasurementModelJacobian_H;

  return expectedMeasurement;
}

void PoseHypothesis2016::updateSymmetryByComparingRemoteToLocalModels(
  const BallModel & theBallModel,
  const RemoteBallModel & theRemoteBallModel,
  const LocalRobotMap & theLocalRobotMap,
  const RemoteRobotMap & theRemoteRobotMap,
  const FrameInfo & frameInfo,
  const SelfLocator2016Parameters & parameters)
{
  // the maps and models only contain the means, not the covariances
  // so we can only compare the differences, not the likelihoods (which would make more sense)
  float difference = 0;
  float differenceOfMirrored = 0;

  const float minDiffToMatter = 500;

  Vector2f localModelInGlobalCoords, localModelInGlobalCoordsMirrored;
  Pose2f pose;
  getRobotPose(pose);

  // check ball
  if (frameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 1000
    && frameInfo.getTimeSince(theRemoteBallModel.timeWhenLastSeen) < 1500
    && theBallModel.validity > 0.7f && theRemoteBallModel.validity > 0.8f)
  {
    localModelInGlobalCoords = pose*theBallModel.estimate.position;
    difference = (theRemoteBallModel.position - localModelInGlobalCoords).norm();
    localModelInGlobalCoordsMirrored = localModelInGlobalCoords*-1;
    differenceOfMirrored = (theRemoteBallModel.position - localModelInGlobalCoordsMirrored).norm();

    const bool isThisPositionMoreLikely = difference < differenceOfMirrored;
    const float& closestDifference = isThisPositionMoreLikely ? difference : differenceOfMirrored;

    if (std::abs(difference - differenceOfMirrored) > minDiffToMatter)
    {
      const bool inRange = closestDifference < parameters.symmetryUpdate_maxDistanceToClosestRemoteModel;
      if (frameInfo.getTimeSince(theRemoteBallModel.timeWhenLastSeenByGoalie) < 1000)
      {
        updateSymmetryConfidence(inRange && isThisPositionMoreLikely, parameters.symmetryUpdate_influenceOfNewMeasurementByGoalie);
      }
      else
      {
        updateSymmetryConfidence(inRange && isThisPositionMoreLikely, parameters.symmetryUpdate_influenceOfNewMeasurement);
      }
      lastSymmetryUpdate = frameInfo.time;
    }
  }

  const float increment = (parameters.localizationState_symmetryFoundAgainWhenBestConfidenceAboveThisThreshold
      - parameters.localizationState_symmetryLostWhenBestConfidenceBelowThisThreshold) / 2;

  if (symmetryConfidence <= parameters.localizationState_symmetryLostWhenBestConfidenceBelowThisThreshold)
  {
    symmetryConfidenceState = SideConfidence::CONFUSED;
  } else if (symmetryConfidence < (parameters.localizationState_symmetryLostWhenBestConfidenceBelowThisThreshold + increment))
  {
    symmetryConfidenceState = SideConfidence::UNSURE;
  } else if (symmetryConfidence < (parameters.localizationState_symmetryFoundAgainWhenBestConfidenceAboveThisThreshold))
  {
    symmetryConfidenceState = SideConfidence::ALMOST_CONFIDENT;
  }
  else
  {
    symmetryConfidenceState = SideConfidence::CONFIDENT;
  }

  // for testing: use ball only.
  // works better so far...
  return;

  difference = 0;
  differenceOfMirrored = 0;

  float smallestDiff;
  float smallestDiffOfMirrored;

  const float bigDiff = 1000000.f;

  int numberOfRobotsToMatch = 0;

  // test if own pose is somewhere in the remote robot map
  localModelInGlobalCoords = pose.translation;
  localModelInGlobalCoordsMirrored = localModelInGlobalCoords*-1;
  smallestDiff = bigDiff;
  smallestDiffOfMirrored = bigDiff;
  for (std::vector<RobotMapEntry>::const_iterator i = theRemoteRobotMap.robots.begin(); i != theRemoteRobotMap.robots.end(); ++i)
  {
    float tempDiff = (localModelInGlobalCoords - i->pose.translation).norm();
    if (tempDiff < smallestDiff)
    {
      smallestDiff = tempDiff;
    }
    tempDiff = (localModelInGlobalCoordsMirrored - i->pose.translation).norm();
    if (tempDiff < smallestDiffOfMirrored)
    {
      smallestDiffOfMirrored = tempDiff;
    }
  }
  difference += smallestDiff;
  differenceOfMirrored += smallestDiffOfMirrored;
  numberOfRobotsToMatch++;

  // test other local observations
  for (std::vector<RobotMapEntry>::const_iterator j = theLocalRobotMap.robots.begin(); j != theLocalRobotMap.robots.end(); ++j)
  {
    smallestDiff = bigDiff;
    smallestDiffOfMirrored = bigDiff;
    localModelInGlobalCoords = pose*j->pose.translation;
    localModelInGlobalCoordsMirrored = localModelInGlobalCoords*-1;
    for (std::vector<RobotMapEntry>::const_iterator i = theRemoteRobotMap.robots.begin(); i != theRemoteRobotMap.robots.end(); ++i)
    {
      float tempDiff = (localModelInGlobalCoords - i->pose.translation).norm();
      if (tempDiff < smallestDiff)
      {
        smallestDiff = tempDiff;
      }
      tempDiff = (localModelInGlobalCoordsMirrored - i->pose.translation).norm();
      if (tempDiff < smallestDiffOfMirrored)
      {
        smallestDiffOfMirrored = tempDiff;
      }
    }
    difference += smallestDiff;
    differenceOfMirrored += smallestDiffOfMirrored;
    numberOfRobotsToMatch++;
  }

  if (std::abs(difference - differenceOfMirrored) > numberOfRobotsToMatch * minDiffToMatter)
  {
    updateSymmetryConfidence(difference < differenceOfMirrored, parameters.symmetryUpdate_influenceOfNewMeasurement);
  }
}

void PoseHypothesis2016::draw(ColorRGBA myselfColor) const
{
  GaussianDistribution2D gd;

  extractGaussianDistribution2DFromStateEstimation(gd);
  drawGaussian(gd, myselfColor, 50, true);

  Pose2f pose;
  getRobotPose(pose);

  POSE_2D_SAMPLE("module:SelfLocator2016:hypothesis", pose, myselfColor);

  DRAWTEXT("module:SelfLocator2016:hypothesis", pose.translation.x(), pose.translation.y() - 180, 50, myselfColor, "pc: " << positionConfidence);
  DRAWTEXT("module:SelfLocator2016:hypothesis", pose.translation.x(), pose.translation.y() - 300, 50, myselfColor, "sc: " << symmetryConfidence);
}

void PoseHypothesis2016::drawGaussian(const GaussianDistribution2D &gd, const ColorRGBA &color, int penSize, bool drawCenter) const
{
  Pose2f mean = Pose2f(gd.mean.cast<float>());
  double averageX = mean.translation.x();
  double averageY = mean.translation.y();


  if (drawCenter)
  {
    CIRCLE("GaussianTools:covarianceEllipse", 
      averageX, averageY, 60, 20, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, color);
  }

  if (gd.mean.norm()>20000 || gd.covariance.col(0).norm()>10000 * 10000 || gd.covariance.col(1).norm()>10000 * 10000)
  {
    return;
  }

  Matrix2d eigenVecs;
  // Vector2<double> eigenVec0 = getEigenVectors(gaussian.covariance).c[0]; 
  Eigen::EigenSolver<Matrix2d> es(gd.covariance); //constructor calls compute() and calculates eigenvectors and values
  Vector2d eigenVec0 = es.eigenvectors().col(0).real();
  //Vector2d eigenVec1 = es.eigenvectors().col(1).real();


  //double temp2 = toDegrees(gaussian.covariance.c[0].angle());
  //double temp = toDegrees(eigenVec0.angle());

  Matrix2d eigenVals = es.eigenvalues().real().asDiagonal(); // TODO BH2015 port: more efficient...
  double eigenVal0 = sgn(eigenVals.col(0).x())*sqrt(std::abs(eigenVals.col(0).x()));   // std-deviation to make it drawable on the field
  double eigenVal1 = sgn(eigenVals.col(1).y())*sqrt(std::abs(eigenVals.col(1).y()));


  // comment out to watch eigenvectors
  //Vector2<double> eigenDraw1, eigenDraw2;
  //eigenDraw1 = eigenVec0.normalize() * eigenVal0;
  //eigenDraw2 = eigenVec1.normalize() * eigenVal1;

  //LINE("GaussianTools:covarianceEllipse", averageX, averageY,averageX+eigenDraw1.x,averageY+eigenDraw1.y,20,Drawings::ps_solid,color);
  //LINE("GaussianTools:covarianceEllipse", averageX, averageY,averageX+eigenDraw2.x,averageY+eigenDraw2.y,20,Drawings::ps_solid,color);

  COMPLEX_DRAWING("GaussianTools:covarianceEllipse")
  {
    double alpha = atan2(eigenVec0.y(), eigenVec0.x());
    double a = eigenVal0;
    double b = eigenVal1;
    double x1 = 0;
    double x2 = 0;
    double y1 = 0;
    double y2 = 0;

    int step = (int)(a / 10);
    if (!step)
    {
      step = 1;
    }

    for (int xc = 0; (xc + step) < a; xc += step)
    {
      x1 = xc;
      y1 = (sqrt(1 - sqr(x1 / a))* b);
      x2 = xc + step;
      y2 = (sqrt(1 - sqr(x2 / a))* b);


      LINE("GaussianTools:covarianceEllipse", cos(alpha)*(x1)+averageX - sin(alpha)*(y1),
        sin(alpha)*(x1)+cos(alpha)*(y1)+averageY,
        cos(alpha)*(x2)+averageX - sin(alpha)*(y2),
        sin(alpha)*(x2)+cos(alpha)*(y2)+averageY,
        penSize, Drawings::solidPen,
        color);
      LINE("GaussianTools:covarianceEllipse", cos(alpha)*(x1)+averageX - sin(alpha)*(-y1),
        sin(alpha)*(x1)+cos(alpha)*(-y1) + averageY,
        cos(alpha)*(x2)+averageX - sin(alpha)*(-y2),
        sin(alpha)*(x2)+cos(alpha)*(-y2) + averageY,
        penSize, Drawings::solidPen,
        color);
      LINE("GaussianTools:covarianceEllipse", cos(alpha)*(-x1) + averageX - sin(alpha)*(y1),
        sin(alpha)*(-x1) + cos(alpha)*(y1)+averageY,
        cos(alpha)*(-x2) + averageX - sin(alpha)*(y2),
        sin(alpha)*(-x2) + cos(alpha)*(y2)+averageY,
        penSize, Drawings::solidPen,
        color);
      LINE("GaussianTools:covarianceEllipse", cos(alpha)*(-x1) + averageX - sin(alpha)*(-y1),
        sin(alpha)*(-x1) + cos(alpha)*(-y1) + averageY,
        cos(alpha)*(-x2) + averageX - sin(alpha)*(-y2),
        sin(alpha)*(-x2) + cos(alpha)*(-y2) + averageY,
        penSize, Drawings::solidPen,
        color);
    }

    x1 = a;
    y1 = 0;

    LINE("GaussianTools:covarianceEllipse", cos(alpha)*(x1)+averageX - sin(alpha)*(y1),
      sin(alpha)*(x1)+cos(alpha)*(y1)+averageY,
      cos(alpha)*(x2)+averageX - sin(alpha)*(y2),
      sin(alpha)*(x2)+cos(alpha)*(y2)+averageY,
      penSize, Drawings::solidPen,
      color);
    LINE("GaussianTools:covarianceEllipse", cos(alpha)*(x1)+averageX - sin(alpha)*(-y1),
      sin(alpha)*(x1)+cos(alpha)*(-y1) + averageY,
      cos(alpha)*(x2)+averageX - sin(alpha)*(-y2),
      sin(alpha)*(x2)+cos(alpha)*(-y2) + averageY,
      penSize, Drawings::solidPen,
      color);
    LINE("GaussianTools:covarianceEllipse", cos(alpha)*(-x1) + averageX - sin(alpha)*(y1),
      sin(alpha)*(-x1) + cos(alpha)*(y1)+averageY,
      cos(alpha)*(-x2) + averageX - sin(alpha)*(y2),
      sin(alpha)*(-x2) + cos(alpha)*(y2)+averageY,
      penSize, Drawings::solidPen,
      color);
    LINE("GaussianTools:covarianceEllipse", cos(alpha)*(-x1) + averageX - sin(alpha)*(-y1),
      sin(alpha)*(-x1) + cos(alpha)*(-y1) + averageY,
      cos(alpha)*(-x2) + averageX - sin(alpha)*(-y2),
      sin(alpha)*(-x2) + cos(alpha)*(-y2) + averageY,
      penSize, Drawings::solidPen,
      color);
  }
}

// Do not call this, it writes over array boundaries!
#if 0
bool PoseHypothesis2016::test()
{
  bool result = true;

  Vector3<double> stateBefore(-1500,-1500,0), stateAfter1, stateAfter2, stateAfter3, stateAfter4;
  state[0] = stateBefore.x();
  state[1] = stateBefore.y();
  state[2] = stateBefore.z;
  covariance.setToIdentity();
  covariance *= 100000.0;
  covariance.setMember(3,3,10);

  Vector2<double> observedStartForZeroAngle(1000, 500);
  Vector2<double> observedEndForZeroAngle(1000, -500);
  Vector2<double> observedStart = observedStartForZeroAngle;
  Vector2<double> observedEnd = observedEndForZeroAngle;
  Vector2<double> correspondenceStart(0, 500-1500);
  Vector2<double> correspondenceEnd(0, -500-1500);

  std::vector<Vector3<double> > observationsAsNormals;
  std::vector<Vector2<double> > correspondencesInGlobalCoords;

  // update test 1
  observationsAsNormals.clear();
  correspondencesInGlobalCoords.clear();

  Vector3<double> observedProjectiveGeometryNormal = Vector3<double>( observedStart.x,
    observedStart.y,
    -cameraHeight )
    ^
    Vector3<double>( observedEnd.x,
    observedEnd.y,
    -cameraHeight );
  observedProjectiveGeometryNormal.normalize();
  observationsAsNormals.push_back( observedProjectiveGeometryNormal );
  correspondencesInGlobalCoords.push_back( correspondenceStart );
  correspondencesInGlobalCoords.push_back( correspondenceEnd   );

  CALL_KALMAN_INFINITE_LINE_UPDATE_FUNCTION(3, observationsAsNormals, correspondencesInGlobalCoords);

  result = result && (state[0] > stateBefore.x);
  stateAfter1.x = state[0];
  stateAfter1.y = state[1];
  stateAfter1.z = state[2];


  // update test 2
  stateBefore.z = pi_2;
  observedStart = observedStartForZeroAngle;
  observedEnd = observedEndForZeroAngle;
  observedStart.rotateRight();
  observedEnd.rotateRight();
  state[0] = stateBefore.x;
  state[1] = stateBefore.y;
  state[2] = stateBefore.z;
  covariance.setToIdentity();
  covariance *= 100000.0;
  covariance.setMember(3, 3, 10);
  observationsAsNormals.clear();
  correspondencesInGlobalCoords.clear();

  observedProjectiveGeometryNormal = Vector3<double>(observedStart.x,
    observedStart.y,
    -cameraHeight)
    ^
    Vector3<double>(observedEnd.x,
    observedEnd.y,
    -cameraHeight);
  observedProjectiveGeometryNormal.normalize();

  observationsAsNormals.push_back(observedProjectiveGeometryNormal);
  correspondencesInGlobalCoords.push_back(correspondenceStart);
  correspondencesInGlobalCoords.push_back(correspondenceEnd);

  CALL_KALMAN_INFINITE_LINE_UPDATE_FUNCTION(3, observationsAsNormals, correspondencesInGlobalCoords);

  result = result && (state[0] > stateBefore.x);
  stateAfter2.x = state[0];
  stateAfter2.y = state[1];
  stateAfter2.z = state[2];


  // update test 3
  stateBefore.z = -pi_2;
  observedStart = observedStartForZeroAngle;
  observedEnd = observedEndForZeroAngle;
  observedStart.rotateLeft();
  observedEnd.rotateLeft();
  state[0] = stateBefore.x;
  state[1] = stateBefore.y;
  state[2] = stateBefore.z;
  covariance.setToIdentity();
  covariance *= 100000.0;
  covariance.setMember(3, 3, 10);
  observationsAsNormals.clear();
  correspondencesInGlobalCoords.clear();

  observedProjectiveGeometryNormal = Vector3<double>(observedStart.x,
    observedStart.y,
    -cameraHeight)
    ^
    Vector3<double>(observedEnd.x,
    observedEnd.y,
    -cameraHeight);
  observedProjectiveGeometryNormal.normalize();

  observationsAsNormals.push_back(observedProjectiveGeometryNormal);
  correspondencesInGlobalCoords.push_back(correspondenceStart);
  correspondencesInGlobalCoords.push_back(correspondenceEnd);

  CALL_KALMAN_INFINITE_LINE_UPDATE_FUNCTION(3, observationsAsNormals, correspondencesInGlobalCoords);

  result = result && (state[0] > stateBefore.x);
  stateAfter3.x = state[0];
  stateAfter3.y = state[1];
  stateAfter3.z = state[2];


  // update test 4
  double angle = pi_2 - 0.01;
  stateBefore.z = angle;
  observedStart = observedStartForZeroAngle;
  observedEnd = observedEndForZeroAngle;
  observedStart.rotate(-angle);
  observedEnd.rotate(-angle);
  state[0] = stateBefore.x;
  state[1] = stateBefore.y;
  state[2] = stateBefore.z;
  covariance.setToIdentity();
  covariance *= 100000.0;
  covariance.setMember(3, 3, 10);
  observationsAsNormals.clear();
  correspondencesInGlobalCoords.clear();

  observedProjectiveGeometryNormal = Vector3<double>(observedStart.x,
    observedStart.y,
    -cameraHeight)
    ^
    Vector3<double>(observedEnd.x,
    observedEnd.y,
    -cameraHeight);
  observedProjectiveGeometryNormal.normalize();

  observationsAsNormals.push_back(observedProjectiveGeometryNormal);
  correspondencesInGlobalCoords.push_back(correspondenceStart);
  correspondencesInGlobalCoords.push_back(correspondenceEnd);

  CALL_KALMAN_INFINITE_LINE_UPDATE_FUNCTION(3, observationsAsNormals, correspondencesInGlobalCoords);

  result = result && (state[0] > stateBefore.x);
  stateAfter4.x = state[0];
  stateAfter4.y = state[1];
  stateAfter4.z = state[2];


  return result;
}




#endif
