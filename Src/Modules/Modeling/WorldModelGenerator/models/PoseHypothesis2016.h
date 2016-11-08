/**
* @file PoseHypothesis2016.h
*
* This class represents a single world model hypothesis.
* Actually, in this version only the robot pose is modeled by an EKF,
* the dynamic elements of the environment are modeled elsewhere,
* but the (mostly) static surrounding of the field is also modeled here (at least that's planned).
*
* @author <a href="mailto:stefan.tasse@tu-dortmund.de">Stefan Tasse</a>
* @author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
*/

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Math/GaussianDistribution2D.h"
#include "Tools/Math/GaussianDistribution3D.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/LineMatchingResult.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/CenterCirclePercept.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Perception/CLIPGoalPercept.h"
#include "Representations/Perception/PenaltyCrossPercept.h"
#include <math.h>
#include <algorithm>
#include "Tools/Debugging/Modify.h"

#include "Modules/Modeling/WorldModelGenerator/SelfLocator2016Parameters.h"
#include "PoseHypothesis2016_Defines.h"


#define MAX_OBSERVATIONS 20
#define MAX_OBSERVATIONS_INFINITELINE 4

#ifdef TARGET_ROBOT
// No need for being thread protected on robot (only needed if team is simulated)
#define tls static 
#define tls_type  
#else //TARGET_ROBOT
// Need to be thread protected due to simulation of one team
// Check platform
#ifdef WINDOWS
// in newer visual c++ versions, thread_local (C++11 standard) will be supported, but for now..
#define tls static __declspec(thread) 
#define tls_type __declspec(thread) 
#else //WIN32
#ifdef LINUX
#define tls thread_local static 
#define tls_type thread_local 
#else //LINUX
#ifdef OSX
// Seems as if Mac doesn't support thread local (need to check, maybe can use "__thread" as linux/clang)
#define tls static __thread
#define tls_type  __thread
#endif //MACOSX
#endif //!LINUX
#endif //!WIN32
#endif //!TARGET_ROBOT

#ifndef tls
// If tls not defined by now define it as static (not thread protected); simulation of one team might get messed up
#pragma message("warning: Thread local storage is not defined for your platform. Please add your platform to the define on top.")
#define tls static 
#define tls_type  
#endif

class PoseHypothesis2016
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  enum FixedParameters
  {
    totalDimension = 3, // 3=ownPose
  };

  PoseHypothesis2016(const Pose2f & newPose, float _positionConfidence, double _symmetryConfidence, const unsigned &timeStamp, const SelfLocator2016Parameters& parameters);
  PoseHypothesis2016(const PoseHypothesis2016& other, const unsigned &timeStamp);
  PoseHypothesis2016& operator=(const PoseHypothesis2016& other);

private:
  typedef KalmanStateUpdateObservations2016<totalDimension, 2 * MAX_OBSERVATIONS> KalmanStateUpdate;
  typedef KalmanStateUpdateInfiniteLines2016<totalDimension, 3 * MAX_OBSERVATIONS_INFINITELINE> KalmanInfiniteLineStateUpdate;

  uint64_t _uniqueId;

  Eigen::Matrix<double, totalDimension, 1>           state;
  Eigen::Matrix<double, totalDimension, totalDimension> covariance;

  float                                     positionConfidence;
  double                                    normalizedPositionConfidence;
  double                                    symmetryConfidence;
  SideConfidence::ConfidenceState           symmetryConfidenceState;
  unsigned                                  lastSymmetryUpdate;

  double                                    temporaryLocalSymmetryLikelihood;


  Matrix2d singleMeasurementCovariance_2x2;

  unsigned int                           timeOfLastPrediction;

  bool debugLineMatching;
  bool initialized;
  
  unsigned creationTime;

  std::vector<AngleObservation> observationAnglesWithLocalFeaturePerceptionsSpherical;
  std::vector< ExpectedMeasurementLandmark<totalDimension> > correspondencesInGlobalCoordsWithLocalFeaturePerceptionsSpherical;

  std::vector<NormalObservation> observationsAsNormalsWithLocalFeaturePerceptionsInfiniteLines;
  std::vector<Vector2d> correspondencesInGlobalCoordsWithLocalFeaturePerceptionsInfiniteLines;

  //required for the kalman updates
  tls Eigen::Matrix<double, 2, totalDimension>* angleObservationMeasurementModelJacobian_H;
  tls Eigen::Matrix<double, 3, totalDimension>* infiniteLineMeasurementModelJacobian_H;

  tls KalmanStateUpdate* stateUpdate;
  tls KalmanInfiniteLineStateUpdate* stateUpdateInfiniteLine;

public:
  static void cleanup();

  uint64_t getUniqueId() const { return _uniqueId; }
  
  const unsigned &getCreationTime() const { return creationTime; }

  bool isInsideCarpet(const FieldDimensions &fd) const { return fd.isInsideCarpet(Vector2f((float)state[0], (float)state[1])); }

  bool containsInvaidValues() const { return state[0] != state[0] || state[1] != state[1] || state[2] != state[2]; }

  void init(const Pose2f & newPose, const SelfLocator2016Parameters& parameters);

  void predict(const Pose2f & odometryDelta, const SelfLocator2016Parameters& parameters);

  void fillCorrectionMatrices(
    const LineMatchingResult & theLineMatchingResult,
    const CLIPCenterCirclePercept &theCenterCirclePercept,
    const CLIPGoalPercept &theGoalPercept,
    const PenaltyCrossPercept &thePenaltyCrossPercept,
    const FieldDimensions & theFieldDimensions,
    const CameraMatrix & theCameraMatrixCorrected,
    const CameraMatrixUpper & theCameraMatrixUpperCorrected,
    const SelfLocator2016Parameters &parameters);

  // this is the normal sensor update used for all features (but for the exceptions below)
  bool updatePositionConfidenceWithLocalFeaturePerceptionsSpherical(
    const LineMatchingResult & theLineMatchingResult,
    const CLIPCenterCirclePercept &theCenterCirclePercept,
    const CLIPGoalPercept &theGoalPercept,
    const PenaltyCrossPercept &thePenaltyCrossPercept,
    const FieldDimensions & theFieldDimensions,
    const CameraMatrix & theCameraMatrix,
    const CameraMatrixUpper & theCameraMatrixUpper,
    const SelfLocator2016Parameters & parameters);

  // this is the normal sensor update used for all features (but for the exceptions below)
  void updateStateWithLocalFeaturePerceptionsSpherical(const SelfLocator2016Parameters & parameters);


  // for very close observations, spherical coordinates come close to singularities, 
  // which should be avoided. I.e. updating with the center circle while standing on its center, etc.
  // Those can be updated with cartesian coordinates.
  void updateWithLocalFeaturePerceptionsCartesian(
    const CLIPCenterCirclePercept &theCenterCirclePercept,
    const PenaltyCrossPercept &thePenaltyCrossPercept,
    const SelfLocator2016Parameters & parameters);

  // Straight lines offer no real corresponding point on the field to update with,
  // which makes updates with max likelihood point correspondences
  // underestimate the real certainty or may even cause drift by updating like that.
  bool updatePositionConfidenceWithLocalFeaturePerceptionsInfiniteLines(
    const LineMatchingResult & theLineMatchingResult,
    const CLIPCenterCirclePercept &theCenterCirclePercept,
    const FieldDimensions & theFieldDimensions,
    const SelfLocator2016Parameters & parameters);
  void updateStateWithLocalFeaturePerceptionsInfiniteLines(const SelfLocator2016Parameters & parameters);

  bool updatePositionConfidenceWithLocalFeaturePerceptionsWeighted(
    const CLIPFieldLinesPercept & theFieldLinesPercept,
    const CLIPCenterCirclePercept &theCenterCirclePercept,
    const CLIPGoalPercept &theGoalPercept,
    const PenaltyCrossPercept &thePenaltyCrossPercept,
    const FieldDimensions & theFieldDimensions,
    const SelfLocator2016Parameters & parameters);

  void updateStateRotationWithLocalFieldLines(
    const CLIPFieldLinesPercept &theFieldLinesPercept,
    const SelfLocator2016Parameters &parameters);

  void updateSymmetryByComparingRemoteToLocalModels(
    const BallModel & theBallModel,
    const RemoteBallModel & theRemoteBallModel,
    const LocalRobotMap & theLocalRobotMap,
    const RemoteRobotMap & theRemoteRobotMap,
    const FrameInfo & frameInfo,
    const SelfLocator2016Parameters & parameters);

  // Do not call this, it writes over array boundaries!
  //bool test();

  void extractGaussianDistribution3DFromStateEstimation(GaussianDistribution3D & gd)
  {
    gd.mean.x() = state[0];
    gd.mean.y() = state[1];
    gd.mean.z() = state[2];
    gd.covariance = covariance.block(0, 0, 3, 3); // in case of more totaldimensions!?!?
  }

  void getRobotPose(Pose2f & robotPose) const
  {
    robotPose.translation.x() = (float)state[0];
    robotPose.translation.y() = (float)state[1];
    robotPose.rotation = (float)state[2];
  }

  void getRobotPose(Pose2f & robotPose, Matrix3d & poseCovar) const
  {
    robotPose.translation.x() = (float)state[0];
    robotPose.translation.y() = (float)state[1];
    robotPose.rotation = (float)state[2];
    poseCovar = covariance.block(0, 0, 3, 3);
  }

  void rotateHypothesis(double angle) // for handling fall downs
  {
    state[2] = Angle::normalize(state[2] + angle);
  }

  double getSymmetryConfidence() const
  {
    return symmetryConfidence;
  }

  SideConfidence::ConfidenceState getSymmetryConfidenceState() const
  {
    return symmetryConfidenceState;
  }

  float getPositionConfidence() const
  {
    return positionConfidence;
  }

  void resetSymmetryConfidence(double newSymmetryConfidence)
  {
    symmetryConfidence = newSymmetryConfidence;
  }

  void resetSymmetryState(SideConfidence::ConfidenceState confidenceState)
  {
    symmetryConfidenceState = confidenceState;
  }

  void scalePositionConfidence(float factor) // for merging/deleting of hypotheses or handling fall downs
  {
    normalizedPositionConfidence *= factor;
    positionConfidence *= factor;
  }

  void normalizePositionConfidence(double factor)
  {
    normalizedPositionConfidence = positionConfidence*factor;
  }

  void draw(ColorRGBA myselfColor = ColorRGBA(255, 255, 255, 255)) const;

private:

  void calculateProcessModelJacobian(int dt);

  // LEVEL 1
  bool updatePositionConfidenceWithSingleLines(
    const CLIPFieldLinesPercept & theFieldLinesPercept,
    const FieldDimensions & theFieldDimensions,
    const SelfLocator2016Parameters & parameters,
    const Pose2f &robotPose,
    const std::vector<int> &linesHorizontal,
    const std::vector<int> &linesVertical);
  // LEVEL 2
  bool updatePositionConfidenceWithLineCrossings(
    const CLIPFieldLinesPercept & theFieldLinesPercept,
    const FieldDimensions & theFieldDimensions,
    const SelfLocator2016Parameters & parameters,
    const Pose2f &robotPose,
    const std::vector<int> &linesHorizontal,
    const std::vector<int> &linesVertical);
  // LEVEL 3
  bool updatePositionConfidenceWithLineAndLandmark(
    const CLIPFieldLinesPercept & theFieldLinesPercept,
    const CLIPCenterCirclePercept &theCenterCirclePercept,
    const CLIPGoalPercept &theGoalPercept,
    const PenaltyCrossPercept &thePenaltyCrossPercept,
    const FieldDimensions & theFieldDimensions,
    const SelfLocator2016Parameters & parameters,
    const Pose2f &robotPose,
    const std::vector<int> &linesHorizontal,
    const std::vector<int> &linesVertical);

  bool findGoalPostMatch(const CLIPGoalPercept::GoalPost & goalPost,
    const CameraMatrix & theCameraMatrix,
    const CameraMatrixUpper & theCameraMatrixUpper,
    AngleObservation & realMeasurement,
    ExpectedMeasurementLandmark<totalDimension> &bestCorrespondenceInGlobalCoords,
    const FieldDimensions & theFieldDimensions,
    const SelfLocator2016Parameters& parameters);


  ExpectedMeasurementLandmark<totalDimension> calculateMeasurementModelForAngularLandmarkObservation(
    const Vector2d & correspondenceInGlobalCoords, const float &cameraHeight, const SelfLocator2016Parameters & parameters);

  ExpectedMeasurementInfiniteLine<totalDimension> calculateMeasurementModelForInfiniteLineObservation(
    const Vector2d & startPointCorrespondenceInGlobalCoords, const Vector2d & endPointCorrespondenceInGlobalCoords, const float &cameraHeight);

  AngleObservation getSphericalRepresentationForCartesianObservation(const Vector2i & pointInRelativeCoordinates, const float &cameraHeight)
  {
    return getSphericalRepresentationForCartesianObservation(Vector2d(pointInRelativeCoordinates.cast<double>()), cameraHeight);
  }

  AngleObservation getSphericalRepresentationForCartesianObservation(const Vector2f & pointInRelativeCoordinates, const float &cameraHeight)
  {
    return getSphericalRepresentationForCartesianObservation(Vector2d(pointInRelativeCoordinates.cast<double>()), cameraHeight);
  }

  AngleObservation getSphericalRepresentationForCartesianObservation(const Vector2d & pointInRelativeCoordinates, const float &cameraHeight)
  {
    Vector2d angularObservation;
    angularObservation.x() = atan2(cameraHeight, pointInRelativeCoordinates.norm()); // vertical angle
    angularObservation.y() = pointInRelativeCoordinates.angle(); // horizontal angle
    return AngleObservation(angularObservation, cameraHeight);
  }


  /**
   * Updates the position confidence of the PoseHypothesis.
   * If this foundCorrespondenceMatch is false, the confidence is decreased by the fully weighted measurement.
   * foundCorrespondenceMatch: Determines wether a corresponding match has been found.
   * measurement: The influence of the perception. This is typically one of the parameters.sensorUpdate_influenceXXX values.
   * weight: The weight of the measurement [0:1] (only used if foundCorrespondenceMatch is true).
   */
  void updatePositionConfidence(bool foundCorrespondenceMatch, const float &measurement, const float weight)
  {
    const float update = weight*measurement;
    //double distanceBasedUpdateChange = foundCorrespondenceMatch ? parameters.sensorUpdate_influenceOfNewMeasurementDistanceOnPositionConfidence*distanceFactor : 0;
    positionConfidence *= (1.f - (foundCorrespondenceMatch ? update : measurement));
    positionConfidence += foundCorrespondenceMatch ? update : 0;
    //positionConfidence += foundCorrespondenceMatch ? distanceBasedUpdateChange : 0;
  }
  void updateSymmetryConfidence(bool thisPositionMoreLikeliThanMirroredOne, const float& influence)
  {
    symmetryConfidence *= (1.0 - influence);
    symmetryConfidence += thisPositionMoreLikeliThanMirroredOne ? influence : 0;
    symmetryConfidence = std::max(symmetryConfidence, 0.01);
  }

  double calculateMeasurementLikelihoodSpherical(const Vector2d & angles1, const Vector2d & angles2, const SelfLocator2016Parameters& parameters) const
  {
    Vector2d diff = angles1 - angles2;
    if (std::abs(diff.x()) < parameters.pointMatching_maxAllowedVerticalAngleDifference
      &&std::abs(diff.y()) < parameters.pointMatching_maxAllowedHorizontalAngleDifference)
    {
      double exponent = sqr(diff.x()) / parameters.sensorUpdate_verticalAngleVariance
        + sqr(diff.y()) / parameters.sensorUpdate_horizontalAngleVariance;
      double probability(1.0 / (pi2 * sqrt(parameters.sensorUpdate_verticalAngleVariance*parameters.sensorUpdate_horizontalAngleVariance)));
      probability *= exp(-0.5*exponent);
      return probability;
    }
    else
    {
      return 0;
    }
  }

  double getNormalizedPositionConfidence() const
  {
    return normalizedPositionConfidence;
  }

  void extractGaussianDistribution2DFromStateEstimation(GaussianDistribution2D & gd) const
  {
    gd.mean.x() = state[0];
    gd.mean.y() = state[1];
    gd.covariance = covariance.block(0, 0, 2, 2);
  }

  bool findBestAngle(const CLIPFieldLinesPercept &theFieldLinesPercept, double &result)
  {
    std::vector<double> angles;
    std::vector<double> weights;
    double maxAngle = -pi_4;
    double minAngle = pi_4;
    for (auto &line : theFieldLinesPercept.lines)
    {
      if (!line.isPlausible)
        continue;

      if (line.validity > .5)
      {
        double angle = pi_2 - (line.endOnField - line.startOnField).angle();
        while (std::abs(angle) >= pi_4)
        {
          angle += pi_2;
          angle = Angle::normalize(angle);
        }
        angles.push_back(angle);
        maxAngle = std::max(angle, maxAngle);
        minAngle = std::min(angle, minAngle);
        weights.push_back(line.validity);
      }
    }
    if (std::abs(maxAngle - minAngle) > pi_4)
      return false;
    double weightSum = 0.0;
    result = 0.0;
    for (unsigned int i = 0; i < angles.size(); i++)
    {
      weightSum += weights[i];
      result += weights[i] * angles[i];
    }
    result /= weightSum;
    return true;
  }

  inline float calcWeightForPoseDifference(const Pose2f &poseDiff, const SelfLocator2016Parameters &parameters)
  {
    float weight = std::max(0.f, (1.f - poseDiff.translation.norm() / 500.f));
    weight *= std::max<float>(0.f, (1.f - std::abs(poseDiff.rotation) / parameters.sensorUpdate_worstAngleDifference));
    return weight;
  }

  inline static const Pose2f& getClosestPose(const Pose2f &robotPose, const Pose2f &pose1, const Pose2f &pose2)
  {
    const Pose2f diffPose1 = (pose1 - robotPose);
    const Pose2f diffPose2 = (pose2 - robotPose);
    const Pose2f diffPose12 = (pose2 - pose2);
    const float maxPoseDiff = 200;
    if ((diffPose1.translation.norm() < maxPoseDiff) && (diffPose2.translation.norm() < maxPoseDiff) && (diffPose12.translation.norm() < maxPoseDiff))
    {
      if (std::abs(Angle::normalize(diffPose1.rotation)) < std::abs(Angle::normalize(diffPose2.rotation)))
        return pose1;
      else
        return pose2;
    }
    else
    {
      if (diffPose1.translation.norm() < diffPose2.translation.norm())
        return pose1;
      else
        return pose2;
    }
  }

  void drawGaussian(const GaussianDistribution2D &gd, const ColorRGBA &color, int penSize = 10, bool drawCenter = true) const;

};
