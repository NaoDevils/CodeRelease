/**
* @file PoseHypothesis2016_Defines.h
*
* TODO BH2015 port: check math and use eigen lib better (e.g. the .block mechanic)
*
* @author <a href="mailto:stefan.tasse@tu-dortmund.de">Stefan Tasse</a>
* @author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
*/

#pragma once

#include "Tools/Math/Eigen.h"

class AngleObservation
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector2d observation;
  float cameraHeight;

  AngleObservation() : observation(Vector2d::Zero()) {};
  AngleObservation(const Vector2d observation, const float &cameraHeight) : observation(observation), cameraHeight(cameraHeight) {};
};

class NormalObservation
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector3d observation;
  float cameraHeight;

  NormalObservation() : observation(Vector3d::Zero()), cameraHeight(0) {};
  NormalObservation(const Vector3d observation, const float &cameraHeight) : observation(observation), cameraHeight(cameraHeight) {};
};

template<int dim>
class ExpectedMeasurementLandmark
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector2d measurement;
  Eigen::Matrix<double, 2, dim>* measurementModelJacobian;
  double weight;

  ExpectedMeasurementLandmark() : measurement(Vector2d::Zero()), measurementModelJacobian(0), weight(0) {}
  ExpectedMeasurementLandmark(const Vector2d &expectedMeasurement, const Eigen::Matrix<double, 2, dim> &measurementModelJacobian, const double &weight)
    : measurement(expectedMeasurement), weight(weight)
  {
    this->measurementModelJacobian = &measurementModelJacobian;
  }
};

template<int dim>
class ExpectedMeasurementInfiniteLine
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector3d measurement;
  Eigen::Matrix<double, 3, dim>* measurementModelJacobian;
  double weight;

  ExpectedMeasurementInfiniteLine() : measurement(Vector3d::Zero()), measurementModelJacobian(0), weight() {}
  ExpectedMeasurementInfiniteLine(const Vector3d &expectedMeasurement, const Eigen::Matrix<double, 3, dim> &measurementModelJacobian, const double &weight)
    : measurement(expectedMeasurement), weight(weight)
  {
    this->measurementModelJacobian = &measurementModelJacobian;
  }
};

template<int stateDim, int nDim>
class KalmanStateUpdateObservations2016
{
private:
  Eigen::Matrix<double, nDim, stateDim> measurementModelJacobian_H;
  Eigen::Matrix<double, stateDim, nDim> measurementModelJacobian_H_transposed;
  Eigen::Matrix<double, nDim, nDim> innovationCovariance;
  Eigen::Matrix<double, nDim, nDim> measurementCovariance;
  Eigen::Matrix<double, stateDim, nDim> kalmanGain;
  Eigen::Matrix<double, nDim, 1> expectedMeasurement;
  Eigen::Matrix<double, nDim, 1> tempVector;
  Eigen::Matrix<double, stateDim, stateDim> tempMatrixOfFullDimension;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KalmanStateUpdateObservations2016() :
    measurementModelJacobian_H(Eigen::Matrix<double, nDim, stateDim>::Zero()),
    measurementModelJacobian_H_transposed(Eigen::Matrix<double, stateDim, nDim>::Zero()),
    innovationCovariance(Eigen::Matrix<double, nDim, nDim>::Zero()),
    measurementCovariance(Eigen::Matrix<double, nDim, nDim>::Zero()),
    kalmanGain(Eigen::Matrix<double, stateDim, nDim>::Zero()),
    expectedMeasurement(Eigen::Matrix<double, nDim, 1>::Zero()),
    tempVector(Eigen::Matrix<double, nDim, 1>::Zero()),
    tempMatrixOfFullDimension(Eigen::Matrix<double, stateDim, stateDim>::Zero()) {}

  void initMeasurementCovariance(const Matrix2d & singleMeasurementCovariance, double correlationFactorBetweenMeasurements)
  {
    const int n = nDim;
    for (int row = 0; row < n; row++)
    {
      for (int col = 0; col < n; col++)
      {
        double factor = 1;
        if (row / 2 != col / 2)
        {
          factor = correlationFactorBetweenMeasurements;
        }
        measurementCovariance(row, col) = factor * singleMeasurementCovariance(row % 2, col % 2);
      }
    }
  }

  Eigen::Matrix<double, stateDim, 1> updateWithLocalObservations(
    const std::vector<AngleObservation> & observationsAsAngles,
    const std::vector<ExpectedMeasurementLandmark<stateDim> > & expectedAngles,
    Eigen::Matrix<double, stateDim, stateDim> & covariance)
  {
    Eigen::Matrix<double, stateDim, 1> movement;
    const int n = 2 * static_cast<int>(observationsAsAngles.size());

    expectedMeasurement = Eigen::Matrix<double, nDim, 1>::Zero();
    tempVector = Eigen::Matrix<double, nDim, 1>::Zero();
    measurementModelJacobian_H = Eigen::Matrix<double, nDim, stateDim>::Zero();
    measurementModelJacobian_H_transposed = Eigen::Matrix<double, stateDim, nDim>::Zero();
    innovationCovariance = Eigen::Matrix<double, nDim, nDim>::Zero();
    kalmanGain = Eigen::Matrix<double, stateDim, nDim>::Zero();
    tempMatrixOfFullDimension = Eigen::Matrix<double, stateDim, stateDim>::Zero();

    for (int row = 0; row < n; row += 2)
    {
      const int landmarkIndex = row / 2;
      expectedMeasurement[row] = expectedAngles[landmarkIndex].measurement.x();
      expectedMeasurement[row + 1] = expectedAngles[landmarkIndex].measurement.y();
      tempVector[row] = observationsAsAngles[landmarkIndex].observation.x();
      tempVector[row + 1] = observationsAsAngles[landmarkIndex].observation.y();
      tempVector[row] = (tempVector[row] - expectedMeasurement[row]) * expectedAngles[landmarkIndex].weight;
      tempVector[row + 1] = (tempVector[row + 1] - expectedMeasurement[row + 1]) * expectedAngles[landmarkIndex].weight;
      measurementModelJacobian_H.block(row, 0, 2, stateDim) = expectedAngles[landmarkIndex].measurementModelJacobian->block(0, 0, 2, stateDim);
    }
    measurementModelJacobian_H_transposed.block(0,0,stateDim,n) = measurementModelJacobian_H.block(0,0,n,stateDim).transpose();
    //tempVector = tempVector - expectedMeasurement;
    innovationCovariance.block(0,0,n,n) = measurementModelJacobian_H.block(0,0,n,stateDim)*covariance*measurementModelJacobian_H_transposed.block(0,0,stateDim,n);
    innovationCovariance.block(0,0,n,n) += measurementCovariance.block(0,0,n,n);
    kalmanGain.block(0,0,stateDim,n) = covariance*(measurementModelJacobian_H_transposed.block(0,0,stateDim,n)*innovationCovariance.block(0,0,n,n).inverse());
    movement = kalmanGain.block(0,0,stateDim,n)*tempVector.block(0,0,n,1);
    measurementModelJacobian_H.block(0, 0, n, stateDim) *= -1.0;
    tempMatrixOfFullDimension = kalmanGain.block(0, 0, stateDim, n)*measurementModelJacobian_H.block(0,0,n, stateDim);
    for (int i = 0; i < stateDim; i++)
    {
      tempMatrixOfFullDimension(i,i) += 1.0;
    }
    covariance = tempMatrixOfFullDimension * covariance;
    
    return movement;
  }
};

template<int stateDim, int nDim>
class KalmanStateUpdateInfiniteLines2016
{
private:
  Eigen::Matrix<double, nDim, stateDim> infiniteLineMeasurementModelJacobian_H;
  Eigen::Matrix<double, stateDim, nDim> infiniteLineMeasurementModelJacobian_H_transposed;
  Eigen::Matrix<double, nDim, nDim> infiniteLineInnovationCovariance;
  Eigen::Matrix<double, nDim, nDim> infiniteLineMeasurementCovariance;
  Eigen::Matrix<double, stateDim, nDim> infiniteLineKalmanGain;
  Eigen::Matrix<double, nDim, 1> infiniteLineExpectedMeasurement;
  Eigen::Matrix<double, nDim, 1> infiniteLineTempVector;
  Eigen::Matrix<double, stateDim, stateDim> tempMatrixOfFullDimension;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KalmanStateUpdateInfiniteLines2016() :
    infiniteLineMeasurementModelJacobian_H(Eigen::Matrix<double, nDim, stateDim>::Zero()),
    infiniteLineMeasurementModelJacobian_H_transposed(Eigen::Matrix<double, stateDim, nDim>::Zero()),
    infiniteLineInnovationCovariance(Eigen::Matrix<double, nDim, nDim>::Zero()),
    infiniteLineMeasurementCovariance(Eigen::Matrix<double, nDim, nDim>::Zero()),
    infiniteLineKalmanGain(Eigen::Matrix<double, stateDim, nDim>::Zero()),
    infiniteLineExpectedMeasurement(Eigen::Matrix<double, nDim, 1>::Zero()),
    infiniteLineTempVector(Eigen::Matrix<double, nDim, 1>::Zero()),
    tempMatrixOfFullDimension(Eigen::Matrix<double, stateDim, stateDim>::Zero()) {}

  void initMeasurementCovarianceInfiniteLine(const Eigen::Matrix<double, stateDim, stateDim> & singleInfiniteLineMeasurementCovariance,
    double correlationFactorBetweenMeasurements)
  {
    const int n = nDim;
    for (int row = 0; row < n; row++)
    {
      for (int col = 0; col < n; col++)
      {
        double factor = 1;
        if (row / stateDim != col / stateDim)
        {
          factor = correlationFactorBetweenMeasurements;
        }
        infiniteLineMeasurementCovariance(row, col) = factor * singleInfiniteLineMeasurementCovariance(row % stateDim, col % stateDim);
      }
    }
  }

  Eigen::Matrix<double, stateDim, 1> updateWithLocalInfiniteLineObservations(
    const std::vector<NormalObservation> & observationsAsNormal,
    const std::vector<ExpectedMeasurementInfiniteLine<3> > & expectedMeasurements,
    Eigen::Matrix<double, stateDim, stateDim> & covariance)
  {
    Eigen::Matrix<double, stateDim, 1> movement;
    const int n = stateDim * static_cast<int>(observationsAsNormal.size());

    infiniteLineExpectedMeasurement = Eigen::Matrix<double, nDim, 1>::Zero();
    infiniteLineTempVector = Eigen::Matrix<double, nDim, 1>::Zero();
    infiniteLineMeasurementModelJacobian_H = Eigen::Matrix<double, nDim, stateDim>::Zero();
    infiniteLineMeasurementModelJacobian_H_transposed = Eigen::Matrix<double, stateDim, nDim>::Zero();
    infiniteLineInnovationCovariance = Eigen::Matrix<double, nDim, nDim>::Zero();
    infiniteLineKalmanGain = Eigen::Matrix<double, stateDim, nDim>::Zero();
    tempMatrixOfFullDimension = Eigen::Matrix<double, stateDim, stateDim>::Zero();

    for (int row = 0; row<n; row += stateDim)
    {
      const int landmarkIndex = row / stateDim;
      infiniteLineExpectedMeasurement[row] = expectedMeasurements[landmarkIndex].measurement.x();
      infiniteLineExpectedMeasurement[row + 1] = expectedMeasurements[landmarkIndex].measurement.y();
      infiniteLineExpectedMeasurement[row + 2] = expectedMeasurements[landmarkIndex].measurement.z();
      if (expectedMeasurements[landmarkIndex].measurement.dot(observationsAsNormal[landmarkIndex].observation) > 0)
      {
        infiniteLineTempVector[row] = observationsAsNormal[landmarkIndex].observation.x();
        infiniteLineTempVector[row + 1] = observationsAsNormal[landmarkIndex].observation.y();
        infiniteLineTempVector[row + 2] = observationsAsNormal[landmarkIndex].observation.z();
      }
      else
      {
        infiniteLineTempVector[row] = -observationsAsNormal[landmarkIndex].observation.x();
        infiniteLineTempVector[row + 1] = -observationsAsNormal[landmarkIndex].observation.y();
        infiniteLineTempVector[row + 2] = -observationsAsNormal[landmarkIndex].observation.z();
      }
      infiniteLineTempVector[row] = (infiniteLineTempVector[row] - infiniteLineExpectedMeasurement[row]) * expectedMeasurements[landmarkIndex].weight;
      infiniteLineTempVector[row + 1] = (infiniteLineTempVector[row + 1] - infiniteLineExpectedMeasurement[row + 1]) * expectedMeasurements[landmarkIndex].weight;
      infiniteLineTempVector[row + 2] = (infiniteLineTempVector[row + 2] - infiniteLineExpectedMeasurement[row + 2]) * expectedMeasurements[landmarkIndex].weight;
      infiniteLineMeasurementModelJacobian_H.block(row, 0, 3, stateDim) = expectedMeasurements[landmarkIndex].measurementModelJacobian->block(0, 0, 3, stateDim);
    }
    infiniteLineMeasurementModelJacobian_H_transposed.block(0,0,stateDim,n) = infiniteLineMeasurementModelJacobian_H.block(0, 0, n, stateDim).transpose();
    //infiniteLineTempVector = infiniteLineTempVector - infiniteLineExpectedMeasurement;
    infiniteLineInnovationCovariance.block(0,0,n,n) = infiniteLineMeasurementModelJacobian_H.block(0,0,n,stateDim) * covariance * infiniteLineMeasurementModelJacobian_H_transposed.block(0,0,stateDim,n);
    infiniteLineInnovationCovariance.block(0,0,n,n) += infiniteLineMeasurementCovariance.block(0,0,n,n);
    infiniteLineKalmanGain.block(0,0,stateDim,n) = covariance*(infiniteLineMeasurementModelJacobian_H_transposed.block(0,0,stateDim,n)*infiniteLineInnovationCovariance.block(0, 0, n, n).inverse());
    movement.block(0,0,stateDim,1) = infiniteLineKalmanGain.block(0,0,stateDim,n)*infiniteLineTempVector.block(0,0,n,1);
    infiniteLineMeasurementModelJacobian_H.block(0,0,n,stateDim) *= -1.0;
    tempMatrixOfFullDimension = infiniteLineKalmanGain.block(0,0,stateDim, n) * infiniteLineMeasurementModelJacobian_H.block(0,0,n, stateDim);
    for (int i = 0; i < stateDim; i++)
    {
      tempMatrixOfFullDimension(i,i) += 1.0;
    }
    covariance = tempMatrixOfFullDimension * covariance;
    return movement;
  }
};

