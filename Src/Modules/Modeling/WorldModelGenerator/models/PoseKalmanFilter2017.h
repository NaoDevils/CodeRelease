/**
* @file PoseKalmanFilter2017.h
*
* @author <a href="mailto:stefan.tasse@tu-dortmund.de">Stefan Tasse</a>
* @author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
*/

#pragma once

#include "Tools/Math/Eigen.h"

template <int dim> class SphericalObservation
{
public:
  typedef Eigen::Matrix<double, 2, dim> JacobianMatrix;

  // The observed angles (is-angles)
  Vector2d realAngles;
  // The expected angles (should/ideal-angles)
  Vector2d nominalAngles;
  // The jacobian matrix (likelihood) of the observation
  JacobianMatrix measurementModelJacobian;
  // The impact of this measurement
  double weight;

  SphericalObservation() : realAngles(Vector2d::Zero()), nominalAngles(Vector2d::Zero()), measurementModelJacobian(JacobianMatrix::Zero()), weight(0) {}
};

template <int dim> class InfiniteLineObservation
{
public:
  typedef Eigen::Matrix<double, 3, dim> JacobianMatrix;

  // The observed angles (is-angles)
  Vector3d realNormals;
  // The expected angles (should/ideal-angles)
  Vector3d nominalNormals;
  // The jacobian matrix (likelihood) of the observation
  JacobianMatrix measurementModelJacobian;
  // The impact of this measurement
  double weight;

  InfiniteLineObservation() : realNormals(Vector3d::Zero()), nominalNormals(Vector3d::Zero()), measurementModelJacobian(JacobianMatrix::Zero()), weight(0) {}
};

template <int dim> using SphericalObservationVector = std::vector<SphericalObservation<dim>>;
template <int dim> using InfiniteLineObservationVector = std::vector<InfiniteLineObservation<dim>>;

template <int stateDim, int nDim> class KalmanStateUpdateObservations2017
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
  KalmanStateUpdateObservations2017()
      : measurementModelJacobian_H(Eigen::Matrix<double, nDim, stateDim>::Zero()), measurementModelJacobian_H_transposed(Eigen::Matrix<double, stateDim, nDim>::Zero()),
        innovationCovariance(Eigen::Matrix<double, nDim, nDim>::Zero()), measurementCovariance(Eigen::Matrix<double, nDim, nDim>::Zero()),
        kalmanGain(Eigen::Matrix<double, stateDim, nDim>::Zero()), expectedMeasurement(Eigen::Matrix<double, nDim, 1>::Zero()), tempVector(Eigen::Matrix<double, nDim, 1>::Zero()),
        tempMatrixOfFullDimension(Eigen::Matrix<double, stateDim, stateDim>::Zero())
  {
  }

  void initMeasurementCovariance(const Matrix2d& singleMeasurementCovariance, double correlationFactorBetweenMeasurements)
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

  Eigen::Matrix<double, stateDim, 1> updateWithLocalObservations(const SphericalObservationVector<stateDim>& sphericalObservations, Eigen::Matrix<double, stateDim, stateDim>& covariance)
  {
    Eigen::Matrix<double, stateDim, 1> movement;
    const size_t n = 2 * sphericalObservations.size();

    tempVector = Eigen::Matrix<double, nDim, 1>::Zero();
    measurementModelJacobian_H = Eigen::Matrix<double, nDim, stateDim>::Zero();
    measurementModelJacobian_H_transposed = Eigen::Matrix<double, stateDim, nDim>::Zero();
    innovationCovariance = Eigen::Matrix<double, nDim, nDim>::Zero();
    kalmanGain = Eigen::Matrix<double, stateDim, nDim>::Zero();
    tempMatrixOfFullDimension = Eigen::Matrix<double, stateDim, stateDim>::Zero();

    for (unsigned int landmarkIndex = 0; landmarkIndex < sphericalObservations.size(); landmarkIndex++)
    {
      const SphericalObservation<stateDim>& observation = sphericalObservations[landmarkIndex];
      const int row = 2 * landmarkIndex;
      tempVector[row] = (observation.realAngles.x() - observation.nominalAngles.x()) * observation.weight;
      tempVector[row + 1] = (observation.realAngles.y() - observation.nominalAngles.y()) * observation.weight;
      measurementModelJacobian_H.block(row, 0, 2, stateDim) = observation.measurementModelJacobian;
    }
    measurementModelJacobian_H_transposed.block(0, 0, stateDim, n) = measurementModelJacobian_H.block(0, 0, n, stateDim).transpose();
    innovationCovariance.block(0, 0, n, n) = measurementModelJacobian_H.block(0, 0, n, stateDim) * covariance * measurementModelJacobian_H_transposed.block(0, 0, stateDim, n);
    innovationCovariance.block(0, 0, n, n) += measurementCovariance.block(0, 0, n, n);
    kalmanGain.block(0, 0, stateDim, n) = covariance * (measurementModelJacobian_H_transposed.block(0, 0, stateDim, n) * innovationCovariance.block(0, 0, n, n).inverse());
    movement = kalmanGain.block(0, 0, stateDim, n) * tempVector.block(0, 0, n, 1);
    measurementModelJacobian_H.block(0, 0, n, stateDim) *= -1.0;
    tempMatrixOfFullDimension = kalmanGain.block(0, 0, stateDim, n) * measurementModelJacobian_H.block(0, 0, n, stateDim);
    for (int i = 0; i < stateDim; i++)
    {
      tempMatrixOfFullDimension(i, i) += 1.0;
    }
    covariance = tempMatrixOfFullDimension * covariance;

    return movement;
  }
};

template <int stateDim, int nDim> class KalmanStateUpdateInfiniteLines2017
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
  KalmanStateUpdateInfiniteLines2017()
      : infiniteLineMeasurementModelJacobian_H(Eigen::Matrix<double, nDim, stateDim>::Zero()),
        infiniteLineMeasurementModelJacobian_H_transposed(Eigen::Matrix<double, stateDim, nDim>::Zero()),
        infiniteLineInnovationCovariance(Eigen::Matrix<double, nDim, nDim>::Zero()), infiniteLineMeasurementCovariance(Eigen::Matrix<double, nDim, nDim>::Zero()),
        infiniteLineKalmanGain(Eigen::Matrix<double, stateDim, nDim>::Zero()), infiniteLineExpectedMeasurement(Eigen::Matrix<double, nDim, 1>::Zero()),
        infiniteLineTempVector(Eigen::Matrix<double, nDim, 1>::Zero()), tempMatrixOfFullDimension(Eigen::Matrix<double, stateDim, stateDim>::Zero())
  {
  }

  void initMeasurementCovarianceInfiniteLine(const Eigen::Matrix<double, stateDim, stateDim>& singleInfiniteLineMeasurementCovariance, double correlationFactorBetweenMeasurements)
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
      const InfiniteLineObservationVector<stateDim>& infiniteLineObservations, Eigen::Matrix<double, stateDim, stateDim>& covariance)
  {

    Eigen::Matrix<double, stateDim, 1> movement;
    const size_t n = stateDim * infiniteLineObservations.size();

    infiniteLineTempVector = Eigen::Matrix<double, nDim, 1>::Zero();
    infiniteLineMeasurementModelJacobian_H = Eigen::Matrix<double, nDim, stateDim>::Zero();
    infiniteLineMeasurementModelJacobian_H_transposed = Eigen::Matrix<double, stateDim, nDim>::Zero();
    infiniteLineInnovationCovariance = Eigen::Matrix<double, nDim, nDim>::Zero();
    infiniteLineKalmanGain = Eigen::Matrix<double, stateDim, nDim>::Zero();
    tempMatrixOfFullDimension = Eigen::Matrix<double, stateDim, stateDim>::Zero();

    for (unsigned int index = 0; index < infiniteLineObservations.size(); index++)
    {
      const InfiniteLineObservation<stateDim>& observation = infiniteLineObservations[index];
      const int row = 3 * index;
      int sign = (observation.nominalNormals.dot(observation.realNormals) > 0) ? 1 : -1;
      infiniteLineTempVector[row] = ((sign * observation.realNormals.x()) - observation.nominalNormals.x()) * observation.weight;
      infiniteLineTempVector[row + 1] = ((sign * observation.realNormals.y()) - observation.nominalNormals.y()) * observation.weight;
      infiniteLineTempVector[row + 2] = ((sign * observation.realNormals.z()) - observation.nominalNormals.z()) * observation.weight;
      infiniteLineMeasurementModelJacobian_H.block(row, 0, 3, stateDim) = observation.measurementModelJacobian;
    }
    infiniteLineMeasurementModelJacobian_H_transposed.block(0, 0, stateDim, n) = infiniteLineMeasurementModelJacobian_H.block(0, 0, n, stateDim).transpose();
    infiniteLineInnovationCovariance.block(0, 0, n, n) = infiniteLineMeasurementModelJacobian_H.block(0, 0, n, stateDim) * covariance
        * infiniteLineMeasurementModelJacobian_H_transposed.block(0, 0, stateDim, n);
    infiniteLineInnovationCovariance.block(0, 0, n, n) += infiniteLineMeasurementCovariance.block(0, 0, n, n);
    infiniteLineKalmanGain.block(0, 0, stateDim, n) = covariance
        * (infiniteLineMeasurementModelJacobian_H_transposed.block(0, 0, stateDim, n) * infiniteLineInnovationCovariance.block(0, 0, n, n).inverse());
    movement.block(0, 0, stateDim, 1) = infiniteLineKalmanGain.block(0, 0, stateDim, n) * infiniteLineTempVector.block(0, 0, n, 1);
    infiniteLineMeasurementModelJacobian_H.block(0, 0, n, stateDim) *= -1.0;
    tempMatrixOfFullDimension = infiniteLineKalmanGain.block(0, 0, stateDim, n) * infiniteLineMeasurementModelJacobian_H.block(0, 0, n, stateDim);
    for (int i = 0; i < stateDim; i++)
    {
      tempMatrixOfFullDimension(i, i) += 1.0;
    }
    covariance = tempMatrixOfFullDimension * covariance;
    return movement;
  }
};
