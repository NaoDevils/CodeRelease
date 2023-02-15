#pragma once

// Extend the Eigen classes with our own methods (see: http://eigen.tuxfamily.org/dox-devel/TopicCustomizingEigen.html)
#define EIGEN_MATRIXBASE_PLUGIN "Tools/Math/EigenMatrixBaseExtensions.h"

#include <Eigen/Dense>

using Vector2f = Eigen::Vector2f;
using Vector3f = Eigen::Vector3f;
using Vector4f = Eigen::Vector4f;

using Matrix2f = Eigen::Matrix2f;
using Matrix3f = Eigen::Matrix3f;
using Matrix4f = Eigen::Matrix4f;

using Quaternionf = Eigen::Quaternionf;
using AngleAxisf = Eigen::AngleAxisf;
