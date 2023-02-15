/**
 * @file RobotPoseHypotheses.h
 *
 * The file contains the definition of the class RobotPoseHypotheses.
 *
 * @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
 * @author <a href="mailto:Stefan.Czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/Eigen.h"


/**
* @class RobotPoseHypothesis
* A single hypothesis about a robot position
*/
STREAMABLE_WITH_BASE(RobotPoseHypothesis, RobotPose,

  RobotPoseHypothesis() {};

  RobotPoseHypothesis(const Pose2f & pose, const Matrix3d & covariance)
    : covariance(covariance)
  {
    translation = pose.translation;
    rotation = pose.rotation;
  }

  /*
  bool derivesFrom(int indexFromPreviousTimeStep) const
  {
    bool result = false;
    for(unsigned int i=0; i < hypothesisOrigins.size() && result==false; ++i)
    {
      result = result || (indexFromPreviousTimeStep==hypothesisOrigins[i]);
    }
    return result;
  }
*/
  Matrix2d getPositionCovariance() const
  {
    return Matrix2d(covariance.block(0, 0, 2, 2));
  }
  ,
  /** Covariance matrix about the position/orientation uncertainty*/
  (Matrix3d)(Matrix3d::Identity()) covariance,

  /** From which former hypotheses this one was generated (empty when newly generated from template) */
    //  std::vector<int> hypothesisOrigins;
  (bool)(false) robotPoseReceivedMeasurementUpdate
);


/**
* @class RobotPoseHypotheses
* A set of hypotheses
*/
STREAMABLE(RobotPoseHypotheses,
    //  int indexOfBestHypothesis;
 
  /** Constructor*/
  RobotPoseHypotheses(){ hypotheses.push_back(RobotPoseHypothesis());}

  /** Draws the hypotheses to the field view*/
  void draw();
  ,

  (std::vector<RobotPoseHypothesis>) hypotheses /**< The list of hypotheses */
);

STREAMABLE(RobotPoseHypothesesCompressed,

  RobotPoseHypothesesCompressed() = default;
  RobotPoseHypothesesCompressed(const RobotPoseHypotheses &other)
  {
    for (const auto &item : other.hypotheses)
    {
      hypotheses.push_back(RobotPoseCompressed(item));
    }
  }

  operator RobotPoseHypotheses() const
  {
    RobotPoseHypotheses robotPoseHypotheses;
    robotPoseHypotheses.hypotheses.clear();

    for (auto &item : hypotheses)
    {
      RobotPoseHypothesis rph;
      static_cast<RobotPose&>(rph) = static_cast<RobotPose>(item);
      robotPoseHypotheses.hypotheses.push_back(rph);
    }

    return robotPoseHypotheses;
  },

  (std::vector<RobotPoseCompressed>) hypotheses

);
