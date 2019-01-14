/**
 * @file RobotPoseHypotheses.h
 *
 * The file contains the definition of the class RobotPoseHypotheses.
 *
 * @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
 * @author <a href="mailto:Stefan.Czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
 */

#pragma once

#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/Eigen.h"


/**
* @class RobotPoseHypothesis
* A single hypothesis about a robot position
*/
class RobotPoseHypothesis: public RobotPose
{
public:
  RobotPoseHypothesis() : robotPoseReceivedMeasurementUpdate(false) { covariance = Matrix3d::Identity(); };

  RobotPoseHypothesis(const Pose2f & pose, const Matrix3d & covariance):
    covariance(covariance)
  {
    translation = pose.translation;
    rotation = pose.rotation;
  }

private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
      STREAM_BASE(RobotPose)
      STREAM(covariance)
      STREAM(robotPoseReceivedMeasurementUpdate)
    STREAM_REGISTER_FINISH;
  }

public:
  /** Covariance matrix about the position/orientation uncertainty*/
  Matrix3d covariance;

  /** From which former hypotheses this one was generated (empty when newly generated from template) */
    //  std::vector<int> hypothesisOrigins;
  bool robotPoseReceivedMeasurementUpdate;

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

  /** Assignment operator
  * @param other Another RobotPoseHypothesis
  * @return A reference to the object after the assignment
  */
  const RobotPoseHypothesis& operator=(const RobotPoseHypothesis& other)
  {
    (RobotPose&) *this = (const RobotPose&) other;
    covariance = other.covariance;
    //    hypothesisOrigins = other.hypothesisOrigins;
    robotPoseReceivedMeasurementUpdate = other.robotPoseReceivedMeasurementUpdate;
    return *this;
  }
};


/**
* @class RobotPoseHypotheses
* A set of hypotheses
*/
class RobotPoseHypotheses: public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
      STREAM(hypotheses);
    STREAM_REGISTER_FINISH;
  }

public:
  std::vector<RobotPoseHypothesis> hypotheses;   /**< The list of hypotheses */

    //  int indexOfBestHypothesis;
 
  /** Constructor*/
  RobotPoseHypotheses(){hypotheses.push_back(RobotPoseHypothesis());}

  /** Assignment operator
  * @param other Another RobotPoseHypotheses
  * @return A reference to the object after the assignment
  */
  RobotPoseHypotheses& operator=(const RobotPoseHypotheses& other)
  {
    hypotheses = other.hypotheses;
    //    indexOfBestHypothesis = other.indexOfBestHypothesis;
    return *this;
  }

  /** Draws the hypotheses to the field view*/
  void draw();
};

STREAMABLE(RobotPoseHypothesesCompressed,
{
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
      ((RobotPose&)rph) = item;
      robotPoseHypotheses.hypotheses.push_back(rph);
    }

    return robotPoseHypotheses;
  },

  (std::vector<RobotPoseCompressed>) hypotheses,

});
