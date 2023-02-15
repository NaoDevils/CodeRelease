/**
* @file LineMatchingResult.cpp
* Implementation of a class that represents the line matching result.
* @author <a href="mailto:stefan.tasse@tu-dortmund.de">Stefan Tasse</a>
*/


#include "LineMatchingResult.h"

void LineMatchingResult::calculateObservationsSphericalCoords()
{
  for (std::vector<FieldLine>::const_iterator i = observations.begin(); i != observations.end(); ++i)
  {
    observationsSphericalCoords.push_back(getSphericalCoordinatesForPointObservation(*i));
  }
}

double LineMatchingResult::calculateMeasurementLikelihoodSphericalCoordinates(const PoseHypothesis& poseHypothesis, const Matrix2d measurementCovariance_inv) const
{
  double likelihood = 1;
  for (unsigned int i = 0; i < observationsSphericalCoords.size(); i++)
  {
    FieldLine correspondingSegment(projectPointToFieldLine(poseHypothesis.pose, observations[i].start, fieldLines[poseHypothesis.lineCorrespondences[i]]),
        projectPointToFieldLine(poseHypothesis.pose, observations[i].end, fieldLines[poseHypothesis.lineCorrespondences[i]]),
        observations[i].cameraHeight);
    FieldLine correspondingSegmentsSphericalCoords = getSphericalCoordinatesForPointObservation(poseHypothesis.pose, correspondingSegment);
    Vector2d diff = observationsSphericalCoords[i].start - correspondingSegmentsSphericalCoords.start;
    likelihood *= exp(-(diff.dot(measurementCovariance_inv * diff)));
    diff = observationsSphericalCoords[i].end - correspondingSegmentsSphericalCoords.end;
    likelihood *= exp(-(diff.dot(measurementCovariance_inv * diff)));
  }
  return likelihood;
}


bool LineMatchingResult::getCorrespondencesForLocalizationHypothesis(const Pose2f& localizationHypothesis,
    const Matrix3d& poseCovariance,
    double likelihoodThreshold,
    const Matrix2d& sphericalPointMeasurementCovariance_inv,
    std::vector<FieldLine>& correspondencesForObservations,
    bool displayWarning,
    bool requestedByLocalization) const
{
  Matrix3d poseCovariance_inv = poseCovariance.inverse();
  correspondencesForObservations.clear();
  Vector3d targetPose(localizationHypothesis.translation.x(), localizationHypothesis.translation.y(), localizationHypothesis.rotation);

  double poseLikelihood, poseLikelihoodOfBestFit = -1, measurementLikelihood, bestMeasurementLikelihood = -1;
  PoseHypothesis bestFit;
  PoseHypothesis fitTarget;
  fitTarget.pose = localizationHypothesis;

  if (poseHypothesis.size() > 0)
  {
    // find closest pose
    for (std::vector<PoseHypothesis>::const_iterator i = poseHypothesis.begin(); i != poseHypothesis.end(); ++i)
    {
      Vector3d testPose(i->pose.translation.x(), i->pose.translation.y(), i->pose.rotation);
      Vector3d diff = targetPose - testPose;
      diff.z() = Angle::normalize(diff.z()); // it's an angle!
      poseLikelihood = exp(-(diff.dot(poseCovariance_inv * diff)));
      if (poseLikelihood > likelihoodThreshold)
      {
        fitTarget.setLineCorrespondences(i->lineCorrespondences);
        measurementLikelihood = calculateMeasurementLikelihoodSphericalCoordinates(fitTarget, sphericalPointMeasurementCovariance_inv);

        if (requestedByLocalization)
        {
          POSE_2D_SAMPLE("representation:LineMatchingResult:requestedCorrespondences_testedCandidates", i->pose, ColorRGBA(150, 150, 255));
        }

        if (measurementLikelihood > bestMeasurementLikelihood)
        {
          bestMeasurementLikelihood = measurementLikelihood;
          poseLikelihoodOfBestFit = poseLikelihood;
          bestFit = (*i);
        }
      }
    }
  }
  else if (poseHypothesisIntervals.size() > 0)
  {
    // find closest pose interval
    for (std::vector<PoseHypothesisInterval>::const_iterator i = poseHypothesisIntervals.begin(); i != poseHypothesisIntervals.end(); ++i)
    {
      // Note: In each pose interval, the rotation and either x or y are constant.
      Vector3d testPose(0, 0, i->start.rotation);
      // project pose.translation onto the line of positions given by the pose interval
      Vector2d temp = projectPointToFieldLine(localizationHypothesis.translation.cast<double>(), FieldLine(i->start.translation.cast<double>(), i->end.translation.cast<double>(), 0));
      testPose.x() = temp.x();
      testPose.y() = temp.y();
      Vector3d diff = targetPose - testPose;
      diff.z() = Angle::normalize(diff.z()); // it's an angle!
      poseLikelihood = exp(-(diff.dot(poseCovariance_inv * diff)));
      if (poseLikelihood > likelihoodThreshold)
      {
        fitTarget.setLineCorrespondences(i->lineCorrespondences);
        measurementLikelihood = calculateMeasurementLikelihoodSphericalCoordinates(fitTarget, sphericalPointMeasurementCovariance_inv);


        if (requestedByLocalization)
        {
          POSE_2D_SAMPLE("representation:LineMatchingResult:requestedCorrespondences_testedCandidates",
              Pose2f(static_cast<float>(testPose.z()), static_cast<float>(testPose.x()), static_cast<float>(testPose.y())),
              ColorRGBA(150, 150, 255));
        }

        if (measurementLikelihood > bestMeasurementLikelihood)
        {
          bestMeasurementLikelihood = measurementLikelihood;
          poseLikelihoodOfBestFit = poseLikelihood;
          bestFit.pose.rotation = static_cast<float>(testPose.z());
          bestFit.pose.translation.x() = static_cast<float>(testPose.x());
          bestFit.pose.translation.y() = static_cast<float>(testPose.y());
          bestFit.setLineCorrespondences(i->lineCorrespondences);
        }
      }
    }
  }
  // else: there were no line observations or matching them didn't work!


  if (poseLikelihoodOfBestFit >= likelihoodThreshold)
  {
    // generate corresponding line segments as seen from the bestFit pose
    for (unsigned int i = 0; i < observations.size(); i++)
    {
      Vector2d start = projectPointToFieldLine(bestFit.pose, observations[i].start, fieldLines[bestFit.lineCorrespondences[i]]);
      Vector2d end = projectPointToFieldLine(bestFit.pose, observations[i].end, fieldLines[bestFit.lineCorrespondences[i]]);

      if (start != end)
        correspondencesForObservations.push_back(FieldLine(start, end, observations[i].cameraHeight));
      else if (displayWarning)
        OUTPUT_WARNING("LineMatchingResult - Projection of observed line does not match corresponding field line of best fitting pose");
    }
  }

  return !correspondencesForObservations.empty();
}

Vector2d LineMatchingResult::projectPointToFieldLine(const Pose2f& pose, const Vector2d& pointInRelativeCoords, const LineMatchingResult::FieldLine& line) const
{
  Vector2d pointInAbsoluteCoords = (pose * pointInRelativeCoords.cast<float>()).cast<double>();
  return projectPointToFieldLine(pointInAbsoluteCoords, line);
}

Vector2d LineMatchingResult::projectPointToFieldLine(const Vector2d& pointInAbsoluteCoords, const LineMatchingResult::FieldLine& line) const
{
  Vector2d direction = line.end - line.start;
  double length = direction.norm();
  direction.normalize();
  double t = direction.dot(pointInAbsoluteCoords - line.start);
  t = std::min(t, length);
  t = std::max(t, (double)0.0);
  return line.start + (direction * t);
}

void LineMatchingResult::drawCorrespondences(const Pose2f& pose) const
{
  std::vector<FieldLine> correspondencesForObservations;
  Matrix3d cov; // initializes to identity
  Matrix2d measurementCov; // initializes to identity
  const ColorRGBA pose2lineObs(0, 255, 255, 128);
  const ColorRGBA lineObs(0, 255, 255, 255);
  const ColorRGBA correspondence(255, 0, 255, 128);
  const ColorRGBA lineObs2correspondence(255, 0, 255, 128);
  const int pose2lineObsWidth = 30;
  const int lineObsWidth = 50;
  const int correspondenceWidth = 50;
  const int lineObs2correspondenceWidth = 30;
  if (getCorrespondencesForLocalizationHypothesis(pose, cov, 0, measurementCov, correspondencesForObservations, false))
  {
    for (unsigned int j = 0; j < observations.size(); j++)
    {
      Vector2f obsStartInFieldCoords(observations[j].start.cast<float>());
      obsStartInFieldCoords = pose * obsStartInFieldCoords;
      Vector2f obsEndInFieldCoords(observations[j].end.cast<float>());
      obsEndInFieldCoords = pose * obsEndInFieldCoords;

      LINE("representation:LineMatchingResult:correspondences", pose.translation.x(), pose.translation.y(), obsStartInFieldCoords.x(), obsStartInFieldCoords.y(), pose2lineObsWidth, Drawings::dashedPen, pose2lineObs);
      LINE("representation:LineMatchingResult:correspondences", pose.translation.x(), pose.translation.y(), obsEndInFieldCoords.x(), obsEndInFieldCoords.y(), pose2lineObsWidth, Drawings::dashedPen, pose2lineObs);

      LINE("representation:LineMatchingResult:correspondences", obsStartInFieldCoords.x(), obsStartInFieldCoords.y(), obsEndInFieldCoords.x(), obsEndInFieldCoords.y(), lineObsWidth, Drawings::solidPen, lineObs);
      LINE("representation:LineMatchingResult:correspondences",
          correspondencesForObservations[j].start.x(),
          correspondencesForObservations[j].start.y(),
          correspondencesForObservations[j].end.x(),
          correspondencesForObservations[j].end.y(),
          correspondenceWidth,
          Drawings::solidPen,
          correspondence);

      ARROW("representation:LineMatchingResult:correspondences",
          obsStartInFieldCoords.x(),
          obsStartInFieldCoords.y(),
          correspondencesForObservations[j].start.x(),
          correspondencesForObservations[j].start.y(),
          lineObs2correspondenceWidth,
          Drawings::solidPen,
          lineObs2correspondence);
      ARROW("representation:LineMatchingResult:correspondences",
          obsEndInFieldCoords.x(),
          obsEndInFieldCoords.y(),
          correspondencesForObservations[j].end.x(),
          correspondencesForObservations[j].end.y(),
          lineObs2correspondenceWidth,
          Drawings::solidPen,
          lineObs2correspondence);
    }
  }
  else
  {
    DRAWTEXT("representation:LineMatchingResult:correspondences", pose.translation.x() - 40, pose.translation.y() - 40, 100, ColorRGBA(255, 255, 255), "no correspondences found!");
  }
}
void LineMatchingResult::drawRequestedCorrespondences(const Pose2f& pose, const Matrix3d& cov, double likelihoodThreshold, const Matrix2d& measurementCov) const
{
  COMPLEX_DRAWING("representation:LineMatchingResult:requestedCorrespondences")
  {
    std::vector<FieldLine> correspondencesForObservations;
    const ColorRGBA pose2lineObs(0, 255, 255, 128);
    const ColorRGBA lineObs(0, 255, 255, 255);
    const ColorRGBA correspondence(255, 0, 255, 128);
    const ColorRGBA lineObs2correspondence(255, 0, 255, 128);
    const int pose2lineObsWidth = 30;
    const int lineObsWidth = 50;
    const int correspondenceWidth = 50;
    const int lineObs2correspondenceWidth = 30;
    if (getCorrespondencesForLocalizationHypothesis(pose, cov, likelihoodThreshold, measurementCov, correspondencesForObservations, false))
    {
      for (unsigned int j = 0; j < observations.size(); j++)
      {
        Vector2f obsStartInFieldCoords(observations[j].start.cast<float>());
        obsStartInFieldCoords = pose * obsStartInFieldCoords;
        Vector2f obsEndInFieldCoords(observations[j].end.cast<float>());
        obsEndInFieldCoords = pose * obsEndInFieldCoords;

        LINE("representation:LineMatchingResult:requestedCorrespondences", pose.translation.x(), pose.translation.y(), obsStartInFieldCoords.x(), obsStartInFieldCoords.y(), pose2lineObsWidth, Drawings::dashedPen, pose2lineObs);
        LINE("representation:LineMatchingResult:requestedCorrespondences", pose.translation.x(), pose.translation.y(), obsEndInFieldCoords.x(), obsEndInFieldCoords.y(), pose2lineObsWidth, Drawings::dashedPen, pose2lineObs);

        LINE("representation:LineMatchingResult:requestedCorrespondences", obsStartInFieldCoords.x(), obsStartInFieldCoords.y(), obsEndInFieldCoords.x(), obsEndInFieldCoords.y(), lineObsWidth, Drawings::solidPen, lineObs);
        LINE("representation:LineMatchingResult:requestedCorrespondences",
            correspondencesForObservations[j].start.x(),
            correspondencesForObservations[j].start.y(),
            correspondencesForObservations[j].end.x(),
            correspondencesForObservations[j].end.y(),
            correspondenceWidth,
            Drawings::solidPen,
            correspondence);

        ARROW("representation:LineMatchingResult:requestedCorrespondences",
            obsStartInFieldCoords.x(),
            obsStartInFieldCoords.y(),
            correspondencesForObservations[j].start.x(),
            correspondencesForObservations[j].start.y(),
            lineObs2correspondenceWidth,
            Drawings::solidPen,
            lineObs2correspondence);
        ARROW("representation:LineMatchingResult:requestedCorrespondences",
            obsEndInFieldCoords.x(),
            obsEndInFieldCoords.y(),
            correspondencesForObservations[j].end.x(),
            correspondencesForObservations[j].end.y(),
            lineObs2correspondenceWidth,
            Drawings::solidPen,
            lineObs2correspondence);
      }
    }
    else
    {
      DRAWTEXT("representation:LineMatchingResult:requestedCorrespondences", pose.translation.x() - 40, pose.translation.y() - 40, 100, ColorRGBA(255, 255, 255), "no correspondences found!");
    }
  }
}

void LineMatchingResult::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:LineMatchingResult:poses", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:LineMatchingResult:correspondences", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:LineMatchingResult:requestedCorrespondences", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:LineMatchingResult:requestedCorrespondences_testedCandidates", "drawingOnField");

  int counter = 0;
  for (std::vector<PoseHypothesis>::const_iterator i = poseHypothesis.begin(); i != poseHypothesis.end(); ++i)
  {
    POSE_2D_SAMPLE("representation:LineMatchingResult:poses", (i->pose), ColorRGBA(255, 0, 0));
    DRAWTEXT("representation:LineMatchingResult:poses", i->pose.translation.x() + 40, i->pose.translation.y() + 40, 100, ColorRGBA(255, 0, 0), counter);
    COMPLEX_DRAWING("representation:LineMatchingResult:correspondences") drawCorrespondences(i->pose);
    counter++;
  }
  for (std::vector<PoseHypothesisInterval>::const_iterator i = poseHypothesisIntervals.begin(); i != poseHypothesisIntervals.end(); ++i)
  {
    for (float t = 0.f; t <= 1.f; t += 1.f / 25)
    {
      float s = 1.f - t;
      Pose2f interpolation(
          t * i->start.rotation + s * i->end.rotation, t * i->start.translation.x() + s * i->end.translation.x(), t * i->start.translation.y() + s * i->end.translation.y());
      POSE_2D_SAMPLE("representation:LineMatchingResult:poses", interpolation, ColorRGBA(255, 0, 0));
    }
    COMPLEX_DRAWING("representation:LineMatchingResult:correspondences") drawCorrespondences(i->start);
    COMPLEX_DRAWING("representation:LineMatchingResult:correspondences") drawCorrespondences(i->end);
    DRAWTEXT("representation:LineMatchingResult:poses", i->start.translation.x() + 40, i->start.translation.y() + 40, 100, ColorRGBA(255, 0, 0), counter);
    counter++;
  }
}
