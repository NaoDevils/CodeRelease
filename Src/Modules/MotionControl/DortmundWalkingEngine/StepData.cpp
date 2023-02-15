/**
* @file StepData.cpp
* @author <a href="mailto:oliver.urbann@tu-dortmund.de"> Oliver Urbann</a>
*/

#include "StepData.h"
#include "Tools/Math/Bspline.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
using namespace DWE;

const int ZMP::phaseToZMPFootMap[] = {LEFT_FOOT, RIGHT_FOOT, RIGHT_FOOT, LEFT_FOOT, LEFT_FOOT, LEFT_FOOT, LEFT_FOOT, RIGHT_FOOT};

void CustomStepsFile::mirror()
{
  ballOffset.y() *= -1.f;
  kickAngle *= -1.f;
  for (CustomStep& step : steps)
    step.mirror();
}

bool CustomStepsFile::isApplicable(float distance)
{
  return kickDistance[0] <= distance && distance < kickDistance[1];
}

void CustomStep::mirror()
{
  {
    Pose2f tmp = footPos[RIGHT_FOOT];
    footPos[RIGHT_FOOT] = footPos[LEFT_FOOT];
    footPos[LEFT_FOOT] = tmp;
    footPos[RIGHT_FOOT].rotation *= -1.f;
    footPos[RIGHT_FOOT].translation.y() *= -1.f;
    footPos[LEFT_FOOT].rotation *= -1.f;
    footPos[LEFT_FOOT].translation.y() *= -1.f;
  }
  {
    bool tmp = onFloor[RIGHT_FOOT];
    onFloor[RIGHT_FOOT] = onFloor[LEFT_FOOT];
    onFloor[LEFT_FOOT] = tmp;
  }
  for (Vector3f& vec : swingFootTraj)
    vec.y() *= -1.f;
  for (Point& point : spline)
    point.y *= -1.f;
  mirrored = !mirrored;
}

void CustomStep::serialize(In* in, Out* out)
{
  int duration = this->duration;

  if (out)
    duration *= Global::getSettings().naoVersion == RobotConfig::V6 ? 12 : 10;

  STREAM_REGISTER_BEGIN;
  STREAM(footPos);
  STREAM(duration);
  STREAM(onFloor);
  STREAM(kick);
  STREAM(swingFootTraj);
  STREAM_REGISTER_FINISH;
  if (in)
  {
    this->duration = std::max(1, duration / (Global::getSettings().naoVersion == RobotConfig::V6 ? 12 : 10));
    int vectorSize = static_cast<int>(swingFootTraj.size());
    std::vector<Point> control(vectorSize);
    spline.resize(this->duration);
    if (vectorSize > 0 && swingFootTraj[0].x() == swingFootTraj[0].x() && swingFootTraj[0].x() == 0 && swingFootTraj[0].y() == 0 && swingFootTraj[0].z() == 0
        && swingFootTraj[vectorSize - 1].x() == 0 && swingFootTraj[vectorSize - 1].y() == 0 && swingFootTraj[vectorSize - 1].z() == 0)
    {
      for (int i = 0; i < vectorSize; i++)
        control[i] = Point(swingFootTraj[i]);
      BSpline<Point>::bspline(vectorSize - 1, 3, &(control[0]), &(spline[0]), this->duration);
    }
    else
      for (int i = 0; i < this->duration; i++)
        spline[i] = Point();
  }
}

void CustomStepsFile::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(ballOffset);
  STREAM(kickAngle);
  STREAM(kickDistance);
  STREAM(translationThresholdXFront);
  STREAM(translationThresholdXBack);
  STREAM(translationThresholdY);
  STREAM(rotationThreshold);
  STREAM(timeUntilKickHackHip);
  STREAM(kickHackDurationHip);
  STREAM(kickHackHipAngle);
  STREAM(timeUntilKickHackKnee);
  STREAM(kickHackDurationKnee);
  STREAM(kickHackKneeAngle);
  STREAM(steps);
  STREAM_REGISTER_FINISH;
}
