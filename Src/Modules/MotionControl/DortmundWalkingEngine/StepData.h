#pragma once

#include "Point.h"
#include "WalkingInformations.h"
#include "Representations/MotionControl/WalkRequest.h"
#include "Tools/Streams/AutoStreamable.h"

/** Maximum number of possible foot positions in buffer */
#define PREVIEW_MAX 300

STREAMABLE(StepData,
  StepData()
  {
    direction=0;
    pitch=roll=0;
    footPos[0]=0;
    footPos[1]=0;
    onFloor[0]=true;
    onFloor[1]=true;
  }

  bool onFloor[2];
  float direction;
  float pitch;
  float roll;
  ,
  (Point[2]) footPos
);

STREAMABLE_WITH_BASE(Footposition, StepData,

  bool customStepRunning = false;
  bool prependStepRunning = false;
  unsigned int timestamp;
  WalkRequest::StepRequest customStep;

  unsigned int singleSupportDurationInFrames;
  unsigned int doubleSupportDurationInFrames;
  unsigned int frameInPhase;
  float stepDurationInSec; // Additional info since dynamic step duration is possible
  int stepsSinceCustomStep = 0; // remember for reset of preview

  // kick hack, set specific angles in LimbCombinator
  int timeUntilKickHackHip = 0;
  int kickHackDurationHip = 0;
  Angle kickHackKneeAngle = 0_deg;

  Angle kickHackHipAngle = 0_deg;
  int timeUntilKickHackKnee = 0;
  int kickHackDurationKnee = 0;

  Pose2f speed;
  Pose2f leftFootPose2f;
  Pose2f rightFootPose2f;
  Pose2f lastLeftFootPose2f;
  Pose2f lastRightFootPose2f;
  float deltaDirection;
  Vector2f lastRefZMPEndPositionFC;

  void operator = (const StepData &p)
  { this->StepData::operator=(p);
  }

  Footposition() {};
  ,
  ((DWE) WalkingPhase)(WalkingPhase::unlimitedDoubleSupport) phase,
  (bool)(false) inKick
);


class ZMP : public Vector2f
{
public:
  ZMP()
  {
    x() = 0;
    y() = 0;
  }

  ZMP(const Point& p)
  {
    this->x() = p.x;
    this->y() = p.y;
  }

  const static int phaseToZMPFootMap[];

  unsigned int timestamp;

  float direction;

  ZMP(float x, float y)
  {
    this->x() = x;
    this->y() = y;
  }

  float operator[](unsigned int i)
  {
    if (i == 0)
      return x();
    else
      return y();
  }

  void operator=(Point& p)
  {
    this->x() = p.x;
    this->y() = p.y;
  }

  void operator=(float p) { x() = y() = 0; }

  ZMP operator+(const ZMP& zmp) { return ZMP(x() + zmp.x(), y() + zmp.y()); }

  ZMP operator+=(const ZMP& zmp)
  {
    x() += zmp.x();
    y() += zmp.y();
    return *this;
  }

  ZMP operator-=(const ZMP& zmp)
  {
    x() -= zmp.x();
    y() -= zmp.y();
    return *this;
  }

  ZMP operator*(const float f)
  {
    x() *= f;
    y() *= f;
    return *this;
  }

  operator Point() { return Point(x(), y(), 0); }
};


/**
  * Coordinate system of footPos is always a standing robot with
  * root in the current swing foot.
  * Foot positions for double support phases are ignored.
  * Foot positions for support foot are ignored.
  * Foot trajectory interpolates positions via b-splines and adds result to swing foot position.
  * TODO: make config files smaller by removing useless information!
  */
struct CustomStep : public Streamable
{
  Pose2f footPos[2] = {Pose2f(), Pose2f()}; // pose2f offset for left/right foot
  bool onFloor[2] = {true, true}; // foot is on floor
  std::vector<Vector3f> swingFootTraj = {}; // control points for trajectory of foot in air, not used atm
  int duration = 1; // number of frames for this step
  std::vector<Point> spline = {}; // calculated on loading
  bool kick = false;
  bool mirrored = false;
  bool isPrepend = false;
  virtual void serialize(In* in, Out* out);
  void mirror();
};

struct CustomStepsFile : Streamable
{
  Vector2f ballOffset;
  Angle kickAngle;
  float kickDistance[2];
  float translationThresholdXFront = 0.015f;
  float translationThresholdXBack = 0.015f;
  float translationThresholdY = 0.015f;
  Angle rotationThreshold = 4_deg;

  int timeUntilKickHackHip = 120; // time (ms) until kickhack is triggered in the hip
  int kickHackDurationHip = 100; // duration (ms) of the kickhack in the hip
  Angle kickHackHipAngle = -2;

  int timeUntilKickHackKnee = 96; // time (ms) until kickhack is triggered in the knee
  int kickHackDurationKnee = 108; // duration (ms) of the kickhack in the knee
  Angle kickHackKneeAngle = -10_deg;

  std::vector<CustomStep> steps;
  virtual void serialize(In* in, Out* out);
  void mirror();
  bool isApplicable(float distance);
};
