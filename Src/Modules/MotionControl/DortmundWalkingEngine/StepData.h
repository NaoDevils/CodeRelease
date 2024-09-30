#pragma once

#include "Point.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkRequest.h"
#include "Tools/Streams/AutoStreamable.h"
#include "WalkingInformations.h"

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
  float timeUntilKickHackHip = 0;
  int kickHackDurationHip = 0;
  Angle kickHackKneeAngle = 0_deg;

  Angle kickHackHipAngle = 0_deg;
  float timeUntilKickHackKnee = 0;
  int kickHackDurationKnee = 0;
  float kickHackKneeIntensity = 1.0;
  int kickHackDurationKneeReverse = 0;
  float kickHackKneeIntensityReverse = 1.0;

  float ankleCompensationMultiplier = 1.0;

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

STREAMABLE(CustomStepsFile,
  char name[260];
  WalkRequest::StepRequest stepRequest = WalkRequest::StepRequest::kickHack;

  void mirror();
  bool isApplicable(float distance);
,
  (float)(0.f) generalValue,
  (float)(0.f) verticalInaccuracy,
  (Angle)(0_deg) horizontalInaccuracy,
  (bool)(false) kickBlind,
  (bool)(false) switchKickFoot,

  (Vector2f)(Vector2f::Zero()) ballOffset,
  (Angle)(0_deg) kickAngle,
  (float[2]) kickDistance,
  (float)(0.015f) translationThresholdXFront,
  (float)(0.015f) translationThresholdXBack,
  (float)(0.015f) translationThresholdY,
  (Angle)(4_deg) rotationThreshold,

  (float)(0) timeUntilKickHackHip, // percentage of the time in the kick phase until kickhack is triggered in the hip
  (int)(100) kickHackDurationHip, // duration in frames of the kickhack in the hip
  (Angle)(-2_deg) kickHackHipAngle,
  (float)(1.0f) ankleCompensationMultiplier,

  (float)(0.5) timeUntilKickHackKnee, // percentage of the time in the kick phase until kickhack is triggered in the knee
  (int)(108) kickHackDurationKnee, // duration in frames of the kickhack in the knee
  (Angle)(-10_deg) kickHackKneeAngle,
  (float)(1.0) kickHackKneeIntensity,
  (int)(0) kickHackDurationKneeReverse,
  (float)(1.0) kickHackKneeIntensityReverse,

  (std::vector<CustomStep>) steps
);
