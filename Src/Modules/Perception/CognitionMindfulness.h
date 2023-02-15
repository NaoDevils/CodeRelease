/**
* @file Modules/Perception/CognitionMindfulness.h
* This file declares a module that is responsible for self diagnosis of the robots mind functions.
* @author <A href="mailto:dominik.braemer@tu-dortmund.de">Dominik Brämer</A>
*/

#pragma once

#include "Tools/Module/Module.h"

// input
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/BehaviorControl/KeySymbols.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Configuration/FieldDimensions.h"

// output
#include "Representations/Perception/CognitionState.h"

// tools
#include "Tools/RingBufferWithSum.h"
#include <cmath>

MODULE(CognitionMindfulness,
  // low level input

  // high level input
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(BallSymbols),
  REQUIRES(BallModel),
  REQUIRES(MotionInfo),
  REQUIRES(MotionRequest),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(FieldColors),
  REQUIRES(FieldColorsUpper),
  REQUIRES(FieldDimensions),

  REQUIRES(FrameInfo),

  REQUIRES(RobotModel),
  REQUIRES(RobotHealth),
  REQUIRES(KeySymbols),

  PROVIDES(CognitionState),
  LOADS_PARAMETERS(,
    (bool)(true) activate,
    (int)(4) locaScoreLimit,
    (int)(50) lowerBrightnessLowThreshold,
    (int)(200) lowerBrightnessBrightThreshold,
    (int)(50) upperBrightnessLowThreshold,
    (int)(200) upperBrightnessBrightThreshold,
    (unsigned)(5) waitFramesBeforeKick,
    (unsigned)(45) maxUnchangedFrames
   )
);

class CognitionMindfulness : public CognitionMindfulnessBase
{
public:
  CognitionMindfulness();

private:
  void update(CognitionState& theCognitionState);

  void checkForLocaProblem(CognitionState& cognitionState);
  void checkForPenaltyProblem(CognitionState& cognitionState);
  void checkForBallLostAfterKick(CognitionState& cognitionState);
  void checkImageBrightness(CognitionState& cognitionState);

  void kickDistanceDetection(CognitionState& cognitionState);
  void kickPathAndTimeDetection(CognitionState& cognitionState);
  void rollResistanceEstimation(CognitionState& cognitionState);
  void kickDistanceSimulation(CognitionState& cognitionState);

  std::vector<Vector2f> kickSimulation(unsigned numberOfPredictions, Vector2f kickTarget, Vector2f ballPosition, double kickPathDistance, double kickTime, bool withoutDeviation = false);

  bool hasProblem(CognitionState& cognitionState, CognitionState::CognitionStateError cognitionStateError);

  // member variables
  // ----------------

  // LocaProblem
  float locaScore = 0.0f;
  bool locaPenaltyActive = false;
  int lastLocaPenalty = 0;

  // PenaltyProblem
  int penaltyScore = 0;
  bool penaltyActive = false;

  // Ball Detection
  unsigned currentKickTime = 0;
  bool ballLost = false;
  int ballLostCounter = 0;
  bool ballLostChange = false;

  // Kick Distance
  bool kicked = false;
  unsigned timeKicked = 0;
  Vector2f ballStartPosition;
  Vector2f ballEndPosition;
  Vector2f lastBallEndPosition;
  float kickDistance = 0;
  unsigned kickedCount = 0;

  // Kick Simulation
  //double footAcceleration = 0.0;
  //double footSpeed = 0.0;
  //double highestFootSpeed = 0.0;

  bool firstPos = true;
  //bool kickStarted = true;
  Vector3f sourcePos;
  Vector3f sinkPos;
  //double kickPathDist = 0.0;
  double highestKickPathDist = 0.0;
  //unsigned stableKickPathDist = 0;
  RingBufferWithSum<double, 60> distVecXBuffer;
  RingBufferWithSum<double, 60> distVecNormBuffer;
  RingBufferWithSum<unsigned, 60> distVecTimeBuffer;
  RingBufferWithSum<double, 3> distVecXFilter;
  RingBufferWithSum<double, 3> distVecNormFilter;
  unsigned sourceTime = 0;
  unsigned sinkTime = 0;
  //bool firstShot = true;
  bool prevFootIntersectsWithBall = false;
  unsigned cooldown = 0;
  bool kickIsStarted = false;


  //bool kickTriggered = false;
  unsigned kickTimeStart = 0;
  unsigned kickTimeStop = 0;
  //unsigned unchangedFrames = 0;
  bool kickDataUpdated = false;
  //double kickDist = 0.0;
  //double maxKickDist = 0.0;

  Vector2f target;
  Vector2f source;
  Angle targetAngle = 0_deg;
  std::vector<Vector2f> pointCloud;
  //float maxX = 0.0f;
  //float minX = 0.0f;
  double boundingBox[4] = {0.0, 0.0, 0.0, 0.0};

  // Rollresistance Estimation
  RingBufferWithSum<double, 3> lastDistances;
  RingBufferWithSum<double, 3> lastKickTimes;
  RingBufferWithSum<double, 3> lastKickPathDistances;
  RingBufferWithSum<double, 3> rollResistanceBuffer;
  //unsigned waitFramesBeforeKickCount = 0;
  double rollResistance = 0.24;
  double prevRollResitance = 0.24;
  double upperRollResistance = 1.0;
  double lowerRollResistance = 0.0;
  double oldEstimatedKickDistance = 0.0;

  double acceleration(double distance, double time, double speed_0) { return (2 * distance) / (time * time) - (2 * speed_0) / time; }

  double speedThruDistance(double distance, double time, double speed_0) { return ((2 * distance) / time) - speed_0; }

  double speed(double acceleration, double time, double speed_0) { return (acceleration * time) + speed_0; }

  double speedOfBall(double footSpeed, double footMass, double ballMass) { return (2 * footSpeed) / (1 + (ballMass / footMass)); }

  double force(double acceleration, double mass) { return acceleration * mass; }

  double accelerationThruSpeed(double speed, double time, double speed_0) { return (speed - speed_0) / time; }

  double distance(double acceleration, double time, double speed_0, double distance_0) { return (0.5 * acceleration * (time * time)) + (speed_0 * time) + distance_0; }

  double airDragBall(double radius, double speed, double dragCoefficient)
  {
    double A = 3.14159265358979323846 * (radius * radius);
    double p = 1.2;
    return 0.5 * p * dragCoefficient * A * (speed * speed);
  }

  Vector2f positionWithDeviation(Vector2f ballPosition, double distance, Angle angle)
  {
    ballPosition[0] += std::cos(angle) * (float)distance;
    ballPosition[1] += std::sin(angle) * (float)distance;
    return ballPosition;
  }
};
