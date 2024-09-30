#pragma once

#include "SelectableShot.h"

class SelectablePose
{

public:
  SelectablePose(const SelectableShot& selectableShot, const Pose2f& pose, const bool kickWithLeft) : selectableShot(selectableShot), pose(pose), kickWithLeft(kickWithLeft) {}

  [[nodiscard]] Vector2f getBallPosition() const { return selectableShot.selectableTarget.selectableDirection.ballPosition; }

  void setPoseTime(const float newTime)
  {
    ASSERT(time == -1.f);
    ASSERT(newTime >= 0.f);
    time = newTime;
  }

  void setScore(const float newScore)
  {
    ASSERT(!scoreSet);
    score = newScore;
    scoreSet = true;
  }

  [[nodiscard]] float getPoseTime() const
  {
    ASSERT(time != -1.f);
    return time;
  }

  [[nodiscard]] float getTotalTime() const { return selectableShot.selectableTarget.selectableKick.kick->time + getPoseTime(); }

  [[nodiscard]] float getScore() const
  {
    ASSERT(scoreSet);
    return score;
  }

  SelectableShot selectableShot;
  Pose2f pose;
  bool kickWithLeft;

private:
  float time = -1.f;
  float score;
  bool scoreSet = false;
};
