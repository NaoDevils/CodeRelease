#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

/* 13 Keypoints:
0: right hand
1: right elbow
2: right shoulder
3: nose
4: left shoulder
5: left elbow
6: left hand
7: right hip
8: left hip
9: right knee
10:left knee
11:right ankle
12:left ankle
*/

STREAMABLE(RefereeKeypoints,
  RefereeKeypoints(){ position.fill(Vector2f::Zero()); }
  ,
  (std::array<Vector2f, 13>) position,
  (std::array<float, 13>)({0}) confidence
);
