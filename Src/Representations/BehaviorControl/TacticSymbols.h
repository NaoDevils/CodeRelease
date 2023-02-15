/**
* \file TacticSymbols.h
* The file declares a class that containts data about shared tactics employed by groups of robots.
* \author Ingmar Schwarz
*/

#pragma once
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Tools/Math/Eigen.h"

/**
* \class TacticSymbols
* A class that containts data about shared tactics employed by groups of robots.
*/
STREAMABLE(TacticSymbols,
  void draw() const,

  // Values for tactical decisions
  (unsigned)(1) numberOfActiveFieldPlayers, // myself and active teammates excluding keeper
  (bool)(false) iAmSupported, // TODO Not filled yet
  (bool)(false) interceptBall, // TODO Not filled yet
  (bool)(true) defensiveBehavior,
  (bool)(true) kickoffToTheLeft,
  (int)(0) numberOfLeftOwnKickOffSuccess,
  (int)(0) numberOfRightOwnKickOffSuccess
);
