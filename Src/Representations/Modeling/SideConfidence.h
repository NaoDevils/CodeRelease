/**
 * @file SideConfidence.h
 *
 * Declaration of struct SideConfidence.
 * @author Michel Bartsch, Thomas Münder
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Settings.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Tools/Module/Blackboard.h"

/**
 * @struct SideConfidence
 */
STREAMABLE(SideConfidence,
  ENUM(ConfidenceState,
    CONFIDENT,
    UNSURE,
    CONFUSED
  ); /**< Discrete states of confidence, mapped by provider */

  /** Draw representation. */
  void draw() const,

  (ConfidenceState)(CONFIDENT) confidenceState /**< The state of confidence */
);

inline void SideConfidence::draw() const
{
  DEBUG_DRAWING("representation:SideConfidence", "drawingOnField")
  {
    DRAWTEXT("representation:SideConfidence", -5000, -3600, 140, ColorRGBA::red, "Sideconfidence: " << confidenceState);
  }

  DEBUG_DRAWING3D("representation:SideConfidence", "robot")
  {
    static const ColorRGBA colors[ConfidenceState::numOfConfidenceStates] = {ColorRGBA::green, ColorRGBA::yellow, ColorRGBA::red};
    const RobotInfo& robotInfo = Blackboard::get<RobotInfo>();
    int pNumber = robotInfo.number;
    float centerDigit = (pNumber > 1) ? 50.f : 0;
    ROTATE3D("representation:SideConfidence", 0, 0, pi_2);
    DRAWDIGIT3D("representation:SideConfidence", pNumber, Vector3f(centerDigit, 0.f, 500.f), 80, 5, colors[confidenceState]);
  }
}
