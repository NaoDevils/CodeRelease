/**
 * @file Representations/MotionControl/WalkRequest.h
 * This file declares a struct that represents a walk request.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author Colin Graf
 * @author Oliver Urbann
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct WalkRequest
 * A struct that represents a walk request.
 */
STREAMABLE(WalkRequest,
{
  ENUM(StandType,
  { ,
    doubleSupport,
    leftSingleSupport,
    rightSingleSupport,
  });

  ENUM(RequestType,
  {,
    speed, /**< Interpret \c speed as absolute walking speed and ignore \c target. */
    destination, /**< Interpret \c speed as percentage walking speed and ignore \c target. */
    ball, /**< Use \c target as walking target relative to the current position of the robot and interpret \c speed as percentage walking speed. */
  });

  ENUM(StepRequest,
  { ,
    none,
    previewKick,
    beginScript, /**< Add custom step file after this marker */
    frontKickLeft, /**< Execute the custom step file */
    frontKickRight, /**< Execute the custom step file */
  });

  bool isValid() const
  {
    return !std::isnan(static_cast<float>(request.rotation)) && !std::isnan(request.translation.x()) && !std::isnan(request.translation.y());
  },

  (RequestType)(speed) requestType, /**< The walking mode. */
  (Pose2f) request, /**< Target relative to robot or speed in mm/s and radian/s. */
  (StandType)(doubleSupport) standType, /**< How should the robot stand when speed is 0? */
  (float)(0.f) kickStrength, /**< Kick strength for Dortmund WE kick. */
  (float)(0.f) kickDirection, /**< Kick direction for Dortmund WE kick. */
  (StepRequest)(none) stepRequest, /**< The stepRequest (for in-walk predefined motions) */
});

/**
* @struct WalkRequestCompressed
* A compressed version of WalkRequest used in team communication
*/
STREAMABLE(WalkRequestCompressed,
{
  WalkRequestCompressed() = default;
  WalkRequestCompressed(const WalkRequest & walkRequest)
  {
    requestType = walkRequest.requestType;
    destination = walkRequest.request;
  }
  operator WalkRequest() const
  {
    WalkRequest wr;
    wr.requestType = requestType;
    wr.request = destination;
    // rest is not needed -> has default values
    return wr;
  },

  ((WalkRequest) RequestType) requestType, /**< The currently executed walk type */
  (Pose2f) destination, /**< The current walk destination (or speed in case of type speed) */
});
