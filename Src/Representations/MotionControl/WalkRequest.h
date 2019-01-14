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
  ENUM(RequestType,
  {,
    speed, /**< Interpret \c speed as absolute walking speed and ignore \c target. */
    destination, /**< Interpret \c speed as percentage walking speed and ignore \c target. */
    ball, /**< Use \c target as walking target relative to the current position of the robot and interpret \c speed as percentage walking speed. */
    dribble,
  });

  ENUM(StepRequest,
  { ,
    none,
    previewKick,
    any, // lets the walking engine choose the kick using kick target from kick request
    beginScript, /**< Add custom step file after this marker */
    frontKickShort, /**< Execute the custom step file */
  });

  // needed to be able to put std::vector<StepRequest> as parameter into LOADS_PARAMETERS macro
  // see e.g. PatternGenerator2017.h
  typedef std::vector<StepRequest> StepRequestVector;

  bool isValid() const
  {
    return !std::isnan(static_cast<float>(request.rotation)) && !std::isnan(request.translation.x()) && !std::isnan(request.translation.y());
  }

  bool isZeroSpeed() const
  {
    return request.translation.x() == 0 && request.translation.y() == 0 && request.rotation == 0;
  }

  /**
   * \return \c true iff the current \c stepRequest is a kick.
   */
  bool isStepRequestKick() const
  {
    switch (stepRequest)
    {
    case frontKickShort:
      return true;
    case none:
    case previewKick:
    case beginScript:
    case numOfStepRequests:
      return false;
    default:
      return true;
    }
  }

  WalkRequest& operator=(const WalkRequest& other)
  {
    if (this == &other)
    {
      return *this;
    }
    requestType = other.requestType;
    request = other.request;
    stepRequest = other.stepRequest;
    return *this;
  },

  (RequestType)(speed) requestType, /**< The walking mode. */
  (Pose2f) request, /**< Target relative to robot or speed in mm/s and radian/s. */
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
    stepRequest = walkRequest.stepRequest;
  }
  operator WalkRequest() const
  {
    WalkRequest wr;
    wr.requestType = requestType;
    wr.request = destination;
    wr.stepRequest = stepRequest;
    // rest is not needed -> has default values
    return wr;
  },

  ((WalkRequest) RequestType) requestType, /**< The currently executed walk type */
  (Pose2f) destination, /**< The current walk destination (or speed in case of type speed) */
  ((WalkRequest) StepRequest) stepRequest, /**< The stepRequest (for in-walk predefined motions) */
});
