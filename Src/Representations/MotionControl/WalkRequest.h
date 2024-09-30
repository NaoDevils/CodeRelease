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
  ENUM(RequestType,
    speed, /**< Interpret \c request as absolute walking speed. */
    destination /**< Interpret \c request as relative target. */
  );

   ENUM(RotationType,
    irrelevant, /**< robot rotation is allowed to be used at will */
    towardsBall /**< robot is supposed to rotate towards ball position */
   );

  ENUM(StepRequest,
    none,
    previewKick,
    any, // lets the walking engine choose the kick using kick target from kick request
    beginScript, /**< Add custom step file after this marker */
    // safety step, not a kick!
    safetyStepFront,
    safetyStepBack,
    // kicks are always with left foot (or to the left), have to be mirrored!
    kickHack,
    kickHackLong,
    kickHackVeryLong,
    frontKickLong, /** Long front kick, slower execution time. */
    frontKickShort, /** Quick short kick to front. */
    rotateKick45, /** Rotate the robot 45 degrees and kick with right foot. */
    rotateKick45Long,
    sideKickOuter45, /** Side kick to left front, ball is left of robot. */
    sideKickOuterFoot, /** Side kick to left with the outer (left) foot if ball is next to foot. */
    sideKickOuterFront, /** Side kick to left with the outer (left) foot if ball is in the front left. */
    sideKickInner45, /** Side kick to left front, ball is directly in front of robot. */
    sideKickInnerFoot, /** Side kick to left with the inner (right) foot. */
    keeperKickFront, /** Quick short kick to front without stepping forward */
    keeperKick45 /** Quick short kick to 45 degrees without stepping forward */
  );

  static StepRequest getStepRequestFromName(const char* name)
  {
    for (int i = 0; i < numOfStepRequests; ++i)
      if (!strcmp(name, getName(StepRequest(i))))
        return StepRequest(i);
    return numOfStepRequests;
  }

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
    case none:
    case previewKick:
    case beginScript:
    case numOfStepRequests:
      return false;
    default:
      return true;
    }
  },

  (RequestType)(speed) requestType, /**< The walking mode. */
  (RotationType)(irrelevant) rotationType, /**< E.g. ball faced positioning */
  (Pose2f) request, /**< Target relative to robot or speed in mm/s and radian/s. */
  (Pose2f) accLimits, /**< Accelerations limits for current speed request in m/s. */
  (StepRequest)(none) stepRequest /**< The stepRequest (for in-walk predefined motions) */
);

/**
* @struct WalkRequestCompressed
* A compressed version of WalkRequest used in team communication
*/
STREAMABLE(WalkRequestCompressed,
  WalkRequestCompressed() = default;
  WalkRequestCompressed(const WalkRequest & walkRequest)
  {
    requestType = walkRequest.requestType;
    rotationType = walkRequest.rotationType;
    destination = walkRequest.request;
    stepRequest = walkRequest.stepRequest;
  }
  operator WalkRequest() const
  {
    WalkRequest wr;
    wr.requestType = requestType;
    wr.rotationType = rotationType;
    wr.request = destination;
    wr.stepRequest = stepRequest;
    // rest is not needed -> has default values
    return wr;
  },

  ((WalkRequest) RequestType) requestType, /**< The currently executed walk type */
  ((WalkRequest) RotationType) rotationType, /**< The currently executed rotation type */
  (Pose2f) destination, /**< The current walk destination (or speed in case of type speed) */
  ((WalkRequest) StepRequest) stepRequest /**< The stepRequest (for in-walk predefined motions) */
);
