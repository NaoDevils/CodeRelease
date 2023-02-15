/**
* @file CLIPGoalPercept.h
*
* Very simple representation of a seen goal
*
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
* @author Ingmar Schwarz
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Enum.h"
#include "stdint.h"

STREAMABLE(CLIPGoalPercept,
  STREAMABLE(GoalPost,
    /** Defines the different goal posts */
    ENUM(GoalPostSide,
      leftPost,
      rightPost,
      unknownPost
    ),

    (Vector2f) locationOnField,
    (Vector2i) bottomInImage, // bottom middle point of post
    (Vector2i) topInImage, // top middle point of post
    (float) topWidth, // topWidth of goalpost (horizontal)
    (float) bottomWidth, // bottomWidth of goalpost (horizontal)
    (GoalPostSide)(unknownPost) goalPostSide,
    (bool) bottomFound, // field found below goal post
    (bool) foundLineAtBottom, // field line found at goal post bottom
    (bool) fromUpper, // found in upper cam ?
    (float) validity
  );

  /** Constructor */
  CLIPGoalPercept() { goalPosts.reserve(8); }

  /** Draws the goal */
  void draw() const,

  (std::vector<GoalPost>) goalPosts,
  (int)(0) numberOfGoalPosts,
  (unsigned)(0) timeWhenLastSeen
);
