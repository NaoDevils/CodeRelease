/**
* @file HeadPOIList.h
* 
* Declaration of class HeadPOIList
* @author <a href="mailto:mahdokht.mohammadi@tu-dortmund.de">Mahdokht Mohammadi</a>
* @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Angle.h"
#include "Tools/Enum.h"

STREAMABLE(HeadPOIList,
  STREAMABLE(Target,
    ENUM(Type,
      ball,
      remoteBall,
      angle,
      fieldPosition, 
      robot
    );
    ,
    (Type)(Type::ball) type, 
    (Vector2f)({0.f,0.f}) position
  );
  ENUM(Type,
    all,
    sweep,
    focus
  );

  void addAngle(Vector2a angle);
  void addBall(Vector2a qoffset = Vector2a::Zero());
  void addRemoteBall(Vector2a offset = Vector2a::Zero());
  void addFieldPosition(Vector2f position);

  ,
  (std::vector<Target>) targets,
  (Type)(Type::sweep) type 
);
