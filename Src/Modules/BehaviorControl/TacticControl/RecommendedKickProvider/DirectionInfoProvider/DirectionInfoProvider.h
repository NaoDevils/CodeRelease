#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RecommendedKick/DirectionInfo.h"
#include "Representations/Modeling/RobotMap.h"

MODULE(DirectionInfoProvider,
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotMap),

  PROVIDES(DirectionInfo),

  LOADS_PARAMETERS(,
    (Angle)(1_deg) stepAngle
  )
);

class DirectionInfoProvider : public DirectionInfoProviderBase
{

public:
  DirectionInfoProvider();

  void update(DirectionInfo& directionInfo) override;
  static void update(DirectionInfo& directionInfo, const Angle stepAngle, const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap);

  [[nodiscard]] static std::vector<Geometry::Circle> getObstacles(const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap);
  [[nodiscard]] static float getBlockedDistance(Angle angle, const Vector2f& ballPosition, const std::vector<Geometry::Circle>& obstacles);
  [[nodiscard]] static std::tuple<float, float, float> getTowardsBordersDistances(const Angle directionAngle, const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions);

private:
  [[nodiscard]] static std::tuple<float, int> getToFieldBorderDistanceAndFieldBorderIndex(const Angle directionAngle, const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions);

  int skips = 0;
};
