#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/KickWheel.h"

MODULE(KickWheelProvider,
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotMap),

  PROVIDES(KickWheel),

  LOADS_PARAMETERS(,
    (float)(0.005f) dummy
  )
);

class KickWheelProvider : public KickWheelProviderBase
{
public:
  KickWheelProvider();

  void update(KickWheel& kickWheel) override;

  [[nodiscard]] inline std::vector<Geometry::Circle> getObstacles() const;
  [[nodiscard]] static float getBlockedDistance(Angle angle, const Vector2f& ballPosition, const std::vector<Geometry::Circle>& obstacles);
  [[nodiscard]] static float getOutsideDistance(Angle angle, const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions);
};
