/**
 * Precalculates heat values for positions. Saves computational resources by grouping positions into cells and only calculating their heat once.
 *
 * Does not increase the heat gradually but overrides the old values every cycle. The RobotMap has to decide if a robot is at a position or not. If there is a robot,
 * it should have heat. If not, the heat should be gone. Adding an additional layer with delay has no benefits but to make good decisions with a bigger delay.
 */

#pragma once

#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RecommendedKick/PositionInfo/PositionInfo.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/InStreams.h"

MODULE(PositionInfoProvider,
  HAS_PREEXECUTION,
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotMap),
  REQUIRES(RobotPoseAfterPreview),

  PROVIDES(PositionInfo),

  LOADS_PARAMETERS(,

  )
);

class PositionInfoProvider : public PositionInfoProviderBase
{

public:
  PositionInfoProvider();

  /**
   * Is called before the modules update method. Used to parallelize the processing
   */
  void execute(tf::Subflow&) override;

  /**
   * The regular update method of the module. Merges the parallelized results of the execute method
   */
  void update(PositionInfo& thePositionInfo) override;

  /**
   * Combines the execute method and the modules update method. Is not used inside this module
   */
  static PositionInfo create(
      const Pose2f& playerPose, const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions, const PositionInfo& thePositionInfo, const RobotMap& theRobotMap, const RobotPoseAfterPreview& theRobotPoseAfterPreview);

private:
  static void updateSplitCells(PositionInfo& thePositionInfo, const std::vector<Pose2f>& playerAtBallPose, const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap);

  static void updateRegularCell(PositionInfo& thePositionInfo,
      int index,
      const std::vector<Pose2f>& selfPose,
      const std::vector<Pose2f>& teamPoses,
      const std::vector<Pose2f>& opponentPoses,
      const std::vector<std::tuple<Pose2f, float>>& crowdedPoses,
      const FieldDimensions& theFieldDimensions);

  static std::vector<Pose2f> getSelfPose(Angle rotation, const Vector2f& translation);

  int skips = 0;
  PositionInfo localPositionInfo;
};
