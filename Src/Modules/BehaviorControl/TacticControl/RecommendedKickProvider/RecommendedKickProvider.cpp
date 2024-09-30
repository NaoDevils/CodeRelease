#include "RecommendedKickProvider.h"

#include "Constants.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Functions/SelectFunctions.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/ShotParameters.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Filterer/Filterer.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserUtils.h"

RecommendedKickProvider::RecommendedKickProvider()
{
  const auto kickEngineParameter = KicksProvider::loadKickEngineParameters();
  const auto customStepFiles = KicksProvider::loadCustomStepFiles();
  kicks = KicksProvider::createKicks(kickEngineParameter, customStepFiles, kickNames);

  const auto kickParameters = ShotParameters::KickParameters(generalParameter, kickBlindParameter);
  const auto targetParameters = ShotParameters::TargetParameters(sidesHeatParameter,
      goalsHeatParameter,
      intoGoalKickOutsideParameter,
      intoKickInOutsideParameter,
      intoCornerKickOutsideParameter,
      intoDirectionOfGoalsHeatParameter,
      intoOpponentsGoalParameter,
      intoOwnGoalParameter,
      selfHeatParameter,
      selfToGoalHeatParameter,
      teammatesHeatParameter,
      teammatesToGoalHeatParameter,
      opponentsHeatParameter,
      opponentsToGoalHeatParameter,
      crowdedParameter);
  const auto poseParameters = ShotParameters::PoseParameters(dontRuntIntoParameter, blockDefensiveConeParameter, blockOpponentParameter);
  configFileParameters = {kickParameters, targetParameters, poseParameters};
}

void RecommendedKickProvider::update(RecommendedKick& theRecommendedKick)
{
  DECLARE_DEBUG_DRAWING(DRAW_RECOMMENDED_KICK, "drawingOnField");

  if (skips < SKIP_UPDATES)
  {
    skips += 1;
    return;
  }
  skips = 0;

  if (thePositioningAndKickSymbols.recommendShot)
  {
    recommend(theRecommendedKick);
  }
  else
  {
    noRecommendation(theRecommendedKick);
  }
}

void RecommendedKickProvider::noRecommendation(RecommendedKick& theRecommendedKick)
{
  theRecommendedKick.hasRecommendation = false;
  currentKickManager.deleteCurrentKick();
}

void RecommendedKickProvider::recommend(RecommendedKick& theRecommendedKick)
{
  ShotParameters cycleParameters = configFileParameters;

  if (theGameInfo.isChampionsCup())
  {
    // todo once a pass counter is implemented or this rule is removed, delete this code

    if (theGameSymbols.kickoffInProgress && theGameSymbols.timeSincePlayingState < 10000 && theGameInfo.setPlay == SET_PLAY_NONE) // TODO fix kickoffInProgress
    {
      cycleParameters.targetParameters.intoOpponentsGoalParameter = 0;
    }
    else if (theGameInfo.setPlay == SET_PLAY_CORNER_KICK || theGameInfo.setPlay == SET_PLAY_GOAL_KICK || theGameInfo.setPlay == SET_PLAY_KICK_IN || theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK)
    {
      //ASSERT(!theGameSymbols.kickoffInProgress); // TODO Uncomment Once the bug with the kickoffInProgress is fixed

      cycleParameters.targetParameters.intoOpponentsGoalParameter = 0;

      float addToPass = 1.f;
      if (theBallSymbols.ballPositionFieldPredicted.x() > theFieldDimensions.xPosOpponentFieldBorder * 3.f / 4.f)
      {
        addToPass = 2.f;
      }
      else if (theBallSymbols.ballPositionFieldPredicted.x() > theFieldDimensions.xPosOpponentFieldBorder * 2.f / 4.f)
      {
        addToPass = 1.f;
      }
      else if (theBallSymbols.ballPositionFieldPredicted.x() > theFieldDimensions.xPosOpponentFieldBorder * 1.f / 4.f)
      {
        addToPass = 0.5f;
      }
      cycleParameters.targetParameters.teammatesToGoalHeatParameter += addToPass;
      cycleParameters.targetParameters.opponentsToGoalHeatParameter += addToPass / 2.f;
      cycleParameters.targetParameters.opponentsHeatParameter += addToPass / 2.f;
      cycleParameters.targetParameters.crowdedParameter += addToPass;
    }
  }

  const Vector2f absoluteBallPosition = theBallSymbols.ballPositionFieldPredicted;
  Filterer filterer = {};
  filterer.filterKickPoseBlockedByGoalPost(theFieldDimensions);
  const std::optional<ExecutableShot> executableShotOptional = SelectFunctions::createAndFilterAndSelect(
      theRobotPoseAfterPreview, absoluteBallPosition, KickUtils::unpack(kicks), currentKickManager.getCurrentKick(theBallSymbols), filterer, cycleParameters, theDirectionInfo, theFieldDimensions, thePositionInfo, theTacticSymbols);

  if (executableShotOptional.has_value())
  {
    if (executableShotOptional.value().hysteresis == Hysteresis::NO)
    {
      // is a new recommendation
      theRecommendedKick.sinceRecommendationTime = theFrameInfo.getTimeSince(theRecommendedKick.recommendationTimeStamp);
      theRecommendedKick.recommendationTimeStamp = theFrameInfo.time;
    }
    theRecommendedKick.hasRecommendation = true;
    theRecommendedKick.estimatedKickTime = executableShotOptional.value().selectablePose.selectableShot.selectableTarget.selectableKick.kick->time;
    theRecommendedKick.estimatedPoseTime = executableShotOptional.value().selectablePose.getPoseTime();
    theRecommendedKick.successProbability = executableShotOptional.value().getSuccessProbability();
    theRecommendedKick.score = executableShotOptional.value().getScore();
    currentKickManager.setCurrentKick(theRecommendedKick, executableShotOptional.value(), theBallSymbols, theFrameInfo);
    executableShotOptional.value().selectablePose.selectableShot.selectableTarget.draw();
  }
  else
  {
    theRecommendedKick.hasRecommendation = false;
    currentKickManager.deleteCurrentKick();
  }
}

MAKE_MODULE(RecommendedKickProvider, behaviorControl)
