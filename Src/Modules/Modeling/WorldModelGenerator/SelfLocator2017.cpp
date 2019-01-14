
#include "SelfLocator2017.h"

#include "Tools/Math/GaussianDistribution2D.h"

/*---------------------------- class SelfLocator2017 PUBLIC methods ------------------------------*/

namespace
{
  /** Helper functions **/
  Pose2f getSymmetricPoseOnField(const Pose2f& pose)
  {
    return pose.dotMirror();
  }


  bool isPoseOnSameFieldSide(const Pose2f& p1, const Pose2f& p2)
  {
    return  (((p1.translation.x() <= 0) && (p2.translation.x() <= 0))
      ||
      ((p1.translation.x() >= 0) && (p2.translation.x() >= 0))
      );
  }

  bool isPoseOnOwnFieldSide(const Pose2f &p)
  {
    static const Pose2f own(-1000, 0);
    return isPoseOnSameFieldSide(p, own);
  }

  bool arePosesCloseToEachOther(const Pose2f &p1, const Pose2f &p2, const SelfLocator2017Parameters &parameters)
  {
    return ((p1.translation - p2.translation).norm() < parameters.sensorReset.maxDistanceForLocalResetting
      &&std::abs(Angle::normalize(p1.rotation - p2.rotation)) < parameters.sensorReset.maxAngleForLocalResetting);
  }

  bool getLeftAndRightGoalPostFromGoalPercept(const CLIPGoalPercept& theGoalPercept, Vector2f& leftPost, Vector2f &rightPost, float &validity)
  {
    validity = 0.f;
    if (theGoalPercept.goalPosts[0].goalPostSide == CLIPGoalPercept::GoalPost::leftPost && theGoalPercept.goalPosts[1].goalPostSide == CLIPGoalPercept::GoalPost::rightPost)
    {
      leftPost = theGoalPercept.goalPosts[0].locationOnField.cast<float>();
      rightPost = theGoalPercept.goalPosts[1].locationOnField.cast<float>();
    }
    else if (theGoalPercept.goalPosts[1].goalPostSide == CLIPGoalPercept::GoalPost::leftPost && theGoalPercept.goalPosts[0].goalPostSide == CLIPGoalPercept::GoalPost::rightPost)
    {
      leftPost = theGoalPercept.goalPosts[1].locationOnField.cast<float>();
      rightPost = theGoalPercept.goalPosts[0].locationOnField.cast<float>();
    }
    else
      return false;

    validity = (theGoalPercept.goalPosts[0].validity + theGoalPercept.goalPosts[1].validity) / 2;
    return true;
  }
}


SelfLocator2017::SelfLocator2017() :
  lastExecuteTimeStamp(0),
  penalizedTimeStamp(0),
  unpenalizedTimeStamp(0),
  lastPenalty(PENALTY_NONE),
  lastNonPlayingTimeStamp(0),
  timeStampFirstReadyState(0),
  lastBestHypothesisUniqueId(0)
{
  initialized = false;

  localizationState = positionTracking;
  localizationStateAfterGettingUp = localizationState;
  foundGoodPosition = false;
  symmetryLost = false;
}

SelfLocator2017::~SelfLocator2017()
{
  PoseHypothesis2017::cleanup();
}

void SelfLocator2017::update(RobotPose& robotPose)
{
  executeCommonCode();
  robotPose = m_robotPose;
}

void SelfLocator2017::update(SideConfidence& confidence)
{
  executeCommonCode();
  confidence = m_sideConfidence;
}


void SelfLocator2017::update(RobotPoseHypothesis& robotPoseHypothesis)
{
  executeCommonCode();
  robotPoseHypothesis = m_robotPoseHypothesis;
}

void SelfLocator2017::update(RobotPoseHypotheses& robotPoseHypotheses)
{
  executeCommonCode();
  robotPoseHypotheses = m_robotPoseHypotheses;
}

void SelfLocator2017::update(RobotPoseHypothesesCompressed& robotPoseHypotheses)
{
  executeCommonCode();
  robotPoseHypotheses = RobotPoseHypothesesCompressed(m_robotPoseHypotheses);
}


void SelfLocator2017::predictHypotheses()
{
  Pose2f odometryDelta = theOdometryData - lastOdometryData;
  lastOdometryData = theOdometryData;

  distanceTraveledFromLastFallDown += odometryDelta.translation.norm();

  for (auto &hypothesis : poseHypotheses)
  {
    hypothesis->predict(odometryDelta, parameters);
  }
}


void SelfLocator2017::updateHypothesesPositionConfidence()
{
  bool updateSpherical, updateInfiniteLines, updateWeighted;
  for (auto &hypothesis : poseHypotheses)
  {
    updateSpherical = hypothesis->updatePositionConfidenceWithLocalFeaturePerceptionsSpherical(theLineMatchingResult, theCLIPCenterCirclePercept, theCLIPGoalPercept, thePenaltyCrossPercept, theFieldDimensions, theCameraMatrix, theCameraMatrixUpper, parameters);
    updateInfiniteLines = hypothesis->updatePositionConfidenceWithLocalFeaturePerceptionsInfiniteLines(theLineMatchingResult, theCLIPCenterCirclePercept, theFieldDimensions, parameters);
    updateWeighted = hypothesis->updatePositionConfidenceWithLocalFeaturePerceptionsWeighted(theCLIPFieldLinesPercept, theCLIPCenterCirclePercept, theCLIPGoalPercept, thePenaltyCrossPercept, theFieldDimensions, parameters);

    if (!(updateSpherical || updateInfiniteLines || updateWeighted))
    {
      // TODO: Negative update as nothing was seen?
    }
  }
}

void SelfLocator2017::updateHypothesesState()
{
  for (auto &hypothesis : poseHypotheses)
  {
    hypothesis->updateStateWithLocalFeaturePerceptionsSpherical(parameters);
    hypothesis->updateStateWithLocalFeaturePerceptionsInfiniteLines(parameters);
    hypothesis->updateStateRotationWithLocalFieldLines(theCLIPFieldLinesPercept, parameters);
  }
}

void SelfLocator2017::updateHypothesesSymmetryConfidence()
{
  for (auto &hypothesis : poseHypotheses)
  {
    // Update symmetry only when playing (no ball on the field in READY! which is the strongest/only symmetry indicator)
    if (theGameInfo.state == STATE_PLAYING && parameters.symmetryUpdate.updateWithRemoteModels)
    {
      hypothesis->updateSymmetryByComparingRemoteToLocalModels(theBallModel, theRemoteBallModel, theLocalRobotMap, theRemoteRobotMap, theTeammateData, theFrameInfo, parameters);
    }
    else
    {
      hypothesis->setSymmetryConfidence(1.f);
    }

    // no symmetrie loss for goalie. ever.
    if (theBehaviorData.role == BehaviorData::keeper)
    {
      hypothesis->setSymmetryConfidence(1.f);
    }
  }

  std::underlying_type<SideConfidence::ConfidenceState>::type cs = m_sideConfidence.confidenceState;
  float bestSymmetryConfidence = getBestHypothesis().getSymmetryConfidence();

  if (bestSymmetryConfidence > lastBestSymmetryConfidence)
  {
    if (bestSymmetryConfidence == 1.f)
    {
      cs = SideConfidence::CONFIDENT;
    }
    // The lower the more confident -> So increase if best hypo switched
    symmetryPosUpdate = ++symmetryPosUpdate % symmetryUpdatesBeforeAdjustingState;
    if (symmetryPosUpdate == 0)
      cs--;
  }
  else if (bestSymmetryConfidence < lastBestSymmetryConfidence)
  {
    symmertryNegUpdate = ++symmertryNegUpdate % symmetryUpdatesBeforeAdjustingState;
    if (symmertryNegUpdate == 0)
      cs++;
  }

  cs = std::min(std::underlying_type<SideConfidence::ConfidenceState>::type(SideConfidence::numOfConfidenceStates - 1),
    std::max(std::underlying_type<SideConfidence::ConfidenceState>::type(SideConfidence::CONFIDENT), cs));

  m_sideConfidence.confidenceState = static_cast<SideConfidence::ConfidenceState>(cs);

  lastBestSymmetryConfidence = bestSymmetryConfidence;
}


void SelfLocator2017::executeCommonCode()
{
  if (!initialized)
  {
    addHypothesesOnInitialKickoffPositions();
    addHypothesesOnManualPositioningPositions();
    // Initialize distance with in a way it will cause spawns on 1st fall down
    distanceTraveledFromLastFallDown = parameters.spawning.minDistanceBetweenFallDowns;
    initialized = true;
  }

  if (lastExecuteTimeStamp == theFrameInfo.time)
  {
    return;
  }
  lastExecuteTimeStamp = theFrameInfo.time;


  initDebugging();

  // Save best hypothesis data
  const PoseHypothesis2017 &bestHypothesis = getBestHypothesis();
  Pose2f pose;
  bestHypothesis.getRobotPose(pose);
  float pc = bestHypothesis.getPositionConfidence();
  float sc = bestHypothesis.getSymmetryConfidence();

  // Predict new position
  predictHypotheses();

  // Fill matrices for update
  for (auto &hyp : poseHypotheses)
    hyp->fillCorrectionMatrices(theLineMatchingResult, theCLIPCenterCirclePercept, theCLIPGoalPercept, thePenaltyCrossPercept, theFieldDimensions,
      theCameraMatrix, theCameraMatrixUpper, parameters);

  // Update state of hypotheses
  updateHypothesesState();

  // Evaluate status of localization (OK, Symmetrie lost, Lost)
  evaluateLocalizationState();

  // Add new hypotheses
  addNewHypotheses();

  // Update confidence of hypotheses
  updateHypothesesPositionConfidence();

  // Update symmetry confidence of each hypothesis
  updateHypothesesSymmetryConfidence();

  // Remove unnecessary hypotheses
  pruneHypotheses();

  doGlobalDebugging();

  // reset hypotheses if no more are left
  if (poseHypotheses.empty())
  {
    OUTPUT_ERROR("SelfLocator2017 pruned all hypotheses because of their state -> resetting!");

    // readd the best last with a small validity bonus, if it was inside carpet

    Vector2f lastPosition = pose.translation;
    if (theFieldDimensions.clipToCarpet(lastPosition) < 100)
      poseHypotheses.push_back(pose, std::max(pc * .8f, parameters.spawning.positionConfidenceWhenPositionedManually + 0.1f), sc, theFrameInfo.time, parameters);

    // re-add best one from before as well as some poses around the field
    // TODO: if it was an state update error and the position was good, maybe we only want new hypotheses from line matches?
    addHypothesesOnManualPositioningPositions();
    addHypothesesOnPenaltyPositions(0.2f);
    addNewHypothesesFromLineMatches();

    // mirror the hypothesis if old best was on the opp field side
    if (pose.translation.x() > 0)
    {
      for (auto& ph : poseHypotheses)
      {
        ph->setSymmetryConfidence(0.f);
      }
      addNewHypothesesWhenSymmetryLost();
    }

  }

  // Fill local storage for representations
  generateOutputData();

}

void SelfLocator2017::normalizeWeights()
{
  double sumOfPositionConfidences = 0;
  for (auto &hypothesis : poseHypotheses)
  {
    sumOfPositionConfidences += hypothesis->getPositionConfidence();
  }
  double factor = 1 / sumOfPositionConfidences;
  for (auto &hypothesis : poseHypotheses)
  {
    hypothesis->normalizePositionConfidence(factor);
  }
}


void SelfLocator2017::pruneHypothesesInOwnHalf()
{
  PoseHypotheses2017::iterator it = poseHypotheses.begin();
  while (it != poseHypotheses.end() && poseHypotheses.size() > 1)
  {
    Pose2f strikerPose;
    (*it)->getRobotPose(strikerPose);
    if (strikerPose.translation.x() < 0) // striker will never reach the own half because of manually placement.
    {
      it = poseHypotheses.erase(it);
    }
    else
    {
      it++;
    }
  }
  // end
}

void SelfLocator2017::pruneHypothesesInOpponentHalf()
{
  PoseHypotheses2017::iterator it = poseHypotheses.begin();
  while (it != poseHypotheses.end() && poseHypotheses.size() > 1)
  {
    Pose2f keeperPose;
    (*it)->getRobotPose(keeperPose);
    if (keeperPose.translation.x() > 0)
    {
      it = poseHypotheses.erase(it);
    }
    else
    {
      it++;
    }
  }
}

void SelfLocator2017::pruneHypothesesOutsideField()
{
  for (PoseHypotheses2017::iterator it = poseHypotheses.begin();
    it != poseHypotheses.end();)
  {
    const PoseHypotheses2017::element_type hypothesis = (*it);
    if (hypothesis->isInsideFieldPlusX(theFieldDimensions, 100))
    {
      //advance
      it++;
    }
    else
    {
      //delete and auto advance
      it = poseHypotheses.erase(it);
    }
  }
}

void SelfLocator2017::pruneHypothesesOutsideCarpet()
{
  for (PoseHypotheses2017::iterator it = poseHypotheses.begin();
    it != poseHypotheses.end();)
  {
    const PoseHypotheses2017::element_type hypothesis = (*it);
    if (hypothesis->isInsideCarpet(theFieldDimensions))
    {
      //advance
      it++;
    }
    else
    {
      if (parameters.debugging.displayWarnings)
        OUTPUT_WARNING("SelfLocator2017 detected hypothesis position that is outside of carpet -> deleting hypothesis");

      //delete and auto advance
      it = poseHypotheses.erase(it);
    }
  }
}

void SelfLocator2017::pruneHypothesesWithInvalidValues()
{
  for (PoseHypotheses2017::iterator it = poseHypotheses.begin();
    it != poseHypotheses.end();)
  {
    const PoseHypotheses2017::element_type hypothesis = (*it);
    if (hypothesis->containsInvalidValues())
    {
      OUTPUT_ERROR("SelfLocator2017 detected impossible state (NaN) -> deleting hypothesis");
      //delete and auto advance
      it = poseHypotheses.erase(it);
    }
    else
    {
      //advance
      it++;
    }
  }
}

void SelfLocator2017::pruneHypotheses()
{
  if (poseHypotheses.empty())
  {
    OUTPUT_WARNING("SelfLocator2017:no hypotheses left before pruning!");
    return;
  }

  pruneHypothesesOutsideCarpet();
  pruneHypothesesWithInvalidValues();

  // special case: no position on opp side
  if (theBehaviorData.role == BehaviorData::keeper // Let's simply assume that the keeper will never be in the opponent half!  :)
    || (theGameInfo.state == STATE_SET && Global::getSettings().gameMode != Settings::penaltyShootout)) // Cannot be in opponent half in set
    pruneHypothesesInOpponentHalf();

  // in penalty shootout the striker will never reach the own half because of manually placement
  if (Global::getSettings().gameMode == Settings::penaltyShootout && theBehaviorData.role != BehaviorData::keeper)
    pruneHypothesesInOwnHalf();

  // Prune hypothesis outside of field in set state. We can be sure to be inside of field after set!
  if (theGameInfo.state == STATE_SET && Global::getSettings().gameMode != Settings::penaltyShootout)
    pruneHypothesesOutsideField();


  // Delete positions that seem to be symmetric
  for (PoseHypotheses2017::size_type i = 0; i < poseHypotheses.size(); i++)
  {
    Pose2f mirrorOfHypothesis;
    poseHypotheses[i]->getRobotPose(mirrorOfHypothesis);
    mirrorOfHypothesis = getSymmetricPoseOnField(mirrorOfHypothesis);
    for (PoseHypotheses2017::size_type j = i + 1; j < poseHypotheses.size(); j++)
    {
      Pose2f hypoPose;
      poseHypotheses[j]->getRobotPose(hypoPose);
      // Identify mirror hypotheses and remove the worse one
      if (arePosesCloseToEachOther(mirrorOfHypothesis, hypoPose, parameters)
        // Identify "better" mirror hypotheses if symmetry is not lost
        && (symmetryLost || fabs(poseHypotheses[i]->getSymmetryConfidence() - poseHypotheses[j]->getSymmetryConfidence()) > 0.2f))
      {
        if (poseHypotheses[i]->getSymmetryConfidence() < poseHypotheses[j]->getSymmetryConfidence())
        {
          poseHypotheses[i]->scalePositionConfidence(0);
        }
        else
        {
          poseHypotheses[j]->scalePositionConfidence(0);
        }
      }
    }
  }


  // "merge" very close hypotheses, i.e. simply delete the less confident one
  GaussianDistribution3D gd1, gd2;
  for (PoseHypotheses2017::size_type k = 0; k < poseHypotheses.size(); k++)
  {
    poseHypotheses[k]->extractGaussianDistribution3DFromStateEstimation(gd1);
    for (PoseHypotheses2017::size_type j = k + 1; j < poseHypotheses.size(); j++)
    {
      poseHypotheses[j]->extractGaussianDistribution3DFromStateEstimation(gd2);
      double likelihood = gd1.normalizedProbabilityAt(gd2.mean) * gd2.normalizedProbabilityAt(gd1.mean);
      if (likelihood > parameters.pruning.likelihoodTresholdForMerging)
      {
        if (poseHypotheses[k]->getPositionConfidence() > poseHypotheses[j]->getPositionConfidence())
        {
          poseHypotheses[j]->scalePositionConfidence(0);
        }
        else
        {
          poseHypotheses[k]->scalePositionConfidence(0);
        }
      }
    }
  }

  // remove hypotheses with too little position confidence
  float maxConfidence = 0.0;
  for (auto &hypothesis : poseHypotheses)
  {
    maxConfidence = std::max(maxConfidence, hypothesis->getPositionConfidence());
  }

  const double pruningThreshold = std::max(maxConfidence / 4, 0.1f);

  PoseHypotheses2017::iterator it = poseHypotheses.begin();
  while (it != poseHypotheses.end() && poseHypotheses.size() > 1)
  {
    if ((*it)->getPositionConfidence() < pruningThreshold) // no need to make this small threshold a magic number
    {
      it = poseHypotheses.erase(it);
    }
    else if (localizationState == positionLost && (*it)->getSymmetryConfidence() < 0.08)
    {
      it = poseHypotheses.erase(it);
    }
    else
    {
      it++;
    }
  }

  // remove the worst ones to keep the number low
  const PoseHypotheses2017::size_type maxHypotheses = static_cast<PoseHypotheses2017::size_type>(parameters.pruning.maxNumberOfHypotheses);
  while (poseHypotheses.size() > maxHypotheses)
  {
    PoseHypotheses2017::iterator worst = poseHypotheses.begin();
    for (it = poseHypotheses.begin(); it != poseHypotheses.end(); it++)
    {
      if ((*it)->getPositionConfidence() < (*worst)->getPositionConfidence())
      {
        worst = it;
      }
    }
    poseHypotheses.erase(worst);
  }
}


bool SelfLocator2017::hasSymmetryBeenLostForBestHypotheses()
{
  const PoseHypothesis2017 &bestHyp = getBestHypothesis();
  return hasSymmetryBeenLost(bestHyp);
}

bool SelfLocator2017::hasSymmetryBeenLost(const PoseHypothesis2017& hypotheses)
{
  bool symmetryConfidenceLost = hypotheses.getSymmetryConfidence() < parameters.localizationStateUpdate.symmetryLostWhenBestConfidenceBelowThisThreshold;
  return symmetryConfidenceLost;
}

bool SelfLocator2017::hasSymmetryBeenFoundAgainAfterLoss()
{
  const PoseHypothesis2017 &bestHyp = getBestHypothesis();
  return hasSymmetryBeenFoundAgainAfterLoss(bestHyp);
}

bool SelfLocator2017::hasSymmetryBeenFoundAgainAfterLoss(const PoseHypothesis2017& hypotheses)
{
  bool symmetryConfidenceFoundAgain = hypotheses.getSymmetryConfidence() > parameters.localizationStateUpdate.symmetryFoundAgainWhenBestConfidenceAboveThisThreshold;
  return symmetryConfidenceFoundAgain;
}

bool SelfLocator2017::hasPositionTrackingFailed()
{
  if (theFrameInfo.getTimeSince(lastNonPlayingTimeStamp) < 5000)
  {
    return false;
  }

  if (poseHypotheses.empty())
    return true;

  const PoseHypothesis2017 &bestHyp = getBestHypothesis();
  bool positionConfidenceLost = bestHyp.getPositionConfidence() < parameters.localizationStateUpdate.positionLostWhenBestConfidenceBelowThisThreshold;
  return positionConfidenceLost;
  //  bool symmetryConfidenceLost = bestHyp.getSymmetryConfidence() < parameters.localizationStateUpdate.symmetryLostWhenBestConfidenceBelowThisThreshold;
  //  bool symmetryLost = positionConfidenceLost || symmetryConfidenceLost;
  //  return symmetryLost;
}

bool SelfLocator2017::hasPositionBeenFoundAfterLoss()
{
  if (poseHypotheses.empty())
    return false;

  const PoseHypothesis2017 &bestHyp = getBestHypothesis();
  bool positionIsConfident = bestHyp.getPositionConfidence() > parameters.localizationStateUpdate.positionFoundAgainWhenBestConfidenceAboveThisThreshold;
  //  bool symmetryIsConfident =  bestHyp.getSymmetryConfidence() > parameters.localizationStateUpdate.symmetryFoundAgainWhenBestConfidenceAboveThisThreshold;
  //  bool symmetryFoundAgain = positionIsConfident && symmetryIsConfident;
  return positionIsConfident;
}

void SelfLocator2017::evaluateLocalizationState()
{
  // are we playing regularly yet?
  if (theRobotInfo.penalty != PENALTY_NONE || (theGameInfo.state != STATE_PLAYING && theGameInfo.state != STATE_READY))
  {
    lastNonPlayingTimeStamp = theFrameInfo.time;
  }


  // falling down is a transition which will be taken no matter which state we had before.
  if (theFallDownState.state != FallDownState::upright)
  {
    if (localizationState != fallenDown)
    {
      handleFallDown();

      localizationStateAfterGettingUp = localizationState;
    }
    localizationState = fallenDown;
  }
  // penalized is a transition which will be taken no matter which state we had before.
  if (theRobotInfo.penalty != PENALTY_NONE)
  {
    if (localizationState != penalized)
    {
      penalizedTimeStamp = theFrameInfo.time;
    }
    lastPenalty = theRobotInfo.penalty;
    localizationState = penalized;
  }
  if (theGameInfo.state == STATE_SET)
  {
    handleSetState();
  }
  if (theGameInfo.state == STATE_INITIAL)
  {
    handleInitialState();
  }


  switch (localizationState)
  {
  case positionTracking:
    if (hasPositionTrackingFailed())
    {
      lastPositionLostTimeStamp = theFrameInfo.time;
      localizationState = positionLost;
    }
    break;
  case positionLost:
    if (hasPositionBeenFoundAfterLoss())
    {
      localizationState = positionTracking;
    }
    break;
  case fallenDown:
    if (theFallDownState.state == FallDownState::upright && theCameraMatrix.isValid)
    {
      handleGettingUpAfterFallDown();
      localizationState = localizationStateAfterGettingUp;
    }
    break;
  case penalized:
    if (theRobotInfo.penalty == PENALTY_NONE)
    {
      handleUnPenalized();
      localizationState = positionTracking;
    }
    break;
  default:
    // should never happen!
    char text[100];
    std::sprintf(text, "SelfLocator2017 in unknown localization state %d (%s)", localizationState, getName(localizationState));
    OUTPUT_ERROR(&text[0]);
    break;
  }

  if (symmetryLost)
    symmetryLost = !hasSymmetryBeenFoundAgainAfterLoss();
  else
    symmetryLost = hasSymmetryBeenLostForBestHypotheses();
}


void SelfLocator2017::handleFallDown()
{
  if (parameters.localizationStateUpdate.symmetryLostWhenFallDownInCenterCircle)
  {
    // Handle fall down in center circle
    const PoseHypothesis2017 &bestHyp = getBestHypothesis();
    Pose2f p;
    bestHyp.getRobotPose(p);
    if (p.translation.norm() < parameters.localizationStateUpdate.unknownSymmetryRadiusAroundCenter)
    {
      for (auto &hypothesis : poseHypotheses)
      {
        hypothesis->setSymmetryConfidence(0.f); //only case where symmetry is set to 0
      }
    }
  }
}

void SelfLocator2017::handleGettingUpAfterFallDown()
{
  // Check if moved far enough that it makes sense to spawn new hypothesis
  if (distanceTraveledFromLastFallDown < parameters.spawning.minDistanceBetweenFallDowns)
  {
    return;
  }
  distanceTraveledFromLastFallDown = 0.f;

  // if we fell on the side, we might end up with +/- 90° orientation errors,
  // so add those hypotheses...
  std::size_t size = poseHypotheses.size();
  for (std::size_t i = 0; i < size; i++)
  {
    PoseHypotheses2017::element_type &hypothesis = poseHypotheses[i];

    hypothesis->scalePositionConfidence(parameters.sensorUpdate.confidenceScaleFactorAfterFallDown); // = 0.75
    for (unsigned char j = 1; j <= parameters.spawning.noAdditionalHypothesisAfterFallDown; ++j)
    {
      poseHypotheses.push_back(*hypothesis, theFrameInfo.time); // copy hypotheses
      PoseHypotheses2017::element_type p1 = poseHypotheses.back(); // Get new hypothesis

      const float rot = pi2 * float(j) / (parameters.spawning.noAdditionalHypothesisAfterFallDown + 1);
      const float scale = 1 - std::abs(Angle::normalize(rot)) / pi;

      const float confidenceScaleMin = 1 - parameters.sensorUpdate.confidenceScaleFactorAfterFallDown;
      const float confidenceScaleMax = parameters.sensorUpdate.confidenceScaleFactorAfterFallDown;
      float confidenceScale = 0;
      if (confidenceScaleMin < confidenceScaleMax)
      {
        confidenceScale = confidenceScaleMin + (confidenceScaleMax - confidenceScaleMin) * scale;
      }
      else
      {
        confidenceScale = confidenceScaleMax + (confidenceScaleMin - confidenceScaleMax) * scale;
      }
      p1->rotateHypothesis(rot);
      p1->scalePositionConfidence(confidenceScale);
    }
  }
}

void SelfLocator2017::handleUnPenalized()
{
  unpenalizedTimeStamp = theFrameInfo.time;

  if (lastPenalty == PENALTY_SPL_ILLEGAL_MOTION_IN_SET)
  {
    // If penalty was illegal motion in set
    // -> robots stayed where they are (none taken out of game)
    // -> do nothing, just keep as is
    return;
  }

  if (theFrameInfo.getTimeSince(penalizedTimeStamp) > 15000)
  {
    // penalized for more than 15 seconds, so it was most likely no mistake
    // and we can delete all other hypotheses
    poseHypotheses.clear();
  }
  else
  {
    // penalized for less than 15 seconds, so it might have been a mistake
    // and we just decrease the confidence of the other hypotheses
    for (auto &hypothesis : poseHypotheses)
    {
      hypothesis->scalePositionConfidence(0.5f);
    }
  }
  if (lastPenalty == PENALTY_MANUAL)
  {
    addHypothesesOnPenaltyPositions(0.3f);
  }
  else
  {
    addHypothesesOnPenaltyPositions(0.7f);
  }
  addHypothesesOnManualPositioningPositions();
}

void SelfLocator2017::handleSetState()
{
  // delete all hypotheses on the wrong side of the field unless in penalty shootout
  if (Global::getSettings().gameMode != Settings::penaltyShootout)
  {
    PoseHypotheses2017::iterator i = poseHypotheses.begin();
    while (i != poseHypotheses.end() && poseHypotheses.size() > 1)
    {
      Pose2f pose;
      (*i)->getRobotPose(pose);
      // should be "pose.translation.x > 0", but we allow a little buffer of 20cm
      bool poseIsInOpponentHalf = pose.translation.x() > 200; // no need to make this small threshold a magic number
      if (poseIsInOpponentHalf)
      {
        i = poseHypotheses.erase(i);
      }
      else
      {
        //also reset symmetry confidence of existing hypotheses
        (*i)->setSymmetryConfidence(parameters.spawning.symmetryConfidenceWhenPositionedManually);
        i++;
      }
    }
  }
}

void SelfLocator2017::handleInitialState()
{
  timeStampFirstReadyState = theFrameInfo.time;
}


void SelfLocator2017::addHypothesesOnManualPositioningPositions()
{
  const float sc = parameters.spawning.symmetryConfidenceWhenPositionedManually;

  bool ownKickoff = theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber;
  // special position for goalie
  const bool isGoalKeeper = (theBehaviorData.role == BehaviorData::keeper);
  if (isGoalKeeper)
  {
    // penalty shootout
    if (Global::getSettings().gameMode == Settings::penaltyShootout)
    {
      poseHypotheses.push_back(Pose2f(0, positionsByRules.penaltyShootoutGoaliePosition),
        parameters.spawning.positionConfidenceWhenPositionedManuallyForGoalKeeper, sc,
        theFrameInfo.time, parameters);
    }
    else
    {
      poseHypotheses.push_back(Pose2f(0, positionsByRules.goaliePosition),
        parameters.spawning.positionConfidenceWhenPositionedManuallyForGoalKeeper, sc,
        theFrameInfo.time, parameters);
    }
  }
  // one of the field positions for field players
  else
  {
    // penalty shootout
    if (Global::getSettings().gameMode == Settings::penaltyShootout)
    {
      addPenaltyStrikerStartingHypothesis();
    }
    else
    {
      for (auto &position : (ownKickoff ? positionsByRules.fieldPlayerPositionsOwnKickoff : positionsByRules.fieldPlayerPositionsOppKickoff))
      {
        poseHypotheses.push_back(Pose2f(0, position),
          parameters.spawning.positionConfidenceWhenPositionedManually, sc,
          theFrameInfo.time, parameters);
      }
    }
  }
}

void SelfLocator2017::addHypothesesOnInitialKickoffPositions()
{
  if (Global::getSettings().gameMode != Settings::penaltyShootout)
  {
    addHypothesesOnPenaltyPositions(parameters.spawning.positionConfidenceWhenPositionedManually);
  }
  else
  {
    addPenaltyStrikerStartingHypothesis();
  }
}

void SelfLocator2017::addPenaltyStrikerStartingHypothesis()
{
  // in 2018 rules: starting position on circle around penalty mark
  Vector2f penaltyMarkPos = Vector2f(
    theFieldDimensions.xPosOpponentPenaltyMark,
    theFieldDimensions.yPosCenterGoal);

  Vector2f backTransl = Vector2f(positionsByRules.penaltyShootStartingRadius, 0);
  Vector2f rot;

  const float sc = parameters.spawning.symmetryConfidenceWhenPositionedManually;

  for (int angle : positionsByRules.penaltyShootAngles) {
    Vector2f rot = Vector2f(backTransl).rotate(Angle::fromDegrees(angle));
    poseHypotheses.push_back(Pose2f(rot.angle(), penaltyMarkPos - rot),
      parameters.spawning.positionConfidenceWhenPositionedManually,
      sc,
      theFrameInfo.time,
      parameters);
  }
}

void SelfLocator2017::addHypothesesOnPenaltyPositions(float newPositionConfidence)
{
  if (Global::getSettings().gameMode != Settings::penaltyShootout)
  {
    float sc = parameters.spawning.symmetryConfidenceWhenPositionedManually;

    for (const auto &offset : positionsByRules.xOffsetPenaltyPositions)
    {
      poseHypotheses.push_back(Pose2f(pi_2, offset, theFieldDimensions.yPosRightSideline), newPositionConfidence,
        sc,
        theFrameInfo.time,
        parameters);
      poseHypotheses.push_back(Pose2f(-pi_2, offset, theFieldDimensions.yPosLeftSideline), newPositionConfidence,
        sc,
        theFrameInfo.time,
        parameters);
    }
  }
}

bool SelfLocator2017::addNewHypothesesWhenSymmetryLost()
{
  // Remember number of hypothesis as new ones will be added
  PoseHypotheses2017::size_type size = poseHypotheses.size();

  bool added = false;
  //for each Hypotheses that has a lost symmetry ... duplicate it?
  for (PoseHypotheses2017::size_type i = 0; i < size; i++)
  {
    PoseHypotheses2017::element_type hypothesis = poseHypotheses[i];

    /** Symmeric positions will be reintroduced
        AUTHOR: D.Hauschildt
        TODO:   Local Symmetry in penalty area is not considered
        **/
    if (hasSymmetryBeenLost(*hypothesis))
    {
      Pose2f symmetricPose;
      hypothesis->getRobotPose(symmetricPose);
      symmetricPose = getSymmetricPoseOnField(symmetricPose);

      poseHypotheses.push_back(
        symmetricPose,
        hypothesis->getPositionConfidence(),
        hypothesis->getSymmetryConfidence() + .1f,
        theFrameInfo.time,
        parameters);

      added = true;
    }
  }
  return added;
}

bool SelfLocator2017::addNewHypothesesFromLineMatches()
{
  const bool isLost = (localizationState == positionLost);

  const PoseHypothesis2017 &bestHypo = getBestHypothesis();
  Pose2f hypoPose;
  bestHypo.getRobotPose(hypoPose);

  std::vector<HypothesisBase> additionalHypotheses;
  size_t size = theLineMatchingResult.poseHypothesis.size();
  if (size > 0)
  {
    for (auto &ph : theLineMatchingResult.poseHypothesis)
    {
      // Only add hypotheses on own side
      // AddPoseToHypothesisVector will handle symmetry
      // Also check if pose is reasonable unique or is close to actual best pose
      if (ph.pose.translation.x() < 0 && (isLost || size < 7 || arePosesCloseToEachOther(ph.pose, hypoPose, parameters)))
        addPoseToHypothesisVector((Pose2f)ph.pose, additionalHypotheses, parameters.spawning.lineBasedPositionConfidenceWhenPositionTracking);
    }
  }

  // Add hypotheses to the system
  for (auto &hyp : additionalHypotheses)
  {
    poseHypotheses.push_back(hyp.pose, hyp.positionConfidence, hyp.symmetryConfidence, theFrameInfo.time, parameters);
  }
  return !additionalHypotheses.empty();
}

bool SelfLocator2017::addNewHypothesesFromPenaltyCrossLine()
{
  // Add new hypotheses by PenaltyCross
  // it gives an absolute Pose

  Pose2f pose;
  float poseConfidence = 0.f;
  float bestHypoConfidence = std::max(getBestHypothesis().getPositionConfidence(), 0.5f);

  std::vector<HypothesisBase> additionalHypotheses;
  if (PoseGenerator::getPoseFromPenaltyCrossAndLine(theFieldDimensions, theCLIPFieldLinesPercept, thePenaltyCrossPercept, pose, poseConfidence))
  {
    float confidence = bestHypoConfidence * poseConfidence*parameters.spawning.penaltyCrossBaseConfidence;
    addPoseToHypothesisVector(pose, additionalHypotheses, confidence);
  }

  // Add hypotheses to the system
  for (auto &hyp : additionalHypotheses)
  {
    poseHypotheses.push_back(hyp.pose, hyp.positionConfidence, hyp.symmetryConfidence, theFrameInfo.time, parameters);
  }
  return !additionalHypotheses.empty();
}

bool SelfLocator2017::addNewHypothesesFromCenterCirleAndLine()
{
  // Add new hypotheses by CenterCircle and it's CenterLine
  // it gives an absolute Pose
  const bool isGoalKeeper = (theBehaviorData.role == BehaviorData::keeper);
  if (isGoalKeeper) return false; // Do not use for goalie

  Pose2f pose;
  float poseConfidence = 0.f;
  float bestHypoConfidence = std::max(getBestHypothesis().getPositionConfidence(), 0.5f);

  std::vector<HypothesisBase> additionalHypotheses;
  if (PoseGenerator::getPoseFromCenterCircleAndCenterLine(theCLIPFieldLinesPercept, theCLIPCenterCirclePercept, pose, poseConfidence) > 0)
  {
    float confidence = bestHypoConfidence * poseConfidence*parameters.spawning.centerCircleBaseConfidence;
    addPoseToHypothesisVector(pose, additionalHypotheses, confidence);
  }

  // Add hypotheses to the system
  for (auto &hyp : additionalHypotheses)
  {
    poseHypotheses.push_back(hyp.pose, hyp.positionConfidence, hyp.symmetryConfidence, theFrameInfo.time, parameters);
  }
  return !additionalHypotheses.empty();
}

bool SelfLocator2017::addNewHypothesesFromGoal()
{
  const bool isGoalKeeper = (theBehaviorData.role == BehaviorData::keeper);
  if (isGoalKeeper) return false; // Do not use for goalie

  std::vector<HypothesisBase> additionalHypotheses;
  if (theCLIPGoalPercept.numberOfGoalPosts >= 2)
  {
    float poseConfidence = 0.f;
    Vector2f leftPost = Vector2f();
    Vector2f rightPost = Vector2f();
    Pose2f pose;
    float bestHypoConfidence = std::max(getBestHypothesis().getPositionConfidence(), 0.5f);

    if (getLeftAndRightGoalPostFromGoalPercept(theCLIPGoalPercept, leftPost, rightPost, poseConfidence)
      && PoseGenerator::getPoseFromGoalObservation(theFieldDimensions, leftPost, rightPost, pose))
    {
      float confidence = bestHypoConfidence * poseConfidence*parameters.spawning.goalBaseConfidence;
      addPoseToHypothesisVector(pose, additionalHypotheses, confidence);
    }
  }

  // Add hypotheses to the system
  for (auto &hyp : additionalHypotheses)
  {
    poseHypotheses.push_back(hyp.pose, hyp.positionConfidence, hyp.symmetryConfidence, theFrameInfo.time, parameters);
  }
  return !additionalHypotheses.empty();
}

bool SelfLocator2017::addNewHypothesesFromLandmark()
{
  return addNewHypothesesFromPenaltyCrossLine()
    || addNewHypothesesFromCenterCirleAndLine()
    || addNewHypothesesFromGoal();
}

bool SelfLocator2017::addNewHypotheses()
{
  bool added = false;

  // Add hypothesis in initial all the time
  if (theGameInfo.state == STATE_INITIAL)
  {
    addHypothesesOnInitialKickoffPositions();
    added = true;
  }

  if (theGameInfo.state == STATE_SET)
  {
    if (!gotPickedUpInSet && theFrameInfo.getTimeSince(timeStampWhenEnteredSetState) > 3000)
    {
      accDataBuffer.push_front(theInertialData.acc);

      if (accDataBuffer.full())
      {
        float minZ = std::numeric_limits<float>::max(), maxZ = std::numeric_limits<float>::min();
        for (const auto &data : accDataBuffer)
        {
          if (data.z() < minZ)
            minZ = data.z();

          if (data.z() > maxZ)
            maxZ = data.z();
        }

        gotPickedUpInSet = std::abs(maxZ - minZ) > parameters.spawning.accZforPickedUpDifference;
      }
    }
  }
  else
  {
    timeStampWhenEnteredSetState = theFrameInfo.time;
    gotPickedUpInSet = false;
    accDataBuffer.clear();
  }

  switch (localizationState)
  {
  case positionLost:
    // Add hypothesis for set state
    if (theGameInfo.state == STATE_SET)
    {
      addHypothesesOnManualPositioningPositions();
      added = true;
    }
    else if (parameters.spawning.landmarkBasedHypothesesSpawn & SelfLocator2017Parameters::Spawning::spawnIfPositionLost)
    {
      added |= addNewHypothesesFromLineMatches();
      added |= addNewHypothesesFromLandmark();
    }
    break;

  case positionTracking:
    // Add hypothesis for set state
    if (theGameInfo.state == STATE_SET && (gotPickedUpInSet || Global::getSettings().gameMode == Settings::penaltyShootout))
    {
      addHypothesesOnManualPositioningPositions();
      added = true;
    }
    else if (parameters.spawning.landmarkBasedHypothesesSpawn & SelfLocator2017Parameters::Spawning::spawnIfPositionTracking
      && getBestHypothesis().getPositionConfidence() < parameters.spawning.spawnWhilePositionTrackingWhenBestConfidenceBelowThisThreshold)
    {
      added |= addNewHypothesesFromLineMatches();
      added |= addNewHypothesesFromLandmark();
    }
    break;

  default:
    void(0); //nothing todo;
  }

  added |= addNewHypothesesWhenSymmetryLost();
  return added;
}


float SelfLocator2017::getRobotPoseValidity(const PoseHypothesis2017 & poseHypothesis)
{
  float validity = 0;

  switch (localizationState)
  {
  case positionTracking:
    validity = poseHypothesis.getPositionConfidence();
    break;
  case positionLost:
    validity = 0;
    break;
  case fallenDown:
    validity = poseHypothesis.getPositionConfidence();
    break;
  default:
    // should never happen!
    break;
  }
  return validity;
}


inline void SelfLocator2017::initDebugging()
{
  //DECLARE_DEBUG_DRAWING("module:SelfLocator2017:hypotheses", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:SelfLocator2017:correspondences", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:SelfLocator2017:LocalizationState", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:SelfLocator2017:poseFromCenterCircle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:SelfLocator2017:poseFromPenaltyCross", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:SelfLocator2017:GaussianTools:covarianceEllipse", "drawingOnField");
}

void SelfLocator2017::doGlobalDebugging()
{
  ; // IMPORTANT!!! Do not remove!

  if (poseHypotheses.empty())
  {
    DRAWTEXT("module:SelfLocator2017:LocalizationState", -500, -2200, 50, ColorRGBA(255, 255, 255), "LocalizationState: no more hypotheses! Should never happen!");
    return;
  }

  const PoseHypothesis2017 &bestHyp = getBestHypothesis();
  if (!bestHyp.isInsideCarpet(theFieldDimensions))
    return;

  /* NOT NEEDED; Can be observed by representation
  COMPLEX_DRAWING("module:SelfLocator2017:hypotheses")
  {
    for (auto &hypothesis : poseHypotheses)
    {
      float factor = std::max(std::min(theFrameInfo.getTimeSince(hypothesis->getCreationTime()) / parameters.debugging.durationHighlightAddedHypothesis, 1.f), 0.f);
      hypothesis->draw(ColorRGBA(255, static_cast<unsigned char>(factor * 255), static_cast<unsigned char>(factor * 255), static_cast<unsigned char>(255.0*hypothesis->getPositionConfidence())));
    }

    //recolor the best
    bestHyp.draw(ColorRGBA(255, 0, 0, (unsigned char)(255.0*bestHyp.getPositionConfidence())));
  }
  */

  switch (localizationState)
  {
  case positionTracking:
    DRAWTEXT("module:SelfLocator2017:LocalizationState", -1000, -3500, 100, ColorRGBA(255, 255, 255), "LocalizationState: positionTracking");
    break;
  case positionLost:
    DRAWTEXT("module:SelfLocator2017:LocalizationState", -1000, -3500, 100, ColorRGBA(255, 255, 255), "LocalizationState: positionLost");
    break;
  case fallenDown:
    DRAWTEXT("module:SelfLocator2017:LocalizationState", -1000, -3500, 100, ColorRGBA(255, 255, 255), "LocalizationState: fallenDown");
    break;
  case penalized:
    DRAWTEXT("module:SelfLocator2017:LocalizationState", -1000, -3500, 100, ColorRGBA(255, 255, 255), "LocalizationState: penalized");
    break;
  default:
    DRAWTEXT("module:SelfLocator2017:LocalizationState", -1000, -3500, 100, ColorRGBA(255, 255, 255), "LocalizationState: default... should never happen!");
    // should never happen!
    break;
  }
}

void SelfLocator2017::generateOutputData()
{
  // Get best hypothesis
  const PoseHypothesis2017 &bestRobotPoseHypothesis = getBestHypothesis();

  // Fill robot pose
  bestRobotPoseHypothesis.getRobotPose(m_robotPose);
  m_robotPose.validity = getRobotPoseValidity(bestRobotPoseHypothesis);
  m_robotPose.symmetry = bestRobotPoseHypothesis.getSymmetryConfidence();
  if (m_robotPose.validity > parameters.spawning.spawnWhilePositionTrackingWhenBestConfidenceBelowThisThreshold)
  {
    foundGoodPosition = true;
    timeStampLastGoodPosition = theFrameInfo.time;
    lastGoodPosition = m_robotPose;
    distanceTraveledFromLastGoodPosition = Pose2f();
  }
  else if (foundGoodPosition)
  {
    distanceTraveledFromLastGoodPosition += (theOdometryData - lastOdometryData);
  }

  // Fill side confidence
  m_sideConfidence.sideConfidence = bestRobotPoseHypothesis.getSymmetryConfidence();

  // Helper
  GaussianDistribution3D gd;

  // Fill best hypothesis
  bestRobotPoseHypothesis.extractGaussianDistribution3DFromStateEstimation(gd);
  m_robotPoseHypothesis.validity = getRobotPoseValidity(bestRobotPoseHypothesis);
  m_robotPoseHypothesis.covariance = gd.covariance;
  m_robotPoseHypothesis.robotPoseReceivedMeasurementUpdate = bestRobotPoseHypothesis.performedSensorUpdate();
  bestRobotPoseHypothesis.getRobotPose((Pose2f&)m_robotPoseHypothesis, m_robotPoseHypothesis.covariance);

  // Fill all hypotheses
  m_robotPoseHypotheses.hypotheses.clear();
  for (const auto &hypothesis : poseHypotheses)
  {
    RobotPoseHypothesis ph;
    hypothesis->extractGaussianDistribution3DFromStateEstimation(gd);
    ph.validity = getRobotPoseValidity(*hypothesis);
    ph.symmetry = hypothesis->getSymmetryConfidence();
    ph.covariance = gd.covariance;
    ph.robotPoseReceivedMeasurementUpdate = hypothesis->performedSensorUpdate();
    hypothesis->getRobotPose((Pose2f&)ph, ph.covariance);
    m_robotPoseHypotheses.hypotheses.push_back(ph);
  }
}


const PoseHypothesis2017 &SelfLocator2017::getBestHypothesis()
{
  PoseHypotheses2017::element_type current = 0;
  PoseHypotheses2017::element_type best = 0;
  float bestConfidence = 0;

  // Search for overall best and current hypothesis
  for (const auto &hypothesis : poseHypotheses)
  {
    float confidence = hypothesis->getPositionConfidence();
    if (!best || confidence > bestConfidence)
    {
      best = hypothesis;
      bestConfidence = confidence;
    }
    if (hypothesis->getUniqueId() == lastBestHypothesisUniqueId)
    {
      current = hypothesis;
    }
  }

  // Check if current is not alive anymore OR
  if (!current ||
    // if overall best is not currently used hypothesis
    (best != current &&
      // Check if best confidence is much better than currently used hypothesis
    (bestConfidence - current->getPositionConfidence()) > parameters.processUpdate.positionConfidenceHysteresisForKeepingBestHypothesis))
  {
    if (lastBestHypothesisUniqueId && (theFrameInfo.getTimeSince(lastNonPlayingTimeStamp) > 15000))
    {
      std::underlying_type<SideConfidence::ConfidenceState>::type cs = m_sideConfidence.confidenceState;

      cs = std::min(std::underlying_type<SideConfidence::ConfidenceState>::type(SideConfidence::numOfConfidenceStates - 1),
        std::max(std::underlying_type<SideConfidence::ConfidenceState>::type(SideConfidence::CONFIDENT), ++cs)); // The lower the more confident -> So increase if best hypo switched

      m_sideConfidence.confidenceState = static_cast<SideConfidence::ConfidenceState>(cs);
    }

    // Set new best hypothesis
    lastBestHypothesisUniqueId = best->getUniqueId();

    return *best;
  }
  else
  {
    return *current;
  }
}

void SelfLocator2017::addPoseToHypothesisVector(const Pose2f &pose, std::vector<HypothesisBase> &additionalHypotheses, const float &poseConfidence)
{
  const PoseHypothesis2017 &bestHypo = getBestHypothesis();

  const Pose2f symmetricPose = getSymmetricPoseOnField(pose), *nearestPose = 0;

  float symmetryConfidence = 0;

  // Determine minimum position confidence for hypothesis to check for closest position
  const float minConfidence = std::max(
    parameters.localizationStateUpdate.positionLostWhenBestConfidenceBelowThisThreshold,
    bestHypo.getPositionConfidence() - parameters.spawning.confidenceIntervalForCheckOfHypothesis);

  double bestDistance = -1;
  for (auto &hypo : poseHypotheses)
  {
    // Check hypothesis qualifies for check
    if (hypo->getPositionConfidence() >= minConfidence)
    {
      Pose2f hypoPose;
      hypo->getRobotPose(hypoPose);
      const float distanceToPose = (pose - hypoPose).translation.norm();
      const float distanceToSymPose = (symmetricPose - hypoPose).translation.norm();

      // Check closest pose for this hypothesis
      const Pose2f *current_closest;
      float distance;
      if (distanceToPose <= distanceToSymPose)
      {
        distance = distanceToPose;
        current_closest = &pose;
      }
      else
      {
        distance = distanceToSymPose;
        current_closest = &symmetricPose;
      }

      // Do not spawn if pose is on other side of the field but we are still entering the field (initial or penalized)
      const int limit = static_cast<int>(parameters.spawning.limitSpawningToOwnSideTimeout);
      if (((theFrameInfo.getTimeSince(timeStampFirstReadyState) < limit) || (theFrameInfo.getTimeSince(unpenalizedTimeStamp) < limit)) &&
        !isPoseOnOwnFieldSide(*current_closest))
        continue;

      // Check if pose is closer than best distance
      if (bestDistance < 0 || (distance < bestDistance))
      {
        bestDistance = distance;
        nearestPose = current_closest;
        symmetryConfidence = hypo->getSymmetryConfidence();
      }
    }
  }

  // Check if odometry from is better than others
  if (parameters.spawning.useOdometryForSpawning && foundGoodPosition)
  {
    Pose2f odometryPose = (lastGoodPosition + distanceTraveledFromLastGoodPosition);
    const float distanceToPose = (pose - odometryPose).translation.norm();
    const float distanceToSymPose = (symmetricPose - odometryPose).translation.norm();

    // Check closest pose for this hypothesis
    const Pose2f *current_closest = (distanceToPose <= distanceToSymPose) ? &pose : &symmetricPose;

    if (bestDistance < 0 || (odometryPose - *current_closest).translation.norm() < bestDistance)
    {
      nearestPose = current_closest;
      symmetryConfidence = parameters.localizationStateUpdate.symmetryFoundAgainWhenBestConfidenceAboveThisThreshold * bestHypo.getSymmetryConfidence();
    }
  }

  // Check if we found a decent pose to spawn
  if (nearestPose)
  {
    // Only spawn if confidence higher than lost threshold
    if (poseConfidence >= parameters.localizationStateUpdate.positionLostWhenBestConfidenceBelowThisThreshold)
      additionalHypotheses.push_back(HypothesisBase(*nearestPose, poseConfidence, symmetryConfidence));
  }
  else if (localizationState == positionLost)
  {
    // Add the normal pose if loca has been lost
    additionalHypotheses.push_back(HypothesisBase(pose,
      parameters.localizationStateUpdate.positionLostWhenBestConfidenceBelowThisThreshold,
      parameters.localizationStateUpdate.symmetryLostWhenBestConfidenceBelowThisThreshold));
  }
}


MAKE_MODULE(SelfLocator2017, modeling)
