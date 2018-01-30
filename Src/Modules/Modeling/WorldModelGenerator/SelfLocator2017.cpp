
#include "SelfLocator2017.h"

#include "Tools/Math/GaussianDistribution2D.h"

/*---------------------------- class SelfLocator2017 PUBLIC methods ------------------------------*/

namespace
{
  /** Helper functions **/
  Pose2f getSymmetricPoseOnField(const Pose2f& pose)
  {
    return Pose2f(Angle::normalize(pose.rotation + pi), pose.translation * -1);
  }


  bool isPoseOnSameFieldSide(const Pose2f& p1, const Pose2f& p2)
  {
    return  (((p1.translation.x() <= 0) && (p2.translation.x() <= 0))
      ||
      ((p1.translation.x() >= 0) && (p2.translation.x() >= 0))
      );
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

  const PoseHypothesis2017 &bestHyp = getBestHypothesis();
  bestHyp.getRobotPose(robotPose);
  robotPose.validity = getRobotPoseValidity(bestHyp);
  if (localizationState != positionLost)
  {
    foundGoodPosition = true;
    timeStampLastGoodPosition = theFrameInfo.time;
    lastGoodPosition = robotPose;
    distanceTraveledFromLastGoodPosition = Pose2f();
  }
  else
    distanceTraveledFromLastGoodPosition += (theOdometryData - lastOdometryData);
}

void SelfLocator2017::update(SideConfidence& confidence)
{
  executeCommonCode();

  const PoseHypothesis2017 &bestHyp = getBestHypothesis();
  confidence.sideConfidence = static_cast<float>(bestHyp.getSymmetryConfidence());
  confidence.confidenceState = bestHyp.getSymmetryConfidenceState();
}


void SelfLocator2017::update(RobotPoseHypothesis& robotPoseHypothesis)
{
  executeCommonCode();

  const PoseHypothesis2017 &bestRobotPoseHypothesis = getBestHypothesis();

  RobotPoseHypothesis ph;
  ph.validity = getRobotPoseValidity(bestRobotPoseHypothesis);
  bestRobotPoseHypothesis.getRobotPose((Pose2f&)ph, ph.covariance);
  robotPoseHypothesis.covariance = ph.covariance;
  robotPoseHypothesis.robotPoseReceivedMeasurementUpdate = ph.robotPoseReceivedMeasurementUpdate;
  robotPoseHypothesis.translation = ph.translation;
  robotPoseHypothesis.rotation = ph.rotation;
  robotPoseHypothesis.validity = ph.validity;
}

void SelfLocator2017::update(RobotPoseHypotheses& robotPoseHypotheses)
{
  executeCommonCode();

  robotPoseHypotheses.hypotheses.clear();
  //  getBestHypothesis(robotPoseHypotheses.indexOfBestHypothesis);
  for (auto &hypothesis : poseHypotheses)
  {
    RobotPoseHypothesis ph;
    hypothesis->getRobotPose((Pose2f&)ph, ph.covariance);
    ph.robotPoseReceivedMeasurementUpdate = true;
    ph.validity = getRobotPoseValidity(*hypothesis);
    robotPoseHypotheses.hypotheses.push_back(ph);
  }
}


void SelfLocator2017::predictHypotheses()
{
  Pose2f odometryDelta = theOdometryData - lastOdometryData;
  lastOdometryData = theOdometryData;

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
    if (theGameInfo.state == STATE_PLAYING && parameters.symmetryUpdate.updateWithRemoteModels) // no ball on the field in READY! (which is the strongest/only symmetry indicator)
    {
      hypothesis->updateSymmetryByComparingRemoteToLocalModels(theBallModel, theRemoteBallModel, theLocalRobotMap, theRemoteRobotMap, theTeammateData, theFrameInfo, parameters);
    }
    else
    {
      hypothesis->setSymmetryConfidence(1., parameters.localizationStateUpdate);
    }

    // no symmetrie loss for goalie. ever.
    if (theBehaviorData.role == BehaviorData::keeper)
    {
      hypothesis->setSymmetryConfidence(1., parameters.localizationStateUpdate);
    }
  }
}


void SelfLocator2017::executeCommonCode()
{
  if (!initialized)
  {
    addHypothesesOnInitialKickoffPositions();
    addHypothesesOnManualPositioningPositions();
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
  double sc = bestHypothesis.getSymmetryConfidence();

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

  // re-add latest best hypothesis if no more are left
  if (poseHypotheses.empty())
  {
    OUTPUT_ERROR("SelfLocator2017 pruned all hypotheses because of their state -> re-add best one!");

    // re-add best one from before
    poseHypotheses.push_back(pose, pc * .8f, sc, theFrameInfo.time, parameters);
  }
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


void SelfLocator2017::pruneHypothesesGoalie()
{
  // Let's simply assume that the keeper will never be in the opponent half!  :)
  if (theBehaviorData.role == BehaviorData::keeper)
  {
    PoseHypotheses2017::iterator it = poseHypotheses.begin();
    while (it != poseHypotheses.end() && poseHypotheses.size() > 1)
    {
      Pose2f keeperPose;
      (*it)->getRobotPose(keeperPose);
      if (keeperPose.translation.x() > 0) // keeper is NOT in the opponent half. Per definition.
      {
        it = poseHypotheses.erase(it);
      }
      else
      {
        it++;
      }
    }
  } // end of keeper specific pruning
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

void SelfLocator2017::pruneHypothesesOutsideCarpet()
{
  for (PoseHypotheses2017::iterator it = poseHypotheses.begin();
    it != poseHypotheses.end();)
  {
    const PoseHypothesis2017 *hypothesis = (*it);
    if (!hypothesis->isInsideCarpet(theFieldDimensions))
    {
      if (parameters.debugging.displayWarnings)
        OUTPUT_WARNING("SelfLocator2017 detected hypothesis position that is outside of carpet -> deleting hypothesis");

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

void SelfLocator2017::pruneHypothesesWithInvalidValues()
{
  for (PoseHypotheses2017::iterator it = poseHypotheses.begin();
    it != poseHypotheses.end();)
  {
    const PoseHypothesis2017 *hypothesis = (*it);
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
    return;

  pruneHypothesesGoalie(); // special case: no position on opp side
  pruneHypothesesOutsideCarpet();
  pruneHypothesesWithInvalidValues();

  // in penalty shootout the striker will never reach the own half because of manually placement
  if (Global::getSettings().gameMode == Settings::penaltyShootout && theBehaviorData.role != BehaviorData::keeper)
    pruneHypothesesInOwnHalf();

  // first, as a safety measure, delete everything which comes too close to the symmetric position of our best hypothesis
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
        && (symmetryLost || fabs(poseHypotheses[i]->getSymmetryConfidence() - poseHypotheses[j]->getSymmetryConfidence()) > 0.2))
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
        // just set the confidence of the less confident one to zero, 
        // it will be deleted automatically in the next step
        double newSymmetryConfidence = std::max(poseHypotheses[k]->getSymmetryConfidence(), poseHypotheses[j]->getSymmetryConfidence());
        poseHypotheses[k]->setSymmetryConfidence(newSymmetryConfidence, parameters.localizationStateUpdate);
        poseHypotheses[j]->setSymmetryConfidence(newSymmetryConfidence, parameters.localizationStateUpdate);
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
  if(parameters.localizationStateUpdate.symmetryLostWhenFallDownInCenterCircle)
  {
    // Handle fall down in center circle
    const PoseHypothesis2017 &bestHyp = getBestHypothesis();
    Pose2f p;
    bestHyp.getRobotPose(p);
    if (p.translation.norm() < parameters.localizationStateUpdate.unknownSymmetryRadiusAroundCenter)
    {
      for (auto &hypothesis : poseHypotheses)
      {
        hypothesis->setSymmetryConfidence(0, parameters.localizationStateUpdate); //only case where symmetry is set to 0
      }
    }
  }
}

void SelfLocator2017::handleGettingUpAfterFallDown()
{
  // Copy current hypotheses to iterate over them
  // Cannot be done on poseHypotheses directly as new hypotheses get added to it!
  std::vector<PoseHypothesis2017*> currentHypotheses;
  for (auto &hypothesis : poseHypotheses)
  {
    currentHypotheses.push_back(hypothesis);
  }

  // if we fell on the side, we might end up with +/- 90° orientation errors,
  // so add those hypotheses...
  for (auto &hypothesis : currentHypotheses)
  {
    hypothesis->scalePositionConfidence(parameters.sensorUpdate.confidenceScaleFactorAfterFallDown); // = 0.75
    for (unsigned char j = 1; j <= parameters.spawning.noAdditionalHypothesisAfterFallDown; ++j)
    {
      poseHypotheses.push_back(*hypothesis, theFrameInfo.time); // copy hypotheses
      PoseHypothesis2017 *p1 = poseHypotheses.back(); // Get new hypothesis

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
        (*i)->setSymmetryConfidence(parameters.spawning.symmetryConfidenceWhenPositionedManually, parameters.localizationStateUpdate);
        i++;
      }
    }
  }

  addHypothesesOnManualPositioningPositions();
  localizationState = positionTracking; // since we know our symmetry for sure
}

void SelfLocator2017::handleInitialState()
{
  timeStampFirstReadyState = theFrameInfo.time;
  addHypothesesOnInitialKickoffPositions();
  localizationState = positionTracking; // since we know our symmetry for sure
}


void SelfLocator2017::addHypothesesOnManualPositioningPositions()
{
  bool ownKickoff = theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber;
  // special position for goalie
  const bool isGoalKeeper = (theBehaviorData.role == BehaviorData::keeper);
  if (isGoalKeeper)
  {
    poseHypotheses.push_back(Pose2f(0, positionsByRules.goaliePosition),
      parameters.spawning.positionConfidenceWhenPositionedManuallyForGoalKeeper, parameters.spawning.symmetryConfidenceWhenPositionedManually,
      theFrameInfo.time, parameters);
  }
  // one of the field positions for field players
  else
  {
    // penalty shootout
    if (Global::getSettings().gameMode == Settings::penaltyShootout)
    {
      poseHypotheses.push_back(Pose2f(0, positionsByRules.penaltyShootOutPosition),
        parameters.spawning.positionConfidenceWhenPositionedManually,
        parameters.spawning.symmetryConfidenceWhenPositionedManually,
        theFrameInfo.time,
        parameters);
    }
    else
    {
      for (auto &position : (ownKickoff ? positionsByRules.fieldPlayerPositionsOwnKickoff : positionsByRules.fieldPlayerPositionsOppKickoff))
      {
        poseHypotheses.push_back(Pose2f(0, position),
          parameters.spawning.positionConfidenceWhenPositionedManually, parameters.spawning.symmetryConfidenceWhenPositionedManually,
          theFrameInfo.time, parameters);
      }
    }
  }
}

void SelfLocator2017::addHypothesesOnInitialKickoffPositions()
{
  if (Global::getSettings().gameMode != Settings::penaltyShootout)
  {
    poseHypotheses.push_back(Pose2f(-pi_2, theFieldDimensions.xPosOwnGroundline / 4.f, theFieldDimensions.yPosLeftSideline), parameters.spawning.positionConfidenceWhenPositionedManually, parameters.spawning.symmetryConfidenceWhenPositionedManually, theFrameInfo.time, parameters);
    poseHypotheses.push_back(Pose2f(pi_2, theFieldDimensions.xPosOwnGroundline / 4.f, theFieldDimensions.yPosRightSideline), parameters.spawning.positionConfidenceWhenPositionedManually, parameters.spawning.symmetryConfidenceWhenPositionedManually, theFrameInfo.time, parameters);
    poseHypotheses.push_back(Pose2f(-pi_2, theFieldDimensions.xPosOwnGroundline / 2.f, theFieldDimensions.yPosLeftSideline), parameters.spawning.positionConfidenceWhenPositionedManually, parameters.spawning.symmetryConfidenceWhenPositionedManually, theFrameInfo.time, parameters);
    poseHypotheses.push_back(Pose2f(pi_2, theFieldDimensions.xPosOwnGroundline / 2.f, theFieldDimensions.yPosRightSideline), parameters.spawning.positionConfidenceWhenPositionedManually, parameters.spawning.symmetryConfidenceWhenPositionedManually, theFrameInfo.time, parameters);
    poseHypotheses.push_back(Pose2f(-pi_2, 3 * theFieldDimensions.xPosOwnGroundline / 4.f, theFieldDimensions.yPosLeftSideline), parameters.spawning.positionConfidenceWhenPositionedManually, parameters.spawning.symmetryConfidenceWhenPositionedManually, theFrameInfo.time, parameters);
    poseHypotheses.push_back(Pose2f(pi_2, 3 * theFieldDimensions.xPosOwnGroundline / 4.f, theFieldDimensions.yPosRightSideline), parameters.spawning.positionConfidenceWhenPositionedManually, parameters.spawning.symmetryConfidenceWhenPositionedManually, theFrameInfo.time, parameters);
  }
  else
  {
    poseHypotheses.push_back(Pose2f(0, positionsByRules.penaltyShootOutPosition),
      parameters.spawning.positionConfidenceWhenPositionedManually,
      parameters.spawning.symmetryConfidenceWhenPositionedManually,
      theFrameInfo.time,
      parameters);
  }
}

void SelfLocator2017::addHypothesesOnPenaltyPositions(float newPositionConfidence)
{
  if (Global::getSettings().gameMode != Settings::penaltyShootout)
  {
    for (auto &position : positionsByRules.penaltyPositions)
    {
      poseHypotheses.push_back(Pose2f(pi_2, position.x(), theFieldDimensions.yPosRightSideline), newPositionConfidence,
        parameters.spawning.symmetryConfidenceWhenPositionedManually,
        theFrameInfo.time,
        parameters);
      poseHypotheses.push_back(Pose2f(-pi_2, position.x(), theFieldDimensions.yPosLeftSideline), newPositionConfidence,
        parameters.spawning.symmetryConfidenceWhenPositionedManually,
        theFrameInfo.time,
        parameters);
    }
  }
}

bool SelfLocator2017::addNewHypothesesWhenSymmetryLost()
{
  // Copy current hypotheses to iterate over them
  // Cannot be done on poseHypotheses directly as new hypotheses get added to it!
  std::vector<PoseHypothesis2017*> currentHypotheses;
  for (auto &hypothesis : poseHypotheses)
  {
    currentHypotheses.push_back(hypothesis);
  }

  bool added = false;
  //for each Hypotheses that has a lost symmetry ... duplicate it?
  for (auto &hypothesis : currentHypotheses)
  {
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
        hypothesis->getSymmetryConfidence(),
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

  std::vector<HypothesesBase> additionalHypotheses;
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

  std::vector<HypothesesBase> additionalHypotheses;
  if (PoseGenerator::getPoseFromPenaltyCrossAndLine(theFieldDimensions, theCLIPFieldLinesPercept, thePenaltyCrossPercept, pose, poseConfidence))
  {
    float confidence = bestHypoConfidence*poseConfidence*parameters.spawning.penaltyCrossBaseConfidence;
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

  std::vector<HypothesesBase> additionalHypotheses;
  if (PoseGenerator::getPoseFromCenterCircleAndCenterLine(theCLIPFieldLinesPercept, theCLIPCenterCirclePercept, pose, poseConfidence) > 0)
  {
    float confidence = bestHypoConfidence*poseConfidence*parameters.spawning.centerCircleBaseConfidence;
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

  std::vector<HypothesesBase> additionalHypotheses;
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
      float confidence = bestHypoConfidence*poseConfidence*parameters.spawning.goalBaseConfidence;
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

  switch (localizationState)
  {
  case positionLost:
    if (parameters.spawning.landmarkBasedHypothesesSpawn & SelfLocator2017Parameters::Spawning::spawnIfPositionLost)
    {
      added |= addNewHypothesesFromLineMatches();
      added |= addNewHypothesesFromLandmark();
    }
    break;

  case positionTracking:
    if (parameters.spawning.landmarkBasedHypothesesSpawn & SelfLocator2017Parameters::Spawning::spawnIfPositionTracking
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
  DECLARE_DEBUG_DRAWING("module:SelfLocator2017:hypotheses", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:SelfLocator2017:correspondences", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:SelfLocator2017:LocalizationState", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:SelfLocator2017:poseFromCenterCircle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:SelfLocator2017:poseFromPenaltyCross", "drawingOnField");

  MODIFY("module:SelfLocator2017:parameters", parameters);
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

  COMPLEX_DRAWING("module:SelfLocator2017:hypotheses")
  {
    for (auto &hypothesis : poseHypotheses)
    {
      float factor = std::max(std::min(theFrameInfo.getTimeSince(hypothesis->getCreationTime()) / parameters.debugging.durationHighlightAddedHypothesis, 1.f), 0.f);
      hypothesis->draw(ColorRGBA(255, static_cast<unsigned char>(factor * 255), static_cast<unsigned char>(factor * 255), (unsigned char)(255.0*hypothesis->getPositionConfidence())));
    }

    //recolor the best
    bestHyp.draw(ColorRGBA(255, 0, 0, (unsigned char)(255.0*bestHyp.getPositionConfidence())));
  }

  switch (localizationState)
  {
  case positionTracking:
    DRAWTEXT("module:SelfLocator2017:LocalizationState", -500, -2200, 50, ColorRGBA(255, 255, 255), "LocalizationState: positionTracking");
    break;
  case positionLost:
    DRAWTEXT("module:SelfLocator2017:LocalizationState", -500, -2200, 50, ColorRGBA(255, 255, 255), "LocalizationState: positionLost");
    break;
  case fallenDown:
    DRAWTEXT("module:SelfLocator2017:LocalizationState", -500, -2200, 50, ColorRGBA(255, 255, 255), "LocalizationState: fallenDown");
    break;
  case penalized:
    DRAWTEXT("module:SelfLocator2017:LocalizationState", -500, -2200, 50, ColorRGBA(255, 255, 255), "LocalizationState: penalized");
    break;
  default:
    DRAWTEXT("module:SelfLocator2017:LocalizationState", -500, -2200, 50, ColorRGBA(255, 255, 255), "LocalizationState: default... should never happen!");
    // should never happen!
    break;
  }
}


const PoseHypothesis2017 &SelfLocator2017::getBestHypothesis()
{
  const PoseHypothesis2017 *current = 0;
  const PoseHypothesis2017 *best = 0;
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
    // Set new best hypothesis
    lastBestHypothesisUniqueId = best->getUniqueId();
    return *best;
  }
  else
  {
    return *current;
  }
}

void SelfLocator2017::addPoseToHypothesisVector(const Pose2f &pose, std::vector<HypothesesBase> &additionalHypotheses,
  const float &poseConfidence)
{
  const bool isLost = localizationState == positionLost;
  Pose2f symmetricPose = getSymmetricPoseOnField(pose);

  const PoseHypothesis2017 &bestHypo = getBestHypothesis();

  Pose2f nearestPose;
  bool allowToSpawn = false;
  double symmetryConfidence = 0;

  if (parameters.spawning.confidenceIntervalForCheckOfHypothesis < 0)
  {
    // Legacy behaviour
    Pose2f hypoPose;
    bestHypo.getRobotPose(hypoPose);
    float distanceToPose = (pose - hypoPose).translation.norm();
    float distanceToSymPose = (symmetricPose - hypoPose).translation.norm();
    nearestPose = (distanceToPose <= distanceToSymPose) ? pose : symmetricPose;

    const int timeSinceLastGoodPosition = theFrameInfo.getTimeSince(timeStampLastGoodPosition);

    allowToSpawn = isPoseOnSameFieldSide(hypoPose, nearestPose);
    allowToSpawn |= ((theFrameInfo.getTimeSince(timeStampFirstReadyState) > 10000)
                     && (theFrameInfo.getTimeSince(unpenalizedTimeStamp) > 10000));

    const float distTraveled = distanceTraveledFromLastGoodPosition.translation.norm();
    const float distToPose = (lastGoodPosition - nearestPose).translation.norm();

    allowToSpawn &= //!parameters.useOdometryForSpawning ||
        (timeSinceLastGoodPosition > 20000 || (std::abs(distTraveled - distToPose) < std::min(2000., timeSinceLastGoodPosition * .5)));

    symmetryConfidence = bestHypo.getSymmetryConfidence();
  }
  else
  {
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

        // Check if pose is closer than best distance
        if (bestDistance < 0 || (distanceToPose < bestDistance))
        {
          bestDistance = distanceToPose;
          nearestPose = pose;
          symmetryConfidence = hypo->getSymmetryConfidence();
        }
        // Check if symmetric pose is closer than best distance
        if (distanceToSymPose < bestDistance)
        {
          bestDistance = distanceToSymPose;
          nearestPose = symmetricPose;
          symmetryConfidence = hypo->getSymmetryConfidence();
        }
      }
    }
    allowToSpawn = bestDistance > 0;
  }

  if (isLost || allowToSpawn)
  {
    // Only spawn if confidence higher than lost threshold
    const float &pc = isLost ? parameters.localizationStateUpdate.positionLostWhenBestConfidenceBelowThisThreshold : poseConfidence;
    if (pc >= parameters.localizationStateUpdate.positionLostWhenBestConfidenceBelowThisThreshold)
      additionalHypotheses.push_back(HypothesesBase(nearestPose, pc, symmetryConfidence));
  }
}


MAKE_MODULE(SelfLocator2017, modeling)
