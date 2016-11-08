
#include "SelfLocator2012.h"

#include "Tools/Math/GaussianDistribution2D.h"

/*---------------------------- class SelfLocator2012 PUBLIC methods ------------------------------*/

namespace
{
  /** Magic Numbers - TODO: move to params **/
  const float  positionConfidenceWhenPositionedManually = 0.2f;
  const float  positionConfidenceWhenPositionedManuallyForGoalKeeper = 0.3f;
  //const float  positionConfidenceWhenPositionLost = 0.1f;
  const float  positionConfidenceWhenPositionTracking = 0.2f;
  const float positionConfidenceForKeepingBestHypothesis = 0.8f;
  //const float  symmetryConfidenceWhenPositionLost = 0.1f; //symmetry is bad
  const   size_t noAdditionalHypothesisAfterFallDown = 8;
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

  bool arePosesCloseToEachOther(const Pose2f &p1, const Pose2f &p2, const SelfLocator2012Parameters &parameters)
  {
    return ((p1.translation - p2.translation).norm() < parameters.sensorResetting_maxDistanceForLocalResetting
      &&std::abs(Angle::normalize(p1.rotation - p2.rotation)) < parameters.sensorResetting_maxAngleForLocalResetting);
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


SelfLocator2012::SelfLocator2012() :
lastExecuteTimeStamp(0),
penalizedTimeStamp(0),
unpenalizedTimeStamp(0),
lastPenalty(PENALTY_NONE),
lastNonPlayingTimeStamp(0),
timeStampFirstReadyState(0)
{
  initialized = false;

  localizationState = positionTracking;
  localizationStateAfterGettingUp = localizationState;
  foundGoodPosition = false;
}

SelfLocator2012::~SelfLocator2012()
{
  PoseHypothesis2012::cleanup();
}

void SelfLocator2012::update(RobotPose& robotPose)
{
  executeCommonCode();

  const PoseHypothesis2012 &bestHyp = getBestHypothesis();
  bestHyp.getRobotPose(robotPose);
  robotPose.validity = getRobotPoseValidity(bestHyp);
  if (robotPose.validity > 0.7f)
  {
    foundGoodPosition = true;
    timeStampLastGoodPosition = theFrameInfo.time;
    lastGoodPosition = robotPose;
    distanceTraveledFromLastGoodPosition = Pose2f();
  }
  else
    distanceTraveledFromLastGoodPosition += (theOdometryData - lastOdometryData);
}

void SelfLocator2012::update(SideConfidence& confidence)
{
  executeCommonCode();

  const PoseHypothesis2012 &bestHyp = getBestHypothesis();
  confidence.sideConfidence = (float)bestHyp.getSymmetryConfidence();
}


void SelfLocator2012::update(RobotPoseHypothesis& robotPoseHypothesis)
{
  executeCommonCode();

  const PoseHypothesis2012 &bestRobotPoseHypothesis = getBestHypothesis();

  RobotPoseHypothesis ph;
  ph.validity = getRobotPoseValidity(bestRobotPoseHypothesis);
  bestRobotPoseHypothesis.getRobotPose((Pose2f&)ph, ph.covariance);
  robotPoseHypothesis.covariance = ph.covariance;
  robotPoseHypothesis.robotPoseReceivedMeasurementUpdate = ph.robotPoseReceivedMeasurementUpdate;
  robotPoseHypothesis.translation = ph.translation;
  robotPoseHypothesis.rotation = ph.rotation;
  robotPoseHypothesis.validity = ph.validity;
}

void SelfLocator2012::update(RobotPoseHypotheses& robotPoseHypotheses)
{
  executeCommonCode();

  robotPoseHypotheses.hypotheses.clear();
  //  getBestHypothesis(robotPoseHypotheses.indexOfBestHypothesis);
  for (auto hypothesis : poseHypotheses)
  {
    RobotPoseHypothesis ph;
    hypothesis->getRobotPose((Pose2f&)ph, ph.covariance);
    ph.robotPoseReceivedMeasurementUpdate = true;
    ph.validity = getRobotPoseValidity(*hypothesis);
    robotPoseHypotheses.hypotheses.push_back(ph);
  }
}


void SelfLocator2012::predictHypotheses()
{

  Pose2f odometryDelta = theOdometryData - lastOdometryData;
  lastOdometryData = theOdometryData;

  for (auto hypothesis : poseHypotheses)
  {
    hypothesis->predict(odometryDelta, parameters);
  }
}


void SelfLocator2012::updateHypothesesPositionConfidence()
{
  for (auto hyp : poseHypotheses)
  {
    if (!(hyp->updatePositionConfidenceWithLocalFeaturePerceptionsSpherical(theLineMatchingResult, theCLIPCenterCirclePercept, theCLIPGoalPercept, thePenaltyCrossPercept, theFieldDimensions, theCameraMatrix, theCameraMatrixUpper, parameters)
      || hyp->updatePositionConfidenceWithLocalFeaturePerceptionsInfiniteLines(theLineMatchingResult, theCLIPCenterCirclePercept, theFieldDimensions, parameters)
      || hyp->updatePositionConfidenceWithLocalFeaturePerceptionsWeighted(theCLIPFieldLinesPercept, theCLIPCenterCirclePercept, theCLIPGoalPercept, thePenaltyCrossPercept, theFieldDimensions, parameters)))
    {
      // TODO: Negative update as nothing was seen?
    }
  }
}

void SelfLocator2012::updateHypothesesState()
{
  const PoseHypothesis2012 &ph = getBestHypothesis();
  Pose2f pose;
  ph.getRobotPose(pose);
  float pc = ph.getPositionConfidence();
  double sc = ph.getSymmetryConfidence();
  
  

  PoseHypotheses2012::iterator it = poseHypotheses.begin();
  while (it != poseHypotheses.end())
  {
    PoseHypothesis2012 *hypothesis = *it;
    hypothesis->updateStateWithLocalFeaturePerceptionsSpherical(theLineMatchingResult, theCLIPCenterCirclePercept, theCLIPGoalPercept, thePenaltyCrossPercept, theFieldDimensions, parameters);
    hypothesis->updateStateWithLocalFeaturePerceptionsInfiniteLines(theLineMatchingResult, parameters);
    hypothesis->updateStateRotationWithLocalFieldLines(theCLIPFieldLinesPercept, parameters);

    if (hypothesis->containsInvaidValues())
    {
      OUTPUT_TEXT("module:SelfLocator2012:updateState produced impossible state -> deleting hypothesis");
      it = poseHypotheses.erase(it);
    }
    else if (hypothesis->isInsideCarpet(theFieldDimensions))
    {
      it++;
    }
    else
    {
      if (parameters.displayWarnings)
        OUTPUT_TEXT("module:SelfLocator2012:updateState produced state that is outside of carpet -> deleting hypothesis");
      it = poseHypotheses.erase(it);
    }
  }
  if (poseHypotheses.empty())
  {
    OUTPUT_TEXT("module:SelfLocator2012:updateState -> no reasonable hypothesis left!");
    // re-add best one from before
    poseHypotheses.push_back(new PoseHypothesis2012(pose, pc, sc, theFrameInfo.time, parameters));
  }
}

void SelfLocator2012::updateHypothesesSymmetryConfidence()
{
  for (auto hypothesis : poseHypotheses)
  {
    if (theGameInfo.state == STATE_PLAYING && parameters.updateSymmetryWithRemoteModels) // no ball on the field in READY! (which is the strongest symmetry indicator)
    {
      hypothesis->updateSymmetryByComparingRemoteToLocalModels(theBallModel, theRemoteBallModel, LocalRobotMap(), RemoteRobotMap(), theFrameInfo, parameters);
    }
    else
      hypothesis->resetSymmetryConfidence(1.);
    // no symmetrie loss for goalie. ever.
    if (theBehaviorData.role == BehaviorData::keeper)
      hypothesis->resetSymmetryConfidence(1.);
  }
}


void SelfLocator2012::executeCommonCode()
{
  if (!initialized)
  {
    addHypothesesOnInitialKickoffPositions(0.2f);
    addHypothesesOnManualPositioningPositions();
    initialized = true;
  }

  if (lastExecuteTimeStamp == theFrameInfo.time)
  {
    return;
  }
  lastExecuteTimeStamp = theFrameInfo.time;


  initDebugging();

  // Predict new position
  predictHypotheses();

  // Fill matrices for update
  for (auto hyp : poseHypotheses)
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

  // Remove unnecessary hypotheses
  pruneHypotheses();

  // Update symmetry confidence of each hypothesis
  updateHypothesesSymmetryConfidence();

  doGlobalDebugging();
}


void SelfLocator2012::normalizeWeights()
{
  double sumOfPositionConfidences = 0;
  for (auto hypothesis : poseHypotheses)
  {
    sumOfPositionConfidences += hypothesis->getPositionConfidence();
  }
  double factor = 1 / sumOfPositionConfidences;
  for (auto hypothesis : poseHypotheses)
  {
    hypothesis->normalizePositionConfidence(factor);
  }
}


void SelfLocator2012::pruneHypothesesGoalie()
{
  // Let's simply assume that the keeper will never be in the opponent half!  :)
  PoseHypotheses2012::iterator i = poseHypotheses.begin();
  if (theBehaviorData.role == BehaviorData::keeper)
  {
    while (i != poseHypotheses.end() && poseHypotheses.size() > 1)
    {
      Pose2f keeperPose;
      (*i)->getRobotPose(keeperPose);
      if (keeperPose.translation.x() > 0) // keeper is NOT in the opponent half. Per definition.
      {
        i = poseHypotheses.erase(i);
      }
      else
      {
        i++;
      }
    }
  } // end of keeper specific pruning
}

void SelfLocator2012::pruneHypothesesOutsideCarpet()
{
  for (PoseHypotheses2012::iterator it = poseHypotheses.begin();
    it != poseHypotheses.end();)
  {
    const bool isInsideCarpet = (*it)->isInsideCarpet(theFieldDimensions);
    if (!isInsideCarpet)
    {
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

void SelfLocator2012::pruneHypotheses()
{
  if (poseHypotheses.empty())
    return;

  pruneHypothesesGoalie(); // special case: no position on opp side
  pruneHypothesesOutsideCarpet();

  // first, as a safety measure, delete everything which comes too close to the symmetric position of our best hypothesis
  // IMPORTANT: Do this only when positionsTracking!
  const PoseHypothesis2012 &bestHypothesis = getBestHypothesis();
  //  const double bestConfidence = bestHypothesis.getPositionConfidence();

  if (this->hasSymmetryBeenLost(bestHypothesis) == false)
  {
    // Identify mirror hypotheses and remove the worse one
    for (unsigned int i = 0; i < poseHypotheses.size(); i++)
    {
      Pose2f mirrorOfHypothesis;
      poseHypotheses[i]->getRobotPose(mirrorOfHypothesis);
      mirrorOfHypothesis = getSymmetricPoseOnField((Pose2f)mirrorOfHypothesis);
      for (unsigned int j = i + 1; j < poseHypotheses.size(); j++)
      {
        Pose2f hypoPose;
        poseHypotheses[j]->getRobotPose(hypoPose);
        if (arePosesCloseToEachOther(mirrorOfHypothesis, hypoPose, parameters))
        {
          if (poseHypotheses[i]->getPositionConfidence() < poseHypotheses[j]->getPositionConfidence())
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
  }
  else
  {
    // Identify good mirror hypotheses and remove the worse one
    for (unsigned int i = 0; i < poseHypotheses.size(); i++)
    {
      Pose2f mirrorOfHypothesis;
      poseHypotheses[i]->getRobotPose(mirrorOfHypothesis);
      mirrorOfHypothesis = getSymmetricPoseOnField((Pose2f)mirrorOfHypothesis);
      for (unsigned int j = i + 1; j < poseHypotheses.size(); j++)
      {
        Pose2f hypoPose;
        poseHypotheses[j]->getRobotPose(hypoPose);
        if (arePosesCloseToEachOther(mirrorOfHypothesis, hypoPose, parameters)
          && fabs(poseHypotheses[i]->getSymmetryConfidence() - poseHypotheses[j]->getSymmetryConfidence()) > 0.2)
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
  }


  // "merge" very close hypotheses, i.e. simply delete the less confident one
  GaussianDistribution3D gd1, gd2;
  for (unsigned int k = 0; k < poseHypotheses.size(); k++)
  {
    poseHypotheses[k]->extractGaussianDistribution3DFromStateEstimation(gd1);
    for (unsigned int j = k + 1; j < poseHypotheses.size(); j++)
    {
      poseHypotheses[j]->extractGaussianDistribution3DFromStateEstimation(gd2);
      double likelihood = gd1.normalizedProbabilityAt(gd2.mean) * gd2.normalizedProbabilityAt(gd1.mean);
      if (likelihood > parameters.pruning_likelihoodTresholdForMerging)
      {
        // just set the confidence of the less confident one to zero, 
        // it will be deleted automatically in the next step
        double newSymmetryConfidence = std::max(poseHypotheses[k]->getSymmetryConfidence(), poseHypotheses[j]->getSymmetryConfidence());
        poseHypotheses[k]->resetSymmetryConfidence(newSymmetryConfidence);
        poseHypotheses[j]->resetSymmetryConfidence(newSymmetryConfidence);
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
  for (auto hypothesis : poseHypotheses)
  {
    maxConfidence = std::max(maxConfidence, hypothesis->getPositionConfidence());
  }

  PoseHypotheses2012::iterator i = poseHypotheses.begin();

  const double pruningThreshold = std::max(maxConfidence / 4, 0.1f);
  while (i != poseHypotheses.end() && poseHypotheses.size() > 1)
  {
    if ((*i)->getPositionConfidence() < pruningThreshold) // no need to make this small threshold a magic number
    {
      i = poseHypotheses.erase(i);
    }
    else if (localizationState == positionLost && (*i)->getSymmetryConfidence() < 0.08)
    {
      i = poseHypotheses.erase(i);
    }
    else
    {
      i++;
    }
  }

  // remove the worst ones to keep the number low
  while (poseHypotheses.size() > parameters.pruning_maxNumberOfHypotheses)
  {
    PoseHypotheses2012::iterator worst = poseHypotheses.begin();
    for (i = poseHypotheses.begin(); i != poseHypotheses.end(); i++)
    {
      if ((*i)->getPositionConfidence() < (*worst)->getPositionConfidence())
      {
        worst = i;
      }
    }
    poseHypotheses.erase(worst);
  }
}


bool SelfLocator2012::hasSymmetryBeenLostForBestHypotheses()
{
  const PoseHypothesis2012 &bestHyp = getBestHypothesis();
  return hasSymmetryBeenLost(bestHyp);
}

bool SelfLocator2012::hasSymmetryBeenLost(const PoseHypothesis2012& hypotheses)
{
  bool symmetryConfidenceLost = hypotheses.getSymmetryConfidence() < parameters.localizationState_symmetryLostWhenBestConfidenceBelowThisThreshold;
  return symmetryConfidenceLost;
}

bool SelfLocator2012::hasPositionTrackingFailed()
{
  if (theFrameInfo.getTimeSince(lastNonPlayingTimeStamp) < 5000)
  {
    return false;
  }

  if (poseHypotheses.empty())
    return true;

  const PoseHypothesis2012 &bestHyp = getBestHypothesis();
  bool positionConfidenceLost = bestHyp.getPositionConfidence() < parameters.localizationState_positionLostWhenBestConfidenceBelowThisThreshold;
  return positionConfidenceLost;
  //  bool symmetryConfidenceLost = bestHyp.getSymmetryConfidence() < parameters.localizationState_symmetryLostWhenBestConfidenceBelowThisThreshold;
  //  bool symmetryLost = positionConfidenceLost || symmetryConfidenceLost;
  //  return symmetryLost;
}

bool SelfLocator2012::hasPositionBeenFoundAfterLoss()
{
  if (poseHypotheses.empty())
    return false;

  const PoseHypothesis2012 &bestHyp = getBestHypothesis();
  bool positionIsConfident = bestHyp.getPositionConfidence() > 0.5;
  //  bool symmetryIsConfident =  bestHyp.getSymmetryConfidence() > parameters.localizationState_symmetryFoundAgainWhenBestConfidenceAboveThisThreshold;
  //  bool symmetryFoundAgain = positionIsConfident && symmetryIsConfident;
  return positionIsConfident;
}

void SelfLocator2012::evaluateLocalizationState()
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
      if(parameters.localizationState_symmetryLostWhenFallDownInCenterCircle)
        // Handle fall down in center circle
        handleFallDown();
      else
        // If handleFallDown not executed, set localizationStateAfterGettingUp here
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
    addHypothesesOnPenaltyPositions(0.2f);
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
      //    if (hasSymmetryBeenFoundAgainAfterLoss())
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
    break;
  }
}


void SelfLocator2012::handleFallDown()
{
  const PoseHypothesis2012 &bestHyp = getBestHypothesis();
  Pose2f p;
  bestHyp.getRobotPose(p);
  if (p.translation.norm() < parameters.localizationState_unknownSymmetryRadiusAroundCenter)
  {
    // well, after getting up we will not be sure which orientation we face  :-(
    localizationStateAfterGettingUp = positionLost;
    for (auto hypothesis : poseHypotheses)
    {
      hypothesis->resetSymmetryConfidence(0); //onlyCase where symmetry is set to 0
    }
  }
  else
  {
    localizationStateAfterGettingUp = localizationState;
  }
}

void SelfLocator2012::handleGettingUpAfterFallDown()
{
  // if we fell on the side, we might end up with +/- 90° orientation errors,
  // so add those hypotheses...
  std::vector<PoseHypothesis2012*> additionalHypotheses;
  for (auto hypothesis : poseHypotheses)
  {
    hypothesis->scalePositionConfidence(parameters.positionConfidence_scaleFactorAfterFallDown); // = 0.75
    for (unsigned int j = 0; j < noAdditionalHypothesisAfterFallDown; ++j)
    {
      PoseHypothesis2012* p1 = new PoseHypothesis2012(*hypothesis); // copy Hypotheses

      const float scale_p = float(j) / float(noAdditionalHypothesisAfterFallDown - 1); //[0..1]
      const float scale_pm = -1.0f + scale_p * 2.0f; //[-1..1]
      const float rot = Angle::normalize(pi*scale_pm); //[-pi/2...+pi/2]
      const float confidenceScale = (0.25f) + (1.0f - float(fabs(scale_pm))) * (parameters.positionConfidence_scaleFactorAfterFallDown - 0.25f);
      p1->rotateHypothesis(rot);
      p1->scalePositionConfidence(confidenceScale); // = 0.25
      additionalHypotheses.push_back(p1);
    }
  }

  poseHypotheses.insert(poseHypotheses.end(), additionalHypotheses.begin(), additionalHypotheses.end());
  additionalHypotheses.clear(); // not needed any more
}

void SelfLocator2012::handleUnPenalized()
{
  unpenalizedTimeStamp = theFrameInfo.time;
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
    for (auto hypothesis : poseHypotheses)
    {
      hypothesis->scalePositionConfidence(0.5f);
    }
  }
  if (lastPenalty == PENALTY_MANUAL)
  {
    //addHypothesesOnPenaltyPositions(0.3f);
    addHypothesesOnInitialKickoffPositions(0.3f);
  }
  else
  {
    //addHypothesesOnPenaltyPositions(0.7f);
    addHypothesesOnInitialKickoffPositions(0.7f);
  }
  addHypothesesOnManualPositioningPositions();
}

void SelfLocator2012::handleSetState()
{
  // delete all hypotheses on the wrong side of the field
  PoseHypotheses2012::iterator i = poseHypotheses.begin();
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
      (*i)->resetSymmetryConfidence(parameters.symmetryConfidence_whenPositionedManually);
      i++;
    }
  }

  addHypothesesOnManualPositioningPositions();
  localizationState = positionTracking; // since we know our symmetry for sure
}

void SelfLocator2012::handleInitialState()
{
  timeStampFirstReadyState = theFrameInfo.time;
  addHypothesesOnInitialKickoffPositions(0.2f);
  localizationState = positionTracking; // since we know our symmetry for sure
}


void SelfLocator2012::addHypothesesOnManualPositioningPositions()
{
  bool ownKickoff = theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber;
  // special position for goalie
  const bool isGoalKeeper = (theBehaviorData.role == BehaviorData::keeper);
  if (isGoalKeeper)
  {
    poseHypotheses.push_back(new PoseHypothesis2012(Pose2f(0, positionsByRules.goaliePosition),
      positionConfidenceWhenPositionedManuallyForGoalKeeper, parameters.symmetryConfidence_whenPositionedManually,
      theFrameInfo.time, parameters));
  }
  // one of the field positions for field players
  else
  {
    // penalty shootout
    if (Global::getSettings().gameMode == Settings::penaltyShootout)
    {
      poseHypotheses.push_back(new PoseHypothesis2012(Pose2f(0, positionsByRules.penaltyShootOutPosition),
        positionConfidenceWhenPositionedManually,
        parameters.symmetryConfidence_whenPositionedManually,
        theFrameInfo.time,
        parameters));
    }
    else
    {
      for (auto &position : (ownKickoff ? positionsByRules.fieldPlayerPositionsOwnKickoff : positionsByRules.fieldPlayerPositionsOppKickoff))
      {
        poseHypotheses.push_back(new PoseHypothesis2012(Pose2f(0, position),
          positionConfidenceWhenPositionedManually, parameters.symmetryConfidence_whenPositionedManually,
          theFrameInfo.time, parameters));
      }
    }
  }
}

void SelfLocator2012::addHypothesesOnInitialKickoffPositions(float newPositionConfidence)
{
  poseHypotheses.push_back(new PoseHypothesis2012(Pose2f(-pi_2, theFieldDimensions.xPosOwnGroundline / 4.f, theFieldDimensions.yPosLeftSideline), newPositionConfidence, parameters.symmetryConfidence_whenPositionedManually, theFrameInfo.time, parameters));
  poseHypotheses.push_back(new PoseHypothesis2012(Pose2f(pi_2, theFieldDimensions.xPosOwnGroundline / 4.f, theFieldDimensions.yPosRightSideline), newPositionConfidence, parameters.symmetryConfidence_whenPositionedManually, theFrameInfo.time, parameters));
  poseHypotheses.push_back(new PoseHypothesis2012(Pose2f(-pi_2, theFieldDimensions.xPosOwnGroundline / 2.f, theFieldDimensions.yPosLeftSideline), newPositionConfidence, parameters.symmetryConfidence_whenPositionedManually, theFrameInfo.time, parameters));
  poseHypotheses.push_back(new PoseHypothesis2012(Pose2f(pi_2, theFieldDimensions.xPosOwnGroundline / 2.f, theFieldDimensions.yPosRightSideline), newPositionConfidence, parameters.symmetryConfidence_whenPositionedManually, theFrameInfo.time, parameters));
  poseHypotheses.push_back(new PoseHypothesis2012(Pose2f(-pi_2, 3 * theFieldDimensions.xPosOwnGroundline / 4.f, theFieldDimensions.yPosLeftSideline), newPositionConfidence, parameters.symmetryConfidence_whenPositionedManually, theFrameInfo.time, parameters));
  poseHypotheses.push_back(new PoseHypothesis2012(Pose2f(pi_2, 3 * theFieldDimensions.xPosOwnGroundline / 4.f, theFieldDimensions.yPosRightSideline), newPositionConfidence, parameters.symmetryConfidence_whenPositionedManually, theFrameInfo.time, parameters));
}

void SelfLocator2012::addHypothesesOnPenaltyPositions(float newPositionConfidence)
{
  for (auto &position : positionsByRules.penaltyPositions)
  {
    poseHypotheses.push_back(new PoseHypothesis2012(Pose2f(pi_2, position.x(), theFieldDimensions.yPosRightSideline), newPositionConfidence,
      parameters.symmetryConfidence_whenPositionedManually,
      theFrameInfo.time,
      parameters));
    poseHypotheses.push_back(new PoseHypothesis2012(Pose2f(-pi_2, position.x(), theFieldDimensions.yPosLeftSideline), newPositionConfidence,
      parameters.symmetryConfidence_whenPositionedManually,
      theFrameInfo.time,
      parameters));
  }
}

bool SelfLocator2012::addNewHypothesesWhenSymmetryLost()
{
  /** Symmeric positions will be reintroduced
      AUTHOR: D.Hauschildt
      TODO:   Local Symmetry in penalty area is not considered
      **/
  std::vector<PoseHypothesis2012*> additionalHypotheses;

  //for each Hypotheses that has a lost symmetry ... duplicate it?
  for (auto hypothesis : poseHypotheses)
  {
    if (hasSymmetryBeenLost(*hypothesis))
    {
      Pose2f symmetricPose;
      hypothesis->getRobotPose(symmetricPose);
      symmetricPose = getSymmetricPoseOnField(symmetricPose);
      additionalHypotheses.push_back(
        new PoseHypothesis2012(
        symmetricPose,
        hypothesis->getPositionConfidence(),
        hypothesis->getSymmetryConfidence(),
        theFrameInfo.time,
        parameters));
    }
  }

  bool added = !additionalHypotheses.empty();
  poseHypotheses.insert(poseHypotheses.end(), additionalHypotheses.begin(), additionalHypotheses.end());
  additionalHypotheses.clear();
  return added;
}

bool SelfLocator2012::addNewHypothesesFromLineMatches()
{
  const bool isLost = localizationState == positionLost;

  const PoseHypothesis2012 &bestHypo = getBestHypothesis();
  Pose2f hypoPose;
  bestHypo.getRobotPose(hypoPose);

  std::vector<PoseHypothesis2012*> additionalHypotheses;
  auto size = theLineMatchingResult.poseHypothesis.size();
  if (size > 0)
  {
    for (auto ph : theLineMatchingResult.poseHypothesis)
    {
      // Only add hypotheses on own side
      // AddPoseToHypothesisVector will handle symmetry
      // Also check if pose is reasonable unique or is close to actual best pose
      if (ph.pose.translation.x() < 0 && (isLost || size < 7 || arePosesCloseToEachOther(ph.pose, hypoPose, parameters)))
        addPoseToHypothesisVector((Pose2f)ph.pose, additionalHypotheses, positionConfidenceWhenPositionTracking);
    }
  }

  bool added = !additionalHypotheses.empty();
  poseHypotheses.insert(poseHypotheses.end(), additionalHypotheses.begin(), additionalHypotheses.end());
  additionalHypotheses.clear();
  return added;
}

bool SelfLocator2012::addNewHypothesesFromPenaltyCrossLine()
{
  // Add new hypotheses by PenaltyCross
  // it gives an absolute Pose
  std::vector<PoseHypothesis2012*> additionalHypotheses;

  Pose2f pose;
  float poseConfidence = 0.f;
  float bestHypoConfidence = std::max(getBestHypothesis().getPositionConfidence(), 0.5f);

  if (PoseGenerator::getPoseFromPenaltyCrossAndLine(theFieldDimensions, theCLIPFieldLinesPercept, thePenaltyCrossPercept, pose, poseConfidence))
  {
    float confidence = bestHypoConfidence*poseConfidence*parameters.landmarkBasedHypothesesSpawn_penaltyCrossBaseConfidence;
    addPoseToHypothesisVector(pose, additionalHypotheses, confidence);
  }

  bool added = !additionalHypotheses.empty();
  poseHypotheses.insert(poseHypotheses.end(), additionalHypotheses.begin(), additionalHypotheses.end());
  additionalHypotheses.clear();
  return added;
}

bool SelfLocator2012::addNewHypothesesFromCenterCirleAndLine()
{
  // Add new hypotheses by CenterCircle and it's CenterLine
  // it gives an absolute Pose
  const bool isGoalKeeper = (theBehaviorData.role == BehaviorData::keeper);
  if (isGoalKeeper) return false; // Do not use for goalie

  std::vector<PoseHypothesis2012*> additionalHypotheses;

  Pose2f pose;
  float poseConfidence = 0.f;
  float bestHypoConfidence = std::max(getBestHypothesis().getPositionConfidence(), 0.5f);

  if (PoseGenerator::getPoseFromCenterCircleAndCenterLine(theCLIPFieldLinesPercept, theCLIPCenterCirclePercept, pose, poseConfidence) > 0)
  {
    float confidence = bestHypoConfidence*poseConfidence*parameters.landmarkBasedHypothesesSpawn_centerCircleBaseConfidence;
    addPoseToHypothesisVector(pose, additionalHypotheses, confidence);
  }

  bool added = !additionalHypotheses.empty();
  poseHypotheses.insert(poseHypotheses.end(), additionalHypotheses.begin(), additionalHypotheses.end());
  additionalHypotheses.clear();
  return added;
}

bool SelfLocator2012::addNewHypothesesFromGoal()
{
  const bool isGoalKeeper = (theBehaviorData.role == BehaviorData::keeper);
  if (isGoalKeeper) return false; // Do not use for goalie

  std::vector<PoseHypothesis2012*> additionalHypotheses;
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
      float confidence = bestHypoConfidence*poseConfidence*parameters.landmarkBasedHypothesesSpawn_goalBaseConfidence;
      addPoseToHypothesisVector(pose, additionalHypotheses, confidence);
    }
  }

  bool added = !additionalHypotheses.empty();
  poseHypotheses.insert(poseHypotheses.end(), additionalHypotheses.begin(), additionalHypotheses.end());
  additionalHypotheses.clear();
  return added;
}

bool SelfLocator2012::addNewHypothesesFromLandmark()
{
  return addNewHypothesesFromPenaltyCrossLine()
    || addNewHypothesesFromCenterCirleAndLine()
    || addNewHypothesesFromGoal();
}

bool SelfLocator2012::addNewHypotheses()
{
  bool added = false;

  switch (localizationState)
  {
  case positionLost:
    added |= addNewHypothesesFromLineMatches();
    if (parameters.landmarkBasedHypothesesSpawn == SelfLocator2012Parameters::spawnIfPositionLost
      || parameters.landmarkBasedHypothesesSpawn == SelfLocator2012Parameters::spawnIfLostOrTracking)
      added |= addNewHypothesesFromLandmark();
    break;

  case positionTracking:
    if (getBestHypothesis().getPositionConfidence() < parameters.spawning_spawnWhilePositionTrackingWhenBestConfidenceBelowThisThreshold)
    {
      added |= addNewHypothesesFromLineMatches();
      if (parameters.landmarkBasedHypothesesSpawn == SelfLocator2012Parameters::spawnIfPositionTracking
        || parameters.landmarkBasedHypothesesSpawn == SelfLocator2012Parameters::spawnIfLostOrTracking)
        added |= addNewHypothesesFromLandmark();
    }
    break;

  default:
    void(0); //nothing todo;
  }

  added |= addNewHypothesesWhenSymmetryLost();
  return added;
}


float SelfLocator2012::getRobotPoseValidity(const PoseHypothesis2012 & poseHypothesis)
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


inline void SelfLocator2012::initDebugging()
{
  DECLARE_DEBUG_DRAWING("module:SelfLocator2012:hypothesis", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:SelfLocator2012:correspondences", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:SelfLocator2012:LocalizationState", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:SelfLocator2012:poseFromCenterCircle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:SelfLocator2012:poseFromPenaltyCross", "drawingOnField");

  MODIFY("module:SelfLocator2012:parameters", parameters);
}

void SelfLocator2012::doGlobalDebugging()
{
  ; // IMPORTANT!!! Do not remove!

  if (poseHypotheses.empty())
  {
    DRAWTEXT("module:SelfLocator2012:LocalizationState", -500, -2200, 50, ColorRGBA(255, 255, 255), "LocalizationState: no more hypotheses! Should never happen!");
    return;
  }

  const PoseHypothesis2012 &bestHyp = getBestHypothesis();
  if (!bestHyp.isInsideCarpet(theFieldDimensions))
    return;

  COMPLEX_DRAWING("module:SelfLocator2012:hypothesis")
  {
    for (auto hypothesis : poseHypotheses)
    {
      float factor = std::max(std::min(theFrameInfo.getTimeSince(hypothesis->getCreationTime()) / parameters.debug_durationHighlightAddedHypothesis, 1.f), 0.f);
      hypothesis->draw(ColorRGBA(static_cast<unsigned char>(factor * 255), static_cast<unsigned char>(factor * 255), 255, (unsigned char)(255.0*hypothesis->getPositionConfidence())));
    }

    //recolor the best
    bestHyp.draw(ColorRGBA(255, 0, 0, (unsigned char)(255.0*bestHyp.getPositionConfidence())));
  }

  switch (localizationState)
  {
  case positionTracking:
    DRAWTEXT("module:SelfLocator2012:LocalizationState", -500, -2200, 50, ColorRGBA(255, 255, 255), "LocalizationState: positionTracking");
    break;
  case positionLost:
    DRAWTEXT("module:SelfLocator2012:LocalizationState", -500, -2200, 50, ColorRGBA(255, 255, 255), "LocalizationState: positionLost");
    break;
  case fallenDown:
    DRAWTEXT("module:SelfLocator2012:LocalizationState", -500, -2200, 50, ColorRGBA(255, 255, 255), "LocalizationState: fallenDown");
    break;
  case penalized:
    DRAWTEXT("module:SelfLocator2012:LocalizationState", -500, -2200, 50, ColorRGBA(255, 255, 255), "LocalizationState: penalized");
    break;
  default:
    DRAWTEXT("module:SelfLocator2012:LocalizationState", -500, -2200, 50, ColorRGBA(255, 255, 255), "LocalizationState: default... should never happen!");
    // should never happen!
    break;
  }
}


const PoseHypothesis2012 &SelfLocator2012::getBestHypothesis()
{
  PoseHypotheses2012::iterator it;
  for (it = poseHypotheses.begin(); it != poseHypotheses.end(); ++it)
  {
    if ((*it)->getUniqueId() == lastBestHypothesisUniqueId)
      break;
  }

  if (it != poseHypotheses.end())
  {
    //found it
    if ((*it)->getPositionConfidence() >= positionConfidenceForKeepingBestHypothesis)
    {
      return (**it);
    }
  }
  unsigned indexOfBestHypothesis = 0;
  //if we did not quit ... find the best
  for (unsigned int i = 1; i < poseHypotheses.size(); i++)
  {
    if (poseHypotheses[indexOfBestHypothesis]->getPositionConfidence() < poseHypotheses[i]->getPositionConfidence())
    {
      indexOfBestHypothesis = i;
    }
  }

  lastBestHypothesisUniqueId = poseHypotheses[indexOfBestHypothesis]->getUniqueId();
  return *poseHypotheses[indexOfBestHypothesis];
}

void SelfLocator2012::addPoseToHypothesisVector(const Pose2f &pose, std::vector<PoseHypothesis2012*> &additionalHypotheses,
  const float &poseConfidence)
{
  const bool isLost = localizationState == positionLost;
  Pose2f symmetricPose = getSymmetricPoseOnField(pose);

  const PoseHypothesis2012 &bestHypo = getBestHypothesis();
  Pose2f hypoPose;
  bestHypo.getRobotPose(hypoPose);
  Pose2f nearestPose = ((pose - hypoPose).translation.norm() <= (symmetricPose - hypoPose).translation.norm())
    ? Pose2f(pose) : symmetricPose;

  const float distanceToNearestPose = (hypoPose - nearestPose).translation.norm();

  const int timeSinceLastGoodPosition = theFrameInfo.getTimeSince(timeStampLastGoodPosition);
  const bool posePossible = !parameters.useOdometryForSpawning || (timeSinceLastGoodPosition > 20000
    || (std::abs(distanceTraveledFromLastGoodPosition.translation.norm() - distanceToNearestPose) < std::min(2000, timeSinceLastGoodPosition * 500)));
  const bool allowToSpawn = (isPoseOnSameFieldSide(hypoPose, nearestPose)
    || ((theFrameInfo.getTimeSince(timeStampFirstReadyState) > 10000)
    && (theFrameInfo.getTimeSince(unpenalizedTimeStamp) > 10000)));
  if (posePossible && (isLost || allowToSpawn))
  {
    // Only spawn if confidence higher than lost threshold
    const float &pc = isLost ? parameters.localizationState_positionLostWhenBestConfidenceBelowThisThreshold : poseConfidence;
    if (pc >= parameters.localizationState_positionLostWhenBestConfidenceBelowThisThreshold)
      additionalHypotheses.push_back(new PoseHypothesis2012(nearestPose,
      pc, bestHypo.getSymmetryConfidence(), theFrameInfo.time, parameters));
  }
}


MAKE_MODULE(SelfLocator2012, modeling)
