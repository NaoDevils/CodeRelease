/*
Copyright 2011, Oliver Urbann
All rights reserved.

This file is part of MoToFlex.

MoToFlex is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

MoToFlex is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with MoToFlex.  If not, see <http://www.gnu.org/licenses/>.

Contact e-mail: oliver.urbann@tu-dortmund.de
*/

/**
* @file SwingLegController.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once

#include <list>
#include <queue>
#include "StepData.h"

#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/ReferenceModificator.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/ObservedError.h"
#include "Tools/DynamicRingBuffer.h"
#include "Tools/Streams/RobotParameters.h"

#ifndef WALKING_SIMULATOR
#include "Representations/Modeling/BallModel.h"
#else
#include "bhumanstub.h"
#endif

#define POLYNOM_DEGREE	4

class SwingLegControllerModule;
/**
* @class SwingLegController
* Calculates positions for the swing leg (not given by PatternGenerator)
*/
class SwingLegController
{
  ROBOT_PARAMETER_CLASS(KickMotion, SwingLegController)
    PARAM(unsigned int, hitPoint)
    PARAM(unsigned int, degree)
    PARAM(std::vector<StreamPoint>, p) 
  END_ROBOT_PARAMETER_CLASS(KickMotion)

  friend class SwingLegControllerModule;
public:
  /** Constructor with all needed source data structures.
  * @param theFootSteps Target positions of feet on the floor.
  * @param theWalkingEngineParams Walking Engine Parameters.
  */
  SwingLegController(
    const WalkingEngineParams       &theWalkingEngineParams,
    const FootSteps                 &theFootSteps,
    const BallModel	                &theBallModel,
    const MotionRequest             &theMotionRequest,
    const WalkingInfo               &theWalkingInfo,
    const FreeLegPhaseParams        &theFreeLegPhaseParams,
    const FallDownState             &theFallDownState,
    const PatternGenRequest         &thePatternGenRequest,
    const ObservedError             &theObservedError,
    const ControllerParams          &theControllerParams):

    theWalkingEngineParams(theWalkingEngineParams),
    theFootSteps(theFootSteps),
    theBallModel(theBallModel),
    theMotionRequest(theMotionRequest),
    theWalkingInfo(theWalkingInfo),
    theFreeLegPhaseParams(theFreeLegPhaseParams),
    theFallDownState(theFallDownState),
    thePatternGenRequest(thePatternGenRequest),
    theObservedError(theObservedError),
    theControllerParams(theControllerParams),
    debug(false),
    lastKick(0),
    modification(localModification),
    infos(phases)
  { reset(); };

  /** Destructor */
    ~SwingLegController(void) { phases.clear(); };

  /** 
  * Calculates the next position.
  * @param footpositions Target data structure.
  */
  void updateFootpositions(Footpositions &footpositions);

  void update(ReferenceModificator &referenceModificator);
private:
  // Different phases when the robot stands on one leg:
  // During the starting phase we are in the single support phase
  // and freeLegBoundary is the last point of the leg on the floor.
  // During the ongoing phase the end is unknown, we are in a unlimitedSingleSupport.
  // During the ending phase we are in a single support, the upcomming double support
  // is planned and the freeLegBoundary is the next position of the free leg on the floor.
  // NA means we are not in a phase where we can use a leg.

  const WalkingEngineParams &theWalkingEngineParams;
  const FootSteps           &theFootSteps;
  const BallModel           &theBallModel;
  const MotionRequest       &theMotionRequest;
  const WalkingInfo         &theWalkingInfo;
  const FreeLegPhaseParams  &theFreeLegPhaseParams;
  const FallDownState       &theFallDownState; /**< Set by constructor */
  const PatternGenRequest   &thePatternGenRequest;
  const ObservedError       &theObservedError;
  const ControllerParams    &theControllerParams;

  FreeLegPhase currentKickPhase;

  class Phase;
  class FootpositionListElement : public Footposition, public Streamable
  {
  public:
    FootpositionListElement() {};
    FootpositionListElement(const Footposition &fp) : Footposition(fp) {};

    void replace(const Point &p, int footNum)
    {
      footPos[footNum] = p;
    }

    void modify(const Point &p, int footNum)
    {
      footPos[footNum] += p;
    }
    
    void serialize(In* in,Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(footPos[0].x);
      STREAM_REGISTER_FINISH;
    };
  };

  typedef std::list<FootpositionListElement> FootList;

  class Phase : public Streamable
  {
  public:
    Phase(const Footposition &fp)
    { members.push_back(fp); }
    ~Phase()
    { members.clear(); }
    bool containsLast() const
    {
      if (members.empty()) return false;
      return members.back().frameInPhase == members.back().singleSupportLen - 1;
    }
    bool containsFirst() const
    {
      if (members.empty()) return false;
      return members.front().frameInPhase == 0;
    }
    void replace(Point p[], int footNum)
    {
      int i = 0;
      for (FootList::iterator fp = members.begin();
           fp != members.end(); fp++, i++)
        fp->replace(p[i], footNum);
    }
    void replace(const Point &p, int footNum)
    {
      for (FootList::iterator fp = members.begin(); fp != members.end(); fp++)
        fp->replace(p, footNum);
    }
    void modify(const Point &p, int footNum)
    {
      
      for (FootList::iterator fp = members.begin(); fp != members.end(); fp++)
        fp->modify(p, footNum);
    }
    const Point &getSidestep() const
    {
      return sidestep;
    }
    void modifySidestepSum(const Point &p)
    {
      sidestep += p;
    }

    FootList members;

  private:
    Point sidestep;
    void serialize(In* in,Out* out)
    {
      STREAM_REGISTER_BEGIN;
      for(auto e: members) STREAM(e);
      STREAM_REGISTER_FINISH;
    };
  };
  
  typedef std::list<Phase> PhaseList;
  PhaseList phases;

  PhaseList::iterator skip(const PhaseList::iterator &ph, unsigned int toSkip)
  {
    PhaseList::iterator ret = ph;
    for(unsigned int i = 0; ret != phases.end() && i < toSkip; i++, ret++);
    return ret;
  }

  PhaseList::iterator rskip(const PhaseList::iterator &ph, unsigned int toSkip)
  {
    PhaseList::iterator ret = ph;
    for(unsigned int i = 0; ret != phases.begin() && i < toSkip; i++, ret--);
    return ret;
  }
  
  struct Infos : Streamable
  {
    struct PhaseInfo : Streamable
    {
      std::string phaseType;
      WalkRequest::StepRequest customStep;
      StreamPoint pos[2];
      int len;
      void serialize(In* in,Out* out)
      {
        STREAM_REGISTER_BEGIN;
        STREAM(phaseType)
        STREAM(len)
        _STREAM_WITH_CLASS(customStep, WalkRequest)
        STREAM(pos)
        STREAM_REGISTER_FINISH;
      };
    };
    Infos(PhaseList &p) : pl(p) {};
    PhaseList &pl;
    std::vector<PhaseInfo> phaseInfos;
    int previewLength;
    void serialize(In* in,Out* out)
    {
      phaseInfos.clear();
      previewLength = 0;
      for (auto const& e: pl)
      {
        PhaseInfo i;
        i.len = (int)e.members.size();
        previewLength += i.len;
        i.customStep = e.members.front().customStep;
        switch (e.members.front().phase)
        {
          case firstDoubleSupport:
            i.phaseType = "firstDoubleSupport";
            break;
          case secondDoubleSupport:
            i.phaseType = "secondDoubleSupport";
            break;
          case firstSingleSupport:
            i.phaseType = "firstSingleSupport";
            break;
          case secondSingleSupport:
            i.phaseType = "secondSingleSupport";
            break;
          default:
            i.phaseType =  "unknown";
        }
        i.pos[LEFT_FOOT] = StreamPoint(e.members.front().footPos[LEFT_FOOT]);
        i.pos[RIGHT_FOOT] = StreamPoint(e.members.front().footPos[RIGHT_FOOT]);
        phaseInfos.push_back(i);
      }
      STREAM_REGISTER_BEGIN;
      STREAM(previewLength)
      STREAM(phaseInfos)
      STREAM_REGISTER_FINISH;
    };
  };
  
  Infos infos;

  bool debug;
  int lastKick;
  /** Is the ZMP/IP-Controller running? */
  bool isRunning;

  int footToModify, lastModified;
  Point modifier;

  ReferenceModificator &modification, localModification;

  Vector3f   kickVec, kickStart, kickStop;
  int kickStartLen, kickStopLen;
  /** Resets the controller. */
  void reset();
  /** 
  * Deletes no more needed elements in footPositions. 
  * @param footpositions Deleted positions contain the swing leg positions, so copy them here.
  */
  Footposition Shrink();
  
  /** 
  * Add a foot position to the footPositions list.
  * @param fp The list to add to.
  */
  void addFootsteps(const Footposition &fp);
  bool initKick(int footNum, Point &endFootPos, bool longKick);

  /**
  * Internal function to plan the next target position.
  * @param footNum Number of foot (0 = left, 1 = right)
  */
  void PlanFootReset(int footNum);

  void sidestep();
  void modifyFp(int startFoot, 
    PhaseList::iterator ph,
    Vector2f posErrRCS,
    Vector2f velErrRCS,
    Vector2f zmpErrRCS,
    float modificatorRCS,
    int dim);
  bool modPossible(PhaseList::iterator ph, 
    float modificatorRCS,
    int dim);
};

