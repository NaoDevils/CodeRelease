#pragma once

#include "Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h"
#include <Representations/BehaviorControl/BallSymbols.h>
#include <Representations/BehaviorControl/GameSymbols.h>
#include <Representations/Infrastructure/GameInfo.h>
#include <optional>
#include <stdexcept>

template <class PS> class RoleProvider
{

protected:
  RoleProvider() = default;
  ~RoleProvider() = default;

  // START :: COPY THESE TO CREATE A NEW ROLE ==========================================================================
  // Some of these methods return a float: These are the seconds for which the method continues to get executed after
  // the setPlay finishes
  virtual void stateInitial_kickOff_own(PS& positioningSymbols, const Vector2f& ballPosition) { stateReady_kickOff_own(positioningSymbols, ballPosition); }
  virtual void stateInitial_kickOff_opponent(PS& positioningSymbols, const Vector2f& ballPosition) { stateReady_kickOff_opponent(positioningSymbols, ballPosition); }
  virtual void stateReady_kickOff_own(PS& positioningSymbols, const Vector2f& ballPosition) = 0;
  virtual void stateReady_kickOff_opponent(PS& positioningSymbols, const Vector2f& ballPosition) = 0;
  virtual void statePlaying_kickOff_own(PS& positioningSymbols, const Vector2f& ballPosition) { stateReady_kickOff_own(positioningSymbols, ballPosition); }
  virtual void statePlaying_kickOff_opponent(PS& positioningSymbols, const Vector2f& ballPosition) { stateReady_kickOff_opponent(positioningSymbols, ballPosition); }
  virtual float goalKick_own(PS& positioningSymbols, bool left) = 0;
  virtual float goalKick_opponent(PS& positioningSymbols, bool left) = 0;
  virtual float pushingFreeKick_own(PS& positioningSymbols) = 0;
  virtual float pushingFreeKick_opponent(PS& positioningSymbols) = 0;
  virtual float cornerKick_own(PS& positioningSymbols, const Vector2f& cornerKickPosition, bool left) = 0;
  virtual float cornerKick_opponent(PS& positioningSymbols, const Vector2f& cornerKickPosition, bool left) = 0;
  virtual float kickIn_own(PS& positioningSymbols, bool left) = 0;
  virtual float kickIn_opponent(PS& positioningSymbols, bool left) = 0;
  virtual float stateReady_penaltyKick_own(PS& positioningSymbols) = 0;
  virtual float stateReady_penaltyKick_opponent(PS& positioningSymbols) = 0;
  virtual float statePlaying_penaltyKick_own(PS& positioningSymbols) { return stateReady_penaltyKick_own(positioningSymbols); }
  virtual float statePlaying_penaltyKick_opponent(PS& positioningSymbols) { return stateReady_penaltyKick_opponent(positioningSymbols); }
  virtual void regularPlay(PS& positioningSymbols) = 0;
  // END :: COPY THESE TO CREATE A NEW ROLE ============================================================================

  /**
   * This method calls the specialized methods for each state and setPlay
   */
  void decide(PS& positioningSymbols, const BallSymbols& theBallSymbols, const FieldDimensions& theFieldDimensions, const GameInfo& theGameInfo, const GameSymbols& theGameSymbols)
  {
    switch (theGameInfo.gamePhase)
    {
    case GAME_PHASE_TIMEOUT:
      positioningSymbols.log_currState = "GAME_PHASE_TIMEOUT";
      return;
    case GAME_PHASE_PENALTYSHOOT:
      switch (theGameInfo.state)
      {
      case STATE_SET:
        stateSet(positioningSymbols);
        positioningSymbols.log_currState = "PHASE_PENALTYSHOOT STATE_SET";
        return;
      case STATE_PLAYING:
        if (theGameSymbols.ownKickOff)
        {
          setplayContinueSeconds = statePlaying_penaltyKick_own(positioningSymbols);
          positioningSymbols.log_currState = "PHASE_PENALTYSHOOT STATE_PLAYING SET_PLAY_PENALTY_KICK statePlaying_penaltyKick_own";
        }
        else
        {
          setplayContinueSeconds = statePlaying_penaltyKick_opponent(positioningSymbols);
          positioningSymbols.log_currState = "PHASE_PENALTYSHOOT STATE_PLAYING SET_PLAY_PENALTY_KICK statePlaying_penaltyKick_opponent";
        }
        return;
      }
      return;
    case GAME_PHASE_NORMAL:
    case GAME_PHASE_OVERTIME:
      switch (theGameInfo.state)
      {
      case STATE_INITIAL:
      {
        bool stop = stateInitial(positioningSymbols);
        if (stop)
        {
          positioningSymbols.log_currState = "STATE_INITIAL";
          return;
        }

        if (theGameSymbols.ownKickOff)
        {
          stateInitial_kickOff_own(positioningSymbols, Vector2f::Zero());
          positioningSymbols.log_currState = "STATE_INITIAL stateInitial_kickOff_own";
        }
        else
        {
          stateInitial_kickOff_opponent(positioningSymbols, Vector2f::Zero());
          positioningSymbols.log_currState = "STATE_INITIAL stateInitial_kickOff_opponent";
        }
        return;
      }
      case STATE_READY:
      {
        bool stop = stateReady(positioningSymbols);

        if (stop)
        {
          positioningSymbols.log_currState = "STATE_READY";
          return;
        }

        if (decideIsSetPlay(theGameInfo, theGameSymbols))
        {
          switch (getSetPlay(theGameInfo, theGameSymbols))
          {
          case SET_PLAY_PENALTY_KICK:
            if (theGameSymbols.ownKickOff)
            {
              setplayContinueSeconds = stateReady_penaltyKick_own(positioningSymbols);
              positioningSymbols.log_currState = "STATE_READY SET_PLAY_PENALTY_KICK stateReady_penaltyKick_own";
            }
            else
            {
              setplayContinueSeconds = stateReady_penaltyKick_opponent(positioningSymbols);
              positioningSymbols.log_currState = "STATE_READY SET_PLAY_PENALTY_KICK stateReady_penaltyKick_opponent";
            }
            return;
          default:
            OUTPUT_WARNING("Unknown Setplay in Ready!");
            regularPlay(positioningSymbols);
            positioningSymbols.log_currState = "Unknown SetPlay in Ready! regularPlay";
            return;
          }
        }

        if (theGameSymbols.ownKickOff)
        {
          stateReady_kickOff_own(positioningSymbols, Vector2f::Zero());
          positioningSymbols.log_currState = "STATE_READY stateReady_kickOff_own";
        }
        else
        {
          stateReady_kickOff_opponent(positioningSymbols, Vector2f::Zero());
          positioningSymbols.log_currState = "STATE_READY stateReady_kickOff_opponent";
        }
        return;
      }
      case STATE_SET:
        stateSet(positioningSymbols);
        positioningSymbols.log_currState = "STATE_SET";
        return;
      case STATE_PLAYING:
      {
        bool stop = statePlaying(positioningSymbols);

        if (stop)
        {
          positioningSymbols.log_currState = "STATE_PLAYING";
          return;
        }

        if (theGameSymbols.kickoffInProgress && theGameSymbols.timeSincePlayingState < 10000 && theGameInfo.setPlay == SET_PLAY_NONE) // TODO KickOff time constant
        {
          if (theGameSymbols.ownKickOff)
          {
            statePlaying_kickOff_own(positioningSymbols, Vector2f::Zero());
            positioningSymbols.log_currState = "STATE_PLAYING stateReady_kickOff_own";
            return;
          }
          else if (theGameSymbols.avoidCenterCircle)
          {
            statePlaying_kickOff_opponent(positioningSymbols, Vector2f::Zero());
            positioningSymbols.log_currState = "STATE_PLAYING stateReady_kickOff_opponent";
            return;
          }
        }

        if (decideIsSetPlay(theGameInfo, theGameSymbols))
        {
          bool ballIsLeft = theBallSymbols.ballPositionFieldPredicted.y() > 0;

          switch (getSetPlay(theGameInfo, theGameSymbols))
          {
          case SET_PLAY_GOAL_KICK:
            if (theGameSymbols.ownKickOff)
            {
              setplayContinueSeconds = goalKick_own(positioningSymbols, ballIsLeft);
              positioningSymbols.log_currState = "STATE_PLAYING SET_PLAY_GOAL_KICK goalKick_own";
            }
            else
            {
              setplayContinueSeconds = goalKick_opponent(positioningSymbols, ballIsLeft);
              positioningSymbols.log_currState = "STATE_PLAYING SET_PLAY_GOAL_KICK goalKick_opponent";
            }
            return;
          case SET_PLAY_PUSHING_FREE_KICK:
            if (theGameSymbols.ownKickOff)
            {
              setplayContinueSeconds = pushingFreeKick_own(positioningSymbols);
              positioningSymbols.log_currState = "STATE_PLAYING SET_PLAY_PUSHING_FREE_KICK pushingFreeKick_own";
            }
            else
            {
              setplayContinueSeconds = pushingFreeKick_opponent(positioningSymbols);
              positioningSymbols.log_currState = "STATE_PLAYING SET_PLAY_PUSHING_FREE_KICK pushingFreeKick_opponent";
            }
            return;
          case SET_PLAY_CORNER_KICK:
          {
            Vector2f cornerKickPosition(theGameSymbols.ownKickOff ? theFieldDimensions.xPosOpponentGroundline : theFieldDimensions.xPosOwnGroundline,
                ballIsLeft ? theFieldDimensions.yPosLeftSideline : theFieldDimensions.yPosRightSideline);
            if (theGameSymbols.ownKickOff)
            {
              setplayContinueSeconds = cornerKick_own(positioningSymbols, cornerKickPosition, ballIsLeft);
              positioningSymbols.log_currState = "STATE_PLAYING SET_PLAY_CORNER_KICK cornerKick_own";
            }
            else
            {
              setplayContinueSeconds = cornerKick_opponent(positioningSymbols, cornerKickPosition, ballIsLeft);
              positioningSymbols.log_currState = "STATE_PLAYING SET_PLAY_CORNER_KICK cornerKick_opponent";
            }
            return;
          }
          case SET_PLAY_KICK_IN:
            if (theGameSymbols.ownKickOff)
            {
              setplayContinueSeconds = kickIn_own(positioningSymbols, ballIsLeft);
              positioningSymbols.log_currState = "STATE_PLAYING SET_PLAY_KICK_IN kickIn_own";
            }
            else
            {
              setplayContinueSeconds = kickIn_opponent(positioningSymbols, ballIsLeft);
              positioningSymbols.log_currState = "STATE_PLAYING SET_PLAY_KICK_IN kickIn_opponent";
            }
            return;
          case SET_PLAY_PENALTY_KICK:
            if (theGameSymbols.ownKickOff)
            {
              setplayContinueSeconds = statePlaying_penaltyKick_own(positioningSymbols);
              positioningSymbols.log_currState = "STATE_PLAYING SET_PLAY_PENALTY_KICK statePlaying_penaltyKick_own";
            }
            else
            {
              setplayContinueSeconds = statePlaying_penaltyKick_opponent(positioningSymbols);
              positioningSymbols.log_currState = "STATE_PLAYING SET_PLAY_PENALTY_KICK statePlaying_penaltyKick_opponent";
            }
            return;
          default:
            throw std::invalid_argument("Unknown Setplay!");
          }
        }

        regularPlay(positioningSymbols);
        positioningSymbols.log_currState = "STATE_PLAYING regularPlay";

        return;
      }
      case STATE_FINISHED:
        stateFinished(positioningSymbols);
        return;
      default:
        throw std::invalid_argument("Unknown GameInfoState!");
      }
    }
  }

  // START :: STATE METHODS ============================================================================================
  // Executed for the specified state. If true is returned no other method for this state will get called (e.g. if
  // statePlaying returns true the regularPlay method won't get called). These methods should not be needed.
  virtual bool stateInitial(PS& positioningSymbols) { return false; }
  virtual bool stateReady(PS& positioningSymbols) { return false; }
  virtual void stateSet(PS& positioningSymbols) {}
  virtual bool statePlaying(PS& positioningSymbols) { return false; }
  virtual void stateFinished(PS& positioningSymbols) {}
  // END :: STATE METHODS ==============================================================================================

private:
  bool decideIsSetPlay(const GameInfo& theGameInfo, const GameSymbols& theGameSymbols)
  {
    bool setPlayOngoing = theGameInfo.setPlay != SET_PLAY_NONE;
    bool continueSetplay = theGameSymbols.timeSinceSetPlayFinished < (int)setplayContinueSeconds;
    return setPlayOngoing || continueSetplay;
  }

  int getSetPlay(const GameInfo& theGameInfo, const GameSymbols& theGameSymbols) { return theGameInfo.setPlay != SET_PLAY_NONE ? theGameInfo.setPlay : theGameSymbols.lastSetPlay; }

  float setplayContinueSeconds = 0.f;
};
