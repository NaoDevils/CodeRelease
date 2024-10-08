/**
 * @file Controller/GameController.h
 * This file declares a class that simulates a console-based GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include <set>
#include <SimRobotCore2.h>
#include "Platform/Thread.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Tools/Enum.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/InOut.h"

class SimulatedRobot;

/**
 * The class simulates a console-based GameController.
 */
class GameController
{
private:
  class Robot
  {
  public:
    SimulatedRobot* simulatedRobot;
    RobotInfo info;
    unsigned timeWhenPenalized;
    Pose2f lastPose;
    bool manuallyPlaced;

    Robot() : simulatedRobot(0), timeWhenPenalized(0), manuallyPlaced(false)
    {
      // Disable robot by default
      info.penalty = PENALTY_SUBSTITUTE;
    }
  };

  ENUM(Penalty,
    none,
    illegalBallContact,
    playerPushing,
    illegalMotionInSet,
    inactivePlayer,
    illegalDefender,
    leavingTheField,
    kickOffGoal,
    requestForPickup,
    manual
  );

  static GameController* theInstance;

  static const int numOfPenalties = numOfPenaltys; /**< Correct typo. */

  DECLARE_SYNC;
  static constexpr int numOfRobots = MAX_NUM_PLAYERS * 2;
  static constexpr int numOfFieldPlayers = numOfRobots / 2 - 2; // Keeper, Substitute
  static constexpr uint16_t messageBudget = 1200;
  static constexpr int durationOfHalf = 600;
  static constexpr int durationOfPS = 45; /**< duration of one penalty shootout attemp. */
  static constexpr float footLength = 120.f; /**< foot length for position check and manual placement at center circle. */
  static constexpr float safeDistance = 150.f; /**< safe distance from penalty areas for manual placement. */
  static constexpr float dropHeight = 320.f; /**< height at which robots are manually placed so the fall a little bit and recognize it. */
  Pose2f lastBallContactPose; /**< Position were the last ball contact of a robot took place, orientation is toward opponent goal (0/180 degress). */
  int timeOfLastBallContact = 0; /**Time when the last ball contact occured*/
  FieldDimensions fieldDimensions;

  RawGameInfo gameInfo;
  TeamInfo teamInfos[2];
  unsigned timeWhenHalfStarted;
  unsigned timeOfLastDropIn;
  unsigned timeWhenLastRobotMoved;
  unsigned timeWhenStateBegan;
  int timeWhenSetPlayStarted = -1; /**Time when the current set play started (-1 during SET_PLAY_NONE)*/
  Robot robots[numOfRobots];

  /** enum which declares the different types of balls leaving the field */
  enum BallOut
  {
    NONE,
    GOAL_BY_RED,
    GOAL_BY_BLUE,
    KICK_IN_FOR_RED,
    KICK_IN_FOR_BLUE,
    GOAL_FREE_KICK_FOR_RED,
    GOAL_FREE_KICK_FOR_BLUE,
    CORNER_KICK_FOR_RED,
    CORNER_KICK_FOR_BLUE
  };

  /**
   * Handles the command "pr".
   * @param robot The number of the robot that received the command.
   * @param command The second part of the command (without "pr").
   */
  bool handleRobotCommand(int robot, const std::string& command);

  /** Return the number of the team the a robot is playing for.*/
  int getTeamNumberOfRobot(int robotNum);

  /**
   * Is a robot it its own penalty area or in its own goal area?
   * @param robot The number of the robot to check [0 ... numOfRobots-1].
   * @return Is it?
   */
  bool inOwnPenaltyArea(int robot) const;

  /**
   * Is a robot it the opponents penalty area or in the opponents own goal area?
   * @param robot The number of the robot to check [0 ... numOfRobots-1].
   * @return Is it?
   */
  bool inOpponentPenaltyArea(int robot) const;

  /**
   * Finds a free place for a (un)penalized robot.
   * @param robot The number of the robot to place [0 ... numOfRobots-1].
   * @param x The optimal x coordinate. Might be moved toward own goal.
   * @param y The y coordinate.
   * @param rotation The rotation when placed.
   */
  void placeForPenalty(int robot, float x, float y, float rotation);

  /**
   * Manually place a goalie if required.
   * @param robot The robot number of the goalie to place (0 or numOfRobots/2).
   */
  void placeGoalie(int robot);

  /**
   * Move a field player to a new pose from a set of possible poses.
   * Pick the pose the teammates would not pick.
   * @param robot The number of the robot to place [0 ... numOfRobots-1].
   * @param minRobot The number of the first field player in the team (1 or numOfRobots/2+1).
   * @param poses Possible placepent poses for the robot.
   */
  void placeFromSet(int robot, int minRobot, const Pose2f* poses);

  /**
   * Manually place the field players of the offensive team if required.
   * @param minRobot The number of the first robot to place (1 or numOfRobots/2+1).
   */
  void placeOffensivePlayers(int minRobot);

  /**
   * Manually place the field players of the defensive team if required.
   * @param minRobot The number of the first robot to place (1 or numOfRobots/2+1).
   */
  void placeDefensivePlayers(int minRobot);

  /**
   * Remove all but one field players from the penalty area.
   * @param minRobot The number of the first field player in the team (1 or numOfRobots/2+1).
   * @param poses Possible placepent poses robots.
   */
  void freePenaltyArea(int minRobot, const Pose2f* poses);

  /** Execute the manual placements decided before. */
  void executePlacement();

  /** Check if the ball left the field and which action should follow. */
  BallOut updateBall();

  /** Place the ball in the correct position based on how it left the field. */
  void placeBallAfterLeavingField(BallOut typeOfBallOut);

  /** Checks if current set play ended because time ran out or offensive team touched the ball. */
  void checkForSetPlayCompletion();

  /** Executes all actions necessary when switching from ready to set during a penalty kick.*/
  void handlePenaltyKickSet();

  /** Determines which robot is supposed to execute a penalty kick by choosing the player with minimal distance to ball. */
  int findPlayerNumberOfPenaltyKickStriker(Vector2f ballPosition);

  /** Determines which robot is the goalkeeper of the team defending a set play.*/
  int findGoalieOfDefendingTeam();

  /** Determines if a robot is placed on its own groundlinde between the posts of its own goal. */
  bool isRobotPlacedOnItsGoalLine(int robotNum);

  /** Moves all players out of the penalty area that are not allowed there during a penalty kick. */
  void removeIllegalPlayersFromPenaltyArea(Vector2f ballPosition);

  /**
   * Write the current information of the team to the stream
   * provided.
   * @param teamInfo The team information (either own or opponent).
   * @param stream The stream the team information is written to.
   */
  void writeTeamInfo(TeamInfo& teamInfo, Out& stream);

public:
  bool automatic; /**< Are the automatic features active? */

  /** Constructor */
  GameController();
  ~GameController();

  static GameController& getInstance() { return *theInstance; }

  /**
   * Each simulated robot must be registered.
   * @param robot The number of the robot [0 ... numOfRobots-1].
   * @param simulatedRobot The simulation interface of that robot.
   */
  void registerSimulatedRobot(int robot, SimulatedRobot& simulatedRobot);

  /**
   * Handles the parameters of the console command "gc".
   * @param stream The stream that provides the parameters.
   * @return Were the parameters correct?
   */
  bool handleGlobalConsole(In& stream);

  /**
   * Handles the command "gc".
   * @param command The second part of the command (without "gc").
   */
  bool handleGlobalCommand(const std::string& command);

  /**
   * Handles the parameters of the console command "pr".
   * @param robot The number of the robot that received the command.
   * @param stream The stream that provides the parameters.
   * @return Were the parameters correct?
   */
  bool handleRobotConsole(int robot, In& stream);

  /** Executes the automatic referee. */
  void referee();

  /**
  * Proclaims which robot touched the ball at last
  * @param robot The robot
  */
  void setLastBallContactRobot(SimRobot::Object* robot);

  /**
   * @brief Decreases message budget counter by one.
   * @param robot The robot
   */
  void decreaseMessageBudget(int robot);

  /**
   * Write the current game information to the stream provided.
   * @param stream The stream the game information is written to.
   */
  void writeGameInfo(Out& stream);

  /**
   * Write the current information of the own team to the stream
   * provided.
   * @param robot A robot from the team.
   * @param stream The stream the team information is written to.
   */
  void writeOwnTeamInfo(int robot, Out& stream);

  /**
   * Write the current information of the opponent team to the
   * stream provided.
   * @param robot A robot from the team.
   * @param stream The stream the team information is written to.
   */
  void writeOpponentTeamInfo(int robot, Out& stream);

  void setTeamColors(uint8_t firstFieldPlayerColor, uint8_t firstGoalkeeperColor, uint8_t secondFieldPlayerColor, uint8_t secondGoalkeeperColor);

  /**
   * Write the current information of a certain robot to the
   * stream provided.
   * @param robot The robot the information is about.
   * @param stream The stream the robot information is written to.
   */
  void writeRobotInfo(int robot, Out& stream);

  /**
   * Adds all commands of this module to the set of tab completion
   * strings.
   * @param completion The set of tab completion strings.
   */
  void addCompletion(std::set<std::string>& completion) const;

  /**
   * Retrieves the SimulatedRobot object of the given robot id
   * @param robot The number of the robot 
   */
  const SimulatedRobot* getSimulatedRobot(int robot);

  /**
   * Retrieves the RobotInfo object of the given robot id
   * @param robot The number of the robot 
   */
  const RobotInfo& getRobotInfo(int robot);
};
