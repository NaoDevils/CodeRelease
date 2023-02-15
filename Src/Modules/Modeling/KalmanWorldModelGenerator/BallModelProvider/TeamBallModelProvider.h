/**
 * \file TeamBallModelProvider.h
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 *
 * Declaration of TeamBallModelProvider which is a part of the module 
 * \c BallModelProvider. It merges local and remote ball models to the 
 * \c TeamBallModel by deciding which of these models is currently most 
 * appropriate.
 */

#pragma once


// ------------- NAO-Framework includes --------------
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h" // frame timestamp
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/RobotPose.h" // robot pose (warning: it's the pose from last frame)
#include "Representations/Modeling/TeamBallModel.h"

// ------------- BallModelProvider includes --------------
#include "TeamBallModelParameters.h"
#include "Models/MultipleBallModel.h"


class TeamBallModelProvider
{
public:
  /** 
   * Constructor.
   */
  TeamBallModelProvider() {}

  /** 
   * Destructor.
   */
  ~TeamBallModelProvider() {}


  // TeamBallModelProvider methods.

  /**
   * Initialize the \c TeamBallModelProvider.
   * \param [in] teamParameters Set the parameters used for merging local
   *                            and remote ball models.
   */
  void initialize(TeamBallModelParameters& teamParameters);

  /**
   * This method merges the \c localBallModel and the \c remoteBallModel into 
   * the \c teamBallModel. While the local ball model is good enough (validity) 
   * it will always be the team ball model. Only if the local ball model validity 
   * is too low and the remote ball model validity is high enough, the remote 
   * ball model will be choosen.
   * \param [out] teamBallModel The team ball model (global field coordinates)
   *                            mearged from local and remote ball model.
   * \param [in] localMultipleBallModel All hypotheses of the local ball model 
   *                                    (relative robot coordinates).
   * \param [in] remoteMultipleBallModel All hypotheses of the remote ball model
   *                                     (global field coordinates).
   * \param [in] theRobotPose Contains the robot pose of the last frame(!), because 
   *                          the ballmodel is required by the \c SelfLocator.
   * \param [in] theFrameInfo Contains the timestamp of the current frame.
   * \see BallModel, TeamBallModel, MultipleBallModel, BallModelProvider
   */
  void generateTeamBallModel(
      TeamBallModel& teamBallModel, LocalMultipleBallModel& localMultipleBallModel, RemoteMultipleBallModel& remoteMultipleBallModel, const RobotPose& theRobotPose, const FrameInfo& theFrameInfo);

private:
  /**
   * Fills the \c teamBallModel with the contents of the \c bestLocalHypothesis.
   * \param [out] teamBallModel The team ball model (global field coordinates)
   *                            generated from local ball model.
   * \param [in] hypothesis Pointer to the ball hypothesis which is used to fill 
   *                        the \c teamBallModel.
   * \param [in] theRobotPose Pointer to the \c RobotPose which is used to transform 
   *                          a local ball hypothesis into global field coordinates.
   *                          If \c nullptr the ball hypothesis is not transformed.
   * \see generateTeamBallModel
   */
  static void generateTeamBallModelFromBallHypothesis(TeamBallModel& teamBallModel, const KalmanPositionHypothesis* hypothesis, const RobotPose* theRobotPose = nullptr);


  // Debug methods.

  /**
   * Allows modification of some parameters in SimRobot.
   */
  void m_initModify();

  /**
   * Draw debug plots from local and remote validity to visualise the decision 
   * for a team ball model.
   * \param [in] localValidity Validity of the best local hypothesis.
   * \param [in] remoteValidity Validity of the best remote hypothesis.
   * \param [in] localHypothesesCount Number of local hypotheses.
   * \param [in] remoteHypothesesCount Number of remote hypotheses.
   * \param [in] bestLocalHypothesis Best local hypothesis.
   * \param [in] bestRemoteHypothesis Best remote hypothesis.
   */
  void m_plot(float localValidity, float remoteValidity, size_t localHypothesesCount, size_t remoteHypothesesCount, const KalmanPositionHypothesis* bestLocalHypothesis, const KalmanPositionHypothesis* bestRemoteHypothesis) const;

private:
  /**
   * The \c TeamBallModel which is provided by this module.
   * This is a combination of local and remote ball models.
   */
  TeamBallModel m_teamBallModel;

  /**
   * Parameters used by \c TeamBallModelProvider for merging local and remote
   * ball models to team ball model.
   */
  TeamBallModelParameters m_teamParameters;
};
