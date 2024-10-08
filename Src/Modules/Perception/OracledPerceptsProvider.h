/**
* @file Modules/Infrastructure/OracledPerceptsProvider.h
*
* This file implements a module that provides percepts based on simulated data.
*
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
* TODO (dortmund): use our percepts
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/CLIPGoalPercept.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Perception/CenterCirclePercept.h"
#include "Representations/Perception/PenaltyCrossPercept.h"
#include "Representations/Perception/RobotsPercept.h"

MODULE(OracledPerceptsProvider,
  REQUIRES(GroundTruthWorldState),
  REQUIRES(FrameInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(FieldDimensions),
  REQUIRES(OwnTeamInfo),
  PROVIDES(BallPercept),
  PROVIDES(MultipleBallPercept),
  PROVIDES(CLIPGoalPercept),
  PROVIDES(CLIPFieldLinesPercept),
  PROVIDES(CLIPCenterCirclePercept),
  PROVIDES(RobotsPercept),
  PROVIDES(PenaltyCrossPercept),
  LOADS_PARAMETERS(,
    (bool)  applyBallNoise,                    /**< Activate / Deactivate noise for ball percepts */
    (float) ballCenterInImageStdDevInPixel,    /**< Standard deviation of error in pixels (x as well as y) */
    (float) ballMaxVisibleDistance,            /**< Maximum distance until which this object can be seen */
    (float) ballRecognitionRateMin,            /**< Likelihood of actually perceiving this object, when it is in the field of view and far away */
    (float) ballRecognitionRateMax,            /**< Likelihood of actually perceiving this object, when it is in the field of view and near */
    (bool)  applyCenterCircleNoise,            /**< Activate / Deactivate noise for center circle percepts */
    (float) centerCircleCenterInImageStdDevInPixel,   /**< Standard deviation of error in pixels (x as well as y) */
    (float) centerCircleMaxVisibleDistance,    /**< Maximum distance until which this object can be seen */
    (float) centerCircleRecognitionRateMin,    /**< Likelihood of actually perceiving this object, when it is in the field of view and far away */
    (float) centerCircleRecognitionRateMax,    /**< Likelihood of actually perceiving this object, when it is in the field of view and near */
    (bool)  applyLineNoise,                    /**< Activate / Deactivate noise for line percepts */
    (float) linePosStdDevInmm,                 /**< Standard deviation of error in mm */
    (float) lineMaxVisibleDistance,            /**< Maximum distance until which this object can be seen */
    (float) lineRecognitionRateMin,            /**< Likelihood of actually perceiving this object, when it is in the field of view and far away */
    (float) lineRecognitionRateMax,            /**< Likelihood of actually perceiving this object, when it is in the field of view and near */
    (bool)  applyPlayerNoise,                  /**< Activate / Deactivate noise for player percepts */
    (float) playerPosInImageStdDevInPixel,     /**< Standard deviation of error in pixels (x as well as y) */
    (float) playerMaxVisibleDistance,          /**< Maximum distance until which this object can be seen */
    (float) playerRecognitionRateMin,          /**< Likelihood of actually perceiving this object, when it is in the field of view and far away */
    (float) playerRecognitionRateMax,          /**< Likelihood of actually perceiving this object, when it is in the field of view and near */
    (float) teamRecognitionRateMin,            /**< Likelihood of actually perceiving this jersey, when it is in the field of view and near */
    (float) teamRecognitionRateMax,            /**< Likelihood of actually perceiving this jersey, when it is in the field of view and near */
    (bool)  applyGoalPostNoise,                /**< Activate / Deactivate noise for goal percepts */
    (float) goalPostPosInImageStdDevInPixel,   /**< Standard deviation of error in pixels (x as well as y) */
    (float) goalPostMaxVisibleDistance,        /**< Maximum distance until which this object can be seen */
    (float) goalPostRecognitionRateMin,        /**< Likelihood of actually perceiving this object, when it is in the field of view and far away */
    (float) goalPostRecognitionRateMax,        /**< Likelihood of actually perceiving this object, when it is in the field of view and near */
    (bool)  applyPenaltyMarkNoise,             /**< Activate / Deactivate noise for penalty marks */
    (float) penaltyMarkPosInImageStdDevInPixel,/**< Standard deviation of error in pixels (x as well as y) */
    (float) penaltyMarkMaxVisibleDistance,     /**< Maximum distance until which this object can be seen */
    (float) penaltyMarkRecognitionRateMin,     /**< Likelihood of actually perceiving this object, when it is in the field of view and far away */
    (float) penaltyMarkRecognitionRateMax     /**< Likelihood of actually perceiving this object when it is in the field of view and near */
  )
);

/**
* @class OracledPerceptsProvider
* A module that provides several percepts
*/
class OracledPerceptsProvider : public OracledPerceptsProviderBase
{
public:
  /** Constructor*/
  OracledPerceptsProvider();

private:
  std::vector<Vector2f> goalPosts; /**< The positions of the four goal posts (needed for computing goal percepts)*/
  std::vector<Vector2f> penaltyMarks; /**< The positions of the two penalty marks (needed for computing penalty mark percepts)*/
  std::vector<Vector2f> ccPoints; /**< The positions of five center circle points (needed for computing center circle percept)*/
  std::vector<std::pair<Vector2f, Vector2f>> lines; /**< The lines on the field */
  Vector2f viewPolygon[4]; /**< A polygon that describes the currently visible area in lower image*/
  Vector2f viewPolygonUpper[4]; /**< A polygon that describes the currently visible area in upper image*/

  /** One main function, might be called every cycle
  * @param ballPercept The data struct to be filled
  */
  void update(BallPercept& ballPercept);

  /** One main function, might be called every cycle
  * @param ballPercept The data struct to be filled
  */
  void update(MultipleBallPercept& multipleBallPercept);

  void updateBallPercept(BallPercept& ballPercept, size_t index);

  /** One main function, might be called every cycle
  * @param goalPercept The data struct to be filled
  */
  void update(CLIPGoalPercept& goalPercept);

  /** One main function, might be called every cycle
  * @param linePercept The data struct to be filled
  */
  void update(CLIPFieldLinesPercept& linePercept);

  /** One main function, might be called every cycle
   * @param penaltyMarkPercept The data struct to be filled
   */
  void update(PenaltyCrossPercept& penaltyMarkPercept);

  /** One main function, might be called every cycle
  * @param playersPercept The data struct to be filled
  */
  void update(RobotsPercept& playersPercept);

  /** One main function, might be called every cycle
  * @param centerCirclePercept The data struct to be filled
  */
  void update(CLIPCenterCirclePercept& centerCirclePercept);

  /** Converts a ground truth player to a perceived player and adds it to the percept
  * @param player The ground truth player
  * @param isBlue true, if the perceived player belongs to the blue team
  * @param playersPercept The players percept (What else?)
  */
  void createPlayerBox(const GroundTruthWorldState::GroundTruthPlayer& player, bool isOpponent, RobotsPercept& playersPercept);

  /** Checks, if a point on the field (relative to the robot) is inside the current image
  * @param  p    The point
  * @param  pImg The point projected to the current image
  * @return      true, if the point can be seen by the robot
  */
  bool pointIsInImage(const Vector2f& p, Vector2f& pImg, bool& upper) const;

  /** Computes some noise and adds it to the given position
  * @param standardDeviation The standard deviation of the pixel error
  * @param p The point in an image that is subject to noise
  */
  void applyNoise(float standardDeviation, Vector2f& p) const;

  /** Computes some noise and adds it to the given position (integer version)
  * @param standardDeviation The standard deviation of the pixel error
  * @param p The point in an image that is subject to noise
  */
  void applyNoise(float standardDeviation, Vector2i& p) const;

  /** Updates viewPolygon member */
  void updateViewPolygon(bool upper);

  /** Checks if a line (or parts of it) is inside the view polygon
  * @param line The line to be checked
  * @param start The start of the part that is inside the view polygon (set by this method)
  * @param end The end of the part that is inside the view polygon (set by this method)
  * @return true, if at aleast a part of the line is visible
  */
  bool partOfLineIsVisible(const std::pair<Vector2f, Vector2f>& line, Vector2f& start, Vector2f& end, bool upper) const;

  /** adds a line percept to list of lines */
  void addLinePercept(const bool upper, const Vector2f& start, const Vector2f& end, const Pose2f& robotPoseInv, CLIPFieldLinesPercept& linePercept);
};
