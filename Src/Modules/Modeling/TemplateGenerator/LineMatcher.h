/**
*
* This file contains the declaration of a LineMatcher.
* The LineMatcher's purpose is to generate position hypotheses from line observations.
*
* @author <a href="mailto:stefan.tasse@tu-dortmund.de">Stefan Tasse</a>
*/

#pragma once

#include <algorithm>
#include <vector>

#include "Tools/Module/Module.h"
#include "Representations/Perception/CenterCirclePercept.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/LineMatchingResult.h"
#include "Tools/Math/Pose2f.h"
#include "stdint.h"

//#include <math.h>


MODULE(LineMatcher,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(CLIPCenterCirclePercept),
  REQUIRES(CLIPFieldLinesPercept),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  PROVIDES_WITHOUT_MODIFY(LineMatchingResult)
);


class LineMatcher : public LineMatcherBase
{
public:
  LineMatcher();
  ~LineMatcher();

  /** Executes this module
  */
  void update(LineMatchingResult& theLineMatchingResult);


private:
  int numberOfFieldLinesX = 10;
  int numberOfFieldLinesY = 9;
  int numberOfFieldLinesTotal = numberOfFieldLinesX + numberOfFieldLinesY;

  STREAMABLE(Parameters,,
    (double)(0.15) relativeAllowedDistanceErrorForLineClustering,
    (double)(200.0) absoluteAllowedDistanceErrorForLineClustering, // for very close lines, the relative distance might be only millimeters
    (bool)(false) allowPosesOutsideOfCarpet
  );

  Parameters parameters;

  class AbstractLine
  {
  public:
    AbstractLine();
    AbstractLine(double _offset, double _min, double _max);

    double offset; // perpendicular to the mainDirection of its class
    double min; // min coordinate along mainDirection
    double max; // max coordinate along mainDirection
    int numberOfContributingLines;

    inline void mirrorCoordinates()
    {
      offset *= -1;
      double temp = -1 * min;
      min = -1 * max;
      max = temp;
    };
  };

  bool preInitDone;
  inline void preExecuteInit(LineMatchingResult& theLineMatchingResult);

  void doGlobalDebugging();

  void execute(LineMatchingResult& theLineMatchingResult);

  double determineMainDirection();

  void buildLineClusters(LineMatchingResult& theLineMatchingResult);

  void findPossiblePositions(LineMatchingResult& theLineMatchingResult);
  void findPossiblePoseIntervals(LineMatchingResult& theLineMatchingResult);

  inline void resetToStartingCorrespondences();
  inline void resetToStartingCorrespondencesForIntervals();

  inline void rotateObservationsBy90Degree();

  inline bool getNextCorrespondenceCombination(int totalNumberOfObservedLines, int conflictingPosition);

  inline bool checkCombinatorialValidityOfCorrespondences(int totalNumberOfObservedLines, int& conflictingPosition);

  inline bool checkGeometricValidityOfCorrespondences(int totalNumberOfObservedLines, int& conflictingPosition, Pose2f& poseHypothesis);
  inline bool checkGeometricValidityOfCorrespondencesForIntervals(int totalNumberOfObservedLines, int& conflictingPosition, Pose2f& poseIntervalHypothesisStart, Pose2f& poseIntervalHypothesisEnd);

  inline bool doesObservationFitModel(const AbstractLine& observationInRelativeCoords, const AbstractLine& modelInRelativeCoords);
  inline bool doesObservationFitModelForInterval(
      const AbstractLine& observationInRelativeCoords, const AbstractLine& modelInRelativeCoords, double& minPositionInLineDirection, double& maxPositionInLineDirection);

  inline void addPoseToLineMatchingResult(const Pose2f& pose, LineMatchingResult& theLineMatchingResult);
  inline void addPoseIntervalToLineMatchingResult(const Pose2f& start, const Pose2f& end, LineMatchingResult& theLineMatchingResult);

  unsigned int lastFieldDimensionsUpdate = 0;

  // helpful variables
  double mainDirection;
  std::vector<int> correspondences;
  std::vector<bool> correspondenceInMainDirectionClass; // true for mainDirection
  std::vector<int> correspondenceIndexOfOrigin;
  std::vector<bool> alreadyAssignedFieldLinesX;
  std::vector<bool> alreadyAssignedFieldLinesY;
  std::vector<AbstractLine> linesInMainDirection;
  std::vector<AbstractLine> lines90DegreeToMainDirection;
  std::vector<AbstractLine> tempLines;
  std::vector<AbstractLine> fieldLinesX;
  std::vector<AbstractLine> fieldLinesY;

  // stuff for mapping the original observations to the associated field line correspondences
  std::vector<int> observationMapToInternalIndex;
  std::vector<bool> observationMapToInternalMainDirectionClass; // true for mainDirection
  std::vector<int> mainDirectionClassIndex2correspondenceIndex;
  std::vector<int> notMainDirectionClassIndex2correspondenceIndex;

  // variables for debugging
  int counterForDrawing;
};
