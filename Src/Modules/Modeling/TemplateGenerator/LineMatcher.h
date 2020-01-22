/**
*
* This file contains the declaration of a LineMatcher.
* The LineMatcher's purpose is to generate position hypotheses from line observations.
*
* @author <a href="mailto:stefan.tasse@tu-dortmund.de">Stefan Tasse</a>
*/

#pragma once

#include <algorithm>

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
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(CLIPCenterCirclePercept),
  REQUIRES(CLIPFieldLinesPercept),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  PROVIDES_WITHOUT_MODIFY(LineMatchingResult),
});


class LineMatcher : public LineMatcherBase
{
public:
  
  LineMatcher();
  ~LineMatcher();

  /** Executes this module
  */
  void update(LineMatchingResult& theLineMatchingResult);


private:

  enum
  {
    numberOfFieldLinesX = 6,
    numberOfFieldLinesY = 7,
    numberOfFieldLinesTotal = numberOfFieldLinesX+numberOfFieldLinesY
  };

  class Parameters : public Streamable
  {
  public:
    Parameters();
    double relativeAllowedDistanceErrorForLineClustering;
    double absoluteAllowedDistanceErrorForLineClustering; // for very close lines, the relative distance might be only millimeters
    bool allowPosesOutsideOfCarpet;
    
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM( relativeAllowedDistanceErrorForLineClustering);
      STREAM( absoluteAllowedDistanceErrorForLineClustering);
      STREAM( allowPosesOutsideOfCarpet);
      STREAM_REGISTER_FINISH;
    }
  };

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
      double temp = -1*min;
      min = -1*max;
      max = temp;
    };
  };

  bool preInitDone;
  inline void preExecuteInit(LineMatchingResult& theLineMatchingResult);

  void doGlobalDebugging();

  unsigned lastUpdateTime;

  void execute(LineMatchingResult& theLineMatchingResult);

  double determineMainDirection();

  void buildLineClusters(LineMatchingResult& theLineMatchingResult);

  void findPossiblePositions(LineMatchingResult & theLineMatchingResult);
  void findPossiblePoseIntervals(LineMatchingResult & theLineMatchingResult);

  inline void resetToStartingCorrespondences();
  inline void resetToStartingCorrespondencesForIntervals();

  inline void rotateObservationsBy90Degree();

  inline bool getNextCorrespondenceCombination(int totalNumberOfObservedLines, int conflictingPosition);

  inline bool checkCombinatorialValidityOfCorrespondences(int totalNumberOfObservedLines, int & conflictingPosition);
  
  inline bool checkGeometricValidityOfCorrespondences(int totalNumberOfObservedLines, int &conflictingPosition, Pose2f& poseHypothesis);
  inline bool checkGeometricValidityOfCorrespondencesForIntervals(int totalNumberOfObservedLines, int &conflictingPosition, Pose2f& poseIntervalHypothesisStart, Pose2f& poseIntervalHypothesisEnd);

  inline bool doesObservationFitModel(const AbstractLine & observationInRelativeCoords, const AbstractLine & modelInRelativeCoords);
  inline bool doesObservationFitModelForInterval(const AbstractLine & observationInRelativeCoords, const AbstractLine & modelInRelativeCoords, double & minPositionInLineDirection, double & maxPositionInLineDirection);

  inline void addPoseToLineMatchingResult(const Pose2f & pose, LineMatchingResult & theLineMatchingResult);
  inline void addPoseIntervalToLineMatchingResult(const Pose2f & start, const Pose2f & end, LineMatchingResult & theLineMatchingResult);
  
  // helpful variables
  double mainDirection;
  int correspondences[numberOfFieldLinesTotal]{ 0 };
  bool correspondenceInMainDirectionClass[numberOfFieldLinesTotal]{ 0 }; // true for mainDirection
  int correspondenceIndexOfOrigin[numberOfFieldLinesTotal]{ 0 };
  bool alreadyAssignedFieldLinesX[numberOfFieldLinesX]{ 0 };
  bool alreadyAssignedFieldLinesY[numberOfFieldLinesY]{ 0 };
  std::vector<AbstractLine> linesInMainDirection;
  std::vector<AbstractLine> lines90DegreeToMainDirection;
  std::vector<AbstractLine> tempLines;
  std::vector<AbstractLine> fieldLinesX;
  std::vector<AbstractLine> fieldLinesY;

  // stuff for mapping the original observations to the associated field line correspondences
  int observationMapToInternalIndex[numberOfFieldLinesTotal]{ 0 };
  bool observationMapToInternalMainDirectionClass[numberOfFieldLinesTotal]{ 0 }; // true for mainDirection
  int mainDirectionClassIndex2correspondenceIndex[numberOfFieldLinesTotal]{ 0 };
  int notMainDirectionClassIndex2correspondenceIndex[numberOfFieldLinesTotal]{ 0 };

  // variables for debugging
  int counterForDrawing;
};

