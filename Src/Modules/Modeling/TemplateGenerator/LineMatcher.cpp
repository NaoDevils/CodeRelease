#include "LineMatcher.h"


LineMatcher::Parameters::Parameters() :
  relativeAllowedDistanceErrorForLineClustering(0.15),
  absoluteAllowedDistanceErrorForLineClustering(200),
  allowPosesOutsideOfCarpet(false)
{
}

LineMatcher::AbstractLine::AbstractLine()
{
}

LineMatcher::AbstractLine::AbstractLine(double _offset, double _min, double _max) :
  offset(_offset),
  min(_min),
  max(_max),
  numberOfContributingLines(1)
{
}

LineMatcher::LineMatcher() :
  preInitDone(false),
  lastUpdateTime(0)
{
}
LineMatcher::~LineMatcher() {


}

void LineMatcher::update(LineMatchingResult& theLineMatchingResult)
{
  preExecuteInit(theLineMatchingResult);

  execute(theLineMatchingResult);

  theLineMatchingResult.calculateObservationsSphericalCoords();
}

void LineMatcher::doGlobalDebugging()
{
  MODIFY("module:LineMatcher:parameters", parameters);
  DECLARE_DEBUG_DRAWING("module:LineMatcher:lineCluster", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LineMatcher:fieldLineSegmentsFromSpecification", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LineMatcher:possiblePositions", "drawingOnField");


  // just to check the field line specification
  COMPLEX_DRAWING("module:LineMatcher:fieldLineSegmentsFromSpecification")
  {
    int counter = 0;
    for (std::vector<AbstractLine>::const_iterator i = fieldLinesX.begin(); i != fieldLinesX.end(); ++i)
    {
      LINE("module:LineMatcher:fieldLineSegmentsFromSpecification",
        i->min, i->offset, i->max, i->offset, 30, Drawings::solidPen, ColorRGBA(255, 150, 255));
      DRAWTEXT("module:LineMatcher:fieldLineSegmentsFromSpecification", (i->min + i->max) / 2 + 40, i->offset + 40, 100, ColorRGBA(255, 150, 255), counter);
      counter++;
    }
    counter = 0;
    for (std::vector<AbstractLine>::const_iterator i = fieldLinesY.begin(); i != fieldLinesY.end(); ++i)
    {
      LINE("module:LineMatcher:fieldLineSegmentsFromSpecification",
        -i->offset, i->min, -i->offset, i->max, 30, Drawings::solidPen, ColorRGBA(200, 100, 255));
      DRAWTEXT("module:LineMatcher:fieldLineSegmentsFromSpecification", -i->offset + 40, (i->min + i->max) / 2 + 40, 100, ColorRGBA(200, 100, 255), counter);
      counter++;
    }
  }
}

inline void LineMatcher::preExecuteInit(LineMatchingResult & theLineMatchingResult)
{
  if (!preInitDone)
  {
    // initialize stuff
    fieldLinesX.clear();
    fieldLinesY.clear();
    theLineMatchingResult.fieldLines.clear();
    theLineMatchingResult.reset();


    // It is important that those field lines are in the same order in theLineMatchingResult,
    // to allow the recovery of correspondences later.

    // field lines (segments) in x-direction
    fieldLinesX.push_back(AbstractLine( // right sideline; 0
      theFieldDimensions.yPosRightSideline,
      theFieldDimensions.xPosOwnGroundline,
      theFieldDimensions.xPosOpponentGroundline));
    theLineMatchingResult.fieldLines.push_back(
      LineMatchingResult::FieldLine(
        Vector2d(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline),
        Vector2d(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline),
        0));

    fieldLinesX.push_back(AbstractLine( // left sideline; 1
      theFieldDimensions.yPosLeftSideline,
      theFieldDimensions.xPosOwnGroundline,
      theFieldDimensions.xPosOpponentGroundline));
    theLineMatchingResult.fieldLines.push_back(
      LineMatchingResult::FieldLine(
        Vector2d(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline),
        Vector2d(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline),
        0));

    fieldLinesX.push_back(AbstractLine( // right own penalty line; 2
      theFieldDimensions.yPosRightPenaltyArea,
      theFieldDimensions.xPosOwnGroundline,
      theFieldDimensions.xPosOwnPenaltyArea));
    theLineMatchingResult.fieldLines.push_back(
      LineMatchingResult::FieldLine(
        Vector2d(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightPenaltyArea),
        Vector2d(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea),
        0));

    fieldLinesX.push_back(AbstractLine( // left own penalty line; 3
      theFieldDimensions.yPosLeftPenaltyArea,
      theFieldDimensions.xPosOwnGroundline,
      theFieldDimensions.xPosOwnPenaltyArea));
    theLineMatchingResult.fieldLines.push_back(
      LineMatchingResult::FieldLine(
        Vector2d(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea),
        Vector2d(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea),
        0));

    fieldLinesX.push_back(AbstractLine( // right opp penalty line; 4
      theFieldDimensions.yPosRightPenaltyArea,
      theFieldDimensions.xPosOpponentPenaltyArea,
      theFieldDimensions.xPosOpponentGroundline));
    theLineMatchingResult.fieldLines.push_back(
      LineMatchingResult::FieldLine(
        Vector2d(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea),
        Vector2d(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightPenaltyArea),
        0));

    fieldLinesX.push_back(AbstractLine( // left opp penalty line; 5
      theFieldDimensions.yPosLeftPenaltyArea,
      theFieldDimensions.xPosOpponentPenaltyArea,
      theFieldDimensions.xPosOpponentGroundline));
    theLineMatchingResult.fieldLines.push_back(
      LineMatchingResult::FieldLine(
        Vector2d(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea),
        Vector2d(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftPenaltyArea),
        0));


    // field lines (segments) in y-direction
    // Note: A negative x in field coordinates is
    // a positive offset in the rotated
    // AbstractLine coordinate system.
    // Min and max are min-y and max-y, respectively.
    fieldLinesY.push_back(AbstractLine( // own groundline; 6
      -theFieldDimensions.xPosOwnGroundline,
      theFieldDimensions.yPosRightSideline,
      theFieldDimensions.yPosLeftSideline));
    theLineMatchingResult.fieldLines.push_back(
      LineMatchingResult::FieldLine(
        Vector2d(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline),
        Vector2d(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline),
        0));

    fieldLinesY.push_back(AbstractLine( // middle line; 7
      -theFieldDimensions.xPosHalfWayLine,
      theFieldDimensions.yPosRightSideline,
      theFieldDimensions.yPosLeftSideline));
    theLineMatchingResult.fieldLines.push_back(
      LineMatchingResult::FieldLine(
        Vector2d(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosRightSideline),
        Vector2d(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosLeftSideline),
        0));

    fieldLinesY.push_back(AbstractLine( // opp groundline; 8
      -theFieldDimensions.xPosOpponentGroundline,
      theFieldDimensions.yPosRightSideline,
      theFieldDimensions.yPosLeftSideline));
    theLineMatchingResult.fieldLines.push_back(
      LineMatchingResult::FieldLine(
        Vector2d(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline),
        Vector2d(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline),
        0));

    fieldLinesY.push_back(AbstractLine( // own penaltyline; 9
      -theFieldDimensions.xPosOwnPenaltyArea,
      theFieldDimensions.yPosRightPenaltyArea,
      theFieldDimensions.yPosLeftPenaltyArea));
    theLineMatchingResult.fieldLines.push_back(
      LineMatchingResult::FieldLine(
        Vector2d(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea),
        Vector2d(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea),
        0));

    fieldLinesY.push_back(AbstractLine( // opponent penaltyline; 10
      -theFieldDimensions.xPosOpponentPenaltyArea,
      theFieldDimensions.yPosRightPenaltyArea,
      theFieldDimensions.yPosLeftPenaltyArea));
    theLineMatchingResult.fieldLines.push_back(
      LineMatchingResult::FieldLine(
        Vector2d(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea),
        Vector2d(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea),
        0));

    fieldLinesY.push_back(AbstractLine( // own penaltyline; 11
      -theFieldDimensions.xPosOwnGoal,
      theFieldDimensions.yPosRightGoal,
      theFieldDimensions.yPosLeftGoal));
    theLineMatchingResult.fieldLines.push_back(
      LineMatchingResult::FieldLine(
        Vector2d(theFieldDimensions.xPosOwnGoal, theFieldDimensions.yPosRightGoal),
        Vector2d(theFieldDimensions.xPosOwnGoal, theFieldDimensions.yPosLeftGoal),
        0));

    fieldLinesY.push_back(AbstractLine( // opponent penaltyline; 12
      -theFieldDimensions.xPosOpponentGoal,
      theFieldDimensions.yPosRightGoal,
      theFieldDimensions.yPosLeftGoal));
    theLineMatchingResult.fieldLines.push_back(
      LineMatchingResult::FieldLine(
        Vector2d(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosRightGoal),
        Vector2d(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosLeftGoal),
        0));


    preInitDone = true;
  }
}


void LineMatcher::execute(LineMatchingResult & theLineMatchingResult)
{
  if (theFrameInfo.time == lastUpdateTime)
  {
    return;
  }
  lastUpdateTime = theFrameInfo.time;

  // reset theLineMatchingResult
  theLineMatchingResult.reset();
  for (const auto& i : theCLIPFieldLinesPercept.lines)
  {
    LineMatchingResult::FieldLine obs(i.startOnField.cast<double>(), i.endOnField.cast<double>(),
      (i.fromUpper ? theCameraMatrix.translation.z() : theCameraMatrixUpper.translation.z()));

    // Plausability check if observed line is in front of robot
    // Still allow lines observed behind robot but one side only (look over shoulder)
    if ((obs.start.x() > 0. && obs.end.x() > 0.) || (sgn(obs.end.y()) == sgn(obs.end.y())))
      theLineMatchingResult.observations.push_back(std::move(obs));
  }

  doGlobalDebugging();


  mainDirection = determineMainDirection();

  buildLineClusters(theLineMatchingResult);

  if (linesInMainDirection.size() > 0 && lines90DegreeToMainDirection.size() > 0)
  {
    findPossiblePositions(theLineMatchingResult);
  }
  else if (linesInMainDirection.size() > 0 || lines90DegreeToMainDirection.size() > 0)
  {
    findPossiblePoseIntervals(theLineMatchingResult);
    theLineMatchingResult.onlyObservedOneFieldLine = (linesInMainDirection.size() + lines90DegreeToMainDirection.size()) == 1;
  }
}

double LineMatcher::determineMainDirection()
{
  static const double tenDegrees = Angle::fromDegrees(10);

  // 10° steps, each bucket is supposed to hold
  // a 30° slice, which is the needed resolution,
  // and the overlapping garantees acceptable results.
  // I don't see a reason not to hardcode those numbers,
  // as this will not be a subject to any parameter tuning.
  int accumulatorCount[9]{ 0 };
  double directionAccumulator[9]{ 0 };

  // "accumulatorCount[0] = 1" means there is 1 line
  // with a direction between 0° and 30° (or +90°)

  for (auto& i : theCLIPFieldLinesPercept.lines)
  {
    float direction = (i.endOnField - i.startOnField).angle();
    //direction = (direction + pi2) % pi_2; // so it is positive and in the first quadrant
    // shitty C++ does not know modulo for non-integers...
    float temp = direction + pi2;                            // so it is positive...
    direction = temp - static_cast<int>(temp / pi_2) * pi_2; // ...and in the first quadrant

    int index = static_cast<int>(direction / tenDegrees); // [0..8]

    double wraparound1 = 0, wraparound2 = 0;
    if (index == 0)
    {
      wraparound1 = pi_2;
      wraparound2 = pi_2;
    }
    else if (index == 1)
    {
      wraparound2 = pi_2;
    }

    // so "index == 0" means, the line is in [0°..9°]
    // which should increase accumulatorCount[i] for i={7,8,0}
    int index2 = (index + 8) % 9;
    int index3 = (index + 7) % 9;
    accumulatorCount[index]++;
    accumulatorCount[index2]++;
    accumulatorCount[index3]++;
    directionAccumulator[index] += direction;
    directionAccumulator[index2] += direction + wraparound1;
    directionAccumulator[index3] += direction + wraparound2;
  }

  int indexOfBest = 0;
  int bestCount = -1;
  for (int i = 0; i < 9; i++)
  {
    if (accumulatorCount[i]) // Dont know if this is correct, but dividing by 0 is
                 // definately bullshit, so avoid this
      directionAccumulator[i] /= accumulatorCount[i];
    else
      directionAccumulator[i] = 0;
    if (accumulatorCount[i] > bestCount)
    {
      bestCount = accumulatorCount[i];
      indexOfBest = i;
    }
  }

  mainDirection = directionAccumulator[indexOfBest];
  return mainDirection;
}

void LineMatcher::buildLineClusters(LineMatchingResult & theLineMatchingResult)
{
  linesInMainDirection.clear();
  lines90DegreeToMainDirection.clear();

  Pose2f poseForLineTransformation(0, 0, 0);

  // for each FieldLine, find out into which cluster it belongs to
  int index = 0;
  for (auto i = theLineMatchingResult.observations.cbegin(); i != theLineMatchingResult.observations.cend(); ++i)
  {
    // find the right cluster
    double direction = (i->end - i->start).angle() - mainDirection;
    direction = std::abs(Angle::normalize(direction)); // -> [0..pi]

    std::vector<AbstractLine> * linesCluster;
    if (direction < pi_4 || direction > pi3_4)
    {
      linesCluster = &linesInMainDirection;
      poseForLineTransformation.rotation = static_cast<float>(-mainDirection);
      observationMapToInternalMainDirectionClass[index] = true;
    }
    else
    {
      linesCluster = &lines90DegreeToMainDirection;
      poseForLineTransformation.rotation = static_cast<float>(-mainDirection - pi_2);
      observationMapToInternalMainDirectionClass[index] = false;
    }

    // calculate the beginning and end points in our transformed reference system
    Vector2d start = (poseForLineTransformation * i->start.cast<float>()).cast<double>();
    Vector2d end = (poseForLineTransformation * i->end.cast<float>()).cast<double>();
    Vector2d center = (start + end) / 2;
    double distance = center.y();

    // find if there already is a line in the cluster fitting this line fragment
    // (if the LineFinder did its job right, all line fragments should have already been fused to lines, but better safe than sorry...)
    bool matchFound = false;
    int matchIndex = 0;
    for (auto j = linesCluster->begin(); j != linesCluster->end(); ++j)
    {
      if (std::abs(distance / j->offset - 1) < parameters.relativeAllowedDistanceErrorForLineClustering
        || std::abs(distance - j->offset) < parameters.absoluteAllowedDistanceErrorForLineClustering)
      {
        matchFound = true;

        // merge with line
        j->numberOfContributingLines++;
        j->offset = j->offset * (j->numberOfContributingLines - 1.0) / j->numberOfContributingLines + 1.0 / j->numberOfContributingLines * distance;
        j->min = std::min(j->min, std::min(start.x(), end.x()));
        j->max = std::max(j->max, std::max(start.x(), end.x()));

        observationMapToInternalIndex[index] = matchIndex;

        break;
      }
      matchIndex++;
    }
    if (!matchFound)
    {
      AbstractLine newLine(distance, std::min(start.x(), end.x()), std::max(start.x(), end.x()));
      linesCluster->push_back(newLine);
      observationMapToInternalIndex[index] = static_cast<int>(linesCluster->size()) - 1;
    }
    index++;
  }

  int counter = 0;
  for (auto i = linesInMainDirection.begin(); i != linesInMainDirection.end(); ++i)
  {
    LINE("module:LineMatcher:lineCluster", i->min, i->offset, i->max, i->offset, 30, Drawings::solidPen, ColorRGBA(255, 0, 255));
    DRAWTEXT("module:LineMatcher:lineCluster", (i->min + i->max) / 2 + 40, i->offset + 40, 100, ColorRGBA(255, 0, 255), counter);
    counter++;
  }
  counter = 0;
  for (auto i = lines90DegreeToMainDirection.begin(); i != lines90DegreeToMainDirection.end(); ++i)
  {
    LINE("module:LineMatcher:lineCluster", -i->offset, i->min, -i->offset, i->max, 30, Drawings::solidPen, ColorRGBA(128, 0, 255));
    DRAWTEXT("module:LineMatcher:lineCluster", -i->offset + 40, (i->min + i->max) / 2 + 40, 100, ColorRGBA(128, 0, 255), counter);
    counter++;
  }

  // ok, fine in theory, but to prevent some situation dependent difficult checks later,
  // let's ensure now that we will never have more lines than we have possible field line correspondences
  unsigned int allowedNumber = std::min((int)numberOfFieldLinesX, (int)numberOfFieldLinesY);
  while (linesInMainDirection.size() > allowedNumber)
  {
    linesInMainDirection.pop_back();
  }
  while (lines90DegreeToMainDirection.size() > allowedNumber)
  {
    lines90DegreeToMainDirection.pop_back();
  }
}


void LineMatcher::findPossiblePositions(LineMatchingResult & theLineMatchingResult)
{
  int totalNumberOfObservedLines = static_cast<int>(linesInMainDirection.size() + lines90DegreeToMainDirection.size());

  counterForDrawing = 0;

  int conflictingPosition; // needed later when checking possible correspondences

  // test for 0°, 90°, 180°, 270° rotation
  Pose2f poseHypothesis(static_cast<float>(-mainDirection));
  for (int i = 0; i < 4; i++)
  {
    resetToStartingCorrespondences();
    do
    {
      bool isValidCorrespondenceChoice = checkCombinatorialValidityOfCorrespondences(totalNumberOfObservedLines, conflictingPosition);
      if (isValidCorrespondenceChoice)
      {
        isValidCorrespondenceChoice = checkGeometricValidityOfCorrespondences(totalNumberOfObservedLines, conflictingPosition, poseHypothesis);
        if (isValidCorrespondenceChoice)
        {
          conflictingPosition = totalNumberOfObservedLines - 1;
          addPoseToLineMatchingResult(poseHypothesis, theLineMatchingResult);
        }
      }
    } while (getNextCorrespondenceCombination(totalNumberOfObservedLines, conflictingPosition));
    rotateObservationsBy90Degree();
    poseHypothesis.rotation += pi_2;
  }
}

void LineMatcher::resetToStartingCorrespondences()
{
  // The first two should come from different classes.
  // This simplifies a lot of the later reasoning.

  // incidentally, the starting correspondence is identical to the origin indices in the observations
  int n = static_cast<int>(linesInMainDirection.size());
  int m = static_cast<int>(lines90DegreeToMainDirection.size());

  int i = 0;
  int j = 0;
  correspondences[i + j] = i; // i+j=0
  correspondenceIndexOfOrigin[i + j] = i;
  correspondenceInMainDirectionClass[i + j] = true;
  mainDirectionClassIndex2correspondenceIndex[i] = i + j;
  i++;
  correspondences[i + j] = j; // i+j=1
  correspondenceIndexOfOrigin[i + j] = j;
  correspondenceInMainDirectionClass[i + j] = false;
  notMainDirectionClassIndex2correspondenceIndex[j] = i + j;
  j++;

  for (; i < n; i++)
  {
    correspondences[i + j] = i;
    correspondenceIndexOfOrigin[i + j] = i;
    correspondenceInMainDirectionClass[i + j] = true;
    mainDirectionClassIndex2correspondenceIndex[i] = i + j;
  }
  for (; j < m; j++)
  {
    correspondences[i + j] = j;
    correspondenceIndexOfOrigin[i + j] = j;
    correspondenceInMainDirectionClass[i + j] = false;
    notMainDirectionClassIndex2correspondenceIndex[j] = i + j;
  }
}

void LineMatcher::rotateObservationsBy90Degree()
{
  tempLines.clear();
  for (auto i = linesInMainDirection.cbegin(); i != linesInMainDirection.cend(); ++i)
  {
    tempLines.push_back(*i);
  }
  linesInMainDirection.clear();
  for (auto i = lines90DegreeToMainDirection.begin(); i != lines90DegreeToMainDirection.end(); ++i)
  {
    i->mirrorCoordinates();
    linesInMainDirection.push_back(*i);
  }
  lines90DegreeToMainDirection.clear();
  for (auto i = tempLines.cbegin(); i != tempLines.cend(); ++i)
  {
    lines90DegreeToMainDirection.push_back(*i);
  }

  for (int i = 0; i < numberOfFieldLinesTotal; i++)
  {
    observationMapToInternalMainDirectionClass[i] = !observationMapToInternalMainDirectionClass[i];
  }
}

bool LineMatcher::getNextCorrespondenceCombination(int totalNumberOfObservedLines, int conflictingPosition)
{
  // find position to increment
  int index;
  for (index = conflictingPosition; index >= 0; index--)
  {
    // is it possible to increment this position?
    if ((correspondenceInMainDirectionClass[index] && correspondences[index] + 1 < numberOfFieldLinesX)  // we search for a x-line correspondence
      ||
      (!correspondenceInMainDirectionClass[index] && correspondences[index] + 1 < numberOfFieldLinesY)  // we search for a y-line correspondence
      )
    {
      break; // possible position found!
    }
    // else: try the next (more significant, i.e. i--) position
  }
  if (index < 0)
  {
    // nothing left to increase, we are done.
    return false;
  }
  else
  {
    correspondences[index]++;
    index++;

    // At least we can do something besides the trivial filling up with zeros...
    // TODO: Think of something clever to avoid double correspondences/associations
    //       with the previous positions.
    //       (Checking all the previous fields using alreadyAssignedFieldLinesX seems overkill/inefficient)
    int i = 0; // mainDirection class
    int j = 0; // non-mainDirection class
    for (; index < totalNumberOfObservedLines; index++)
    {
      if (correspondenceInMainDirectionClass[index])
      {
        correspondences[index] = i;
        i++;
      }
      else
      {
        correspondences[index] = j;
        j++;
      }
    }
    // found another correspondence combination
    return true;
  }
}

bool LineMatcher::checkCombinatorialValidityOfCorrespondences(int totalNumberOfObservedLines, int& conflictingPosition)
{
  // reset the "already used" fields
  for (int i = 0; i < numberOfFieldLinesX; i++)
  {
    alreadyAssignedFieldLinesX[i] = false;
  }
  for (int i = 0; i < numberOfFieldLinesY; i++)
  {
    alreadyAssignedFieldLinesY[i] = false;
  }

  // check for "double associations" first (i.e. combinatorial validity)
  for (int i = 0; i < totalNumberOfObservedLines; i++)
  {
    if (correspondenceInMainDirectionClass[i])
    {
      // test for assignment in x
      if (alreadyAssignedFieldLinesX[correspondences[i]])
      {
        conflictingPosition = i;
        return false;
      }
      else
      {
        alreadyAssignedFieldLinesX[correspondences[i]] = true;
      } // ok so far, test the next one...
    }
    else
    {
      // test for assignment in y
      if (alreadyAssignedFieldLinesY[correspondences[i]])
      {
        conflictingPosition = i;
        return false;
      }
      else
      {
        alreadyAssignedFieldLinesY[correspondences[i]] = true;
      } // ok so far, test the next one...
    }
  }
  // passed the "double correspondences" test, so this might be a legal combination
  conflictingPosition = totalNumberOfObservedLines - 1; // last position, just in case somebody uses it even for "return true"
  return true;
}

bool LineMatcher::doesObservationFitModel(const AbstractLine & observationInRelativeCoords, const AbstractLine & modelInRelativeCoords)
{
  // check the distances of the line
  if (std::abs(observationInRelativeCoords.offset / modelInRelativeCoords.offset - 1) < parameters.relativeAllowedDistanceErrorForLineClustering
    || std::abs(observationInRelativeCoords.offset - modelInRelativeCoords.offset) < parameters.absoluteAllowedDistanceErrorForLineClustering)
  {
    // check if the segment might lie outside of the prediction too much
    if ((std::abs(observationInRelativeCoords.min / modelInRelativeCoords.min - 1) < parameters.relativeAllowedDistanceErrorForLineClustering
      || -1 * (observationInRelativeCoords.min - modelInRelativeCoords.min) < parameters.absoluteAllowedDistanceErrorForLineClustering)
      &&
      (std::abs(observationInRelativeCoords.max / modelInRelativeCoords.max - 1) < parameters.relativeAllowedDistanceErrorForLineClustering
        || (observationInRelativeCoords.max - modelInRelativeCoords.max) < parameters.absoluteAllowedDistanceErrorForLineClustering))
    {
      // Using absoluteAllowedDistanceErrorForLineClustering might allow small line fragments on the wrong side of
      // another perpendicular line. Calculate the overlap (how much of the observation is covered by the model)
      // to filter those ones out.
      double overlap = std::max(.0, std::min(modelInRelativeCoords.max, observationInRelativeCoords.max) - std::max(modelInRelativeCoords.min, observationInRelativeCoords.min));
      if (overlap / (observationInRelativeCoords.max - observationInRelativeCoords.min) > 1 - parameters.relativeAllowedDistanceErrorForLineClustering)
      {
        return true;
      }
    }
  }
  return false;
}


bool LineMatcher::checkGeometricValidityOfCorrespondences(int totalNumberOfObservedLines, int& conflictingPosition, Pose2f & poseHypothesis)
{
  // use the first two correspondences (which are of different classes) to uniquely determine the position
  // first is always mainDirection which corresponds to x-lines (side line etc.)

  const AbstractLine& xLineOnField = fieldLinesX[correspondences[0]];
  const AbstractLine& yLineOnField = fieldLinesY[correspondences[1]];
  const AbstractLine& xLineObserved = linesInMainDirection[0];
  const AbstractLine& yLineObserved = lines90DegreeToMainDirection[0];

  poseHypothesis.translation.x() = static_cast<float>(yLineObserved.offset - yLineOnField.offset);
  poseHypothesis.translation.y() = static_cast<float>(xLineOnField.offset - xLineObserved.offset);

  // now check the geometric plausibility
  for (int i = 0; i < totalNumberOfObservedLines; i++)
  {
    const AbstractLine& observationInRelativeCoords = correspondenceInMainDirectionClass[i] ? linesInMainDirection[correspondenceIndexOfOrigin[i]] : lines90DegreeToMainDirection[correspondenceIndexOfOrigin[i]];
    AbstractLine modelInRelativeCoords;
    if (correspondenceInMainDirectionClass[i])
    { // x line
      modelInRelativeCoords = fieldLinesX[correspondences[i]];
      // make is relative
      modelInRelativeCoords.offset -= poseHypothesis.translation.y();
      modelInRelativeCoords.min -= poseHypothesis.translation.x();
      modelInRelativeCoords.max -= poseHypothesis.translation.x();
    }
    else
    { // y line
      modelInRelativeCoords = fieldLinesY[correspondences[i]];
      // make is relative
      modelInRelativeCoords.offset += poseHypothesis.translation.x();
      modelInRelativeCoords.min -= poseHypothesis.translation.y();
      modelInRelativeCoords.max -= poseHypothesis.translation.y();
    }
    if (!doesObservationFitModel(observationInRelativeCoords, modelInRelativeCoords))
    {
      // does not fit. -> Conflict found.
      conflictingPosition = std::max(i, 1);
      return false;
    }
  }
  // passed all tests! Yeah!  :-D
  conflictingPosition = totalNumberOfObservedLines - 1;
  return true;
}

void LineMatcher::addPoseToLineMatchingResult(const Pose2f & pose, LineMatchingResult & theLineMatchingResult)
{
  LineMatchingResult::PoseHypothesis ph;
  ph.pose = pose;
  for (unsigned int i = 0; i < theLineMatchingResult.observations.size(); i++)
  {
    int offset = observationMapToInternalMainDirectionClass[i] ? 0 : numberOfFieldLinesX;
    int indexInCorrespondenceArray = observationMapToInternalMainDirectionClass[i] ? mainDirectionClassIndex2correspondenceIndex[observationMapToInternalIndex[i]] : notMainDirectionClassIndex2correspondenceIndex[observationMapToInternalIndex[i]];
    ph.lineCorrespondences[i] = correspondences[indexInCorrespondenceArray] + offset;
  }
  if (parameters.allowPosesOutsideOfCarpet || theFieldDimensions.isInsideCarpet(ph.pose.translation))
  {
    theLineMatchingResult.poseHypothesis.push_back(ph);
  }

  // for debugging (this draws all poses, even outside the carpet.
  // See theLineMatchingResult if you only want to see the forwarded ones.)
  POSE_2D_SAMPLE("module:LineMatcher:possiblePositions", pose, ColorRGBA(0, 255, 255));
  DRAWTEXT("module:LineMatcher:possiblePositions", pose.translation.x() + 40, pose.translation.y() + 40, 100, ColorRGBA(0, 255, 255), counterForDrawing);
  counterForDrawing++;
}

void LineMatcher::addPoseIntervalToLineMatchingResult(const Pose2f & start, const Pose2f & end, LineMatchingResult & theLineMatchingResult)
{
  LineMatchingResult::PoseHypothesisInterval phi;
  phi.start = start;
  phi.end = end;
  for (unsigned int i = 0; i < theLineMatchingResult.observations.size(); i++)
  {
    int offset = observationMapToInternalMainDirectionClass[i] ? 0 : numberOfFieldLinesX;
    int indexInCorrespondenceArray = observationMapToInternalMainDirectionClass[i] ? mainDirectionClassIndex2correspondenceIndex[observationMapToInternalIndex[i]] : notMainDirectionClassIndex2correspondenceIndex[observationMapToInternalIndex[i]];
    phi.lineCorrespondences[i] = correspondences[indexInCorrespondenceArray] + offset;
  }

  if (parameters.allowPosesOutsideOfCarpet
    || theFieldDimensions.isInsideCarpet(phi.start.translation)
    || theFieldDimensions.isInsideCarpet(phi.end.translation))
  {
    theLineMatchingResult.poseHypothesisIntervals.push_back(phi);
  }

  // for debugging (this draws all poses, even outside the carpet.
  // See theLineMatchingResult if you only want to see the forwarded ones.)
  for (float t = 0.f; t <= 1.f; t += 1.f / 25)
  {
    float s = 1.f - t;
    Pose2f interpolation(t * phi.start.rotation + s * phi.end.rotation,
      t * phi.start.translation.x() + s * phi.end.translation.x(),
      t * phi.start.translation.y() + s * phi.end.translation.y());
    POSE_2D_SAMPLE("module:LineMatcher:possiblePositions", interpolation, ColorRGBA(0, 255, 255));
  }
  DRAWTEXT("module:LineMatcher:possiblePositions", phi.start.translation.x() + 40, phi.start.translation.y() + 40, 100, ColorRGBA(0, 255, 255), counterForDrawing);
  counterForDrawing++;
}

void LineMatcher::findPossiblePoseIntervals(LineMatchingResult & theLineMatchingResult)
{
  int totalNumberOfObservedLines = static_cast<int>(linesInMainDirection.size() + lines90DegreeToMainDirection.size());

  counterForDrawing = 0;

  int conflictingPosition; // needed later when checking possible correspondences

  // test for 0°, 90°, 180°, 270° rotation
  Pose2f poseIntervalHypothesisStart(static_cast<float>(-mainDirection)),
    poseIntervalHypothesisEnd(static_cast<float>(-mainDirection));
  for (int i = 0; i < 4; i++)
  {
    resetToStartingCorrespondencesForIntervals();
    do
    {
      bool isValidCorrespondenceChoice = checkCombinatorialValidityOfCorrespondences(totalNumberOfObservedLines, conflictingPosition);
      if (isValidCorrespondenceChoice)
      {
        isValidCorrespondenceChoice = checkGeometricValidityOfCorrespondencesForIntervals(totalNumberOfObservedLines, conflictingPosition, poseIntervalHypothesisStart, poseIntervalHypothesisEnd);
        if (isValidCorrespondenceChoice)
        {
          conflictingPosition = totalNumberOfObservedLines - 1;
          addPoseIntervalToLineMatchingResult(poseIntervalHypothesisStart, poseIntervalHypothesisEnd, theLineMatchingResult);
        }
      }
    } while (getNextCorrespondenceCombination(totalNumberOfObservedLines, conflictingPosition));
    rotateObservationsBy90Degree();
    poseIntervalHypothesisStart.rotation += pi_2;
    poseIntervalHypothesisEnd.rotation += pi_2;
  }
}

void LineMatcher::resetToStartingCorrespondencesForIntervals()
{
  // this will only be done if observations exist in only one of the classes,
  // and there must be at least 2 observations
  bool allObservatiosInMainClass = linesInMainDirection.size() > 0;

  int n = static_cast<int>(linesInMainDirection.size() + lines90DegreeToMainDirection.size());
  for (int i = 0; i < n; i++)
  {
    correspondences[i] = i;
    correspondenceIndexOfOrigin[i] = i;
    correspondenceInMainDirectionClass[i] = allObservatiosInMainClass;
    // one of the following is not needed... (but this is faster than questioning which)  ;)
    mainDirectionClassIndex2correspondenceIndex[i] = i;
    notMainDirectionClassIndex2correspondenceIndex[i] = i;
  }
}

bool LineMatcher::checkGeometricValidityOfCorrespondencesForIntervals(int totalNumberOfObservedLines, int& conflictingPosition, Pose2f & poseIntervalHypothesisStart, Pose2f & poseIntervalHypothesisEnd)
{
  // This is relatively easy, we only have to distinguish between the observation classes
  // at the end of this procedure to calculate poseIntervalHypothesisStart/poseIntervalHypothesisEnd.
  // The validity check can mostly be done without that knowledge.
  bool allObservatiosInMainClass = linesInMainDirection.size() > 0;
  const std::vector<AbstractLine>& lineObservations = allObservatiosInMainClass ? linesInMainDirection : lines90DegreeToMainDirection;
  int n = static_cast<int>(lineObservations.size());
  const std::vector<AbstractLine>& fieldLines = allObservatiosInMainClass ? fieldLinesX : fieldLinesY;

  double positionPerpendicularToLineDirection = fieldLines[correspondences[0]].offset - lineObservations[0].offset;
  double minPositionInLineDirection = fieldLines[correspondences[0]].min - lineObservations[0].min - parameters.absoluteAllowedDistanceErrorForLineClustering;
  double maxPositionInLineDirection = fieldLines[correspondences[0]].max - lineObservations[0].max + parameters.absoluteAllowedDistanceErrorForLineClustering;

  for (int i = 0; i < n; i++)
  {
    AbstractLine modelInRelativeCoords = fieldLines[correspondences[i]];
    // make is relative
    modelInRelativeCoords.offset -= positionPerpendicularToLineDirection;

    if (!doesObservationFitModelForInterval(lineObservations[i], modelInRelativeCoords, minPositionInLineDirection, maxPositionInLineDirection))
    {
      // does not fit. -> Conflict found.
      conflictingPosition = i;
      return false;
    }
  }
  // passed all tests! Yeah!  :-D

  // build the interval poses
  if (allObservatiosInMainClass)
  {
    poseIntervalHypothesisStart.translation.y() = static_cast<float>(positionPerpendicularToLineDirection);
    poseIntervalHypothesisEnd.translation.y() = static_cast<float>(positionPerpendicularToLineDirection);
    poseIntervalHypothesisStart.translation.x() = static_cast<float>(minPositionInLineDirection);
    poseIntervalHypothesisEnd.translation.x() = static_cast<float>(maxPositionInLineDirection);
  }
  else
  {
    poseIntervalHypothesisStart.translation.x() = static_cast<float>(-positionPerpendicularToLineDirection);
    poseIntervalHypothesisEnd.translation.x() = static_cast<float>(-positionPerpendicularToLineDirection);
    poseIntervalHypothesisStart.translation.y() = static_cast<float>(minPositionInLineDirection);
    poseIntervalHypothesisEnd.translation.y() = static_cast<float>(maxPositionInLineDirection);
  }

  conflictingPosition = totalNumberOfObservedLines - 1;
  return true;
}

bool LineMatcher::doesObservationFitModelForInterval(const AbstractLine & observationInRelativeCoords, const AbstractLine & modelInRelativeCoords, double& minPositionInLineDirection, double& maxPositionInLineDirection)
{
  // check the distances of the line
  if (std::abs(observationInRelativeCoords.offset / modelInRelativeCoords.offset - 1) < parameters.relativeAllowedDistanceErrorForLineClustering
    || std::abs(observationInRelativeCoords.offset - modelInRelativeCoords.offset) < parameters.absoluteAllowedDistanceErrorForLineClustering)
  {
    // in contrast to absolute pose hypotheses, we can only use the new constraints
    // to decrease the interval and check if it is still "there" afterwards.

    // Those are the new constraints:
    double minPosFromNewLineCorrespondence = modelInRelativeCoords.min - observationInRelativeCoords.min - parameters.absoluteAllowedDistanceErrorForLineClustering;
    double maxPosFromNewLineCorrespondence = modelInRelativeCoords.max - observationInRelativeCoords.max + parameters.absoluteAllowedDistanceErrorForLineClustering;

    // Adjust the old ones:
    minPositionInLineDirection = std::max(minPositionInLineDirection, minPosFromNewLineCorrespondence);
    maxPositionInLineDirection = std::min(maxPositionInLineDirection, maxPosFromNewLineCorrespondence);

    if (maxPositionInLineDirection - minPositionInLineDirection > 0)
    {
      return true;
    }
  }
  return false;
}

MAKE_MODULE(LineMatcher, modeling)
