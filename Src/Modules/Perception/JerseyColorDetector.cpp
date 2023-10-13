#include "JerseyColorDetector.h"
#include "Tools/ColorModelConversions.h"
#include <iomanip>
#include "Tools/Build.h"

void JerseyColorDetector::printTeamAssignmentDebug(const RobotEstimate& re, float* ratios, int upperLeftX, int upperLeftY, int lowerRightX, int lowerRightY)
{
  COMPLEX_DRAWING("module:JerseyColorDetector:shirtScanUpper")
  {
    if constexpr (!Build::targetSimulator())
      return;

    DRAWTEXT("module:JerseyColorDetector:shirtScanUpper", lowerRightX + 20, lowerRightY - 65, 6, ColorRGBA::green, "tcProb");
    for (int i = 0; i < 4; i++)
    {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(1) << ratios[i] * 100.f << "%";
      LINE("module:JerseyColorDetector:shirtScanUpper", lowerRightX + 10, lowerRightY - 55 + i * 20, lowerRightX + 10, lowerRightY - 55 + i * 20, 10, Drawings::solidPen, asRGB(jerseyColors[i]));
      DRAWTEXT("module:JerseyColorDetector:shirtScanUpper", lowerRightX + 20, lowerRightY - 50 + i * 20, 6, i >= 2 ? ColorRGBA::red : ColorRGBA::green, ss.str());
    }

    ColorRGBA color = re.robotType == RobotEstimate::teammateRobot ? ColorRGBA::green : (re.robotType == RobotEstimate::opponentRobot ? ColorRGBA::red : ColorRGBA::black);
    if (!re.fromUpperImage)
      RECTANGLE("module:JerseyColorDetector:shirtScanUpper", upperLeftX, upperLeftY, lowerRightX, lowerRightY, 10, Drawings::solidPen, color);
  }
}

void JerseyColorDetector::printPixelAssignmentDebug(const Vector2f& checkPoint, int debugPixelSize, ColorRGBA& debugColor)
{
  COMPLEX_DRAWING("module:JerseyColorDetector:shirtScanUpper")
  {
    if constexpr (!Build::targetSimulator())
      return;
    LINE("module:JerseyColorDetector:shirtScanUpper", checkPoint.x(), checkPoint.y(), checkPoint.x(), checkPoint.y(), debugPixelSize, Drawings::solidPen, debugColor);
  }
}

void JerseyColorDetector::printAutoWeightDebug(const RobotEstimate& re, float* weights)
{
  if constexpr (!Build::targetSimulator())
    return;

  DRAWTEXT("module:JerseyColorDetector:shirtScanUpper", re.imageLowerRight.x() + 80, re.imageLowerRight.y() - 65, 6, ColorRGBA::green, "autoWeight");

  for (int i = 0; i < 4; i++)
  {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << weights[i];
    DRAWTEXT("module:JerseyColorDetector:shirtScanUpper", re.imageLowerRight.x() + 80, re.imageLowerRight.y() - 50 + i * 20, 6, i >= 2 ? ColorRGBA::red : ColorRGBA::green, ss.str());
  }

  for (int i = 0; i < 4; i++)
  {
    std::stringstream ss;
    ss << "(" << std::fixed << std::setprecision(1) << weights[i + 4] << ")";
    DRAWTEXT("module:JerseyColorDetector:shirtScanUpper", re.imageLowerRight.x() + 100, re.imageLowerRight.y() - 50 + i * 20, 6, i >= 2 ? ColorRGBA::red : ColorRGBA::green, ss.str());
  }
}

void JerseyColorDetector::printBackgroundProbabilitiesDebug(const RobotEstimate& re, const float* backgroundProbs)
{
  if constexpr (!Build::targetSimulator())
    return;

  DRAWTEXT("module:JerseyColorDetector:shirtScanUpper", re.imageLowerRight.x() + 50, re.imageLowerRight.y() - 65, 6, ColorRGBA::green, "bgProb");

  for (int i = 0; i < 4; i++)
  {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << backgroundProbs[i] * 100.f << "%";
    DRAWTEXT("module:JerseyColorDetector:shirtScanUpper", re.imageLowerRight.x() + 50, re.imageLowerRight.y() - 50 + i * 20, 6, i >= 2 ? ColorRGBA::red : ColorRGBA::green, ss.str());
  }
}

void JerseyColorDetector::update(RobotsPerceptTeam& theRobotsPerceptTeam)
{
  STOPWATCH("JerseyColorDetector-updateRobotColor-update")
  {
    DECLARE_DEBUG_DRAWING("module:JerseyColorDetector:shirtScanUpper", "drawingOnImage");
    setRobotColorFromGC();
    theRobotsPerceptTeam.robots.clear();
    for (const auto& estimate : theRobotsPerceptClassified.robots)
    {
      RobotEstimate e(estimate);
      updateRobotColor(e);
      theRobotsPerceptTeam.robots.push_back(e);
    }
  }
}

void JerseyColorDetector::setRobotColorFromGC()
{
  if (theOwnTeamInfo.teamNumber > 0)
  {
    jerseyColors[OWN] = TeamColor(theOwnTeamInfo.fieldPlayerColour);
    //jerseyColors[OWN_KEEPER] = TeamColor(theOwnTeamInfo.goalkeeperColour);
    // if own is red, use optimized ownMagenta
    jerseyColors[OWN_KEEPER] = TeamColor(theOwnTeamInfo.goalkeeperColour).color() == TeamColor::Color::Red ? TeamColor(TeamColor::OwnMagenta) : TeamColor(theOwnTeamInfo.goalkeeperColour);
  }
  if (theOpponentTeamInfo.teamNumber > 0)
  {
    jerseyColors[OPP] = TeamColor(theOpponentTeamInfo.fieldPlayerColour);
    jerseyColors[OPP_KEEPER] = TeamColor(theOpponentTeamInfo.goalkeeperColour);
  }
}

// return the distance in degrees between h1 and h2 or 1000, if h1 or h2 are not between 0 and 360
float JerseyColorDetector::hueDistance(float h1, float h2)
{
  if (h1 < 0 || h2 < 0)
  {
    return 1000.f;
  }
  float diff = std::abs(h1 - h2);
  if (diff <= 180.f)
  {
    return diff;
  }
  else if (diff <= 360.f)
  {
    return 360.f - diff;
  }
  else
  {
    return 1000.f;
  }
}

// randomized startpoint for the grid
Vector2f JerseyColorDetector::getInitialCheckpoint(int upperLeftX, int upperLeftY, int lowerRightX, int lowerRightY, int& gridSampleSizeVar, float& xInterval, float& yInterval)
{
  int width = lowerRightX - upperLeftX;
  lowerRightX -= static_cast<int>(round(gridSideMargin * width));
  upperLeftX += static_cast<int>(round(gridSideMargin * width));
  int randRadius = static_cast<int>(round(randRadiusPercentage * (lowerRightX - upperLeftX)));

  // bbox
  int xGridSize = lowerRightX - upperLeftX - 2 * randRadius;
  int yOffset = static_cast<int>(round((lowerRightY - upperLeftY) * gridTopMargin));
  int yGridSize = static_cast<int>((lowerRightY - upperLeftY - yOffset) / upperFractionOfEstimate - (2 * randRadius));

  // Do not use more steps than pixels if the estimate is very small
  gridSampleSizeVar = std::min(gridSampleSize, xGridSize);
  xInterval = xGridSize / static_cast<float>(gridSampleSizeVar - 1);
  yInterval = yGridSize / static_cast<float>(gridSampleSizeVar - 1);

  // use random offsets and intervals to calculate starting point
  float xRandOffset = randRadius == 0 ? 0.f : static_cast<float>(rand() % (2 * randRadius) - randRadius);
  float yRandOffset = randRadius == 0 ? 0.f : static_cast<float>(rand() % (2 * randRadius) - randRadius);
  float checkPointY = upperLeftY + randRadius + yRandOffset + yOffset;
  Vector2f checkPoint(upperLeftX + randRadius + xRandOffset - static_cast<int>(round(xInterval)), checkPointY);
  return checkPoint;
}

static int inHueRange(float x, float min, float max)
{
  if (min <= max)
    return min <= x && x <= max;
  return x <= max || min <= x;
}

bool JerseyColorDetector::fitsInDistribution(float h, float s, float l, ColorDistribution& colorDistribution)
{
  const bool fitsHue = inHueRange(h, colorDistribution.minVal[0], colorDistribution.maxVal[0]);
  const bool fitsSaturation = colorDistribution.minVal[1] <= s && s <= colorDistribution.maxVal[1];
  const bool fitsLightness = colorDistribution.minVal[2] <= l && l <= colorDistribution.maxVal[2];
  return fitsHue && fitsSaturation && fitsLightness;
}

bool JerseyColorDetector::isFieldColor(const Image::Pixel& p)
{
  if (theFieldColorsUpper.isPixelFieldColor(p.y, p.cb, p.cr))
    return true;
  //short int pH;
  //float pS, pL;
  //ColorModelConversions::fromYCbCrToHSL(p.y, p.cb, p.cr, pH, pS, pL);
  //// if the color does not look green anyways, return
  //if (hueDistance(pH, hue(TeamColor::Darkgreen)) / deviation(TeamColor::Darkgreen) > 1)
  //  return false;
  //short int fH;
  //float fS, fL;
  //ColorModelConversions::fromYCbCrToHSL(
  //    theFieldColorsUpper.fieldColorArray[0].fieldColorOptY, theFieldColorsUpper.fieldColorArray[0].fieldColorOptCb, theFieldColorsUpper.fieldColorArray[0].fieldColorOptCr, fH, fS, fL);
  //if ((hueDistance(pH, fH) / deviation(TeamColor::Darkgreen) > 1) || std::abs(pS - fS) > fieldColorSDiff || std::abs(pL - fL) > fieldColorLDiff)
  //  return false;
  //return true;
  return false;
}

void JerseyColorDetector::updateRobotColor(RobotEstimate& re)
{
  STOPWATCH("JerseyColorDetector-updateRobotColor-single")
  {
    // counter for how many pixels are estimated to belong to which team and how many were actually scanned
    int counts[4] = {0};
    int numScanned = 0;

    // Get the bounding box. If the estimate comes from the lower image, the robot is very near and its jersey should be seen in the upper image.
    Vector2i ul(re.imageUpperLeft.x(), re.imageUpperLeft.y());
    Vector2i lr(re.imageLowerRight.x(), re.imageLowerRight.y());
    if (!re.fromUpperImage)
    {
      re.getUpperImageCoordinates(ul, lr);
    }

    int gridSampleSizeVar;
    float xInterval, yInterval;
    Vector2f checkPoint = getInitialCheckpoint(ul.x(), ul.y(), lr.x(), lr.y(), gridSampleSizeVar, xInterval, yInterval);
    float checkPointY = checkPoint.y();
    int debugPixelSize = static_cast<int>(ceil(std::max(xInterval, yInterval)));

    // two loops that iterate over a grid in the robot estimate
    for (int i = 0; i < gridSampleSizeVar; i++)
    {
      checkPoint.x() += xInterval;
      checkPoint.y() = checkPointY;
      int colMax = gridSampleSizeVar;
      //int colMax = gridSampleSizeVar - abs(gridSampleSizeVar / 2 - i);
      for (int j = 0; j < colMax; j++)
      {
        assignPixel(checkPoint, counts, numScanned, debugPixelSize);
        checkPoint += Vector2f(0, yInterval);
      }
    }

    // there are 8 weights, 2 for each OWN, OWN_KEEPER, etc.
    // first 4 are to discern between own/opp, last 4 are to discern between normal and keeper
    float weights[8];
    for (int i = 0; i < 8; i++)
    {
      weights[i] = 1.f;
    }
    if (autoWeights)
    {
      // only calculate auto weights if it has to differentiate between own and opponent
      if (((counts[OWN] + counts[OWN_KEEPER]) > 0) + ((counts[OPP] + counts[OPP_KEEPER]) > 0) > 1)
      {
        calculateAutoWeights(weights, re);
      }
    }
    else
    {
      for (int i = 0; i < 4; i++)
      {
        weights[i] = colorDistributions.at(jerseyColors[i].color()).get().weight;
      }
    }

    // leave weightedCount for debugging purposes
    float weightedCount[8] = {0}, ratios[8] = {0};
    int totalGridSize = gridSampleSizeVar * gridSampleSizeVar;
    for (int i = 0; i < 8; i++)
    {
      weightedCount[i] = counts[i % 4] * weights[i];
      ratios[i] = weightedCount[i] / totalGridSize;
    }

    re.robotType = RobotEstimate::unknownRobot;

    float ratioOwnColorTotal = ratios[OWN] + ratios[OWN_KEEPER], ratioOppColorTotal = ratios[OPP] + ratios[OPP_KEEPER];
    // if enough pixels are assigned to atleast one team, consider assignment
    if (std::max(ratioOwnColorTotal, ratioOppColorTotal) > minAssignedPixelsPercentage)
    {
      // if atleast 2/3 belong to one team, return the according team
      // if then atleast 2/3 belong to the keeper, set keeper true
      if (ratioOwnColorTotal >= 2 * std::max(ratios[OPP], ratios[OPP_KEEPER]))
      {
        re.robotType = RobotEstimate::teammateRobot;
        re.teamAssignmentConfidence = ratioOwnColorTotal;
        if (ratios[OWN_KEEPER + 4] >= 2 * ratios[OWN + 4])
        {
          re.keeper = true;
        }
      }
      else if (ratioOppColorTotal >= 2 * std::max(ratios[OWN], ratios[OWN_KEEPER]))
      {
        re.robotType = RobotEstimate::opponentRobot;
        re.teamAssignmentConfidence = ratioOppColorTotal;
        if (ratios[OPP_KEEPER + 4] >= 2 * ratios[OPP + 4])
        {
          re.keeper = true;
        }
      }
    }
    printTeamAssignmentDebug(re, ratios, ul.x(), ul.y(), lr.x(), lr.y());
  }
}

bool JerseyColorDetector::pixelIntersectsRobot(const Vector2f& checkPoint)
{
  for (auto& estimate : theRobotsPerceptClassified.robots)
  {
    bool intersectsX = checkPoint.x() >= estimate.imageUpperLeft.x() && checkPoint.x() <= estimate.imageLowerRight.x();
    bool intersectsY = checkPoint.y() >= estimate.imageUpperLeft.y() && checkPoint.y() <= estimate.imageLowerRight.y();

    if (intersectsX && intersectsY)
    {
      return true;
    }
  }
  return false;
}

void JerseyColorDetector::assignPixel(const Vector2f& checkPoint, int* counts, int& numScanned, int debugPixelSize)
{
  // the debugColor shows information about how pixels are considered. If they are assigned to a team, considered field color etc.
  ColorRGBA debugColor = ColorRGBA::brown;
  if (theImageUpper.isOutOfImage(checkPoint.x(), checkPoint.y(), 4))
  {
    return;
  }

  numScanned += 1;

  Image::Pixel p = theImageUpper[static_cast<int>(checkPoint.y())][static_cast<int>(checkPoint.x())];
  // if we dont look at the field, try to estimate the team for this point
  if (isFieldColor(p))
  {
    debugColor = ColorRGBA::darkgreen;
    return;
  }

  short int pH;
  float pS, pL;
  ColorModelConversions::fromYCbCrToHSL(p.y, p.cb, p.cr, pH, pS, pL);

  for (int i = 0; i < 4; i++)
  {
    if (fitsInDistribution(pH, pS, pL, colorDistributions.at(jerseyColors[i].color())))
    {
      counts[i]++;
      debugColor = asRGB(jerseyColors[i]);
    }
    else if (i >= 2 && acceptBlackOpponent && fitsInDistribution(pH, pS, pL, colorDistributions.at(TeamColor::Black)))
    {
      counts[i]++;
      debugColor = ColorRGBA::black;
    }
  }

  printPixelAssignmentDebug(checkPoint, debugPixelSize, debugColor);
}

void JerseyColorDetector::calculateAutoWeights(float* weights, RobotEstimate& re)
{
  STOPWATCH("JerseyColorDetector-calculateAutoWeights")
  {
    float backgroundProbs[4] = {0};
    estimateBackgroundProbabilities(backgroundProbs, re);

    for (int i = 0; i < 8; i++)
    {
      weights[i] = 1.f;
    }
    float ownTotal = backgroundProbs[OWN] + backgroundProbs[OWN_KEEPER], oppTotal = backgroundProbs[OPP] + backgroundProbs[OPP_KEEPER];
    if (ownTotal != oppTotal)
    {
      float autoWeight = std::max(ownTotal, oppTotal) / std::min(ownTotal, oppTotal);
      autoWeight = std::min(maxAutoWeight, autoWeight);
      if (ownTotal < oppTotal)
      {
        weights[OWN] = autoWeight;
        weights[OWN_KEEPER] = autoWeight;
        autoWeight = std::max(backgroundProbs[OWN], backgroundProbs[OWN_KEEPER]) / std::min(backgroundProbs[OWN], backgroundProbs[OWN_KEEPER]);
        autoWeight = std::min(maxAutoWeight, autoWeight);
        if (backgroundProbs[OWN] < backgroundProbs[OWN_KEEPER])
          weights[OWN + 4] = autoWeight;
        else
          weights[OWN_KEEPER + 4] = autoWeight;
      }
      else
      {
        weights[OPP] = autoWeight;
        weights[OPP_KEEPER] = autoWeight;
        autoWeight = std::max(backgroundProbs[OPP], backgroundProbs[OPP_KEEPER]) / std::min(backgroundProbs[OPP], backgroundProbs[OPP_KEEPER]);
        autoWeight = std::min(maxAutoWeight, autoWeight);
        if (backgroundProbs[OPP] < backgroundProbs[OPP_KEEPER])
          weights[OPP + 4] = autoWeight;
        else
          weights[OPP_KEEPER + 4] = autoWeight;
      }
    }
    printAutoWeightDebug(re, weights);
  }
}


void JerseyColorDetector::estimateBackgroundProbabilities(float* backgroundProbs, RobotEstimate& re)
{
  for (int i = 0; i < 4; i++)
  {
    backgroundProbs[i] = 0.f;
  }
  // calculate the grids rows, columns, margins
  int heightHalf = (re.imageLowerRight.y() - re.imageUpperLeft.y()) / 2;

  int scanlineLen = (int)std::round(relativeBackgroundWidth * (re.imageLowerRight.x() - re.imageUpperLeft.x()));
  int gridSize = (int)std::round(std::sqrt(backgroundSampleSize / 2));

  int gridWidth = (int)std::round(scanlineLen / (float)gridSize);
  int gridHeight = (int)std::round(heightHalf / (float)gridSize);

  // starting points for scanning
  Vector2f rCp = Vector2f(re.imageLowerRight.x() + 2 * gridWidth, re.imageUpperLeft.y());
  Vector2f lCp = Vector2f(re.imageUpperLeft.x() - 2 * gridWidth, re.imageUpperLeft.y());

  // do a few scanlines
  int counts[4] = {0}, numScanned = 0;
  for (int i = 0; i < gridSize; i++)
  {
    rCp += Vector2f(0, gridHeight);
    lCp += Vector2f(0, gridHeight);
    countColorsOnHorizontalScanLine(counts, numScanned, rCp, false, scanlineLen, gridSize);
    countColorsOnHorizontalScanLine(counts, numScanned, lCp, true, scanlineLen, gridSize);
  }
  // a min amount of teamcolor pixels have to be found in the background
  int total = counts[OWN] + counts[OWN_KEEPER] + counts[OPP] + counts[OPP_KEEPER];
  bool minPixelsAssigned = total / (float)numScanned > minBackgroundAssignments;

  // the teamcolor pixel count has to be different enough, i.e. more than a standard deviation
  float minDiff = static_cast<float>(0.5 / std::sqrt(total));
  bool diffIsSignificant = std::max(counts[OWN] + counts[OWN_KEEPER], counts[OPP] + counts[OPP_KEEPER]) > total / 2.f + minDiff;

  if (minPixelsAssigned && diffIsSignificant)
  {
    for (int i = 0; i < 4; i++)
    {
      backgroundProbs[i] = counts[i] / (float)numScanned;
    }
  }
  printBackgroundProbabilitiesDebug(re, backgroundProbs);
}

void JerseyColorDetector::countColorsOnHorizontalScanLine(int* counts, int& scanned, Vector2f start, bool toLeft, int len, int amt)
{
  int jump = static_cast<int>(round(len / (float)amt + 1));
  for (int i = 0; i < amt; i++)
  {
    int offset = i * jump;
    Vector2f checkPoint;
    if (toLeft)
    {
      checkPoint = start - Vector2f((float)offset, 0);
    }
    else
    {
      checkPoint = start + Vector2f((float)offset, 0);
    }
    if (pixelIntersectsRobot(checkPoint))
    {
      continue;
    }
    assignPixel(checkPoint, counts, scanned, jump);
  }
}

ColorRGBA JerseyColorDetector::asRGB(TeamColor c)
{
  switch (c.color())
  {
  case TC::White:
    return ColorRGBA::white;
  case TC::Gray:
    return ColorRGBA::gray;
  case TC::Black:
    return ColorRGBA::black;
  default:
    ColorDistribution cd = colorDistributions.at(c.color()).get();
    const short int h = hue(c);
    //const short int s = (cd.minVal[1] + cd.maxVal[1]) / 2;
    const float s = cd.maxVal[1];
    //const float l = (cd.minVal[2] + cd.maxVal[2]) / 2;
    const float l = cd.maxVal[2];
    unsigned char r, g, b;
    ColorModelConversions::fromHSLToRGB(h, s, l, r, g, b);
    return ColorRGBA(r, g, b);
  }
}

short int JerseyColorDetector::hue(TeamColor c)
{
  float minVal = colorDistributions.at(c.color()).get().minVal[0];
  float maxVal = colorDistributions.at(c.color()).get().maxVal[0];
  if (minVal < maxVal)
    return static_cast<short int>(std::round((minVal + maxVal) / 2));
  float avg = (minVal - 360 + maxVal) / 2;
  if (avg < 0)
    avg += 360;
  return static_cast<short int>(std::round(avg));
}

MAKE_MODULE(JerseyColorDetector, perception);
