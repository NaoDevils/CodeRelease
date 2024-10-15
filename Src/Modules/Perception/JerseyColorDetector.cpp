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
    DECLARE_DEBUG_DRAWING("module:JerseyColorDetector:distributionEstimation", "drawingOnImage");
    setRobotColorFromGC();
    theRobotsPerceptTeam.robots.clear();
    for (const auto& estimate : theRobotsPerceptClassified.robots)
    {
      RobotEstimate e(estimate);
      if (useKMeans)
        runKMeans(e);
      else
        updateRobotColor(e);
      theRobotsPerceptTeam.robots.push_back(e);
    }
  }
}

void JerseyColorDetector::printDistributionDebug(const RobotEstimate& re, ColorDistribution& distribution, ColorDistribution& expectedDistribution)
{
  if constexpr (!Build::targetSimulator())
    return;

  COMPLEX_DRAWING("module:JerseyColorDetector:distributionEstimation")
  {
    bool isHueWrap = distribution.minVal[0] > distribution.maxVal[0];
    // distribution
    short minH = static_cast<short>(!isHueWrap ? distribution.minVal[0] : distribution.maxVal[0]);
    short maxH = static_cast<short>(!isHueWrap ? distribution.maxVal[0] : distribution.minVal[0]);
    short midH = static_cast<short>(((minH + maxH + isHueWrap * 360) / 2) % 360);
    float midS = (distribution.minVal[1] + distribution.maxVal[1]) / 2;
    float midL = (distribution.minVal[2] + distribution.maxVal[2]) / 2;

    int x = re.imageUpperLeft.x(), y = re.imageLowerRight.y() - 50;

    ColorRGBA textColor = re.robotType == RobotEstimate::teammateRobot ? ColorRGBA::green : ColorRGBA::red;
    DRAWTEXT("module:JerseyColorDetector:distributionEstimation", x - 70, y, 6, textColor, "estimated :");

    unsigned char minR, minG, minB;
    ColorModelConversions::fromHSLToRGB(minH, midS, midL, minR, minG, minB);
    ColorRGBA minColor(minR, minG, minB);
    LINE("module:JerseyColorDetector:distributionEstimation", x - 30, y, x - 30, y, 10, Drawings::solidPen, minColor);

    unsigned char midR, midG, midB;
    ColorModelConversions::fromHSLToRGB(midH, midS, midL, midR, midG, midB);
    ColorRGBA midColor(midR, midG, midB);
    LINE("module:JerseyColorDetector:distributionEstimation", x - 20, y, x - 20, y, 10, Drawings::solidPen, midColor);

    unsigned char maxR, maxG, maxB;
    ColorModelConversions::fromHSLToRGB(maxH, midS, midL, maxR, maxG, maxB);
    ColorRGBA maxColor(maxR, maxG, maxB);
    LINE("module:JerseyColorDetector:distributionEstimation", x - 10, y, x - 10, y, 10, Drawings::solidPen, maxColor);


    // expected distribution
    isHueWrap = expectedDistribution.minVal[0] > expectedDistribution.maxVal[0];
    minH = static_cast<short>(!isHueWrap ? expectedDistribution.minVal[0] : expectedDistribution.maxVal[0]);
    maxH = static_cast<short>(!isHueWrap ? expectedDistribution.maxVal[0] : expectedDistribution.minVal[0]);
    midH = static_cast<short>(((minH + maxH + isHueWrap * 360) / 2) % 360);

    midS = (expectedDistribution.minVal[1] + expectedDistribution.maxVal[1]) / 2;
    midL = (expectedDistribution.minVal[2] + expectedDistribution.maxVal[2]) / 2;

    y = re.imageLowerRight.y() - 60;

    DRAWTEXT("module:JerseyColorDetector:distributionEstimation", x - 70, y, 6, textColor, "expected :");

    ColorModelConversions::fromHSLToRGB(minH, midS, midL, minR, minG, minB);
    minColor = ColorRGBA(minR, minG, minB);
    LINE("module:JerseyColorDetector:distributionEstimation", x - 30, y, x - 30, y, 10, Drawings::solidPen, minColor);

    ColorModelConversions::fromHSLToRGB(midH, midS, midL, midR, midG, midB);
    midColor = ColorRGBA(midR, midG, midB);
    LINE("module:JerseyColorDetector:distributionEstimation", x - 20, y, x - 20, y, 10, Drawings::solidPen, midColor);

    ColorModelConversions::fromHSLToRGB(maxH, midS, midL, maxR, maxG, maxB);
    maxColor = ColorRGBA(maxR, maxG, maxB);
    LINE("module:JerseyColorDetector:distributionEstimation", x - 10, y, x - 10, y, 10, Drawings::solidPen, maxColor);
  }
}

void JerseyColorDetector::runKMeans(RobotEstimate& re)
{
  Stopwatch s("JerseyColorDetector-runKMeans-full");
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

  std::vector<ABC> data;
  std::vector<Vector2f> positions;

  // fill data
  for (int i = 0; i < gridSampleSizeVar; i++)
  {
    checkPoint.x() += xInterval;
    checkPoint.y() = checkPointY;
    int colMax = gridSampleSizeVar;
    //int colMax = gridSampleSizeVar - abs(gridSampleSizeVar / 2 - i);
    for (int j = 0; j < colMax; j++)
    {
      checkPoint += Vector2f(0, yInterval);
      if (theImageUpper.isOutOfImage(checkPoint.x(), checkPoint.y(), 4))
      {
        continue;
      }
      Image::Pixel p = theImageUpper[static_cast<int>(checkPoint.y())][static_cast<int>(checkPoint.x())];
      // if we dont look at the field, try to estimate the team for this point
      if (isFieldColor(p))
      {
        continue;
      }
      unsigned char r, g, b;
      ColorModelConversions::fromYCbCrToRGB(p.y, p.cb, p.cr, r, g, b);
      data.emplace_back(ABC{static_cast<int>(r), static_cast<int>(g), static_cast<int>(b)});
      positions.emplace_back(checkPoint);
    }
  }

  // execute kmeans and count fitting clusters
  if (data.size() == 0)
    return;
  KMeans kmeans(data, maxKMeansIter, k);
  {
    Stopwatch s1("JerseyColorDetector-runKMeans-clustering-only");
    kmeans.run();
  }

  std::vector<ABC> centroids = kmeans.getCentroids();
  std::vector<int> labels = kmeans.getLabels();

  for (size_t i = 0; i < positions.size(); i++)
  {
    ABC assignedCentroid = centroids.at(labels.at(i));
    ColorRGBA debugColor(static_cast<unsigned char>(assignedCentroid.a), static_cast<unsigned char>(assignedCentroid.b), static_cast<unsigned char>(assignedCentroid.c));
    printPixelAssignmentDebug(positions.at(i), debugPixelSize, debugColor);
  }

  int counts[4] = {0};
  std::array<std::vector<int>, 4> fittingClusters;
  for (size_t i = 0; i < kmeans.getCentroids().size(); i++)
  {
    if (kmeans.getCounts().at(i) / float(data.size()) < minAssignedPixelsPercentage)
      continue;
    ABC col = kmeans.getCentroids().at(i);
    short int pH;
    float pS, pL;
    ColorModelConversions::fromRGBToHSL(static_cast<unsigned char>(col.a), static_cast<unsigned char>(col.b), static_cast<unsigned char>(col.c), pH, pS, pL);
    for (size_t j = 0; j < 4; j++)
    {
      if (fitsInDistribution(pH, pS, pL, colorDistributions.at(jerseyColors[j].color()))
          || (j >= 2 && acceptBlackOpponent && fitsInDistribution(pH, pS, pL, colorDistributions.at(TeamColor::Black))))
      {
        counts[j] += kmeans.getCounts().at(i);
        fittingClusters[j].emplace_back(static_cast<int>(i));
      }
    }
  }

  // leave weightedCount for debugging purposes
  float weightedCount[8] = {0}, ratios[8] = {0};
  int totalGridSize = static_cast<int>(data.size());
  for (int i = 0; i < 8; i++)
  {
    weightedCount[i] = counts[i % 4] * 1.0f; //weights[i];
    ratios[i] = weightedCount[i] / totalGridSize;
  }

  float ratioOwnColorTotal = ratios[OWN] + ratios[OWN_KEEPER], ratioOppColorTotal = ratios[OPP] + ratios[OPP_KEEPER];
  TeamColorIndex robotType = OWN;
  re.robotType = RobotEstimate::unknownRobot;
  // if enough pixels are assigned to atleast one team, consider assignment
  if (std::max(ratioOwnColorTotal, ratioOppColorTotal) > minAssignedPixelsPercentage)
  {
    // if atleast 2/3 belong to one team, return the according team
    // if then atleast 2/3 belong to the keeper, set keeper true
    if (ratioOwnColorTotal >= 2 * std::max(ratios[OPP], ratios[OPP_KEEPER]))
    {
      re.robotType = RobotEstimate::teammateRobot;
      re.teamAssignmentConfidence = ratioOwnColorTotal;
      robotType = OWN;
      if (ratios[OWN_KEEPER + 4] >= 2 * ratios[OWN + 4])
      {
        re.keeper = true;
        robotType = OWN_KEEPER;
      }
    }
    else if (ratioOppColorTotal >= 2 * std::max(ratios[OWN], ratios[OWN_KEEPER]))
    {
      re.robotType = RobotEstimate::opponentRobot;
      re.teamAssignmentConfidence = ratioOppColorTotal;
      robotType = OPP;
      if (ratios[OPP_KEEPER + 4] >= 2 * ratios[OPP + 4])
      {
        re.keeper = true;
        robotType = OPP_KEEPER;
      }
    }
  }

  if (re.robotType != RobotEstimate::unknownRobot && useDistributionEstimation)
  {
    Stopwatch s2("JerseyColorDetector-runKMeans-distributionEstimation");
    std::set<int> assignedClusters(fittingClusters[robotType].begin(), fittingClusters[robotType].end());
    ColorDistribution& expectedDistribution = preconfiguredColorDistributions.at(jerseyColors[robotType].color());
    std::optional<ColorDistribution> distributionOpt = estimateTeamColorDistribution(re, expectedDistribution, data, kmeans, assignedClusters);
    if (!distributionOpt.has_value())
      return;

    ColorDistribution& curCol = colorDistributions.at(jerseyColors[robotType].color());
    ColorDistribution& estimatedDistribution = distributionOpt.value();
    // if we are not in a hue wrap situation, interpolation is easy
    if (expectedDistribution.minVal[0] < expectedDistribution.maxVal[0])
    {
      curCol.minVal[0] = distribuitonInterpolationAlpha * estimatedDistribution.minVal[0] + (1 - distribuitonInterpolationAlpha) * curCol.minVal[0];
      curCol.maxVal[0] = distribuitonInterpolationAlpha * estimatedDistribution.maxVal[0] + (1 - distribuitonInterpolationAlpha) * curCol.maxVal[0];
    }
    else // else, it isnt
    {
      // basically, we can assume that cur hue and estimated hue are always between the expected (preconfigured) hue values.
      // It is therefore sufficient to check if both are on one side of the hue wrap, e.g. cur and estimated are above expected min and below 360. Then, we can simply interpolate.
      if ((expectedDistribution.minVal[0] <= std::min(curCol.minVal[0], estimatedDistribution.minVal[0]))
          || (std::max(curCol.minVal[0], estimatedDistribution.minVal[0]) <= expectedDistribution.maxVal[0]))
        curCol.minVal[0] = distribuitonInterpolationAlpha * estimatedDistribution.minVal[0] + (1 - distribuitonInterpolationAlpha) * curCol.minVal[0];
      else
      // if one is between expected min and 360, the other between 0 and expected max, we add 360 to the min, interpolate, subtract 360 again
      {
        bool estimatedIsSmaller = estimatedDistribution.minVal[0] <= curCol.minVal[0];
        curCol.minVal[0] = distribuitonInterpolationAlpha * (estimatedDistribution.minVal[0] + estimatedIsSmaller * 360)
            + (1 - distribuitonInterpolationAlpha) * (curCol.minVal[0] + !estimatedIsSmaller * 360) - 360;
      }

      // same as above, but for maxVal
      if ((expectedDistribution.minVal[0] <= std::min(curCol.maxVal[0], estimatedDistribution.maxVal[0]))
          || (std::max(curCol.maxVal[0], estimatedDistribution.maxVal[0]) <= expectedDistribution.maxVal[0]))
        curCol.maxVal[0] = distribuitonInterpolationAlpha * estimatedDistribution.maxVal[0] + (1 - distribuitonInterpolationAlpha) * curCol.maxVal[0];
      else
      {
        bool estimatedIsSmaller = estimatedDistribution.maxVal[0] <= curCol.maxVal[0];
        curCol.maxVal[0] = distribuitonInterpolationAlpha * (estimatedDistribution.maxVal[0] + estimatedIsSmaller * 360)
            + (1 - distribuitonInterpolationAlpha) * (curCol.maxVal[0] + !estimatedIsSmaller * 360) - 360;
      }
    }
    for (int i = 1; i < 3; i++)
    {
      curCol.minVal[i] = distribuitonInterpolationAlpha * estimatedDistribution.minVal[i] + (1 - distribuitonInterpolationAlpha) * curCol.minVal[i];
      curCol.maxVal[i] = distribuitonInterpolationAlpha * estimatedDistribution.maxVal[i] + (1 - distribuitonInterpolationAlpha) * curCol.maxVal[i];
    }


    printDistributionDebug(re, estimatedDistribution, expectedDistribution);
  }

  printTeamAssignmentDebug(re, ratios, ul.x(), ul.y(), lr.x(), lr.y());
}

std::optional<ColorDistribution> JerseyColorDetector::estimateTeamColorDistribution(
    RobotEstimate& re, ColorDistribution& expectedDistribution, std::vector<ABC>& data, KMeans& kmeans, std::set<int>& assignedClusters)
{
  std::vector<ABC> filteredData;
  std::copy_if(data.begin(),
      data.end(),
      std::back_inserter(filteredData),
      [&](const ABC& item)
      {
        size_t index = &item - &data[0];
        return assignedClusters.find(kmeans.getLabels()[index]) != assignedClusters.end();
      });

  std::vector<short> hVals(0);
  std::vector<float> sVals(0);
  std::vector<float> lVals(0);

  for (size_t i = 0; i < filteredData.size(); i++)
  {
    ABC item = filteredData.at(i);
    short h;
    float s, l;
    ColorModelConversions::fromRGBToHSL(static_cast<unsigned char>(item.a), static_cast<unsigned char>(item.b), static_cast<unsigned char>(item.c), h, s, l);
    if (!fitsInDistribution(h, s, l, expectedDistribution))
      continue;
    hVals.emplace_back(h), sVals.emplace_back(s), lVals.emplace_back(l);
  }

  if (hVals.empty() || hVals.size() < static_cast<size_t>(data.size() * minAssignedPixelsPercentage))
    return std::nullopt;

  std::sort(hVals.begin(), hVals.end()), std::sort(sVals.begin(), sVals.end()), std::sort(lVals.begin(), lVals.end());
  int minIndex = static_cast<int>(std::round(0.05 * hVals.size()));
  int maxIndex = static_cast<int>(std::round(0.95 * hVals.size())) - 1;

  // red wraps around the hue circle, so we need to handle the hVals a bit complicated if we are looking at an expectedDistribution with this property
  if (expectedDistribution.minVal[0] > expectedDistribution.maxVal[0])
  {
    double maxGap = 0;
    int maxGapIndex = 0;
    for (size_t i = 0; i < hVals.size() - 1; i++)
    {
      double gap = hVals[i + 1] - hVals[i];
      if (gap > maxGap)
      {
        maxGap = gap;
        maxGapIndex = static_cast<int>(i);
      }
    }
    if (maxGap > 36 && maxGapIndex > 0 && maxGapIndex < static_cast<int>(hVals.size() - 1))
      std::rotate(hVals.begin(), hVals.begin() + maxGapIndex + 1, hVals.end());
  }

  float minH = static_cast<float>(hVals[minIndex]), maxH = static_cast<float>(hVals[maxIndex]);
  float minS = sVals[minIndex], maxS = sVals[maxIndex];
  float minL = lVals[minIndex], maxL = lVals[maxIndex];

  ColorDistribution distribution;
  distribution.weight = 1;
  distribution.minVal[0] = minH, distribution.minVal[1] = minS, distribution.minVal[2] = minL;
  distribution.maxVal[0] = maxH, distribution.maxVal[1] = maxS, distribution.maxVal[2] = maxL;

  return distribution;
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
  //short pH;
  //float pS, pL;
  //ColorModelConversions::fromYCbCrToHSL(p.y, p.cb, p.cr, pH, pS, pL);
  //// if the color does not look green anyways, return
  //if (hueDistance(pH, hue(TeamColor::Darkgreen)) / deviation(TeamColor::Darkgreen) > 1)
  //  return false;
  //short fH;
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
        weights[i] = colorDistributions.at(jerseyColors[i].color()).weight;
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

  short pH;
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
    ColorDistribution cd = colorDistributions.at(c.color());
    const short h = hue(c);
    //const short s = (cd.minVal[1] + cd.maxVal[1]) / 2;
    const float s = cd.maxVal[1];
    //const float l = (cd.minVal[2] + cd.maxVal[2]) / 2;
    const float l = cd.maxVal[2];
    unsigned char r, g, b;
    ColorModelConversions::fromHSLToRGB(h, s, l, r, g, b);
    return ColorRGBA(r, g, b);
  }
}

short JerseyColorDetector::hue(TeamColor c)
{
  float minVal = colorDistributions.at(c.color()).minVal[0];
  float maxVal = colorDistributions.at(c.color()).maxVal[0];
  if (minVal < maxVal)
    return static_cast<short>(std::round((minVal + maxVal) / 2));

  short avg = static_cast<short>(std::round((minVal + maxVal + 360) / 2)) % 360;
  return avg;
}

MAKE_MODULE(JerseyColorDetector, perception);
