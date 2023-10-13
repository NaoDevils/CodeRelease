#pragma once

#include <optional>
#include <functional>

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"

class TeamColor
{
public:
  enum Color : uint8_t
  {
    Cyan = TEAM_BLUE,
    Red = TEAM_RED,
    Yellow = TEAM_YELLOW,
    Black = TEAM_BLACK,
    White = TEAM_WHITE,
    Darkgreen = TEAM_GREEN,
    Orange = TEAM_ORANGE,
    Purple = TEAM_PURPLE,
    Brown = TEAM_BROWN,
    Gray = TEAM_GRAY,
    OwnMagenta
  };
  TeamColor(uint8_t teamColor) : _color(Color(teamColor)) {}
  TeamColor(Color teamColor) : _color(teamColor) {}
  Color color() { return _color; };
  bool is(Color c) { return c == _color; };
  bool isGrayScale() { return _color == Color::Black || _color == Color::Gray || _color == Color::White; };

private:
  Color _color;
};

STREAMABLE(ColorDistribution,, // the three values are corresponding to h,s,l respectively
  (std::array<float, 3>) minVal,
  (std::array<float, 3>) maxVal,
  (float) weight
);

MODULE(JerseyColorDetector,
  REQUIRES(FieldColors),
  REQUIRES(FieldColorsUpper),
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(OwnTeamInfo),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(RobotsPerceptClassified),

  PROVIDES_CONCURRENT(RobotsPerceptTeam),

  LOADS_PARAMETERS(,
    
    (ColorDistribution) blackDistribution,
    (ColorDistribution) whiteDistribution,
    (ColorDistribution) grayDistribution,
    (ColorDistribution) cyanDistribution,
    (ColorDistribution) redDistribution,
    (ColorDistribution) yellowDistribution,
    (ColorDistribution) darkgreenDistribution,
    (ColorDistribution) orangeDistribution,
    (ColorDistribution) purpleDistribution,
    (ColorDistribution) brownDistribution,
    (ColorDistribution) ownMagentaDistribution,

    (bool) autoWeights,
    (float) relativeBackgroundWidth,
    (int) backgroundSampleSize,
    (float) minBackgroundAssignments,
    (float) maxAutoWeight,

    (float) upperFractionOfEstimate,
    (int) gridSampleSize,
    (float) randRadiusPercentage,
    (float) gridSideMargin,
    (float) gridTopMargin,

    (float) minColorConfidence,
    (float) minAssignedPixelsPercentage,
    (bool) acceptBlackOpponent
  )
);


class JerseyColorDetector : public JerseyColorDetectorBase
{
public:
  void update(RobotsPerceptTeam& theRobotsPerceptTeam);

private:
  using TC = TeamColor::Color;

  std::map<TC, std::reference_wrapper<ColorDistribution>> colorDistributions{{TC::Cyan, cyanDistribution},
      {TC::Red, redDistribution},
      {TC::Yellow, yellowDistribution},
      {TC::Black, blackDistribution},
      {TC::White, whiteDistribution},
      {TC::Darkgreen, darkgreenDistribution},
      {TC::Orange, orangeDistribution},
      {TC::Purple, purpleDistribution},
      {TC::Brown, brownDistribution},
      {TC::Gray, grayDistribution},
      {TC::OwnMagenta, ownMagentaDistribution}};

  enum TeamColorIndex : char
  {
    OWN,
    OWN_KEEPER,
    OPP,
    OPP_KEEPER
  };

  TeamColor jerseyColors[4] = {TeamColor(TEAM_YELLOW), TeamColor(TeamColor::OwnMagenta), TeamColor(TEAM_BLACK), TeamColor(TEAM_BLACK)};

  void setRobotColorFromGC();

  void updateRobotColor(RobotEstimate& re);
  void assignPixel(const Vector2f& checkPoint, int* counts, int& numScanned, int debugPixelSize);
  bool pixelIntersectsRobot(const Vector2f& checkPoint);
  Vector2f getInitialCheckpoint(int upperLeftX, int upperLeftY, int lowerRightX, int lowerRightY, int& gridSampleSize, float& xInterval, float& yInterval);
  void estimateBackgroundProbabilities(float* backgroundProbs, RobotEstimate& re);
  void countColorsOnHorizontalScanLine(int* counts, int& scanned, Vector2f start, bool toLeft, int len, int amt);
  void calculateAutoWeights(float* weights, RobotEstimate& re);

  bool isFieldColor(const Image::Pixel& p);
  bool fitsInDistribution(float h, float s, float l, ColorDistribution& colorDistribution);
  float hueDistance(float, float);

  ColorRGBA asRGB(TeamColor);
  short int hue(TeamColor);

  void printTeamAssignmentDebug(const RobotEstimate& re, float* ratios, int upperLeftX, int upperLeftY, int lowerRightX, int lowerRightY);
  void printPixelAssignmentDebug(const Vector2f& checkPoint, int debugPixelSize, ColorRGBA& debugColor);
  void printAutoWeightDebug(const RobotEstimate& re, float* weights);
  void printBackgroundProbabilitiesDebug(const RobotEstimate& re, const float* backgroundProbs);
};
