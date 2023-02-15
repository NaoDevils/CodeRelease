/**
 * @file FieldDimensions.cpp
 *
 * Some useful functions regarding field dimensions.
 *
 * @author Max Risler
 */

#include "FieldDimensions.h"
#include "Tools/Debugging/Modify.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Eigen.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <algorithm>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>

using namespace std;

/**
 * Helper struct that supports the use of symbolic values in float fields.
 */
struct InSymbolicMapFile : public InMapFile
{
private:
  std::unordered_map<std::string, float> values; /**< All symbolic values known. */
  const char* entry = nullptr; /**< The name of the current entry processed. */

public:
  InSymbolicMapFile(const std::string& name) : InMapFile(name) {}

  virtual void select(const char* name, int type, const char* (*enumToString)(int))
  {
    Streaming::trimName(name);
    InMapFile::select(name, type, enumToString);
    entry = name;
  }

protected:
  /**
   * When reading a float, read a string instead. Try to replace symbolic value.
   * Symbolic value can be preceeded by a minus sign (without whitespace in between).
   */
  virtual void inFloat(float& value)
  {
    std::string buf;
    inString(buf);
    float sign = 1.f;
    if (buf[0] == '-')
    {
      sign = -1.f;
      buf = buf.substr(1);
    }

    std::unordered_map<std::string, float>::const_iterator i = values.find(buf);
    if (i != values.end())
      value = i->second * sign;
    else if (!buf.empty() && (isdigit(buf[0]) || buf[0] == '.'))
      value = static_cast<float>(strtod(buf.c_str(), 0)) * sign;
    else
      OUTPUT_ERROR("fieldDimensions.cfg: Unknown symbol '" << buf << "'");

    if (entry)
      values[entry] = value;
  }
};

void FieldDimensions::load()
{
  InSymbolicMapFile stream("fieldDimensions.cfg");
  ASSERT(stream.exists());
  stream >> *this;
  boundary.add(Vector2f(xPosOpponentFieldBorder, yPosLeftFieldBorder));
  boundary.add(Vector2f(xPosOwnFieldBorder, yPosRightFieldBorder));
  lastUpdate = SystemCall::getCurrentSystemTime();
}

bool FieldDimensions::loadFromJsonFile(const std::string& path)
{
  try
  {
    const auto loadFromJSON = [&](const nlohmann::json& j)
    {
      // using j.at() for bounds checking
      const auto field = j.at("field");
      goalAreaPresent = field.contains("goalBoxAreaLength") && field.contains("goalBoxAreaWidth");

      const float length = field.at("length").get<float>() * 1000.f;
      const float width = field.at("width").get<float>() * 1000.f;
      const float penaltyCrossSize = field.at("penaltyCrossSize").get<float>() * 1000.f;
      const float penaltyAreaLength = field.at("penaltyAreaLength").get<float>() * 1000.f;
      const float penaltyAreaWidth = field.at("penaltyAreaWidth").get<float>() * 1000.f;
      const float goalBoxAreaLength = goalAreaPresent ? field.at("goalBoxAreaLength").get<float>() * 1000.f : penaltyAreaLength;
      const float goalBoxAreaWidth = goalAreaPresent ? field.at("goalBoxAreaWidth").get<float>() * 1000.f : penaltyAreaWidth;
      const float penaltyCrossDistance = field.at("penaltyCrossDistance").get<float>() * 1000.f;
      const float centerCircleDiameter = field.at("centerCircleDiameter").get<float>() * 1000.f;
      const float borderStripWidth = field.at("borderStripWidth").get<float>() * 1000.f;

      const auto goal = j.at("goal");
      const float postDiameter = goal.at("postDiameter").get<float>() * 1000.f;
      const float height = goal.at("height").get<float>() * 1000.f;
      const float innerWidth = goal.at("innerWidth").get<float>() * 1000.f;
      const float depth = goal.at("depth").get<float>() * 1000.f;

      // map values
      fieldLinesWidth = 50.f;
      constexpr float penaltyStrikerStartPositionXOffset = 1000.f;

      xPosOpponentGroundline = length / 2.f;
      xPosOpponentFieldBorder = xPosOpponentGroundline + borderStripWidth;
      xPosOpponentGoal = xPosOpponentGroundline + depth;
      xPosOpponentGoalArea = xPosOpponentGroundline - goalBoxAreaLength;
      xPosOpponentGoalPost = xPosOpponentGroundline - fieldLinesWidth / 2.f;
      xPosOpponentPenaltyArea = xPosOpponentGroundline - penaltyAreaLength;
      xPosOpponentPenaltyMark = xPosOpponentGroundline - penaltyCrossDistance;
      xPosPenaltyStrikerStartPosition = xPosOpponentPenaltyMark - penaltyStrikerStartPositionXOffset;
      xPosHalfWayLine = 0.f;
      xPosOwnPenaltyMark = -xPosOpponentPenaltyMark;
      xPosOwnPenaltyArea = -xPosOpponentPenaltyArea;
      xPosOwnGroundline = -xPosOpponentGroundline;
      xPosOwnGoalPost = -xPosOpponentGoalPost;
      xPosOwnGoalArea = -xPosOpponentGoalArea;
      xPosOwnGoal = -xPosOpponentGoal;
      xPosOwnFieldBorder = -xPosOpponentFieldBorder;
      xPosKickOffPoint = 0.f;

      yPosLeftSideline = width / 2.f;
      yPosLeftFieldBorder = yPosLeftSideline + borderStripWidth;
      yPosLeftPenaltyArea = penaltyAreaWidth / 2.f;
      yPosLeftGoal = innerWidth / 2.f + postDiameter / 2.f;
      yPosLeftGoalArea = goalBoxAreaWidth / 2.f;
      yPosCenterGoal = 0.f;
      yPosRightGoal = -yPosLeftGoal;
      yPosRightGoalArea = -yPosLeftGoalArea;
      yPosRightPenaltyArea = -yPosLeftPenaltyArea;
      yPosRightSideline = -yPosLeftSideline;
      yPosRightFieldBorder = -yPosLeftFieldBorder;
      yPosKickOffPoint = 0.f;

      centerCircleRadius = centerCircleDiameter / 2.f;
      goalPostRadius = postDiameter / 2.f;
      crossBarRadius = goalPostRadius;
      goalHeight = height;
      ballType = BallType::whiteBlack;
      ballRadius = 50.f;
      penaltyMarkSize = penaltyCrossSize;
    };

    std::ifstream ifs(path);
    if (ifs.fail())
      return false;
    const auto json = nlohmann::json::parse(ifs);

    loadFromJSON(json);
    fill();
    lastUpdate = SystemCall::getCurrentSystemTime();

    OUTPUT_TEXT("FieldDimensions: Loaded field_dimensions.json!");
    return true;
  }
  catch (const std::exception& e)
  {
    OUTPUT_WARNING("FieldDimensions: Malformed field_dimensions.json: " << e.what());
    return false;
  }
}

void FieldDimensions::fill()
{
  carpetBorder = {{{xPosOpponentFieldBorder, yPosRightFieldBorder}, {xPosOpponentFieldBorder, yPosLeftFieldBorder}},
      {{xPosOpponentFieldBorder, yPosLeftFieldBorder}, {xPosOwnFieldBorder, yPosLeftFieldBorder}},
      {{xPosOwnFieldBorder, yPosLeftFieldBorder}, {xPosOwnFieldBorder, yPosRightFieldBorder}},
      {{xPosOwnFieldBorder, yPosRightFieldBorder}, {xPosOpponentFieldBorder, yPosRightFieldBorder}}};

  goalFrameLines = {
      {{xPosOwnGoalPost, yPosLeftGoal}, {xPosOwnGoal, yPosLeftGoal}},
      {{xPosOwnGoal, yPosLeftGoal}, {xPosOwnGoal, yPosRightGoal}},
      {{xPosOwnGoalPost, yPosRightGoal}, {xPosOwnGoal, yPosRightGoal}},
      {{xPosOpponentGoalPost, yPosLeftGoal}, {xPosOpponentGoal, yPosLeftGoal}},
      {{xPosOpponentGoal, yPosLeftGoal}, {xPosOpponentGoal, yPosRightGoal}},
      {{xPosOpponentGoalPost, yPosRightGoal}, {xPosOpponentGoal, yPosRightGoal}},
  };

  fieldBorder = {
      {{xPosOpponentGroundline, yPosRightSideline}, {xPosOpponentGroundline, yPosLeftSideline}},
      {{xPosOpponentGroundline, yPosLeftSideline}, {xPosOwnGroundline, yPosLeftSideline}},
      {{xPosOwnGroundline, yPosLeftSideline}, {xPosOwnGroundline, yPosRightSideline}},
      {{xPosOwnGroundline, yPosRightSideline}, {xPosOpponentGroundline, yPosRightSideline}},
  };

  fieldLines = {
      // field border lines
      fieldBorder.lines[0],
      fieldBorder.lines[1],
      fieldBorder.lines[2],
      fieldBorder.lines[3],

      // center line
      {{xPosHalfWayLine, yPosLeftSideline}, {xPosHalfWayLine, yPosRightSideline}},

      // own penalty area
      {{xPosOwnGroundline, yPosLeftPenaltyArea}, {xPosOwnPenaltyArea, yPosLeftPenaltyArea}},
      {{xPosOwnPenaltyArea, yPosLeftPenaltyArea}, {xPosOwnPenaltyArea, yPosRightPenaltyArea}},
      {{xPosOwnPenaltyArea, yPosRightPenaltyArea}, {xPosOwnGroundline, yPosRightPenaltyArea}},

      // opp penalty area
      {{xPosOpponentGroundline, yPosLeftPenaltyArea}, {xPosOpponentPenaltyArea, yPosLeftPenaltyArea}},
      {{xPosOpponentPenaltyArea, yPosLeftPenaltyArea}, {xPosOpponentPenaltyArea, yPosRightPenaltyArea}},
      {{xPosOpponentPenaltyArea, yPosRightPenaltyArea}, {xPosOpponentGroundline, yPosRightPenaltyArea}},

      // penalty marks
      {{xPosOpponentPenaltyMark - penaltyMarkSize / 2.f, 0.f}, {xPosOpponentPenaltyMark + penaltyMarkSize / 2.f, 0.f}},
      {{xPosOpponentPenaltyMark, -penaltyMarkSize / 2.f}, {xPosOpponentPenaltyMark, penaltyMarkSize / 2.f}},
      {{xPosOwnPenaltyMark + penaltyMarkSize / 2.f, 0.f}, {xPosOwnPenaltyMark - penaltyMarkSize / 2.f, 0.f}},
      {{xPosOwnPenaltyMark, -penaltyMarkSize / 2.f}, {xPosOwnPenaltyMark, penaltyMarkSize / 2.f}},

      // center mark
      {{-fieldLinesWidth, 0.f}, {fieldLinesWidth, 0.f}},
  };

  if (goalAreaPresent)
  {
    fieldLines.lines.insert(fieldLines.lines.end(),
        {
            // own goal area
            {{xPosOwnGroundline, yPosLeftGoalArea}, {xPosOwnGoalArea, yPosLeftGoalArea}},
            {{xPosOwnGoalArea, yPosLeftGoalArea}, {xPosOwnGoalArea, yPosRightGoalArea}},
            {{xPosOwnGoalArea, yPosRightGoalArea}, {xPosOwnGroundline, yPosRightGoalArea}},

            // opp goal area
            {{xPosOpponentGroundline, yPosLeftGoalArea}, {xPosOpponentGoalArea, yPosLeftGoalArea}},
            {{xPosOpponentGoalArea, yPosLeftGoalArea}, {xPosOpponentGoalArea, yPosRightGoalArea}},
            {{xPosOpponentGoalArea, yPosRightGoalArea}, {xPosOpponentGroundline, yPosRightGoalArea}},
        });
  }

  fieldLines.pushCircle({0.f, 0.f}, centerCircleRadius, 16);

  fieldLinesWithGoalFrame.lines = {fieldLines.lines.begin(), fieldLines.lines.end()};
  fieldLinesWithGoalFrame.lines.insert(fieldLinesWithGoalFrame.lines.end(), goalFrameLines.lines.begin(), goalFrameLines.lines.end());
}

bool FieldDimensions::isBallInsideField(const Vector2f& ballPos) const
{
  float offset = fieldLinesWidth / 2 + ballRadius / 2;
  float x, y;

  if (ballPos.x() > 0)
    x = std::max(0.f, ballPos.x() - offset);
  else
    x = std::min(0.f, ballPos.x() + offset);

  if (ballPos.y() > 0)
    y = std::max(0.f, ballPos.y() - offset);
  else
    y = std::min(0.f, ballPos.y() + offset);

  return fieldBorder.isInside(Vector2f(x, y));
}

Pose2f FieldDimensions::randomPoseOnField() const
{
  Pose2f pose;
  do
    pose = Pose2f::random(boundary.x, boundary.y, Rangef(-pi, pi));
  while (!isInsideField(pose.translation));
  return pose;
}

Pose2f FieldDimensions::randomPoseOnCarpet() const
{
  Pose2f pose;
  do
    pose = Pose2f::random(boundary.x, boundary.y, Rangef(-pi, pi));
  while (!isInsideCarpet(pose.translation));
  return pose;
}

void FieldDimensions::draw() const
{
  drawLines();
  drawGoalFrame();
}

void FieldDimensions::drawGoalFrame() const
{
  DECLARE_DEBUG_DRAWING("goal frame", "drawingOnField");
  COMPLEX_DRAWING("goal frame")
  {
    for (const LinesTable::Line& l : goalFrameLines.lines)
    {
      ColorRGBA lineColor(192, 192, 192);
      LINE("goal frame", l.from.x(), l.from.y(), l.to.x(), l.to.y(), fieldLinesWidth * 0.7, Drawings::solidPen, lineColor);
    }
  }
}

void FieldDimensions::drawLines() const
{
  DECLARE_DEBUG_DRAWING("field lines", "drawingOnField");
  COMPLEX_DRAWING("field lines")
  {
    ASSERT(carpetBorder.lines.size() <= 4);
    Vector2f points[4];
    for (unsigned i = 0; i < carpetBorder.lines.size(); ++i)
      points[i] = carpetBorder.lines[i].from;
    POLYGON("field lines", static_cast<int>(carpetBorder.lines.size()), points, 0, Drawings::solidPen, ColorRGBA(0, 180, 0), Drawings::solidBrush, ColorRGBA(0, 140, 0));

    ColorRGBA lineColor(192, 192, 192);
    for (vector<LinesTable::Line>::const_iterator i = fieldLines.lines.begin(); i != fieldLines.lines.end(); ++i)
    {
      LINE("field lines", i->from.x(), i->from.y(), i->to.x(), i->to.y(), fieldLinesWidth, Drawings::solidPen, lineColor);
    }
  }
}

void FieldDimensions::drawPolygons(int ownColor) const
{
  DECLARE_DEBUG_DRAWING("field polygons", "drawingOnField");
  COMPLEX_DRAWING("field polygons")
  {
    static const ColorRGBA colors[] = {ColorRGBA(50, 120, 127), ColorRGBA(127, 80, 80), ColorRGBA(127, 120, 50), ColorRGBA(80, 80, 80)};
    const ColorRGBA& own = colors[ownColor];
    const ColorRGBA& opp = colors[1 ^ ownColor];

    Vector2f goal[4];
    goal[0] = Vector2f(xPosOwnGroundline - fieldLinesWidth * 0.5f, yPosLeftGoal);
    goal[1] = Vector2f(xPosOwnGroundline - fieldLinesWidth * 0.5f, yPosRightGoal);
    goal[2] = Vector2f(xPosOwnGoal, yPosRightGoal);
    goal[3] = Vector2f(xPosOwnGoal, yPosLeftGoal);
    POLYGON("field polygons", 4, goal, 0, Drawings::solidPen, own, Drawings::solidBrush, own);

    goal[0] = Vector2f(xPosOpponentGroundline + fieldLinesWidth * 0.5f, yPosLeftGoal);
    goal[1] = Vector2f(xPosOpponentGroundline + fieldLinesWidth * 0.5f, yPosRightGoal);
    goal[2] = Vector2f(xPosOpponentGoal, yPosRightGoal);
    goal[3] = Vector2f(xPosOpponentGoal, yPosLeftGoal);
    POLYGON("field polygons", 4, goal, 0, Drawings::solidPen, opp, Drawings::solidBrush, opp);

    CIRCLE("field polygons", xPosOpponentGoalPost, yPosLeftGoal, 50, 0, Drawings::solidPen, ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);
    CIRCLE("field polygons", xPosOpponentGoalPost, yPosRightGoal, 50, 0, Drawings::solidPen, ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);

    CIRCLE("field polygons", xPosOwnGoalPost, yPosLeftGoal, 50, 0, Drawings::solidPen, ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);
    CIRCLE("field polygons", xPosOwnGoalPost, yPosRightGoal, 50, 0, Drawings::solidPen, ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);
  }
}

FieldDimensions::LinesTable::Line::Line(Vector2f from, Vector2f to, bool isPartOfCircle)
{
  this->length = (to - from).norm();
  this->isPartOfCircle = isPartOfCircle;
  this->from = std::move(from);
  this->to = std::move(to);
}

void FieldDimensions::LinesTable::push(const Vector2f& from, const Vector2f& to, bool isPartOfCircle)
{
  LinesTable::Line line;
  line.from = from;
  line.to = to;
  line.length = (to - from).norm();
  line.isPartOfCircle = isPartOfCircle;
  lines.push_back(line);
}

bool FieldDimensions::LinesTable::getClosestIntersection(const Geometry::Line& l, Vector2f& outIntersection) const
{
  int wayne;
  return getClosestIntersection(l, wayne, outIntersection);
}

bool FieldDimensions::LinesTable::getClosestIntersection(const Geometry::Line& l, int& outLineIndex, Vector2f& outIntersection) const
{
  float currentMinimumDistance = std::numeric_limits<float>::max(); //square distance
  Vector2f closestPoint(-100000.f, -1000000.f);
  bool found = false;
  for (int i = 0; i < (int)lines.size(); ++i)
  {
    const Line& fieldLine = lines[i];
    Vector2f intersection;
    Geometry::Line line(fieldLine.from, (fieldLine.to - fieldLine.from).normalized());
    if (Geometry::getIntersectionOfLines(l, line, intersection))
    {
      //we already know that the intersection is on the line,
      //just need to know if it is inside the rectangle defined by the field line coordinates
      Rangef xRange(std::min(fieldLine.from.x(), fieldLine.to.x()), std::max(fieldLine.from.x(), fieldLine.to.x()));
      Rangef yRange(std::min(fieldLine.from.y(), fieldLine.to.y()), std::max(fieldLine.from.y(), fieldLine.to.y()));
      if (xRange.isInside(intersection.x()) && yRange.isInside(intersection.y()))
      {
        const float squareDist = (l.base - intersection).squaredNorm();
        if (squareDist < currentMinimumDistance)
        {
          found = true;
          outLineIndex = i;
          currentMinimumDistance = squareDist;
          closestPoint = intersection;
        }
      }
    }
  }
  if (found)
    outIntersection = closestPoint;
  return found;
}

Vector2f FieldDimensions::LinesTable::getClosestPoint(const Vector2f& point) const
{
  float currentMinimumDistance = std::numeric_limits<float>::max();
  Vector2f closestPoint(-1.f, -1.f);
  Vector2f tempClosestPoint(-1.f, -1.f);

  for (vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
  {
    Geometry::Line line(i->from, (i->to - i->from).normalized());
    //calculate orthogonal projection of point onto line (see http://de.wikipedia.org/wiki/Orthogonalprojektion)
    const float numerator = (point - line.base).dot(line.direction);
    const Vector2f projection = line.base + (line.direction * numerator);

    //check if projected point is on the line segment
    //We already know that it is on the line, therefore we just need to check
    //if it is inside the rectangle created by start and end point of the line
    const Vector2f& a = line.base; //a, b and c only exist to make the following if readable
    const Vector2f& b = i->to;
    const Vector2f& c = projection;

    float distance = -1;
    //not optimized expression
    //    if(c.x >= b.x && c.x <= a.x && c.y >= b.y && c.y <= a.y ||
    //       c.x >= a.x && c.x <= b.x && c.y >= a.y && c.y <= b.y ||
    //       c.x >= a.x && c.x <= b.x && c.y >= b.y && c.y <= a.y ||
    //       c.x >= b.x && c.x <= a.x && c.y >= a.y && c.y <= b.y)
    //optimized expression
    if (((c.y() >= b.y() && c.y() <= a.y()) || (c.y() >= a.y() && c.y() <= b.y())) && ((c.x() >= b.x() && c.x() <= a.x()) || (c.x() >= a.x() && c.x() <= b.x())))
    { //If projection is on the line segment just calculate the distance between the point
      //and it's projection.
      distance = (projection - point).norm();
      tempClosestPoint = projection;
    }
    else
    { //If the projection is not on the line segment it is either left or
      //right of the line segment. Therefore use distance to edge
      const float distBase = (line.base - point).norm();
      const float distEnd = (i->to - point).norm();
      if (distBase <= distEnd)
      {
        distance = distBase;
        tempClosestPoint = line.base;
      }
      else
      {
        distance = distEnd;
        tempClosestPoint = i->to;
      }
    }

    if (distance < currentMinimumDistance)
    {
      currentMinimumDistance = distance;
      closestPoint = tempClosestPoint;
    }
  }
  return closestPoint;
}

void FieldDimensions::LinesTable::pushCircle(const Vector2f& center, float radius, int numOfSegments)
{
  Vector2f p1, p2;
  for (float a = 0; a <= pi_4; a += pi2 / numOfSegments)
  {
    p1 = Vector2f(sin(a), cos(a)) * radius;
    if (a > 0)
    {
      push(center + p1, center + p2, true);
      push(center + Vector2f(p1.x(), -p1.y()), center + Vector2f(p2.x(), -p2.y()), true);
      push(center + Vector2f(-p1.x(), p1.y()), center + Vector2f(-p2.x(), p2.y()), true);
      push(center - p1, center - p2, true);
      push(center + Vector2f(p1.y(), p1.x()), center + Vector2f(p2.y(), p2.x()), true);
      push(center + Vector2f(p1.y(), -p1.x()), center + Vector2f(p2.y(), -p2.x()), true);
      push(center + Vector2f(-p1.y(), p1.x()), center + Vector2f(-p2.y(), p2.x()), true);
      push(center + Vector2f(-p1.y(), -p1.x()), center + Vector2f(-p2.y(), -p2.x()), true);
    }
    p2 = p1;
  }
}

bool FieldDimensions::LinesTable::isInside(const Vector2f& v) const
{
  //note:
  //This function assumes that the point (0,0) is inside and
  //that for any point inside the area the line to (0,0) belongs to the area too.

  Geometry::Line testLine(v, -v);
  for (vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
  {
    float factor;
    Geometry::Line border(i->from, i->to - i->from);
    if (Geometry::getIntersectionOfRaysFactor(border, testLine, factor))
      return false;
  }
  return true;
}

float FieldDimensions::LinesTable::clip(Vector2f& v) const
{
  if (isInside(v))
    return 0;
  else
  {
    const Vector2f old = v;
    Vector2f v2;
    float minDist = 100000;
    for (vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    {
      if (old == i->from)
        v2 = i->from;
      else
      {
        float a = (old - i->from).dot(i->to - i->from) / ((old - i->from).norm() * (i->to - i->from).norm());
        if (a <= 0)
          v2 = i->from;
        else if (a >= 1.f)
          v2 = i->to;
        else
          v2 = i->from + a * (i->to - i->from);
      }
      float dist = (old - v2).norm();
      if (minDist > dist)
      {
        minDist = dist;
        v = v2;
      }
    }
    return (v - old).norm();
  }
}

bool FieldDimensions::LinesTable::getClosestPoint(Vector2f& vMin, const Pose2f& p, int numberOfRotations, float minLength) const
{
  int trueNumberOfRotations = numberOfRotations;
  if (numberOfRotations == 2)
    numberOfRotations = 4;

  // target angle -> target index
  float r = p.rotation / pi2 * numberOfRotations + 0.5f;
  if (r < 0)
    r += numberOfRotations;
  int targetRot = int(r);
  ASSERT(targetRot >= 0 && targetRot < numberOfRotations);
  targetRot %= trueNumberOfRotations;
  Vector2f v2;
  float minDist = 100000;
  for (vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    if ((i->to - i->from).squaredNorm() >= sqr(minLength))
    {
      // angle -> index
      float r = ((i->to - i->from).angle() + pi_2) / pi2 * numberOfRotations + 0.5f;
      if (r < 0)
        r += numberOfRotations;
      else if (r >= numberOfRotations)
        r -= numberOfRotations;
      int rot = int(r);
      ASSERT(rot >= 0 && rot < numberOfRotations);
      rot %= trueNumberOfRotations;

      // index must be target index
      if (rot == targetRot)
      {
        if (p.translation == i->from)
          v2 = i->from;
        else
        {
          float a = (p.translation - i->from).dot(i->to - i->from) / ((p.translation - i->from).norm() * (i->to - i->from).norm());
          if (a <= 0)
            v2 = i->from;
          else if (a >= 1.f)
            v2 = i->to;
          else
            v2 = i->from + a * (i->to - i->from);
        }
        const Vector2f vDiff = v2 - p.translation;
        float dist = vDiff.norm();
        if (minDist > dist)
        {
          minDist = dist;
          vMin = v2;
        }
      }
    }
  return (minDist < 100000);
}

float FieldDimensions::LinesTable::getDistance(const Pose2f& pose) const
{
  float minDist = 100000;
  for (vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    if (i->from.y() < 0 && i->to.y() > 0)
    {
      const float dist = i->from.x() + (i->to.x() - i->from.x()) * -i->from.y() / (i->to.y() - i->from.y());
      if (dist >= 0 && dist < minDist)
        minDist = dist;
    }
  return minDist == 100000 ? -1 : minDist;
}

void FieldDimensions::serialize(In* in, Out* out)
{
  ASSERT(in); // handling center circle only works when reading

  std::vector<LinesTable::Line>& carpetBorder(this->carpetBorder.lines);
  std::vector<LinesTable::Line>& fieldBorder(this->fieldBorder.lines);
  std::vector<LinesTable::Line>& fieldLines(this->fieldLines.lines);
  std::vector<LinesTable::Line>& goalFrameLines(this->goalFrameLines.lines);
  LinesTable::Circle centerCircle;

  STREAM_REGISTER_BEGIN;
  STREAM_BASE(SimpleFieldDimensions)
  STREAM(goalAreaPresent);
  STREAM(carpetBorder);
  STREAM(goalFrameLines);
  STREAM(fieldBorder);
  STREAM(fieldLines);
  for (LinesTable::Line& line : fieldLines)
    line.length = (line.from - line.to).norm();
  STREAM(centerCircle);
  this->fieldLines.pushCircle(centerCircle.center, centerCircle.radius, centerCircle.numOfSegments);
  STREAM_REGISTER_FINISH;

  //merge fieldLines and goalFrameLines into fieldLinesWithGoalFrame
  for (LinesTable::Line& line : fieldLines)
    fieldLinesWithGoalFrame.lines.push_back(line);
  for (LinesTable::Line& line : goalFrameLines)
    fieldLinesWithGoalFrame.lines.push_back(line);
}
