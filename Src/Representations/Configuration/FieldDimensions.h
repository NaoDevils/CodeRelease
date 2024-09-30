/**
 * @file FieldDimensions.h
 *
 * Description of the dimensions of the field.
 *
 * @author Matthias Jüngel
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Boundary.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Geometry.h"

STREAMABLE(SimpleFieldDimensions,
  ENUM(BallType,
    orange,
    whiteBlack,
    any
  ),
  (float)(0.f) xPosOpponentFieldBorder,
  (float)(0.f) xPosOpponentGoal,
  (float)(0.f) xPosOpponentGoalArea,
  (float)(0.f) xPosOpponentGoalPost,
  (float)(0.f) xPosOpponentGroundline,
  (float)(0.f) xPosOpponentPenaltyArea,
  (float)(0.f) xPosOpponentPenaltyMark,
  (float)(0.f) xPosPenaltyStrikerStartPosition,
  (float)(0.f) xPosHalfWayLine,
  (float)(0.f) xPosOwnPenaltyArea,
  (float)(0.f) xPosOwnPenaltyMark,
  (float)(0.f) xPosOwnGroundline,
  (float)(0.f) xPosOwnGoalPost,
  (float)(0.f) xPosOwnGoalArea,
  (float)(0.f) xPosOwnGoal,
  (float)(0.f) xPosOwnFieldBorder,
  // Begin: Added by Dortmund. Used by module BallModelProvider.
  (float)(0.f) xPosKickOffPoint,
  // End

  (float)(0.f) yPosLeftFieldBorder,
  (float)(0.f) yPosLeftSideline,
  (float)(0.f) yPosLeftPenaltyArea,
  (float)(0.f) yPosLeftGoalArea,
  (float)(0.f) yPosLeftGoal,
  (float)(0.f) yPosCenterGoal,
  (float)(0.f) yPosRightGoal,
  (float)(0.f) yPosRightGoalArea,
  (float)(0.f) yPosRightPenaltyArea,
  (float)(0.f) yPosRightSideline,
  (float)(0.f) yPosRightFieldBorder,
  // Begin: Added by Dortmund. Used by module BallModelProvider.
  (float)(0.f) yPosKickOffPoint,
  // End

  //other dimensions
  (float)(0.f) fieldLinesWidth,
  (float)(0.f) centerCircleRadius,
  (float)(0.f) goalPostRadius,
  (float)(0.f) crossBarRadius,
  (float)(0.f) goalHeight,
  (BallType)(BallType::whiteBlack) ballType,
  (float)(0.f) ballRadius,
  (float)(0.f) penaltyMarkSize //vertical (and horizontal) size of a penaltyMark
);

/**
 * Class containing definitions and functions
 * regarding field dimensions.
 *
 * @author Max Risler
 */
struct FieldDimensions : public SimpleFieldDimensions
{
  /**
   * This is a collection of line- or boundary segments with start-Pose2f and length.
   */
  struct LinesTable
  {
    STREAMABLE(Line,
      Line() = default;
      Line(Vector2f from, Vector2f to, bool isPartOfCircle = false);

      float length;
      bool isPartOfCircle = false;/**< Whether the line is a part of a circle. */
      , 

      (Vector2f)(Vector2f::Zero()) from, /**< Begin of the line. */
      (Vector2f)(Vector2f::Zero()) to /**< End of the line. */
    );

    STREAMABLE(Circle,,
      (Vector2f) center, /**< The center of the circle. */
      (float) radius, /**< The radius of the circle. */
      (int) numOfSegments /**< The number of segments used to discretize the circle. */
    );

    LinesTable() = default;
    LinesTable(std::initializer_list<Line> lines) : lines(std::move(lines)){};

    std::vector<Line> lines;

    void push(const Pose2f& p, float l, bool isPartOfCircle = false);
    void push(const Vector2f& s, const Vector2f& e, bool isPartOfCircle = false);
    void pushCircle(const Vector2f& center, float radius, int numOfSegments);

    /**
     * Get the the closest point to p on a field line
     */
    Vector2f getClosestPoint(const Vector2f& p) const;

    /**
     * Intersects the specified line with each field line, returns the
     * intersection point that is closest to the base of the line.
     * @param outLineIndex is set to the index of the line that contains the intersection
     */
    bool getClosestIntersection(const Geometry::Line& l, int& outLineIndex, Vector2f& outIntersection) const;
    bool getClosestIntersection(const Geometry::Line& l, Vector2f& outIntersection) const;

    /**
     * Returns whether a given point is inside the polygon described by the line segments.
     * Only valid if the line segment table describes a closed polygon.
     */
    bool isInside(const Vector2f& v) const;

    /**
     * The function clips a point to the polygon described by the line segments.
     * Only valid if the line segment table describes a closed polygon.
     * @param v The point.
     * @return How far was the point moved?
     */
    float clip(Vector2f& v) const;

    /**
     * The function returns the point on a line of a certain type closest to given a point.
     * @param point The point on a line.
     * @param p The reference point and the rotation of the line.
     * @param numberOfRotations The number of discretizations of line rotations.
     * @param minLength The minimum length of the line segments that are considered.
     * @return whether there is a matching point in that direction
     */
    bool getClosestPoint(Vector2f& point, const Pose2f& p, int numberOfRotations, float minLength) const;

    /**
     * The function returns the distance between a point and the closest point on a line of a certain type in a certain direction.
     * @param pose The reference point and direction.
     * @return The distance. It is -1 if no line of that type exists in the certain direction.
     */
    float getDistance(const Pose2f& pose) const;
  };

  Boundaryf boundary; ///< The outer boundary of the field.
  LinesTable fieldLines; ///< Table of line segments
  LinesTable goalFrameLines; ///< Table of line segments that contains the parts of the goal frame that are on the ground.
  LinesTable fieldLinesWithGoalFrame; ///< Table of line segments that contains both fieldLines and goalFrameLines
  LinesTable carpetBorder; ///< Describes a polygon around the border of the field carpet. All legal robot positions are inside this polygon.
  LinesTable fieldBorder; ///< Describes a polygon around the border of the playing field. All legal ball positions are inside this polygon.
  bool goalAreaPresent = true;
  unsigned int lastUpdate = 0;

  /**
   * Read field dimensions from configuration file.
   */
  void load();

  /**
   * Read field dimensions from json file.
   */
  bool loadFromJsonFile(const std::string& path);

  /**
   * Returns true when p is inside the carpet.
   */
  bool isInsideCarpet(const Vector2f& p) const { return carpetBorder.isInside(p); }

  /**
   * The function clips a point to the carpet.
   * @param v The point.
   * @return How far was the point moved?
   */
  float clipToCarpet(Vector2f& v) const { return carpetBorder.clip(v); }

  /**
   * Returns true when p is inside the playing field.
   */
  bool isInsideField(const Vector2f& p) const { return fieldBorder.isInside(p); }

  /**
  * Returns true when ballPos is touching the playing fields borders.
  */
  bool isBallInsideField(const Vector2f& ballPos) const;

  /**
   * The function clips a point to the field.
   * @param v The point.
   * @return How far was the point moved?
   */
  float clipToField(Vector2f& v) const { return fieldBorder.clip(v); }

  /**
   * The function returns a random pose inside the field.
   * @return The random pose.
   */
  Pose2f randomPoseOnField() const;

  /**
   * The function returns a random pose on the carpet.
   * @return The random pose.
   */
  Pose2f randomPoseOnCarpet() const;

  /**
   * The method draws the field lines.
   */
  void draw() const;

  /**
   * Draws the goal frame.
   */
  void drawGoalFrame() const;

  /**
   * The method draws the field polygons.
   * @param ownColor The color of the own team.
   */
  void drawPolygons(int ownColor) const;

private:
  static constexpr char fieldDimensionsConfig[] = "/home/nao/logs/field_dimensions.json";
  long long int fieldDimensionsLastModified = 0;

  /**
   * Fill LinesTables and positions.
   */
  void fill();

  virtual void serialize(In* in, Out* out);

  /**
   * The method draws the field lines.
   */
  void drawLines() const;
};
