#pragma once

#include "Tools/Math/BHMath.h"
#include "Tools/Math/GaussianDistribution2D.h"
#include "Tools/Math/GaussianDistribution.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Geometry.h"
#include <math.h>
#include <algorithm>

struct Potential
{
  Potential()
  {
    position = Vector2f::Zero();
    influence = 0;
  }
  Potential(float _x, float _y, float _influence)
  {
    position.x() = _x;
    position.y() = _y;
    influence = _influence;
  }
  Potential(Vector2f _position, float _influence)
  {
    position = _position;
    influence = _influence;
  }
  float distance(Potential pot) { return (position - pot.position).norm(); }
  float distance(const Vector2f& point) { return (position - point).norm(); }
  Vector2f position;
  float influence;
};

class QPotential : public Potential
{
public:
  QPotential()
  {
    position.x() = 0;
    position.y() = 0;
    influence = 0;
    influenceRadius = 0;
  }
  QPotential(float _x, float _y, float _influenceRadius, float _influence)
  {
    position.x() = _x;
    position.y() = _y;
    influenceRadius = _influenceRadius;
    influence = _influence;
  }
  QPotential(Vector2f _position, float _influenceRadius, float _influence)
  {
    position = _position;
    influence = _influence;
    influenceRadius = _influenceRadius;
  }
  float influenceRadius;
};

class EPotential : public Potential
{
public:
  EPotential(float _x, float _y, float _influence, QPotential _left, QPotential _right)
  {
    position.x() = _x;
    position.y() = _y;
    influence = _influence;
    left = _left;
    right = _right;
  }
  EPotential(Vector2f _position, float _influence, QPotential _left, QPotential _right)
  {
    position = _position;
    influence = _influence;
    left = _left;
    right = _right;
  }
  EPotential(QPotential _left, QPotential _right)
  {
    position = Vector2f::Zero();
    influence = 0;
    left = _left;
    right = _right;
  }
  float angleTo(const Vector2f& point)
  {
    Vector2f r1 = point - position;
    Vector2f r2 = ((left.position - position).norm() > 0) ? (left.position - position) : (position - right.position);
    return std::acos(r1.dot(r2 / ((0.0001f + r1.norm() * r2.norm()))));
  }
  float distance(const Vector2f& point) { return (position - point).norm(); }
  //calculate nearest point on the line from left to right from point
  Vector2f nearestPoint(const Vector2f& point)
  {
    Vector2f r(right.position - left.position);
    Vector2f normal(-r.y(), r.x());
    float s = (-r.y() * left.position.y() + r.x() * point.x() - r.x() * left.position.x() + r.y() * point.y()) / (r.x() * r.x() + r.y() * r.y() + 0.0001f);
    if (s > 1)
      s = 1;
    if (s < 0)
      s = 0;
    return Vector2f(left.position.x() + s * r.x(), left.position.y() + s * r.y());
  }

  Vector2f oppositeToNearestPoint(const Vector2f& point)
  {
    Vector2f r(right.position - left.position);
    Vector2f normal(-r.y(), r.x());
    float s = (-r.y() * left.position.y() + r.x() * point.x() - r.x() * left.position.x() + r.y() * point.y()) / (r.x() * r.x() + r.y() * r.y() + 0.0001f);
    if (s > 1)
      s = 1;
    if (s < 0)
      s = 0;
    s = 1 - s;
    return Vector2f(left.position.x() + s * r.x(), left.position.y() + s * r.y());
  }
  QPotential left;
  QPotential right;


  float influenceRadiusAtAngle(float alpha)
  {
    if (alpha < 0)
      alpha = -alpha;
    if (alpha <= (pi / 2))
    {
      return (Vector2f(((left.position - position).norm() + left.influenceRadius) * std::cos(alpha),
                  (std::max(left.influenceRadius, right.influenceRadius) + ((left.position - right.position).norm()) / 2) * std::sin(alpha)))
          .norm();
    }
    return (Vector2f(((right.position - position).norm() + right.influenceRadius) * std::cos(alpha),
                (std::max(left.influenceRadius, right.influenceRadius) + ((left.position - right.position).norm()) / 2) * std::sin(alpha)))
        .norm();
  }
  float influenceAtAngle(float alpha)
  {
    if (alpha < 0)
      alpha = -alpha;
    if (alpha <= pi_2)
    {
      return left.influence + (alpha / (pi_2)) * right.influence;
    }
    return right.influence + (alpha / (pi_2)) * left.influence;
  }
  //TODO
};

//rectangular potential
class RPotential : public Potential
{
public:
  Vector2f front, back, left, right;

  RPotential(Vector2f pos, float inf, Vector2f _front, Vector2f _back, Vector2f _left, Vector2f _right)
  {
    position = pos;
    influence = inf;
    back = _back;
    front = _front;
    left = _left;
    right = _right;
  }

  //rectangular potential with equal sides and center on the back
  RPotential(Vector2f pos, float inf, Vector2f _front, Vector2f side)
  {
    position = pos;
    influence = inf;
    back = Vector2f(0, 0);
    front = _front;
    left = side;
    right = -side;
  }
  float angleToFront(const Vector2f& direction) { return std::acos(direction.dot(front / (direction.norm() * front.norm() + 0.0001f))); }

  Vector2f diagonalFL()
  {
    if (left.norm() == 0)
      return front;
    if (front.norm() == 0)
      return left;
    float length = std::sqrt(sqr(front.norm()) + sqr(left.norm()));
    return Vector2f((length / (front + left).norm()) * (front + left).x(), (length / (front + left).norm()) * (front + left).y());
  }
  Vector2f diagonalFR()
  {
    if (right.norm() == 0)
      return front;
    if (front.norm() == 0)
      return right;
    float length = std::sqrt(sqr(front.norm()) + sqr(right.norm()));
    return Vector2f((length / (front + right).norm()) * (front + right).x(), (length / (front + right).norm()) * (front + right).y());
  }
  Vector2f diagonalBR()
  {
    if (back.norm() == 0)
      return right;
    if (right.norm() == 0)
      return back;
    float length = std::sqrt(sqr(back.norm()) + sqr(right.norm()));
    return Vector2f((length / (back + right).norm()) * (back + right).x(), (length / (back + right).norm()) * (back + right).y());
  }
  Vector2f diagonalBL()
  {
    if (back.norm() == 0)
      return left;
    if (left.norm() == 0)
      return back;
    float length = std::sqrt(sqr(back.norm()) + sqr(left.norm()));
    return Vector2f((length / (back + left).norm()) * (back + left).x(), (length / (back + left).norm()) * (back + left).y());
  }

  //returns the vector normalized to fit the dimensions of the rectangle
  Vector2f fitToRectangle(Vector2f direction)
  {
    Vector2f intersection = Vector2f::Zero();
    Geometry::Line line1(Vector2f(Vector2f::Zero()), direction);

    if ((angleToFront(direction) > angleToFront(diagonalFL()) && angleToFront(direction) <= angleToFront(diagonalBL()))
        || (angleToFront(direction) < angleToFront(diagonalFL()) && angleToFront(direction) >= angleToFront(diagonalBL())))
    {
      //left
      Geometry::Line line2(left, diagonalFL() - left);

      if (!Geometry::getIntersectionOfLines(line1, line2, intersection))
        return intersection;
    }
    else if ((angleToFront(direction) > angleToFront(diagonalBL()) && angleToFront(direction) <= angleToFront(diagonalBR()))
        || (angleToFront(direction) < angleToFront(diagonalBL()) && angleToFront(direction) >= angleToFront(diagonalBR())))
    {
      //back
      Geometry::Line line2(back, diagonalBR() - back);

      if (!Geometry::getIntersectionOfLines(line1, line2, intersection))
        return intersection;
    }
    else if ((angleToFront(direction) > angleToFront(diagonalBR()) && angleToFront(direction) <= angleToFront(diagonalFR()))
        || (angleToFront(direction) < angleToFront(diagonalBR()) && angleToFront(direction) >= angleToFront(diagonalFR())))
    {
      //right
      Geometry::Line line2(right, diagonalFR() - right);

      if (!Geometry::getIntersectionOfLines(line1, line2, intersection))
        return intersection;
    }
    else
    {
      //front
      Geometry::Line line2(front, diagonalFL() - front);

      if (!Geometry::getIntersectionOfLines(line1, line2, intersection))
        return intersection;
    }

    return intersection;
  }
};

class PotentialField
{
public:
  PotentialField(void);
  ~PotentialField(void);
  void addLinearPotential(const Potential& potential);
  void addQuadraticPotential(const QPotential& potential);
  void addEllipticalPotential(const EPotential& potential);
  void addRectangularPotential(const RPotential& potential);
  void clear();
  Vector2f getGradient(const Vector2f& point);

  //private:
  std::vector<Potential> linearPotentials;
  std::vector<QPotential> quadraticPotentials;
  std::vector<EPotential> ellipticalPotentials;
  std::vector<RPotential> rectangularPotentials;
};
