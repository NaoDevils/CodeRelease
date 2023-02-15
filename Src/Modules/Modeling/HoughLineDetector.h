#include "Tools/Math/Angle.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"

#define BALLPERCEPT_BUFFER_LENGTH 30

class HoughLineDetector
{
public:
  HoughLineDetector();

  struct HoughPoint
  {
    std::vector<const BallPercept*> points;
    unsigned count;
    float alpha;
    float dist;
    short processed;
  };

  //struct HoughLine
  //{
  //  std::vector<HoughPoint*> houghPoints;
  //  float meanAlpha = 0.f;
  //  float meanDist = 0.f;
  //  float sumDist = 0.f;
  //  float sumSinAlpha = 0.f;
  //  float sumCosAlpha = 0.f;
  //};

  void setParameters(
      const Angle anglePrecision, const unsigned int distancePrecision, unsigned int minPointsForLine, unsigned int minPointsForLineWalking, const unsigned int height, const unsigned int width);
  void reset();
  bool execute(const RingBuffer<std::vector<BallPercept>, BALLPERCEPT_BUFFER_LENGTH>& bpr, const CameraMatrix& cameraMatrix, bool isWalking, BallPercept& rollingBall, Vector2f& velocity);

private:
  bool initialized = false;

  //int getPointfromHessianNormal(const float& x, const float& a, const float& d, const bool& upper);
  int getDistanceFromHoughTransform(const int& x, const int& y, const int& alpha);

  double distanceAlpha(const double& a1, const double& a2);
  double distanceDist(const double& d1, const double& d2);

  Vector2a getAngles(const Vector2f& relativePosition, const CameraMatrix& cameraMatrix);

  static bool sortCoordinates(const Vector3i first, const Vector3i second)
  {
    if (first.z() != second.z())
      return first.z() > second.z();
    else if (first.x() != second.x())
      return first.x() < second.x();
    else
      return first.y() < second.y();
  }

  static bool sortBPByDistance(const BallPercept* first, const BallPercept* second)
  {
    float distanceFirst = first->relativePositionOnField.norm();
    float distanceSecond = second->relativePositionOnField.norm();
    if (distanceFirst > distanceSecond)
      return false;
    else
      return true;
  }

  static bool sortBPByTimestamp(const BallPercept* first, const BallPercept* second)
  {
    if (first->timestamp > second->timestamp)
      return false;
    else
      return true;
  }

  Angle anglePrecision;
  unsigned int distancePrecision = 0;
  unsigned int width = 0, height = 0;

  unsigned int threshold = 0, thresholdWalking = 0;

  std::vector<Vector2i> changedHoughPoints;
  std::vector<std::vector<HoughPoint>> houghSpace;

  std::vector<float> sin_cache;
  std::vector<float> cos_cache;

  unsigned int circumference;
  unsigned int semiCircumference;
  unsigned int diagonal;
};
