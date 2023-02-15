#include "Tools/Math/Angle.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"

#define BALLPERCEPT_BUFFER_LENGTH 30

class RANSACLineFitter
{
public:
  RANSACLineFitter();

  void setParameters(const Angle anglePrecision, const unsigned int distancePrecision, unsigned int minPointsForLine, unsigned int minPointsForLineWalking);
  void reset();
  bool execute(const RingBuffer<std::vector<BallPercept>, BALLPERCEPT_BUFFER_LENGTH>& bpr, const CameraMatrix& cameraMatrix, bool isWalking, BallPercept& rollingBall, Vector2f& velocity);

private:
  bool initialized = false;
  Angle anglePrecision;
  unsigned int distancePrecision = 0;
  unsigned int threshold = 0, thresholdWalking = 0;

  static bool sortDistance(const BallPercept* first, const BallPercept* second, const Vector2f& base)
  {
    //float distanceFirst = first->relativePositionOnField.norm();
    //float distanceSecond = second->relativePositionOnField.norm();

    Vector2f diffFirst = (first->relativePositionOnField - base);
    float baseDistanceFirst = diffFirst.norm();
    Vector2f diffSecond = (second->relativePositionOnField - base);
    float baseDistanceSecond = diffSecond.norm();

    if (baseDistanceFirst > baseDistanceSecond)
    {
      return false;
    }
    else
      return true;
  }

  static bool sortTimestamp(const BallPercept* first, const BallPercept* second)
  {
    if (first->timestamp > second->timestamp)
    {
      return true;
    }
    else
      return false;
  }

  Vector2a getAngles(const Vector2f& relativePosition, const CameraMatrix& cameraMatrix);
};
