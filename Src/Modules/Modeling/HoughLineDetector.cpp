#include "HoughLineDetector.h"

HoughLineDetector::HoughLineDetector() {}

void HoughLineDetector::setParameters(
    const Angle anglePrecision, const unsigned int distancePrecision, unsigned int minPointsForLine, unsigned int minPointsForLineWalking, const unsigned int height, const unsigned int width)
{
  if (!initialized || this->anglePrecision != anglePrecision || this->distancePrecision != distancePrecision || this->threshold != minPointsForLine
      || this->thresholdWalking != minPointsForLineWalking || this->height != height || this->width != width)
  {
    this->anglePrecision = anglePrecision;
    this->distancePrecision = distancePrecision;
    this->height = height;
    this->width = width;
    this->threshold = minPointsForLine;
    this->thresholdWalking = minPointsForLineWalking;

    circumference = static_cast<unsigned int>(360.f / this->anglePrecision.toDegrees());
    semiCircumference = static_cast<unsigned int>(180.f / this->anglePrecision.toDegrees());

    diagonal = static_cast<unsigned int>(std::ceil(std::sqrt(this->height * this->height + this->width * this->width) / distancePrecision));

    // initialize Sin & Cos Cache
    sin_cache.resize(circumference);
    cos_cache.resize(circumference);

    // iniatialize Hough-Space-Accumulator
    houghSpace.resize(circumference);

    for (unsigned alpha = 0; alpha < circumference; alpha++)
    {
      houghSpace[alpha].resize(diagonal);
      for (unsigned dist = 0; dist < diagonal; dist++)
      {
        houghSpace[alpha][dist].count = 0;
        houghSpace[alpha][dist].alpha = static_cast<float>(alpha);
        houghSpace[alpha][dist].dist = static_cast<float>(dist);
        houghSpace[alpha][dist].processed = 0;
        houghSpace[alpha][dist].points.reserve(30);
      }

      sin_cache[alpha] = std::sin(alpha * pi / semiCircumference);
      cos_cache[alpha] = std::cos(alpha * pi / semiCircumference);
    }

    changedHoughPoints.clear();

    initialized = true;
  }
}

void HoughLineDetector::reset()
{
  //for (unsigned alpha = 0; alpha < circumference; alpha++)
  //{
  //  for (unsigned dist = 0; dist < diagonal; dist++)
  //  {
  //    houghSpace[alpha][dist].count = 0;
  //    houghSpace[alpha][dist].alpha = static_cast<float>(alpha);
  //    houghSpace[alpha][dist].dist = static_cast<float>(dist);
  //    houghSpace[alpha][dist].processed = 0;
  //    houghSpace[alpha][dist].points.clear();
  //  }
  //}

  for (Vector2i& v : changedHoughPoints)
  {
    const unsigned alpha = v.x();
    const unsigned dist = v.y();
    houghSpace[alpha][dist].count = 0;
    houghSpace[alpha][dist].alpha = static_cast<float>(alpha);
    houghSpace[alpha][dist].dist = static_cast<float>(dist);
    houghSpace[alpha][dist].processed = 0;
    houghSpace[alpha][dist].points.clear();
  }
  changedHoughPoints.clear();
}

bool HoughLineDetector::execute(const RingBuffer<std::vector<BallPercept>, BALLPERCEPT_BUFFER_LENGTH>& bpr, const CameraMatrix& cameraMatrix, bool isWalking, BallPercept& rollingBall, Vector2f& velocity)
{
  if (!initialized || !(bpr.size() >= (isWalking ? thresholdWalking : threshold)))
    return false;

  std::vector<Vector3i> processedHoughPoints;

  for (const std::vector<BallPercept>& bpv : bpr)
  {
    for (const BallPercept& p : bpv)
    {
      for (unsigned alpha = 0; alpha < semiCircumference; alpha++)
      {
        int dist;
        dist = static_cast<int>(getDistanceFromHoughTransform(p.relativePositionOnField.cast<int>().x(), p.relativePositionOnField.cast<int>().y(), alpha));

        int alphaTemp;
        alphaTemp = alpha;
        if (dist < 0)
        {
          dist = -dist;
          alphaTemp = semiCircumference + alphaTemp;
        }
        dist = (dist + static_cast<int>(distancePrecision / 2.f)) / distancePrecision;

        bool duplicate = false;
        Vector2a anglesP = getAngles(p.relativePositionOnField, cameraMatrix);
        for (size_t i = 0; i < houghSpace[alphaTemp][dist].points.size(); i++)
        {
          float distance = (p.relativePositionOnField - (*houghSpace[alphaTemp][dist].points.at(i)).relativePositionOnField).norm();
          Vector2a anglesHSP = getAngles((*houghSpace[alphaTemp][dist].points.at(i)).relativePositionOnField, cameraMatrix);
          Vector2a anglesDiff = anglesHSP - anglesP;
          //float minDistance = p.relativePositionOnField.norm() * 0.02f;
          //if (isWalking)
          //{
          //  minDistance *= 2.f;
          //}

          //if (distance < std::max(static_cast<float>(distancePrecision), minDistance))
          Angle angleXDiff = anglesDiff.x();
          Angle angleYDiff = anglesDiff.y();
          if ((angleXDiff < 2_deg && angleYDiff < 0.5_deg) || distance < distancePrecision)
          {
            duplicate = true;
            //CIRCLE("module:HoughLineDetector:houghLine", p.relativePositionOnField.x(), p.relativePositionOnField.y(), 50, 2, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA(0, 0, 0, 127));
            break;
          }
        }


        if (!duplicate)
        {
          houghSpace[alphaTemp][dist].count++;
          houghSpace[alphaTemp][dist].points.push_back(&p);
          changedHoughPoints.push_back(Vector2i(alphaTemp, dist));

          if (houghSpace[alphaTemp][dist].processed == 0 && houghSpace[alphaTemp][dist].count >= (isWalking ? threshold * 2 : threshold))
          {
            houghSpace[alphaTemp][dist].processed = 1;
            processedHoughPoints.push_back(Vector3i(alphaTemp, dist, 0));
          }
        }
      }
    }
  }

  for (Vector3i& v : processedHoughPoints)
    v.z() = houghSpace[v.x()][v.y()].count;

  std::sort(processedHoughPoints.begin(), processedHoughPoints.end(), sortCoordinates);

  if (processedHoughPoints.size() > 0)
  {
    HoughPoint hp = houghSpace[processedHoughPoints[0].x()][processedHoughPoints[0].y()];
    std::sort(hp.points.begin(), hp.points.end(), sortBPByTimestamp);
    rollingBall = *hp.points.back();
    float timeDiff = abs(static_cast<float>(hp.points.back()->timestamp) - static_cast<float>(hp.points.front()->timestamp));
    Vector2f distanceDiff = hp.points.back()->relativePositionOnField - hp.points.front()->relativePositionOnField;
    velocity = (distanceDiff / (timeDiff / 1000.f));

    DEBUG_DRAWING("module:HoughLineDetector:houghLine", "drawingOnField")
    {
      CROSS("module:HoughLineDetector:houghLine", rollingBall.relativePositionOnField.x(), rollingBall.relativePositionOnField.y(), 20, 3, Drawings::solidPen, isWalking ? ColorRGBA::red : ColorRGBA::blue);
      for (const BallPercept* bp : hp.points)
      {
        CIRCLE("module:HoughLineDetector:houghLine",
            bp->relativePositionOnField.x(),
            bp->relativePositionOnField.y(),
            50,
            2,
            Drawings::solidPen,
            isWalking ? ColorRGBA::red : ColorRGBA::blue,
            Drawings::solidBrush,
            isWalking ? ColorRGBA(255, 0, 0, 127) : ColorRGBA(0, 0, 255, 127));
      }
    }
    return true;
  }
  return false;
}

//int HoughLineDetector::getPointfromHessianNormal(const float& x, const float& a, const float& d, const bool& upper)
//{
//  float result = -(x * cos_cache[static_cast<unsigned>(std::ceil(a))] - d) / (sin_cache[static_cast<unsigned>(std::ceil(a))]);
//  return std::min<int>(static_cast<int>(std::abs(result)), this->height);
//}

int HoughLineDetector::getDistanceFromHoughTransform(const int& x, const int& y, const int& alpha)
{
  return static_cast<int>(x * cos_cache[alpha] + y * sin_cache[alpha]);
}

double HoughLineDetector::distanceAlpha(const double& a1, const double& a2)
{
  double deltaAlpha = abs(a1 - a2);
  if (deltaAlpha > semiCircumference)
    deltaAlpha = circumference - deltaAlpha;
  return deltaAlpha;
}

double HoughLineDetector::distanceDist(const double& d1, const double& d2)
{
  return abs(d1 - d2);
}

Vector2a HoughLineDetector::getAngles(const Vector2f& relativePosition, const CameraMatrix& cameraMatrix)
{
  Vector2a angles;
  angles.x() = std::atan2(relativePosition.y(), relativePosition.x());
  angles.y() = std::atan2(cameraMatrix.translation.z(), relativePosition.norm());
  return angles;
}
