#include "RANSACLineFitter.h"
#include "Tools/Math/Random.h"
#include <numeric>

RANSACLineFitter::RANSACLineFitter() {}

void RANSACLineFitter::setParameters(const Angle anglePrecision, const unsigned int distancePrecision, unsigned int minPointsForLine, unsigned int minPointsForLineWalking)
{
  if (!initialized || this->anglePrecision != anglePrecision || this->distancePrecision != distancePrecision || this->threshold != minPointsForLine || this->thresholdWalking != minPointsForLineWalking)
  {
    this->anglePrecision = anglePrecision;
    this->distancePrecision = distancePrecision;
    this->threshold = minPointsForLine;
    this->thresholdWalking = minPointsForLineWalking;

    initialized = true;
  }
}

void RANSACLineFitter::reset() {}

bool RANSACLineFitter::execute(const RingBuffer<std::vector<BallPercept>, BALLPERCEPT_BUFFER_LENGTH>& bpr, const CameraMatrix& cameraMatrix, bool isWalking, BallPercept& rollingBall, Vector2f& velocity)
{
  if (!initialized || !(bpr.size() >= (isWalking ? thresholdWalking : threshold)))
    return false;

  std::vector<const BallPercept*> allBPs;
  for (const std::vector<BallPercept>& bpv : bpr)
  {
    for (const BallPercept& p : bpv)
    {
      bool duplicate = false;
      Vector2a anglesP = getAngles(p.relativePositionOnField, cameraMatrix);
      for (size_t i = 0; i < allBPs.size(); i++)
      {
        float distance = (p.relativePositionOnField - allBPs.at(i)->relativePositionOnField).norm();
        Vector2a anglesHSP = getAngles(allBPs.at(i)->relativePositionOnField, cameraMatrix);
        Vector2a anglesDiff = anglesHSP - anglesP;

        if (anglesDiff.norm() < anglePrecision || distance < distancePrecision / 2.f)
        {
          duplicate = true;
          break;
        }
      }
      if (!duplicate)
      {
        allBPs.push_back(std::addressof(p));
        CIRCLE("module:RANSACLineFitter:perceptBuffer",
            p.relativePositionOnField.x(),
            p.relativePositionOnField.y(),
            distancePrecision / 2.f,
            2,
            Drawings::solidPen,
            ColorRGBA::white,
            Drawings::solidBrush,
            ColorRGBA(255, 255, 255, 100));
      }
    }
  }

  int indexBase = 0;
  int indexDirection = 0;

  std::vector<const BallPercept*> inliers;
  unsigned numOfInliers = 0;
  float summedDistanceInliers = 0.f;

  std::vector<const BallPercept*> outliers;
  unsigned numOfOutliers = 0;
  float summedDistanceOutliers = 0.f;

  unsigned numOfBPs = static_cast<unsigned>(allBPs.size());
  if (numOfBPs < (isWalking ? thresholdWalking : threshold))
    return false;

  unsigned bestNoOfInliers = 0;
  Geometry::Line bestLine;
  std::vector<const BallPercept*> bestInliers;
  std::vector<const BallPercept*> bestOutliers;
  float bestSummedDistanceInliers = 0.f;
  float bestSummedDistanceOutliers = 0.f;

  for (unsigned round = 0; round < numOfBPs; round++)
  {
    numOfInliers = 0;
    numOfOutliers = 0;
    summedDistanceInliers = 0.f;
    summedDistanceOutliers = 0.f;
    inliers.clear();
    outliers.clear();

    //indexBase = random(numOfBPs);
    indexBase = 0;
    indexDirection = random(numOfBPs);
    while (indexBase == indexDirection)
      indexDirection = random(numOfBPs);

    Geometry::Line testLine;
    testLine.base = allBPs.at(indexBase)->relativePositionOnField;
    testLine.direction = allBPs.at(indexDirection)->relativePositionOnField - allBPs.at(indexBase)->relativePositionOnField;

    for (unsigned i = 0; i < numOfBPs; i++)
    {
      float distanceToLine = std::abs(Geometry::getDistanceToLine(testLine, allBPs.at(i)->relativePositionOnField));
      if (distanceToLine < static_cast<float>(distancePrecision))
      {
        summedDistanceInliers += distanceToLine;
        numOfInliers++;
        inliers.push_back(allBPs.at(i));
      }
      else
      {
        summedDistanceOutliers += distanceToLine;
        numOfOutliers++;
        outliers.push_back(allBPs.at(i));
      }
    }

    if (numOfInliers > bestNoOfInliers || (numOfInliers == bestNoOfInliers && summedDistanceInliers < bestSummedDistanceInliers))
    {
      bestNoOfInliers = numOfInliers;
      bestLine = testLine;
      bestInliers = inliers;
      bestOutliers = outliers;
      bestSummedDistanceInliers = summedDistanceInliers;
      bestSummedDistanceOutliers = summedDistanceOutliers;
    }

    if (numOfInliers >= BALLPERCEPT_BUFFER_LENGTH)
      break;
  }

  float meanDistanceInliers = (bestInliers.size() == 0 ? 0.f : bestSummedDistanceInliers / bestInliers.size());
  float meanDistanceOutliers = (bestOutliers.size() == 0 ? 0.f : bestSummedDistanceOutliers / bestOutliers.size());

  if (bestNoOfInliers >= (isWalking ? thresholdWalking : threshold) && meanDistanceInliers <= (distancePrecision * 2.f / 3.f) && meanDistanceOutliers <= (distancePrecision * 4.f / 3.f))
  {
    Vector2f start(bestLine.base.x(), bestLine.base.y());
    Vector2f end;
    std::sort(bestInliers.begin(), bestInliers.end(), std::bind(sortDistance, std::placeholders::_1, std::placeholders::_2, start));
    if (Geometry::getPerpendicularFootPointToLine(bestLine, bestInliers.at(bestInliers.size() - 1)->relativePositionOnField, end))
    {
      bestLine.direction = end - start;
      end.x() = bestLine.base.x() + bestLine.direction.x();
      end.y() = bestLine.base.y() + bestLine.direction.y();
    }
    else
    {
      return false;
    }


    std::sort(bestInliers.begin(), bestInliers.end(), sortTimestamp);
    std::vector<Vector2f> velocities;
    rollingBall = *bestInliers.front();
    for (unsigned i = 1; i < bestInliers.size(); i++)
    {
      float timeDiff = static_cast<float>(bestInliers.front()->timestamp) - static_cast<float>(bestInliers.at(i)->timestamp);
      Vector2f distanceDiff = bestInliers.front()->relativePositionOnField - bestInliers.at(i)->relativePositionOnField;
      velocities.push_back((distanceDiff / (timeDiff / 1000.f)));
    }

    float timeDiff = abs(static_cast<float>(bestInliers.front()->timestamp) - static_cast<float>(bestInliers.back()->timestamp));
    Vector2f distanceDiff = -bestLine.direction;
    Vector2f avgVel = distanceDiff / (timeDiff / 1000.f);
    //Vector2f avgVel = Vector2f::Zero();
    //for (const Vector2f& t : velocities)
    //{
    //  avgVel += t;
    //}
    //avgVel /= static_cast<float>(velocities.size());

    velocity = avgVel;

    Vector2f varVel = Vector2f::Zero();
    for (const Vector2f& t : velocities)
    {
      Vector2f diff = (t - avgVel);
      //var += diff.cwiseProduct(diff);
      varVel += (diff.cwiseProduct(diff)).cwiseSqrt();
    }
    varVel = (1.f / (static_cast<float>(velocities.size()) - 1.f)) * varVel;

    //Vector2f avgVelNew = Vector2f::Zero();
    //unsigned counter = 0;
    //for (const Vector2f& t : velocities)
    //{
    //  if ((t - avgVel).norm() < varVel.norm())
    //  {
    //    avgVelNew += t;
    //    counter++;
    //  }
    //}
    //avgVelNew /= static_cast<float>(counter);
    //velocity = avgVelNew;

    if (velocity.norm() > 500 && velocity.norm() * 0.33f > varVel.norm())
    {
      DEBUG_DRAWING("module:RANSACLineFitter:line", "drawingOnField")
      {
        for (const BallPercept* bp : bestInliers)
        {
          CIRCLE("module:RANSACLineFitter:line",
              bp->relativePositionOnField.x(),
              bp->relativePositionOnField.y(),
              distancePrecision / 2.f,
              2,
              Drawings::solidPen,
              isWalking ? ColorRGBA::red : ColorRGBA::blue,
              Drawings::solidBrush,
              isWalking ? ColorRGBA(255, 0, 0, 127) : ColorRGBA(0, 0, 255, 127));
        }
        for (const BallPercept* bp : bestOutliers)
        {
          CIRCLE("module:RANSACLineFitter:line",
              bp->relativePositionOnField.x(),
              bp->relativePositionOnField.y(),
              distancePrecision / 2.f,
              2,
              Drawings::solidPen,
              ColorRGBA::yellow,
              Drawings::solidBrush,
              ColorRGBA(255, 255, 0, 127));
        }
        LINE("module:RANSACLineFitter:line", start.x(), start.y(), end.x(), end.y(), 10, Drawings::solidPen, ColorRGBA::red);
        //OUTPUT_TEXT("Inliers:" << meanDistanceInliers << " | Outliers:" << meanDistanceOutliers << " | Velocity:" << velocity.norm() << " | Variance:" << varVel.norm());
      }

      return true;
    }
    return false;
  }
  return false;
}

Vector2a RANSACLineFitter::getAngles(const Vector2f& relativePosition, const CameraMatrix& cameraMatrix)
{
  Vector2a angles;
  angles.x() = std::atan2(relativePosition.y(), relativePosition.x());
  angles.y() = std::atan2(cameraMatrix.translation.z(), relativePosition.norm());
  return angles;
}
