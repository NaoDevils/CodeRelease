#include "SonarDetector.h"
#include "Tools/Debugging/DebugDrawings.h"

void SonarDetector::update(SonarPercept& theSonarPercept)
{
  declarePlots();
  clearSonarPercept(theSonarPercept);

  if (theFallDownState.state != FallDownState::State::upright)
  {
    resetFilterValues();
    return;
  }

  constexpr float mToMmConversionValue = 1000;
  float leftMeasurementMm = theSonarSensorData.leftDistanceM * mToMmConversionValue;
  float rightMeasurementMm = theSonarSensorData.rightDistanceM * mToMmConversionValue;

  if (sensorHasNewData(leftMeasurementMm, leftSensorAttributes.lastMeasurementMm, leftSensorAttributes.lastTimestamp))
  {
    /* left value has changed or timed out */
    SonarEstimate leftSonarEstimate;
    updateSonarValue(leftMeasurementMm, leftMeasurementRingBuffer, theRobotDimensions.leftSonarInfo.rotation, theRobotDimensions.leftSonarInfo.translation, true, leftSonarEstimate);
    leftSensorAttributes.currentSonarEstimate = leftSonarEstimate;
    leftSensorAttributes.lastMeasurementMm = leftMeasurementMm;
    leftSensorAttributes.lastTimestamp = leftSensorAttributes.currentSonarEstimate.timestamp;
  }

  if (sensorHasNewData(rightMeasurementMm, rightSensorAttributes.lastMeasurementMm, rightSensorAttributes.lastTimestamp))
  {
    /* right value has changed or timed out */
    SonarEstimate rightSonarEstimate;
    updateSonarValue(rightMeasurementMm, rightMeasurementRingBuffer, theRobotDimensions.rightSonarInfo.rotation, theRobotDimensions.rightSonarInfo.translation, false, rightSonarEstimate);
    rightSensorAttributes.currentSonarEstimate = rightSonarEstimate;
    rightSensorAttributes.lastMeasurementMm = rightMeasurementMm;
    rightSensorAttributes.lastTimestamp = rightSensorAttributes.currentSonarEstimate.timestamp;
  }

  bool isLeftSonarEstimateValid = isValidSonarEstimate(leftSensorAttributes.currentSonarEstimate);
  bool isRightSonarEstimateValid = isValidSonarEstimate(rightSensorAttributes.currentSonarEstimate);
  if (isLeftSonarEstimateValid && isRightSonarEstimateValid)
  {
    /* left and right estimate is valid, possibly merge, otherwise add both */
    SonarEstimate mergedSonarEstimate;
    if (mergeSonarEstimates(leftSensorAttributes.currentSonarEstimate, rightSensorAttributes.currentSonarEstimate, mergedSonarEstimate))
      theSonarPercept.sonarEstimates.push_back(mergedSonarEstimate);
    else
    {
      theSonarPercept.sonarEstimates.push_back(leftSensorAttributes.currentSonarEstimate);
      theSonarPercept.sonarEstimates.push_back(rightSensorAttributes.currentSonarEstimate);
    }
  }
  else if (isLeftSonarEstimateValid)
    /* only left estimate is valid */
    theSonarPercept.sonarEstimates.push_back(leftSensorAttributes.currentSonarEstimate);
  else if (isRightSonarEstimateValid)
    /* only right estimate is valid */
    theSonarPercept.sonarEstimates.push_back(rightSensorAttributes.currentSonarEstimate);
}

bool SonarDetector::sensorHasNewData(const float& sensorMeasurement, const float& lastMeasurement, const unsigned& lastTimestamp)
{
  /* 8Hz = value updates every 125 ms, then round up a little */
  constexpr int timeoutTimespan = 130;
  return sensorMeasurement != lastMeasurement || theSonarSensorData.timestamp - lastTimestamp >= timeoutTimespan;
}

void SonarDetector::declarePlots()
{
  DECLARE_PLOT("perception:SonarPercept:left:raw");
  DECLARE_PLOT("perception:SonarPercept:left:movingAverage");
  DECLARE_PLOT("perception:SonarPercept:left:exponential");
  DECLARE_PLOT("perception:SonarPercept:left:minMaxExponential");
  DECLARE_PLOT("perception:SonarPercept:left:kalman");
  DECLARE_PLOT("perception:SonarPercept:right:raw");
  DECLARE_PLOT("perception:SonarPercept:right:movingAverage");
  DECLARE_PLOT("perception:SonarPercept:right:exponential");
  DECLARE_PLOT("perception:SonarPercept:right:minMaxExponential");
  DECLARE_PLOT("perception:SonarPercept:right:kalman");
}

void SonarDetector::clearSonarPercept(SonarPercept& theSonarPercept)
{
  theSonarPercept.sonarEstimates.clear();
}

void SonarDetector::resetFilterValues()
{
  leftMeasurementRingBuffer.clear();
  rightMeasurementRingBuffer.clear();
  lastLeftExponentialMeasurement = 0;
  lastRightExponentialMeasurement = 0;
  leftMinMaxExpoAttrs = {0, 0};
  rightMinMaxExpoAttrs = {0, 0};
  /* TODO: do we need to clear the kalman attributes? */
  /*leftKalmanParameters = {};
  rightKalmanParameters = {};*/
}

bool SonarDetector::isValidMeasurement(const float& sensorMeasurement)
{
  return theSonarConfiguration.minDistanceMm <= sensorMeasurement && sensorMeasurement <= theSonarConfiguration.maxDistanceMm;
}

bool SonarDetector::isValidSonarEstimate(const SonarEstimate& sonarEstimate)
{
  /* subtract the sonar offset from the sonar estimate's distance to get the real measured distance. This
     offset is the same for the left and the right sonar. If this function would need to be applied to the middle
     sonar, just use the x coordinates of the left sonar offset */
  return sonarEstimate.validity >= minValidity && isValidMeasurement(sonarEstimate.distance - theRobotDimensions.leftSonarInfo.translation.norm());
}

void SonarDetector::updateSonarValue(
    const float& sensorMeasurement, RingBuffer<float, ringBufferSize>& ringBuffer, const Angle& sensorAngle, const Vector2f& sonarOffset, const bool leftSensor, SonarEstimate& sonarEstimate)
{
  bool validMeasurement = isValidMeasurement(sensorMeasurement);
  std::tuple<float, float> filteredTuple;

  switch (filterMode)
  {
  case SonarFilter::movingAverage:
    ringBuffer.push_front(validMeasurement ? sensorMeasurement : -1);
    filteredTuple = movingAverageFilter(ringBuffer);
    break;
  case SonarFilter::exponential:
    filteredTuple = exponentialFilter(sensorMeasurement, leftSensor ? lastLeftExponentialMeasurement : lastRightExponentialMeasurement);
    break;
  case SonarFilter::minMaxExponential:
    filteredTuple = minMaxExponentialFilter(sensorMeasurement, leftSensor ? leftMinMaxExpoAttrs : rightMinMaxExpoAttrs);
    break;
  case SonarFilter::kalman:
    filteredTuple = kalmanFilter(sensorMeasurement, leftSensor ? leftKalmanParameters : rightKalmanParameters);
    break;
  default:
    OUTPUT_ERROR("filterMode must be specified.");
  }

  sonarEstimate.sensorPosition = leftSensor ? SonarEstimate::SensorPosition::left : SonarEstimate::SensorPosition::right;
  sonarEstimate.timestamp = theSonarSensorData.timestamp;
  sonarEstimate.validity = std::get<1>(filteredTuple);
  // add sonar offset and rotate, since we want the relative position of the sonar estimate from the middle of the robot
  sonarEstimate.relativePosition = Pose2f(sonarOffset).rotate(sensorAngle).translate(std::get<0>(filteredTuple), 0);
  sonarEstimate.distance = sonarEstimate.relativePosition.translation.norm();

  if (drawPlots)
    drawGraphs(ringBuffer, sensorMeasurement, validMeasurement, leftSensor);
}

bool SonarDetector::mergeSonarEstimates(const SonarEstimate& leftSonarEstimate, const SonarEstimate& rightSonarEstimate, SonarEstimate& mergedSonarEstimate)
{
  if (std::abs(leftSonarEstimate.distance - rightSonarEstimate.distance) <= maxMergeDistance)
  {
    mergedSonarEstimate.sensorPosition = SonarEstimate::SensorPosition::middle;
    mergedSonarEstimate.timestamp = std::min(leftSonarEstimate.timestamp, rightSonarEstimate.timestamp);
    mergedSonarEstimate.validity = std::max(leftSonarEstimate.validity, rightSonarEstimate.validity);
    mergedSonarEstimate.distance = std::min(leftSonarEstimate.distance, rightSonarEstimate.distance);
    mergedSonarEstimate.relativePosition = Pose2f(mergedSonarEstimate.distance, 0);
    return true;
  }
  return false;
}

void SonarDetector::drawGraphs(RingBuffer<float, ringBufferSize>& ringBuffer, float sensorMeasurement, bool validMeasurement, const bool leftSensor)
{
  float validSensorMeasurement = isValidMeasurement(sensorMeasurement) ? sensorMeasurement : 0;
  std::tuple<float, float> movingAverageFilteredTuple = movingAverageFilter(ringBuffer);
  std::tuple<float, float> exponentialFilteredTuple = exponentialFilter(sensorMeasurement, leftSensor ? lastLeftExponentialMeasurement : lastRightExponentialMeasurement);
  std::tuple<float, float> minMaxExponentialFilteredTuple = minMaxExponentialFilter(sensorMeasurement, leftSensor ? leftMinMaxExpoAttrs : rightMinMaxExpoAttrs);
  std::tuple<float, float> kalmanFilteredTuple = kalmanFilter(sensorMeasurement, leftSensor ? leftKalmanParameters : rightKalmanParameters);

  if (leftSensor)
  {
    PLOT("perception:SonarPercept:left:raw", validSensorMeasurement);
    PLOT("perception:SonarPercept:left:movingAverage", std::get<0>(movingAverageFilteredTuple));
    PLOT("perception:SonarPercept:left:exponential", std::get<0>(exponentialFilteredTuple));
    PLOT("perception:SonarPercept:left:minMaxExponential", std::get<0>(minMaxExponentialFilteredTuple));
    PLOT("perception:SonarPercept:left:kalman", std::get<0>(kalmanFilteredTuple));
  }
  else
  {
    PLOT("perception:SonarPercept:right:raw", validSensorMeasurement);
    PLOT("perception:SonarPercept:right:movingAverage", std::get<0>(movingAverageFilteredTuple));
    PLOT("perception:SonarPercept:right:exponential", std::get<0>(exponentialFilteredTuple));
    PLOT("perception:SonarPercept:right:minMaxExponential", std::get<0>(minMaxExponentialFilteredTuple));
    PLOT("perception:SonarPercept:right:kalman", std::get<0>(kalmanFilteredTuple));
  }
}

std::tuple<float, float> SonarDetector::movingAverageFilter(RingBuffer<float, ringBufferSize>& ringBuffer)
{
  float nonNegativeSum = 0;
  int nonNegativeCount = 0;
  for (auto it = ringBuffer.begin(); it != ringBuffer.end(); it++)
  {
    if (*it >= 0)
    {
      nonNegativeSum += *it;
      nonNegativeCount++;
    }
  }
  if (nonNegativeCount == 0)
    return {0.f, 0.f};
  else
  {
    float movingAverageMeasurement = nonNegativeSum / static_cast<float>(nonNegativeCount);
    float movingAverageFilteredValidity = nonNegativeCount / static_cast<float>(ringBufferSize);
    return {movingAverageMeasurement, movingAverageFilteredValidity};
  }
}

std::tuple<float, float> SonarDetector::exponentialFilter(float sensorMeasurement, float& lastExponentialMeasurement)
{
  float filteredMeasurement = sensorMeasurement * exponent + lastExponentialMeasurement * (1 - exponent);
  lastExponentialMeasurement = filteredMeasurement;
  if (isValidMeasurement(filteredMeasurement))
  {
    return {filteredMeasurement, 1.f};
  }
  else
  {
    return {0.f, 0.f};
  }
}

std::tuple<float, float> SonarDetector::minMaxExponentialFilter(float sensorMeasurement, MinMaxExponentialAttributes& minMaxExpoAttrs)
{
  bool validMeasurement = isValidMeasurement(sensorMeasurement);
  float minMaxExponentialValidity = validMeasurement * minMaxExponent + minMaxExpoAttrs.lastValidity * (1 - minMaxExponent);
  if (validMeasurement)
  {
    float filteredMeasurement = sensorMeasurement * minMaxExponent + minMaxExpoAttrs.lastMeasurement * (1 - minMaxExponent);
    minMaxExpoAttrs.lastMeasurement = filteredMeasurement;
    return {filteredMeasurement, minMaxExponentialValidity};
  }
  else
  {
    return {0.f, 0.f};
  }
}

std::tuple<float, float> SonarDetector::kalmanFilter(float sonarSensorMeasurement, KalmanParameters& kalmanParameters)
{
  /* update kalman gain */
  kalmanParameters.K = kalmanParameters.P * kalmanParameters.H / (kalmanParameters.H * kalmanParameters.P * kalmanParameters.H + kalmanParameters.R);
  /* update estimate */
  kalmanParameters.filteredMeasurement = kalmanParameters.filteredMeasurement + kalmanParameters.K * (sonarSensorMeasurement - kalmanParameters.H * kalmanParameters.filteredMeasurement);
  /* update error covariance */
  kalmanParameters.P = (1 - kalmanParameters.K * kalmanParameters.H) * kalmanParameters.P + kalmanParameters.Q;

  if (isValidMeasurement(kalmanParameters.filteredMeasurement))
    return {kalmanParameters.filteredMeasurement, 1.f};
  else
    return {0.f, 0.f};
}

MAKE_MODULE(SonarDetector, perception);
