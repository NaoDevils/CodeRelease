/**
  * SonarDetector to detect near obstacles in front of the robot with the sonar sensors
  */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/SensorData/SonarSensorData.h"
#include "Tools/RingBuffer.h"
#include "Tools/Math/Pose2f.h"
#include "Representations/Configuration/SonarConfiguration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Perception/SonarPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/FallDownState.h"

MODULE(SonarDetector,
  REQUIRES(SonarSensorData),
  REQUIRES(SonarConfiguration),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotPose),
  REQUIRES(FallDownState),
  PROVIDES(SonarPercept),

  LOADS_PARAMETERS( 
    ENUM(SonarFilter,
      movingAverage,
      exponential,
      minMaxExponential,
      kalman
    );
    STREAMABLE(KalmanParameters,
      ,
      (float)(10)R,
      (float)(1)H,
      (float)(10)Q,
      (float)(0)P,
      (float)(0)K,
      (float)(0)filteredMeasurement
    ),
    /* Selected filter method */
    (SonarFilter)(movingAverage) filterMode,
    /* Maximum distance to merge left and right SonarEstimate */
    (unsigned)(100) maxMergeDistance,
    /* Minimum validity to add SonarEstimate to SonarPercept */
    (float)(0.1f) minValidity,
    /* Exponent for exponential filter */
    (float)(0.2f) exponent,
    /* Exponent for minmax exponential filter */
    (float)(0.2f) minMaxExponent,
    /* Parameters for kalman filter */
    (KalmanParameters) leftKalmanParameters,
    (KalmanParameters) rightKalmanParameters,
    /* Select if sonar plots should be drawn */
    (bool)(true) drawPlots
  )
);

class SonarDetector : public SonarDetectorBase
{
public:
  /**
   * @brief Updates the SonarPercept with values from SonarSensorData.
  */
  void update(SonarPercept& theSonarPercept);

private:
  /* Attributes to track if sensor has measured new data */
  struct SensorAttributes
  {
    float lastMeasurementMm = 0;
    unsigned lastTimestamp = 0;
    SonarEstimate currentSonarEstimate;
  } leftSensorAttributes, rightSensorAttributes;

  /* Attributes for exponential filter */
  float lastLeftExponentialMeasurement = 0;
  float lastRightExponentialMeasurement = 0;

  /* Attributes for minmax exponential filter */
  struct MinMaxExponentialAttributes
  {
    float lastMeasurement = 0;
    float lastValidity = 0;
  } leftMinMaxExpoAttrs, rightMinMaxExpoAttrs;

  /* Attributes for moving average filter */
  static constexpr unsigned ringBufferSize = 10;
  RingBuffer<float, ringBufferSize> leftMeasurementRingBuffer;
  RingBuffer<float, ringBufferSize> rightMeasurementRingBuffer;

  /**
   * @brief Declares plots of filtered sensor data
  */
  void declarePlots();

  /**
   * @brief Clears all SonarEstimates in the SonarPercept
  */
  void clearSonarPercept(SonarPercept& theSonarPercept);

  /**
   * @brief Resets all values used for sensor filtering
  */
  void resetFilterValues();

  /**
   * @brief Checks if the sensor has measured new data
   * @return true if sensor has new data, false otherwise
  */
  bool sensorHasNewData(const float& sensorMeasurement, const float& lastMeasurement, const unsigned& lastTimestamp);

  /**
   * @brief Adds the current measurement to the ring buffer and fills the filtered value to the sonar percept
   * @param sensorMeasurement: current sensor measurement
   * @param ringBuffer: ring buffer containing measurements
   * @param sonarEstimate: sonar estimate that should be filled
  */
  void updateSonarValue(
      const float& sensorMeasurement, RingBuffer<float, ringBufferSize>& ringBuffer, const Angle& sensorAngle, const Vector2f& sonarOffset, const bool leftSensor, SonarEstimate& sonarEstimate);

  /**
   * @brief Draws graphs to visualize the filtered sonar sensor measurements. Can be used to fine tune filter parameters
  */
  void drawGraphs(RingBuffer<float, ringBufferSize>& ringBuffer, float sensorMeasurement, bool validMeasurement, bool leftSensor);

  /**
   * @brief Applies a moving average filter on sonar measurements
   * @return a tuple of the filtered distance and validity
  */
  std::tuple<float, float> movingAverageFilter(RingBuffer<float, ringBufferSize>& ringBuffer);

  /**
   * @brief Applies an exponential filter on sonar measurements
   * @return a tuple of the filtered distance and validity
  */
  std::tuple<float, float> exponentialFilter(float sensorMeasurement, float& lastExponentialMeasurement);

  /**
   * @brief Applies an exponential filter with additional min-max filtering on sonar measurements
   * @return a tuple of the filtered distance and validity
  */
  std::tuple<float, float> minMaxExponentialFilter(float sensorMeasurement, MinMaxExponentialAttributes& minMaxExpoAttrs);

  /**
   * @brief Applies a kalman filter on sonar measurements
   * @return a tuple of the filtered distance and validity
  */
  std::tuple<float, float> kalmanFilter(float sonarSensorMeasurement, KalmanParameters& kalmanParameters);

  /**
   * @brief Checks if the raw measurement is valid.
   * A measurement is valid if it lies between the minimum and maximum detection range
   * @return true when measurement is valid, false otherwise
  */
  bool isValidMeasurement(const float& sensorMeasurement);

  /**
   * @brief Checks if a SonarEstimate is valid.
   * A SonarEstimate is valid if its measured distance is valid and validity is above minValidity.
   * The SonarEstimate's distance also contains the sonarOffset. To get the actual measured distance, we need to
   * subtract this value for the valid measurement check.
   * @return true when SonarEstimate is valid, false otherwise
  */
  bool isValidSonarEstimate(const SonarEstimate& sonarEstimate);

  /**
   * @brief Checks if the left and right SonarEstimate should be merged.
   * If yes, it fills the mergedSonarEstimate.
   * @return true if the SonarEstimates were merged, false otherwise
  */
  bool mergeSonarEstimates(const SonarEstimate& leftSonarEstimate, const SonarEstimate& rightSonarEstimate, SonarEstimate& mergedSonarEstimate);
};
