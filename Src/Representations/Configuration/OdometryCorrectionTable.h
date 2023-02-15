#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Pose2f.h"
#include <string>

STREAMABLE(OdometryCorrectionTable,
  STREAMABLE(Entry,,
    (float)(400.f) speed,
    (float)(1.f) multiplier
  ),
  (std::vector<Entry>) odometryTable
);

STREAMABLE(OdometryCorrectionTable2D,
  STREAMABLE(Entry,,
    (float)(0.f) speedX,
    (float)(0.f) speedR,
    (float)(1.f) multiplier
  ),
  (std::vector<Entry>) odometryTable
);

STREAMABLE(OdometryCorrectionTables,,
  (OdometryCorrectionTable) backCorrectionTablePreview,
  (OdometryCorrectionTable) forwardCorrectionTablePreview,
  (OdometryCorrectionTable) sideCorrectionTablePreview,
  (OdometryCorrectionTable) backCorrectionTable,
  (OdometryCorrectionTable) forwardCorrectionTable,
  (OdometryCorrectionTable) sideCorrectionTable,
  (OdometryCorrectionTable) rotCorrectionTable,
  (OdometryCorrectionTable2D) rot2DCorrectionTable
);

class OdometryCorrection
{
public:
  static float correct1D(const OdometryCorrectionTable& table, const float& odometryAbs, const float& absExecutedSpeed)
  {
    float result = 1.f;
    if (odometryAbs < 0.001f)
      return result;
    const int entries = static_cast<int>(table.odometryTable.size());
    int entry = 0, lastEntry = 0;
    while (entry < (entries - 1) && absExecutedSpeed >= table.odometryTable[entry].speed)
      entry++;
    lastEntry = std::max(0, entry - 1);


    float regStartSpeed;
    float regEndSpeed;
    float speedDiff;
    regStartSpeed = table.odometryTable[lastEntry].speed;
    regEndSpeed = table.odometryTable[entry].speed;
    speedDiff = std::abs(regEndSpeed - regStartSpeed);
    if (speedDiff > 0)
    {
      float factor = std::min<float>((std::abs(regEndSpeed - absExecutedSpeed)) / speedDiff, 1.f);
      result = table.odometryTable[lastEntry].multiplier * (factor) + table.odometryTable[entry].multiplier * (1 - factor);
    }
    else
      result = table.odometryTable[lastEntry].multiplier;
    return result;
  }

  static float correct2D(const OdometryCorrectionTable2D& table, const float& odometryAbs, const float& absExecutedSpeed, const float& absExecutedRot)
  {
    float result = 1.f;
    if (odometryAbs < 0.001f)
      return result;
    const int entries = static_cast<int>(table.odometryTable.size());
    int entry = 0;
    int entryLowSpeed = 0;
    int entryHighSpeed = 0;
    float regStartSpeedX = 0;
    float regEndSpeedX = 0;
    float speedDiffX = 0;

    while (entry < (entries - 1) && absExecutedSpeed >= table.odometryTable[entry].speedX)
      entry++;
    entry = std::max(entry - 1, 0);
    entryLowSpeed = entry;
    while (entryLowSpeed > 0 && absExecutedRot <= table.odometryTable[entryLowSpeed].speedR)
      entryLowSpeed--;
    entryHighSpeed = std::min(entries - 1, entry + 1);
    while (entryHighSpeed < (entries - 1) && absExecutedRot <= table.odometryTable[entryHighSpeed].speedR)
      entryHighSpeed++;

    regStartSpeedX = table.odometryTable[entryLowSpeed].speedX;
    regEndSpeedX = table.odometryTable[entryHighSpeed].speedX;
    speedDiffX = std::abs(regEndSpeedX - regStartSpeedX);
    float rFactor = 1.f;
    if (speedDiffX > 0)
      rFactor = (std::abs(regEndSpeedX - absExecutedSpeed)) / speedDiffX;
    else
      rFactor = table.odometryTable[entryLowSpeed].multiplier;
    float factor = std::min(rFactor, 1.f);
    result = table.odometryTable[entryLowSpeed].multiplier * (factor) + table.odometryTable[entryHighSpeed].multiplier * (1 - factor);

    return result;
  }

  static Pose2f correctPreview(
      const Pose2f& speed, const Pose2f& odometry, const OdometryCorrectionTable& backCorrectionPreview, const OdometryCorrectionTable& forwardCorrectionPreview, const OdometryCorrectionTable& sideCorrectionPreview)
  {
    float xCorrectionFactor, yCorrectionFactor, rotCorrectionFactor;
    xCorrectionFactor = yCorrectionFactor = rotCorrectionFactor = 1.f;
    Pose2f correctedOdometry;
    //OdometryCorrection
    if (speed.translation.x() < 0)
    {
      xCorrectionFactor = correct1D(backCorrectionPreview, std::abs(odometry.translation.x()), std::abs(speed.translation.x()));
    }
    else if (speed.translation.x() > 0)
    {
      xCorrectionFactor = correct1D(forwardCorrectionPreview, std::abs(odometry.translation.x()), std::abs(speed.translation.x()));
    }
    if (speed.translation.y() != 0)
    {
      yCorrectionFactor = correct1D(sideCorrectionPreview, std::abs(odometry.translation.y()), std::abs(speed.translation.y()));
    }

    correctedOdometry.translation.x() = odometry.translation.x() * xCorrectionFactor;
    correctedOdometry.translation.y() = odometry.translation.y() * yCorrectionFactor;
    correctedOdometry.rotation = odometry.rotation * rotCorrectionFactor;
    return correctedOdometry;
  }

  static Pose2f correct(const Pose2f& speed,
      const Pose2f& odometry,
      const OdometryCorrectionTable& backCorrection,
      const OdometryCorrectionTable& forwardCorrection,
      const OdometryCorrectionTable& sideCorrection,
      const OdometryCorrectionTable& rotCorrection,
      const OdometryCorrectionTable2D& rot2DCorrection,
      const bool predict = false)
  {
    float xCorrectionFactor, yCorrectionFactor, rotCorrectionFactor;
    xCorrectionFactor = yCorrectionFactor = rotCorrectionFactor = 1.f;
    Pose2f correctedOdometry;
    //OdometryCorrection
    if (speed.translation.x() < 0)
    {
      xCorrectionFactor = correct1D(backCorrection, std::abs(odometry.translation.x()), std::abs(speed.translation.x()));
    }
    else if (speed.translation.x() > 0)
    {
      xCorrectionFactor = correct1D(forwardCorrection, std::abs(odometry.translation.x()), std::abs(speed.translation.x()));
    }
    if (speed.translation.y() != 0)
    {
      yCorrectionFactor = correct1D(sideCorrection, std::abs(odometry.translation.y()), std::abs(speed.translation.y()));
    }
    if (speed.translation.x() == 0 && speed.rotation != 0)
    {
      rotCorrectionFactor = correct1D(rotCorrection, std::abs(odometry.rotation), std::abs(speed.rotation));
    }
    else if (speed.rotation != 0)
    {
      rotCorrectionFactor = correct2D(rot2DCorrection, std::abs(odometry.rotation), std::abs(speed.translation.x()), std::abs(speed.rotation));
    }

    if (predict)
    {
      correctedOdometry.translation.x() = odometry.translation.x() / xCorrectionFactor;
      correctedOdometry.translation.y() = odometry.translation.y() / yCorrectionFactor;
      correctedOdometry.rotation = odometry.rotation / rotCorrectionFactor;
    }
    else
    {
      correctedOdometry.translation.x() = odometry.translation.x() * xCorrectionFactor;
      correctedOdometry.translation.y() = odometry.translation.y() * yCorrectionFactor;
      correctedOdometry.rotation = odometry.rotation * rotCorrectionFactor;
    }
    return correctedOdometry;
  }
};
