/**
* @file FieldColor.h
*
* Dynamic FieldColor detection for IP without color table
*
* @author <a href="mailto:ingmar.schwarz@uni-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(FieldColors,
  STREAMABLE(FieldColor,
    inline bool isPixelFieldColor(const int &y, const int &cb, const int &cr) const
    {
      return std::abs(y - fieldColorOptY) < 65 &&
        std::abs(cb - fieldColorOptCb) < fieldColorMaxDistCb &&
        std::abs(cr - fieldColorOptCr) < fieldColorMaxDistCr;
    }

    inline bool isPixelBallColor(const int &y, const int &cb, const int &cr) const
    {
      return cr > 150 && cb < 150;
    }
    FieldColor& operator=(const FieldColor &other)
    {
      if (this == &other)
        return *this;
      fieldColorOptY = other.fieldColorOptY;
      fieldColorOptCb = other.fieldColorOptCb;
      fieldColorOptCr = other.fieldColorOptCr;
      fieldColorMaxDistCr = other.fieldColorMaxDistCr;
      fieldColorMaxDistCb = other.fieldColorMaxDistCb;
      fieldColorMaxYDiff = other.fieldColorMaxYDiff;
      fieldColorYDiffFactor = other.fieldColorYDiffFactor;
      fieldColorOptCbCrRatio = other.fieldColorOptCbCrRatio;
      fieldColorMaxCbCrRatioDiff = other.fieldColorMaxCbCrRatioDiff;
      fieldColorCbCrFactor = other.fieldColorCbCrFactor;
      lineToFieldColorYThreshold = other.lineToFieldColorYThreshold;
      maxFieldColorY = other.maxFieldColorY;
      return *this;
    },
    (int)(100) fieldColorOptY,
    (int)(80) fieldColorOptCb,
    (int)(100) fieldColorOptCr,
    (int)(15) fieldColorMaxDistCb,
    (int)(10) fieldColorMaxDistCr,
    (int)(65) fieldColorMaxYDiff,
    (int)(1300) fieldColorOptCbCrRatio,
    (int)(400) fieldColorMaxCbCrRatioDiff,
    (int)(10) fieldColorCbCrFactor,
    (int)(40) lineToFieldColorYThreshold,
    (int)(150) maxFieldColorY,
    (float)(1.5f) fieldColorYDiffFactor
  );
  FieldColors& operator=(const FieldColors &other)
  {
    if (this == &other)
      return *this;
    for (int i = 0; i < 10; i++)
      fieldColorArray[i] = other.fieldColorArray[i];
    horizonYAvg = other.horizonYAvg;
    areaHeight = other.areaHeight;
    areaWidth = other.areaWidth;
    return *this;
  }
  inline bool isPixelFieldColorInArea(const int &y, const int &cb, const int &cr, const int &xPos, const int &yPos) const
  {
    // TODO: interpolate?
    // TODO: better to not call this above horizon, using std::abs for now for debugging
    int areaNo = 1 + 3 * (std::abs(yPos - horizonYAvg) / areaHeight) + (xPos /areaWidth);
    return std::abs(y - fieldColorArray[areaNo].fieldColorOptY) < 65 &&
      std::abs(cb - fieldColorArray[areaNo].fieldColorOptCb) < fieldColorArray[areaNo].fieldColorMaxDistCb &&
      std::abs(cr - fieldColorArray[areaNo].fieldColorOptCr) < fieldColorArray[areaNo].fieldColorMaxDistCr;
  }
  inline bool isPixelFieldColor(const int &y, const int &cb, const int &cr) const
  {
    return std::abs(y - fieldColorArray[0].fieldColorOptY) < 65 &&
      std::abs(cb - fieldColorArray[0].fieldColorOptCb) < fieldColorArray[0].fieldColorMaxDistCb &&
      std::abs(cr - fieldColorArray[0].fieldColorOptCr) < fieldColorArray[0].fieldColorMaxDistCr;
  },
  (FieldColor[10]) fieldColorArray, /**< [0] refers to whole image, [1] to [9] to areas on image */
  (int)(4) horizonYAvg,
  (int)(104) areaWidth,
  (int)(77) areaHeight
);

struct FieldColorsUpper : public FieldColors
{
};
