/**
 * @file Representations/Infrastructure/LEDRequest.h
 *
 * This file contains the LEDRequest struct.
 *
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#include <array>

/**
 * This describes a LEDRequest
 */
STREAMABLE(LEDRequest,
  ENUM(EyeLED,
    eye0Deg,
    eye45Deg,
    eye90Deg,
    eye135Deg,
    eye180Deg,
    eye225Deg,
    eye270Deg,
    eye315Deg
  );

  ENUM(EarLED,
    ear0Deg,
    ear36Deg,
    ear72Deg,
    ear108Deg,
    ear144Deg,
    ear180Deg,
    ear216Deg,
    ear252Deg,
    ear288Deg,
    ear324Deg
  );

  ENUM(HeadLED,
    rearLeft0,
    rearLeft1,
    rearLeft2,
    rearRight0,
    rearRight1,
    rearRight2,
    middleRight0,
    frontRight0,
    frontRight1,
    frontLeft0,
    frontLeft1,
    middleLeft0
  );

  STREAMABLE(RGBLED, PROTECT(
    constexpr RGBLED() = default;
    constexpr RGBLED(float r, float g, float b) : r(r), g(g), b(b) {}
    static RGBLED fromHSV(short h, float s = 1.f, float v = 1.f);

    operator std::array<float, 3>&() { return reinterpret_cast<std::array<float, 3>&>(r); }
    operator const std::array<float, 3>&() const { return reinterpret_cast<const std::array<float, 3>&>(r); }

    float& operator[](size_t i) { return reinterpret_cast<std::array<float, 3>&>(r)[i]; }
    const float& operator[](size_t i) const { return reinterpret_cast<const std::array<float, 3>&>(r)[i]; }

    RGBLED& operator*=(float scale) { r *= scale; g *= scale; b *= scale; return *this; }
    RGBLED operator*(float scale) const { return RGBLED(*this) *= scale; }

    RGBLED& operator+=(const RGBLED& other) { r += other.r; g += other.g; b += other.b; return *this; }
    RGBLED operator+(const RGBLED& other) const { return RGBLED(*this) += other; }
    
    static RGBLED white, black, red, green, blue, yellow, cyan, magenta, orange, violet;
    ),
    (float)(0.f) r,
    (float)(0.f) g,
    (float)(0.f) b
  );

  void setBothEyes(size_t i, RGBLED val, bool mirror = false)
  {
    leftEye[i] = val;

    if (mirror && i > 0)
      i = rightEye.size() - i;

    rightEye[i] = val;
  }

  ,
  (std::array<float, HeadLED::numOfHeadLEDs>)({0.f}) head,
  (std::array<RGBLED, EyeLED::numOfEyeLEDs>) leftEye,
  (std::array<RGBLED, EyeLED::numOfEyeLEDs>) rightEye,
  (std::array<float, EarLED::numOfEarLEDs>)({0.f}) leftEar,
  (std::array<float, EarLED::numOfEarLEDs>)({0.f}) rightEar,
  (RGBLED) chest,
  (RGBLED) leftFoot,
  (RGBLED) rightFoot
);
