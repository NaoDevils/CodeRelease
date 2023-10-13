/**
 * @file ColorModelConversions.h
 *
 * Declaration and implementation of class ColorModelConversions
 */

#pragma once

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include "Tools/Math/BHMath.h"

/**
 * A class that defines static conversions between color models for single pixels.
 */
class ColorModelConversions
{
public:
  /** Converts an YCbCr pixel into an RGB pixel.
   *  @param Y The Y channel of the source pixel.
   *  @param Cb The Cb channel of the source pixel.
   *  @param Cr The Cr channel of the source pixel.
   *  @param R The R channel of the target pixel.
   *  @param G The G channel of the target pixel.
   *  @param B The B channel of the target pixel.
   */
  static void fromYCbCrToRGB(unsigned char Y, unsigned char Cb, unsigned char Cr, unsigned char& R, unsigned char& G, unsigned char& B)
  {
    int r = Y + ((1436 * (Cr - 128)) >> 10), g = Y - ((354 * (Cb - 128) + 732 * (Cr - 128)) >> 10), b = Y + ((1814 * (Cb - 128)) >> 10);
    if (r < 0)
      r = 0;
    else if (r > 255)
      r = 255;
    if (g < 0)
      g = 0;
    else if (g > 255)
      g = 255;
    if (b < 0)
      b = 0;
    else if (b > 255)
      b = 255;
    R = static_cast<unsigned char>(r);
    G = static_cast<unsigned char>(g);
    B = static_cast<unsigned char>(b);
  }

  /** Converts an YCbCr pixel into an RGB pixel.
   *  @param Y The Y channel of the source pixel.
   *  @param Cb The Cb channel of the source pixel.
   *  @param Cr The Cr channel of the source pixel.
   *  @param R The R channel of the target pixel.
   *  @param G The G channel of the target pixel.
   *  @param B The B channel of the target pixel.
   */
  static void fromYCbCrToRGB(unsigned char Y, unsigned char Cb, unsigned char Cr, float& R, float& G, float& B)
  {
    unsigned char charR, charG, charB;
    fromYCbCrToRGB(Y, Cb, Cr, charR, charG, charB);
    R = charR / 255.f;
    G = charG / 255.f;
    B = charB / 255.f;
  }

  /** Converts an RGB pixel into an YCbCr pixel.
   *  @param R The R channel of the source pixel.
   *  @param G The G channel of the source pixel.
   *  @param B The B channel of the source pixel.
   *  @param Y The Y channel of the target pixel.
   *  @param Cb The Cb channel of the target pixel.
   *  @param Cr The Cr channel of the target pixel.
   */
  static void fromRGBToYCbCr(unsigned char R, unsigned char G, unsigned char B, unsigned char& Y, unsigned char& Cb, unsigned char& Cr)
  {
    int y = (int)(0.2990 * R + 0.5870 * G + 0.1140 * B), cb = 127 + (int)(-0.1687 * R - 0.3313 * G + 0.5000 * B), cr = 127 + (int)(0.5000 * R - 0.4187 * G - 0.0813 * B);
    if (y < 0)
      y = 0;
    else if (y > 255)
      y = 255;
    if (cb < 0)
      cb = 0;
    else if (cb > 255)
      cb = 255;
    if (cr < 0)
      cr = 0;
    else if (cr > 255)
      cr = 255;
    Y = static_cast<unsigned char>(y);
    Cb = static_cast<unsigned char>(cb);
    Cr = static_cast<unsigned char>(cr);
  }

  /** Converts an YCbCr pixel into an HSI pixel.
   *  @param Y The Y channel of the source pixel.
   *  @param Cb The Cb channel of the source pixel.
   *  @param Cr The Cr channel of the source pixel.
   *  @param H The H channel of the target pixel.
   *  @param S The S channel of the target pixel.
   *  @param I The I channel of the target pixel.
   */
  static void fromYCbCrToHSI(unsigned char Y, unsigned char Cb, unsigned char Cr, unsigned char& H, unsigned char& S, unsigned char& I)
  {
    int r = Y + ((1436 * (Cr - 128)) >> 10);
    int g = Y - ((354 * (Cb - 128) + 732 * (Cr - 128)) >> 10);
    int b = Y + ((1814 * (Cb - 128)) >> 10);
    int k = 0;

    if (g < b)
    {
      std::swap(g, b);
      k = -256;
    }

    if (r < g)
    {
      std::swap(r, g);
      k = -85 - k;
    }

    int chroma = r - std::min(g, b);
    int h = std::abs(k + ((g - b) << 8) / (chroma ? 6 * chroma : 1));
    int s = (chroma << 8) / (r ? r : 1);

    // normalize values to their boundaries
    H = static_cast<unsigned char>(std::min(255, h));
    S = static_cast<unsigned char>(std::max(0, std::min(255, s)));
    I = static_cast<unsigned char>(std::max(0, std::min(255, r)));
  }

  /** Converts an HSI pixel into an YCbCr pixel.
   *  @param H The H channel of the source pixel.
   *  @param S The S channel of the source pixel.
   *  @param I The I channel of the source pixel.
   *  @param Y The Y channel of the target pixel.
   *  @param Cb The Cb channel of the target pixel.
   *  @param Cr The Cr channel of the target pixel.
   */
  static void fromHSIToYCbCr(unsigned char H, unsigned char S, unsigned char I, unsigned char& Y, unsigned char& Cb, unsigned char& Cr)
  {
    float h = 1.0f - H / 255.0f, s = S / 255.0f, r, g, b;

    if (h < 1.0f / 3.0f)
    {
      g = (1 - s) / 3;
      b = (1 + s * std::cos(pi2 * h) / std::cos(pi2 * (1.0f / 6.0f - h))) / 3.0f;
      r = 1 - (g + b);
    }
    else if (h < 2.0f / 3.0f)
    {
      h -= 1.0f / 3.0f;
      b = (1 - s) / 3;
      r = (1 + s * std::cos(pi2 * h) / std::cos(pi2 * (1.0f / 6.0f - h))) / 3.0f;
      g = 1 - (b + r);
    }
    else
    {
      h -= 2.0f / 3.0f;
      r = (1 - s) / 3;
      g = (1 + s * std::cos(pi2 * h) / std::cos(pi2 * (1.0f / 6.0f - h))) / 3.0f;
      b = 1 - (r + g);
    }

    r *= I * 3;
    g *= I * 3;
    b *= I * 3;
    if (r > 255)
      r = 255;
    if (g > 255)
      g = 255;
    if (b > 255)
      b = 255;

    fromRGBToYCbCr(static_cast<unsigned char>(r), static_cast<unsigned char>(g), static_cast<unsigned char>(b), Y, Cb, Cr);
  }

  /**
   * @brief Converts a YCbCr Pixel into an HSV Pixel. Uses the fromYCbCrToRGB method, inaccuracy follows
   * @param Y The Y channel of the source pixel.
   * @param Cb The Cb channel of the source pixel.
   * @param Cr The Cr channel of the source pixel.
   * @param H The H channel of the target pixel.
   * @param S The S channel of the target pixel.
   * @param V The V channel of the target pixel.
   */
  static void fromYCbCrToHSV(unsigned char Y, unsigned char Cb, unsigned char Cr, short int& H, float& S, float& V)
  {
    unsigned char r, g, b;
    fromYCbCrToRGB(Y, Cb, Cr, r, g, b);
    fromRGBToHSV(r, g, b, H, S, V);
  }

  /**
   * @brief Converts a RGB Pixel into an HSV Pixel.
   * @param R The R channel of the source pixel in interval [0, 255].
   * @param G The G channel of the source pixel in interval [0, 255].
   * @param B The B channel of the source pixel in interval [0, 255].
   * @param H The H channel of the target pixel in interval [0°, 360°].
   * @param S The S channel of the target pixel in interval [0, 1].
   * @param V The V channel of the target pixel in interval [0, 1].
   */
  static void fromRGBToHSV(unsigned char R, unsigned char G, unsigned char B, short int& H, float& S, float& V)
  {
    float r = static_cast<float>(R) / 255.0f, g = static_cast<float>(G) / 255.0f, b = static_cast<float>(B) / 255.0f;
    float rgbMax = std::max(r, std::max(g, b));
    float rgbMin = std::min(r, std::min(g, b));
    float C = rgbMax - rgbMin;
    // H
    if (rgbMax == rgbMin)
    {
      H = 0;
    }
    else
    {
      if (rgbMax == r)
      {
        // in this case H can become below zero -> add 360, so that H stays in interval [0, 360]
        H = (static_cast<short int>(std::round(60.f * ((g - b) / C))) + 360) % 360;
      }
      else if (rgbMax == g)
      {
        H = static_cast<short int>(std::round(60.f * (((b - r) / C) + 2.f)));
      }
      else
      {
        H = static_cast<short int>(std::round(60.f * (((r - g) / C) + 4.f)));
      }
    }
    // S
    S = rgbMax == 0 ? 0 : C / rgbMax;
    // V
    V = rgbMax;
  }

  /**
   * @brief Converts a HSV Pixel into an RGB Pixel.
   * @param H The H channel of the source pixel in interval [0°, 360°].
   * @param S The S channel of the source pixel in interval [0, 1].
   * @param V The V channel of the source pixel in interval [0, 1].
   * @param R The R channel of the target pixel in interval [0, 255].
   * @param G The G channel of the target pixel in interval [0, 255].
   * @param B The B channel of the target pixel in interval [0, 255].
   */
  static void fromHSVToRGB(short int H, float S, float V, unsigned char& R, unsigned char& G, unsigned char& B)
  {
    float C = S * V;
    float X = C * (1 - std::abs(fmodf(static_cast<float>(H) / 60.0f, 2) - 1));
    float m = V - C;
    float r, g, b;
    if (H < 60)
    {
      r = C, g = X, b = 0;
    }
    else if (H < 120)
    {
      r = X, g = C, b = 0;
    }
    else if (H < 180)
    {
      r = 0, g = C, b = X;
    }
    else if (H < 240)
    {
      r = 0, g = X, b = C;
    }
    else if (H < 300)
    {
      r = X, g = 0, b = C;
    }
    else
    {
      r = C, g = 0, b = X;
    }
    R = static_cast<unsigned char>(std::round((r + m) * 255));
    G = static_cast<unsigned char>(std::round((g + m) * 255));
    B = static_cast<unsigned char>(std::round((b + m) * 255));
  }

  /**
   * @brief Converts a YCbCr Pixel into an HSL Pixel. Uses the fromYCbCrToRGB method, inaccuracy follows
   * @param Y The Y channel of the source pixel.
   * @param Cb The Cb channel of the source pixel.
   * @param Cr The Cr channel of the source pixel.
   * @param H The H channel of the target pixel.
   * @param S The S channel of the target pixel.
   * @param L The L channel of the target pixel.
   */
  static void fromYCbCrToHSL(unsigned char Y, unsigned char Cb, unsigned char Cr, short int& H, float& S, float& L)
  {
    unsigned char r, g, b;
    fromYCbCrToRGB(Y, Cb, Cr, r, g, b);
    fromRGBToHSL(r, g, b, H, S, L);
  }

  /**
   * @brief Converts a HSL Pixel into an RGB Pixel.
   * @param H The H channel of the source pixel in interval [0°, 360°].
   * @param S The S channel of the source pixel in interval [0, 1].
   * @param L The L channel of the source pixel in interval [0, 1].
   * @param R The R channel of the target pixel in interval [0, 255].
   * @param G The G channel of the target pixel in interval [0, 255].
   * @param B The B channel of the target pixel in interval [0, 255].
   */
  static void fromHSLToRGB(short int H, float S, float L, unsigned char& R, unsigned char& G, unsigned char& B)
  {
    float C = (1 - std::abs(2 * L - 1)) * S;
    float X = C * (1 - std::abs(fmodf(static_cast<float>(H) / 60.0f, 2) - 1));
    float r, g, b;
    if (H < 60)
    {
      r = C, g = X, b = 0;
    }
    else if (H < 120)
    {
      r = X, g = C, b = 0;
    }
    else if (H < 180)
    {
      r = 0, g = C, b = X;
    }
    else if (H < 240)
    {
      r = 0, g = X, b = C;
    }
    else if (H < 300)
    {
      r = X, g = 0, b = C;
    }
    else
    {
      r = C, g = 0, b = X;
    }
    float m = L - C / 2;
    R = static_cast<unsigned char>(std::round((r + m) * 255));
    G = static_cast<unsigned char>(std::round((g + m) * 255));
    B = static_cast<unsigned char>(std::round((b + m) * 255));
  }

  /**
  * @brief Converts a RGB Pixel into an HSL Pixel.
  * @param R The R channel of the source pixel in interval [0, 255].
  * @param G The G channel of the source pixel in interval [0, 255].
  * @param B The B channel of the source pixel in interval [0, 255].
  * @param H The H channel of the target pixel in interval [0°, 360°].
  * @param S The S channel of the target pixel in interval [0, 1].
  * @param L The L channel of the target pixel in interval [0, 1].
  */
  static void fromRGBToHSL(unsigned char R, unsigned char G, unsigned char B, short int& H, float& S, float& L)
  {
    float r = static_cast<float>(R) / 255.0f, g = static_cast<float>(G) / 255.0f, b = static_cast<float>(B) / 255.0f;
    float rgbMax = std::max(r, std::max(g, b));
    float rgbMin = std::min(r, std::min(g, b));
    // L
    L = (rgbMax + rgbMin) / 2.0f;
    // H
    if (rgbMax == rgbMin)
    {
      H = 0;
    }
    else
    {
      float C = rgbMax - rgbMin;
      if (rgbMax == r)
      {
        // in this case H can become below zero -> add 360, so that H stays in interval [0, 360]
        H = (static_cast<short int>(std::round(60.f * ((g - b) / C))) + 360) % 360;
      }
      else if (rgbMax == g)
      {
        H = static_cast<short int>(std::round(60.f * (((b - r) / C) + 2.f)));
      }
      else
      {
        H = static_cast<short int>(std::round(60.f * (((r - g) / C) + 4.f)));
      }
    }
    // S
    if (L == 0 || L == 1)
    {
      S = 0;
    }
    else
    {
      S = (rgbMax - L) / std::min(L, 1 - L);
    }
  }
};
