//
// Created by simon on 03.01.18.
//

#ifndef SSE_SOBEL_IMPLEMENTATIONS_HPP
#define SSE_SOBEL_IMPLEMENTATIONS_HPP

#include "Representations/Infrastructure/SobelImage.h"

namespace SobelImplementations
{
  static inline __m128i _mm_absdiff_epu8(__m128i x, __m128i y)
  {
    return _mm_or_si128(_mm_subs_epu8(x, y), _mm_subs_epu8(y, x));
  }

  static inline __m128i _mm_mysrli_epi8(__m128i a, unsigned char imm)
  {
    return _mm_and_si128(_mm_set1_epi8(0xff >> imm), _mm_srli_epi32(a, imm));
  }

  template<unsigned int R, bool A>
  static inline __m128i deinterleave(unsigned char *p)
  {
    ASSERT(R == 1 || R == 2 || R == 4);

    if (R == 1)
    {
      const __m128i mask = _mm_set1_epi16(0x00ff);
      __m128i loadA = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      __m128i loadB = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p + 16)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p + 16));

      loadA = _mm_and_si128(loadA, mask);
      loadB = _mm_and_si128(loadB, mask);

      __m128i rt = _mm_packus_epi16(loadA, loadB);
      return rt;
    }
    else if (R == 2)
    {
      const __m128i mask = _mm_set1_epi32(0x000000ff);
      __m128i loadA = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      __m128i loadB = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p + 16)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p+16));
      __m128i loadC = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p + 32)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p+32));
      __m128i loadD = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p + 48)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p+48));

      loadA = _mm_and_si128(loadA, mask);
      loadB = _mm_and_si128(loadB, mask);
      loadC = _mm_and_si128(loadC, mask);
      loadD = _mm_and_si128(loadD, mask);

      __m128i packAB = _mm_packus_epi16(loadA, loadB);
      __m128i packCD = _mm_packus_epi16(loadC, loadD);

      __m128i rt = _mm_packus_epi16(packAB, packCD);
      return rt;
    }
    else if (R == 4)
    {
      const __m128i mask = _mm_set_epi32(0x00000000, 0x000000ff, 0x00000000, 0x000000ff); // little endian

      __m128i loadA = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p));
      __m128i loadB = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p + 16)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p + 16));
      __m128i loadC = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p + 32)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p + 32));
      __m128i loadD = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p + 48)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p + 48));
      __m128i loadE = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p + 64)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p + 64));
      __m128i loadF = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p + 80)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p + 80));
      __m128i loadG = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p + 96)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p + 96));
      __m128i loadH = A ? _mm_load_si128(reinterpret_cast<__m128i *>(p + 112)) :
                      _mm_loadu_si128(reinterpret_cast<__m128i *>(p + 112));

      loadA = _mm_and_si128(loadA, mask);
      loadB = _mm_and_si128(loadB, mask);
      loadC = _mm_and_si128(loadC, mask);
      loadD = _mm_and_si128(loadD, mask);
      loadE = _mm_and_si128(loadE, mask);
      loadF = _mm_and_si128(loadF, mask);
      loadG = _mm_and_si128(loadG, mask);
      loadH = _mm_and_si128(loadH, mask);

      __m128i packAB = _mm_packus_epi16(loadA, loadB);
      __m128i packCD = _mm_packus_epi16(loadC, loadD);
      __m128i packEF = _mm_packus_epi16(loadE, loadF);
      __m128i packGH = _mm_packus_epi16(loadG, loadH);

      __m128i packABCD = _mm_packus_epi16(packAB, packCD);
      __m128i packEFGH = _mm_packus_epi16(packEF, packGH);

      __m128i rt = _mm_packus_epi16(packABCD, packEFGH);
      return rt;
    }
  }

  template<unsigned char D>
  static inline void sobel(__m128i a, __m128i b, __m128i c, __m128i d, __m128i e, __m128i f, __m128i &gx,
                              __m128i &gy, __m128i &mag)
  {
    __m128i rt0 = a;
    __m128i rt1 = _mm_alignr_epi8(b, a, 1);
    __m128i rt2 = _mm_alignr_epi8(b, a, 2);

    __m128i rm0 = c;
    __m128i rm2 = _mm_alignr_epi8(d, c, 2);

    __m128i rb0 = e;
    __m128i rb1 = _mm_alignr_epi8(f, e, 1);
    __m128i rb2 = _mm_alignr_epi8(f, e, 2);

    rt0 = _mm_mysrli_epi8(rt0, 1); //shift every byte right 1 bit, shifting in 0
    rt2 = _mm_mysrli_epi8(rt2, 1);

    rb0 = _mm_mysrli_epi8(rb0, 1);
    rb2 = _mm_mysrli_epi8(rb2, 1);

    __m128i gx_, gy_, mag_;

    __m128i sum_pos = _mm_adds_epu8(rt0, _mm_adds_epu8(rm0, rb0));
    __m128i sum_neg = _mm_adds_epu8(rt2, _mm_adds_epu8(rm2, rb2));

    gx_ = _mm_absdiff_epu8(sum_pos, sum_neg);

    sum_pos = _mm_adds_epu8(rt0, _mm_adds_epu8(rt1, rt2));
    sum_neg = _mm_adds_epu8(rb0, _mm_adds_epu8(rb1, rb2));

    gy_ = _mm_absdiff_epu8(sum_pos, sum_neg);

    __m128i mins = _mm_min_epu8(gx_, gy_);
    __m128i maxs = _mm_max_epu8(gx_, gy_);
    mins = _mm_mysrli_epi8(mins, 2);
    mag_ = _mm_adds_epu8(mins, maxs);

    if (D == 1)
    {
      mag = mag_;
    }
    else if (D == 2)
    {
      gy = gy_;
    }
    else if (D == 3)
    {
      mag = mag_;
      gy = gy_;
    }
    else if (D == 4)
    {
      gx = gx_;
    }
    else if (D == 5)
    {
      mag = mag_;
      gx = gx_;
    }
    else if (D == 6)
    {
      gy = gy_;
      gx = gx_;
    }
    else if (D == 7)
    {
      mag = mag_;
      gy = gy_;
      gx = gx_;
    }
  }

  template<unsigned char D>
  static inline void sobel(__m128i rt, __m128i rm, __m128i rb, __m128i &gx, __m128i &gy, __m128i &mag)
  {
    __m128i rt0 = rt;
    __m128i rt1 = _mm_srli_si128(rt, 1); //shift complete vector right 1 byte, shifting in 0
    __m128i rt2 = _mm_srli_si128(rt, 2);

    __m128i rm0 = rm;
    __m128i rm2 = _mm_srli_si128(rm, 2);

    __m128i rb0 = rb;
    __m128i rb1 = _mm_srli_si128(rb, 1);
    __m128i rb2 = _mm_srli_si128(rb, 2);

    rt0 = _mm_mysrli_epi8(rt0, 1); //shift every byte right 2 bits, shifting in 0
    rt2 = _mm_mysrli_epi8(rt2, 1);

    rb0 = _mm_mysrli_epi8(rb0, 1);
    rb2 = _mm_mysrli_epi8(rb2, 1);

    __m128i gx_, gy_, mag_;

    __m128i sum_pos = _mm_adds_epu8(rt0, _mm_adds_epu8(rm0, rb0));
    __m128i sum_neg = _mm_adds_epu8(rt2, _mm_adds_epu8(rm2, rb2));

    gx_ = _mm_absdiff_epu8(sum_pos, sum_neg);

    sum_pos = _mm_adds_epu8(rt0, _mm_adds_epu8(rt1, rt2));
    sum_neg = _mm_adds_epu8(rb0, _mm_adds_epu8(rb1, rb2));

    gy_ = _mm_absdiff_epu8(sum_pos, sum_neg);

    __m128i mins = _mm_min_epu8(gx_, gy_);
    __m128i maxs = _mm_max_epu8(gx_, gy_);
    mins = _mm_mysrli_epi8(mins, 2);
    mag_ = _mm_adds_epu8(mins, maxs);

    if (D == 1)
    {
      mag = mag_;
    }
    else if (D == 2)
    {
      gy = gy_;
    }
    else if (D == 3)
    {
      mag = mag_;
      gy = gy_;
    }
    else if (D == 4)
    {
      gx = gx_;
    }
    else if (D == 5)
    {
      mag = mag_;
      gx = gx_;
    }
    else if (D == 6)
    {
      gy = gy_;
      gx = gx_;
    }
    else if (D == 7)
    {
      mag = mag_;
      gy = gy_;
      gx = gx_;
    }
  }

  template<unsigned int R, unsigned char D>
  void reference(const Image &image, SobelImage &sobelImage)
  {
    const unsigned int w_reduced = 2 * image.width / R;
    const unsigned int h_reduced = 2 * image.height / R;
    const unsigned int byte_width = 4 * image.width;

    unsigned char *image_ptr = reinterpret_cast<unsigned char *>(image.image);

    const unsigned int y_start = 1;
    const unsigned int y_end = h_reduced - 1;
    const unsigned int y_step = 1;

    const unsigned int x_start = 1;
    const unsigned int x_end = w_reduced - 1;
    const unsigned int x_step = 1;

    const unsigned int stride = 2 * R;
    const unsigned int vertical_stride = R * byte_width;

    for (unsigned int y = y_start; y < y_end; y += y_step)
    {
      for (unsigned int x = x_start; x < x_end; x += x_step)
      {
        const unsigned int y_pos = y * R * byte_width;
        unsigned char a = image_ptr[y_pos - vertical_stride + stride * (x - 1)];
        unsigned char b = image_ptr[y_pos - vertical_stride + stride * x];
        unsigned char c = image_ptr[y_pos - vertical_stride + stride * (x + 1)];

        unsigned char d = image_ptr[y_pos + stride * (x - 1)];
//        unsigned char e = image_ptr[y_pos + stride * x];
        unsigned char f = image_ptr[y_pos + stride * (x + 1)];

        unsigned char g = image_ptr[y_pos + vertical_stride + stride * (x - 1)];
        unsigned char h = image_ptr[y_pos + vertical_stride + stride * x];
        unsigned char i = image_ptr[y_pos + vertical_stride + stride * (x + 1)];

        unsigned char result = 0;

        // horizontal = false, vertical = false, magnitude = false
        if (D == 0)
        {
          continue;
        }
          // horizontal = false, vertical = false, magnitude = true
        else if (D == 1)
        {
          unsigned char pos = (unsigned char) (a / 4 + d / 2 + g / 4);
          unsigned char neg = (unsigned char) (c / 4 + f / 2 + i / 4);
          unsigned char max = std::max(pos, neg);
          unsigned char min = std::min(pos, neg);
          unsigned char g_x = max - min;

          pos = (unsigned char) (a / 4 + b / 2 + c / 4);
          neg = (unsigned char) (g / 4 + h / 2 + i / 4);
          max = std::max(pos, neg);
          min = std::min(pos, neg);
          unsigned char g_y = max - min;

          min = std::min(g_x, g_y);
          max = std::max(g_x, g_y);
          min = (unsigned char) (min / 4);

          // simulate saturated addition
          result = max + min;
          // overflow?
          if (result < max)
          {
            result = 0xff;
          }
          sobelImage.magnitude.at(y * w_reduced + x) = result;
        }
          // horizontal = false, vertical = true, magnitude = false
        else if (D == 2)
        {
          unsigned char pos = (unsigned char) (a / 4 + b / 2 + c / 4);
          unsigned char neg = (unsigned char) (g / 4 + h / 2 + i / 4);
          unsigned char max = std::max(pos, neg);
          unsigned char min = std::min(pos, neg);
          result = max - min;
          sobelImage.gy.at(y * w_reduced + x) = result;
        }
          // horizontal = false, vertical = true, magnitude = true
        else if (D == 3)
        {
          unsigned char pos = (unsigned char) (a / 4 + d / 2 + g / 4);
          unsigned char neg = (unsigned char) (c / 4 + f / 2 + i / 4);
          unsigned char max = std::max(pos, neg);
          unsigned char min = std::min(pos, neg);
          unsigned char g_x = max - min;

          pos = (unsigned char) (a / 4 + b / 2 + c / 4);
          neg = (unsigned char) (g / 4 + h / 2 + i / 4);
          max = std::max(pos, neg);
          min = std::min(pos, neg);
          unsigned char g_y = max - min;

          min = std::min(g_x, g_y);
          max = std::max(g_x, g_y);
          min = (unsigned char) (min / 4);

          // simulate saturated addition
          result = max + min;
          // overflow?
          if (result < max)
          {
            result = 0xff;
          }
          sobelImage.gy.at(y * w_reduced + x) = g_y;
          sobelImage.magnitude.at(y * w_reduced + x) = result;
        }
          // horizontal = true, vertical = false, magnitude = false
        else if (D == 4)
        {
          unsigned char pos = (unsigned char) (a / 4 + d / 2 + g / 4);
          unsigned char neg = (unsigned char) (c / 4 + f / 2 + i / 4);
          unsigned char max = std::max(pos, neg);
          unsigned char min = std::min(pos, neg);
          unsigned char result = max - min;
          sobelImage.gx.at(y * w_reduced + x) = result;
        }
          // horizontal = true, vertical = false, magnitude = true
        else if (D == 5)
        {
          unsigned char pos = (unsigned char) (a / 4 + d / 2 + g / 4);
          unsigned char neg = (unsigned char) (c / 4 + f / 2 + i / 4);
          unsigned char max = std::max(pos, neg);
          unsigned char min = std::min(pos, neg);
          unsigned char g_x = max - min;

          pos = (unsigned char) (a / 4 + b / 2 + c / 4);
          neg = (unsigned char) (g / 4 + h / 2 + i / 4);
          max = std::max(pos, neg);
          min = std::min(pos, neg);
          unsigned char g_y = max - min;

          min = std::min(g_x, g_y);
          max = std::max(g_x, g_y);
          min = (unsigned char) (min / 4);

          // simulate saturated addition
          result = max + min;
          // overflow?
          if (result < max)
          {
            result = 0xff;
          }
          sobelImage.gx.at(y * w_reduced + x) = g_x;
          sobelImage.magnitude.at(y * w_reduced + x) = result;
        }
          // horizontal = true, vertical = true, magnitude = false
        else if (D == 6)
        {
          unsigned char pos = (unsigned char) (a / 4 + d / 2 + g / 4);
          unsigned char neg = (unsigned char) (c / 4 + f / 2 + i / 4);
          unsigned char max = std::max(pos, neg);
          unsigned char min = std::min(pos, neg);
          unsigned char g_x = max - min;

          pos = (unsigned char) (a / 4 + b / 2 + c / 4);
          neg = (unsigned char) (g / 4 + h / 2 + i / 4);
          max = std::max(pos, neg);
          min = std::min(pos, neg);
          unsigned char g_y = max - min;
          sobelImage.gx.at(y * w_reduced + x) = g_x;
          sobelImage.gy.at(y * w_reduced + x) = g_y;
        }
          // horizontal = true, vertical = true, magnitude = true
        else if (D == 7)
        {
          unsigned char pos = (unsigned char) (a / 4 + d / 2 + g / 4);
          unsigned char neg = (unsigned char) (c / 4 + f / 2 + i / 4);
          unsigned char max = std::max(pos, neg);
          unsigned char min = std::min(pos, neg);
          unsigned char g_x = max - min;

          pos = (unsigned char) (a / 4 + b / 2 + c / 4);
          neg = (unsigned char) (g / 4 + h / 2 + i / 4);
          max = std::max(pos, neg);
          min = std::min(pos, neg);
          unsigned char g_y = max - min;

          min = std::min(g_x, g_y);
          max = std::max(g_x, g_y);
          min = (unsigned char) (min / 4);

          // simulate saturated addition
          result = max + min;
          // overflow?
          if (result < max)
          {
            result = 0xff;
          }
          sobelImage.gx.at(y * w_reduced + x) = g_x;
          sobelImage.gy.at(y * w_reduced + x) = g_y;
          sobelImage.magnitude.at(y * w_reduced + x) = result;
        }
      }
    }
  }

  template<unsigned int R, unsigned char D, bool A>
  void sse(const Image &image, SobelImage &sobelImage)
  {
    if (D == 0)
    {
      return;
    }

    unsigned char *image_ptr = reinterpret_cast<unsigned char *>(image.image);
    unsigned char *gx_ptr = sobelImage.gx.data();
    unsigned char *gy_ptr = sobelImage.gy.data();
    unsigned char *mag_ptr = sobelImage.magnitude.data();


    const unsigned int w_reduced = 2 * image.width / R;
//    const unsigned int h_reduced = 2 * image.height / R;
    const unsigned int byte_width = 4 * image.width;

    const unsigned int image_width = w_reduced;
//    const unsigned int image_height = h_reduced;

    const unsigned int stride = 2 * R;
    const unsigned int vertical_stride = R * byte_width;

    unsigned char *row_0_ptr = image_ptr;
    unsigned char *row_1_ptr = row_0_ptr + vertical_stride;
    unsigned char *row_2_ptr = row_0_ptr + 2 * vertical_stride;

    const unsigned int y_start = R;
    const unsigned int y_end = 2 * image.height - R;
    const unsigned int y_step = R;

    const unsigned int x_start = 0;
    const unsigned int x_end = w_reduced / 16 - 1;
    const unsigned int x_step = 1;

    for (unsigned int y = y_start; y < y_end; y += y_step)
    {
      unsigned char *row_0_tmp = row_0_ptr;
      unsigned char *row_1_tmp = row_1_ptr;
      unsigned char *row_2_tmp = row_2_ptr;

      unsigned char *gx_tmp = gx_ptr + y / R * image_width + 1;
      unsigned char *gy_tmp = gy_ptr + y / R * image_width + 1;
      unsigned char *mag_tmp = mag_ptr + y / R * image_width + 1;

      __m128i a, c, e, gx, gy, mag;

      a = deinterleave<R, A>(row_0_tmp);
      c = deinterleave<R, A>(row_1_tmp);
      e = deinterleave<R, A>(row_2_tmp);

      a = _mm_mysrli_epi8(a, 1);
      c = _mm_mysrli_epi8(c, 1);
      e = _mm_mysrli_epi8(e, 1);

      for (unsigned int x = x_start; x < x_end; x += x_step)
      {
        __m128i b = deinterleave<R, A>(row_0_tmp + 16 * stride);
        __m128i d = deinterleave<R, A>(row_1_tmp + 16 * stride);
        __m128i f = deinterleave<R, A>(row_2_tmp + 16 * stride);

        b = _mm_mysrli_epi8(b, 1);
        d = _mm_mysrli_epi8(d, 1);
        f = _mm_mysrli_epi8(f, 1);

        sobel<D>(a, b, c, d, e, f, gx, gy, mag);

        if (D == 1)
        {
          _mm_storeu_si128(reinterpret_cast<__m128i *>(mag_tmp), mag);
        }
        else if (D == 2)
        {
          _mm_storeu_si128(reinterpret_cast<__m128i *>(gy_tmp), gy);
        }
        else if (D == 3)
        {
          _mm_storeu_si128(reinterpret_cast<__m128i *>(mag_tmp), mag);
          _mm_storeu_si128(reinterpret_cast<__m128i *>(gy_tmp), gy);
        }
        else if (D == 4)
        {
          _mm_storeu_si128(reinterpret_cast<__m128i *>(gx_tmp), gx);
        }
        else if (D == 5)
        {
          _mm_storeu_si128(reinterpret_cast<__m128i *>(mag_tmp), mag);
          _mm_storeu_si128(reinterpret_cast<__m128i *>(gx_tmp), gx);
        }
        else if (D == 6)
        {
          _mm_storeu_si128(reinterpret_cast<__m128i *>(gy_tmp), gy);
          _mm_storeu_si128(reinterpret_cast<__m128i *>(gx_tmp), gx);
        }
        else if (D == 7)
        {
          _mm_storeu_si128(reinterpret_cast<__m128i *>(mag_tmp), mag);
          _mm_storeu_si128(reinterpret_cast<__m128i *>(gy_tmp), gy);
          _mm_storeu_si128(reinterpret_cast<__m128i *>(gx_tmp), gx);
        }

        row_0_tmp += stride * 16;
        row_1_tmp += stride * 16;
        row_2_tmp += stride * 16;

        mag_tmp += 16;
        gy_tmp += 16;
        gx_tmp += 16;

        a = b;
        c = d;
        e = f;
      }

      // last 16 bytes are missing (first byte already set)
      sobel<D>(a, c, e, gx, gy, mag);

      if (D == 1)
      {
        _mm_storeu_si128(reinterpret_cast<__m128i *>(mag_tmp), mag);
        *(mag_tmp + 14) = 0;
        *(mag_tmp + 15) = 0;
      }
      else if (D == 2)
      {
        _mm_storeu_si128(reinterpret_cast<__m128i *>(gy_tmp), gy);
        *(gy_tmp + 14) = 0;
        *(gy_tmp + 15) = 0;
      }
      else if (D == 3)
      {
        _mm_storeu_si128(reinterpret_cast<__m128i *>(mag_tmp), mag);
        *(mag_tmp + 14) = 0;
        *(mag_tmp + 15) = 0;
        _mm_storeu_si128(reinterpret_cast<__m128i *>(gy_tmp), gy);
        *(gy_tmp + 14) = 0;
        *(gy_tmp + 15) = 0;
      }
      else if (D == 4)
      {
        _mm_storeu_si128(reinterpret_cast<__m128i *>(gx_tmp), gx);
        *(gx_tmp + 14) = 0;
        *(gx_tmp + 15) = 0;
      }
      else if (D == 5)
      {
        _mm_storeu_si128(reinterpret_cast<__m128i *>(mag_tmp), mag);
        *(mag_tmp + 14) = 0;
        *(mag_tmp + 15) = 0;
        _mm_storeu_si128(reinterpret_cast<__m128i *>(gx_tmp), gx);
        *(gx_tmp + 14) = 0;
        *(gx_tmp + 15) = 0;
      }
      else if (D == 6)
      {
        _mm_storeu_si128(reinterpret_cast<__m128i *>(gy_tmp), gy);
        *(gy_tmp + 14) = 0;
        *(gy_tmp + 15) = 0;
        _mm_storeu_si128(reinterpret_cast<__m128i *>(gx_tmp), gx);
        *(gx_tmp + 14) = 0;
        *(gx_tmp + 15) = 0;
      }
      else if (D == 7)
      {
        _mm_storeu_si128(reinterpret_cast<__m128i *>(mag_tmp), mag);
        *(mag_tmp + 14) = 0;
        *(mag_tmp + 15) = 0;
        _mm_storeu_si128(reinterpret_cast<__m128i *>(gy_tmp), gy);
        *(gy_tmp + 14) = 0;
        *(gy_tmp + 15) = 0;
        _mm_storeu_si128(reinterpret_cast<__m128i *>(gx_tmp), gx);
        *(gx_tmp + 14) = 0;
        *(gx_tmp + 15) = 0;
      }

      row_0_ptr += vertical_stride;
      row_1_ptr += vertical_stride;
      row_2_ptr += vertical_stride;
    }
  }
}

#endif //SSE_SOBEL_IMPLEMENTATIONS_HPP