/**
 * @file SIMD.h
 *
 * The file defines some compiler specific specializations of MMX and SSE intrinsics
 *
 * @author Alexander Härtl, Fabian Rensen
 */

#pragma once

 // TODO: check this warning
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <tmmintrin.h>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#ifdef TARGET_SIM
inline __m128i my_mm_shuffle_epi8(const __m128i& a, const __m128i& m)
{
  __m128i r;
  const char* ac = reinterpret_cast<const char*>(&a);
  const char* mc = reinterpret_cast<const char*>(&m);
  char* rc = reinterpret_cast<char*>(&r);

  for(size_t i = 0; i < 16; ++i)
  {
    if(*(mc + i) & 0x80)
      *(rc + i) = 0;
    else
      *(rc + i) = *(ac + ((*(mc + i)) & 0x0F)); //a[b[i] & 0x0F];
  }
  return r;
}

#define SHUFFLE(a, m) (my_mm_shuffle_epi8(a, m))
#else
#define SHUFFLE(a, m) (_mm_shuffle_epi8(a, m))
#endif

/**
* \brief Shifts each of the 16 8-bit integers in a bits right, shifting in zeroes.
* This function is not available in SSE3, so it emulates the function by copying the register content to
* two 16-Bit interpreted registers. _mm_srli_epi16 is then called on those two registers and the contents are wrote back
* to one register.
* \param [in] a SSE Register containing 16 8-bit integers.
* \param [in] bits Number of bits to shift the Register a.
* \return The shifted register
*/
inline __m128i _mm_srli_epi8(__m128i a, int bits)
{
  __m128i u = _mm_unpacklo_epi8(a, _mm_setzero_si128());
  __m128i v = _mm_unpackhi_epi8(a, _mm_setzero_si128());
  u = _mm_srli_epi16(u, bits);
  v = _mm_srli_epi16(v, bits);
  return _mm_packus_epi16(u, v);
}

/**
* \brief Shifts each of the 16 8-bit integers in a bits left, shifting in zeroes.
* This function is not available in SSE3, so it emulates the function by copying the register content to
* two 16-Bit interpreted registers. _mm_srli_epi16 is then called on those two registers and the contents are wrote back
* to one register.
* \param [in] a SSE Register containing 16 8-bit integers.
* \param [in] bits Number of bits to shift the Register a.
* \return The shifted register
*/
inline __m128i _mm_slli_epi8(__m128i a, int bits)
{
  __m128i u = _mm_unpacklo_epi8(a, _mm_setzero_si128());
  __m128i v = _mm_unpackhi_epi8(a, _mm_setzero_si128());
  u = _mm_slli_epi16(u, bits);
  v = _mm_slli_epi16(v, bits);
  return _mm_packus_epi16(u, v);
}
