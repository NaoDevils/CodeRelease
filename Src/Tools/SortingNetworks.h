/**
 * @file ProcessedImageProvider.h
 * This file declares SSE rank filter using optimal sorting networks
 * for various input sizes. The Networks were derived using SorterHunter
 * with an applied postfix. 
 * @author <a href="mailto:andre.schurat@tu-dortmund.de">Andre Schurat</a>
 */

#pragma once
#include <tmmintrin.h>

namespace
{
  inline void s(__m128i& a, __m128i& b)
  {
    __m128i temp = a;
    a = _mm_min_epu8(a, b);
    b = _mm_max_epu8(temp, b);
  }

  inline __m128i x(__m128i a, __m128i b)
  {
    return _mm_max_epu8(a, b);
  }

  inline __m128i i(__m128i a, __m128i b)
  {
    return _mm_min_epu8(a, b);
  }
} // namespace

inline __m128i rank1n5(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4)
{
  return i(i(i(r0, r1), i(r2, r3)), r4);
}

inline __m128i& rank2n5(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4)
{
  s(r0, r1);
  s(r0, r2);
  s(r1, r4);
  s(r0, r3);
  s(r1, r2);
  s(r1, r3);
  s(r0, r1);
  return r1;
}

inline __m128i& rank3n5(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4)
{
  s(r0, r1);
  s(r2, r4);
  s(r0, r2);
  s(r1, r4);
  s(r1, r2);
  s(r1, r3);
  s(r2, r3);
  s(r0, r1);
  return r2;
}

inline __m128i& rank4n5(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4)
{
  s(r1, r3);
  s(r0, r4);
  s(r2, r3);
  s(r0, r3);
  s(r2, r4);
  s(r1, r4);
  s(r3, r4);
  return r3;
}

inline __m128i rank5n5(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4)
{
  return x(x(x(r0, r1), x(r2, r3)), r4);
}

inline __m128i rank1n9(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i r8)
{
  return i(i(i(i(r0, r1), i(r2, r3)), i(i(r4, r5), i(r6, r7))), r8);
}

inline __m128i& rank2n9(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8)
{
  s(r1, r8);
  s(r2, r4);
  s(r0, r6);
  s(r1, r5);
  s(r3, r7);
  s(r0, r2);
  s(r1, r2);
  s(r5, r6);
  s(r3, r8);
  s(r3, r4);
  s(r0, r7);
  s(r3, r5);
  s(r0, r1);
  s(r1, r3);
  s(r0, r1);
  return r1;
}

inline __m128i& rank3n9(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8)
{
  s(r2, r3);
  s(r4, r7);
  s(r1, r6);
  s(r0, r5);
  s(r2, r4);
  s(r3, r5);
  s(r6, r7);
  s(r0, r1);
  s(r3, r6);
  s(r0, r2);
  s(r1, r4);
  s(r1, r3);
  s(r2, r8);
  s(r2, r3);
  s(r1, r8);
  s(r1, r2);
  return r2;
}

inline __m128i& rank4n9(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8)
{
  s(r1, r3);
  s(r5, r8);
  s(r2, r4);
  s(r1, r2);
  s(r3, r5);
  s(r6, r8);
  s(r0, r7);
  s(r3, r4);
  s(r5, r8);
  s(r6, r7);
  s(r0, r2);
  s(r5, r7);
  s(r1, r6);
  s(r2, r4);
  s(r2, r5);
  s(r0, r3);
  s(r3, r6);
  s(r2, r6);
  s(r2, r3);
  return r3;
}

inline __m128i& rank5n9(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8)
{
  s(r0, r7);
  s(r2, r5);
  s(r2, r7);
  s(r1, r8);
  s(r3, r4);
  s(r1, r5);
  s(r0, r4);
  s(r0, r1);
  s(r3, r8);
  s(r7, r8);
  s(r2, r3);
  s(r4, r5);
  s(r6, r7);
  s(r1, r3);
  s(r4, r7);
  s(r3, r6);
  s(r4, r6);
  s(r1, r3);
  s(r3, r4);
  return r4;
}

inline __m128i& rank6n9(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8)
{
  s(r2, r3);
  s(r6, r7);
  s(r0, r8);
  s(r2, r7);
  s(r5, r8);
  s(r1, r4);
  s(r0, r7);
  s(r3, r4);
  s(r1, r2);
  s(r0, r2);
  s(r4, r8);
  s(r4, r6);
  s(r4, r5);
  s(r3, r7);
  s(r3, r5);
  s(r6, r7);
  s(r2, r6);
  s(r2, r5);
  s(r5, r6);
  return r5;
}

inline __m128i& rank7n9(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8)
{
  s(r4, r7);
  s(r1, r5);
  s(r6, r8);
  s(r2, r3);
  s(r1, r4);
  s(r3, r7);
  s(r2, r6);
  s(r5, r8);
  s(r7, r8);
  s(r3, r5);
  s(r4, r6);
  s(r0, r7);
  s(r5, r6);
  s(r0, r6);
  s(r5, r7);
  s(r6, r7);
  return r6;
}

inline __m128i& rank8n9(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8)
{
  s(r5, r6);
  s(r0, r1);
  s(r2, r7);
  s(r6, r7);
  s(r3, r4);
  s(r1, r7);
  s(r3, r8);
  s(r0, r1);
  s(r4, r5);
  s(r5, r8);
  s(r2, r6);
  s(r6, r8);
  s(r1, r8);
  s(r5, r7);
  s(r7, r8);
  s(r0, r6);
  return r7;
}

inline __m128i rank9n9(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i r8)
{
  return x(x(x(x(r0, r1), x(r2, r3)), x(x(r4, r5), x(r6, r7))), r8);
}

inline __m128i rank1n13(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8, __m128i& r9, __m128i& r10, __m128i& r11, __m128i& r12)
{
  return i(i(i(i(r0, r1), i(r2, r3)), i(i(r4, r5), i(r6, r7))), i(i(r8, r9), i(i(r10, r11), r12)));
}

inline __m128i& rank2n13(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8, __m128i& r9, __m128i& r10, __m128i& r11, __m128i& r12)
{
  s(r4, r12);
  s(r2, r11);
  s(r3, r10);
  s(r1, r6);
  s(r1, r4);
  s(r1, r5);
  s(r6, r12);
  s(r7, r8);
  s(r0, r9);
  s(r2, r9);
  s(r3, r8);
  s(r3, r7);
  s(r2, r10);
  s(r2, r7);
  s(r0, r4);
  s(r0, r5);
  s(r1, r2);
  s(r6, r11);
  s(r3, r6);
  s(r0, r3);
  s(r1, r3);
  s(r0, r2);
  s(r0, r1);
  return r1;
}

inline __m128i& rank3n13(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8, __m128i& r9, __m128i& r10, __m128i& r11, __m128i& r12)
{
  s(r8, r10);
  s(r3, r4);
  s(r5, r9);
  s(r3, r7);
  s(r0, r11);
  s(r4, r9);
  s(r6, r12);
  s(r10, r12);
  s(r7, r11);
  s(r2, r5);
  s(r0, r1);
  s(r6, r8);
  s(r0, r3);
  s(r1, r4);
  s(r2, r6);
  s(r5, r8);
  s(r0, r2);
  s(r1, r10);
  s(r5, r7);
  s(r1, r5);
  s(r3, r6);
  s(r2, r3);
  s(r1, r3);
  s(r2, r5);
  s(r1, r2);
  return r2;
}

inline __m128i& rank4n13(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8, __m128i& r9, __m128i& r10, __m128i& r11, __m128i& r12)
{
  s(r4, r6);
  s(r3, r9);
  s(r3, r4);
  s(r0, r7);
  s(r5, r9);
  s(r0, r2);
  s(r8, r10);
  s(r1, r12);
  s(r2, r4);
  s(r2, r11);
  s(r5, r12);
  s(r1, r3);
  s(r6, r8);
  s(r9, r12);
  s(r10, r11);
  s(r3, r5);
  s(r9, r10);
  s(r0, r6);
  s(r2, r3);
  s(r1, r6);
  s(r3, r6);
  s(r7, r8);
  s(r5, r7);
  s(r4, r5);
  s(r0, r1);
  s(r1, r2);
  s(r3, r9);
  s(r2, r4);
  s(r3, r4);
  s(r2, r9);
  s(r2, r3);
  return r3;
}

inline __m128i& rank5n13(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8, __m128i& r9, __m128i& r10, __m128i& r11, __m128i& r12)
{
  s(r6, r11);
  s(r8, r10);
  s(r10, r11);
  s(r3, r7);
  s(r2, r4);
  s(r0, r10);
  s(r2, r3);
  s(r1, r12);
  s(r6, r9);
  s(r5, r7);
  s(r3, r9);
  s(r0, r3);
  s(r1, r4);
  s(r1, r5);
  s(r7, r10);
  s(r5, r11);
  s(r2, r6);
  s(r4, r9);
  s(r8, r12);
  s(r5, r6);
  s(r2, r8);
  s(r0, r1);
  s(r4, r7);
  s(r3, r12);
  s(r1, r4);
  s(r3, r5);
  s(r1, r3);
  s(r4, r5);
  s(r4, r6);
  s(r3, r8);
  s(r4, r8);
  s(r3, r4);
  return r4;
}

inline __m128i& rank6n13(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8, __m128i& r9, __m128i& r10, __m128i& r11, __m128i& r12)
{
  s(r2, r8);
  s(r1, r2);
  s(r3, r12);
  s(r5, r9);
  s(r3, r5);
  s(r4, r7);
  s(r1, r4);
  s(r8, r9);
  s(r0, r5);
  s(r0, r6);
  s(r1, r3);
  s(r2, r7);
  s(r5, r7);
  s(r4, r12);
  s(r0, r3);
  s(r9, r12);
  s(r2, r6);
  s(r5, r10);
  s(r6, r10);
  s(r6, r9);
  s(r2, r5);
  s(r4, r8);
  s(r6, r11);
  s(r3, r5);
  s(r0, r2);
  s(r3, r8);
  s(r3, r6);
  s(r2, r4);
  s(r5, r8);
  s(r4, r6);
  s(r5, r11);
  s(r5, r6);
  s(r4, r5);
  return r5;
}

inline __m128i& rank7n13(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8, __m128i& r9, __m128i& r10, __m128i& r11, __m128i& r12)
{
  s(r2, r11);
  s(r7, r9);
  s(r6, r10);
  s(r0, r1);
  s(r3, r5);
  s(r8, r12);
  s(r0, r3);
  s(r10, r11);
  s(r7, r8);
  s(r3, r10);
  s(r1, r5);
  s(r9, r12);
  s(r8, r10);
  s(r11, r12);
  s(r2, r6);
  s(r3, r7);
  s(r7, r9);
  s(r1, r8);
  s(r6, r9);
  s(r1, r6);
  s(r5, r10);
  s(r5, r11);
  s(r2, r7);
  s(r8, r9);
  s(r0, r7);
  s(r5, r7);
  s(r4, r8);
  s(r7, r8);
  s(r4, r5);
  s(r5, r6);
  s(r5, r7);
  s(r6, r7);
  return r6;
}

inline __m128i& rank8n13(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8, __m128i& r9, __m128i& r10, __m128i& r11, __m128i& r12)
{
  s(r3, r6);
  s(r0, r12);
  s(r4, r8);
  s(r6, r9);
  s(r1, r5);
  s(r2, r11);
  s(r5, r12);
  s(r0, r2);
  s(r8, r9);
  s(r6, r7);
  s(r5, r11);
  s(r3, r4);
  s(r0, r6);
  s(r3, r5);
  s(r5, r7);
  s(r6, r10);
  s(r11, r12);
  s(r2, r8);
  s(r1, r4);
  s(r2, r6);
  s(r5, r6);
  s(r4, r8);
  s(r9, r11);
  s(r7, r8);
  s(r4, r10);
  s(r1, r6);
  s(r7, r9);
  s(r9, r10);
  s(r4, r7);
  s(r6, r7);
  s(r9, r12);
  s(r7, r9);
  s(r6, r7);
  return r7;
}

inline __m128i& rank9n13(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8, __m128i& r9, __m128i& r10, __m128i& r11, __m128i& r12)
{
  s(r6, r7);
  s(r3, r4);
  s(r1, r5);
  s(r0, r10);
  s(r0, r9);
  s(r5, r7);
  s(r1, r2);
  s(r3, r11);
  s(r4, r8);
  s(r9, r11);
  s(r4, r5);
  s(r10, r12);
  s(r5, r10);
  s(r4, r5);
  s(r0, r5);
  s(r8, r12);
  s(r6, r8);
  s(r2, r8);
  s(r7, r12);
  s(r7, r10);
  s(r7, r9);
  s(r10, r11);
  s(r1, r7);
  s(r9, r10);
  s(r3, r6);
  s(r2, r9);
  s(r6, r7);
  s(r8, r10);
  s(r8, r9);
  s(r5, r7);
  s(r7, r8);
  s(r8, r9);
  return r8;
}

inline __m128i& rank10n13(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8, __m128i& r9, __m128i& r10, __m128i& r11, __m128i& r12)
{
  s(r7, r11);
  s(r0, r9);
  s(r1, r6);
  s(r3, r4);
  s(r5, r8);
  s(r4, r11);
  s(r0, r12);
  s(r2, r8);
  s(r6, r9);
  s(r4, r9);
  s(r6, r10);
  s(r8, r11);
  s(r3, r5);
  s(r9, r11);
  s(r5, r7);
  s(r3, r10);
  s(r7, r8);
  s(r10, r12);
  s(r0, r7);
  s(r8, r9);
  s(r2, r6);
  s(r1, r7);
  s(r9, r12);
  s(r8, r10);
  s(r4, r6);
  s(r7, r10);
  s(r5, r9);
  s(r7, r9);
  s(r6, r10);
  s(r6, r9);
  s(r9, r10);
  return r9;
}

inline __m128i& rank11n13(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8, __m128i& r9, __m128i& r10, __m128i& r11, __m128i& r12)
{
  s(r0, r12);
  s(r2, r7);
  s(r8, r11);
  s(r3, r4);
  s(r6, r9);
  s(r5, r9);
  s(r4, r7);
  s(r2, r8);
  s(r1, r10);
  s(r11, r12);
  s(r7, r9);
  s(r5, r11);
  s(r1, r8);
  s(r10, r12);
  s(r4, r11);
  s(r0, r3);
  s(r3, r6);
  s(r6, r8);
  s(r7, r10);
  s(r10, r11);
  s(r9, r12);
  s(r8, r9);
  s(r8, r11);
  s(r9, r10);
  s(r10, r11);
  return r10;
}

inline __m128i& rank12n13(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8, __m128i& r9, __m128i& r10, __m128i& r11, __m128i& r12)
{
  s(r8, r9);
  s(r3, r11);
  s(r1, r10);
  s(r5, r11);
  s(r0, r2);
  s(r2, r8);
  s(r4, r12);
  s(r6, r12);
  s(r9, r10);
  s(r6, r11);
  s(r0, r1);
  s(r7, r12);
  s(r7, r11);
  s(r1, r9);
  s(r8, r10);
  s(r10, r12);
  s(r4, r9);
  s(r3, r5);
  s(r5, r12);
  s(r8, r10);
  s(r9, r11);
  s(r10, r11);
  s(r11, r12);
  return r11;
}

inline __m128i rank13n13(__m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3, __m128i& r4, __m128i& r5, __m128i& r6, __m128i& r7, __m128i& r8, __m128i& r9, __m128i& r10, __m128i& r11, __m128i& r12)
{
  return x(x(x(x(r0, r1), x(r2, r3)), x(x(r4, r5), x(r6, r7))), x(x(r8, r9), x(x(r10, r11), r12)));
}
