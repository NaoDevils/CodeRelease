// includes
#include "Tools/SIMD.h"

#include "CLIPBallPerceptorCNNs.h"

#include <emmintrin.h>
#include <pmmintrin.h>
#include <tmmintrin.h>
#include <immintrin.h>
#include <math.h>

#include <cstring>
#include <cassert>

#define __STDC_FORMAT_MACROS
#include <cinttypes>

#ifdef _MSC_VER
#pragma warning(push)
// potentially uninitialized local variable 'x_in' used
// declaration of 'buffer' hides previous local declaration
#pragma warning(disable : 4701 4456)
#endif

namespace cnn_qball
{
#include "cnn_qball.c"
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif
