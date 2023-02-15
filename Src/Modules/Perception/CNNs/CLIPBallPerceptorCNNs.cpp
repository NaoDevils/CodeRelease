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

namespace cnn_qball
{
#include "cnn_qball.c"
}
