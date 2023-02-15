// includes
#include "Tools/SIMD.h"
#include "YoloRobotDetectorCNNs.h"

#include <emmintrin.h>
#include <math.h>

#include <cstring>
#include <cassert>

#define __STDC_FORMAT_MACROS
#include <cinttypes>


////// UPPER /////
namespace YoloRobotDetectorCNNUpper
{

#include "nao_U16_V32_stride_res_no_horizon.c" // V6: 6.1ms (DCG - ArneImplSelector)

} // namespace YoloRobotDetectorCNNUpper

/////// LOWER /////
namespace YoloRobotDetectorCNNLower
{

#include "nao_L16_V32_stride_res.c" // V6: 1.5ms (DCG - ArneImplSelector)

} // namespace YoloRobotDetectorCNNLower
