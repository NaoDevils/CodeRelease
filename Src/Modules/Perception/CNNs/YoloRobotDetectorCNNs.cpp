// includes
#include "Tools/SIMD.h"
#include "YoloRobotDetectorCNNs.h"

#include <emmintrin.h>
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

#ifdef _MSC_VER
#pragma warning(pop)
#endif
