// includes
#include <emmintrin.h>
#include <math.h>

#define YOLO 1
#define HEAP 1

#if YOLO

#include <vector>

// dimensions
const unsigned int input_height = 120;
const unsigned int input_width = 160;
const unsigned int input_channel = 3;
const unsigned int output_height = 8;
const unsigned int output_width = 10;
const unsigned int output_channel = 21;
const unsigned int num_of_boxes = 3;
const unsigned int num_of_classes = 2;
const unsigned int num_of_coords = 4;
const std::vector<float> anchors = {0.70f,1.36f, 0.89f,3.63f, 2.66f,6.06f};
#endif

#include <cstring>
#include <cassert>

#ifdef CNN_TEST
#include <iostream>
#include <map>
#include <vector>
#include <chrono>

#define __STDC_FORMAT_MACROS
#include <cinttypes>

std::map<std::string, std::vector<uint64_t> > stopwatch_timings;
std::map<std::string, std::chrono::steady_clock::time_point> start_times;

std::chrono::steady_clock::time_point current_time() {
  return std::chrono::steady_clock::now();
}

uint64_t total_time(std::vector<uint64_t> timings)
{
    uint64_t sum = 0;
    for(auto const& timing: timings)
    {
        sum += timing;
    }
    return sum;
}

void start_timing(const char* identifier)
{
    if (start_times.find(identifier) == start_times.end())
    {
        start_times.insert({ identifier, current_time() });
    } else
    {
        start_times[identifier] = current_time();
    }
}

void stop_timing(const char* identifier)
{
    std::chrono::steady_clock::time_point end = current_time();
    std::chrono::steady_clock::time_point start = start_times[identifier];
    uint64_t duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

    if (stopwatch_timings.find(identifier) == stopwatch_timings.end())
    {
        stopwatch_timings.insert({ identifier, std::vector<uint64_t>() });
    }

    stopwatch_timings[identifier].push_back(duration);
}

#define CNN_STOPWATCH(name) for(bool _start = true; (_start ? start_timing(name) : stop_timing(name)), _start; _start = false)
#else
#define CNN_STOPWATCH(name) STOPWATCH(name)
#endif
#ifdef LAYER_STOPWATCH
#define INTERNAL_CNN_STOPWATCH(name) CNN_STOPWATCH(name)
#else
#define INTERNAL_CNN_STOPWATCH(name) // CNN_STOPWATCH(name)
#endif

#ifdef __clang__
#define FORCE_INLINE __attribute__((always_inline))
#elif __gnuc__
#define FORCE_INLINE __attribute__((always_inline)) inline
#elif _MSC_VER
#define FORCE_INLINE __forceinline
#else
#define FORCE_INLINE /* inline */
#endif

FORCE_INLINE int LINEAR_2(int i1, int i2, int d2)
{
    return i1 * d2 + i2;
}

FORCE_INLINE int LINEAR_3(int i1, int i2, int i3, int d2, int d3)
{
    return i1 * d2 * d3 + i2 * d3 + i3;
}

FORCE_INLINE int LINEAR_4(int i1, int i2, int i3, int i4, int d2, int d3, int d4)
{
    return i1 * d2 * d3 * d4 + i2 * d3 * d4 + i3 * d4 + i4;
}

FORCE_INLINE int LINEAR_5(int i1, int i2, int i3, int i4, int i5, int d2, int d3, int d4, int d5)
{
    return i1 * d2 * d3 * d4 * d5 + i2 * d3 * d4 * d5 + i3 * d4 * d5 + i4 * d5 + i5;
}

FORCE_INLINE int LINEAR_6(int i1, int i2, int i3, int i4, int i5, int i6, int d2, int d3, int d4, int d5, int d6)
{
    return i1 * d2 * d3 * d4 * d5 * d6 + i2 * d3 * d4 * d5 * d6 + i3 * d4 * d5 * d6 + i4 * d5 * d6 + i5 * d6 + i6;
}

FORCE_INLINE float MAX_DCG(float a, float b)
{
    return a > b ? a : b;
}

FORCE_INLINE int MAX_DCG(int a, int b)
{
    return a > b ? a : b;
}

FORCE_INLINE float MIN_DCG(float a, float b)
{
    return a < b ? a : b;
}

FORCE_INLINE int MIN_DCG(int a, int b)
{
    return a < b ? a : b;
}

FORCE_INLINE void COPY(void *dest, const void *src, int n, int method)
{
    if (method == 0)
    {        
        memcpy(dest, src, n);
    }
    else
    {
        assert(0);
    }
}

// IM2ROW

// IM2ROW implementations

FORCE_INLINE void im2row_0(int H, int W, int H_OUT, int W_OUT, int KH, int KW, int SH, int SW, int C, const float* in, float* out)
{
    for (int ix = 0; ix < H - KH + 1; ix += SH)
    {
        int x_out_1 = ix / SH;
        for (int jx = 0; jx < W - KW + 1; jx += SW)
        {
            int x_out_2 = jx / SW;
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    for (int kw = 0; kw < C; kw++)
                    {
                        out[LINEAR_5(x_out_1, x_out_2, iw, jw, kw, W_OUT, KH, KW, C)] = in[LINEAR_3(x_1, x_2, kw, W, C)];
                    }
                }
            }
        }
    }
}

FORCE_INLINE void im2row_1(int H, int W, int H_OUT, int W_OUT, int KH, int KW, int SH, int SW, int C, const float* in, float* out)
{
    for (int ix = 0; ix < H - KH + 1; ix += SH)
    {
        int x_out_1 = ix / SH;
        for (int jx = 0; jx < W - KW + 1; jx += SW)
        {
            int x_out_2 = jx / SW;
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                COPY(&out[LINEAR_5(x_out_1, x_out_2, iw, 0, 0, W_OUT, KH, KW, C)], &in[LINEAR_3(x_1, jx, 0, W, C)], KW * C * sizeof(float), 0);
            }
        }
    }
}

FORCE_INLINE void IM2ROW(int H, int W, int H_OUT, int W_OUT, int KH, int KW, int SH, int SW, int C, const float* in, float* out, int method)
{
    if (method == 0)
    {        
        im2row_0(H, W, H_OUT, W_OUT, KH, KW, SH, SW, C, in, out);
    }
    else if (method == 1)
    {        
        im2row_1(H, W, H_OUT, W_OUT, KH, KW, SH, SW, C, in, out);
    }
    else
    {
        assert(0);
    }
}


// GEMM

// GEMM optimizations adopted from https://github.com/flame/how-to-optimize-gemm


FORCE_INLINE void add_dot(int K, const float* a, const float* b, int ldb, float* result)
{
    for (int k=0; k<K; k++)
    {
        *result += a[k] * b[LINEAR_2(k, 0, ldb)];
    }
}

FORCE_INLINE void add_dot_1_4(int K, int N, const float* a, const float* b, float* c)
{
    add_dot(K, &a[LINEAR_2(0, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(0, 0, N)]);
    add_dot(K, &a[LINEAR_2(1, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(1, 0, N)]);
    add_dot(K, &a[LINEAR_2(2, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(2, 0, N)]);
    add_dot(K, &a[LINEAR_2(3, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(3, 0, N)]);
}

FORCE_INLINE void add_dot_1_4_inline(int K, int N, const float* a, const float* b, float* c)
{
    //add_dot(K, &a[LINEAR_2(0, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(0, 0, N)]);
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(0, 0, N)] += a[LINEAR_2(0, k, K)] * b[LINEAR_2(k, 0, N)];
    }
    
    //add_dot(K, &a[LINEAR_2(1, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(1, 0, N)]);
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(1, 0, N)] += a[LINEAR_2(1, k, K)] * b[LINEAR_2(k, 0, N)];
    }
    
    //add_dot(K, &a[LINEAR_2(2, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(2, 0, N)]);
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(2, 0, N)] += a[LINEAR_2(2, k, K)] * b[LINEAR_2(k, 0, N)];
    }
    
    //add_dot(K, &a[LINEAR_2(3, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(3, 0, N)]);
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(3, 0, N)] += a[LINEAR_2(3, k, K)] * b[LINEAR_2(k, 0, N)];
    }
}

FORCE_INLINE void add_dot_1_4_fuse(int K, int N, const float* a, const float* b, float* c)
{
    //add_dot(K, &a[LINEAR_2(0, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(0, 0, N)]);
    //add_dot(K, &a[LINEAR_2(1, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(1, 0, N)]);
    //add_dot(K, &a[LINEAR_2(2, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(2, 0, N)]);
    //add_dot(K, &a[LINEAR_2(3, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(3, 0, N)]);
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(0, 0, N)] += a[LINEAR_2(0, k, K)] * b[LINEAR_2(k, 0, N)];
        c[LINEAR_2(1, 0, N)] += a[LINEAR_2(1, k, K)] * b[LINEAR_2(k, 0, N)];
        c[LINEAR_2(2, 0, N)] += a[LINEAR_2(2, k, K)] * b[LINEAR_2(k, 0, N)];
        c[LINEAR_2(3, 0, N)] += a[LINEAR_2(3, k, K)] * b[LINEAR_2(k, 0, N)];
    }
}

FORCE_INLINE void add_dot_1_4_reg(int K, int N, const float* a, const float* b, float* c)
{
    float b_k;
    float c_0 = 0.0f;
    float c_1 = 0.0f;
    float c_2 = 0.0f;
    float c_3 = 0.0f;
    
    for (int k=0; k<K; k++)
    {
        b_k = b[LINEAR_2(k, 0, N)];
        c_0 += a[LINEAR_2(0, k, K)] * b_k;
        c_1 += a[LINEAR_2(1, k, K)] * b_k;
        c_2 += a[LINEAR_2(2, k, K)] * b_k;
        c_3 += a[LINEAR_2(3, k, K)] * b_k;
    }
    
    c[LINEAR_2(0, 0, N)] += c_0;
    c[LINEAR_2(1, 0, N)] += c_1;
    c[LINEAR_2(2, 0, N)] += c_2;
    c[LINEAR_2(3, 0, N)] += c_3;
}

FORCE_INLINE void add_dot_1_4_ptr(int K, int N, const float* a, const float* b, float* c)
{
    float b_k;
    float c_0 = 0.0f;
    float c_1 = 0.0f;
    float c_2 = 0.0f;
    float c_3 = 0.0f;
    
    const float* a_0_ptr = &a[LINEAR_2(0, 0, K)];
    const float* a_1_ptr = &a[LINEAR_2(1, 0, K)];
    const float* a_2_ptr = &a[LINEAR_2(2, 0, K)];
    const float* a_3_ptr = &a[LINEAR_2(3, 0, K)];
    
    for (int k=0; k<K; k++)
    {
        b_k = b[LINEAR_2(k, 0, N)];
        
        c_0 += *a_0_ptr++ * b_k;
        c_1 += *a_1_ptr++ * b_k;
        c_2 += *a_2_ptr++ * b_k;
        c_3 += *a_3_ptr++ * b_k;
    }
    
    c[LINEAR_2(0, 0, N)] += c_0;
    c[LINEAR_2(1, 0, N)] += c_1;
    c[LINEAR_2(2, 0, N)] += c_2;
    c[LINEAR_2(3, 0, N)] += c_3;
}

FORCE_INLINE void add_dot_1_4_unroll(int K, int N, const float* a, const float* b, float* c)
{
    float b_k;
    float c_0 = 0.0f;
    float c_1 = 0.0f;
    float c_2 = 0.0f;
    float c_3 = 0.0f;
    
    const float* a_0_ptr = &a[LINEAR_2(0, 0, K)];
    const float* a_1_ptr = &a[LINEAR_2(1, 0, K)];
    const float* a_2_ptr = &a[LINEAR_2(2, 0, K)];
    const float* a_3_ptr = &a[LINEAR_2(3, 0, K)];
    
    int k;
    for (k=0; k + 3 < K; k += 4)
    {
        b_k = b[LINEAR_2(k, 0, N)];
        
        c_0 += *a_0_ptr++ * b_k;
        c_1 += *a_1_ptr++ * b_k;
        c_2 += *a_2_ptr++ * b_k;
        c_3 += *a_3_ptr++ * b_k;
        
        b_k = b[LINEAR_2(k + 1, 0, N)];
        
        c_0 += *a_0_ptr++ * b_k;
        c_1 += *a_1_ptr++ * b_k;
        c_2 += *a_2_ptr++ * b_k;
        c_3 += *a_3_ptr++ * b_k;
        
        b_k = b[LINEAR_2(k + 2, 0, N)];
        
        c_0 += *a_0_ptr++ * b_k;
        c_1 += *a_1_ptr++ * b_k;
        c_2 += *a_2_ptr++ * b_k;
        c_3 += *a_3_ptr++ * b_k;
        
        b_k = b[LINEAR_2(k + 3, 0, N)];
        
        c_0 += *a_0_ptr++ * b_k;
        c_1 += *a_1_ptr++ * b_k;
        c_2 += *a_2_ptr++ * b_k;
        c_3 += *a_3_ptr++ * b_k;
    }
    for (; k < K; k++)
    {
        b_k = b[LINEAR_2(k, 0, N)];
        
        c_0 += *a_0_ptr++ * b_k;
        c_1 += *a_1_ptr++ * b_k;
        c_2 += *a_2_ptr++ * b_k;
        c_3 += *a_3_ptr++ * b_k;
    }
    
    c[LINEAR_2(0, 0, N)] += c_0;
    c[LINEAR_2(1, 0, N)] += c_1;
    c[LINEAR_2(2, 0, N)] += c_2;
    c[LINEAR_2(3, 0, N)] += c_3;
}

FORCE_INLINE void add_dot_1_4_indirect(int K, int N, const float* a, const float* b, float* c)
{
    float b_k;
    float c_0 = 0.0f;
    float c_1 = 0.0f;
    float c_2 = 0.0f;
    float c_3 = 0.0f;
    
    const float* a_0_ptr = &a[LINEAR_2(0, 0, K)];
    const float* a_1_ptr = &a[LINEAR_2(1, 0, K)];
    const float* a_2_ptr = &a[LINEAR_2(2, 0, K)];
    const float* a_3_ptr = &a[LINEAR_2(3, 0, K)];
    
    int k;
    for (k=0; k + 3 < K; k += 4)
    {
        b_k = b[LINEAR_2(k, 0, N)];
        
        c_0 += *a_0_ptr * b_k;
        c_1 += *a_1_ptr * b_k;
        c_2 += *a_2_ptr * b_k;
        c_3 += *a_3_ptr * b_k;
        
        b_k = b[LINEAR_2(k + 1, 0, N)];
        
        c_0 += *(a_0_ptr + 1) * b_k;
        c_1 += *(a_1_ptr + 1) * b_k;
        c_2 += *(a_2_ptr + 1) * b_k;
        c_3 += *(a_3_ptr + 1) * b_k;
        
        b_k = b[LINEAR_2(k + 2, 0, N)];
        
        c_0 += *(a_0_ptr + 2) * b_k;
        c_1 += *(a_1_ptr + 2) * b_k;
        c_2 += *(a_2_ptr + 2) * b_k;
        c_3 += *(a_3_ptr + 2) * b_k;
        
        b_k = b[LINEAR_2(k + 3, 0, N)];
        
        c_0 += *(a_0_ptr + 3) * b_k;
        c_1 += *(a_1_ptr + 3) * b_k;
        c_2 += *(a_2_ptr + 3) * b_k;
        c_3 += *(a_3_ptr + 3) * b_k;
        
        a_0_ptr += 4;
        a_1_ptr += 4;
        a_2_ptr += 4;
        a_3_ptr += 4;
    }
    for (; k < K; k++)
    {
        b_k = b[LINEAR_2(k, 0, N)];
        
        c_0 += *a_0_ptr++ * b_k;
        c_1 += *a_1_ptr++ * b_k;
        c_2 += *a_2_ptr++ * b_k;
        c_3 += *a_3_ptr++ * b_k;
    }
    
    c[LINEAR_2(0, 0, N)] += c_0;
    c[LINEAR_2(1, 0, N)] += c_1;
    c[LINEAR_2(2, 0, N)] += c_2;
    c[LINEAR_2(3, 0, N)] += c_3;
}

FORCE_INLINE void add_dot_4_4(int K, int N, const float* a, const float* b, float* c)
{
    add_dot(K, &a[LINEAR_2(0, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(0, 0, N)]);
    add_dot(K, &a[LINEAR_2(1, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(1, 0, N)]);
    add_dot(K, &a[LINEAR_2(2, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(2, 0, N)]);
    add_dot(K, &a[LINEAR_2(3, 0, K)], &b[LINEAR_2(0, 0, N)], N, &c[LINEAR_2(3, 0, N)]);
    
    add_dot(K, &a[LINEAR_2(0, 0, K)], &b[LINEAR_2(0, 1, N)], N, &c[LINEAR_2(0, 1, N)]);
    add_dot(K, &a[LINEAR_2(1, 0, K)], &b[LINEAR_2(0, 1, N)], N, &c[LINEAR_2(1, 1, N)]);
    add_dot(K, &a[LINEAR_2(2, 0, K)], &b[LINEAR_2(0, 1, N)], N, &c[LINEAR_2(2, 1, N)]);
    add_dot(K, &a[LINEAR_2(3, 0, K)], &b[LINEAR_2(0, 1, N)], N, &c[LINEAR_2(3, 1, N)]);
    
    add_dot(K, &a[LINEAR_2(0, 0, K)], &b[LINEAR_2(0, 2, N)], N, &c[LINEAR_2(0, 2, N)]);
    add_dot(K, &a[LINEAR_2(1, 0, K)], &b[LINEAR_2(0, 2, N)], N, &c[LINEAR_2(1, 2, N)]);
    add_dot(K, &a[LINEAR_2(2, 0, K)], &b[LINEAR_2(0, 2, N)], N, &c[LINEAR_2(2, 2, N)]);
    add_dot(K, &a[LINEAR_2(3, 0, K)], &b[LINEAR_2(0, 2, N)], N, &c[LINEAR_2(3, 2, N)]);
    
    add_dot(K, &a[LINEAR_2(0, 0, K)], &b[LINEAR_2(0, 3, N)], N, &c[LINEAR_2(0, 3, N)]);
    add_dot(K, &a[LINEAR_2(1, 0, K)], &b[LINEAR_2(0, 3, N)], N, &c[LINEAR_2(1, 3, N)]);
    add_dot(K, &a[LINEAR_2(2, 0, K)], &b[LINEAR_2(0, 3, N)], N, &c[LINEAR_2(2, 3, N)]);
    add_dot(K, &a[LINEAR_2(3, 0, K)], &b[LINEAR_2(0, 3, N)], N, &c[LINEAR_2(3, 3, N)]);
}

FORCE_INLINE void add_dot_4_4_inline(int K, int N, const float* a, const float* b, float* c)
{
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(0, 0, N)] += a[LINEAR_2(0, k, K)] * b[LINEAR_2(k, 0, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(1, 0, N)] += a[LINEAR_2(1, k, K)] * b[LINEAR_2(k, 0, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(2, 0, N)] += a[LINEAR_2(2, k, K)] * b[LINEAR_2(k, 0, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(3, 0, N)] += a[LINEAR_2(3, k, K)] * b[LINEAR_2(k, 0, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(0, 1, N)] += a[LINEAR_2(0, k, K)] * b[LINEAR_2(k, 1, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(1, 1, N)] += a[LINEAR_2(1, k, K)] * b[LINEAR_2(k, 1, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(2, 1, N)] += a[LINEAR_2(2, k, K)] * b[LINEAR_2(k, 1, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(3, 1, N)] += a[LINEAR_2(3, k, K)] * b[LINEAR_2(k, 1, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(0, 2, N)] += a[LINEAR_2(0, k, K)] * b[LINEAR_2(k, 2, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(1, 2, N)] += a[LINEAR_2(1, k, K)] * b[LINEAR_2(k, 2, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(2, 2, N)] += a[LINEAR_2(2, k, K)] * b[LINEAR_2(k, 2, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(3, 2, N)] += a[LINEAR_2(3, k, K)] * b[LINEAR_2(k, 2, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(0, 3, N)] += a[LINEAR_2(0, k, K)] * b[LINEAR_2(k, 3, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(1, 3, N)] += a[LINEAR_2(1, k, K)] * b[LINEAR_2(k, 3, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(2, 3, N)] += a[LINEAR_2(2, k, K)] * b[LINEAR_2(k, 3, N)];
    }
    
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(3, 3, N)] += a[LINEAR_2(3, k, K)] * b[LINEAR_2(k, 3, N)];
    }
}

FORCE_INLINE void add_dot_4_4_fuse(int K, int N, const float* a, const float* b, float* c)
{
    for (int k=0; k<K; k++)
    {
        c[LINEAR_2(0, 0, N)] += a[LINEAR_2(0, k, K)] * b[LINEAR_2(k, 0, N)];
        c[LINEAR_2(1, 0, N)] += a[LINEAR_2(1, k, K)] * b[LINEAR_2(k, 0, N)];
        c[LINEAR_2(2, 0, N)] += a[LINEAR_2(2, k, K)] * b[LINEAR_2(k, 0, N)];
        c[LINEAR_2(3, 0, N)] += a[LINEAR_2(3, k, K)] * b[LINEAR_2(k, 0, N)];
        
        c[LINEAR_2(0, 1, N)] += a[LINEAR_2(0, k, K)] * b[LINEAR_2(k, 1, N)];
        c[LINEAR_2(1, 1, N)] += a[LINEAR_2(1, k, K)] * b[LINEAR_2(k, 1, N)];
        c[LINEAR_2(2, 1, N)] += a[LINEAR_2(2, k, K)] * b[LINEAR_2(k, 1, N)];
        c[LINEAR_2(3, 1, N)] += a[LINEAR_2(3, k, K)] * b[LINEAR_2(k, 1, N)];
        
        c[LINEAR_2(0, 2, N)] += a[LINEAR_2(0, k, K)] * b[LINEAR_2(k, 2, N)];
        c[LINEAR_2(1, 2, N)] += a[LINEAR_2(1, k, K)] * b[LINEAR_2(k, 2, N)];
        c[LINEAR_2(2, 2, N)] += a[LINEAR_2(2, k, K)] * b[LINEAR_2(k, 2, N)];
        c[LINEAR_2(3, 2, N)] += a[LINEAR_2(3, k, K)] * b[LINEAR_2(k, 2, N)];
        
        c[LINEAR_2(0, 3, N)] += a[LINEAR_2(0, k, K)] * b[LINEAR_2(k, 3, N)];
        c[LINEAR_2(1, 3, N)] += a[LINEAR_2(1, k, K)] * b[LINEAR_2(k, 3, N)];
        c[LINEAR_2(2, 3, N)] += a[LINEAR_2(2, k, K)] * b[LINEAR_2(k, 3, N)];
        c[LINEAR_2(3, 3, N)] += a[LINEAR_2(3, k, K)] * b[LINEAR_2(k, 3, N)];
    }
}

FORCE_INLINE void add_dot_4_4_reg(int K, int N, const float* a, const float* b, float* c)
{
    float b_k_0;
    float b_k_1;
    float b_k_2;
    float b_k_3;
    float c_0_0 = 0.0f;
    float c_1_0 = 0.0f;
    float c_2_0 = 0.0f;
    float c_3_0 = 0.0f;
    float c_0_1 = 0.0f;
    float c_1_1 = 0.0f;
    float c_2_1 = 0.0f;
    float c_3_1 = 0.0f;
    float c_0_2 = 0.0f;
    float c_1_2 = 0.0f;
    float c_2_2 = 0.0f;
    float c_3_2 = 0.0f;
    float c_0_3 = 0.0f;
    float c_1_3 = 0.0f;
    float c_2_3 = 0.0f;
    float c_3_3 = 0.0f;
    
    for (int k=0; k<K; k++)
    {
        b_k_0 = b[LINEAR_2(k, 0, N)];
        b_k_1 = b[LINEAR_2(k, 1, N)];
        b_k_2 = b[LINEAR_2(k, 2, N)];
        b_k_3 = b[LINEAR_2(k, 3, N)];
        
        c_0_0 += a[LINEAR_2(0, k, K)] * b_k_0;
        c_1_0 += a[LINEAR_2(1, k, K)] * b_k_0;
        c_2_0 += a[LINEAR_2(2, k, K)] * b_k_0;
        c_3_0 += a[LINEAR_2(3, k, K)] * b_k_0;
        
        c_0_1 += a[LINEAR_2(0, k, K)] * b_k_1;
        c_1_1 += a[LINEAR_2(1, k, K)] * b_k_1;
        c_2_1 += a[LINEAR_2(2, k, K)] * b_k_1;
        c_3_1 += a[LINEAR_2(3, k, K)] * b_k_1;
        
        c_0_2 += a[LINEAR_2(0, k, K)] * b_k_2;
        c_1_2 += a[LINEAR_2(1, k, K)] * b_k_2;
        c_2_2 += a[LINEAR_2(2, k, K)] * b_k_2;
        c_3_2 += a[LINEAR_2(3, k, K)] * b_k_2;
        
        c_0_3 += a[LINEAR_2(0, k, K)] * b_k_3;
        c_1_3 += a[LINEAR_2(1, k, K)] * b_k_3;
        c_2_3 += a[LINEAR_2(2, k, K)] * b_k_3;
        c_3_3 += a[LINEAR_2(3, k, K)] * b_k_3;
    }
    
    c[LINEAR_2(0, 0, N)] += c_0_0;
    c[LINEAR_2(1, 0, N)] += c_1_0;
    c[LINEAR_2(2, 0, N)] += c_2_0;
    c[LINEAR_2(3, 0, N)] += c_3_0;
    
    c[LINEAR_2(0, 1, N)] += c_0_1;
    c[LINEAR_2(1, 1, N)] += c_1_1;
    c[LINEAR_2(2, 1, N)] += c_2_1;
    c[LINEAR_2(3, 1, N)] += c_3_1;
    
    c[LINEAR_2(0, 2, N)] += c_0_2;
    c[LINEAR_2(1, 2, N)] += c_1_2;
    c[LINEAR_2(2, 2, N)] += c_2_2;
    c[LINEAR_2(3, 2, N)] += c_3_2;
    
    c[LINEAR_2(0, 3, N)] += c_0_3;
    c[LINEAR_2(1, 3, N)] += c_1_3;
    c[LINEAR_2(2, 3, N)] += c_2_3;
    c[LINEAR_2(3, 3, N)] += c_3_3;
}

FORCE_INLINE void add_dot_4_4_ptr(int K, int N, const float* a, const float* b, float* c)
{
    float b_k_0;
    float b_k_1;
    float b_k_2;
    float b_k_3;
    float c_0_0 = 0.0f;
    float c_1_0 = 0.0f;
    float c_2_0 = 0.0f;
    float c_3_0 = 0.0f;
    float c_0_1 = 0.0f;
    float c_1_1 = 0.0f;
    float c_2_1 = 0.0f;
    float c_3_1 = 0.0f;
    float c_0_2 = 0.0f;
    float c_1_2 = 0.0f;
    float c_2_2 = 0.0f;
    float c_3_2 = 0.0f;
    float c_0_3 = 0.0f;
    float c_1_3 = 0.0f;
    float c_2_3 = 0.0f;
    float c_3_3 = 0.0f;
    
    const float* a_0_ptr = &a[LINEAR_2(0, 0, K)];
    const float* a_1_ptr = &a[LINEAR_2(1, 0, K)];
    const float* a_2_ptr = &a[LINEAR_2(2, 0, K)];
    const float* a_3_ptr = &a[LINEAR_2(3, 0, K)];
    
    for (int k=0; k<K; k++)
    {
        b_k_0 = b[LINEAR_2(k, 0, N)];
        b_k_1 = b[LINEAR_2(k, 1, N)];
        b_k_2 = b[LINEAR_2(k, 2, N)];
        b_k_3 = b[LINEAR_2(k, 3, N)];
        
        c_0_0 += *a_0_ptr * b_k_0;
        c_1_0 += *a_1_ptr * b_k_0;
        c_2_0 += *a_2_ptr * b_k_0;
        c_3_0 += *a_3_ptr * b_k_0;
        
        c_0_1 += *a_0_ptr * b_k_1;
        c_1_1 += *a_1_ptr * b_k_1;
        c_2_1 += *a_2_ptr * b_k_1;
        c_3_1 += *a_3_ptr * b_k_1;
        
        c_0_2 += *a_0_ptr * b_k_2;
        c_1_2 += *a_1_ptr * b_k_2;
        c_2_2 += *a_2_ptr * b_k_2;
        c_3_2 += *a_3_ptr * b_k_2;
        
        c_0_3 += *a_0_ptr * b_k_3;
        c_1_3 += *a_1_ptr * b_k_3;
        c_2_3 += *a_2_ptr * b_k_3;
        c_3_3 += *a_3_ptr * b_k_3;
        
        a_0_ptr++;
        a_1_ptr++;
        a_2_ptr++;
        a_3_ptr++;
    }
    
    c[LINEAR_2(0, 0, N)] += c_0_0;
    c[LINEAR_2(1, 0, N)] += c_1_0;
    c[LINEAR_2(2, 0, N)] += c_2_0;
    c[LINEAR_2(3, 0, N)] += c_3_0;
    
    c[LINEAR_2(0, 1, N)] += c_0_1;
    c[LINEAR_2(1, 1, N)] += c_1_1;
    c[LINEAR_2(2, 1, N)] += c_2_1;
    c[LINEAR_2(3, 1, N)] += c_3_1;
    
    c[LINEAR_2(0, 2, N)] += c_0_2;
    c[LINEAR_2(1, 2, N)] += c_1_2;
    c[LINEAR_2(2, 2, N)] += c_2_2;
    c[LINEAR_2(3, 2, N)] += c_3_2;
    
    c[LINEAR_2(0, 3, N)] += c_0_3;
    c[LINEAR_2(1, 3, N)] += c_1_3;
    c[LINEAR_2(2, 3, N)] += c_2_3;
    c[LINEAR_2(3, 3, N)] += c_3_3;
}

FORCE_INLINE void add_dot_4_4_ptr_reg(int K, int N, const float* a, const float* b, float* c)
{
    float b_k_0;
    float b_k_1;
    float b_k_2;
    float b_k_3;
    float c_0_0 = 0.0f;
    float c_1_0 = 0.0f;
    float c_2_0 = 0.0f;
    float c_3_0 = 0.0f;
    float c_0_1 = 0.0f;
    float c_1_1 = 0.0f;
    float c_2_1 = 0.0f;
    float c_3_1 = 0.0f;
    float c_0_2 = 0.0f;
    float c_1_2 = 0.0f;
    float c_2_2 = 0.0f;
    float c_3_2 = 0.0f;
    float c_0_3 = 0.0f;
    float c_1_3 = 0.0f;
    float c_2_3 = 0.0f;
    float c_3_3 = 0.0f;
    
    const float* a_0_ptr = &a[LINEAR_2(0, 0, K)];
    const float* a_1_ptr = &a[LINEAR_2(1, 0, K)];
    const float* a_2_ptr = &a[LINEAR_2(2, 0, K)];
    const float* a_3_ptr = &a[LINEAR_2(3, 0, K)];
    
    for (int k=0; k<K; k++)
    {
        const float a_0 = *a_0_ptr++;
        const float a_1 = *a_1_ptr++;
        const float a_2 = *a_2_ptr++;
        const float a_3 = *a_3_ptr++;
        
        b_k_0 = b[LINEAR_2(k, 0, N)];
        b_k_1 = b[LINEAR_2(k, 1, N)];
        b_k_2 = b[LINEAR_2(k, 2, N)];
        b_k_3 = b[LINEAR_2(k, 3, N)];
        
        c_0_0 += a_0 * b_k_0;
        c_1_0 += a_1 * b_k_0;
        c_2_0 += a_2 * b_k_0;
        c_3_0 += a_3 * b_k_0;
        
        c_0_1 += a_0 * b_k_1;
        c_1_1 += a_1 * b_k_1;
        c_2_1 += a_2 * b_k_1;
        c_3_1 += a_3 * b_k_1;
        
        c_0_2 += a_0 * b_k_2;
        c_1_2 += a_1 * b_k_2;
        c_2_2 += a_2 * b_k_2;
        c_3_2 += a_3 * b_k_2;
        
        c_0_3 += a_0 * b_k_3;
        c_1_3 += a_1 * b_k_3;
        c_2_3 += a_2 * b_k_3;
        c_3_3 += a_3 * b_k_3;
    }
    
    c[LINEAR_2(0, 0, N)] += c_0_0;
    c[LINEAR_2(1, 0, N)] += c_1_0;
    c[LINEAR_2(2, 0, N)] += c_2_0;
    c[LINEAR_2(3, 0, N)] += c_3_0;
    
    c[LINEAR_2(0, 1, N)] += c_0_1;
    c[LINEAR_2(1, 1, N)] += c_1_1;
    c[LINEAR_2(2, 1, N)] += c_2_1;
    c[LINEAR_2(3, 1, N)] += c_3_1;
    
    c[LINEAR_2(0, 2, N)] += c_0_2;
    c[LINEAR_2(1, 2, N)] += c_1_2;
    c[LINEAR_2(2, 2, N)] += c_2_2;
    c[LINEAR_2(3, 2, N)] += c_3_2;
    
    c[LINEAR_2(0, 3, N)] += c_0_3;
    c[LINEAR_2(1, 3, N)] += c_1_3;
    c[LINEAR_2(2, 3, N)] += c_2_3;
    c[LINEAR_2(3, 3, N)] += c_3_3;
}

FORCE_INLINE void add_dot_4_4_rearrange(int K, int N, const float* a, const float* b, float* c)
{
    float b_k_0;
    float b_k_1;
    float b_k_2;
    float b_k_3;
    float c_0_0 = 0.0f;
    float c_1_0 = 0.0f;
    float c_2_0 = 0.0f;
    float c_3_0 = 0.0f;
    float c_0_1 = 0.0f;
    float c_1_1 = 0.0f;
    float c_2_1 = 0.0f;
    float c_3_1 = 0.0f;
    float c_0_2 = 0.0f;
    float c_1_2 = 0.0f;
    float c_2_2 = 0.0f;
    float c_3_2 = 0.0f;
    float c_0_3 = 0.0f;
    float c_1_3 = 0.0f;
    float c_2_3 = 0.0f;
    float c_3_3 = 0.0f;
    
    const float* a_0_ptr = &a[LINEAR_2(0, 0, K)];
    const float* a_1_ptr = &a[LINEAR_2(1, 0, K)];
    const float* a_2_ptr = &a[LINEAR_2(2, 0, K)];
    const float* a_3_ptr = &a[LINEAR_2(3, 0, K)];
    
    for (int k=0; k<K; k++)
    {
        const float a_0 = *a_0_ptr++;
        const float a_1 = *a_1_ptr++;
        const float a_2 = *a_2_ptr++;
        const float a_3 = *a_3_ptr++;
        
        b_k_0 = b[LINEAR_2(k, 0, N)];
        b_k_1 = b[LINEAR_2(k, 1, N)];
        b_k_2 = b[LINEAR_2(k, 2, N)];
        b_k_3 = b[LINEAR_2(k, 3, N)];
        
        c_0_0 += a_0 * b_k_0;
        c_0_1 += a_0 * b_k_1;
        c_0_2 += a_0 * b_k_2;
        c_0_3 += a_0 * b_k_3;
        
        c_1_0 += a_1 * b_k_0;
        c_1_1 += a_1 * b_k_1;
        c_1_2 += a_1 * b_k_2;
        c_1_3 += a_1 * b_k_3;
        
        c_2_0 += a_2 * b_k_0;
        c_2_1 += a_2 * b_k_1;
        c_2_2 += a_2 * b_k_2;
        c_2_3 += a_2 * b_k_3;
        
        c_3_0 += a_3 * b_k_0;
        c_3_1 += a_3 * b_k_1;
        c_3_2 += a_3 * b_k_2;
        c_3_3 += a_3 * b_k_3;
    }
    
    c[LINEAR_2(0, 0, N)] += c_0_0;
    c[LINEAR_2(0, 1, N)] += c_0_1;
    c[LINEAR_2(0, 2, N)] += c_0_2;
    c[LINEAR_2(0, 3, N)] += c_0_3;
    
    c[LINEAR_2(1, 0, N)] += c_1_0;
    c[LINEAR_2(1, 1, N)] += c_1_1;
    c[LINEAR_2(1, 2, N)] += c_1_2;
    c[LINEAR_2(1, 3, N)] += c_1_3;
    
    c[LINEAR_2(2, 0, N)] += c_2_0;
    c[LINEAR_2(2, 1, N)] += c_2_1;
    c[LINEAR_2(2, 2, N)] += c_2_2;
    c[LINEAR_2(2, 3, N)] += c_2_3;
    
    c[LINEAR_2(3, 0, N)] += c_3_0;
    c[LINEAR_2(3, 1, N)] += c_3_1;
    c[LINEAR_2(3, 2, N)] += c_3_2;
    c[LINEAR_2(3, 3, N)] += c_3_3;
}

FORCE_INLINE void add_dot_4_4_vec(int K, int N, const float* a, const float* b, float* c)
{
    __m128 c0 = _mm_setzero_ps(); 
    __m128 c1 = _mm_setzero_ps(); 
    __m128 c2 = _mm_setzero_ps(); 
    __m128 c3 = _mm_setzero_ps(); 
    
    const float* a_0_ptr = &a[LINEAR_2(0, 0, K)];
    const float* a_1_ptr = &a[LINEAR_2(1, 0, K)];
    const float* a_2_ptr = &a[LINEAR_2(2, 0, K)];
    const float* a_3_ptr = &a[LINEAR_2(3, 0, K)];
    
    for (int k=0; k<K; k++)
    {
        const __m128 a0 = _mm_set_ps1(*a_0_ptr++);
        const __m128 a1 = _mm_set_ps1(*a_1_ptr++);
        const __m128 a2 = _mm_set_ps1(*a_2_ptr++);
        const __m128 a3 = _mm_set_ps1(*a_3_ptr++);
        
        __m128 b_ = _mm_loadu_ps(&b[LINEAR_2(k, 0, N)]);
        
        c0 = _mm_add_ps(c0, _mm_mul_ps(a0, b_));
        c1 = _mm_add_ps(c1, _mm_mul_ps(a1, b_));
        c2 = _mm_add_ps(c2, _mm_mul_ps(a2, b_));
        c3 = _mm_add_ps(c3, _mm_mul_ps(a3, b_));
    }
    
    _mm_storeu_ps(&c[LINEAR_2(0, 0, N)], _mm_add_ps(_mm_loadu_ps(&c[LINEAR_2(0, 0, N)]), c0));
    _mm_storeu_ps(&c[LINEAR_2(1, 0, N)], _mm_add_ps(_mm_loadu_ps(&c[LINEAR_2(1, 0, N)]), c1));
    _mm_storeu_ps(&c[LINEAR_2(2, 0, N)], _mm_add_ps(_mm_loadu_ps(&c[LINEAR_2(2, 0, N)]), c2));
    _mm_storeu_ps(&c[LINEAR_2(3, 0, N)], _mm_add_ps(_mm_loadu_ps(&c[LINEAR_2(3, 0, N)]), c3));
}

FORCE_INLINE void inner_kernel(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot_4_4_vec(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot_1_4_ptr(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
    
    for (; m < M; m++)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 1, N)], N, &c[LINEAR_2(m, n + 1, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 2, N)], N, &c[LINEAR_2(m, n + 2, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 3, N)], N, &c[LINEAR_2(m, n + 3, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_0(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    for (int m = 0; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            for (int k = 0; k < K; k++)
            {
                c[LINEAR_2(m, n, ldc)] = c[LINEAR_2(m, n, ldc)] + a[LINEAR_2(m, k, lda)] * b[LINEAR_2(k, n, ldb)];
            }
        }
    }
}

FORCE_INLINE void gemm_1(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    for (int m = 0; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_2(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
            add_dot(K, &a[LINEAR_2(m + 1, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m + 1, n, N)]);
            add_dot(K, &a[LINEAR_2(m + 2, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m + 2, n, N)]);
            add_dot(K, &a[LINEAR_2(m + 3, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m + 3, n, N)]);
        }
    }
        
    for (; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_3(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot_1_4(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
    
    for (; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_4(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot_1_4_inline(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
        
    for (; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_5(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot_1_4_fuse(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
        
    for (; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_6(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot_1_4_reg(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
        
    for (; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_7(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot_1_4_ptr(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
        
    for (; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_8(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot_1_4_unroll(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
        
    for (; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_9(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot_1_4_indirect(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
        
    for (; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_10(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot_4_4(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot_1_4(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
    
    for (; m < M; m++)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 1, N)], N, &c[LINEAR_2(m, n + 1, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 2, N)], N, &c[LINEAR_2(m, n + 2, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 3, N)], N, &c[LINEAR_2(m, n + 3, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_11(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot_4_4_inline(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot_1_4_inline(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
    
    for (; m < M; m++)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 1, N)], N, &c[LINEAR_2(m, n + 1, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 2, N)], N, &c[LINEAR_2(m, n + 2, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 3, N)], N, &c[LINEAR_2(m, n + 3, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_12(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot_4_4_fuse(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot_1_4_fuse(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
    
    for (; m < M; m++)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 1, N)], N, &c[LINEAR_2(m, n + 1, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 2, N)], N, &c[LINEAR_2(m, n + 2, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 3, N)], N, &c[LINEAR_2(m, n + 3, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_13(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot_4_4_reg(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot_1_4_reg(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
    
    for (; m < M; m++)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 1, N)], N, &c[LINEAR_2(m, n + 1, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 2, N)], N, &c[LINEAR_2(m, n + 2, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 3, N)], N, &c[LINEAR_2(m, n + 3, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_14(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot_4_4_ptr(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot_1_4_ptr(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
    
    for (; m < M; m++)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 1, N)], N, &c[LINEAR_2(m, n + 1, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 2, N)], N, &c[LINEAR_2(m, n + 2, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 3, N)], N, &c[LINEAR_2(m, n + 3, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_15(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot_4_4_ptr_reg(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot_1_4_ptr(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
    
    for (; m < M; m++)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 1, N)], N, &c[LINEAR_2(m, n + 1, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 2, N)], N, &c[LINEAR_2(m, n + 2, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 3, N)], N, &c[LINEAR_2(m, n + 3, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_16(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot_4_4_rearrange(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot_1_4_ptr(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
    
    for (; m < M; m++)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 1, N)], N, &c[LINEAR_2(m, n + 1, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 2, N)], N, &c[LINEAR_2(m, n + 2, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 3, N)], N, &c[LINEAR_2(m, n + 3, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_17(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // gemm - MxK * KxN -> MxN
    
    int m;
    for (m = 0; m + 3 < M; m += 4)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot_4_4_vec(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot_1_4_ptr(K, N, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], &c[LINEAR_2(m, n, N)]);
        }
    }
    
    for (; m < M; m++)
    {
        int n;
        for (n = 0; n + 3 < N; n += 4)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 1, N)], N, &c[LINEAR_2(m, n + 1, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 2, N)], N, &c[LINEAR_2(m, n + 2, N)]);
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n + 3, N)], N, &c[LINEAR_2(m, n + 3, N)]);
        }
        
        for (; n < N; n++)
        {
            add_dot(K, &a[LINEAR_2(m, 0, K)], &b[LINEAR_2(0, n, N)], N, &c[LINEAR_2(m, n, N)]);
        }
    }
}

FORCE_INLINE void gemm_18(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc)
{
    // BUGGY
    // gemm - MxK * KxN -> MxN
    
    // blocking factors
    const int mb = 64;
    const int kb = 64;
    
    for (int k = 0; k < K; k += kb)
    {
        int k_ = MIN_DCG(K - k, kb);
        for (int m = 0; m < M; m += mb)
        {
            int m_ = MIN_DCG(M - m, mb);
            inner_kernel(m_, N, k_, &a[LINEAR_2(m, k, K)], lda, &b[LINEAR_2(k, 0, N)], ldb, &c[LINEAR_2(m, 0, N)], ldc);
        }
    }
}

FORCE_INLINE void GEMM(int M, int N, int K, const float *a, int lda, const float *b, int ldb, float *c, int ldc, int method)
{
    if (method == 0)
    {        
        gemm_0(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 1)
    {        
        gemm_1(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 2)
    {        
        gemm_2(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 3)
    {        
        gemm_3(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 4)
    {        
        gemm_4(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 5)
    {        
        gemm_5(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 6)
    {        
        gemm_6(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 7)
    {        
        gemm_7(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 8)
    {        
        gemm_8(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 9)
    {        
        gemm_9(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 10)
    {        
        gemm_10(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 11)
    {        
        gemm_11(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 12)
    {        
        gemm_12(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 13)
    {        
        gemm_13(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 14)
    {        
        gemm_14(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 15)
    {        
        gemm_15(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 16)
    {        
        gemm_16(M, N, K, a, lda, b, ldb, c, ldc);
    }
    else if (method == 17)
    {        
        gemm_17(M, N, K, a, lda, b, ldb, c, ldc);
    }
    //else if (method == 18)
    //{        
    //    gemm_18(M, N, K, a, lda, b, ldb, c, ldc);
    //}
    else
    {
        assert(0);
    }
}


// buffers
alignas(16) float separable_conv2d_internal_0_VALUES[] = {0.0f,0.0f,0.0f};
alignas(16) float separable_conv2d_internal_1_W[] = {-0.03055526129901409f,0.02280188351869583f,0.12219002842903137f,-0.02527778409421444f,0.024780964478850365f,-0.06898313760757446f,0.047689832746982574f,0.031127993017435074f,0.059030357748270035f,-0.05063827708363533f,0.03958272561430931f,0.1333763748407364f,-0.01493535004556179f,0.13811707496643066f,0.11594463884830475f,0.030719459056854248f,-0.01710161752998829f,-0.22035782039165497f,-0.005675758235156536f,0.04588913545012474f,-0.024005575105547905f,0.03454546630382538f,0.03557819873094559f,0.1069808155298233f,0.0006212780135683715f,0.0472605898976326f,0.04110546410083771f,0.08647172898054123f,0.05833888426423073f,-0.07804368436336517f,0.02155213989317417f,0.04171255603432655f,0.07443536818027496f,0.093661367893219f,0.027527382597327232f,0.023950200527906418f,-0.02908443473279476f,0.11114534735679626f,-0.14817439019680023f,-0.05454916134476662f,0.08664626628160477f,0.1114107295870781f,-0.10575760155916214f,0.09414530545473099f,-0.07409290969371796f,-0.082329161465168f,0.0643395259976387f,0.03061707317829132f,-0.08360210806131363f,0.0017930144676938653f,-0.0808112844824791f,0.03205952048301697f,-0.10245847702026367f,0.023432498797774315f,-0.1633933186531067f,0.08073870837688446f,-0.15462499856948853f,0.06181058660149574f,0.07044153660535812f,-0.07481243461370468f,-0.03089587204158306f,0.01770266704261303f,-0.06033456698060036f,0.07390692085027695f,0.06317319720983505f,0.03282206505537033f,-0.08807123452425003f,0.0681559294462204f,0.00405022781342268f,0.0658915638923645f,0.018176984041929245f,-0.04596484452486038f,-0.054515961557626724f,-0.04708714410662651f,-0.02274322882294655f,-0.04869662597775459f,-0.07962914556264877f,0.03335050120949745f,0.07999110966920853f,0.049303412437438965f,0.044618889689445496f,-0.04598008468747139f,0.041995033621788025f,-0.03744598478078842f,-0.08851223438978195f,-0.15398237109184265f,0.0010193591006100178f,-0.052062030881643295f,-0.10931297391653061f,0.10555203258991241f,0.13284257054328918f,0.061791736632585526f,0.021739695221185684f,-0.034682609140872955f,0.05953885242342949f,-0.10991315543651581f,-0.016638576984405518f,-0.08958489447832108f,0.03845495358109474f,-0.003475466277450323f,0.05247846245765686f,0.03204507380723953f,0.10942614823579788f,0.024242937564849854f,0.05087786912918091f,-0.018597528338432312f,0.020222177729010582f,-0.0430685356259346f};
alignas(16) float separable_conv2d_internal_2_W[] = {0.5361745953559875f,0.44209790229797363f,0.023558292537927628f,0.43617498874664307f,-1.64181649684906f,-0.13910852372646332f,2.6575961112976074f,0.20638354122638702f,-2.0084304809570312f,-0.6051183342933655f,-0.24716714024543762f,0.1911950707435608f,-0.6267585754394531f,-8.628725051879883f,2.638068437576294f,-3.1102888584136963f,2.825766086578369f,-1.2667568922042847f,1.311564564704895f,1.2683382034301758f,-2.6762969493865967f,1.3815208673477173f,-0.8273264169692993f,-0.7116434574127197f,-0.06667126715183258f,-3.80291748046875f,0.25173208117485046f,-0.6823490262031555f,-1.185552954673767f,0.6960899233818054f,-1.1523997783660889f,-0.536334216594696f,-1.5822944641113281f,0.14611966907978058f,-0.4503228962421417f,-3.794431447982788f,-1.6869714260101318f,-1.2917710542678833f,0.5602949857711792f,-0.5374712347984314f,-1.0997952222824097f,-1.8865286111831665f,0.8323432803153992f,3.2291290760040283f,0.4276069402694702f,0.21385858952999115f,0.07267750799655914f,-0.4350196123123169f,0.7795330286026001f,1.4458203315734863f,-0.42533159255981445f,-0.5607343912124634f,-2.1103079319000244f,-1.2073428630828857f,1.229932188987732f,-2.064647674560547f,0.4178963303565979f,-0.0834140032529831f,0.01085604541003704f,-0.9492608904838562f,0.058008935302495956f,1.3715717792510986f,-0.46062421798706055f,0.22923283278942108f,1.2847754955291748f,1.0850486755371094f,1.011297583580017f,1.3756473064422607f,1.0996848344802856f,4.878654956817627f,0.45502910017967224f,-1.9862821102142334f,0.47315478324890137f,-1.3776942491531372f,-1.33498215675354f,-0.10936026275157928f,-0.05124635994434357f,-0.4799762964248657f,0.07016521692276001f,-1.1968309879302979f,-0.6341545581817627f,0.13197454810142517f,-1.1454966068267822f,0.47578948736190796f,-1.7114882469177246f,1.3250099420547485f,-0.9006505012512207f,-0.8155913352966309f,-0.8168135285377502f,4.016932010650635f,0.8195555806159973f,-4.484323024749756f,3.4779093265533447f,1.2217369079589844f,0.09386894851922989f,-0.3862318694591522f,-5.258845806121826f,0.4962978661060333f,-2.3722360134124756f,0.391097754240036f,0.28851595520973206f,0.9995742440223694f,-0.6395072340965271f,0.10879885405302048f,-0.2749381363391876f,1.5100499391555786f,-0.8508294820785522f,3.318641424179077f,0.7393746376037598f,0.6044792532920837f,-1.335098385810852f,-0.4436210095882416f,0.12853367626667023f,2.6171164512634277f,-1.0072884559631348f,1.3766841888427734f,-0.18943439424037933f,-0.11617811769247055f,-1.7012972831726074f,0.12914887070655823f,-3.007021903991699f,0.29438939690589905f,-4.7469916343688965f,-0.5955095291137695f,-0.1942998468875885f,-3.5344302654266357f,3.277647018432617f,4.68989372253418f,-2.5315849781036377f,-1.2127207517623901f,0.04737845063209534f,-2.2461516857147217f,-0.6559818983078003f,1.2919012308120728f,-2.000709056854248f,-1.7334694862365723f,0.47606804966926575f,0.7696011662483215f,-0.41200584173202515f,1.2999029159545898f,0.05983130261301994f,-0.37881749868392944f,1.418979525566101f,-0.29272663593292236f,-0.031055398285388947f,2.5456881523132324f,-1.2135162353515625f,-0.05639396980404854f,-1.9719510078430176f,-2.5656585693359375f,0.5655308365821838f,-2.0873005390167236f,0.7827319502830505f,-0.9285256862640381f,0.8372616767883301f,-0.4970872402191162f,0.49498865008354187f,0.28711140155792236f,-1.1119564771652222f,0.15439589321613312f,0.17453670501708984f,-0.7721444964408875f,-0.22155210375785828f,0.8867388367652893f,-1.0630322694778442f,0.5276391506195068f,2.401217222213745f,1.0278618335723877f,2.8752963542938232f,-0.2794078588485718f,5.493013381958008f,0.4116935431957245f,-1.0738897323608398f,-3.3783156871795654f,-1.6713348627090454f,1.287007451057434f,-0.7701375484466553f,-3.2199788093566895f,-0.8009837865829468f,1.343772530555725f,-2.0465598106384277f,1.333046317100525f,-1.5385923385620117f,2.11130952835083f,-1.0694042444229126f,-2.42096209526062f,-0.5399940609931946f,2.5507559776306152f,2.7006711959838867f,2.0072214603424072f,1.35935378074646f,0.2923825681209564f};
alignas(16) float batch_normalization_A[] = {0.0020840836223214865f,-1.1334819793701172f,0.29952242970466614f,-0.6915727257728577f,0.263507604598999f,0.02289643883705139f,0.43215012550354004f,-0.09112115204334259f,0.3487090468406677f,-0.045001670718193054f,0.3979329466819763f,0.024208009243011475f,0.168293759226799f,-0.18721942603588104f,-0.001726299524307251f,-2.404994010925293f};
alignas(16) float conv2d_internal_1_W[] = {0.24490942060947418f,0.08676435798406601f,-2.367748260498047f,0.16174791753292084f,-0.6871324777603149f,-0.46163785457611084f,0.29571616649627686f,-0.10936564952135086f,0.6269776225090027f,0.3003818988800049f,0.002748962724581361f,-0.2717297375202179f,-0.39671969413757324f,-0.4982887804508209f,-0.4403209984302521f,-0.6946688890457153f,0.22405357658863068f,0.1412944793701172f,0.06452985107898712f,-1.284449815750122f,0.6049697995185852f,-0.12286058068275452f,0.12018138915300369f,0.3861541450023651f,-0.3813258707523346f,0.6138767600059509f,0.6184582710266113f,-0.323359876871109f,0.8858565092086792f,-0.3897782266139984f,0.45267054438591003f,-0.18727239966392517f,0.864221453666687f,-0.15190261602401733f,0.38291850686073303f,0.2592778205871582f,-0.027363043278455734f,-0.10332321375608444f,-2.3840601444244385f,0.15172207355499268f,0.7093883752822876f,0.16658034920692444f,-0.10107039660215378f,0.5726926326751709f,0.16783541440963745f,0.06256546080112457f,-2.1293604373931885f,0.08080364763736725f,0.1986037790775299f,0.6516528725624084f,0.17429952323436737f,0.27609750628471375f,-1.2749594449996948f,-0.08807246387004852f,-0.16204580664634705f,0.8671300411224365f,-0.5442814826965332f,-0.23053541779518127f,-0.026720570400357246f,-1.2424614429473877f,0.08711092174053192f,-3.640836715698242f,0.18465395271778107f,-0.16800585389137268f};
alignas(16) float batch_normalization_1_A[] = {0.28900986909866333f,0.0901833027601242f,0.510325014591217f,1.1050100326538086f};
alignas(16) float separable_conv2d_1_internal_0_VALUES[] = {0.0f,0.0f,0.0f,0.0f};
alignas(16) float separable_conv2d_1_internal_1_W[] = {-0.0625806525349617f,0.05786450207233429f,0.034080471843481064f,0.1442604660987854f,0.15687355399131775f,0.04020112007856369f,-0.02382364496588707f,-0.04942072182893753f,0.03412046283483505f,0.014367508701980114f,-0.08354032784700394f,-0.02647433988749981f,0.03975071758031845f,0.02691500633955002f,0.006002899259328842f,0.13880543410778046f,-0.04269719868898392f,-0.07925818115472794f,0.05311841890215874f,0.0932241827249527f,0.0722634494304657f,0.10126279294490814f,-0.06817886978387833f,-0.004773757886141539f,0.03434993326663971f,-0.02910759299993515f,-0.039104484021663666f,-0.018269652500748634f,0.04231308028101921f,0.07497820258140564f,0.030264005064964294f,0.055013976991176605f,0.026491830125451088f,-0.06506700068712234f,0.0399274080991745f,0.04552089050412178f,-0.05030231922864914f,0.13686907291412354f,-0.03103259764611721f,0.06484919041395187f,-0.014205397106707096f,-0.06621673703193665f,-0.008045775815844536f,0.021058455109596252f,-0.010262761265039444f,0.07380624860525131f,-0.0003992619167547673f,-0.02150745503604412f,-0.0778132975101471f,0.16009274125099182f,0.030176522210240364f,-0.09282141178846359f,0.018222913146018982f,-0.08206824958324432f,0.007484461180865765f,-0.12275393307209015f,0.04969008266925812f,0.023705005645751953f,-0.042559750378131866f,-0.03654319792985916f,-0.08011451363563538f,-0.09162646532058716f,0.05045866221189499f,0.06565510481595993f,-0.0931711420416832f,-0.006360237020999193f,0.036779895424842834f,-0.12551361322402954f,-0.06401529908180237f,-0.033829379826784134f,-0.06037098914384842f,-0.12071158736944199f,0.04110068455338478f,-0.012211618945002556f,0.010221260599792004f,-0.06786344945430756f,-0.053976431488990784f,-0.08080179989337921f,0.10975338518619537f,-0.022357773035764694f,-0.022161753848195076f,-0.05120798572897911f,0.04798628389835358f,-0.10558891296386719f,-0.16795270144939423f,0.006897969637066126f,-0.02880563586950302f,-0.046866219490766525f,-0.003254104405641556f,-0.0638139396905899f,0.03562789410352707f,-0.00918482430279255f,-0.02720552310347557f,0.0723448395729065f,0.030889621004462242f,-0.11911725252866745f,0.020432600751519203f,0.05481303855776787f,-0.11665523052215576f,0.034789443016052246f,0.022207319736480713f,-0.09841939806938171f,-0.0333808995783329f,0.07298550009727478f,-0.03323524445295334f,0.0303647480905056f,0.00040371992508880794f,-0.029613394290208817f,0.029391739517450333f,-0.05516483262181282f,-0.03968295454978943f,0.02587776444852352f,0.018519336357712746f,-0.03294559195637703f,-0.1416582465171814f,0.02345433086156845f,0.019103258848190308f,-0.0884946957230568f,-0.044089432805776596f,0.10206373780965805f,-0.05700858682394028f,0.026566501706838608f,0.014245344325900078f,-0.08764918893575668f,0.1201748251914978f,-0.05633671209216118f,0.005621857475489378f,-0.023119892925024033f,0.01846955716609955f,-0.05719705671072006f,-0.08197341114282608f,0.01862598955631256f,-0.02656131610274315f,-0.007752636913210154f,-0.025262705981731415f,0.12227647751569748f,-0.012064659968018532f,-0.04724666476249695f,0.0350133441388607f,-0.016268905252218246f,0.04337749630212784f,0.050256647169589996f,0.0039666458033025265f,-0.07550610601902008f};
alignas(16) float separable_conv2d_1_internal_2_W[] = {0.11047271639108658f,0.7595366835594177f,1.0535314083099365f,1.4635995626449585f,-0.5956885814666748f,0.34114646911621094f,-1.244428277015686f,0.14171724021434784f,-1.124006748199463f,-0.943419873714447f,-0.7883989810943604f,-0.820767879486084f,-1.2989978790283203f,2.4498496055603027f,2.521735668182373f,-0.6640709042549133f,0.7826241850852966f,-4.672478199005127f,0.1362808793783188f,1.161622166633606f,-0.07611511647701263f,-0.9160624742507935f,-0.7365928292274475f,3.641500473022461f,0.20970933139324188f,1.3484688997268677f,-2.8779430389404297f,-0.9424415826797485f,-0.2974751889705658f,1.91359281539917f,1.1310014724731445f,-0.2404853105545044f,-0.5904150009155273f,0.2991001307964325f,0.8817365169525146f,-0.25901123881340027f,0.8998821973800659f,-0.8196491599082947f,-1.597506046295166f,-0.5629405379295349f,-5.191555976867676f,0.0400371178984642f,0.7987284064292908f,1.383774995803833f,3.0893850326538086f,-0.884018063545227f,1.4186078310012817f,0.7324696779251099f,-0.006589700002223253f,-1.340743064880371f,1.4213626384735107f,-3.0255305767059326f,1.3240399360656738f,0.9925910234451294f,2.815054178237915f,-0.3507798910140991f,0.5921390056610107f,1.2058638334274292f,3.3174262046813965f,-0.4948001503944397f,1.1495622396469116f,1.1691268682479858f,-1.8303923606872559f,0.9667267203330994f,0.6106392741203308f,-2.2121334075927734f,1.7768691778182983f,-0.822232186794281f,-0.4980182647705078f,2.5204875469207764f,0.1179049164056778f,-0.5212504267692566f,0.3427455723285675f,-0.6146647930145264f,0.6318982839584351f,2.5746796131134033f,2.9646615982055664f,-0.2856900095939636f,-4.225829601287842f,-0.1412411332130432f,-0.19882452487945557f,-0.9448089003562927f,1.15182363986969f,-0.8595625162124634f,-4.130357265472412f,1.7242215871810913f,-1.1847468614578247f,-0.05361069738864899f,-1.7720749378204346f,-0.3483048379421234f,-1.3101811408996582f,-0.5668413639068604f,1.0100429058074951f,0.8798010349273682f,0.18568114936351776f,-3.343872547149658f,0.9835290908813477f,-0.1898951232433319f,-0.49810948967933655f,0.1285908967256546f,-0.11313110589981079f,0.8739785552024841f,-1.157405972480774f,-0.33554354310035706f,-1.1660656929016113f,1.2898277044296265f,0.3005247116088867f,-0.21663330495357513f,2.8981404304504395f,1.6298774480819702f,-2.700767755508423f,1.3730987310409546f,0.42409250140190125f,0.9070039987564087f,-3.5961570739746094f,-0.06386393308639526f,4.041375160217285f,-0.4156162440776825f,-3.153001546859741f,-0.24312999844551086f,1.2074164152145386f,0.625713586807251f,0.6648715734481812f,1.1128456592559814f,-4.1694207191467285f,0.19308216869831085f,0.8264880180358887f,-0.7928258180618286f,-0.8028879761695862f,0.49826326966285706f,0.9145544767379761f,-0.2540244460105896f,1.1779158115386963f,0.8082671761512756f,-0.33679863810539246f,-0.28176790475845337f,-0.6465290784835815f,0.27823570370674133f,-3.0353939533233643f,-1.1284116506576538f,-2.8116962909698486f,3.9881432056427f,2.093816041946411f,1.1294379234313965f,3.892742156982422f,-1.6226426362991333f,-1.1505405902862549f,0.7145992517471313f,0.4139375686645508f,0.055935826152563095f,0.010828819125890732f,-3.8005735874176025f,0.3745557963848114f,-0.5858626365661621f,-0.6797522902488708f,3.875570058822632f,-0.16859763860702515f,-0.20921553671360016f,0.23672524094581604f,1.3908222913742065f,0.14355038106441498f,-0.23744726181030273f,-0.01975242793560028f,0.2227124571800232f,-0.08291526138782501f,-1.368925929069519f,0.10767840594053268f,0.3194355368614197f,-1.0147675275802612f,-0.9926406145095825f,0.9801033139228821f,-2.7858288288116455f,-0.4096790850162506f,-0.5728517770767212f,-4.179309368133545f,1.1473151445388794f,-1.3104792833328247f,-1.325299620628357f,-1.9293464422225952f,0.5838866829872131f,3.2171294689178467f,-0.08137103170156479f,3.1459944248199463f,2.8015432357788086f,-0.6379387974739075f,0.787811279296875f,0.07485741376876831f,0.23737366497516632f,-2.028252363204956f,-2.272913694381714f,1.9119677543640137f,0.3268977701663971f,0.6703528761863708f,-0.23855142295360565f,-0.27583980560302734f,0.732759952545166f,-0.5767711997032166f,0.2705998420715332f,-0.05973433703184128f,-0.25801390409469604f,-1.2866557836532593f,2.5603644847869873f,-0.7314684987068176f,-0.24427559971809387f,0.01483101211488247f,-0.8383801579475403f,0.5589813590049744f,1.5497252941131592f,0.04262218251824379f,0.857391893863678f,1.125871181488037f,0.04683532565832138f,-0.3331737816333771f,0.24559763073921204f,0.7156556844711304f,0.8893510103225708f,-0.48860153555870056f,0.5643926858901978f,-1.2425048351287842f,1.0338877439498901f,0.9330800771713257f,0.35109782218933105f,-0.2935468852519989f,0.44166985154151917f,0.030006418004631996f,-1.7181556224822998f,-1.0682225227355957f,-0.11560991406440735f,-0.14977383613586426f,-0.6259226202964783f,-0.769919216632843f,-2.752082586288452f,-0.0043372199870646f,0.5265258550643921f,0.6204290986061096f,-0.028268804773688316f,-0.8069069385528564f,1.1414673328399658f,0.686797559261322f,1.3263369798660278f,-0.2846538722515106f,0.3021652400493622f,0.8318270444869995f,1.2263550758361816f,-1.173356533050537f,-0.22560390830039978f,-0.975579023361206f,-0.5759410858154297f,0.5576289892196655f,-3.349025011062622f,1.086092472076416f,0.32075756788253784f,-0.5432404279708862f,0.7136332988739014f,0.7988681793212891f,-0.7724019885063171f,-0.22300522029399872f,-0.34830227494239807f,-1.6936441659927368f,0.5125388503074646f,-0.08843652904033661f,-1.0411977767944336f,-0.29809945821762085f,-0.5525991320610046f,-2.256861686706543f,-1.820688247680664f,-0.16403718292713165f,0.8004534244537354f,-2.073042154312134f,-0.24157951772212982f,-0.08124513924121857f,1.0145550966262817f,0.19664673507213593f,-0.628322958946228f,0.46302905678749084f,1.1029181480407715f,0.8460707664489746f,-2.545274257659912f,-0.28367531299591064f,-1.3197956085205078f,-0.44352948665618896f,1.1611677408218384f,0.6417937874794006f,1.7577959299087524f,0.02021174132823944f,-0.6512826085090637f,-0.7182174921035767f,-0.3562016487121582f,0.6146661639213562f,-0.4365236759185791f,-0.43535560369491577f,2.4539403915405273f,0.367191344499588f,-0.3629693388938904f,-2.166696548461914f,-0.15159505605697632f,-4.139291286468506f,-1.0363224744796753f,-0.9577732682228088f,-0.2632967531681061f,1.8943322896957397f,-0.201816588640213f,1.1715763807296753f,-0.7083596587181091f,0.7092726230621338f,-0.6715103983879089f,0.2738632559776306f,-0.7465149164199829f,0.017789330333471298f,-0.005870588589459658f,-0.7135331034660339f,0.9876502752304077f,0.10243415087461472f,-1.5822298526763916f,1.6273678541183472f,0.21330182254314423f,0.901135265827179f,3.4453282356262207f,-0.4842050075531006f,0.2751004695892334f,-2.952960968017578f,0.34289100766181946f,2.0925979614257812f,-0.5847598314285278f,-0.7257728576660156f,-0.010922246612608433f,-0.6291767358779907f,-0.8120710253715515f,-1.0576255321502686f,0.09847047179937363f,-0.983552098274231f,0.6503065824508667f,-2.1567270755767822f,0.8154127597808838f,-1.342389464378357f,-1.1933765411376953f,0.059924621134996414f,-0.701546311378479f,-0.49238884449005127f,-1.0580474138259888f,-1.466060996055603f,1.0995588302612305f,-2.0782463550567627f,0.044814176857471466f,1.237681269645691f,0.350061297416687f,1.265108346939087f,-0.17624159157276154f,-0.7359534502029419f,-0.462550550699234f,0.9085701107978821f,-0.19161389768123627f,0.47798970341682434f,0.0597299225628376f,1.675718903541565f,-3.1126081943511963f,-0.021890288218855858f,0.016941314563155174f,-0.5427082180976868f,2.4922773838043213f,-0.1378813087940216f,-0.7385150194168091f,-1.9581457376480103f,0.4501711428165436f,0.10681721568107605f,-3.3934874534606934f,1.880684494972229f,0.7398711442947388f,-2.2042393684387207f,1.2092797756195068f,2.3717710971832275f,0.28430768847465515f,-0.4536786377429962f,0.7263216972351074f,-1.9576735496520996f,-0.5922818183898926f,0.11041149497032166f,-0.618760883808136f,0.35237857699394226f,-0.6622597575187683f,2.216014862060547f,0.15138134360313416f,2.5400662422180176f,-1.447587013244629f};
alignas(16) float batch_normalization_2_A[] = {0.1289006918668747f,0.28420490026474f,0.34313124418258667f,0.15018326044082642f,-0.33307337760925293f,-0.013900678604841232f,-0.33252227306365967f,-0.17850428819656372f,-0.005934208631515503f,-0.44046837091445923f,-0.12637591361999512f,0.4684005677700043f,-0.04802931845188141f,0.32931622862815857f,-0.007376454770565033f,-0.17092454433441162f,-0.06746432185173035f,-0.5242332816123962f,-0.15805482864379883f,1.3171839714050293f,-0.05407184362411499f,-0.1077185571193695f,-0.1485302448272705f,0.35971689224243164f};
alignas(16) float conv2d_1_internal_1_W[] = {-0.2604027986526489f,0.013232167810201645f,-0.22232428193092346f,-0.0464172288775444f,0.3798797130584717f,0.022095462307333946f,0.21799539029598236f,0.17684276401996613f,0.1754571497440338f,0.07154512405395508f,-0.2819589078426361f,-0.18411897122859955f,0.00012520021118689328f,-0.22819015383720398f,-0.1911306530237198f,-0.3192985951900482f,0.13177074491977692f,-0.051762085407972336f,-0.23417480289936066f,-0.0940733253955841f,-0.11693418025970459f,0.3397342562675476f,-0.15419171750545502f,0.19381946325302124f,0.1774618923664093f,0.41150155663490295f,-0.10570338368415833f,0.26944100856781006f,0.017674876376986504f,0.34902024269104004f,-0.09733545780181885f,-0.21498045325279236f,-0.0283364187926054f,0.22854356467723846f,-0.24127931892871857f,-0.0015008628834038973f,-0.07175173610448837f,0.04693619906902313f,-0.1528455764055252f,-0.4667004644870758f,-0.11344174295663834f,-0.7690399289131165f,-0.0513668917119503f,-0.10648741573095322f,-0.0716460719704628f,0.08685319870710373f,0.06716962903738022f,0.07542858272790909f,0.43256139755249023f,0.3088531792163849f,0.19601744413375854f,-0.7138976454734802f,0.19000521302223206f,0.07983306050300598f,-0.03485754132270813f,-0.08955181390047073f,0.36420637369155884f,-0.0064235953614115715f,0.1767578423023224f,-0.057983484119176865f,0.13013212382793427f,-0.08368746936321259f,-0.18084679543972015f,0.04698721319437027f,0.07475968450307846f,0.3390745520591736f,-0.19954350590705872f,0.2809986174106598f,-0.04223233461380005f,0.26086360216140747f,-0.08657445758581161f,-0.35538050532341003f,0.10129957646131516f,0.3436143100261688f,-0.27965864539146423f,-0.016299238428473473f,0.0966666117310524f,0.0708623081445694f,-0.10064098984003067f,0.030942587181925774f,0.08424968272447586f,0.009639658965170383f,-0.13267944753170013f,0.6433190107345581f,0.2101089209318161f,0.0005300194025039673f,0.007858832366764545f,0.04797381907701492f,-0.12325744330883026f,0.018953019753098488f,-0.20432038605213165f,0.01590239256620407f,-0.08283676952123642f,0.2586574852466583f,0.34092769026756287f,-0.28989192843437195f,0.08562440425157547f,0.4953414797782898f,-0.136904776096344f,0.32427603006362915f,0.018618687987327576f,0.24892540276050568f,-0.09092209488153458f,-0.34682050347328186f,-0.024213504046201706f,0.036541715264320374f,-0.2936253547668457f,0.108240507543087f,0.16723306477069855f,-0.10951723903417587f,0.06456315517425537f,0.26112306118011475f,0.04625853896141052f,-0.43655863404273987f,0.11156756430864334f,0.07796114683151245f,0.29436588287353516f,0.008208206854760647f,-0.06403768062591553f,0.05089622735977173f,-0.017664125189185143f,0.057627368718385696f,-0.22024786472320557f,0.11568329483270645f,0.08287503570318222f,-0.36377930641174316f,0.09934951364994049f,-0.047628846019506454f,-0.09018386155366898f,-1.00082528591156f,-0.06538281589746475f,-0.09853636473417282f,-0.03327760472893715f,0.058438949286937714f,0.04144454747438431f,-0.00960963312536478f,-0.08888964354991913f,-0.3870772421360016f,0.02631557174026966f,0.04587450250983238f,0.38854578137397766f,0.2017502635717392f,-0.2992844581604004f,-0.027616694569587708f,0.007797823287546635f,-0.4276143014431f,0.0017635172698646784f,0.5219635963439941f,0.26462480425834656f,-0.17571495473384857f,-0.022853629663586617f,0.2831189036369324f,0.21577709913253784f,-0.29245805740356445f,-0.4586576819419861f,0.1167706623673439f,-0.17066511511802673f,-0.08195272833108902f,0.08275333046913147f,-0.08118274807929993f,-0.016560321673750877f,-1.000259280204773f,0.17291362583637238f,-0.051759395748376846f,0.06566892564296722f,0.06216715648770332f,0.03166982904076576f,0.28492388129234314f,-0.15727853775024414f,-0.3138240575790405f,-0.10091326385736465f,0.14903274178504944f,0.18090307712554932f,-0.0629345178604126f,-0.3077261447906494f,0.20405037701129913f,0.03190658986568451f,-0.9275369644165039f,-0.08820644021034241f,-0.22736839950084686f,0.010201564058661461f,0.0011495571816340089f,0.24233266711235046f,-0.3176971971988678f,0.11081240326166153f,0.25349417328834534f,-0.04158657044172287f,0.08610638976097107f,0.015616398304700851f,0.2708036005496979f,-0.07541286200284958f,0.4592408239841461f};
alignas(16) float batch_normalization_3_A[] = {0.43385806679725647f,0.6494448184967041f,1.2338136434555054f,0.12801533937454224f,0.16129939258098602f,0.13863670825958252f,0.7978956699371338f,0.5855744481086731f};
alignas(16) float separable_conv2d_2_internal_0_VALUES[] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
alignas(16) float separable_conv2d_2_internal_1_W[] = {0.11405051499605179f,-0.062269944697618484f,0.03977755084633827f,-0.026951506733894348f,-0.02801314927637577f,0.035219863057136536f,0.06333611905574799f,-0.03360391780734062f,0.031717605888843536f,0.10076811164617538f,0.005015676841139793f,0.058904338628053665f,0.05884912982583046f,0.020885620266199112f,-0.014659887179732323f,-0.07011287659406662f,0.04745521768927574f,-0.07028778642416f,0.04041954129934311f,-0.07897118479013443f,-0.04698079079389572f,-0.11955001950263977f,-0.003530568443238735f,-0.0010157047072425485f,-0.05250613018870354f,-0.05149255692958832f,0.07843978703022003f,0.01924101449549198f,0.06550340354442596f,0.04176831990480423f,0.05810020864009857f,-0.03947530314326286f,0.009171734564006329f,0.10924089699983597f,0.21218155324459076f,0.0960426852107048f,0.07642152905464172f,0.07812129706144333f,-0.02029619924724102f,0.030109765008091927f,-0.07298152148723602f,-0.050431735813617706f,-0.062317777425050735f,0.1706153005361557f,-0.050942450761795044f,0.07119401544332504f,-0.019938314333558083f,-0.08020750433206558f,0.08089206367731094f,0.08146654069423676f,-0.005695353727787733f,-0.039121657609939575f,-0.019591299816966057f,-0.03034535050392151f,-0.09066668152809143f,-0.07752031087875366f,-0.05837690457701683f,-0.08128857612609863f,0.09490177780389786f,-0.07077433913946152f,0.0906904935836792f,-0.07186097651720047f,-0.016569970175623894f,0.09266360104084015f,-0.11675016582012177f,-0.0037704231217503548f,-0.022131891921162605f,-0.15815335512161255f,-0.032118286937475204f,0.024544883519411087f,0.043147873133420944f,0.04812583699822426f,0.08019532263278961f,-0.06062169373035431f,0.0583466961979866f,0.08141309767961502f,-0.05103566497564316f,-0.02356284111738205f,-0.07755017280578613f,-0.008123223669826984f,-0.11201339960098267f,0.09849677234888077f,0.05731292441487312f,-0.013315252028405666f,0.015608862042427063f,0.08093458414077759f,-0.06922449916601181f,0.010718746110796928f,0.08879595994949341f,-0.08337106555700302f,-0.019321026280522346f,-0.015962691977620125f,0.014093045145273209f,-0.020369218662381172f,0.06924401968717575f,-0.009339842945337296f,-0.04578037187457085f,-0.1427174061536789f,-0.01000987272709608f,-0.041472554206848145f,0.013027629815042019f,0.045082252472639084f,0.027318784967064857f,-0.01316642202436924f,-0.011911851353943348f,0.1311807632446289f,0.009539162740111351f,-0.007437759079039097f,-0.013137920759618282f,-0.033225078135728836f,0.049807410687208176f,-0.04526301473379135f,0.017371315509080887f,0.02278447523713112f,0.09751761704683304f,-0.0632961094379425f,-0.02470506727695465f,-0.10135055333375931f,0.0006326987640932202f,0.025541681796312332f,-0.07288471609354019f,-0.023507116362452507f,0.021027667447924614f,0.03613636642694473f,-0.04794389382004738f,0.01046288013458252f,0.1060888022184372f,-0.07014862447977066f,-0.07151896506547928f,-0.03599241375923157f,-0.0003564277139957994f,0.03567902371287346f,0.06556781381368637f,0.06072922423481941f,-0.016311317682266235f,-0.02833288349211216f,-0.09296838194131851f,-0.03816143795847893f,-0.14200372993946075f,-0.02082555741071701f,-0.012111151590943336f,0.17382489144802094f,-0.015153048560023308f,0.15264470875263214f,0.01603798009455204f,-0.1470600962638855f,0.028877707198262215f,0.07928793132305145f,-0.11359386891126633f,0.1088712066411972f,-0.018650928512215614f,-0.03662918508052826f,-0.11969853192567825f,0.03368254750967026f,-0.15026511251926422f,-0.039582595229148865f,0.052401624619960785f,0.007016334682703018f,-0.04597271606326103f,-0.026868073269724846f,-0.02648976258933544f,0.019970275461673737f,0.01603415235877037f,-0.02565518580377102f,-0.0663573369383812f,0.01377682015299797f,0.03703536093235016f,0.035673338919878006f,0.03295040503144264f,-0.03815370798110962f,-0.03990493714809418f,-0.06358284503221512f,-0.07267491519451141f,-0.11375974118709564f,-0.00902086403220892f,0.022837385535240173f,0.06032022461295128f,0.029145007953047752f,0.07326352596282959f,0.07712214440107346f,0.07663551717996597f,-0.0073159621097147465f,-0.033866334706544876f,0.06197364255785942f,-0.001313520479016006f,0.041112665086984634f,0.01584155298769474f,-0.04887199401855469f,0.024283437058329582f,-0.0030482662841677666f,-0.05347321182489395f,-0.036399852484464645f,-0.036104168742895126f,0.004852555692195892f,-0.08393890410661697f,0.1138051226735115f,-0.0671394094824791f,-0.004530380945652723f,-0.0029273703694343567f,-0.09894949197769165f,-0.004407245200127363f,0.034105334430933f,0.08144809305667877f,-0.03471989557147026f,-0.009679284878075123f,0.002691355301067233f,0.026329081505537033f,-0.03843003138899803f,-0.08873897790908813f,-0.015904100611805916f,-0.07556641101837158f,-0.10915503650903702f,-0.04931691661477089f,0.03570995852351189f,0.07097388058900833f,0.04500412568449974f,0.09799529612064362f,0.09185513854026794f,0.02011070027947426f,0.09205976873636246f,-0.00232471851631999f,-0.10067898035049438f,0.011305203661322594f,0.084269218146801f,-0.008571810089051723f,0.0011546441819518805f,-0.05898703634738922f,0.04618031904101372f,-0.0023902971297502518f,-0.007445108611136675f,-0.0730297863483429f,-0.04767932742834091f,-0.04984740912914276f,-0.039214860647916794f,0.054020609706640244f,-0.11675946414470673f,-0.02168514020740986f,-0.01637083850800991f,0.09545266628265381f,-0.00378748239018023f,-0.09268627315759659f,-0.017027495428919792f,-0.001112329657189548f,0.08460495620965958f,-0.025654006749391556f,0.04788161814212799f,0.07160542160272598f,0.1385658085346222f,0.032748255878686905f,0.029028117656707764f,-0.03953557461500168f,0.029061226174235344f,-0.12607285380363464f,-0.06273192167282104f,-0.04983862116932869f,-0.05674906447529793f,0.06526406854391098f,0.08170633763074875f,-0.0711960643529892f,0.014328761026263237f,-0.04634059593081474f,-0.0024330965243279934f,-0.07735384255647659f,0.0439269058406353f,-0.054433826357126236f,-0.025915227830410004f,0.08899733424186707f,-0.040325336158275604f,-0.030805781483650208f,0.013372390531003475f,0.08076260983943939f,-0.04645900800824165f,0.018266815692186356f,-0.04561295732855797f,-0.020103879272937775f,0.0030120755545794964f,0.06906071305274963f,-0.04008815065026283f,-0.04200455918908119f,0.026583971455693245f,0.035360246896743774f,0.1107647716999054f,0.06737029552459717f,-0.05715565010905266f,0.008197441697120667f,0.004847498144954443f,-0.025685567408800125f,-0.05388958752155304f};
alignas(16) float separable_conv2d_2_internal_2_W[] = {0.7727068662643433f,-0.0882846862077713f,2.7131288051605225f,0.00917434599250555f,2.4574568271636963f,1.935444951057434f,0.8535791039466858f,2.432925224304199f,0.10566416382789612f,0.21189576387405396f,-1.741805076599121f,2.259596109390259f,-0.09132169932126999f,0.5818507671356201f,-0.3381882905960083f,0.13943450152873993f,0.19508647918701172f,-0.8097362518310547f,2.8612239360809326f,-1.9604167938232422f,-3.407741069793701f,-0.5100805163383484f,0.7832661867141724f,1.7723913192749023f,1.2628111839294434f,-0.8527428507804871f,0.5141244530677795f,1.5958160161972046f,0.02487861178815365f,0.8520088195800781f,0.7293905019760132f,-1.0439178943634033f,0.329219251871109f,-0.815784752368927f,-0.4394870102405548f,1.8873884677886963f,-0.6932172179222107f,0.3063576817512512f,-2.7170612812042236f,0.23365651071071625f,-1.6326326131820679f,-0.6933431029319763f,1.3530783653259277f,0.4226190149784088f,2.7500388622283936f,0.1440398246049881f,-0.8932685256004333f,3.194920778274536f,-0.33031290769577026f,2.200275182723999f,-1.9661176204681396f,0.6770377159118652f,0.6395536065101624f,0.9864102602005005f,-0.0385805144906044f,2.577082633972168f,0.19235655665397644f,-1.9441912174224854f,0.11192676424980164f,-0.9956024885177612f,-2.9339468479156494f,-0.05208795890212059f,0.3931579887866974f,-0.6706806421279907f,0.6770042777061462f,0.24123190343379974f,0.9755893349647522f,2.7795138359069824f,2.5861012935638428f,0.40413349866867065f,-0.18138612806797028f,-0.24591590464115143f,0.20166805386543274f,-2.671884298324585f,2.605726957321167f,0.9487352967262268f,-1.6374003887176514f,1.2749508619308472f,1.326256275177002f,0.564680814743042f,0.7036811709403992f,-1.4979768991470337f,-0.8260795474052429f,1.5118649005889893f,0.09880220890045166f,2.72872257232666f,-0.9446782469749451f,-1.7438738346099854f,0.37131354212760925f,-0.5765564441680908f,-2.839486837387085f,-0.16172921657562256f,-0.4133215546607971f,-1.656638741493225f,-1.583694577217102f,0.30400052666664124f,0.33492445945739746f,-2.76033353805542f,0.7966311573982239f,1.8816601037979126f,0.5205485820770264f,-0.7838483452796936f,1.4250586032867432f,-1.2578154802322388f,-2.403940439224243f,2.051863670349121f,0.8080207705497742f,-1.422805905342102f,0.19810914993286133f,-0.4855217933654785f,-3.0577828884124756f,0.42124077677726746f,-0.9926310777664185f,-3.2371346950531006f,3.112668991088867f,-0.8495588898658752f,1.428123116493225f,-0.050028953701257706f,-1.4295786619186401f,0.5759776830673218f,-2.77378511428833f,-2.7182071208953857f,0.01779467798769474f,1.2092186212539673f,0.5478223562240601f,-0.7839228510856628f,-0.5689021348953247f,-0.5706114768981934f,-0.27912867069244385f,-0.9543474912643433f,-0.36071887612342834f,0.012742998078465462f,2.104998826980591f,1.5685632228851318f,-1.126220703125f,-0.5233342051506042f,-0.7279176115989685f,0.24726025760173798f,-1.41648530960083f,-0.3315322697162628f,-0.5421552062034607f,0.9874318838119507f,0.15707482397556305f,-0.6163230538368225f,-1.0133135318756104f,-1.5531584024429321f,-0.30518803000450134f,0.14282415807247162f,0.4051379859447479f,-2.2646408081054688f,1.2067997455596924f,2.3881993293762207f,-0.13240016996860504f,1.5104286670684814f,-0.10903555154800415f,0.8041697144508362f,-1.4484440088272095f,1.028434157371521f,1.1374444961547852f,1.376725435256958f,0.9659476280212402f,0.17766690254211426f,-0.02835415117442608f,3.772932767868042f,1.504568099975586f,-1.239053726196289f,0.7238608598709106f,-1.6184617280960083f,2.232996940612793f,-1.117805004119873f,1.0893226861953735f,-0.9505861401557922f,2.0488674640655518f,-0.49566277861595154f,-2.4417943954467773f,-1.1869118213653564f,-0.4076135754585266f,0.6285327672958374f,-0.6776601672172546f,-0.5713587999343872f,0.9954277276992798f,2.4975786209106445f,1.6757032871246338f,0.888292670249939f,-2.6498067378997803f,-0.3490009307861328f,-0.6731442213058472f,1.26571786403656f,-0.704755425453186f,0.26891475915908813f,0.49543020129203796f,-1.8593581914901733f,-0.7044507265090942f,0.9625547528266907f,-0.000960927689447999f,-0.43873172998428345f,2.3180737495422363f,-0.5092635154724121f,-0.6900827288627625f,-0.5806281566619873f,-1.8598567247390747f,-0.36856403946876526f,-0.5812375545501709f,0.030883103609085083f,0.01881074719130993f,-0.7014957070350647f,-1.427685022354126f,2.3739752769470215f,-0.5459364056587219f,0.6289598941802979f,0.3940713703632355f,-2.6149489879608154f,1.1189309358596802f,-0.6361382007598877f,1.0425890684127808f,-0.1287297010421753f,0.12745603919029236f,-0.35663366317749023f,-2.3365378379821777f,-0.9676693677902222f,-1.241987943649292f,0.21890416741371155f,0.052007775753736496f,0.37004488706588745f,-3.235192060470581f,-0.6435443758964539f,0.261808305978775f,-0.7765076756477356f,0.7983863353729248f,1.1393944025039673f,1.514522910118103f,0.8053292036056519f,0.7718766331672668f,0.6495379209518433f,0.22077135741710663f,0.2088121771812439f,-2.320276975631714f,-0.5076470375061035f,1.6311357021331787f,0.3149840533733368f,-2.760176420211792f,-1.8413666486740112f,0.2596569359302521f,-0.9317395687103271f,0.07263826578855515f,-0.06115228682756424f,1.0660041570663452f,-2.622525453567505f,0.7051804661750793f,-1.0034929513931274f,-0.5474191904067993f,1.2285634279251099f,-1.222508192062378f,-0.4964534640312195f,0.17064322531223297f,0.23824600875377655f,0.6087743639945984f,2.088453769683838f,0.33805298805236816f,0.39394161105155945f,0.3840373456478119f,-0.6170600056648254f,0.5894798636436462f,-1.6440688371658325f,0.06789561361074448f,-0.06956607103347778f,-1.369396686553955f,-0.5684683322906494f,2.30055570602417f,-2.238271474838257f,0.7773444652557373f,0.07098396867513657f,-0.5867503881454468f,-2.8217616081237793f,0.032190952450037f,-1.4710972309112549f,0.3912443518638611f,-0.33246907591819763f,-0.5981795787811279f,-1.1545077562332153f,-1.8243035078048706f,-1.4243063926696777f,-0.895013153553009f,-0.32496377825737f,-1.8511866331100464f,0.105390764772892f,-0.749458909034729f,-2.88446044921875f,-1.5269054174423218f,0.9903740882873535f,1.4635332822799683f,0.818202555179596f,-0.9337217211723328f,0.4157179296016693f,-4.445087909698486f,0.5127710700035095f,0.6686853170394897f,0.29733806848526f,1.0892740488052368f,-0.26215359568595886f,-1.499977469444275f,-0.31161215901374817f,-0.015973905101418495f,1.822492003440857f,-0.17814770340919495f,2.6871769428253174f,-0.45628121495246887f,0.7399523258209229f,0.4068366587162018f,-0.08302753418684006f,-0.5093911290168762f,-1.4561721086502075f,-3.7428627014160156f,1.6183123588562012f,-1.321743369102478f,-1.0732187032699585f,-1.4446418285369873f,0.20298482477664948f,-0.6033000946044922f,-0.8586205244064331f,-0.6638492941856384f,0.2740854322910309f,-1.2834291458129883f,-1.624104380607605f,0.8610568642616272f,-0.6976915001869202f,0.7298334240913391f,1.0540896654129028f,-3.1549441814422607f,-0.13727711141109467f,2.2739431858062744f,0.35112464427948f,-0.3157315254211426f,0.014062096364796162f,0.7574594020843506f,-0.06178797036409378f,-1.2405333518981934f,-0.07017798721790314f,0.23880933225154877f,0.7418656945228577f,0.06258206069469452f,-0.7121767401695251f,-0.304892361164093f,1.261778712272644f,-1.299565076828003f,0.7909151911735535f,-3.3817622661590576f,1.7034037113189697f,-0.6639251112937927f,-1.0531474351882935f,-0.48021337389945984f,0.024946533143520355f,-1.5639564990997314f,-0.8004994988441467f,0.4280300438404083f,0.33299028873443604f,0.9354056119918823f,4.723449230194092f,0.10011567920446396f,0.062959223985672f,1.522552728652954f,0.20238980650901794f,-1.8700087070465088f,-2.7986526489257812f,1.2504825592041016f,-0.3003098666667938f,-1.5497016906738281f,0.8530795574188232f,1.1075749397277832f,0.14972075819969177f,-1.1660490036010742f,0.17058180272579193f,1.3497042655944824f,-3.4602530002593994f,-1.7594659328460693f,-2.3552887439727783f,-0.47558847069740295f,-0.6197741627693176f,0.06123027577996254f,-2.1821115016937256f,1.4155960083007812f,1.2232457399368286f,1.0119929313659668f,-0.18121324479579926f,1.2348741292953491f,-2.305729866027832f,-0.7662358283996582f,0.05099428445100784f,-1.5760867595672607f,0.535306453704834f,0.04397871345281601f,3.5227315425872803f,1.4577559232711792f,-0.5340590476989746f,0.7660272717475891f,-1.161205768585205f,0.06945320963859558f,-1.4727646112442017f,1.9888454675674438f,-0.3440340459346771f,2.9652509689331055f,-0.007253617513924837f,-0.005781619809567928f,-1.4943737983703613f,0.8633798956871033f,0.4936459958553314f,-0.3710843622684479f,-0.2662979066371918f,-0.6372429132461548f,2.741556406021118f,-0.6289186477661133f,-0.22572241723537445f,-1.179793357849121f,1.931235909461975f,-0.32617825269699097f,-0.9657226204872131f,0.991975724697113f,1.1384142637252808f,-0.9396967887878418f,0.6595690250396729f,-1.6608494520187378f,-1.2700148820877075f,-0.4940677881240845f,-3.54842209815979f,-0.09941978752613068f,0.6824215650558472f,-0.9363695979118347f,1.5375949144363403f,-0.3085547685623169f,-1.129501223564148f,-0.8465964794158936f,-0.07566111534833908f,2.4770021438598633f,2.402818202972412f,0.42249712347984314f,0.04116787388920784f,0.7524535059928894f,1.9422451257705688f,2.2347869873046875f,0.7665467858314514f,0.6115862131118774f,-2.243938446044922f,-0.9363468289375305f,-0.99409019947052f,0.6355476975440979f,0.29623669385910034f,-1.343591332435608f,-0.7346621155738831f,1.8236165046691895f,2.3322980403900146f,0.08166274428367615f,-1.983425259590149f,-0.4800400733947754f,0.9228244423866272f,-0.9805662631988525f,0.08336386829614639f,-1.280646562576294f,0.009681335650384426f,1.7967332601547241f,2.306436777114868f,-0.43325474858283997f,4.87346076965332f,0.7782071828842163f,-0.07920488715171814f,-2.885608673095703f,0.9428650736808777f,-0.5787724256515503f,-1.241117238998413f,1.5210537910461426f,1.683721899986267f,-2.407224655151367f,0.4273519217967987f,0.056025851517915726f,-2.515693426132202f,-0.2893640995025635f,-2.5489659309387207f,-0.0020420418586581945f,2.18886661529541f,0.6377070546150208f,0.8028883337974548f,-0.7881250977516174f,2.7379252910614014f,0.06816553324460983f,-1.6287269592285156f,1.9466899633407593f,0.29485300183296204f,0.19227969646453857f,0.6388348340988159f,-0.0317406952381134f,-1.6064685583114624f,1.1171225309371948f,-0.5773195624351501f,-1.6030105352401733f,0.3787568509578705f,0.45572730898857117f,0.07404735684394836f,2.163424015045166f,-1.723260521888733f,-0.9363448023796082f,3.5180575847625732f,2.0880119800567627f,1.2065778970718384f,-0.09019673615694046f,-0.5821986198425293f,0.7667783498764038f,0.09598936140537262f,2.268812417984009f,1.3577308654785156f,-1.1373833417892456f,-1.163507103919983f,-0.7840215563774109f,-1.111504077911377f,-1.0769383907318115f,0.6549071073532104f,-1.7881428003311157f,-1.0155421495437622f,1.8825618028640747f,1.0248911380767822f,1.765415072441101f,0.9203227758407593f,-0.6690731048583984f,-0.07494822889566422f,-1.1014469861984253f,-0.6646749973297119f,0.6989920139312744f,-0.2418062388896942f,1.080351710319519f,-0.4006079435348511f,-1.1631642580032349f,2.0631918907165527f,0.929774284362793f,-0.6078660488128662f,-1.6214289665222168f,-1.0705437660217285f,-0.934026837348938f,-2.1372814178466797f,-1.41327702999115f,-0.9565317034721375f,-0.7982556819915771f,-0.393828421831131f,1.7009154558181763f,1.544089436531067f,-0.9842469692230225f,-1.9676213264465332f,1.5363249778747559f,0.04046911373734474f,0.2899151146411896f,-1.6004517078399658f,-0.11783283948898315f,-0.2060745656490326f,0.07860804349184036f,-2.8505067825317383f,-2.1483259201049805f,0.8588961362838745f,-2.416653871536255f,-2.2731881141662598f,-0.04057740420103073f,1.9897377490997314f,1.684787392616272f,1.232765793800354f,1.0982468128204346f,-1.832008957862854f,-2.094758987426758f,-0.14765329658985138f,-0.5085232257843018f,-0.09686550498008728f,1.3426129817962646f,-0.4818122088909149f,-0.7348586916923523f,1.1672368049621582f,-0.12585784494876862f,2.0634968280792236f,1.6943316459655762f,0.2940138876438141f,1.0194432735443115f,-1.176377773284912f,-1.1639256477355957f,1.2170072793960571f,1.4818503856658936f,0.9823028445243835f,-1.4760466814041138f,-1.8256171941757202f,0.6209479570388794f,-2.909403085708618f,-0.2317400723695755f,0.2767607867717743f,-1.223260521888733f,0.8837426900863647f,-0.4094844162464142f,0.09321721643209457f,-1.0314712524414062f,-2.1666531562805176f,-2.806690216064453f,-2.193070650100708f,0.4265742599964142f,-1.3192260265350342f,0.4956788122653961f,-0.5119540095329285f,-2.589336395263672f,-2.0907375812530518f,0.04118482768535614f,-4.33796501159668f,-0.09154542535543442f,-0.7108204364776611f,0.041377026587724686f,2.0167226791381836f,-0.3180806338787079f,-1.4419169425964355f,3.2436819076538086f,1.1295546293258667f,0.27470430731773376f,1.2443602085113525f,-0.5885379910469055f,0.6104193925857544f,1.9626494646072388f,1.2850360870361328f,-2.395582437515259f,-2.1993651390075684f,-1.0174380540847778f,1.4626798629760742f,-0.585379421710968f,1.3870210647583008f,0.6437475681304932f,-0.9952017664909363f,0.4218565821647644f,1.5801252126693726f,-1.4938775300979614f,2.4870991706848145f,-1.0561057329177856f,-0.09012909978628159f,0.509956419467926f,0.005234557669609785f,0.4394755959510803f,-0.5805076360702515f,-0.41946181654930115f,-1.228371024131775f,1.6078094244003296f,1.0977590084075928f,-0.26885974407196045f,3.3377058506011963f,-0.5081592202186584f,1.6443164348602295f,-0.5342617034912109f,-2.0975077152252197f,0.21765322983264923f,-1.1153433322906494f,0.11038761585950851f,0.5152636766433716f,1.6754869222640991f,1.4061833620071411f,0.2579227685928345f,0.5350533723831177f,0.011282500810921192f,1.2671431303024292f,0.4776960015296936f,3.3945815563201904f,0.891342282295227f,1.5600134134292603f,1.7020591497421265f,-0.667149543762207f,3.2259936332702637f,1.4973629713058472f,-0.03224272280931473f,-1.7382566928863525f,-1.2374674081802368f,1.0108165740966797f,0.984021008014679f,0.45565834641456604f,0.4728088974952698f,-0.17896509170532227f,-1.5295264720916748f,-1.12247896194458f,1.7142938375473022f,2.703554153442383f,1.9266704320907593f,1.928794264793396f,4.622605323791504f,0.8728722929954529f,2.4915144443511963f,0.8063444495201111f,0.6148059964179993f,0.8201760053634644f,-1.9139676094055176f,-0.23227909207344055f,0.01807267591357231f,1.418835163116455f,-0.6227285265922546f,0.07714959979057312f,-0.5539132952690125f,0.7166020274162292f,1.8123050928115845f,-0.17175257205963135f,1.7060579061508179f,0.11393602937459946f,-0.11521254479885101f,-0.022765548899769783f,0.8973333239555359f,0.7354632019996643f,-0.8984056711196899f,-2.8223798274993896f,0.7582996487617493f,0.1794753223657608f,0.7595695853233337f,-0.5680579543113708f,-2.643336534500122f,-1.0111048221588135f,1.8546634912490845f,-0.397745281457901f,2.3105485439300537f,0.2521176040172577f,-1.4899967908859253f,0.2664075195789337f,0.9625970721244812f,0.26850050687789917f,1.6015840768814087f,-0.5797486305236816f,-0.5124163031578064f,2.4926459789276123f,0.9518364667892456f,-1.040564775466919f,1.070418357849121f,-1.1654773950576782f,-0.9726280570030212f,-1.6448050737380981f,1.9814656972885132f,0.619979977607727f,0.7650571465492249f,0.5052438974380493f,0.24779509007930756f,-1.171751618385315f,0.6374931335449219f,0.6058974862098694f,-1.6246119737625122f,2.22214674949646f,2.3946852684020996f,-0.13373532891273499f,-1.3995860815048218f,-0.8882855176925659f,1.3413527011871338f,-0.2810038626194f,0.3800971806049347f,-0.8414586186408997f,1.2032349109649658f,-0.5778260827064514f,-1.0170131921768188f,-2.181471347808838f,0.14136968553066254f,-0.26694396138191223f,-1.5631192922592163f,0.5634057521820068f,1.3215433359146118f,-0.06869331747293472f,0.948438823223114f,-1.4050863981246948f,-0.2548688054084778f,-0.2229381948709488f,1.8497577905654907f,2.910236358642578f,-3.3499250411987305f,-2.215944290161133f,1.4980428218841553f,1.3482722043991089f,-2.39264178276062f,0.4214238226413727f,-1.2755537033081055f,-0.603266716003418f,1.5233232975006104f,0.9642064571380615f,-0.1678590625524521f,-0.7535858154296875f,2.8060524463653564f,1.0909572839736938f,-1.5872117280960083f,-1.1717290878295898f,-1.1310491561889648f,-1.7740036249160767f,0.559306263923645f,-0.5796713829040527f,0.3377187252044678f,-1.923252820968628f,-2.161503791809082f,-1.8094500303268433f,1.8612148761749268f,0.2149721086025238f,0.7675042748451233f,0.24351538717746735f,2.027738094329834f,-0.6974979639053345f,-1.8770060539245605f,-1.5307148694992065f,-0.5343643426895142f,0.5390347242355347f,0.8974162936210632f,-1.268535852432251f,0.012051948346197605f,-0.556645393371582f,0.08032545447349548f,-0.03091547079384327f,-2.7847564220428467f,0.28592246770858765f,0.40130531787872314f,-2.0738778114318848f,1.6028611660003662f,0.07174860686063766f,-0.6539488434791565f,0.7739179134368896f,-1.21435546875f,-0.2665678560733795f,2.0618982315063477f,-1.9503761529922485f,1.3146721124649048f,1.4766079187393188f,0.04687517136335373f,1.9001142978668213f,1.936962604522705f,-0.041208311915397644f,2.942077398300171f,0.828583300113678f,1.7211945056915283f,1.1575497388839722f,1.8147392272949219f,-0.3463267683982849f,-0.6972729563713074f,0.12522996962070465f,1.471782922744751f,1.173871636390686f,-0.694880485534668f,-3.8527421951293945f,-0.9687622785568237f,-1.2589877843856812f,-0.42745041847229004f,0.39412862062454224f,-0.3344894051551819f,-1.5823674201965332f,-0.06885508447885513f,-1.097266435623169f,0.7426743507385254f,0.22137551009655f,-2.130417585372925f,0.7102112174034119f,-0.6115367412567139f,0.0034251841716468334f,1.5141141414642334f,-0.8053135275840759f,0.12045659869909286f,-1.92839515209198f,-4.247014999389648f,0.674856960773468f,1.9763695001602173f,0.6494745016098022f,-0.48764023184776306f,-0.22617195546627045f,0.42663443088531494f,-1.291373372077942f,3.447695732116699f,1.3324494361877441f,0.9188675284385681f,-0.558994472026825f,-0.5533842444419861f,-0.4775923788547516f,0.6108986139297485f,-0.12084242701530457f,0.2810763120651245f,0.6100782752037048f,-0.9227126836776733f,0.9399399161338806f,0.3324912488460541f,0.6489871740341187f,-0.44157201051712036f,0.2764219343662262f,0.0020432346500456333f,0.09995270520448685f,-2.7142226696014404f,-1.9390991926193237f,2.739666223526001f,-0.08708589524030685f,-0.6700440049171448f,0.7655234336853027f,1.168712854385376f,0.5536405444145203f,-0.8051095604896545f,-2.6512818336486816f,0.0586138479411602f,1.790456771850586f,1.5704289674758911f,-1.377571702003479f,-0.668733537197113f,1.62869131565094f,0.13383394479751587f,-2.548990488052368f,0.2246420532464981f,0.5841155648231506f,-1.867889165878296f,-2.315819501876831f,0.34360942244529724f,-0.8240790367126465f,0.8508859872817993f,-1.734929084777832f,0.7553833723068237f,1.0776749849319458f,0.7113546133041382f,-0.7524158954620361f,0.3341044485569f,0.7955412864685059f,-1.114839792251587f,-0.2308548092842102f,-1.7194898128509521f,-1.4215468168258667f,0.49393001198768616f,3.564680337905884f,-1.8948217630386353f,0.5238788723945618f,1.4012943506240845f,-1.3503713607788086f,-2.0244529247283936f,0.350968599319458f,2.0937087535858154f,-0.8563277721405029f,0.32846030592918396f,2.1368987560272217f,1.7616429328918457f,-0.10833454877138138f,-0.4253924489021301f,0.631511926651001f,-0.9062463641166687f,-0.9079723358154297f,0.5704559683799744f,1.330686092376709f,-0.5556319355964661f,0.023390164598822594f,-0.20784132182598114f,-0.047990020364522934f,0.5421010255813599f,-0.6782923340797424f,0.027372345328330994f,-0.12932489812374115f,0.06076623499393463f,-0.4847815930843353f,-0.08730892837047577f,-0.2392718642950058f,0.08708630502223969f,-0.4093974828720093f,-1.7616462707519531f,1.9000954627990723f,1.2307913303375244f,1.2648117542266846f,-0.4697507619857788f,0.17428572475910187f,0.05462115257978439f,2.5877556800842285f,0.13680067658424377f,0.9403180480003357f,-2.745783567428589f,2.7822728157043457f,0.8644014596939087f,1.891825795173645f,0.21197055280208588f,-2.0940678119659424f,1.9571040868759155f,-0.1992838829755783f,0.8193398714065552f,2.052248001098633f,-2.204808473587036f,0.6088144183158875f,1.2531765699386597f,0.23475565016269684f,0.7316268682479858f,3.5179059505462646f,0.06236806511878967f,0.24479159712791443f,-1.0637274980545044f,0.4270811975002289f,0.9905828237533569f,1.1241116523742676f,-0.549380898475647f,-2.0769355297088623f,-0.3765730857849121f,0.8252902626991272f,-1.4241422414779663f,0.6370969414710999f,0.1273748278617859f,2.367454767227173f,1.3674049377441406f,0.9447811245918274f,1.5179831981658936f,0.05107032507658005f,-0.4072481691837311f,-1.5900707244873047f,1.2695693969726562f,-0.18862539529800415f,1.0123581886291504f,2.7732701301574707f,0.2859257757663727f,0.09498070180416107f,-2.0599067211151123f,1.6294031143188477f,-0.492242693901062f,-0.7651855945587158f,-1.0644640922546387f,-0.1391359269618988f,-1.3236232995986938f,0.7136773467063904f,0.5673457384109497f,0.1166570782661438f,0.9580778479576111f,0.3660583198070526f,-0.31849849224090576f,-0.20763248205184937f,-0.6229010820388794f,-0.5255523920059204f,-0.23162733018398285f,0.2446686327457428f,-0.3129655122756958f,-3.566544771194458f,-2.1279706954956055f,0.441494882106781f,-2.008206844329834f,1.3401952981948853f,-0.31790000200271606f,-0.9806270003318787f,-0.47248589992523193f,0.9390115141868591f,1.0542631149291992f,0.4826279282569885f};
alignas(16) float batch_normalization_4_A[] = {-0.13129836320877075f,0.1073637455701828f,0.050003789365291595f,-0.20294052362442017f,0.09740401804447174f,0.23943570256233215f,-0.07797980308532715f,0.23710964620113373f,-0.12158188968896866f,-0.1163959801197052f,-0.5600103735923767f,-0.17606306076049805f,0.09762553870677948f,-0.37957966327667236f,0.5470427870750427f,0.45838648080825806f,0.07610716670751572f,-0.38810211420059204f,0.15021590888500214f,-0.3184272050857544f,-0.43854838609695435f,-0.4964540898799896f,-0.07679437100887299f,0.13713757693767548f,-0.8101775646209717f,0.7858002185821533f,0.38606947660446167f,0.6001504063606262f,-0.3555493950843811f,-0.17803239822387695f,0.17754828929901123f,-0.39747798442840576f};
alignas(16) float conv2d_2_internal_1_W[] = {0.5130208730697632f,-1.229115605354309f,0.351449579000473f,-1.2750781774520874f,-2.496497392654419f,-0.3186627924442291f,0.4324100613594055f,-0.019509483128786087f,-2.0902788639068604f,0.5603690147399902f,0.4990060031414032f,-1.1801263093948364f,-0.37961268424987793f,-1.6366417407989502f,-0.3870941698551178f,-0.4707166850566864f,-1.0677992105484009f,-0.9480201601982117f,-0.10958582907915115f,-0.3605595827102661f,1.7968919277191162f,-1.7399381399154663f,-0.005341863725334406f,0.027400648221373558f,0.8154468536376953f,0.11376681923866272f,-2.352510690689087f,0.28601545095443726f,-0.19377398490905762f,-2.0092532634735107f,-0.045919980853796005f,0.30597975850105286f,0.8104857206344604f,-2.4546468257904053f,-0.9368513822555542f,-1.0878773927688599f,-0.3229837119579315f,2.175536632537842f,0.07265309244394302f,1.4803825616836548f,0.45267167687416077f,2.094621419906616f,0.45614534616470337f,2.299860715866089f,0.3847159743309021f,-0.0054640453308820724f,0.43683916330337524f,-0.5547744035720825f,0.2003183662891388f,1.3165556192398071f,1.6230723857879639f,-1.048136830329895f,1.3670949935913086f,-0.6328951716423035f,1.4739793539047241f,0.18378661572933197f,-2.499202251434326f,-1.0397021770477295f,-1.0641980171203613f,-1.108871340751648f,-1.0642112493515015f,0.30824944376945496f,1.8024080991744995f,-0.6101169586181641f,-0.3914162218570709f,-0.18320053815841675f,-0.5763814449310303f,1.0272756814956665f,-2.107509136199951f,0.9769259095191956f,0.5289664268493652f,-0.39054393768310547f,-0.230448916554451f,-0.41522476077079773f,-0.09576790779829025f,0.3867550194263458f,2.0425586700439453f,-0.686274528503418f,1.467971682548523f,-0.44760966300964355f,-0.018796099349856377f,0.5309951901435852f,-1.6288875341415405f,-0.6542044281959534f,1.7143176794052124f,-2.406749725341797f,-1.1448944807052612f,-1.271447777748108f,0.9048523306846619f,0.3434986174106598f,-1.010552167892456f,-1.2274975776672363f,1.441704511642456f,0.7768670916557312f,-0.497159481048584f,2.4220573902130127f,-0.9073615670204163f,-1.5306243896484375f,1.031822919845581f,0.05956907197833061f,0.34521961212158203f,-1.5119445323944092f,-1.5472609996795654f,-1.2494066953659058f,0.10518216341733932f,-2.4754574298858643f,-0.2094673216342926f,0.9123243093490601f,-0.21886515617370605f,-0.7185989618301392f,-1.3496716022491455f,-2.264232635498047f,-1.882226824760437f,-1.6956194639205933f,-1.1487085819244385f,1.260273814201355f,0.8866585493087769f,0.7503343820571899f,-0.1841103881597519f,2.5207533836364746f,0.44304803013801575f,0.13370110094547272f,1.9205029010772705f,-0.9148610234260559f,-0.6038976907730103f,1.398032784461975f,0.8458629846572876f,-0.4080914556980133f,0.30745330452919006f,-0.7768868207931519f,0.49492010474205017f,1.104859471321106f,0.7966069579124451f,2.0504579544067383f,0.9894856810569763f,0.1540486067533493f,2.542208194732666f,-2.586085081100464f,-0.5669149160385132f,-1.4967787265777588f,0.36389660835266113f,-0.03630705177783966f,1.2620189189910889f,-1.3256289958953857f,-0.9365402460098267f,0.2989014983177185f,1.5998234748840332f,0.6858652830123901f,-1.19418466091156f,-0.2575867176055908f,-2.110617160797119f,-0.732901394367218f,0.5964099764823914f,0.5162420272827148f,1.8441787958145142f,1.90277099609375f,-1.210330843925476f,0.2477167844772339f,-1.2714804410934448f,0.14301642775535583f,-2.2936956882476807f,0.5881849527359009f,-1.6420159339904785f,-1.1704816818237305f,-0.03695399686694145f,-1.308302640914917f,0.6761059761047363f,1.464853048324585f,1.565985918045044f,-0.4867292046546936f,-0.2772771418094635f,1.065260648727417f,-0.19748640060424805f,1.7511792182922363f,-0.0036012919154018164f,0.6435419321060181f,-1.3390908241271973f,-0.5568758249282837f,-0.9524654746055603f,-0.6339623332023621f,-0.17325039207935333f,1.066940426826477f,-1.8872507810592651f,0.13172058761119843f,-0.6649278998374939f,1.000565767288208f,0.3037426173686981f,0.3102457821369171f,1.905110239982605f,1.1508855819702148f,1.5982775688171387f,1.5213855504989624f,-0.9684637188911438f,-1.0560685396194458f,-1.052276372909546f,-0.3635288178920746f,-1.8714228868484497f,-2.1256115436553955f,0.8407607674598694f,-1.3667048215866089f,-0.173075869679451f,-0.33529132604599f,0.28723961114883423f,-0.46064493060112f,0.7415045499801636f,-3.271209478378296f,-0.28837552666664124f,0.4224340617656708f,0.24766409397125244f,-1.482719898223877f,0.0837474837899208f,-0.678235650062561f,1.1842797994613647f,2.153843641281128f,-0.552555501461029f,0.7536384463310242f,-0.4881990849971771f,-0.39899587631225586f,-1.3820608854293823f,-1.402172565460205f,-0.4408596158027649f,-1.8397724628448486f,0.6211258172988892f,-1.3001315593719482f,-1.2496730089187622f,2.018937587738037f,-1.1036608219146729f,0.044945430010557175f,0.6454224586486816f,1.640610694885254f,-1.4671051502227783f,1.897242784500122f,-2.2334253787994385f,-0.7149649858474731f,1.0067999362945557f,-1.9928865432739258f,0.7219101190567017f,0.552091658115387f,-0.09162123501300812f,-1.5968369245529175f,1.9226795434951782f,2.590513229370117f,0.9313892126083374f,-0.5237039923667908f,-0.9075883626937866f,-0.45712417364120483f,-1.2918305397033691f,-0.7890974879264832f,0.633717954158783f,0.18288518488407135f,2.5626347064971924f,-0.40021973848342896f,0.5976970791816711f,-1.0497095584869385f,0.11487993597984314f,1.9215208292007446f};
alignas(16) float batch_normalization_5_A[] = {1.0500479936599731f,0.5662854909896851f,-0.001337975263595581f,1.0384392738342285f,-0.30759117007255554f,0.2515256404876709f,0.8315661549568176f,-0.3294496536254883f};
alignas(16) float separable_conv2d_3_internal_0_VALUES[] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
alignas(16) float separable_conv2d_3_internal_1_W[] = {-0.02935483679175377f,-0.05183839052915573f,-0.0607605054974556f,-0.03752342611551285f,0.0035696965642273426f,-0.028354831039905548f,0.03321009874343872f,0.057318978011608124f,0.0517788827419281f,0.011587302200496197f,0.07924778759479523f,-0.08675555884838104f,-0.0682162344455719f,0.03174551948904991f,0.025376804172992706f,0.03507107123732567f,0.09037771075963974f,0.0025272206403315067f,-0.037136007100343704f,-0.030198898166418076f,0.09807825833559036f,0.022601986303925514f,-0.04626728966832161f,-0.017079848796129227f,0.026959475129842758f,0.021058477461338043f,0.06190898269414902f,-0.061369892209768295f,-0.021114345639944077f,-0.02523377537727356f,-0.041432492434978485f,0.029355794191360474f,0.05272861570119858f,-0.000972700014244765f,-0.0312548466026783f,-0.07400964945554733f,-0.012940951623022556f,0.06936901062726974f,-0.016293108463287354f,0.08436178416013718f,0.0632585734128952f,0.06785325706005096f,0.04753432050347328f,-0.021013785153627396f,-0.05246906355023384f,0.02853158861398697f,0.04463304951786995f,0.007739536464214325f,0.017197636887431145f,-0.0030335888732224703f,-0.05860043317079544f,-0.03424570709466934f,0.09083503484725952f,-0.053098637610673904f,-0.03663939610123634f,-0.010694265365600586f,0.08848590403795242f,0.034154314547777176f,0.05515265092253685f,-0.02881595492362976f,-0.0341060645878315f,-0.04712868109345436f,-0.04394930601119995f,0.05546431243419647f,0.052752017974853516f,0.026787349954247475f,0.007608707994222641f,-0.04056529700756073f,0.005171551834791899f,0.04834100604057312f,0.08905386924743652f,-0.028060145676136017f,0.035171668976545334f,0.02864273078739643f,-0.06875873357057571f,-0.006391346454620361f,-0.013266789726912975f,-0.04196588695049286f,0.025514598935842514f,-0.030501017346978188f,-0.04042225703597069f,-0.02875632978975773f,-0.06761206686496735f,0.010301914997398853f,0.03816552832722664f,-0.055316559970378876f,0.03032141551375389f,-0.01958741433918476f,0.042829081416130066f,-0.03275166451931f,0.03007020242512226f,0.018088821321725845f,-0.036350928246974945f,0.04160898178815842f,-0.032863665372133255f,0.008298016153275967f,-0.017803553491830826f,-0.05133931338787079f,0.007281162776052952f,-0.036071501672267914f,0.03171258792281151f,-0.008128395304083824f,0.007792361546307802f,0.04465971142053604f,-0.0765419453382492f,0.06412971764802933f,0.0373770147562027f,0.01811952516436577f,0.0143788056448102f,0.06244904175400734f,0.028123319149017334f,0.07584037631750107f,-0.016147049143910408f,0.021104268729686737f,-0.004533976782113314f,-0.08003529161214828f,0.04992854967713356f,0.05593569949269295f,-0.0824502632021904f,-0.1179032027721405f,0.03514930233359337f,-0.005762405693531036f,-0.05970733240246773f,-0.06318461149930954f,0.05786161869764328f,0.043553970754146576f,-0.007330368738621473f,0.06413832306861877f,-0.015422466211020947f,0.002082226797938347f,-0.01865893416106701f,-0.07425564527511597f,0.06152435764670372f,0.02224831096827984f,-0.027222730219364166f,0.05041894316673279f,0.0007851453265175223f,0.09845981746912003f,-0.005275784060359001f,0.017544088885188103f,-0.0025495397858321667f,-0.020831452682614326f,0.06721283495426178f,0.02194918692111969f,-0.05345144867897034f,0.0025637417566031218f,-0.041525691747665405f,-0.0805511549115181f,-0.046971216797828674f,-0.06665385514497757f,-0.11469800770282745f,-0.07137575000524521f,0.05796361714601517f,-0.019046353176236153f,-0.025672750547528267f,-0.06101445481181145f,0.04894973710179329f,0.04003619775176048f,-0.037553686648607254f,0.034439265727996826f,-0.048653554171323776f,0.058497410267591476f,-0.008726941421627998f,-0.04031896963715553f,0.05017918720841408f,0.04617425799369812f,0.02423115074634552f,-0.007799334824085236f,0.020725801587104797f,0.017909079790115356f,-0.06915197521448135f,-0.012051442638039589f,-0.0254613496363163f,-0.021926600486040115f,0.03681745380163193f,-0.033861611038446426f,-0.03265395760536194f,-0.034830573946237564f,-0.012485978193581104f,-0.013327749446034431f,0.01743282750248909f,-0.08548413217067719f,0.000672210764605552f,-0.012091693468391895f,0.08052072674036026f,-0.0028380195144563913f,-0.042971204966306686f,0.04593249037861824f,-0.0273390244692564f,0.03769471123814583f,-0.04893685504794121f,-0.03402869030833244f,0.019749736413359642f,-0.02702687494456768f,0.07473734766244888f,-0.026885468512773514f,0.014751606620848179f,-0.08070585876703262f,0.041441675275564194f,0.01644880138337612f,-0.05654432624578476f,0.007445388473570347f,-0.06407221406698227f,-0.009093102067708969f,0.021741347387433052f,0.015274157747626305f,0.05378659442067146f,-0.044105272740125656f,-0.0016724533634260297f,0.018052559345960617f,0.025646129623055458f,-0.06760022044181824f,-0.0005672693368978798f,0.05407020077109337f,0.0715746134519577f,-0.13429617881774902f,0.0018058052519336343f,-0.04476189985871315f,-0.017629949375987053f,-0.003244925057515502f,0.039602141827344894f,-0.02242206037044525f,-0.003728720359504223f,0.047378648072481155f,-0.012215744704008102f,-0.008070597425103188f,0.046412233263254166f,-0.024626968428492546f,0.0480586513876915f,-0.026416031643748283f,-0.003249415662139654f,-0.005648591089993715f,-0.04020906239748001f,0.020586002618074417f,-0.04310068488121033f,-0.014801884070038795f,0.028299717232584953f,0.05128096044063568f,0.03588766232132912f,-0.05203930288553238f,0.009463774971663952f,-0.06004244461655617f,0.04155104607343674f,-0.058842942118644714f,-0.05152873694896698f,-0.061798401176929474f,-0.00830892939120531f,-0.11064682900905609f,0.0033622817136347294f,-0.10559900104999542f,-0.0062022036872804165f,-0.025962231680750847f,0.06411360949277878f,-0.011655227281153202f,-0.050353921949863434f,-0.01777992583811283f,-0.0683366134762764f,0.021598339080810547f,0.01753498986363411f,-0.02463308908045292f,0.0370372012257576f,0.010720283724367619f,0.013957450166344643f,-0.044079072773456573f,-0.020042508840560913f,-0.01816398836672306f,-0.0034315611701458693f,-0.07038561999797821f,0.023245828226208687f,-0.02183108776807785f,0.03560073673725128f,0.03383345529437065f,0.04034699499607086f,-0.07827930152416229f,-0.011424887925386429f,-0.02389904297888279f,-0.04696698114275932f,-0.07881898432970047f,0.0028075426816940308f,-0.013623390346765518f,0.0045811268500983715f,-0.055793941020965576f,-0.021776951849460602f,0.007939563132822514f,0.0362386479973793f,-0.016917036846280098f,-0.06167196109890938f,-0.047173671424388885f};
alignas(16) float separable_conv2d_3_internal_2_W[] = {0.04114563390612602f,-0.3707338273525238f,-0.8542796969413757f,0.2007577270269394f,0.10252249240875244f,1.1233876943588257f,-0.2640749216079712f,-0.7795569896697998f,0.13795827329158783f,0.8152347803115845f,-1.2153258323669434f,-0.15550458431243896f,0.16050392389297485f,-0.3232785165309906f,0.31462562084198f,0.18853215873241425f,-0.1636360138654709f,-0.6066967844963074f,-0.3215283751487732f,-1.0472654104232788f,0.5678232312202454f,0.04586641117930412f,-0.2423950880765915f,-0.1392490565776825f,-1.7031949758529663f,0.8648649454116821f,0.6307303309440613f,1.03032386302948f,0.12705042958259583f,-0.7976139187812805f,0.4642220735549927f,-0.03807980194687843f,0.8905877470970154f,-1.2670994997024536f,-0.5980709791183472f,0.14642003178596497f,0.06952258199453354f,-0.39816147089004517f,0.6629632115364075f,-0.7227495908737183f,-0.09251536428928375f,-0.2521597743034363f,0.12250932306051254f,0.12746024131774902f,-0.4644448757171631f,0.3515355885028839f,-0.1397157460451126f,-1.120626449584961f,0.569338321685791f,0.10774245113134384f,0.49551016092300415f,0.985573947429657f,-0.30648350715637207f,-1.2105296850204468f,0.44848504662513733f,-1.0369688272476196f,-0.4969119131565094f,0.40266847610473633f,0.8418685793876648f,-1.2486423254013062f,-0.1068137064576149f,-0.1347518116235733f,0.3848838806152344f,0.4448104500770569f,0.4245586395263672f,0.354025661945343f,-0.4699424207210541f,0.3817936182022095f,-0.18899628520011902f,0.3203393816947937f,-0.7316924929618835f,0.6511970162391663f,-0.33232372999191284f,1.0963478088378906f,0.8203777074813843f,-1.071533441543579f,0.04802670329809189f,1.1906191110610962f,-0.08825964480638504f,1.1201146841049194f,1.6975024938583374f,1.5723166465759277f,0.9488059282302856f,0.2418341487646103f,-0.24956628680229187f,-1.5193848609924316f,1.0863834619522095f,0.1944209486246109f,1.4247416257858276f,-0.632134199142456f,-1.249361276626587f,-1.572145700454712f,0.9288736581802368f,1.3707562685012817f,-0.545005738735199f,-0.5617142915725708f,0.2033461332321167f,-1.1409879922866821f,0.8876320719718933f,-0.8289178609848022f,-0.1701708436012268f,0.19527649879455566f,1.053807020187378f,-0.4075033664703369f,-1.0004802942276f,-0.07321207970380783f,-0.35419780015945435f,-0.43889036774635315f,0.08308020234107971f,-0.5501236319541931f,-0.2375137358903885f,0.3201514482498169f,0.2657557725906372f,-0.38799551129341125f,-1.0749847888946533f,-0.2227320671081543f,-0.432517945766449f,-0.6357243061065674f,1.3991870880126953f,0.0501769557595253f,-1.4591292142868042f,-0.4698588252067566f,0.05730132758617401f,0.5434334874153137f,-0.8225919604301453f,1.826224684715271f,0.010820352472364902f,0.264335036277771f,0.06724360585212708f,-0.4076492488384247f,0.30377429723739624f,1.2778007984161377f,0.6119689345359802f,-0.23551662266254425f,1.1391642093658447f,0.8473624587059021f,0.11807793378829956f,0.01864035427570343f,-0.6793831586837769f,0.009850435890257359f,-0.08367307484149933f,0.4616205394268036f,0.6956711411476135f,0.1786430925130844f,-0.9893186688423157f,0.056150149554014206f,-0.9234306216239929f,-0.6168453097343445f,0.39272770285606384f,0.5195133090019226f,0.13127733767032623f,-0.30542242527008057f,0.1353011131286621f,-0.8756096363067627f,0.31364721059799194f,-1.0239652395248413f,-0.9642723202705383f,-0.4168068468570709f,-0.7434622049331665f,0.4263027012348175f,-0.24656759202480316f,-0.59637451171875f,-0.4995249807834625f,0.069857157766819f,-0.6286346316337585f,-0.040788788348436356f,0.9480696320533752f,0.2223028987646103f,1.4947185516357422f,-0.3005228042602539f,-0.2708793878555298f,0.39146074652671814f,0.5342618227005005f,0.8694555163383484f,1.124163031578064f,-0.12479269504547119f,0.5444923043251038f,-0.06649225950241089f,0.8547946214675903f,0.5018695592880249f,1.0707283020019531f,-0.23810775578022003f,-0.44507792592048645f,-0.9234177470207214f,0.5503885746002197f,0.3561045527458191f,-0.06877965480089188f,0.7339855432510376f,-0.6422010064125061f,-0.3088400363922119f,1.5873587131500244f,0.15991684794425964f,-1.372510552406311f,-0.34563449025154114f,-0.44237178564071655f,0.5966784358024597f,0.4723339080810547f,1.3018455505371094f,-0.7298223376274109f,0.11744159460067749f,-1.2764242887496948f,0.7196171879768372f,0.4063241183757782f,-0.04552776366472244f,0.3288511335849762f,0.5956760048866272f,-0.14586347341537476f,-0.44455355405807495f,-0.11065016686916351f,-0.48269301652908325f,-0.7480509877204895f,-1.0681394338607788f,-0.2687084376811981f,0.7090470790863037f,-0.08883033692836761f,0.6956025958061218f,-0.7941193580627441f,0.42307719588279724f,-0.5172887444496155f,-1.7215934991836548f,0.9783604145050049f,-1.241066575050354f,1.4700918197631836f,1.0568854808807373f,-1.1085861921310425f,0.8329318165779114f,0.4543518126010895f,2.0698859691619873f,-0.30918553471565247f,-0.7512418031692505f,-0.7230141162872314f,0.7513840198516846f,1.1883279085159302f,-2.895272970199585f,-0.3541507124900818f,0.5486251711845398f,-1.028924584388733f,0.31978318095207214f,-0.32811856269836426f,-0.1592995673418045f,1.89254629611969f,0.024413174018263817f,0.2983458340167999f,0.7331257462501526f,-0.7898598909378052f,0.3028933107852936f,-1.088118553161621f,-0.23374275863170624f,0.04577168449759483f,1.0688344240188599f,-0.1117570698261261f,0.8260109424591064f,-0.2167317271232605f,-0.08921071887016296f,2.0846450328826904f,-0.6435548067092896f,-0.062197960913181305f,-0.18397681415081024f,-0.37687256932258606f,-0.6838318705558777f,0.7091323733329773f,0.1831357628107071f,-0.8880596160888672f,1.2895786762237549f,0.5283112525939941f,0.22264543175697327f,0.5631645321846008f,-1.0619031190872192f,-0.03575000539422035f,0.1617574542760849f,-1.260552167892456f,-1.351140022277832f,0.35128653049468994f,-0.20561444759368896f,-1.1258386373519897f,-0.4841710925102234f,-0.4035532474517822f,0.17607660591602325f,1.2167744636535645f,0.1561327576637268f,-0.6867267489433289f,1.281037449836731f,0.2878226339817047f,-1.0216443538665771f,-0.11645685136318207f,-0.2291049212217331f,0.6972664594650269f,-0.10512696951627731f,0.19896794855594635f,0.027318313717842102f,0.8753662109375f,-0.44099748134613037f,-0.6220501661300659f,-0.5273616313934326f,-0.910081684589386f,-0.12877817451953888f,1.0041184425354004f,0.4896809756755829f,0.03785762935876846f,0.3631848394870758f,-0.18414217233657837f,-0.035892561078071594f,0.7336063385009766f,1.5643222332000732f,0.244400292634964f,-0.6872051358222961f,-0.6869564056396484f,0.48780739307403564f,0.17730584740638733f,0.23378466069698334f,0.10052657127380371f,0.7938016653060913f,0.04401114583015442f,0.6371650695800781f,0.3884086310863495f,0.0925208255648613f,-0.12368153035640717f,-1.468376636505127f,0.08036810904741287f,-0.072557233273983f,-0.16861343383789062f,0.9390589594841003f,-0.5224202275276184f,0.7737913131713867f,-0.417224258184433f,0.003290831111371517f,0.049753736704587936f,0.015437961556017399f,-0.4046185612678528f,-0.000601540319621563f,1.1189924478530884f,-0.33618980646133423f,-0.7711687088012695f,0.3442756235599518f,-0.3784380853176117f,-0.4688257873058319f,0.9422997832298279f,1.3584052324295044f,-0.6756497025489807f,-0.4708390235900879f,-0.5823542475700378f,-0.7786768078804016f,1.1344571113586426f,1.1638628244400024f,0.13110293447971344f,-1.3101580142974854f,-1.095196008682251f,0.2904509902000427f,0.3160480856895447f,0.8039680123329163f,0.3082893490791321f,-1.2294913530349731f,0.012922090478241444f,0.6894289255142212f,1.2358741760253906f,-1.6320160627365112f,-1.191612720489502f,2.3101773262023926f,0.7074489593505859f,-0.7185457348823547f,-0.6244204044342041f,-0.1567457765340805f,-0.36722099781036377f,0.3416944742202759f,0.5770370364189148f,0.3826741576194763f,0.06815148144960403f,-0.2928660809993744f,-0.20778901875019073f,1.0139533281326294f,-0.6883397102355957f,-0.11921250075101852f,-0.9385573267936707f,-0.39437174797058105f,0.24726593494415283f,-0.8112238049507141f,0.25431323051452637f,0.28772398829460144f,1.2685184478759766f,-0.09498601406812668f,-0.3744863271713257f,0.27249497175216675f,-0.3729794919490814f,0.6887258887290955f,0.5146931409835815f,-1.2753338813781738f,0.3563036322593689f,0.493339478969574f,-0.40619364380836487f,1.0936665534973145f,0.676503598690033f,-0.35353317856788635f,0.7168443202972412f,-1.3669166564941406f,-0.3440139889717102f,0.40576106309890747f,-0.6562995910644531f,-0.44597822427749634f,-0.33406928181648254f,-0.3511539399623871f,-0.21644143760204315f,0.10865620523691177f,-0.42864271998405457f,0.35164642333984375f,-0.09412340074777603f,1.0390046834945679f,0.3055202066898346f,0.1896185427904129f,1.0248907804489136f,0.7679454684257507f,-0.8186479210853577f,-0.13839010894298553f,0.7014126777648926f,-0.25143489241600037f,0.2935670018196106f,0.7484427690505981f,0.4213787615299225f,-1.398922324180603f,-0.10370685160160065f,0.4877516031265259f,1.1750727891921997f,1.0428625345230103f,0.3026556372642517f,0.7741247415542603f,0.5207916498184204f,0.22144052386283875f,-0.7920995354652405f,-0.012629146687686443f,-0.02333490364253521f,0.6050227880477905f,-0.3575209975242615f,0.6331956386566162f,-0.2992269992828369f,0.3177245855331421f,-0.28254857659339905f,-0.3312937915325165f,0.09009186178445816f,-0.8753327131271362f,0.5563344359397888f,-1.4312987327575684f,-0.3179566562175751f,0.5959880352020264f,-0.6746141314506531f,1.2199926376342773f,-0.07326730340719223f,0.7619592547416687f,0.8790014982223511f,1.5020837783813477f,-0.6138581037521362f,0.6775954961776733f,-1.4522584676742554f,-0.7356336712837219f,1.8669013977050781f,0.10704926401376724f,-0.5331226587295532f,0.18062689900398254f,-1.5796024799346924f,0.7361577749252319f,-0.31020352244377136f,-0.04778427630662918f,2.5473034381866455f,0.646745502948761f,-0.019262246787548065f,1.5720999240875244f,-2.057647705078125f,-0.5521314740180969f,-1.2430660724639893f,0.28512048721313477f,-0.07012505829334259f,0.8216871023178101f,0.8322785496711731f,1.2438408136367798f,-0.14815238118171692f,0.7851803302764893f,-1.1853381395339966f,-0.6082082986831665f,0.16159693896770477f,1.303998589515686f,-1.0435229539871216f,2.3007028102874756f,0.3336467146873474f,0.9249401688575745f,0.23319277167320251f,1.3804196119308472f,-0.09678838402032852f,1.53714919090271f,-0.11078404635190964f,-0.39241141080856323f,-0.1743612438440323f,-2.983971118927002f,0.4915883243083954f,-2.1610758304595947f,-0.8039648532867432f,-0.2962624728679657f,0.8336135149002075f,1.6680773496627808f,0.5052087306976318f,-1.1942870616912842f,0.19752420485019684f,0.3701653778553009f,0.6503604650497437f,1.1802961826324463f,-0.01061796210706234f,0.34999847412109375f,0.1964998096227646f,1.2400842905044556f,-0.011370576918125153f,0.4213053286075592f,0.3104530870914459f,-0.5951037406921387f,-0.4524906277656555f,0.7613822817802429f,0.29249075055122375f,-0.1576472669839859f,0.019397197291254997f,0.40201640129089355f,-3.6417083740234375f,0.8240573406219482f,1.0624574422836304f,0.7686517238616943f,-0.5979763865470886f,-0.5331359505653381f,-0.190601646900177f,-1.2357995510101318f,0.23329801857471466f,-1.0569396018981934f,0.4874386489391327f,0.5415170192718506f,0.3611760139465332f,-0.837584376335144f,0.12380149215459824f,-1.9338055849075317f,0.4499491751194f,-0.57975172996521f,0.2585710883140564f,0.12645743787288666f,0.32172074913978577f,0.19099609553813934f,-1.5168380737304688f,0.7919151186943054f,-1.8012712001800537f,0.3698969781398773f,0.08060622215270996f,-0.5545824766159058f,-0.7641023397445679f,-0.4299675226211548f,0.02396269515156746f,-0.2781926095485687f,0.15841276943683624f,0.5321552157402039f,0.13401244580745697f,-2.0185041427612305f,-0.900355875492096f,-2.681274652481079f,0.05370842665433884f,0.48620423674583435f,-1.393539547920227f,-0.22967691719532013f,1.1015311479568481f,1.7104825973510742f,0.14115266501903534f,3.039074182510376f,-0.07914568483829498f,-1.2156609296798706f,-0.2797727584838867f,1.9978126287460327f,1.5161246061325073f,-0.0931687280535698f,-1.095018744468689f,-0.8515334725379944f,-0.5187264680862427f,-0.3341737687587738f,0.17163003981113434f,0.490330308675766f,-0.8984159231185913f,-1.1071432828903198f,-0.18458011746406555f,0.02305341511964798f,-0.7082414627075195f,-1.732086420059204f,-1.1704699993133545f,-0.21088851988315582f,1.238957405090332f,-0.28629544377326965f,1.0152817964553833f,-0.27078694105148315f,-1.7171813249588013f,0.062482334673404694f,0.11642837524414062f,1.0587701797485352f,-0.8853598833084106f,-0.253586083650589f,1.789674997329712f,-0.42509379982948303f,0.5651367902755737f,0.6438101530075073f,0.5218249559402466f,-0.3120579421520233f,-0.9828017950057983f,1.1096807718276978f,0.024452563375234604f,-0.1002347320318222f,0.034374676644802094f,1.5267837047576904f,-0.3959771394729614f,-1.0372027158737183f,-0.6414584517478943f,-0.17002542316913605f,-0.5790648460388184f,-0.816576361656189f,-0.35552528500556946f,1.9162923097610474f,-0.9214270710945129f,-0.08867087215185165f,0.6429508924484253f,0.6901881694793701f,-0.25265881419181824f,-0.5599374771118164f,-1.3887882232666016f,0.17547957599163055f,-0.756071925163269f,1.1599982976913452f,0.5807795524597168f,-0.529159426689148f,0.9683133959770203f,0.5179402232170105f,-0.5475096106529236f,0.41166844964027405f,-0.5661510229110718f,0.742750346660614f,0.01941930130124092f,-1.1807372570037842f,-0.7884646654129028f,-0.5138060450553894f,-0.15240660309791565f,0.04875297099351883f,-0.3962964713573456f,0.18970908224582672f,0.5478140711784363f,1.1745858192443848f,0.6366966962814331f,-0.2789836525917053f,-1.089847207069397f,0.3368976414203644f,-0.30209267139434814f,0.4881857633590698f,0.25836580991744995f,-0.31698647141456604f,-0.845137894153595f,0.10834913700819016f,-0.2621931731700897f,0.04136107489466667f,0.673793613910675f,-0.01528492197394371f,-0.7380877137184143f,-0.8356032371520996f,0.4374094307422638f,-0.26446467638015747f,0.6673840284347534f,0.6924564242362976f,-0.049606841057538986f,0.16726185381412506f,0.7423899173736572f,-0.17428331077098846f,-1.9723206758499146f,-0.5998635292053223f,-0.6197211742401123f,0.04331163316965103f,0.09991520643234253f,-0.42683830857276917f,0.8926361203193665f,-0.049132443964481354f,-0.015051065012812614f,0.27509817481040955f,-0.2376699149608612f,0.4732913672924042f,-0.6153292059898376f,0.9057424068450928f,0.3948935568332672f,0.15247637033462524f,0.15507933497428894f,-0.0423738956451416f,0.286307692527771f,-1.608660101890564f,-0.7027640342712402f,-0.7143025398254395f,-0.8989073634147644f,1.4659907817840576f,-0.05843392387032509f,0.31960707902908325f,0.340117871761322f,0.18711897730827332f,0.4310569763183594f,-1.0308834314346313f,0.7235599160194397f,-0.9080191850662231f,1.1154683828353882f,0.421766459941864f,0.13036522269248962f,-0.3485519289970398f,-0.1356632560491562f,0.6881083846092224f,0.3834572434425354f,-0.5544503927230835f,-0.02027781493961811f,-0.606901228427887f,-0.5639290809631348f,-0.2539217174053192f,-0.6611170172691345f,-0.19781121611595154f,-0.13308647274971008f,-0.5463945865631104f,0.11364179849624634f,-0.06359490007162094f,-0.3633210361003876f,0.3500532805919647f,-0.3015579581260681f,0.3493924140930176f,0.9317256808280945f,-0.7020434141159058f,-0.2001575082540512f,-1.2023613452911377f,-2.320594310760498f,1.17098069190979f,0.640349268913269f,-1.1130526065826416f,1.4082047939300537f,-0.47577351331710815f,0.7447048425674438f,-0.5813650488853455f,0.48425260186195374f,0.3227272927761078f,0.5225859880447388f,-0.46676862239837646f,0.6876243352890015f,-1.3510209321975708f,0.08890870958566666f,0.07475556433200836f,0.4675607979297638f,0.13821670413017273f,0.960428774356842f,0.7239591479301453f,-0.6014723777770996f,0.36473673582077026f,-1.6293842792510986f,-0.7343560457229614f,1.9205087423324585f,-0.1454249620437622f,-0.5569413900375366f,0.8463084697723389f,0.4757040739059448f,-0.6414592862129211f,0.5206016898155212f,0.5778899788856506f,-0.17656053602695465f,-0.05772111937403679f,1.0383939743041992f,0.11042137444019318f,-0.14606067538261414f,0.5597759485244751f,-0.27182504534721375f,0.46053677797317505f,-0.3580709993839264f,-0.15870392322540283f,-0.1860489547252655f,-0.17232337594032288f,0.9940075278282166f,0.36380571126937866f};
alignas(16) float batch_normalization_6_A[] = {-0.49072062969207764f,0.3564291000366211f,-0.2866218686103821f,1.2873163223266602f,1.5819944143295288f,0.02791730687022209f,-1.7018225193023682f,0.034591324627399445f,0.08246639370918274f,0.1586420238018036f,0.8384915590286255f,0.6605826616287231f,-0.36425840854644775f,0.014849364757537842f,1.8695881366729736f,0.6384830474853516f,-1.0591546297073364f,0.3151761293411255f,-1.5305044651031494f,-0.9494251012802124f,1.4132345914840698f,0.5378519892692566f,-0.6396204233169556f,0.10810460895299911f};
alignas(16) float conv2d_3_internal_1_W[] = {0.021308334544301033f,-0.49909675121307373f,-0.11864056438207626f,0.4581950008869171f,-0.1909477412700653f,-0.23689448833465576f,0.07000739872455597f,-0.4966084659099579f,0.047569334506988525f,-0.16829703748226166f,0.05478639155626297f,-0.2743387222290039f,-0.1461566984653473f,0.07329513132572174f,-0.2764744162559509f,-0.16051416099071503f,-0.09951602667570114f,-0.014958292245864868f,-0.19287890195846558f,0.20219109952449799f,-0.0015338165685534477f,0.4082108736038208f,0.1456967145204544f,0.2046995460987091f,0.27387043833732605f,0.06642724573612213f,0.42730167508125305f,0.11398748308420181f,-0.22893983125686646f,0.496530681848526f,0.38392141461372375f,0.0993538498878479f,0.5070680379867554f,0.48884135484695435f,0.4432353675365448f,-0.19224394857883453f,0.038617026060819626f,-0.22425700724124908f,0.13053733110427856f,0.3751946985721588f,0.06585358083248138f,-0.5452827215194702f,0.10232335329055786f,0.060838982462882996f,-0.25389188528060913f,0.09067890048027039f,-0.07717067748308182f,-0.3089704215526581f,-0.16792438924312592f,0.35523760318756104f,-0.17758673429489136f,0.5167239904403687f,0.09378102421760559f,-0.25134655833244324f,-0.25896331667900085f,-0.3425098955631256f,0.2357579618692398f,0.17908567190170288f,-0.003920488990843296f,-0.24367715418338776f,-0.009295087307691574f,-0.5457624793052673f,-0.27279016375541687f,0.01987922191619873f,0.2433737814426422f,-0.14611083269119263f,-0.35696905851364136f,0.363612562417984f,-0.03585052862763405f,0.2503693997859955f,-0.007258236408233643f,0.547500491142273f,-0.22426030039787292f,-0.08723614364862442f,0.11019597947597504f,-0.6507747173309326f,0.29763129353523254f,-0.3940315842628479f,-0.03214414045214653f,-0.07460962980985641f,-0.22839251160621643f,-0.6515280604362488f,-0.2449532002210617f,-0.05717417225241661f,0.0989227220416069f,-0.4243752360343933f,-0.3329732418060303f,0.1958327442407608f,0.3620152175426483f,0.2812398672103882f,0.45472124218940735f,0.2901220917701721f,0.05614618957042694f,0.3794846832752228f,0.00024735863553360105f,-0.07592999935150146f,-0.13550058007240295f,-0.3409351706504822f,-0.5856871604919434f,-0.17301112413406372f,-0.24236595630645752f,-0.9537381529808044f,-0.4782324731349945f,0.4602450728416443f,-0.03353431075811386f,0.23237019777297974f,0.38430777192115784f,-0.23085299134254456f,-0.21457763016223907f,-0.11010261625051498f,0.3454676568508148f,0.39413103461265564f,0.19619964063167572f,0.0928964838385582f,-0.22343365848064423f,-0.0024324182886630297f,0.04345136135816574f,-0.6233649849891663f,-0.030132703483104706f,0.039954494684934616f,-0.23662889003753662f,-0.036614593118429184f,-0.04750867187976837f,0.20534823834896088f,-0.35215798020362854f,0.1902690976858139f,-0.3631539046764374f,-0.12984368205070496f,0.21421752870082855f,-0.35090917348861694f,-0.06858561187982559f,0.22368520498275757f,0.24211613833904266f,0.07789196074008942f,0.1119852364063263f,0.05419643595814705f,-0.25464269518852234f,-0.011494319885969162f,-0.2454516589641571f,0.4269767105579376f,0.02320041134953499f,0.2673678994178772f,-0.28493109345436096f,-0.11717568337917328f,0.3475174307823181f,-0.07602356374263763f,0.13570471107959747f,0.11658989638090134f,0.12907133996486664f,0.22836750745773315f,-0.3342269957065582f,-0.04803406074643135f,-0.16780129075050354f,0.1534767895936966f,0.39185887575149536f,-0.2657477855682373f,0.035375941544771194f,-0.1889577955007553f,-0.33360719680786133f,-0.24743147194385529f,-0.08305219560861588f,0.22080782055854797f,-0.13427948951721191f,-0.2104838341474533f,-0.2132178395986557f,-0.35482093691825867f,-0.08965837210416794f,0.19657161831855774f,0.09591355919837952f,0.20105044543743134f,0.02451580949127674f,-0.076695516705513f,-0.06305520981550217f,0.7916758060455322f,0.10779773443937302f,-0.6643263101577759f,-0.029824210330843925f,0.09701105952262878f,-0.245217964053154f,0.22585177421569824f,-0.11975820362567902f,-0.5843067765235901f,0.2122180163860321f,0.1609434336423874f,0.25985002517700195f,0.10182629525661469f,-0.11260165274143219f,0.1576201617717743f,-0.06638611853122711f,-0.4367961883544922f,0.5857040882110596f,-0.34541982412338257f,0.11954733729362488f,0.0721944123506546f,0.1194879338145256f,0.20406903326511383f,-0.31877902150154114f,0.19983282685279846f,0.38003602623939514f,0.3073737919330597f,0.31672438979148865f,-0.21056894958019257f,0.1433810144662857f,-0.0664367601275444f,0.11960935592651367f,0.401839941740036f,-0.10353866964578629f,0.5745140910148621f,0.30506080389022827f,0.06844989955425262f,0.264413982629776f,-0.031012384220957756f,0.23324812948703766f,-0.6410925984382629f,0.06580468267202377f,-0.6357600092887878f,-0.10539858788251877f,-0.17542974650859833f,1.0568568706512451f,-0.004875839687883854f,0.1024598553776741f,0.03287322074174881f,-0.2776896357536316f,-0.2466781586408615f,-0.3642849028110504f,-0.00795880239456892f,-0.447550892829895f,-0.43622127175331116f,0.033072005957365036f,-0.19423799216747284f,-0.11932232230901718f,-0.8467841744422913f,-0.2227102667093277f,0.11066776514053345f,-0.4556748569011688f,0.15736909210681915f,-0.014458331279456615f,-0.02665691077709198f,-0.1101844385266304f,0.4572362005710602f,0.5818269848823547f,0.06493376195430756f,-0.08551828563213348f,-0.11650010198354721f,-0.3164655268192291f,0.4681044816970825f,0.08428119868040085f,-0.3761568069458008f,-0.049181438982486725f,0.06280387192964554f,0.20383532345294952f,0.037672653794288635f,0.04862315580248833f,0.14281217753887177f,0.07140754163265228f,0.3785007894039154f,0.04288862645626068f,-0.09758005291223526f,-0.09853870421648026f,-0.21008166670799255f,0.05392540618777275f,-0.15356487035751343f,0.54208904504776f,-0.11692647635936737f,-0.02806718461215496f,-0.1479344367980957f,-0.09225933253765106f,0.32286563515663147f,0.35628461837768555f,-0.8968763947486877f,-0.6243941783905029f,-0.16960906982421875f,-0.440308153629303f,0.04451526701450348f,0.2704094648361206f,0.12233295291662216f,0.3279355764389038f,0.6579003930091858f,-0.050350598990917206f,0.2976178824901581f,-0.47141769528388977f,-0.20938368141651154f,0.18321488797664642f,-0.309835284948349f,0.3539893627166748f,0.6731845140457153f,0.022493543103337288f,-0.2002611607313156f,-0.6117262244224548f,0.11906818300485611f,0.2157668173313141f,-0.07874416559934616f,-0.2079485058784485f,-0.06670192629098892f,0.06296900659799576f,-0.4516774117946625f,0.15688841044902802f,0.5879196524620056f,0.44961076974868774f,0.27997642755508423f,-0.44540882110595703f,-0.3852202892303467f,-0.10105672478675842f,0.08669496327638626f,0.24597343802452087f,-0.4542124271392822f,0.6836285591125488f,0.004987526219338179f,0.08079800754785538f,0.4995625913143158f,0.21823357045650482f,-0.6165297627449036f,-0.38712984323501587f,-0.0768071785569191f,-0.4504164457321167f,-0.13582554459571838f,0.19925670325756073f,-0.23931153118610382f,0.4295584261417389f,-0.021314093843102455f,0.06281847506761551f,0.13668616116046906f,0.011398215778172016f,-0.22065864503383636f,0.04130500555038452f,0.4744643270969391f,-0.47146230936050415f,0.1502917855978012f,0.014326181262731552f,0.06994415819644928f,-0.10404746979475021f,-0.367143452167511f,-0.35036712884902954f,-0.4821207821369171f,0.08816078305244446f,-0.523866593837738f,0.19249790906906128f,0.17662404477596283f,-0.030385665595531464f,0.11342277377843857f,0.4480866491794586f,0.00075595109956339f,0.2450055181980133f,0.3854452669620514f,-0.2601277530193329f,-0.5062729120254517f,-0.20315003395080566f,0.5196954607963562f,-0.12116339802742004f,0.26334235072135925f,-0.27487418055534363f,0.1462434083223343f,-0.20622463524341583f,0.3879951536655426f,-0.08076409250497818f,-0.12363830953836441f,-0.3693770468235016f,0.560225248336792f,-0.308084636926651f,-0.45754966139793396f,-0.15464109182357788f,-0.30216848850250244f,0.5799550414085388f,0.36599135398864746f,0.3650206923484802f,-0.32560214400291443f,-0.09561479091644287f,-0.13038493692874908f,-0.14336207509040833f,-0.1691218465566635f,0.173542782664299f,0.25055065751075745f,-0.10828577727079391f,-0.03349830582737923f,-0.3746046721935272f,-0.06764153391122818f,0.12633632123470306f,-0.40715354681015015f,0.19313611090183258f,0.20155121386051178f,-0.07688786834478378f,-0.0422467440366745f,0.17958030104637146f,0.665822446346283f};
alignas(16) float batch_normalization_7_A[] = {0.33636051416397095f,0.3682556450366974f,0.3861720561981201f,-0.06535391509532928f,1.1330357789993286f,0.07727845013141632f,0.607160210609436f,0.48637300729751587f,0.7014861106872559f,0.6768978834152222f,-1.324289083480835f,-0.14877885580062866f,0.5525873899459839f,0.46057313680648804f,0.3606899380683899f,0.8763309717178345f};
alignas(16) float separable_conv2d_4_internal_0_VALUES[] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
alignas(16) float separable_conv2d_4_internal_1_W[] = {0.12328769266605377f,-0.014816495589911938f,-0.03895790874958038f,0.02336181327700615f,0.053701434284448624f,-0.0514657087624073f,0.03168974071741104f,0.024460235610604286f,-0.017798403277993202f,-0.028671910986304283f,0.03984030708670616f,-0.09489902853965759f,-0.027800044044852257f,-0.053634412586688995f,-0.012018018402159214f,0.07787448912858963f,0.04519147425889969f,-0.01409800723195076f,0.0757518857717514f,0.09018698334693909f,0.09330520778894424f,0.005512610077857971f,0.08257414400577545f,-0.0011927731102332473f,0.00826007779687643f,0.044472940266132355f,-0.04599951580166817f,-0.055277563631534576f,-0.05940929427742958f,-0.04174560308456421f,-0.06157204136252403f,0.07360140234231949f,0.09452521055936813f,0.03395914286375046f,-0.022565199062228203f,0.023000510409474373f,0.021756796166300774f,0.122175432741642f,-0.020373810082674026f,-0.05056625232100487f,0.07546096295118332f,-0.03845689818263054f,-0.03072800487279892f,-0.06945977360010147f,0.052035972476005554f,0.016658376902341843f,-0.008796850219368935f,-0.03697444871068001f,-0.04087921977043152f,0.03291968256235123f,-0.11510506272315979f,-0.004561388865113258f,0.04511495307087898f,0.017111919820308685f,-0.06263246387243271f,-0.06083673611283302f,0.04179897904396057f,0.022135194391012192f,0.04545559361577034f,-0.05092575401067734f,0.053300850093364716f,0.018555860966444016f,-0.006327595561742783f,-0.0872693806886673f,-0.02530810795724392f,0.08009468764066696f,0.009850493632256985f,0.0557258315384388f,0.04899829253554344f,-0.012603215873241425f,-0.03117840364575386f,0.08173663914203644f,-0.05925831198692322f,-0.03093907982110977f,-0.03892515227198601f,-0.0009985221549868584f,0.045651279389858246f,-0.074689120054245f,-0.04903454706072807f,0.030564866960048676f,-0.038503699004650116f,-0.007420239970088005f,0.07480704039335251f,-0.04865305498242378f,-0.037822045385837555f,0.005211554002016783f,0.06640756130218506f,-0.047614485025405884f,-0.0646403431892395f,0.05395963042974472f,0.0018722345121204853f,-0.027333732694387436f,0.022510357201099396f,0.06102677434682846f,-0.04179191589355469f,0.055257149040699005f,0.05490662157535553f,0.12083403766155243f,0.05500693991780281f,-0.051857806742191315f,0.015901168808341026f,0.005846474319696426f,-0.025613317266106606f,0.09007352590560913f,-0.10412897169589996f,0.017596350982785225f,0.03421730920672417f,-0.02383919432759285f,-0.07893269509077072f,0.009282533079385757f,0.003813480492681265f,-0.07284172624349594f,-0.06789972633123398f,0.06080839782953262f,0.03767534717917442f,-0.010967636480927467f,0.04445841163396835f,-0.06877759844064713f,-0.03653770685195923f,0.010113509371876717f,0.041752781718969345f,0.025324005633592606f,0.048085104674100876f,-0.023682959377765656f,0.04869468882679939f,-0.0398484542965889f,0.00569251598790288f,-0.0192819032818079f,0.01373337022960186f,0.02745743840932846f,0.07072628289461136f,-0.012344171293079853f,-0.055412739515304565f,-0.11449079215526581f,-0.018680676817893982f,-0.06783029437065125f,0.03984672576189041f,-0.1033606231212616f,0.04557602480053902f,0.036020487546920776f,-0.01587400585412979f,-0.05699135363101959f,-0.024157224223017693f,-0.07866785675287247f,0.06528518348932266f,0.06270698457956314f,-0.08112853765487671f,0.02930624596774578f,-0.019735105335712433f,0.014218669384717941f,0.02698773518204689f,-0.007358986884355545f,-0.05011322349309921f,-0.02438802272081375f,0.020537253469228745f,-0.0006111815455369651f,0.050512589514255524f,-0.018017368391156197f,-0.027943801134824753f,0.013601571321487427f,0.02227277308702469f,0.005478063132613897f,0.031021390110254288f,0.014813549816608429f,0.05515426769852638f,-0.07040201872587204f,0.04419473931193352f,-0.009723314084112644f,0.025094369426369667f,-0.019179310649633408f,0.06678739935159683f,-0.008434807881712914f,0.02192184142768383f,-0.048248887062072754f,0.022616500034928322f,0.027700727805495262f,0.11007329076528549f,0.04272618517279625f,0.016967201605439186f,-0.03811488300561905f,-0.01103806309401989f,0.045075468719005585f,0.0005555588868446648f,0.08470068126916885f,-0.04340699687600136f,0.0050818524323403835f,0.03706870600581169f,-0.03451504930853844f,-0.021566569805145264f,-0.06290925294160843f,-0.01009424775838852f,-0.00921537633985281f,-0.0007524469983763993f,0.010451494716107845f,-0.048264164477586746f,-0.01057465560734272f,0.02460482344031334f,-0.03004537522792816f,-0.07234390825033188f,-0.037754036486148834f,0.007736723870038986f,0.023327194154262543f,0.023692147806286812f,-0.04634363204240799f,0.056551478803157806f,-0.02129008248448372f,0.09084310382604599f,0.016211340203881264f,0.06165847182273865f,-0.08993959426879883f,-0.049746252596378326f,-0.013028312474489212f,0.00788810569792986f,0.0497879795730114f,0.033942800015211105f,0.02200351469218731f,0.013831608928740025f,-0.03371550887823105f,0.025599952787160873f,0.010288426652550697f,0.007875069044530392f,-0.07924249768257141f,0.0578630156815052f,0.00216450123116374f,-0.009603696875274181f,-0.08367203176021576f,0.09180355817079544f,0.0791316032409668f,0.054882947355508804f,0.0032549521420150995f,0.006355742923915386f,-0.08903004229068756f,0.004587947856634855f,-0.05763212591409683f,0.016752703115344048f,-0.0031994006130844355f,0.0707988440990448f,0.001756336190737784f,0.05981919914484024f,-0.010652774944901466f,-0.060414765030145645f,-0.03550822287797928f,-0.03504860773682594f,-0.009200948290526867f,0.019498659297823906f,0.019874099642038345f,-0.02389843575656414f,0.013304146006703377f,-0.06868673861026764f,-0.02223294787108898f,-0.0024959382135421038f,-0.06158426031470299f,-0.07127143442630768f,-0.05939079076051712f,-0.04139042645692825f,0.018845848739147186f,0.040121663361787796f,0.03415554761886597f,-0.03558064624667168f,-0.01479419693350792f,-0.009737687185406685f,-0.013509021140635014f,0.0696173682808876f,0.06541720032691956f,-0.10639505833387375f,0.027734246104955673f,0.00846242718398571f,0.015275541692972183f,0.021255550906062126f,0.06369906663894653f,-0.07278736680746078f,0.05801357328891754f,-0.032021861523389816f,-0.0030324168037623167f,0.002629514317959547f,-0.12004748731851578f,-0.08744016289710999f,0.05178741365671158f,-0.014888651669025421f,-0.07042304426431656f,0.008277557790279388f,-0.06982569396495819f,-0.004387245047837496f,-0.03340291231870651f,-0.06238754838705063f,0.03475642204284668f,0.0702662467956543f,0.036884378641843796f,-0.04985984042286873f,-0.04946183040738106f,-0.00025147583801299334f,-0.08017104864120483f,0.02979850023984909f,-0.04923420026898384f,0.002814297331497073f,-0.011765774339437485f,-0.027805902063846588f,-0.0029145905282348394f,-0.005521558225154877f,-0.02248505689203739f,-0.0029634141828864813f,-0.0445910207927227f,-0.05484478920698166f,-0.08819572627544403f,-0.08049536496400833f,0.0083671435713768f,0.02349065989255905f,-0.03911570459604263f,-0.0684654712677002f,-0.09232565015554428f,-0.04180477559566498f,0.01450522243976593f,0.03199972212314606f,-0.025596454739570618f,-0.057477328926324844f,-0.07142946124076843f,-0.025895962491631508f,0.05308472737669945f,-0.08492837846279144f,0.017197206616401672f,0.021551528945565224f,0.01569713093340397f,0.033860255032777786f,-0.03650316596031189f,0.009809273295104504f,-0.03737708553671837f,0.018922103568911552f,-0.025271926075220108f,0.01531110517680645f,-0.027095559984445572f,0.033184029161930084f,0.03948557749390602f,-0.030161572620272636f,-0.05174849554896355f,0.06983064115047455f,-0.022824395447969437f,-0.03409450128674507f,-0.006825718097388744f,-0.06639908254146576f,0.011622530408203602f,-0.007553016766905785f,-0.038073427975177765f,0.04034169018268585f,0.07212825119495392f,0.04820796847343445f,-0.005594915244728327f,0.0014400052605196834f,-0.08326669782400131f,-0.034735307097435f,-0.027573974803090096f,-0.027965836226940155f,-0.03222894296050072f,-0.03345711901783943f,0.02136145532131195f,0.01074385829269886f,-0.028814641758799553f,0.006441137287765741f,0.03905068337917328f,0.06901928037405014f,0.02165004424750805f,0.05428740754723549f,0.026869740337133408f,0.05451121926307678f,-0.030274061486124992f,0.027627715840935707f,-0.0745663046836853f,0.031791817396879196f,0.03411009535193443f,0.020078549161553383f,0.073751300573349f,-0.05040367320179939f,0.038158778101205826f,0.03190422058105469f,0.024300919845700264f,-0.006960801314562559f,0.015568067319691181f,-0.054139088839292526f,0.014880591072142124f,0.03750242665410042f,-0.025471044704318047f,0.08374878019094467f,0.033590201288461685f,-0.027440527454018593f,-0.007482071872800589f,-0.03524061664938927f,-0.01033977884799242f,-0.03763282299041748f,0.004056716337800026f,0.04068772867321968f,0.0221173707395792f,-0.11308079212903976f,0.0015283827669918537f,0.015618182718753815f,0.005967454519122839f,0.013340439647436142f,-0.020087165758013725f,0.12108583003282547f,0.034915465861558914f,0.0038192870561033487f,-0.027386991307139397f,-0.035130154341459274f,-0.03795851022005081f,-0.01800774224102497f,3.0373252229765058e-05f,0.05301394313573837f,0.04639621451497078f,-0.0314800962805748f,0.04341302812099457f,-0.03858343884348869f,-0.037323933094739914f,-0.045607391744852066f,0.054900504648685455f,0.04966308921575546f,-0.06400831043720245f,-0.006227297708392143f,-0.006067617330700159f,-0.004625207744538784f,-0.02878020890057087f,-0.06417825818061829f,0.047075819224119186f,0.027162767946720123f,-0.06735222041606903f,-0.11418384313583374f,-0.010046656243503094f,-0.0216707494109869f,-0.09625421464443207f,0.037597086280584335f,0.05691005289554596f,0.03155273199081421f,0.0643860399723053f,0.04086752608418465f,-0.005589374806731939f,0.06496353447437286f,-0.08153092116117477f,-0.05699721723794937f,0.031239261850714684f,-0.027760254219174385f,0.09158843755722046f,-0.04058516025543213f,-0.031744763255119324f,-0.0638229101896286f,-0.04956915229558945f,-0.02658126689493656f,0.050258319824934006f,0.014800941571593285f,-0.010540246963500977f,0.020852604880928993f,0.04564395174384117f,-0.0518757626414299f,0.04642665013670921f,-0.0556548535823822f,-0.021727800369262695f,-0.10931645333766937f,-0.0181533545255661f,-0.045236244797706604f,0.0434676855802536f,-0.013499356806278229f,-0.03412826359272003f,0.015611563809216022f,0.031035861000418663f,-0.03349907323718071f,0.02001398615539074f,0.009552855044603348f,-0.08198825269937515f,0.02348017320036888f,0.060634031891822815f,0.06336086988449097f,0.024105064570903778f,-0.042253222316503525f,0.09218496829271317f,-0.04051223397254944f,0.03597399219870567f,-0.04814964905381203f,-0.017102913931012154f,-0.06059608235955238f,0.04227795451879501f,0.04500520974397659f,0.02095513604581356f,0.06976906955242157f,0.028700843453407288f,0.1060892790555954f,-0.07623307406902313f,0.0057358709163963795f,-0.03523761034011841f,0.08320377767086029f,0.060510873794555664f,-0.011180708184838295f,0.013729441910982132f,-0.044195473194122314f,0.01773780956864357f,-0.03492811694741249f,-0.03194727003574371f,-0.07594987750053406f,0.020843271166086197f,0.021378250792622566f,-0.029833152890205383f,0.022934280335903168f,0.037727005779743195f,0.026309985667467117f,-0.008845679461956024f,-0.044660668820142746f,0.004513537511229515f,-0.027493400499224663f,0.03777250647544861f,0.07186010479927063f,-0.11618596315383911f,0.009935325011610985f,0.03634442389011383f,0.012723094783723354f,-0.02291117049753666f,0.0071691288612782955f,0.08587869256734848f,0.00485638901591301f,-0.018465664237737656f,-0.03333825618028641f,-0.09191292524337769f,-0.029222708195447922f,0.011675352230668068f,-0.009127296507358551f,-0.0008003833354450762f,-0.01453571580350399f,0.06953610479831696f,0.08362580090761185f,0.010632677935063839f,-0.031009860336780548f,0.010826456360518932f,0.08559932559728622f,0.04615725204348564f,-0.061487022787332535f,-0.043882135301828384f,0.0207653995603323f,0.04131549224257469f,-0.030622532591223717f,-0.049504492431879044f,0.03541094437241554f,0.09714692831039429f,0.06328238546848297f,0.02365294098854065f,-0.08070456236600876f,0.026065008714795113f,0.04264639690518379f,0.05027736350893974f,0.025650573894381523f,0.020496295765042305f,-0.025770679116249084f,0.006232763174921274f,-0.04067440330982208f,-0.05710792914032936f,0.038097038865089417f,-0.006072804797440767f,0.010388295166194439f,0.07597482204437256f,0.0008498539100401103f,-0.030872158706188202f,-0.009143504314124584f,-0.04397475719451904f,-0.03214037045836449f,-0.049343448132276535f,0.06394538283348083f,0.02078901045024395f,-0.031047604978084564f,0.027741167694330215f,0.022728752344846725f,0.11962985247373581f,-0.04282335191965103f,-0.01309494860470295f,0.029119418933987617f,-0.0846557691693306f,-0.03477925434708595f,-0.0006193423178046942f,0.07767879217863083f,0.06087798625230789f,0.0224811639636755f,-0.01215406134724617f,0.03052041120827198f,-0.002217252738773823f};
alignas(16) float separable_conv2d_4_internal_2_W[] = {-0.45818570256233215f,0.9037526845932007f,0.9777704477310181f,0.26408275961875916f,0.8210169076919556f,0.2584284245967865f,2.0275158882141113f,0.3786698877811432f,-1.7613612413406372f,0.4675356447696686f,-0.9893388748168945f,0.3027666211128235f,0.6291092038154602f,-0.6014472246170044f,-1.1107733249664307f,-1.353966236114502f,-0.1746682971715927f,0.9575541019439697f,-0.1894775778055191f,1.3432754278182983f,-1.3155314922332764f,0.3313007652759552f,0.9439566135406494f,-2.1033709049224854f,-2.155508041381836f,0.0148384440690279f,0.5664070844650269f,-0.08118444681167603f,-0.8176690936088562f,-0.5242890119552612f,-0.6775573492050171f,0.7711057662963867f,1.907926321029663f,-0.3739053010940552f,0.3120238482952118f,-0.5962878465652466f,-2.29221773147583f,0.11170120537281036f,-0.05357911437749863f,0.8734753727912903f,-0.36791011691093445f,0.6956573724746704f,-0.39602819085121155f,-0.878494143486023f,-0.0523800253868103f,-1.183751106262207f,-0.48877912759780884f,-1.371482014656067f,-1.0130547285079956f,-1.243110179901123f,-1.1024107933044434f,0.4779684245586395f,-0.930990993976593f,-1.3730082511901855f,-1.008250117301941f,-0.10438356548547745f,-1.6648575067520142f,-0.055526260286569595f,-2.1113970279693604f,1.283933162689209f,0.15592479705810547f,0.15779618918895721f,-0.030682818964123726f,0.6412808895111084f,-1.2170811891555786f,-2.153719902038574f,-0.29211002588272095f,1.0951083898544312f,0.06708396226167679f,1.7202821969985962f,-0.6120308041572571f,0.9831281304359436f,2.511361837387085f,-1.0328073501586914f,0.4151701033115387f,1.1071425676345825f,1.7142821550369263f,1.0570993423461914f,-0.9068493843078613f,0.7495927214622498f,-0.33941105008125305f,0.2215277999639511f,0.4760671854019165f,-0.34186097979545593f,-1.7451934814453125f,-0.5683545470237732f,0.7483039498329163f,-0.4925958216190338f,0.6237262487411499f,-0.7608420252799988f,0.5796656608581543f,0.32787302136421204f,0.6423363089561462f,-0.21542462706565857f,0.3755928575992584f,-2.4792399406433105f,-0.14131291210651398f,0.5334272384643555f,-0.8547093868255615f,0.5673109292984009f,0.7332920432090759f,-0.08816451579332352f,1.4877227544784546f,-0.4214129149913788f,-0.41881754994392395f,-0.7294678688049316f,-0.1367339789867401f,-1.7144346237182617f,1.8608896732330322f,-0.04132963716983795f,-0.7945674657821655f,-1.471027135848999f,0.9150275588035583f,-0.09209492057561874f,0.9869140982627869f,-1.0110797882080078f,0.8190597891807556f,-0.8583789467811584f,-1.3186793327331543f,-0.8710289597511292f,-0.7992820143699646f,0.07221570611000061f,0.9286996722221375f,0.877615213394165f,1.2478960752487183f,-1.0145217180252075f,0.8287155628204346f,-0.17757171392440796f,0.26033779978752136f,-0.6989982724189758f,1.128780484199524f,2.177741765975952f,2.3642401695251465f,0.07017365843057632f,0.6226662397384644f,-0.28157517313957214f,-0.42148059606552124f,0.8629224896430969f,0.27304184436798096f,-0.6325933337211609f,-0.9727331399917603f,-0.15200872719287872f,-0.7022653222084045f,0.10456232726573944f,0.058762237429618835f,1.1048036813735962f,1.3861124515533447f,-0.5434530377388f,-0.10427226126194f,0.9665995836257935f,1.0458859205245972f,-0.6773579716682434f,-0.2595031261444092f,1.1740683317184448f,-1.6779950857162476f,0.7072075009346008f,-1.352565884590149f,0.6712821125984192f,0.11479008942842484f,-0.3326703906059265f,1.4165234565734863f,2.262171745300293f,-0.3796878457069397f,0.8230000734329224f,0.8424920439720154f,-0.3727046847343445f,0.5971625447273254f,2.321115255355835f,1.5161292552947998f,0.8199976086616516f,0.560585618019104f,-1.974605917930603f,-0.8330052495002747f,1.3278424739837646f,0.15085342526435852f,-0.1666843444108963f,2.8179938793182373f,0.6667150855064392f,-0.6453374028205872f,0.31178179383277893f,-0.39564281702041626f,1.234923005104065f,-0.4813549220561981f,-0.5470229983329773f,-0.1544041931629181f,-2.221189022064209f,2.4215054512023926f,0.1265123337507248f,0.03611423820257187f,-0.5279200077056885f,-0.06377764046192169f,-0.716326892375946f,0.859980046749115f,-0.3632906675338745f,-0.21254391968250275f,-1.2236977815628052f,0.4258466958999634f,0.601451575756073f,0.48119011521339417f,1.8753880262374878f,1.0784060955047607f,-0.7936840653419495f,-0.8055721521377563f,0.19573837518692017f,-0.6939582824707031f,1.2823231220245361f,0.10451247543096542f,-0.12515240907669067f,1.2959532737731934f,-1.7441166639328003f,1.0025054216384888f,1.2055970430374146f,1.9037672281265259f,0.1268341988325119f,1.4572713375091553f,1.1768662929534912f,0.6892369985580444f,-0.5929073691368103f,-1.7281033992767334f,1.5462335348129272f,1.5808930397033691f,0.08137873560190201f,-1.8552714586257935f,1.0407288074493408f,0.4739236831665039f,-0.5644071698188782f,0.36039042472839355f,1.677718997001648f,-0.11261982470750809f,-1.5171935558319092f,-0.045532193034887314f,-0.7362446188926697f,-0.7587958574295044f,-0.31565770506858826f,0.5659992098808289f,0.700294554233551f,-1.7652226686477661f,0.08052973449230194f,-1.814220666885376f,-0.25100159645080566f,-0.25981253385543823f,-1.8599387407302856f,-0.04541175812482834f,-0.5170531272888184f,0.39342108368873596f,0.5480591654777527f,-0.8523315191268921f,-0.14340971410274506f,-0.6459559798240662f,0.8931331634521484f,0.7750149369239807f,-1.1910324096679688f,-0.21539618074893951f,0.19845211505889893f,-1.0913546085357666f,0.1598922163248062f,-1.2751597166061401f,-1.7713819742202759f,0.24741779267787933f,-1.5831025838851929f,-0.9799963235855103f,0.4001997411251068f,-0.10570167005062103f,-0.11295950412750244f,2.513845920562744f,-1.5806068181991577f,-0.6411637663841248f,1.73582124710083f,1.1006131172180176f,0.7978643178939819f,-1.08958101272583f,-0.003023264231160283f,0.4867038130760193f,-0.08751456439495087f,0.10699129104614258f,1.3376744985580444f,-0.7025737166404724f,0.4608950614929199f,0.30683138966560364f,1.1163791418075562f,2.0597939491271973f,-1.7627575397491455f,-0.23318706452846527f,-1.3621339797973633f,-0.37164977192878723f,-1.3014898300170898f,-0.5037921667098999f,-0.25013595819473267f,-0.03796644136309624f,0.7494964599609375f,-1.0709481239318848f,1.8006395101547241f,0.5585532784461975f,-1.488680362701416f,0.9277142286300659f,-0.8450036644935608f,1.334997296333313f,1.7823925018310547f,1.5157661437988281f,2.0209097862243652f,1.1757451295852661f,-1.6221871376037598f,0.6389644145965576f,0.4328884482383728f,-0.1753571629524231f,-0.2472619265317917f,1.2034868001937866f,1.2016292810440063f,-1.1366424560546875f,-0.6777498126029968f,-0.47835448384284973f,-0.1688840091228485f,-0.035492803901433945f,-2.162341356277466f,-0.9248910546302795f,-0.6180814504623413f,0.8140172958374023f,0.20725853741168976f,-0.07324114441871643f,0.11663004755973816f,0.03177189081907272f,1.980831503868103f,0.04957912117242813f,-0.04044477641582489f,-0.07060708850622177f,0.7741182446479797f,0.38895851373672485f,-1.0802085399627686f,-0.40365493297576904f,1.3534212112426758f,-1.9069232940673828f,0.19342249631881714f,-1.5886707305908203f,0.006476708687841892f,-0.23478366434574127f,-1.967962384223938f,-0.5830697417259216f,-0.5640664100646973f,0.19623777270317078f,0.025398658588528633f,1.8302233219146729f,0.2875412702560425f,0.8271177411079407f,-1.0131782293319702f,-0.05146465450525284f,0.058616310358047485f,0.9227434992790222f,1.6829019784927368f,-0.3120417296886444f,1.4103710651397705f,1.152818202972412f,0.6120114922523499f,-0.9116092920303345f,-0.8364357352256775f,0.43058741092681885f,-0.5728820562362671f,0.10811515152454376f,-0.24945731461048126f,1.034734845161438f,-2.841944456100464f,-1.2640119791030884f,-0.11942184716463089f,1.5548900365829468f,-0.41843464970588684f,0.7979758977890015f,0.7112104296684265f,0.5301234722137451f,-0.6857537627220154f,-0.8591688275337219f,0.20769141614437103f,-0.5191900134086609f,-0.08882095664739609f,-0.9046608209609985f,0.8243966698646545f,0.6021547913551331f,-0.41917684674263f,1.663885474205017f,-1.3204563856124878f,-0.7985268235206604f,0.9668105244636536f,-1.350037693977356f,-1.208952784538269f,1.1482083797454834f,-1.1611601114273071f,1.2403664588928223f,-1.4877272844314575f,-0.29880213737487793f,-0.7173106670379639f,-0.8174130916595459f,0.6466531753540039f,-0.8720555901527405f,-0.19655384123325348f,0.6107693910598755f,0.2708218991756439f,2.1069436073303223f,-2.240617275238037f,-1.1263924837112427f,-0.7042426466941833f,1.747347354888916f,0.19462496042251587f,0.6119424700737f,-0.5979731678962708f,-0.8324791789054871f,-0.29272371530532837f,0.34162113070487976f,-2.1095778942108154f,-0.0005955796223133802f,-1.351768136024475f,1.3637639284133911f,0.102676160633564f,-1.053505778312683f,-1.2870312929153442f,-0.1572932004928589f,0.015187443234026432f,-0.14049851894378662f,-1.264841914176941f,-0.40810906887054443f,1.2636572122573853f,-0.731345534324646f,-0.28325727581977844f,-0.7312699556350708f,-0.12249846756458282f,-1.313462495803833f,-2.1402828693389893f,1.210566520690918f,-0.7046552300453186f,-0.5017718076705933f,0.597848653793335f,-1.2785240411758423f,0.7985445857048035f,0.5338132381439209f,1.1380761861801147f,0.41994741559028625f,0.1570303589105606f,-1.694541573524475f,2.0812454223632812f,0.05640217661857605f,0.5194481015205383f,-0.9513453841209412f,-0.008805031888186932f,0.21649526059627533f,1.181749701499939f,-1.0032575130462646f,-0.5144718289375305f,2.0633482933044434f,-0.4787787199020386f,1.1830049753189087f,-0.2671375572681427f,0.021432142704725266f,0.3087659180164337f,-2.6705923080444336f,0.38444244861602783f,0.981890857219696f,-1.7657523155212402f,-0.8269010782241821f,0.5525885820388794f,2.394991874694824f,0.6460742354393005f,-0.9105230569839478f,-0.519303560256958f,0.5243203639984131f,-0.9140294790267944f,-1.0554271936416626f,-1.2940624952316284f,0.9360337853431702f,-0.45824673771858215f,-0.17639696598052979f,-2.7288944721221924f,-1.4615375995635986f,0.6084029674530029f,-0.6745960712432861f,-0.8190451860427856f,-1.3436466455459595f,0.6876946091651917f,-0.5775073766708374f,1.441763997077942f,-0.42202669382095337f,-1.0595860481262207f,1.8486454486846924f,-0.8566848039627075f,0.12861961126327515f,1.2255852222442627f,-0.1746777892112732f,-0.011553574353456497f,0.6596789360046387f,-1.3792551755905151f,-0.0638892725110054f,1.5316321849822998f,-1.068790316581726f,1.982728123664856f,-0.02205703780055046f,0.09344028681516647f,-0.9562537670135498f,1.2662041187286377f,0.37841692566871643f,-1.690844178199768f,0.45378434658050537f,-2.140475034713745f,0.48743993043899536f,1.3677583932876587f,0.5061408877372742f,-0.8660110235214233f,-0.33069196343421936f,-0.759806215763092f,0.611027717590332f,-0.673078715801239f,-0.11213362962007523f,-0.5518667697906494f,0.09163962304592133f,1.8898316621780396f,-1.7128188610076904f,0.9178412556648254f,-1.1455943584442139f,0.13873279094696045f,-0.1555839329957962f,0.5633805394172668f,0.1800704300403595f,-1.2596385478973389f,1.8153170347213745f,1.0326582193374634f,-0.18129399418830872f,0.13536646962165833f,-2.1241021156311035f,0.4959338903427124f,1.9071292877197266f,-0.9168763160705566f,0.36433279514312744f,0.02048795484006405f,-0.5246992111206055f,-0.5419352650642395f,-0.17320449650287628f,-0.19561193883419037f,0.4030665159225464f,1.5007721185684204f,0.0479341596364975f,0.46964725852012634f,0.27326393127441406f,-1.175347089767456f,-0.16556379199028015f,1.5454232692718506f,1.41597318649292f,0.4605678617954254f,0.9454969763755798f,0.014837912283837795f,0.504946768283844f,0.32441288232803345f,0.5769133567810059f,0.8534194827079773f,-0.5305050015449524f,2.297714948654175f,-1.4632519483566284f,0.6195449233055115f,-0.9382340908050537f,-0.3061293363571167f,1.9522459506988525f,0.15872043371200562f,0.3257664442062378f,-0.191583514213562f,2.001338005065918f,-1.4246679544448853f,0.0867234393954277f,0.7265593409538269f,-0.4492762088775635f,0.9068373441696167f,-1.87379789352417f,0.901649534702301f,0.8785324096679688f,0.09604521095752716f,0.6615089178085327f,0.07442691922187805f,-0.8899999856948853f,-1.551485538482666f,-0.6674992442131042f,0.018939444795250893f,1.4832763671875f,1.2022894620895386f,-1.3596707582473755f,-0.4306415915489197f,-1.0329943895339966f,-0.29554104804992676f,-1.8424192667007446f,-0.5006223320960999f,-1.0642729997634888f,0.1417110562324524f,0.025853857398033142f,-0.8594401478767395f,0.2616759240627289f,0.7895429134368896f,-0.15577882528305054f,0.09635009616613388f,-0.4716329276561737f,-1.6900238990783691f,-0.6554105281829834f,-1.0143375396728516f,-0.9960827231407166f,-1.3508843183517456f,1.7117983102798462f,-0.30648210644721985f,-2.167276382446289f,-0.30126821994781494f,-1.1511642932891846f,-0.2471996545791626f,-0.4698069095611572f,1.309350609779358f,3.105881690979004f,0.20472566783428192f,0.43093639612197876f,-1.424817681312561f,0.3401587903499603f,-0.552861213684082f,0.008703077211976051f,0.8645807504653931f,0.505298912525177f,-0.2635340690612793f,-0.44866883754730225f,1.1203868389129639f,-0.47064656019210815f,0.6688498258590698f,-1.499226450920105f,0.4303622543811798f,-2.0557498931884766f,0.3120390474796295f,-0.18217383325099945f,-0.9849417209625244f,0.2680796682834625f,2.8499772548675537f,0.6877276301383972f,0.49869269132614136f,0.5227124691009521f,1.1356091499328613f,-0.5821874737739563f,0.45820292830467224f,0.5136645436286926f,0.4970094859600067f,0.022805292159318924f,-0.11805552244186401f,-0.20213933289051056f,0.21575990319252014f,0.11284612864255905f,-0.6090803146362305f,0.1406993865966797f,-0.23063789308071136f,0.697425365447998f,0.43705442547798157f,-2.0800774097442627f,0.38368192315101624f,1.1023768186569214f,-0.9186594486236572f,-2.0204639434814453f,-0.13294751942157745f,0.13267943263053894f,0.8123026490211487f,1.7105406522750854f,-0.7276941537857056f,-1.5695372819900513f,-0.9860161542892456f,0.3773617744445801f,-1.2956602573394775f,-0.6076367497444153f,-1.2804059982299805f,0.8832780718803406f,0.6692712903022766f,-0.037189409136772156f,-0.2012755125761032f,0.8031619787216187f,-0.5229789614677429f,-0.7559437155723572f,-0.7354531288146973f,1.8517345190048218f,-0.5652714967727661f,-1.1334456205368042f,-0.5961317420005798f,0.42024484276771545f,1.737728476524353f,0.37042132019996643f,-0.6265977621078491f,0.4121973216533661f,-0.07464686036109924f,1.1824814081192017f,-1.2456861734390259f,3.1828813552856445f,0.42395952343940735f,0.15388184785842896f,-0.9703521132469177f,-0.5254735946655273f,-0.4303393065929413f,-0.3122848868370056f,0.44932878017425537f,0.31034305691719055f,0.2956753373146057f,1.6177998781204224f,-0.47878387570381165f,1.882003664970398f,1.0156264305114746f,-0.6993023157119751f,-1.3555386066436768f,0.13315612077713013f,-0.4373696744441986f,1.5215116739273071f,-0.8457788825035095f,0.2910754680633545f,0.12578143179416656f,-0.48669421672821045f,1.1216925382614136f,0.599159836769104f,0.005991449113935232f,0.013891498558223248f,1.4190763235092163f,0.2882464528083801f,1.8802824020385742f,0.5068897604942322f,-0.6425840258598328f,-1.836530327796936f,-0.5393480658531189f,-1.0972540378570557f,-1.8175212144851685f,-0.6028347611427307f,-2.427229881286621f,-1.7258450984954834f,-0.24002669751644135f,-0.7336971759796143f,1.0859025716781616f,0.06965427845716476f,0.30090105533599854f,1.1657072305679321f,0.6245478987693787f,0.758503794670105f,-0.4279698431491852f,-2.8250060081481934f,0.6164880394935608f,0.7363792657852173f,-0.24750052392482758f,-0.09409024566411972f,-0.844992458820343f,1.6948579549789429f,1.5642460584640503f,0.927674412727356f,-1.0591955184936523f,0.7161778807640076f,-2.7157223224639893f,-1.933381199836731f,1.2831233739852905f,1.0839803218841553f,0.1280050277709961f,0.3983841836452484f,-1.4697916507720947f,-0.33869847655296326f,-0.7091305255889893f,-0.7047624588012695f,1.8022403717041016f,-0.4663436710834503f,-0.21897411346435547f,0.45664671063423157f,0.8964056968688965f,-0.8114674091339111f,-1.6466649770736694f,-1.5471103191375732f,-0.03745563328266144f,-0.08067499101161957f,-0.9197610020637512f,0.57808518409729f,-0.5726067423820496f,1.294964075088501f,-0.8632072806358337f,-0.8153347969055176f,-0.789910614490509f,1.0856034755706787f,0.7309388518333435f,0.5188997983932495f,0.04486279562115669f,-0.00921828392893076f,0.7742498517036438f,-0.6413037776947021f,1.1697765588760376f,0.3000171482563019f,2.878645420074463f,-1.445685625076294f,0.7381811141967773f,0.27369630336761475f,0.3709444999694824f,0.46310412883758545f,0.9214299917221069f,0.04621582850813866f,-0.8943658471107483f,-0.0953112468123436f,-0.11984412372112274f,0.04412943497300148f,0.4369554817676544f,-1.6724910736083984f,0.1646820604801178f,-0.6393880248069763f,-1.126151442527771f,1.3969216346740723f,-0.47979578375816345f,-2.616541624069214f,0.31955692172050476f,-0.8202734589576721f,-0.49397632479667664f,0.35837259888648987f,1.0928195714950562f,-0.00329558365046978f,1.1836931705474854f,0.4830920398235321f,0.3580463230609894f,1.3020180463790894f,0.3569164276123047f,0.49126461148262024f,1.0524487495422363f,1.0536878108978271f,0.12719932198524475f,-1.2682560682296753f,-0.6209090352058411f,0.8768739104270935f,0.20079292356967926f,-1.4972633123397827f,0.005599646829068661f,-0.3203415870666504f,1.6541591882705688f,-1.5716586112976074f,-0.02578038163483143f,-0.8460231423377991f,-0.15371645987033844f,1.910263180732727f,1.0017225742340088f,0.8754696846008301f,-0.6583574414253235f,1.1309574842453003f,1.0773932933807373f,-0.0879080519080162f,0.32116150856018066f,2.0561585426330566f,0.19335806369781494f,1.7679001092910767f,0.7361366748809814f,1.3739590644836426f,0.7875096201896667f,-0.9618638753890991f,0.9346457123756409f,1.2141375541687012f,-1.2994961738586426f,0.0008524558506906033f,1.2515498399734497f,-0.894689679145813f,2.02166485786438f,0.2844415009021759f,-0.17036110162734985f,0.2106323093175888f,0.2381334751844406f,-1.146198034286499f,-0.623969554901123f,-1.520570158958435f,0.4210394620895386f,-0.6647722721099854f,-0.44443315267562866f,1.1984748840332031f,-0.206891730427742f,0.7560681104660034f,-1.185962200164795f,0.28885820508003235f,1.4379444122314453f,-1.5348140001296997f,2.780304431915283f,0.2695072889328003f,1.7016304731369019f,-0.9054626822471619f,0.7126684188842773f,-0.0004463880031835288f,0.21060054004192352f,-0.9740526080131531f,-1.2009639739990234f,0.9951440691947937f,0.29274123907089233f,0.2936437726020813f,0.9120286107063293f,0.01051237527281046f,0.8242721557617188f,0.04637601971626282f,-0.5311675667762756f,-0.005915848538279533f,3.2954671382904053f,-0.5875473618507385f,-0.0806112289428711f,1.5007096529006958f,-0.4460664689540863f,0.2656744122505188f,-0.5227727293968201f,0.19893814623355865f,1.724891185760498f,-0.7926793098449707f,-0.46468469500541687f,-1.803125262260437f,-0.8756406903266907f,1.745570421218872f,-1.4395400285720825f,-0.11616470664739609f,-0.0048182690516114235f,1.9962562322616577f,-0.19761694967746735f,0.27370020747184753f,-1.6797034740447998f,0.7017892003059387f,-0.8241676092147827f,-1.3461940288543701f,-0.3524572551250458f,-0.570697009563446f,0.8464847207069397f,-0.07642439007759094f,-0.2676635980606079f,-0.9779157638549805f,-0.6281117796897888f,-0.870724618434906f,0.24720615148544312f,-0.7100973725318909f,0.08144115656614304f,0.644426167011261f,-0.11032658070325851f,-0.570443332195282f,-0.03714090585708618f,-0.2873310148715973f,0.5668289065361023f,-0.11229991167783737f,1.4102060794830322f,-0.5668971538543701f,-1.3387292623519897f,2.5515334606170654f,-0.8645418286323547f,0.718576192855835f,-0.2006036341190338f,1.1083202362060547f,0.6935703158378601f,-1.1494026184082031f,0.8692928552627563f,-0.5344797372817993f,0.7863809466362f,1.233498215675354f,-0.4450817108154297f,-0.971595287322998f,-0.5946666598320007f,1.5235178470611572f,1.5312713384628296f,-0.7659926414489746f,1.9607051610946655f,-1.024343490600586f,0.9772480726242065f,1.4866487979888916f,-1.6937000751495361f,0.7110504508018494f,-0.4473058879375458f,-1.3146308660507202f,-2.4992051124572754f,-1.4708184003829956f,-0.8460449576377869f,-0.6120154857635498f,-0.7611915469169617f,-0.1446794718503952f,0.202377051115036f,0.469967246055603f,-0.9274566769599915f,0.20447121560573578f,-0.11948313564062119f,1.3133295774459839f,0.24947114288806915f,0.39289358258247375f,-0.8787792325019836f,0.0683729499578476f,-0.8688265681266785f,-2.3266963958740234f,-0.7156980037689209f,1.671601414680481f,0.5426890850067139f,0.024984274059534073f,3.158081293106079f,1.4832065105438232f,-1.0387316942214966f,-0.10519637912511826f,0.06817209720611572f,-0.3286139667034149f,-0.40272483229637146f,-1.5200555324554443f,0.37947413325309753f,-0.2760988175868988f,0.778770923614502f,1.0502543449401855f,0.43888571858406067f,1.041853904724121f,-0.32234883308410645f,-0.2193177342414856f,0.05195494741201401f,1.126029133796692f,-0.7615205645561218f,1.7932606935501099f,1.6974971294403076f,-1.6356068849563599f,-0.3222317695617676f,-1.7612298727035522f,1.6321392059326172f,1.8857420682907104f,-0.9545304775238037f,-0.3926316499710083f,-1.13313627243042f,0.2198915332555771f,-1.9197227954864502f,1.4950190782546997f,0.7772778272628784f,0.9653788208961487f,-1.8640587329864502f,0.8129338622093201f,0.21448230743408203f,-1.8462481498718262f,-1.1233633756637573f,-1.166490077972412f,0.7240221500396729f,0.07114974409341812f,-0.04781360179185867f,0.9584393501281738f,1.3488996028900146f,0.021164799109101295f,1.0044926404953003f,-2.1179299354553223f,-0.7340103387832642f,-0.18532606959342957f,0.7665616869926453f,-0.8992302417755127f,1.4760016202926636f,-0.844778835773468f,-0.17743700742721558f,-0.2075784057378769f,-1.303357481956482f,-1.0679364204406738f,-0.27820977568626404f,-0.9413095712661743f,0.4739021956920624f,-0.3420158326625824f,-1.0529637336730957f,-0.678544282913208f,1.3109923601150513f,-0.3676772117614746f,0.04169384390115738f,0.11304508149623871f,1.5757721662521362f,-0.7964182496070862f,1.5643084049224854f,-0.5563143491744995f,-0.8262993097305298f,1.7833620309829712f,3.2309908866882324f,0.6513899564743042f,0.5168408155441284f,0.33890601992607117f,1.0489054918289185f,-1.1919670104980469f,-1.7719460725784302f,0.46150368452072144f,0.6690313816070557f,-1.71444833278656f,-0.09937994182109833f,1.0182218551635742f,1.4442474842071533f,-1.021550178527832f,-0.35942134261131287f,-2.203774929046631f,-0.14229141175746918f,0.2923114597797394f,-0.13307391107082367f,-0.8080893754959106f,-2.3918118476867676f,1.3212121725082397f,0.2644335925579071f,0.11071489751338959f,-1.2738652229309082f,-1.1077326536178589f,2.2102344036102295f,2.6059629917144775f,-0.17943964898586273f,-0.5934012532234192f,0.1292661875486374f,0.013628888875246048f,-1.3873629570007324f,2.52449631690979f,-0.3410528302192688f,0.27240660786628723f,0.1527288258075714f,0.4485844373703003f,0.6705405712127686f,0.7008087038993835f,0.872833251953125f,-0.6852473020553589f,-0.950685441493988f,1.6517362594604492f,-0.9213799834251404f,0.22739967703819275f,-0.3932245075702667f,-0.3851090669631958f,0.08820641785860062f,1.2249265909194946f,-1.2696199417114258f,-0.4278702139854431f,0.18602149188518524f,0.6868520975112915f,0.19512596726417542f,0.5672711730003357f,0.999553918838501f,1.80446457862854f,1.27751624584198f,-0.06788136065006256f,-0.2918865382671356f,0.3681251108646393f,-0.5460108518600464f,-0.6513246297836304f,-0.4313068091869354f,-0.6752592325210571f,-2.227308511734009f,0.5910868644714355f,0.240905299782753f,-1.0276238918304443f,-1.1245051622390747f,0.7200947403907776f,1.6043366193771362f,-0.4367493689060211f,-0.8720501065254211f,0.25722241401672363f,0.2539846897125244f,0.2030230611562729f,-0.7211213111877441f,0.22194625437259674f,2.4303483963012695f,1.1404616832733154f,-0.8220437169075012f,-0.9876709580421448f,0.09678484499454498f,-1.0190166234970093f,-2.195558786392212f,-2.855680465698242f,-0.1857719123363495f,-0.028149530291557312f,0.08423854410648346f,-0.1989116370677948f,0.9905266165733337f,1.3334258794784546f,-0.8502844572067261f,0.8121299147605896f,-1.704866647720337f,-1.1376672983169556f,-0.15122486650943756f,-1.4243154525756836f,0.7665794491767883f,1.3441433906555176f,-1.2621307373046875f,-1.3341575860977173f,-0.4336588680744171f,-0.7309579849243164f,-0.030574340373277664f,-0.4355711042881012f,1.2212529182434082f,-1.1063898801803589f,-1.172154426574707f,-0.8627814650535583f,-0.4302836060523987f,1.2104847431182861f,-3.0641489028930664f,0.8773333430290222f,-2.1637346744537354f,-0.5436927676200867f,-2.577155113220215f,-0.9589971303939819f,0.19372135400772095f,-1.1880146265029907f,0.6376563310623169f,-1.462622880935669f,-1.362638235092163f,-0.31584489345550537f,-0.46070167422294617f,-1.3019191026687622f,-1.440940022468567f,-0.45384716987609863f,0.46064069867134094f,0.9741104245185852f,-2.265073299407959f,-0.21468684077262878f,-0.5548924803733826f,0.18231792747974396f,-2.3856699466705322f,1.1243174076080322f,0.6485188007354736f,1.088131308555603f,0.15911999344825745f,-1.0449975728988647f,0.31549495458602905f,0.7928712368011475f,-0.3354618549346924f,-0.15376149117946625f,2.517289400100708f,-1.0165561437606812f,-0.7484356760978699f,-0.6412963271141052f,-1.3161780834197998f,0.030117694288492203f,1.033859372138977f,0.6516194939613342f,-0.02023760788142681f,-0.21202287077903748f,-1.0774378776550293f,1.392085075378418f,1.9046648740768433f,0.2434518039226532f,-0.8014963865280151f,-1.0205270051956177f,-0.27750757336616516f,-1.2267944812774658f,-1.3686790466308594f,-0.606843888759613f,1.6045193672180176f,1.0847095251083374f,0.30693668127059937f,0.8354124426841736f,0.4494607448577881f,-1.3359277248382568f,-1.1731992959976196f,2.9941153526306152f,-1.3633226156234741f,-0.44813811779022217f,-0.3888394236564636f,-1.1401090621948242f,-1.290391206741333f,-0.555599570274353f,-0.8435399532318115f,0.8617756366729736f,-0.26582005620002747f,0.18830235302448273f,0.017404688522219658f,1.199514627456665f,0.021609896793961525f,-1.4919862747192383f,1.164421558380127f,2.7844300270080566f,1.3321536779403687f,0.46667975187301636f,0.6193737387657166f,-0.4000229239463806f,0.3436392843723297f,-0.30895403027534485f,-0.20498013496398926f,-0.2606310248374939f,0.6052592992782593f,-0.4798106253147125f,0.2052098512649536f,-0.5682278275489807f,-0.9317939877510071f,-0.9980362057685852f,1.1841065883636475f,1.4833275079727173f,-0.9471282362937927f,0.18394765257835388f,0.7444853186607361f,0.9699220657348633f,-0.9281286597251892f,-0.7492998838424683f,1.6625100374221802f,0.24749192595481873f,-1.1960281133651733f,-0.2941094934940338f,-0.6715885400772095f,-0.511110782623291f,1.3824360370635986f,-1.0183249711990356f,0.36558789014816284f,0.024783747270703316f,1.3088632822036743f,0.49283114075660706f,2.0509824752807617f,-0.4682035744190216f,-0.7581542730331421f,0.6420157551765442f,-0.07707694172859192f,1.1094696521759033f,-1.9874463081359863f,-0.038736842572689056f,1.3997853994369507f,1.6137865781784058f,1.2992007732391357f,0.4880669116973877f,-0.9357162714004517f,-0.37200087308883667f,-0.38137173652648926f,-0.4478505551815033f,-0.5125453472137451f,0.877228319644928f,-1.8489829301834106f,-0.4547339379787445f,0.26277410984039307f,-1.0381461381912231f,-1.4971567392349243f,-0.4048415720462799f,0.14588306844234467f,0.08583936840295792f,-0.06311609596014023f,-0.6400038599967957f,1.1929343938827515f,-1.2568349838256836f,-1.23932945728302f,-0.12978632748126984f,-0.7530977129936218f,-0.14773862063884735f,-0.3697745203971863f,1.981331467628479f,-0.765129804611206f,-0.10750474780797958f,1.3802506923675537f,-0.4920231103897095f,-0.982941210269928f,1.855879545211792f,1.040174126625061f,0.09465838223695755f,-0.017559509724378586f,1.088394045829773f,-1.0588915348052979f,1.490423321723938f,-0.01306916680186987f,-0.3397555351257324f,-1.1323572397232056f,2.2666618824005127f,1.4141037464141846f,-1.11232590675354f,-0.7868990302085876f,-0.010089032351970673f,0.7999180555343628f,-0.47812721133232117f,0.34051230549812317f,1.0280685424804688f,0.5871667265892029f,-0.4958074390888214f,0.5793802738189697f,0.3843204081058502f,0.5252801775932312f,-0.6842359304428101f,0.9694971442222595f,1.3462532758712769f,0.5025534629821777f,0.19195066392421722f,-1.3739877939224243f,1.3391761779785156f,-1.4364533424377441f,0.03973628580570221f,-0.31046822667121887f,-0.37443163990974426f,0.8158426880836487f,-1.1399611234664917f,-0.5136551260948181f,-0.9008309841156006f,0.10978754609823227f,0.18428030610084534f,-0.09151797741651535f,-0.7518681883811951f,-0.19462651014328003f,0.2587728500366211f,-1.0363543033599854f,-0.42261803150177f,-0.6231069564819336f,-0.3552058935165405f,-1.10930335521698f,-1.873268961906433f,-0.8699515461921692f,-0.3147698640823364f,0.609792172908783f,-0.3400805592536926f,0.2659437954425812f,1.2463711500167847f,0.14534196257591248f,-1.3359503746032715f,-0.7860035300254822f,-0.5985881090164185f,0.451890230178833f,0.697793185710907f,-1.4069381952285767f,-1.0418018102645874f,-0.7119022011756897f,1.41397225856781f,0.6565264463424683f,-0.03796425834298134f,0.7924030423164368f,1.5292495489120483f,-0.06350357830524445f,1.3313581943511963f,0.8149471282958984f,0.9051089882850647f,0.3132663369178772f,0.3785961866378784f,0.14864696562290192f,-0.17064571380615234f,0.17201171815395355f,-0.04224751889705658f,1.4226555824279785f,1.0933611392974854f,1.218302607536316f,0.9665552973747253f,0.8874569535255432f,-1.5169124603271484f,1.0033711194992065f,0.14605645835399628f,-1.131580114364624f,0.1883944869041443f,0.1133599802851677f,-0.13387852907180786f,1.4762437343597412f,0.00924405362457037f,-0.9682169556617737f,-0.2900542616844177f,0.7320302128791809f,-1.0031497478485107f,-1.594253420829773f,-1.1116387844085693f,0.015668291598558426f,-0.7644034624099731f,-0.6989546418190002f,0.6370564103126526f,0.42224863171577454f,-0.26799657940864563f,-1.3822897672653198f,-0.4830514192581177f,1.8727835416793823f,0.28115519881248474f,0.5195196866989136f,-0.43390053510665894f,0.8186826109886169f,-1.8578300476074219f,0.4241664707660675f,-0.0226131659001112f,-0.023896176367998123f,0.9498432278633118f,1.1250436305999756f,0.6204732656478882f,1.662628412246704f,0.3977164924144745f,-0.3029177188873291f,0.2469610720872879f,0.404527872800827f,0.5058419704437256f,0.4512306749820709f,-1.4407196044921875f,0.9510821104049683f,0.37268778681755066f,1.8861812353134155f,0.3749529719352722f,-0.3318961262702942f,-0.5868123769760132f,0.7539868354797363f,2.658139228820801f,0.8756155967712402f,-0.7003512978553772f,0.6821007132530212f,-0.8688985109329224f,0.771131157875061f,0.6117385625839233f,-0.8425668478012085f,1.229691982269287f,-0.347575306892395f,1.2531148195266724f,-0.6618359684944153f,0.6273525357246399f,-0.5738652348518372f,0.1498671919107437f,0.41319236159324646f,0.1646520495414734f,-0.61203533411026f,-1.9496488571166992f,0.8116382360458374f,-0.657063901424408f,-1.0166256427764893f,-1.0511268377304077f,0.38045957684516907f,-0.6381061673164368f,2.272854804992676f,-1.27745521068573f,-0.385579377412796f,0.0025206790305674076f,-1.0154662132263184f,0.6719985008239746f,-0.13906507194042206f,-1.3871610164642334f,-0.9932414889335632f,-0.18206198513507843f,-1.6111904382705688f,-1.5223482847213745f,-0.36490321159362793f,0.7837305068969727f,1.9138898849487305f,-0.33529818058013916f,-1.3891311883926392f,0.3586421310901642f,0.07320959866046906f,0.7724571228027344f,1.5848186016082764f,-0.15577581524848938f,-0.041646454483270645f,0.07070975005626678f,-0.12444987148046494f,1.700880765914917f,-0.5642918944358826f,0.32835954427719116f,-1.0837172269821167f,-0.5400665998458862f,-1.5124272108078003f,1.0165799856185913f,0.7415467500686646f,1.6892656087875366f,1.1085330247879028f,0.6218583583831787f,1.3665508031845093f,-0.4310118854045868f,-0.10369608551263809f,0.046026747673749924f,0.754670262336731f,-0.8709783554077148f,-1.2639360427856445f,0.1441393941640854f,0.313613623380661f,0.09283972531557083f,0.20872990787029266f,1.9798368215560913f,1.9124586582183838f,1.3634893894195557f,1.8912118673324585f,-0.12823186814785004f,-0.5693175792694092f,-0.4504988491535187f,0.9707249999046326f,1.1663297414779663f,-1.5337111949920654f,0.09952367097139359f,-1.2605127096176147f,-0.8147484064102173f,1.1535204648971558f,0.7716715335845947f,-0.3628326654434204f,0.25004932284355164f,0.6733137965202332f,-1.750161051750183f,0.26058489084243774f,0.577061653137207f,-1.5763559341430664f,1.5331720113754272f,0.7564440369606018f,1.1833924055099487f,-0.23397645354270935f,0.052939075976610184f,-1.4243518114089966f,-0.2597634196281433f,-0.2717660665512085f,-1.845718264579773f,-0.1246877983212471f,1.0225491523742676f,0.996931254863739f,-1.0247242450714111f,2.91927433013916f,-1.1613702774047852f,0.7692045569419861f,-0.15877389907836914f,-0.8827149868011475f,-1.1074914932250977f,1.145854115486145f,2.0484938621520996f,0.389050155878067f,0.6879400610923767f,-0.3330444395542145f,-0.6332424283027649f,0.7231623530387878f,0.7298260927200317f,1.6657044887542725f,0.33918213844299316f,0.44401973485946655f,0.31505438685417175f,0.43476253747940063f,1.2397977113723755f,0.8384671211242676f,-1.9019371271133423f,-1.9266610145568848f,0.2583029568195343f,0.18575935065746307f,1.1815494298934937f,0.7840414047241211f,2.6442129611968994f,-0.330480694770813f,-0.2621943950653076f,-0.7582839727401733f,1.1466894149780273f,0.3410622477531433f,-0.001370299607515335f,1.131089448928833f,1.9758533239364624f,-0.6691845655441284f,0.4054560363292694f,1.03248929977417f,-2.546680450439453f,-0.3213258981704712f,1.725123405456543f,0.10602279752492905f,1.1109366416931152f,-1.2932440042495728f,1.4934093952178955f,0.40042105317115784f,0.49502384662628174f,-0.5012800097465515f,-0.26426583528518677f,0.593137800693512f,1.3092962503433228f,-0.14264655113220215f,2.055636405944824f,-2.3773138523101807f,0.21482114493846893f,-0.38612866401672363f,1.8898546695709229f,-0.8178251385688782f,0.15115758776664734f,-0.4162648916244507f,-0.6678737998008728f,-0.3290272355079651f,-1.5965182781219482f,0.85188227891922f,0.22369226813316345f,-0.9939760565757751f,-0.39624273777008057f,0.26683881878852844f,-0.9202895760536194f,-0.4343731105327606f,0.7676315903663635f,-0.2875520586967468f,-1.7740377187728882f,-0.05687549337744713f,0.7146432399749756f,2.194685220718384f,2.2815334796905518f,-0.2506597340106964f,-0.8275009393692017f,0.9731980562210083f,0.029071878641843796f,1.5207436084747314f,-0.3632897734642029f,-0.4376896321773529f,0.2552236318588257f,-0.6149164438247681f,0.7593989968299866f,-0.252898246049881f,-0.02413666620850563f,-0.6229268312454224f,-0.325311541557312f,-0.6928778886795044f,0.020295187830924988f,2.125971555709839f,0.5795890688896179f,0.9680367708206177f,0.3230687975883484f,0.5364418625831604f,-1.0136661529541016f,-0.182967871427536f,0.6859906315803528f,0.8733580112457275f,-0.7492735981941223f,0.585235595703125f,1.9265936613082886f,0.37329185009002686f,1.709346890449524f,0.16554954648017883f,0.2036943882703781f,2.2600367069244385f,-0.4497513473033905f,-0.8111067414283752f,-0.9683404564857483f,-2.041199207305908f,-0.437983900308609f,-0.3284623324871063f,-1.0019505023956299f,1.2694473266601562f,0.5102028250694275f,1.7266201972961426f,-1.0176942348480225f,0.0650283470749855f,-0.34120824933052063f,0.03442501276731491f,-1.2069591283798218f,0.10973066091537476f,-0.355510950088501f,0.7691724896430969f,-0.8208763003349304f,0.47093603014945984f,0.1510644555091858f,0.3211715519428253f,1.2406994104385376f,-0.4429633915424347f,0.8118273019790649f,2.287729263305664f,-0.5969668626785278f,0.1278614103794098f,0.21569587290287018f,0.7961338758468628f,0.2302606701850891f,1.0965337753295898f,-1.4806901216506958f,-1.0369895696640015f,0.44902485609054565f,-1.2970272302627563f,-1.6442445516586304f,0.20654022693634033f,0.0022055478766560555f,-1.2789922952651978f,-0.34715941548347473f,0.30724793672561646f,1.0798567533493042f,2.51782488822937f,-0.33137649297714233f,0.643437922000885f,0.4152810275554657f,-0.6612932682037354f,-0.21083146333694458f,0.9991422891616821f,0.22278963029384613f,-0.20890271663665771f,-0.5352425575256348f,0.501498281955719f,-0.8297406435012817f,-0.5794671773910522f,-0.34718069434165955f,-0.30100756883621216f,0.2938772439956665f,-2.0564777851104736f,-0.893881618976593f,-0.5077381730079651f,-2.2734920978546143f,-0.4716598093509674f,-0.05767469480633736f,-0.8070166707038879f,-0.3646572530269623f,1.453871488571167f,1.5554896593093872f,-0.5073213577270508f,-1.3137050867080688f,-0.41519564390182495f,-0.6567728519439697f,-0.9081588387489319f,-0.15021009743213654f,1.186706781387329f,-0.17116433382034302f,-0.30843430757522583f,0.7430745959281921f,-0.4241465628147125f,1.7154572010040283f,1.577714443206787f,-0.40340423583984375f,1.443329930305481f,1.299181342124939f,-0.5929728746414185f,0.020065397024154663f,0.1239800900220871f,-0.18905805051326752f,1.2778542041778564f,-0.6085430979728699f,-0.7269587516784668f,-1.23896062374115f,2.0789105892181396f,0.2776493430137634f,-0.9643083214759827f,1.0738083124160767f,-0.8182777166366577f,-0.11019857227802277f,0.7539490461349487f,-0.125859797000885f,-0.4457692503929138f,-0.36970388889312744f,1.0901178121566772f,0.12797676026821136f,-0.7138673663139343f,-1.0935999155044556f,-1.5664608478546143f,-1.7905099391937256f,-0.3262624740600586f,-0.31212225556373596f,-0.32548192143440247f,-0.1850655972957611f,1.294219732284546f,-0.41454240679740906f,-0.35822179913520813f,-0.7180394530296326f,-1.2821606397628784f,-1.8750221729278564f,0.4960038363933563f,0.2301272451877594f,-0.09396887570619583f,-0.3564194440841675f,-1.114431381225586f,0.834762692451477f,0.6311439275741577f,-0.8844623565673828f,0.7802298665046692f,1.931333065032959f,-1.9169723987579346f,1.3062273263931274f,0.0024843448773026466f,1.3126871585845947f,-0.16901785135269165f,0.3133578300476074f,-0.13439910113811493f,-0.615325927734375f,0.28535762429237366f,-2.121596336364746f,-0.04064201936125755f,-0.37594500184059143f,1.0016851425170898f,-1.6447842121124268f,-0.32162347435951233f,0.6737081408500671f,0.5274333357810974f,-0.3607132136821747f,1.2734748125076294f,-1.6382321119308472f,1.3869264125823975f,-0.2266584038734436f,-0.7288854122161865f,-0.45908603072166443f,-0.44158539175987244f,0.07830405980348587f,-0.6325344443321228f,-1.6357252597808838f,0.5329836010932922f,1.4390074014663696f,0.9060115218162537f,0.0022513060830533504f,-1.2228446006774902f,0.6616469621658325f,-1.2544169425964355f,-1.8525004386901855f,-0.21738871932029724f,-0.21428918838500977f,-0.013080962002277374f,-0.6642910242080688f,0.05477393791079521f,-0.4110487699508667f,-0.1607380509376526f,-1.2139455080032349f,1.7795124053955078f,-0.6843727827072144f,-1.8669792413711548f,-3.2479960918426514f,-1.3123573064804077f,-0.6826921701431274f,0.32889342308044434f,1.0980181694030762f,0.12141948193311691f,2.0105032920837402f,-0.3840530812740326f,0.06487546116113663f,0.48738324642181396f,0.759915292263031f,2.119755506515503f,0.4141649007797241f,-0.005212273448705673f,-0.6963022947311401f,1.3444617986679077f,0.6101139783859253f,1.0891075134277344f,1.3366539478302002f,0.7317367792129517f,0.32434800267219543f,-2.844165563583374f,-0.6899847984313965f,-1.0766265392303467f,0.47059759497642517f,1.906303882598877f,0.4073641896247864f,-0.21241788566112518f,0.32508546113967896f,0.6383122801780701f,1.1903773546218872f,-1.0555751323699951f,-0.42348480224609375f,0.33884701132774353f,1.9834496974945068f,-0.709403395652771f,0.04012640565633774f,0.5389353632926941f,0.3171064853668213f,-0.45255419611930847f,-2.0390844345092773f,-0.152287095785141f,-1.1294664144515991f,-0.4777590036392212f,2.0533645153045654f,-0.11155280470848083f,0.28274187445640564f,-1.5477885007858276f,0.250251442193985f,-0.38228827714920044f,0.26290363073349f,-1.545539379119873f,0.62013179063797f,1.9464104175567627f,-1.6354950666427612f,0.4364285469055176f,-1.0445417165756226f,-0.7381042242050171f,0.15970754623413086f,1.3429261445999146f,-0.3266427218914032f,-2.6380014419555664f,-0.1988370716571808f,-0.8394617438316345f,0.0444381944835186f,-0.9056607484817505f,-1.4165351390838623f,0.13426655530929565f,2.949815273284912f,-0.3561593294143677f,-0.35715511441230774f,1.509838342666626f,-2.199455499649048f,0.593909740447998f,0.6620469093322754f,-1.2175785303115845f,0.17896263301372528f,3.035071611404419f,0.8348044157028198f,0.9565883874893188f,1.3599411249160767f,-0.7935436964035034f,-0.5525858402252197f,-1.0903156995773315f,0.8508240580558777f,-1.1785980463027954f,-0.7758182883262634f,-0.9648252725601196f,0.019172564148902893f,0.9212126731872559f,0.6014358401298523f,0.6938998103141785f,-2.0755693912506104f,-0.06646181643009186f,0.7447649836540222f,0.29706183075904846f,0.14295996725559235f,-0.018519079312682152f,0.2419288605451584f,-1.285326600074768f,1.2021666765213013f,0.7223170399665833f,-0.8025743961334229f,-0.9055061340332031f,0.04578005522489548f,-1.5868830680847168f,-0.6110506057739258f,-0.6356794238090515f,-0.6728383898735046f,0.900360643863678f,-0.9739851951599121f,-0.9464011192321777f,-0.7216681241989136f,-0.5681552886962891f,1.0006332397460938f,-0.22957921028137207f,0.8794645071029663f,0.7875645160675049f,0.3639301061630249f,-0.1614764779806137f,0.25929203629493713f,-0.5561756491661072f,0.2832454442977905f,0.21363207697868347f,-0.15221257507801056f,1.0854660272598267f,-0.09190812706947327f,-0.8284652829170227f,0.7526371479034424f,0.6121322512626648f,0.5926626920700073f,-1.3215312957763672f,0.9189770221710205f,0.17391347885131836f,0.677888035774231f,1.1608256101608276f,-0.5537968277931213f,2.2463111877441406f,-0.47183558344841003f,1.7609734535217285f,-1.4124391078948975f,-0.3567349910736084f,-0.9003021717071533f,-0.4799445569515228f,1.0256857872009277f,0.13942834734916687f,-0.2856835126876831f,-1.0618867874145508f,-0.8381634950637817f,0.1601478010416031f,-1.180640459060669f,1.263064980506897f,1.577191710472107f,-0.9758840799331665f,1.7369332313537598f,-0.1954740434885025f,0.45154035091400146f,0.4738253951072693f,-0.49824851751327515f,-0.8647451400756836f,-0.11044134944677353f,1.3764958381652832f,-0.7480646967887878f,-1.388769268989563f,-0.7302124500274658f,-0.28659480810165405f,-0.13782653212547302f,0.6037797331809998f,0.2874702513217926f,2.0454328060150146f,0.13249675929546356f,-0.7631547451019287f,0.02423296868801117f,0.3009120523929596f,0.4573410749435425f,1.2839027643203735f,-0.6086496114730835f,-0.8533879518508911f,-0.8177829384803772f,0.1260855793952942f,0.8668956756591797f,-1.8144327402114868f,0.3702865242958069f,-1.916422963142395f,2.2393884658813477f,0.5807275772094727f,-1.6194226741790771f,2.435323476791382f,1.084457278251648f,-1.5315871238708496f,-0.15647833049297333f,1.7418843507766724f,-0.6203929781913757f,0.9268569946289062f,-1.0341440439224243f,1.3280378580093384f,0.16312895715236664f,-1.0159324407577515f,-0.34317052364349365f,-0.32650479674339294f,1.120491623878479f,0.9492122530937195f,-0.43526121973991394f,-1.4484645128250122f,0.29177871346473694f,0.13792525231838226f,-0.1569686233997345f,0.3536624312400818f};
alignas(16) float batch_normalization_8_A[] = {0.34247663617134094f,-0.42713576555252075f,0.549497663974762f,0.3185077905654907f,-0.2643669843673706f,0.5892261266708374f,0.23996667563915253f,1.052004337310791f,-0.3471733331680298f,-0.19431845843791962f,0.3451538681983948f,-0.07902184128761292f,-0.041289135813713074f,0.0583624541759491f,0.023995667695999146f,-0.1760888695716858f,0.5661495923995972f,0.19324398040771484f,-0.3540426194667816f,0.02396637201309204f,-0.7163875102996826f,-0.11045807600021362f,0.07293161749839783f,0.23143158853054047f,0.3877856731414795f,-0.1423357129096985f,-0.3739481568336487f,0.15473875403404236f,-0.39588698744773865f,0.5927395224571228f,-0.14409154653549194f,-0.21549516916275024f};
alignas(16) float conv2d_4_internal_1_W[] = {-1.2853840589523315f,0.2204841673374176f,-1.7377815246582031f,-1.1139848232269287f,0.198179692029953f,-2.03291654586792f,-1.347900390625f,1.2367188930511475f,1.1807827949523926f,0.5944589376449585f,-0.8795735836029053f,0.7831876277923584f,-0.028599044308066368f,-1.9265140295028687f,1.9480012655258179f,1.2992241382598877f,0.17663832008838654f,0.3147464394569397f,-1.7654439210891724f,-1.985405445098877f,-1.9682387113571167f,0.18832895159721375f,1.4090843200683594f,-0.8599035739898682f,-0.7617185711860657f,2.286004066467285f,-0.0643836036324501f,1.5666757822036743f,-2.437347650527954f,0.33270716667175293f,0.7357276678085327f,0.9466575980186462f,-1.166403889656067f,-1.7715239524841309f,2.8005614280700684f,1.3444947004318237f,1.5305107831954956f,0.33853018283843994f,-0.08039572834968567f,0.8566628098487854f,-0.5932856798171997f,-0.2751302719116211f,-1.8096396923065186f,0.4872552156448364f,0.5259173512458801f,-2.7954556941986084f,0.36852458119392395f,-0.08099241554737091f,-0.35481467843055725f,-0.8304710388183594f,-1.0014170408248901f,3.0215632915496826f,0.46054506301879883f,1.4355272054672241f,2.395874500274658f,0.6947036981582642f,0.14671318233013153f,0.1651795357465744f,-0.3570721745491028f,-0.4465687572956085f,1.5946873426437378f,-1.0421383380889893f,-0.7271730899810791f,-0.5598292350769043f,1.18001127243042f,0.17932742834091187f,-1.274190068244934f,1.4915910959243774f,-0.23072801530361176f,-1.5874061584472656f,-1.1684842109680176f,1.009120225906372f,-1.0660679340362549f,-0.9518648982048035f,3.1002538204193115f,3.5278842449188232f,-1.1615980863571167f,-2.2004823684692383f,2.67645263671875f,0.9766259789466858f,0.8320721983909607f,0.5353145003318787f,-1.814509630203247f,0.6760846376419067f,-2.1385512351989746f,3.4777588844299316f,0.7600777745246887f,-0.010749194771051407f,0.29929447174072266f,1.2113037109375f,1.3017823696136475f,1.302230715751648f,-1.0134354829788208f,0.2793489098548889f,0.7999265193939209f,-1.8272486925125122f,0.5951200127601624f,0.6309840083122253f,1.8640644550323486f,-1.0937345027923584f,0.2955022156238556f,0.7023739814758301f,0.4873516261577606f,1.0261286497116089f,0.4333697259426117f,-0.7962640523910522f,-0.1876165121793747f,-0.23664245009422302f,1.9156955480575562f,-2.576580286026001f,1.4946552515029907f,0.5569416284561157f,0.15436413884162903f,0.2402953952550888f,2.657986640930176f,0.4412704110145569f,1.040267825126648f,-1.3224592208862305f,0.3575933873653412f,-1.731070876121521f,0.6109585165977478f,-1.0407015085220337f,1.2297990322113037f,2.136953830718994f,-1.0657256841659546f,-2.288860559463501f,0.40093621611595154f,-1.1357961893081665f,-0.31702086329460144f,0.06189471110701561f,2.486314058303833f,4.917355060577393f,-0.5528632402420044f,1.9696170091629028f,1.0902092456817627f,-0.7998723983764648f,0.8664481043815613f,-1.8434665203094482f,-1.8438916206359863f,-1.6366400718688965f,-0.5290699601173401f,0.3677404224872589f,1.1234170198440552f,0.9437522292137146f,1.3797235488891602f,1.2793911695480347f,-1.9458339214324951f,2.0717358589172363f,0.008715956471860409f,1.9759238958358765f,-0.632235050201416f,0.5809832215309143f,-0.10037978738546371f,-0.4790199100971222f,-0.6992026567459106f,-1.0754667520523071f,-0.003904997371137142f,0.7726036906242371f,0.911788821220398f,-3.1802337169647217f,1.0101683139801025f,0.49063244462013245f,0.8463931679725647f,-0.6394978761672974f,-0.701560378074646f,-0.1510414034128189f,-0.06440006196498871f,-0.9943686723709106f,-1.3818739652633667f,-1.2910535335540771f,0.9265483021736145f,-2.8709840774536133f,-1.0307133197784424f,0.12275240570306778f,1.9471263885498047f,-1.2183256149291992f,-1.9053316116333008f,-0.4994461238384247f,-1.4620429277420044f,-2.0531790256500244f,0.24512134492397308f,0.8776833415031433f,2.6437675952911377f,0.9790754318237305f,0.5349478721618652f,1.105609655380249f,1.56438148021698f,-0.23012763261795044f,3.318718671798706f,-1.1444121599197388f,2.411022424697876f,-0.2796249985694885f,-0.04424108937382698f,0.7427278757095337f,0.9102823138237f,-2.0132999420166016f,-0.9636228084564209f,-1.1379594802856445f,1.3092999458312988f,2.2151906490325928f,-0.06563989073038101f,0.6845806241035461f,0.5472236275672913f,0.472996324300766f,-3.3971073627471924f,-2.078011989593506f,0.17719769477844238f,1.2975460290908813f,0.938733696937561f,0.04604927822947502f,1.6067806482315063f,-0.07207772135734558f,0.7175002694129944f,-1.6579867601394653f,-0.4936918318271637f,2.988849639892578f,1.437449336051941f,1.2560099363327026f,0.026020465418696404f,0.5260882377624512f,-3.436502695083618f,2.5129971504211426f,-1.1557152271270752f,-0.17640961706638336f,0.05052737519145012f,2.3180394172668457f,0.9902620315551758f,0.5716314911842346f,-0.5686664581298828f,-0.5173323154449463f,-0.4797552227973938f,-0.9328243136405945f,2.4709949493408203f,-0.14033058285713196f,-0.1571054309606552f,1.1537396907806396f,2.3597328662872314f,-2.599268674850464f,-1.7944363355636597f,1.0308784246444702f,-1.556721568107605f,-0.019158007577061653f,1.0281367301940918f,1.5420541763305664f,-1.2229012250900269f,-1.0277730226516724f,-0.606025755405426f,0.38787975907325745f,0.15226492285728455f,1.028347373008728f,1.9526320695877075f,1.0069295167922974f,-2.022599458694458f,2.4920599460601807f,-0.8173086643218994f,-0.8997820019721985f,0.6659595966339111f,0.791109561920166f,3.0244908332824707f,-0.8150982856750488f,-0.4409644901752472f,1.9581787586212158f,-0.1781969964504242f,-2.285668134689331f,-2.09985613822937f,0.07605969905853271f,-0.2923983931541443f,-0.5347921252250671f,3.2887120246887207f,2.2342960834503174f,-0.11083555966615677f,0.9728935360908508f,0.1474566012620926f,2.5250165462493896f,-1.6292798519134521f,-1.974740743637085f,-0.38953182101249695f,1.6361597776412964f,2.143699884414673f,1.446474313735962f,1.6206485033035278f,2.473811149597168f,2.568157434463501f,0.2640230655670166f,0.6580507159233093f,-1.1306995153427124f,1.2906482219696045f,-0.019581696018576622f,1.1885912418365479f,-2.389204740524292f,-1.3452026844024658f,0.10802358388900757f,-0.4174359142780304f,2.0323657989501953f,-1.1489229202270508f,-0.8878543376922607f,-0.1659744381904602f,-1.5922191143035889f,-0.9230082035064697f,2.6524500846862793f,0.08949190378189087f,0.02594417706131935f,0.28507840633392334f,0.09585059434175491f,-1.8737374544143677f,1.267063856124878f,-1.630558729171753f,-1.5362656116485596f,-0.10908454656600952f,1.2854039669036865f,-0.6370503902435303f,0.3579524755477905f,0.16018332540988922f,-0.4826522469520569f,-1.4505932331085205f,-3.1572489738464355f,2.186610698699951f,0.6201396584510803f,1.0144000053405762f,1.1936538219451904f,0.5650550723075867f,-0.4116857349872589f,2.524855613708496f,-0.10741162300109863f,0.45377224683761597f,-2.148872137069702f,-3.08486008644104f,0.09931986033916473f,-0.003247977001592517f,-1.2408459186553955f,-1.7760810852050781f,-1.6274476051330566f,-0.4558844566345215f,1.018496036529541f,2.124800205230713f,-1.048348307609558f,-2.3849027156829834f,0.4379466772079468f,0.8792309761047363f,-0.07876250147819519f,-1.5241663455963135f,1.998054027557373f,-1.7924160957336426f,1.3982902765274048f,0.7278715968132019f,2.515157699584961f,1.8777751922607422f,2.2549502849578857f,0.31526923179626465f,2.6106576919555664f,0.6405754685401917f,-1.277882695198059f,1.505857229232788f,-0.7553980946540833f,0.23325519263744354f,0.2914827764034271f,2.842120885848999f,-0.6085287928581238f,0.5613057017326355f,-1.0282013416290283f,0.8520697951316833f,-1.9786734580993652f,-0.19360002875328064f,-0.004906422924250364f,-1.459498405456543f,-0.6918061971664429f,0.7316548228263855f,3.133209466934204f,-0.5339165329933167f,2.4178903102874756f,-0.8853030204772949f,-1.4757161140441895f,-0.053669966757297516f,1.045829176902771f,1.0566121339797974f,0.9580751061439514f,-0.9326703548431396f,0.1851375699043274f,2.5248780250549316f,0.30081868171691895f,-1.4673649072647095f,0.33149415254592896f,-0.2524372935295105f,-1.7375314235687256f,1.7937278747558594f,0.5700977444648743f,-0.39421147108078003f,-1.4182298183441162f,-0.9037442207336426f,1.5736217498779297f,0.8043471574783325f,-0.1384146511554718f,0.8467594981193542f,-1.1955913305282593f,-1.4618544578552246f,0.8153374195098877f,1.5629801750183105f,-1.6936476230621338f,0.575295627117157f,0.5567035675048828f,1.1722640991210938f,-2.802619695663452f,0.21973928809165955f,-0.2881275415420532f,-1.6088114976882935f,1.0911916494369507f,1.5870283842086792f,-0.23365052044391632f,2.2452232837677f,-3.5111355781555176f,1.2414605617523193f,-0.04794004186987877f,1.0276596546173096f,-0.8277565836906433f,-0.8741362690925598f,-0.5301339626312256f,-0.6513071060180664f,-0.6694681644439697f,-1.1385889053344727f,-1.5694639682769775f,-0.38154423236846924f,0.27187037467956543f,-0.7453526854515076f,0.3337799906730652f,2.156283378601074f,0.173455610871315f,2.092381000518799f,-2.4823050498962402f,2.25844144821167f,-0.5130009055137634f,-1.9542691707611084f,-0.9033218622207642f,-1.277065634727478f,0.287902295589447f,-0.5292735695838928f,0.45668143033981323f,-0.5278447270393372f,-1.1852761507034302f,-1.84463369846344f,2.218636989593506f,-1.997448205947876f,-0.23633432388305664f,2.01134991645813f,-0.7169119715690613f,3.4894003868103027f,1.4643882513046265f,-0.9561036825180054f,0.9612333178520203f,2.8228466510772705f,-0.2757144272327423f,-0.2849276065826416f,0.7631656527519226f,-0.3913048803806305f,0.9506573677062988f,1.0582202672958374f,1.5941622257232666f,-0.42068734765052795f,-0.1642000377178192f,-0.7284967303276062f,0.9873946309089661f,2.675985097885132f,-0.6182083487510681f,1.3740835189819336f,0.9050222039222717f,2.054965019226074f,-1.0928072929382324f,1.1045345067977905f,-0.5289409160614014f,0.34163567423820496f,1.7747528553009033f,0.24086250364780426f,0.9665800333023071f,-2.34255051612854f,1.7703402042388916f,0.39000383019447327f,-0.726700484752655f,-1.0683836936950684f,0.03692123666405678f,-0.6756712198257446f,0.9127182960510254f,0.10049279034137726f,-0.24827231466770172f,3.5466654300689697f,-1.998044490814209f,0.7294316291809082f,-1.531376600265503f,0.8057363033294678f,-1.874957799911499f,0.5806083679199219f,-0.84194016456604f,-0.5047256350517273f,-0.03191123157739639f,-0.6529700756072998f,0.8974668383598328f,0.09812391549348831f,-2.085052013397217f,-0.34373557567596436f,-0.5473200678825378f,-0.23742616176605225f,2.777712345123291f,0.12008274346590042f,-0.3953514099121094f,-1.1410466432571411f,0.0507170744240284f,-1.1816307306289673f,-2.2551655769348145f,-0.5150294303894043f,0.3225052058696747f,-0.6344008445739746f,0.4863587021827698f,-1.757042407989502f,-0.4619581401348114f};
alignas(16) float batch_normalization_9_A[] = {0.5068461298942566f,0.7499054670333862f,-0.726949155330658f,-0.13018013536930084f,0.28259772062301636f,-1.232141375541687f,0.06637920439243317f,-0.7089108228683472f,0.5238178968429565f,0.8026599884033203f,-0.18113133311271667f,0.2087104320526123f,-0.5858199596405029f,1.87249755859375f,-0.47246018052101135f,0.837809145450592f};
alignas(16) float separable_conv2d_5_internal_0_VALUES[] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
alignas(16) float separable_conv2d_5_internal_1_W[] = {-0.0030980028677731752f,-0.012607678771018982f,-0.04168038070201874f,-0.06838903576135635f,-0.054210416972637177f,0.1375623345375061f,0.022993046790361404f,0.0026303769554942846f,0.029582077637314796f,-0.08955282717943192f,0.0006554993451572955f,-0.06801863014698029f,-0.08060241490602493f,0.0957721695303917f,0.025637280195951462f,0.03728225454688072f,-0.026665931567549706f,-0.05542875453829765f,0.12546668946743011f,0.012951132841408253f,0.005607675760984421f,-0.11069835722446442f,-0.02330596186220646f,0.08279422670602798f,-0.10027557611465454f,0.004986973013728857f,0.034059081226587296f,-0.07137776166200638f,0.03707360848784447f,-0.04477309808135033f,0.014145826920866966f,-0.09876196086406708f,-0.07899826020002365f,-0.036422692239284515f,0.031390003859996796f,0.07299670577049255f,-0.13011179864406586f,-0.015850497409701347f,-0.06349903345108032f,0.07157082855701447f,0.06339873373508453f,0.012771653942763805f,-0.0602850504219532f,0.0003938671143259853f,-0.10497671365737915f,-0.013579935766756535f,0.04566839337348938f,0.045064784586429596f,-0.03906678035855293f,0.029922975227236748f,0.02612650766968727f,-0.06393076479434967f,-0.004057618323713541f,0.08513082563877106f,0.014589056372642517f,0.06978408992290497f,0.06774440407752991f,-0.01725798286497593f,0.12958204746246338f,0.06971056759357452f,0.08966442197561264f,0.030778342857956886f,-0.037865009158849716f,0.03340455889701843f,0.048302408307790756f,-0.01976991631090641f,-0.09284251928329468f,0.009613318368792534f,-0.12668254971504211f,0.05486997589468956f,0.12017830461263657f,-0.02359992265701294f,2.230179961770773e-05f,0.0006883179885335267f,0.06406699120998383f,-0.06522228568792343f,-0.06810387223958969f,0.12458410114049911f,0.052098337560892105f,-0.05363646522164345f,-0.004824795760214329f,-0.11366919428110123f,0.029943129047751427f,-0.08824838697910309f,-0.04799966886639595f,0.033765047788619995f,-0.014921334572136402f,0.0947960764169693f,-0.03819815441966057f,-0.05182608217000961f,-0.009937294758856297f,0.03402237594127655f,0.03358132019639015f,0.0031843106262385845f,0.020941993221640587f,-0.10913168638944626f,-0.10707658529281616f,-0.0014501461992040277f,0.03833768889307976f,0.07119981199502945f,-0.13745929300785065f,-0.022422663867473602f,-0.0430600605905056f,0.03199218213558197f,0.12130077183246613f,-0.025213368237018585f,-0.02835770882666111f,0.013519464060664177f,-0.09649504721164703f,-0.003465785877779126f,0.0032704307232052088f,0.005075770430266857f,-0.04084204509854317f,0.08203890174627304f,0.01810232177376747f,-0.014940072782337666f,0.04150295630097389f,0.10079774260520935f,-0.021755386143922806f,0.003191986819729209f,0.14225390553474426f,0.0031281993724405766f,-0.004495751112699509f,-0.014492709189653397f,0.06022007763385773f,0.09969811141490936f,-0.06770296394824982f,-0.013063055463135242f,0.0072678811848163605f,-0.0009148209355771542f,-0.031904395669698715f,-0.039190009236335754f,-0.09065356850624084f,-0.031083151698112488f,-0.004298738669604063f,-0.01685521937906742f,-0.06685001403093338f,0.036998700350522995f,-0.03192666172981262f,-0.09007192403078079f,-0.0011349587002769113f,-0.004509963095188141f,0.012814401648938656f,-0.021547850221395493f,-0.0027909749187529087f,-0.07845939695835114f,-0.0786011666059494f,0.050052378326654434f,-0.03495178371667862f,-0.016290320083498955f,-0.003419052343815565f,-0.06875204294919968f,-0.007353668101131916f,0.011402965523302555f,-0.07539419829845428f,0.05442533642053604f,-0.04645358398556709f,0.05755801498889923f,-0.04835721105337143f,-0.09441408514976501f,-0.05140853300690651f,0.03743644803762436f,-0.04667559266090393f,0.016674881801009178f,0.008551087230443954f,-0.09237699955701828f,0.1066880151629448f,-0.01787552610039711f,0.00910334475338459f,-0.06091955304145813f,-0.0387839712202549f,0.006783557590097189f,-0.03285786136984825f,0.0009165367810055614f,-0.024820493534207344f,-0.02414538897573948f,-0.08356980979442596f,0.005374687723815441f,-0.04795926809310913f,-0.021486623212695122f,0.03342849761247635f,0.01757509633898735f,-0.02307252585887909f,-0.018458658829331398f,0.04209893196821213f,0.07863172888755798f,0.005427767522633076f,-0.06571565568447113f,-0.05763464421033859f,0.06029891222715378f,-0.0034778635017573833f,-0.08478690683841705f,-0.05530095472931862f,0.0876743346452713f,0.0018451049691066146f,-0.04040547087788582f,0.010668473318219185f,0.043714672327041626f,-0.024564918130636215f,0.10830650478601456f,-0.05691478028893471f,-0.10870048403739929f,-0.052297454327344894f,0.021969981491565704f,-0.11375310271978378f,0.010098089464008808f,-0.06315785646438599f,-0.011787809431552887f,0.032382942736148834f,-0.06696618348360062f,0.07220996916294098f,-0.0016886513913050294f,-0.00712953507900238f,-0.07064715027809143f,0.11717581003904343f,0.0278315432369709f,-0.09505456686019897f,0.00493813818320632f,0.02333752252161503f,0.0253312885761261f,-0.05735485628247261f,-0.05623594671487808f,0.05495193973183632f,0.011494405567646027f,-0.0912569984793663f,-0.010408702306449413f,-0.02565735951066017f,-0.11081884801387787f,-0.0728527307510376f,0.03802095353603363f,-0.027445169165730476f,-0.15335240960121155f,-0.06211605668067932f,-0.01284078136086464f,0.0040642255917191505f,0.06878228485584259f,-0.004206088371574879f,-0.009769530966877937f,-0.0812176913022995f,0.0487934909760952f,0.08248619735240936f,0.00758734717965126f,-0.11972317099571228f,-0.05995218828320503f,-0.0639747679233551f,0.05339494347572327f,0.005511815194040537f,0.09883187711238861f,0.008967366069555283f,0.07366865128278732f,0.048807960003614426f,0.0034561397042125463f,0.07501732558012009f,0.03115927428007126f,0.12479206174612045f,0.0013718559639528394f,0.005820624530315399f,0.11705086380243301f,-0.04037366434931755f,0.05970730632543564f,-0.022438690066337585f,-0.056653060019016266f,0.06739462912082672f,0.07541871070861816f,-0.058218732476234436f,-0.019370796158909798f,0.0879247784614563f,0.02925085462629795f,-0.12200897186994553f,-0.02737819403409958f,-0.01858198083937168f,-0.08642105758190155f,0.0805378258228302f,-0.03571252524852753f,0.0029282167088240385f,-0.12395127862691879f,-0.07378070056438446f,0.019724775105714798f,0.06066175550222397f,0.07908792793750763f,-0.002209154888987541f,-0.03521125018596649f,0.0784149169921875f,0.08955931663513184f,-0.10186729580163956f,-0.060002345591783524f,0.020222799852490425f,-0.007343782112002373f,-0.0791817381978035f,-0.05957062914967537f,-0.029057303443551064f,-0.029463743790984154f,-0.031173884868621826f,-0.04736167937517166f,-0.05073247104883194f,-0.10605049133300781f,0.043734919279813766f,0.01944315992295742f,0.05620136111974716f,0.11231482028961182f,-0.06582184880971909f,-0.009578239172697067f,-0.11657985299825668f,-0.02018911950290203f,0.07600410282611847f,0.09307152032852173f,-0.02440885454416275f,-0.08483615517616272f,0.05937522277235985f,-0.003027376951649785f,-0.060445524752140045f,0.0896550789475441f,0.07178184390068054f,0.03310512378811836f,-0.03256319463253021f,0.019115149974822998f,-0.005438432563096285f,0.06821774691343307f,0.07076268643140793f,-0.049777381122112274f,0.02538587898015976f,0.02028040960431099f,-0.05593649670481682f,-0.04442676156759262f,-0.05520150437951088f,-0.03945021703839302f,-0.030751299113035202f,-0.02581244334578514f,-0.08317813277244568f,0.027730049565434456f,0.035890184342861176f,-0.04390796646475792f,0.002728970954194665f,0.0034329439513385296f,0.016114167869091034f,-0.08075208961963654f,0.062043704092502594f,-0.07105989009141922f,-0.09056388586759567f,0.015277118422091007f,-0.09196827560663223f,-0.0699530690908432f,-0.058185141533613205f,-0.0018348215380683541f,-0.033894944936037064f,0.09978783130645752f,-0.002910082461312413f,0.03680696710944176f,-0.09120703488588333f,0.019966743886470795f,-0.07759970426559448f,-0.029889272525906563f,0.013385782949626446f,-0.06224067509174347f,-0.06137160211801529f,0.04759179800748825f,0.03632724657654762f,-0.08910487592220306f,0.02113587036728859f,0.013094625435769558f,-0.029437346383929253f,-0.1100563332438469f,-0.025443073362112045f,0.04356464371085167f,-0.02836776338517666f,0.06582500040531158f,0.00798738095909357f,-0.02636200748383999f,-0.06670620292425156f,0.04567187279462814f,-0.023192964494228363f,-0.04926213622093201f,0.01916893571615219f,0.029631778597831726f,-0.08349914103746414f,-0.03362421691417694f,-0.01022320706397295f,0.07650889456272125f,0.035606540739536285f,-0.02388523519039154f,-0.01717110350728035f,0.010062819346785545f,0.013404536992311478f,0.017805570736527443f,0.0354098342359066f,0.03486556187272072f,0.06501486897468567f,-0.08462993800640106f,-0.022150058299303055f,0.02098878100514412f,-0.029960984364151955f,0.08038734644651413f,-0.025531429797410965f,-0.05013881251215935f,-0.007165472023189068f,0.07626538723707199f,-0.0430869534611702f,-0.054715655744075775f,-0.0016046507516875863f,0.05857529863715172f,0.06941331177949905f,0.0015542694600299f,0.051658447831869125f,0.10899478197097778f,0.005350273102521896f,-0.0006816696259193122f,0.09824014455080032f,0.008463447913527489f,0.0459611751139164f,0.029104990884661674f,0.05383476987481117f,0.05506211146712303f,0.004855094477534294f,0.08128345012664795f,0.07581339031457901f,-0.028934692963957787f,-0.02358011156320572f,0.09050678461790085f,-0.06236974895000458f,0.028460970148444176f,0.02747505158185959f,-0.011963734403252602f,-0.07492799311876297f,-0.038239311426877975f,-0.05092721804976463f,0.04401372745633125f,-0.07853426784276962f,0.033625807613134384f,-0.0336042083799839f,0.11828015744686127f,0.03514741361141205f,0.08245053887367249f,-0.05803964287042618f,-0.07672816514968872f,-0.008874570950865746f,-0.07719417661428452f,-0.11801143735647202f,0.030755050480365753f,-0.039734289050102234f,0.02832101285457611f,0.015990525484085083f,0.06981875002384186f,-0.033885613083839417f,0.06563901901245117f,-0.0440654456615448f,0.03545172140002251f,0.017360076308250427f,0.0728166401386261f,0.0949370339512825f,0.024613285437226295f,0.10416197031736374f,0.031296707689762115f,-0.037284884601831436f,-0.009016319178044796f,0.09101687371730804f,-0.0012076611164957285f,-0.00510329008102417f,-0.005568521097302437f,0.10583315789699554f,0.02999407984316349f,-0.05854939669370651f,-0.04195248708128929f,0.08212340623140335f,0.03349337354302406f,0.1260147988796234f,0.0651397854089737f,0.023929761722683907f,-0.014136308804154396f,-0.053720492869615555f,-0.006135119590908289f,0.07551440596580505f,0.0005814178730361164f,-0.011415969580411911f,0.029280535876750946f,0.07450323551893234f,0.02222304232418537f,-0.03996012732386589f,0.041232652962207794f,0.07719339430332184f,0.025154514238238335f,-0.009233443066477776f,0.04137067124247551f,-0.09549510478973389f,0.03344042971730232f,0.05532245710492134f,-0.06621848046779633f,-0.1064605638384819f,-0.015509539283812046f,0.017562471330165863f,0.057624056935310364f,-0.034031860530376434f,0.08732777088880539f,-0.028948068618774414f,0.12017261236906052f,-0.0043370528146624565f,-0.018603157252073288f,0.02583407051861286f,-0.05774499103426933f,0.08175919950008392f,-0.0907372236251831f,0.027249079197645187f,-0.05888701230287552f,-0.03826773911714554f,0.042736925184726715f,0.05616668611764908f,0.0252373069524765f,-0.06376825273036957f,0.07669900357723236f,-0.07493146508932114f,0.05060095340013504f,-0.01331216748803854f,0.059684284031391144f,0.09355189651250839f,-0.01749597303569317f,-0.011532715521752834f,-0.011437997221946716f,-0.06555300205945969f,-0.014415289275348186f,-0.08889860659837723f,-0.0260666124522686f,0.020405517891049385f,-0.020334428176283836f,0.07134521752595901f,-0.04330712929368019f,0.020300619304180145f,-0.022751765325665474f,0.07517785578966141f,-0.006872262805700302f,0.030850224196910858f,-0.027986034750938416f,-0.009069014340639114f,0.02596057578921318f,-0.04833108186721802f,-0.0045714243315160275f,0.019491981714963913f,-0.07449730485677719f,0.033646147698163986f,0.10196205973625183f,0.010867778211832047f,-0.01715216413140297f,-0.036416154354810715f,0.10386497527360916f,0.02378658950328827f,0.04078316688537598f,0.02137688919901848f,-0.05435242876410484f,-0.09401966631412506f,0.00796655286103487f,0.03244767338037491f,-0.03456037864089012f,0.007017357274889946f,0.028621438890695572f,-0.04416073486208916f,-0.03790447115898132f,-0.07535957545042038f,0.03183247148990631f,-0.00720746535807848f,0.03430134803056717f,-0.021631726995110512f,-0.06587612628936768f,0.01310250535607338f,0.0014744900399819016f,-0.002243591705337167f,-0.0181482695043087f,-0.0019806751515716314f,0.02008979208767414f,-0.08247740566730499f,-0.04976397752761841f,-0.017356818541884422f,0.031358420848846436f,0.0002119414712069556f,0.05293918773531914f,-0.019220592454075813f,0.03734682872891426f,0.004587898496538401f,0.04740619659423828f};
alignas(16) float separable_conv2d_5_internal_2_W[] = {-1.4386746883392334f,-2.67684006690979f,-1.2589131593704224f,-0.7376110553741455f,0.8529043793678284f,0.4097231924533844f,0.3444520831108093f,-0.005173912737518549f,1.888487458229065f,-0.3231520652770996f,1.122949242591858f,0.7321312427520752f,-0.021946009248495102f,1.776252031326294f,-1.7869998216629028f,-0.31535011529922485f,-0.10750911384820938f,-0.5770248770713806f,-1.1146184206008911f,-1.8647239208221436f,-0.2121269851922989f,-1.3944265842437744f,0.7620585560798645f,-1.4996808767318726f,0.22177138924598694f,-0.11064434796571732f,0.512958824634552f,-1.3556689023971558f,0.1743907779455185f,-1.297139048576355f,0.7662548422813416f,-2.3447487354278564f,1.346418023109436f,0.6639185547828674f,-0.8090384006500244f,-0.4708074927330017f,0.07560186833143234f,0.8946528434753418f,0.8875355124473572f,-0.08740207552909851f,-1.7885518074035645f,-0.3833397626876831f,0.4622246325016022f,-1.2852555513381958f,0.323329359292984f,0.12624980509281158f,0.36534371972084045f,-0.48558109998703003f,1.2141337394714355f,0.8014482855796814f,0.4654168486595154f,-0.9796165227890015f,-1.274828553199768f,-2.393033981323242f,-0.3639063835144043f,-0.3334193527698517f,0.4133051037788391f,-0.7546467185020447f,-0.13826407492160797f,1.0700255632400513f,-1.2687147855758667f,0.22764143347740173f,0.7183967232704163f,0.584098756313324f,2.6819465160369873f,-0.23802007734775543f,1.0998634099960327f,0.942890465259552f,-0.6209675669670105f,0.11123576760292053f,1.0694211721420288f,-0.538776159286499f,-0.4372261166572571f,-0.36868995428085327f,0.06702566891908646f,-1.4613001346588135f,-1.871541976928711f,1.1211906671524048f,-1.1449764966964722f,0.6543411016464233f,-0.11247889697551727f,-1.4126089811325073f,-0.22785456478595734f,1.2023800611495972f,0.4055192470550537f,-0.35803717374801636f,-0.07368708401918411f,0.2661416828632355f,-0.3803464472293854f,-1.9527174234390259f,1.2058403491973877f,0.9253708124160767f,0.4001452326774597f,-0.005828277673572302f,0.05893615260720253f,-0.2747974693775177f,-0.6982495188713074f,1.4466720819473267f,1.5614663362503052f,2.322834014892578f,-0.6871576905250549f,-2.0201303958892822f,0.903448760509491f,1.2222716808319092f,-0.04955149441957474f,-0.9846811294555664f,-1.3931257724761963f,-0.2043985277414322f,0.3947148621082306f,0.6173238158226013f,-0.19943122565746307f,-0.008710343390703201f,3.812940835952759f,-0.9651985764503479f,0.37730804085731506f,-2.191364288330078f,-1.8204282522201538f,-0.3546057641506195f,-0.20612698793411255f,0.3433819115161896f,1.3471306562423706f,-0.4861176013946533f,1.2884976863861084f,0.4152141809463501f,-0.9520725011825562f,0.6993329524993896f,-0.2861172556877136f,1.3449187278747559f,-0.4401116669178009f,0.8572158217430115f,-0.9478152990341187f,-0.698216438293457f,0.13876527547836304f,-0.9726452827453613f,-1.0183411836624146f,0.4427003860473633f,-1.332270860671997f,0.7064805626869202f,0.3350677490234375f,0.6984326243400574f,-0.5845972895622253f,1.2056660652160645f,-0.31907886266708374f,0.43193545937538147f,-0.32705914974212646f,0.16474005579948425f,0.13083356618881226f,-0.42094889283180237f,-0.05766133591532707f,-3.334251880645752f,-1.0322265625f,-2.738307237625122f,2.452871561050415f,-0.8614428043365479f,-0.11731430888175964f,-0.2927100360393524f,0.5621594786643982f,-0.8016607761383057f,-1.269699215888977f,0.08560453355312347f,-0.3343615233898163f,-0.03278585523366928f,1.2654532194137573f,-2.440690040588379f,-0.4688786268234253f,-1.8075110912322998f,1.6466050148010254f,-1.3049437999725342f,-0.5480542182922363f,3.18070125579834f,1.3488601446151733f,1.4521507024765015f,0.5158248543739319f,0.7999163866043091f,1.1171298027038574f,1.3002328872680664f,1.6680208444595337f,2.1594479084014893f,-0.03360600769519806f,1.297844648361206f,0.3891448378562927f,-2.8442134857177734f,2.167820453643799f,1.2296075820922852f,1.191149353981018f,-1.9961998462677002f,1.0356123447418213f,0.7014438509941101f,0.6144263744354248f,0.16145651042461395f,1.9549896717071533f,-0.27705398201942444f,0.015267345122992992f,-2.8976922035217285f,-2.0844149589538574f,-0.5967212319374084f,-0.735729455947876f,0.6484607458114624f,0.7681988477706909f,-1.1421362161636353f,1.0733674764633179f,-1.0617460012435913f,-2.0906448364257812f,1.6276061534881592f,0.42224377393722534f,-0.5214524865150452f,-0.5214567184448242f,0.052499428391456604f,0.8617459535598755f,-2.1182680130004883f,0.769184410572052f,-1.0998330116271973f,0.06728611886501312f,1.4730968475341797f,1.0711286067962646f,-0.051597680896520615f,-0.7924278974533081f,1.3192821741104126f,-0.8379590511322021f,1.9616050720214844f,1.5576766729354858f,-0.3622736930847168f,-1.277166485786438f,-0.2466008961200714f,0.6253069639205933f,-0.2871941328048706f,0.3177436292171478f,0.9160968661308289f,1.0652703046798706f,0.40589818358421326f,-2.024679660797119f,0.9689739942550659f,-0.36067909002304077f,1.4517802000045776f,0.5952110290527344f,-1.1156882047653198f,-0.6450594663619995f,0.3194410800933838f,-0.822083592414856f,-2.330761194229126f,0.42508408427238464f,0.23932521045207977f,-0.09857917577028275f,1.1301541328430176f,1.6538503170013428f,2.502044200897217f,-0.4846610426902771f,-1.5340938568115234f,0.5163201689720154f,-1.4512442350387573f,-0.770409107208252f,-0.1858479231595993f,-0.9098955988883972f,-1.2635371685028076f,1.7307652235031128f,-0.7649394869804382f,-2.2832703590393066f,1.5524588823318481f,-0.5100573897361755f,1.959808349609375f,0.2954278290271759f,0.3527626395225525f,-1.0484970808029175f,0.2855689525604248f,0.3762337267398834f,-1.4854793548583984f,-0.9110740423202515f,-2.454648017883301f,0.7056829929351807f,0.060089752078056335f,-0.00459368946030736f,-1.850989818572998f,-0.5457445383071899f,1.384254813194275f,1.6608502864837646f,-1.565853476524353f,0.6250019073486328f,-0.622869074344635f,-0.3445308804512024f,-0.1968112289905548f,-0.5833415985107422f,1.431091070175171f,-1.1185033321380615f,1.5539183616638184f,1.532816767692566f,0.23676152527332306f,0.6711841225624084f,0.9043854475021362f,0.047089241445064545f,0.6900320053100586f,-0.40897753834724426f,-1.2602351903915405f,-2.0688629150390625f,0.37026098370552063f,2.6742069721221924f,-1.6369473934173584f,-0.748658299446106f,-0.10357508808374405f,-0.4056137502193451f,0.2672900855541229f,-0.1096663549542427f,0.7733435034751892f,1.457160472869873f,-0.7237011790275574f,1.6554697751998901f,2.0386886596679688f,-1.0439565181732178f,1.6459938287734985f,-0.3258987367153168f,-0.6996079683303833f,0.08024214208126068f,-0.5290414094924927f,-2.3068387508392334f,-0.33630502223968506f,-0.12564672529697418f,1.1433143615722656f,0.15029920637607574f,0.7888237237930298f,-1.0031405687332153f,-1.9772419929504395f,-0.05340984836220741f,-1.0780634880065918f,-2.3612284660339355f,0.33863896131515503f,-0.1708216369152069f,2.069037675857544f,0.21956641972064972f,-0.6582201719284058f,-0.09071291983127594f,0.03125070780515671f,0.834283709526062f,0.05838305130600929f,0.5708482265472412f,0.055247873067855835f,-0.7184125781059265f,-0.6388586163520813f,0.030744455754756927f,-1.1455141305923462f,0.17842842638492584f,-0.9681465029716492f,0.3703087866306305f,0.7277587056159973f,-0.4709133803844452f,-0.32021239399909973f,0.6308941841125488f,-1.4652597904205322f,-1.7517846822738647f,1.2738138437271118f,1.5092788934707642f,-0.4908367991447449f,0.041681837290525436f,-0.9256628155708313f,-1.5040972232818604f,-0.172694593667984f,0.08758221566677094f,-1.8672746419906616f,1.512007236480713f,0.36325427889823914f,1.549841284751892f,-0.5629050731658936f,0.18579162657260895f,-0.4552655518054962f,-2.2278311252593994f,0.7599595189094543f,-0.9379028081893921f,0.34504106640815735f,-0.1876770257949829f,-0.39459308981895447f,0.07177062332630157f,1.5292110443115234f,-0.014445397071540356f,-0.2965262234210968f,-0.16825494170188904f,-1.0886915922164917f,-0.9553137421607971f,-0.7250219583511353f,-0.04269731789827347f,-1.5409867763519287f,-0.4606945812702179f,3.526916742324829f,-0.4102654457092285f,1.7506455183029175f,-0.013851112686097622f,0.25539395213127136f,0.22516077756881714f,-2.283233642578125f,-0.409194678068161f,-1.450358510017395f,0.6144447326660156f,-0.16872826218605042f,-0.472048819065094f,0.15975487232208252f,0.8325971364974976f,0.49090006947517395f,1.1764459609985352f,-0.6039641499519348f,-1.355305552482605f,-1.4292654991149902f,0.33984965085983276f,1.2315186262130737f,-1.292277216911316f,-0.3195531666278839f,1.8090816736221313f,-0.5670902132987976f,0.6941554546356201f,-0.7047973871231079f,0.12119699269533157f,-0.23240463435649872f,-1.0544118881225586f,1.095043659210205f,-0.9619264602661133f,0.5333226919174194f,0.19555196166038513f,-1.1115810871124268f,2.9136030673980713f,0.05468520522117615f,-1.5892353057861328f,-1.7120968103408813f,1.1524893045425415f,1.1424723863601685f,-2.4633305072784424f,0.5466411113739014f,-1.3379065990447998f,-0.9484472274780273f,1.1033433675765991f,1.3516457080841064f,0.017823757603764534f,0.5496267080307007f,-0.586707592010498f,-0.6144579648971558f,0.15056242048740387f,-0.39314502477645874f,1.8462389707565308f,0.1445470005273819f,0.5243549346923828f,-0.6264775991439819f,0.676252543926239f,-1.0993082523345947f,-0.9305732250213623f,-2.5059571266174316f,-0.5412716865539551f,1.1284630298614502f,-2.256913900375366f,-0.9642587304115295f,-2.582606792449951f,-0.5580254793167114f,0.9732159376144409f,0.33677351474761963f,0.23153062164783478f,0.6513933539390564f,0.10077324509620667f,2.0031535625457764f,-0.2160717099905014f,-0.42104724049568176f,0.06253542006015778f,-0.42395129799842834f,0.005793680436909199f,0.21373623609542847f,0.9288405179977417f,-2.2959067821502686f,-0.35959795117378235f,-0.6139965057373047f,-0.7323582768440247f,-0.5634002685546875f,0.8017867207527161f,-0.8269174098968506f,0.6082448363304138f,1.0001925230026245f,-0.41065266728401184f,-0.9113529324531555f,0.4644300937652588f,-0.3095788359642029f,-1.620693325996399f,0.8379873633384705f,-0.311580628156662f,2.5434367656707764f,-2.411137104034424f,0.8017994165420532f,0.35481035709381104f,-1.3654931783676147f,2.671325445175171f,0.09061888605356216f,0.9293180704116821f,-1.6120115518569946f,1.3602114915847778f,-0.7401294112205505f,-0.6461191773414612f,0.5316904783248901f,0.694923460483551f,2.774077892303467f,-0.8092615604400635f,4.189223289489746f,0.7100096940994263f,-3.251075029373169f,0.033651046454906464f,0.9685001373291016f,-1.0453073978424072f,-1.194317102432251f,-0.37350448966026306f,0.860101044178009f,-2.106126546859741f,1.0282617807388306f,-0.741039514541626f,0.10639532655477524f,-0.047832049429416656f,0.3772740364074707f,3.2809789180755615f,-0.6848471164703369f,0.9236735105514526f,0.26644816994667053f,0.1224658340215683f,0.11170858144760132f,1.668760061264038f,-0.15284007787704468f,0.9663118720054626f,-0.8662467002868652f,-0.46261271834373474f,0.05908922851085663f,-0.000597587670199573f,0.7305224537849426f,1.0654371976852417f,-1.7550963163375854f,1.96224844455719f,0.5494788885116577f,-1.6227971315383911f,-1.3170338869094849f,-0.04630075395107269f,-0.4784018397331238f,-0.09032886475324631f,1.4289357662200928f,-0.28693151473999023f,-2.9964752197265625f,-0.9933866858482361f,1.6148316860198975f,0.6692688465118408f,0.16837060451507568f,0.6729397773742676f,1.1209681034088135f,-0.7699370384216309f,0.0385960154235363f,-2.217783212661743f,0.25012636184692383f,-0.31959566473960876f,-0.3548058569431305f,-0.6599153280258179f,-0.6318744421005249f,-2.571331262588501f,0.759570837020874f,0.8153977394104004f,-0.140189990401268f,-0.38970935344696045f,-0.09116879105567932f,0.9659661054611206f,-0.5937257409095764f,-1.3347549438476562f,-0.7192671895027161f,-1.4920233488082886f,0.3169536590576172f,-1.6474618911743164f,0.5741874575614929f,-0.6803908944129944f,-0.805145263671875f,-0.6086041927337646f,0.6979539394378662f,1.6166598796844482f,0.570880651473999f,1.150160551071167f,-0.40877753496170044f,-0.2767113149166107f,0.6774614453315735f,0.1843925416469574f,1.5504636764526367f,-1.4069727659225464f,2.3127615451812744f,0.5478899478912354f,-1.0320568084716797f,-0.17012684047222137f,-0.6556769013404846f,0.3291684091091156f,-0.1523454338312149f,-0.8669133186340332f,-0.06553133577108383f,0.6114697456359863f,-0.03785388171672821f,1.341965675354004f,-0.3011685013771057f,1.2147719860076904f,1.3380651473999023f,-0.3391225039958954f,0.24134282767772675f,0.2221871018409729f,0.7997653484344482f,-0.9849832057952881f,-1.2709298133850098f,-1.467889428138733f,0.22762666642665863f,0.05078804865479469f,0.8671961426734924f,-1.7675832509994507f,-0.5959670543670654f,0.6561174392700195f,0.778321385383606f,-0.5513443946838379f,-0.1795460730791092f,-2.502396583557129f,0.8550284504890442f,0.05451291799545288f,0.40108391642570496f,0.6620722413063049f,-0.1409124881029129f,-0.29856783151626587f,1.5523645877838135f,-0.06337806582450867f,-1.8023576736450195f,1.818579912185669f,-0.7539287805557251f,-0.8945998549461365f,-2.193216323852539f,0.5769312381744385f,-1.7239946126937866f,-0.7401064038276672f,0.8090687394142151f,0.20207437872886658f,1.9527428150177002f,-2.5069820880889893f,0.31011104583740234f,0.5914620757102966f,0.9199524521827698f,-1.1273137331008911f,-0.15872520208358765f,0.19116713106632233f,1.0484519004821777f,-0.356651246547699f,-0.7084157466888428f,0.23663415014743805f,-0.19112785160541534f,0.02924436144530773f,0.7348992228507996f,0.4701566696166992f,0.6001735329627991f,0.7866923809051514f,-1.646618366241455f,-0.6241147518157959f,-0.06250552833080292f,1.5137490034103394f,0.04571152478456497f,1.3329874277114868f,-0.33624809980392456f,0.9978716373443604f,2.559534788131714f,-1.6781072616577148f,1.2797516584396362f,-0.2859233021736145f,0.7994413375854492f,0.25155362486839294f,-2.0693459510803223f,-0.13966922461986542f,2.76239013671875f,-0.42755764722824097f,1.3027628660202026f,-1.608056902885437f,0.6523507237434387f,2.4204394817352295f,0.27215221524238586f,-0.9155605435371399f,0.21879111230373383f,-1.3587044477462769f,1.8041603565216064f,-2.1966159343719482f,-1.0525710582733154f,-0.45399656891822815f,-0.3000835180282593f,-1.7705637216567993f,-1.4976963996887207f,0.7814754247665405f,-3.0176453590393066f,-1.4665429592132568f,-0.04778594896197319f,-0.2833700180053711f,0.24131062626838684f,3.36946964263916f,-0.3079456090927124f,0.0033631587866693735f,2.0986523628234863f,-0.7453552484512329f,0.28449010848999023f,-1.9640897512435913f,-0.24052192270755768f,-1.4699293375015259f,-0.03799280896782875f,1.4917898178100586f,0.21267704665660858f,-0.05242989957332611f,1.8569562435150146f,-3.118330955505371f,0.2541738450527191f,-0.5180458426475525f,0.35446205735206604f,-0.9612507224082947f,-1.676642656326294f,-0.10327883064746857f,0.9540094137191772f,0.20142924785614014f,0.8537408709526062f,1.4423500299453735f,-0.012299202382564545f,1.3709676265716553f,0.5363823175430298f,3.1322286128997803f,2.229555130004883f,0.2551809549331665f,2.921283483505249f,-0.294575572013855f,-0.4556659460067749f,0.19017666578292847f,0.9415396451950073f,2.309890031814575f,-3.0051677227020264f,2.636960506439209f,0.5890194177627563f,-1.6777054071426392f,0.8719160556793213f,-1.6320686340332031f,-0.0813727006316185f,0.8818224668502808f,0.7544249892234802f,0.8832366466522217f,-0.22049307823181152f,1.6600077152252197f,1.5160595178604126f,-0.26818549633026123f,-1.5000418424606323f,-0.9037604928016663f,-0.7333419919013977f,-0.4833708107471466f,-0.31322887539863586f,0.35425716638565063f,-0.20727232098579407f,-2.4505667686462402f,-1.2476444244384766f,-1.1167774200439453f,-0.14544573426246643f,1.1538656949996948f,-0.6331469416618347f,0.4055328369140625f,1.1095051765441895f,-1.5449168682098389f,-1.055942177772522f,-0.5298082828521729f,2.489147424697876f,-0.25945231318473816f,1.163574457168579f,2.736154794692993f,-0.08715713769197464f,-0.27458256483078003f,0.604035496711731f,-2.5429842472076416f,-1.7697454690933228f,-1.6722874641418457f,-0.8571601510047913f,0.6783826947212219f,-0.841535747051239f,0.742276132106781f,2.2280445098876953f,-0.9875062108039856f,0.04946770891547203f,-0.46646997332572937f,1.6051757335662842f,-1.3015179634094238f,0.3253156542778015f,-0.614842414855957f,1.1263148784637451f,-0.7279192805290222f,1.1639984846115112f,0.9580516219139099f,-1.0991215705871582f,-1.4370871782302856f,-1.3467637300491333f,0.3982987701892853f,0.8490127921104431f,0.28480732440948486f,2.56160831451416f,-0.03620578721165657f,-2.0255722999572754f,1.613209843635559f,-1.5493866205215454f,-0.3248535394668579f,0.6774243116378784f,-0.47401028871536255f,2.5331904888153076f,0.24219021201133728f,0.7845487594604492f,1.398142695426941f,-0.601815938949585f,-1.3678423166275024f,-0.15683838725090027f,0.9777995347976685f,0.0714598000049591f,0.6403569579124451f,-0.4002455770969391f,0.6641001105308533f,-0.1824488490819931f,0.1926044523715973f,1.1750462055206299f,0.19670648872852325f,1.025186538696289f,-0.3835868835449219f,0.3625183403491974f,0.008660460822284222f,0.9793345332145691f,1.6503934860229492f,-1.0695576667785645f,0.34881824254989624f,0.2663240432739258f,-0.9320947527885437f,0.3935990035533905f,-1.1505303382873535f,-0.995653510093689f,0.46870845556259155f,2.3670992851257324f,-1.1123886108398438f,-0.4894392788410187f,-1.9875986576080322f,0.7478883266448975f,0.2113485187292099f,1.6683787107467651f,-0.13892097771167755f,0.31041693687438965f,3.4023783206939697f,0.30320191383361816f,-0.0388718843460083f,2.071769952774048f,-1.024218201637268f,-0.6112669706344604f,0.8828108310699463f,2.2652201652526855f,-2.0210347175598145f,-1.7614251375198364f,0.3637765944004059f,1.4605460166931152f,0.40358030796051025f,-0.12036494165658951f,0.25473901629447937f,-0.08288730680942535f,1.5226149559020996f,-0.41532039642333984f,0.3358231484889984f,-2.8726189136505127f,-0.18767032027244568f,0.0598902590572834f,0.45585426688194275f,-0.4137415885925293f,0.5925952196121216f,0.7081480026245117f,0.17820033431053162f,-0.6116564869880676f,0.2321205586194992f,-0.7210386395454407f,-0.3304693102836609f,-0.8434184193611145f,0.3734823763370514f,0.8756067156791687f,-2.0477235317230225f,1.340958833694458f,-0.14996813237667084f,-0.4682350754737854f,-1.4686810970306396f,0.9279235601425171f,2.4426305294036865f,1.8831943273544312f,-0.512262761592865f,0.990182101726532f,-0.7273392677307129f,0.5218666195869446f,-0.913773238658905f,2.157318592071533f,0.3790164589881897f,0.6269590258598328f,0.39902931451797485f,-1.0768851041793823f,-0.698706328868866f,0.02638053335249424f,-1.902590274810791f,-0.49929237365722656f,-1.477085828781128f,0.1492319256067276f,1.1368528604507446f,0.7216945886611938f,-0.8962017893791199f,1.8229691982269287f,-0.32637646794319153f,1.6200745105743408f,-2.5827646255493164f,-1.357191801071167f,-1.3378708362579346f,-0.7803982496261597f,0.9412155151367188f,1.3902288675308228f,-2.3240816593170166f,-0.10435659438371658f,3.873899221420288f,-2.3446261882781982f,0.401874840259552f,-0.22231647372245789f,1.1856482028961182f,-0.887840986251831f,1.6237311363220215f,0.0917694941163063f,-1.1192349195480347f,-2.2016563415527344f,-0.3555130660533905f,1.028699278831482f,0.05305534973740578f,-0.5920637249946594f,0.464079886674881f,1.029913067817688f,0.36122843623161316f,0.9095802903175354f,0.23802198469638824f,0.02359265275299549f,-0.7949323654174805f,0.0028077373281121254f,-1.91932213306427f,0.610261082649231f,-1.135606288909912f,-0.05360027775168419f,1.1490280628204346f,-1.6032202243804932f,0.9272337555885315f,-0.8140318989753723f,-0.5831787586212158f,1.4368444681167603f,-0.8974206447601318f,0.9809390902519226f,-3.385773181915283f,-0.20532651245594025f,-0.15298373997211456f,-1.0097863674163818f,-0.5083795785903931f,0.9218211770057678f,-0.30347612500190735f,-1.2380797863006592f,2.851242780685425f,-0.26484811305999756f,0.5921862721443176f,-2.6930153369903564f,0.35713624954223633f,-0.09631561487913132f,-0.4786906838417053f,-1.8665457963943481f,1.069900631904602f,0.21728062629699707f,1.4842751026153564f,-0.6507008075714111f,-1.672194004058838f,0.6040531992912292f,0.21215631067752838f,-1.0371289253234863f,-1.0427886247634888f,0.24183544516563416f,-0.7599464058876038f,0.3009507656097412f,-0.11676438897848129f,0.8159930109977722f,-0.10178612917661667f,0.4295283257961273f,1.8399319648742676f,-0.6787740588188171f,-0.34635189175605774f,-0.49702760577201843f,1.1307693719863892f,-1.4246008396148682f,-1.212632417678833f,1.9073888063430786f,-3.2453315258026123f,-0.16817711293697357f,0.6270322203636169f,0.0004900501226074994f,-1.4498077630996704f,-1.3556803464889526f,0.17762553691864014f,1.4123958349227905f,-0.9924794435501099f,1.2168209552764893f,-1.9801229238510132f,0.9019576907157898f,-1.37984037399292f,-1.1724083423614502f,-1.7901332378387451f,-1.1499615907669067f,0.17649009823799133f,-1.0228705406188965f,-1.25484299659729f,-0.6473818421363831f,0.42217913269996643f,1.151187539100647f,0.07371050864458084f,1.5567978620529175f,0.300904780626297f,-0.138519287109375f,-0.09730836749076843f,-0.4570743441581726f,-0.5411611795425415f,-0.21067427098751068f,1.118664026260376f,0.2881525158882141f,-0.8033052682876587f,0.42809250950813293f,-2.0904438495635986f,-0.29088231921195984f,-2.8576555252075195f,-1.5306695699691772f,-0.053306516259908676f,1.2546815872192383f,-0.8430288434028625f,0.749517023563385f,-0.7757005095481873f,0.6946942806243896f,-0.550862729549408f,2.1431052684783936f,0.8454188108444214f,0.23936930298805237f,0.49742382764816284f,-0.32810768485069275f,0.4372243583202362f,0.10153914988040924f,-2.124605178833008f,-0.08055760711431503f,0.9955433011054993f,-0.00921530369669199f,-1.3957716226577759f,1.8037484884262085f,0.9665947556495667f,-0.8879483342170715f,0.41950494050979614f,-0.4092254042625427f,-0.3554549813270569f,0.2042342573404312f,-0.48891717195510864f,-0.8320111632347107f,-1.8533258438110352f,-1.5059921741485596f,-0.5927066802978516f,1.290746808052063f,-1.4390196800231934f,-0.8781144618988037f,-0.5980752110481262f,-0.8132898211479187f,-1.0784655809402466f,1.0891543626785278f,-1.1720081567764282f,0.49386370182037354f,1.6472052335739136f,0.2587195932865143f,1.7061837911605835f,0.8166637420654297f,-0.06647897511720657f,1.950082778930664f,-0.2776555120944977f,0.8894288539886475f,-0.9079443216323853f,-0.6392115354537964f,1.2174230813980103f,-1.4243810176849365f,0.7174960374832153f,-0.04648023471236229f,-0.46294906735420227f,0.42293781042099f,-0.05763930454850197f,-0.22376570105552673f,-0.8683498501777649f,-1.4605764150619507f,0.22063036262989044f,-0.6078930497169495f,-0.6729058027267456f,0.916539192199707f,-0.3032803535461426f,0.48425811529159546f,-0.1988210380077362f,1.1294490098953247f,0.1599910408258438f,-0.3176489472389221f,-0.1354299783706665f,1.329930305480957f,1.4395866394042969f,0.18902428448200226f,0.2147434651851654f,-0.37774938344955444f,0.05739722400903702f,0.06586064398288727f,-1.2587275505065918f,0.20138981938362122f,1.4672670364379883f,1.4166043996810913f,0.01992037333548069f,-0.23820045590400696f,0.19782103598117828f,1.4483505487442017f,-0.3008737564086914f,-1.8369454145431519f,-1.0323930978775024f,-1.266922950744629f,1.3744525909423828f,-1.8161027431488037f,2.951124429702759f,0.23298430442810059f,-0.07841599732637405f,1.5281802415847778f,0.02296992391347885f,0.41290655732154846f,-0.8144248723983765f,0.9666826128959656f,-1.208092451095581f,-0.9689918160438538f,-1.0871679782867432f,-1.286173939704895f,-0.752160370349884f,0.008538910187780857f,1.3229076862335205f,-0.5307122468948364f,-1.2463301420211792f,-0.27644866704940796f,3.371122121810913f,-0.33787089586257935f,-1.3086823225021362f,0.14037950336933136f,1.0358809232711792f,0.32112056016921997f,0.022639764472842216f,0.8661656975746155f,0.9812530875205994f,0.8256194591522217f,-1.1491069793701172f,0.896453320980072f,-0.6697739362716675f,-0.4733733534812927f,-1.1455811262130737f,0.08171514421701431f,0.27891772985458374f,-0.4384491443634033f,0.7436513304710388f,-0.9037678837776184f,-0.2975301444530487f,0.18864354491233826f,0.46550536155700684f,-0.9468450546264648f,2.1519370079040527f,-0.43405088782310486f,2.49446964263916f,-0.019359691068530083f,0.05611523613333702f,-0.5084030628204346f,-0.9770478010177612f,2.105987071990967f,1.2480838298797607f,1.1999804973602295f,-2.0231454372406006f,-2.559880495071411f,0.32857269048690796f,-0.8895281553268433f,-2.4634571075439453f,2.0698082447052f,1.5870829820632935f,0.18537801504135132f,-1.2159523963928223f,-0.06354548037052155f,0.3758745789527893f,1.4039949178695679f,0.1746959239244461f,-2.683344841003418f,-0.410748153924942f,-1.1581755876541138f,0.5322403311729431f,0.368929922580719f,2.290428638458252f,-0.6117760539054871f,0.037374258041381836f,-0.1620044708251953f,0.035779062658548355f,1.8711267709732056f,1.0136628150939941f,-0.938406765460968f,-0.23507511615753174f,-1.0680546760559082f,-1.8243056535720825f,-0.00631767138838768f,-0.007985914126038551f,0.4307727813720703f,-1.3928555250167847f,-0.8895374536514282f,-0.3467986583709717f,-2.0300800800323486f,0.7740612030029297f,-0.40332701802253723f,-0.6722761392593384f,-0.2730669677257538f,-0.0404735691845417f,0.8366473317146301f,-1.051485300064087f,-0.49330228567123413f,-2.3775765895843506f,0.23622693121433258f,0.16151785850524902f,0.7141546607017517f,-0.09878614544868469f,-1.6468561887741089f,0.9704274535179138f,-1.8306540250778198f,0.30859050154685974f,0.7740083336830139f,-0.08425729721784592f,0.2720980942249298f,-1.2837520837783813f,1.4733691215515137f,0.17978627979755402f,0.7978969812393188f,0.7628876566886902f,0.2513452470302582f,-0.6404973268508911f,-0.042962685227394104f,0.05495264753699303f,-0.25049012899398804f,0.57319176197052f,0.42807722091674805f,0.5044932961463928f,-0.09161818772554398f,-1.5671790838241577f,2.2583680152893066f,-1.7302956581115723f,2.229196548461914f,0.06556876003742218f,-1.5289863348007202f,1.6578588485717773f,0.16718073189258575f,-1.7673155069351196f,-0.267366498708725f,-0.2983796298503876f,0.027515126392245293f,-1.0274922847747803f,1.9502171277999878f,0.8792858123779297f,0.04906710982322693f,1.7436096668243408f,0.6325004696846008f,-0.2541612982749939f,-0.8400025367736816f,-0.23891423642635345f,0.5659277439117432f,-0.2847617566585541f,0.43340736627578735f,0.013233186677098274f,2.9458882808685303f,-0.3423921465873718f,0.941499650478363f,1.0942764282226562f,-0.17781053483486176f,-0.24451550841331482f,-1.6415646076202393f,3.904663324356079f,1.346897006034851f,1.9709383249282837f,-0.4846959412097931f,0.09277988225221634f,1.0464907884597778f,-1.0114386081695557f,1.1340051889419556f,-0.9638634920120239f,0.3288901746273041f,0.49371927976608276f,-0.28828030824661255f,-0.5329083204269409f,0.8531731367111206f,2.9628195762634277f,-1.9643198251724243f,-0.15644291043281555f,0.9424254894256592f,0.5399545431137085f,-1.1153078079223633f,1.8598766326904297f,0.6442419290542603f,1.0089836120605469f,1.2878907918930054f,-1.9881038665771484f,2.8317058086395264f,1.1602861881256104f,-1.844463586807251f,-3.0570178031921387f,0.6290323734283447f,1.663747787475586f,-1.6320650577545166f,2.547261953353882f,1.835383415222168f,1.615139126777649f,0.7430793642997742f,0.3303638994693756f,0.8243595361709595f,0.7746881246566772f,-0.1923285275697708f,1.8520172834396362f,-0.4145340323448181f,2.8331923484802246f,-0.2586694657802582f,1.9912993907928467f,-0.6078344583511353f,-0.6402265429496765f,0.7967747449874878f,-0.06893665343523026f,-0.6729717254638672f,0.29690349102020264f,1.12904691696167f,-0.024891572073101997f,1.6625134944915771f,1.709723949432373f,0.739285409450531f,1.566907286643982f,-0.1637556552886963f,1.9980672597885132f,0.06043974310159683f,0.6422818303108215f,0.4249740540981293f,-1.521942377090454f,1.282397747039795f,0.17259757220745087f,0.5730740427970886f,0.24029819667339325f,-1.2804208993911743f,-0.2908363342285156f,0.059230417013168335f,-0.7273935675621033f,-0.09449266642332077f,0.6863933801651001f,1.5034409761428833f,0.8422051668167114f,0.5354863405227661f,-0.21026326715946198f,-0.1436523050069809f,-0.39220911264419556f,-0.5392526984214783f,0.1470942497253418f,-1.0631043910980225f,-1.1203612089157104f,1.252819538116455f,0.007314485497772694f,0.19819104671478271f,-0.7698088884353638f,-0.3526085615158081f,-1.9544934034347534f,1.4482810497283936f,0.27248191833496094f,-1.2741035223007202f,-0.8377154469490051f,-2.2069201469421387f,1.368956208229065f,-1.5118517875671387f,0.524700939655304f,-0.4883117079734802f,2.199824571609497f,0.5476477146148682f,0.8920593857765198f,-1.7252808809280396f,0.6861522197723389f,1.544434666633606f,0.8890660405158997f,-1.4649068117141724f,-0.892783522605896f,2.002742290496826f,-0.8498727679252625f,-0.01830108091235161f,-1.570434808731079f,-1.6042453050613403f,-0.3120894730091095f,0.32967475056648254f,-0.5513544678688049f,-1.0838327407836914f,1.3268976211547852f,0.7290648221969604f,0.10919851064682007f,0.764512836933136f,0.4353013336658478f,-1.488632082939148f,3.3710503578186035f,-0.1218806654214859f,2.1034576892852783f,-2.1237940788269043f,2.597689390182495f,-1.1688580513000488f,-0.7084618210792542f,0.06274686753749847f,1.0453886985778809f,-0.6061282157897949f,-0.029487257823348045f,-0.2095043808221817f,0.24608738720417023f,1.705068588256836f,1.2631341218948364f,0.30248016119003296f,2.165067672729492f,0.6001384854316711f,1.1462746858596802f,-2.6581003665924072f,-1.9829329252243042f,0.4851074814796448f,-0.14897018671035767f,-0.7044930458068848f,0.6305034756660461f,0.388045996427536f,0.013476487249135971f,-0.6463794708251953f,-2.191901683807373f,-0.038594432175159454f,0.3653838634490967f,-2.001847982406616f,1.112019419670105f,0.49303507804870605f,-1.591144323348999f,-0.7414734363555908f,0.4937164783477783f,0.008304892107844353f,0.013968843966722488f,0.4228125810623169f,-0.9648374915122986f,-0.64283287525177f,1.1877537965774536f,-1.1046602725982666f,0.24680699408054352f,0.4636841118335724f,0.02219153195619583f,0.4391997754573822f,0.491487979888916f,1.3356406688690186f,-0.055833593010902405f,-0.3618185222148895f,1.4252015352249146f,-0.16722935438156128f,0.6144564151763916f,-1.9512532949447632f,-1.8512139320373535f,0.27432411909103394f,-2.6987109184265137f,0.37442460656166077f,-1.1734250783920288f,0.35881537199020386f,-1.675338864326477f,-1.0225714445114136f,-1.4613420963287354f,-0.18225374817848206f,1.2202776670455933f,1.0696306228637695f,0.2510738968849182f,2.262212038040161f,0.018546735867857933f,2.395096778869629f,1.6159297227859497f,1.771372675895691f,-1.2806174755096436f,-0.29527682065963745f,0.5811581015586853f,0.862312376499176f,1.6834293603897095f,-1.528931736946106f,-1.5059185028076172f,-3.1542279720306396f,3.1131248474121094f,-1.1848727464675903f,1.108247995376587f,-0.8149212598800659f,0.7394925951957703f,3.3911566734313965f,1.1472268104553223f,0.4116857945919037f,1.637917160987854f,0.41757792234420776f,-0.45089244842529297f,0.1155407652258873f,-0.5964391231536865f,1.4452037811279297f,-0.4783417582511902f,-1.0434339046478271f,-1.1392416954040527f,-1.6995623111724854f,1.6233397722244263f,0.027044031769037247f,0.9907153844833374f,0.15809950232505798f,1.619773030281067f,1.4861761331558228f,1.0833337306976318f,1.487990140914917f,0.006802490912377834f,-0.37643325328826904f,-1.2717974185943604f,-1.761868953704834f,1.4071218967437744f,-0.25715237855911255f,1.1062543392181396f,0.7114192247390747f,-2.0714187622070312f,-0.4500868618488312f,2.082934617996216f,-0.3945252299308777f,-1.5015935897827148f,0.17752376198768616f,-1.6064037084579468f,-1.7997013330459595f,-1.0118157863616943f,-0.5313818454742432f,0.20002490282058716f,-0.24465681612491608f,-0.49915504455566406f,0.1801995187997818f,0.9886937737464905f,-0.7478365302085876f,-1.5840319395065308f,0.1445058137178421f,-1.2806881666183472f,0.2948397099971771f,1.6484414339065552f,-0.6022219657897949f,-2.5130879878997803f,-1.6480631828308105f,0.05707727000117302f,0.46958401799201965f,-0.905180811882019f,-0.31253865361213684f,-2.6270592212677f,0.004499627742916346f,0.9555362462997437f,0.4036746621131897f,0.1501794308423996f,0.2555849850177765f,-0.3198799192905426f,-1.1670600175857544f,0.025719227269291878f,0.6848921179771423f,-1.4980720281600952f,1.3811956644058228f,1.0328325033187866f,0.8341465592384338f,0.6202756762504578f,-0.9483784437179565f,2.0606043338775635f,1.1088694334030151f,1.070185899734497f,-2.5434820652008057f,-0.8369025588035583f,-0.5758345127105713f,-0.14965754747390747f,2.0742294788360596f,-0.42380619049072266f,-0.8434051275253296f,0.31586191058158875f,0.16276073455810547f,-1.8854399919509888f,2.367121934890747f,-3.7252132892608643f,1.1828633546829224f,-0.0384889654815197f,-0.24641631543636322f,0.38904857635498047f,1.0914170742034912f,0.20291204750537872f,-1.2708604335784912f,0.8753858208656311f,-1.137158751487732f,-0.3541924059391022f,-1.0229676961898804f,1.2653515338897705f,0.6542249917984009f,-1.014748215675354f,-1.7828848361968994f,-0.31565678119659424f,0.20600305497646332f,-0.32428812980651855f,-0.796262800693512f,0.19982397556304932f,0.16044098138809204f,1.0614664554595947f,-1.0581562519073486f,0.4247366487979889f,-1.5768107175827026f,3.3050003051757812f,-1.8229305744171143f,0.8538153767585754f,-0.31178149580955505f,-0.9490960240364075f,-1.2645821571350098f,0.6693182587623596f,-1.2585855722427368f,0.5458351969718933f,0.543404221534729f,-0.9404487609863281f,1.052384376525879f,-2.25746750831604f,3.3752734661102295f,0.05711750686168671f,0.6390909552574158f,-0.8789930939674377f,-0.7359002232551575f,-1.4256110191345215f,-1.211543083190918f,0.006537978537380695f,0.6536858081817627f,-0.2339327037334442f,-1.1947256326675415f,-0.0100533002987504f,-0.7788879871368408f,2.14616060256958f,-1.4266371726989746f,0.7630965709686279f,0.14535853266716003f,0.9101121425628662f,0.8866475224494934f,0.6536537408828735f,-0.3857017457485199f,-0.504727840423584f,-0.9320661425590515f,-1.709588646888733f,-0.5553205013275146f,-0.7344920635223389f,0.34880977869033813f,-0.6436276435852051f,-2.81303071975708f,2.242589235305786f,-0.7794023752212524f,0.7767474055290222f,-2.134403944015503f,-0.2854751646518707f,-1.954939603805542f,-1.2419326305389404f,-1.02098548412323f,0.5434340834617615f,-0.5857666730880737f,1.0764468908309937f,1.4906092882156372f,-0.15183700621128082f,0.9077348113059998f,0.4986744225025177f,-0.20616069436073303f,2.3055131435394287f,0.7092002630233765f,0.7473849654197693f,0.7919960618019104f,-2.0725483894348145f,0.9922245740890503f,2.4423296451568604f,0.15152829885482788f,1.3202933073043823f,-1.1332825422286987f,0.6076366901397705f,-0.11819945275783539f,0.403073787689209f,-2.5161848068237305f,-0.3247171640396118f,-1.0911589860916138f,0.28199395537376404f,0.4744606018066406f,0.012335250154137611f,0.24188610911369324f,-0.15116599202156067f,0.3433033525943756f,-0.05405851826071739f,0.7852124571800232f,-1.5188944339752197f,0.1731400191783905f,-0.5582787394523621f,-0.5535029768943787f,0.623785674571991f,-2.213632583618164f,0.07050793617963791f,-0.5450383424758911f,-1.0708242654800415f,-0.3572850823402405f,-0.0033042309805750847f,-1.2661736011505127f,1.061342716217041f,-0.4739219844341278f,-0.9345172643661499f,-0.3186170160770416f,1.3758264780044556f,1.089018702507019f,-1.9078700542449951f,-1.0849941968917847f,-0.17507320642471313f,1.3217295408248901f,0.803501307964325f,-1.103543758392334f,0.7521857023239136f,-0.8382604122161865f,-1.3199584484100342f,-0.6146004796028137f,0.19987742602825165f,-0.15076759457588196f,-0.45371192693710327f,0.9659580588340759f,-0.6052287817001343f,-0.10902513563632965f,1.4926642179489136f,0.04505370929837227f,-0.22179128229618073f,-0.2799515724182129f,-0.8316747546195984f,0.9446575045585632f,0.9118770360946655f,0.44035613536834717f,0.7548295855522156f,-2.393145799636841f,0.7406708002090454f,1.674892544746399f,0.8605520129203796f,0.6470003128051758f,-1.4606213569641113f,-0.49649685621261597f,2.7263410091400146f,-1.5955147743225098f,0.3169640004634857f,-0.9863699674606323f,-0.17566567659378052f,0.23148959875106812f,1.188055157661438f,1.7032521963119507f,-1.6120153665542603f,1.41989266872406f,2.417792320251465f,-1.7554147243499756f,-0.29865822196006775f,0.8905677795410156f,0.02839648351073265f,-2.1598737239837646f,3.2910964488983154f,-0.44800639152526855f,-0.9085531830787659f,-0.9200415015220642f,-0.8642756342887878f,0.835997998714447f,-0.17153044044971466f,0.029125932604074478f,-1.3775749206542969f,-1.0488771200180054f,0.3289307951927185f,0.5236402153968811f,-0.17848140001296997f,-0.17381371557712555f,-0.7867611050605774f,0.739956259727478f,-3.191396474838257f,-1.764717936515808f,-0.34314629435539246f,0.4372195899486542f,-0.6249881982803345f,-0.39497140049934387f,0.8358083963394165f,-0.0914231464266777f,-2.4062814712524414f,0.0683310329914093f,1.5039976835250854f,0.611531138420105f,1.6703284978866577f,-0.4916670620441437f,-0.8434085845947266f,-1.0669654607772827f,0.7690457701683044f,0.3555384874343872f,0.8539628386497498f,1.0716958045959473f,-0.14855638146400452f,-1.7242681980133057f,-0.6462874412536621f,0.26862287521362305f,1.555037021636963f,1.156722903251648f,2.0716910362243652f,-1.8626086711883545f,1.632273554801941f,-0.9528644680976868f,-1.5978535413742065f,-0.539528489112854f,0.810626208782196f,1.9884681701660156f,0.6645267605781555f,-0.46168768405914307f,-3.405440092086792f,-0.487112820148468f,-1.2014235258102417f,-0.3892582654953003f,-1.6199438571929932f,-0.28119856119155884f,0.4410479664802551f,-0.7279338240623474f,1.4777735471725464f,-0.3744106888771057f,0.6189870834350586f,0.9143320322036743f,0.8577114939689636f,0.23346258699893951f,0.4484798014163971f,-0.5774275064468384f,0.9307960271835327f,-1.2314702272415161f,-2.857562780380249f,2.6408441066741943f,-1.0105373859405518f,-1.3627440929412842f,-1.320552110671997f,-1.4952733516693115f,-0.07688074558973312f,1.3864387273788452f,-0.5324141383171082f,-1.1532323360443115f,-1.8901299238204956f,-2.8454015254974365f,-0.4162881076335907f,-2.0039987564086914f,-2.296027898788452f,0.5275098085403442f,-0.2383410632610321f,0.11445657163858414f,-0.2594928443431854f,-0.1350776106119156f,-0.8251363635063171f,1.8438435792922974f,-0.5139245986938477f,-1.165298581123352f,1.8325570821762085f,0.9468061923980713f,-0.10496139526367188f,-0.5944676399230957f,-0.07312615215778351f,-0.6910510063171387f,-1.9301995038986206f,-0.5917983055114746f,-0.04974186047911644f,0.40507280826568604f,2.406986951828003f,0.3362482190132141f,-1.6701165437698364f,-0.5673093199729919f,-0.666908323764801f,0.7879440784454346f,-0.6380097270011902f,0.6767065525054932f,0.4526113271713257f,-0.10403106361627579f,1.0430908203125f,0.3229455351829529f,-0.21117791533470154f,-0.6283307671546936f,-1.393379807472229f,0.6768177151679993f,-0.7272028923034668f,-0.5459121465682983f,0.41889315843582153f,-0.5619668364524841f,1.7082009315490723f,1.1690512895584106f,1.9122925996780396f,-0.4352494776248932f,0.15223649144172668f,-0.06047076731920242f,-0.3356687128543854f,-2.169330358505249f,-0.3405950665473938f,-0.21960614621639252f,1.7804224491119385f,-0.34144148230552673f,-1.7685956954956055f,0.5123203992843628f,0.6375284194946289f,0.33405807614326477f,0.8181975483894348f,0.48938223719596863f,0.07812103629112244f,-1.690181851387024f,1.6287766695022583f,1.5738813877105713f,1.000588297843933f,-0.3531302809715271f,0.3048488199710846f,-1.6451948881149292f,-1.1946144104003906f,0.3251016139984131f,-1.369127869606018f,0.49394533038139343f,0.17766179144382477f,-1.4020757675170898f,-0.7049133777618408f,-0.852059543132782f,0.07049847394227982f,0.8020787239074707f,1.1836934089660645f,-0.5816571116447449f,-0.8840271830558777f,0.42526429891586304f,-1.3144567012786865f,0.469441682100296f,1.3407158851623535f,-0.1413649171590805f,-0.3950916528701782f,-1.1526767015457153f,-0.15618374943733215f,-0.8764053583145142f,-0.13861869275569916f,-1.612448811531067f,0.9017841219902039f,0.5030103921890259f,-1.05451238155365f,1.7658450603485107f,1.4745992422103882f,-0.7999541759490967f,-0.7724373936653137f,0.00036995927803218365f,0.09923338145017624f,-1.9237143993377686f,-0.9830876588821411f,0.49910905957221985f,-0.27845391631126404f,0.9248873591423035f,0.41680800914764404f,-0.03185313940048218f,-0.49686864018440247f,0.12052998691797256f,0.24719911813735962f,-1.2289443016052246f,-1.404310703277588f,-1.2141132354736328f,0.5651806592941284f,2.5840983390808105f,0.4109136760234833f,-1.8266212940216064f,0.3916972577571869f,-0.6110849976539612f,1.1959201097488403f,1.4305936098098755f,-0.12955184280872345f,1.4395477771759033f,0.7162885069847107f,-0.39635875821113586f,-0.37333303689956665f,0.9918316006660461f,1.2109512090682983f,0.9463554620742798f,-1.8034839630126953f,-0.8657571077346802f,1.4296947717666626f,-0.7798068523406982f,-1.0134727954864502f,0.44037455320358276f,1.6474690437316895f,-1.2543809413909912f,0.08203057944774628f,0.7583051919937134f,0.5751850008964539f,0.11074154078960419f,-0.8083529472351074f,-0.6837704181671143f,1.109656572341919f,0.7181801199913025f,-0.9463144540786743f,-1.513676404953003f,-0.5549150109291077f,1.522830605506897f,-1.7192199230194092f,-0.7333300113677979f,0.6904146075248718f,-0.6302593350410461f,0.8518911600112915f,-0.7571724653244019f,0.5343695878982544f,-0.22817283868789673f,-0.4008234143257141f,-0.9715999960899353f,0.8007027506828308f,1.3182666301727295f,0.22229276597499847f,-1.1760015487670898f,1.5501283407211304f,0.5667211413383484f,-1.460830807685852f,-0.6766603589057922f,-0.8015434145927429f,0.028337758034467697f,1.561995029449463f,-1.344071626663208f,0.8159985542297363f,-0.9302825927734375f,0.8901576995849609f,0.18537336587905884f,0.5190820097923279f,-1.253859519958496f,-1.00090491771698f,0.49418315291404724f,-0.585269033908844f,0.10323049128055573f,-0.3657646179199219f,0.5025789737701416f,-0.6067593693733215f,-2.2910208702087402f,-3.958357572555542f,1.8103890419006348f,-0.8388055562973022f,-0.655453622341156f,0.7827287912368774f,1.6976646184921265f,0.6714677810668945f,-0.19171883165836334f,0.8097403049468994f,0.7827147841453552f,0.1654689908027649f,-0.8519846796989441f,-0.09170179814100266f,-0.5442929863929749f,0.7943688631057739f,-1.4682742357254028f,-0.1095227375626564f,-1.1204679012298584f,0.8193399310112f,-0.7074892520904541f,0.2697201371192932f,-0.6301442384719849f,0.8650641441345215f,0.6649906635284424f,1.1769943237304688f,1.8496755361557007f,-2.5950989723205566f,0.441440224647522f,1.224871039390564f,1.1333788633346558f,-0.21087712049484253f,1.649213194847107f,0.32065051794052124f,1.3745489120483398f,-0.2332271784543991f,-0.8828077912330627f,1.4383562803268433f,-1.6377981901168823f,-0.3623806834220886f,1.2387274503707886f,1.011687159538269f,-1.2821464538574219f,-1.3513109683990479f,-0.2665187120437622f,-0.5740842819213867f,1.0184893608093262f,0.931063175201416f,0.5772363543510437f,0.348047137260437f,0.7865549921989441f,0.007483010645955801f,0.26755714416503906f,-3.043757915496826f,-1.2988362312316895f,0.9340150952339172f,-1.9145961999893188f,0.9566500186920166f,0.5816296935081482f,0.03014213591814041f,0.7122130393981934f,-0.20266874134540558f,-1.0025092363357544f,1.3120841979980469f,3.6726651191711426f,-0.9634189009666443f,-0.7557514905929565f,0.7348593473434448f,1.6465027332305908f,-0.4392993748188019f,-1.2644386291503906f,-3.444506883621216f,-2.0311341285705566f,2.3681788444519043f,-0.5917953252792358f,0.6753288507461548f,-0.6311056613922119f,2.0569660663604736f,-1.366877555847168f,-1.6730207204818726f,1.0161399841308594f,0.12468093633651733f,-1.237154483795166f,-1.8324626684188843f,2.1744656562805176f,1.5468332767486572f,1.545769453048706f,-0.10888958722352982f,-1.3935538530349731f,0.0406964085996151f,0.31628280878067017f,-1.006877064704895f,2.3537604808807373f,-0.2408725619316101f,0.9570009708404541f,-0.014843462035059929f,-0.07216102629899979f,0.48574861884117126f,-0.9755825996398926f,0.2643338143825531f,-0.035590656101703644f,0.9472708106040955f,1.3475664854049683f,-0.8205063939094543f,-1.3052879571914673f,-1.4723976850509644f,1.123404622077942f,-1.6180318593978882f,0.7701107859611511f,-0.2737184762954712f,-0.7088325619697571f,0.3870910704135895f,0.5571297407150269f,-0.9855000376701355f,-1.0291448831558228f,0.5788575410842896f,1.6172667741775513f,-0.33549773693084717f,0.049510255455970764f,1.4958975315093994f,0.4961611330509186f,0.145393505692482f,-0.4788372218608856f,0.9194347858428955f,0.494093656539917f,-0.1203279048204422f,0.2591913044452667f,-0.4605264663696289f,-1.1350436210632324f,1.0690383911132812f,-0.7589387893676758f,-0.5843458771705627f,-1.527658224105835f,-1.849275827407837f,0.11358566582202911f,-0.5700493454933167f,-1.4108625650405884f,0.5168339014053345f,1.4238333702087402f,1.1323583126068115f,0.8341362476348877f,1.5165735483169556f,-0.8827870488166809f,-0.11524499207735062f,-0.27206045389175415f,1.1389702558517456f,0.08321408927440643f,-0.6399293541908264f,-0.5089830756187439f,-0.31970635056495667f,-0.7526801824569702f,0.8048105835914612f,-0.36921945214271545f,-0.1374274641275406f,-0.7188088893890381f,-0.09181176126003265f,-0.5363782644271851f,1.7576426267623901f,-0.8078666925430298f,-1.7443625926971436f,1.0908632278442383f,-0.5586481690406799f,-0.09203246235847473f,-1.3961913585662842f,3.1029558181762695f,0.9446165561676025f,2.136518716812134f,0.3641417920589447f,1.224227786064148f,-0.9633148312568665f,0.29402294754981995f,0.3217005133628845f,0.3281654119491577f,0.8345431089401245f,1.1212327480316162f,0.17647592723369598f,0.81875079870224f,-0.732746422290802f,1.3630902767181396f,-1.970564842224121f,0.6373173594474792f,-1.8909218311309814f,-3.029334545135498f,0.2649010419845581f,1.4393188953399658f,-0.31844276189804077f,0.2028888761997223f,0.5551660656929016f,-0.8856019973754883f,0.20215092599391937f,0.8216363191604614f,0.9164647459983826f,0.7700613141059875f,-0.27110958099365234f,1.2605679035186768f,-0.15945030748844147f,0.6285874247550964f,1.2370635271072388f,0.10217241197824478f,-0.0736391469836235f,1.2499152421951294f,0.7283025979995728f,0.47076812386512756f,1.228377103805542f,-0.25604724884033203f,-0.002748875180259347f,0.507714033126831f,-0.16055382788181305f,0.8014211654663086f,0.4455019235610962f,2.001584529876709f,-0.701387882232666f,-0.7193229794502258f,-1.2994134426116943f,-0.7542117238044739f,-2.080566644668579f,-0.7920111417770386f,0.9722210764884949f,-1.4645410776138306f,0.38704395294189453f,-0.41496068239212036f,0.11468875408172607f,-1.2679369449615479f,-0.4212641716003418f,1.3288357257843018f,2.5193629264831543f,-0.35892292857170105f,0.04888753592967987f,0.9878664612770081f,-0.4121502935886383f,-1.501356601715088f,-1.429789423942566f,-1.4257214069366455f,0.9313159584999084f,0.5075592994689941f,-0.013004393316805363f,0.08526547253131866f,2.8652665615081787f,-1.2167036533355713f,0.039916425943374634f,0.43877771496772766f,-1.4615072011947632f,-0.41372594237327576f,-0.8793684840202332f,0.8161892294883728f,1.8098957538604736f,-0.011558800004422665f,1.9870487451553345f,-0.4561881422996521f,-0.019071796908974648f,-0.17909272015094757f,0.3302956819534302f,-0.21797992289066315f,-0.3100755214691162f,0.2909838855266571f,-0.8826980590820312f,0.17673268914222717f,-1.5837271213531494f,-0.8515401482582092f,-0.0068322885781526566f,-0.4586170017719269f,0.3443510830402374f,1.2452499866485596f,-1.629412055015564f,-2.7628519535064697f,-2.09863018989563f,0.7819324135780334f,-2.3992278575897217f,2.2157766819000244f,-1.2294188737869263f,1.1263936758041382f,0.4309731125831604f,-2.152099370956421f,-0.0036289289128035307f,-0.9578574895858765f,-1.0279704332351685f,-0.6625849604606628f,-2.175522565841675f,0.24165314435958862f,1.3909144401550293f,3.615788221359253f,-0.5316050052642822f,1.9713832139968872f,0.1983528733253479f,-0.720172107219696f,1.2950454950332642f,0.25434058904647827f,0.9071210622787476f,1.0068970918655396f,-0.600578784942627f,-0.3456747531890869f,0.5710563659667969f,-1.1861287355422974f,0.748508870601654f,1.2810070514678955f,0.9449062943458557f,-0.40134257078170776f,-0.4149029552936554f,0.018010836094617844f,-1.523486614227295f,-0.4887743890285492f,0.7579069137573242f,-0.738191545009613f,-0.642854630947113f,0.18995364010334015f,-3.083946704864502f,0.26058632135391235f,-1.381650447845459f,0.5542463660240173f,-1.3411109447479248f,-1.299497127532959f,0.3958074450492859f,1.193619728088379f,1.1530804634094238f,-1.6740504503250122f,-1.1056479215621948f,-2.257690668106079f,1.0757923126220703f,-0.891315221786499f,1.9287612438201904f,1.393011450767517f,0.6058003306388855f,0.8903329968452454f,-0.2505173683166504f,-1.7747591733932495f,0.8656775951385498f,0.84621262550354f,1.5977015495300293f,-0.1519547998905182f,0.3732171058654785f,-0.36858227849006653f,-1.1925833225250244f,-1.8202781677246094f,-0.3909122049808502f,-0.1706009805202484f,-1.3361130952835083f,-0.1355111300945282f,1.47627592086792f,-0.6697826385498047f,-0.19516612589359283f,0.12884469330310822f,0.5247139930725098f,0.2245154082775116f,-0.47935402393341064f,-2.213930130004883f,-1.2726997137069702f,0.4284067451953888f,-0.3656277656555176f,-0.5094860196113586f,1.6106287240982056f,1.145370602607727f,-1.3263658285140991f,-1.4879032373428345f,-0.0951094701886177f,-1.8836431503295898f,-1.5182551145553589f,-0.8096978664398193f,1.0977146625518799f,0.9828727841377258f,0.3949543237686157f,-2.3535661697387695f,1.5038982629776f,0.4238961637020111f,-0.067790187895298f,0.7814857959747314f,-0.14309608936309814f,-0.35230913758277893f,0.07165157049894333f,0.4057614207267761f,-0.44283679127693176f,1.1768229007720947f,0.09856419265270233f,0.38042205572128296f,1.9629614353179932f,1.407677412033081f,0.41513898968696594f,-1.2573416233062744f,0.6766692399978638f,-1.82950758934021f,-1.3383533954620361f,-1.8069844245910645f,0.3659455478191376f,-0.32251355051994324f,-0.5782795548439026f,-2.5489394664764404f,1.090264916419983f,0.04671822860836983f,-1.107897162437439f,1.1803598403930664f,-0.9602208733558655f,0.49198588728904724f,0.9445222020149231f,0.02069859579205513f,0.24301180243492126f,-0.4509675204753876f,-0.1382749378681183f,-2.274081230163574f,-1.7395482063293457f,0.2833351194858551f,-0.17727558314800262f,0.0970531553030014f,0.20017509162425995f,-0.9207878708839417f,-0.6465522050857544f,-0.32377317547798157f,-0.2990858852863312f,2.1044607162475586f,-0.2418794184923172f,1.0339276790618896f,-0.6214080452919006f,1.8835358619689941f,-1.3467566967010498f,-1.0397663116455078f,0.8652984499931335f,-2.4705023765563965f,-1.5860041379928589f,1.8214514255523682f,0.5791401863098145f,0.2729234993457794f,-0.451443612575531f,0.7442241311073303f,-1.1528632640838623f,0.03480111435055733f,0.05437438562512398f,0.04103058949112892f,-0.5315932631492615f,1.4128217697143555f,-2.475980043411255f,-0.502456545829773f,-0.295437753200531f,-0.49903425574302673f,-0.9598294496536255f,0.8722996711730957f,-1.0275033712387085f,-0.12660132348537445f,0.22319509088993073f,-1.4321149587631226f,0.18120881915092468f,-0.6846978068351746f,-0.6859265565872192f,1.3152470588684082f,0.5064482688903809f,0.4518512189388275f,0.8607357740402222f,1.2902610301971436f,0.4024292826652527f,1.1165812015533447f,-0.9567060470581055f,0.4888966381549835f,-0.038273755460977554f,-0.9760127067565918f,0.056751709431409836f,-0.22747236490249634f,-1.6628623008728027f,-1.2300591468811035f,-0.717407763004303f,2.5280466079711914f,0.9119608998298645f,1.2603923082351685f,-2.8816959857940674f,-1.052251935005188f,-0.657572329044342f,-0.39798179268836975f,1.5094105005264282f,0.26676902174949646f,2.2740254402160645f,-1.6054601669311523f,2.1747639179229736f,-1.8717321157455444f,1.6680458784103394f,-3.1646883487701416f,-1.3188108205795288f,0.21673348546028137f,-0.2681081295013428f,0.36260658502578735f,-1.6354652643203735f,-0.287887305021286f,-0.9334951639175415f,-0.13801023364067078f,-1.4812815189361572f,2.064267158508301f,-0.25866934657096863f,-1.3915882110595703f,-0.163361057639122f,0.0742439478635788f,-0.46687859296798706f,-0.6622889041900635f,0.059231050312519073f,-0.8687027096748352f,0.42822322249412537f,0.6228588819503784f,0.3000008165836334f,0.9258795976638794f,-0.5275732278823853f,0.05522198602557182f,-0.5586180686950684f,1.0429610013961792f,-1.8415241241455078f,-0.8514246940612793f,-2.319793462753296f,0.5763906240463257f,-0.7643123269081116f,-1.09731125831604f,0.025503790006041527f,1.591051459312439f,-0.2881583571434021f,-0.14848339557647705f,-0.866591215133667f,1.2573572397232056f,1.4427051544189453f,-1.2866712808609009f,-1.7453622817993164f,-2.143064260482788f,-1.4606618881225586f,-1.5505913496017456f,-0.0844249576330185f,0.47234249114990234f,-0.4656508266925812f,-1.1205151081085205f,0.9041775465011597f,0.38405048847198486f,-2.0104339122772217f,-0.7638546228408813f,-0.756013035774231f,0.31782013177871704f,0.0399051234126091f,-1.0493077039718628f,1.1942881345748901f,0.9829039573669434f,-2.057117223739624f,-0.009615771472454071f,-0.32205137610435486f,-0.29941684007644653f,0.3612554669380188f,1.9800009727478027f,0.29243651032447815f,-1.1280206441879272f,0.2129707783460617f,-0.23936261236667633f,-1.8502010107040405f,-1.4235774278640747f,-0.8197726607322693f,0.23281848430633545f,1.7252253293991089f,1.044521450996399f,-0.13796091079711914f,-0.8495212197303772f,1.1234692335128784f,-0.30374640226364136f,-0.5668472051620483f,1.5504872798919678f,0.030537500977516174f,-1.3468525409698486f,0.5502271056175232f,0.04123450815677643f,-0.32863515615463257f,-1.3610453605651855f,-2.8166232109069824f,-0.5490298271179199f,0.8995400071144104f,0.5588274002075195f,-0.813610315322876f,-1.0064053535461426f,0.6668316721916199f,-0.718866229057312f};
alignas(16) float batch_normalization_10_A[] = {6.427140712738037f,3.0918233394622803f,2.971226215362549f,2.4775969982147217f,-3.534148931503296f,-0.7595987319946289f,-1.2230818271636963f,4.55009651184082f,-1.5115137100219727f,0.9498412609100342f,8.276556015014648f,2.1130905151367188f,5.506711006164551f,0.39464545249938965f,-1.8509156703948975f,4.37265157699585f,1.1128580570220947f,-1.2337665557861328f,6.0995588302612305f,-0.13573098182678223f,1.2293369770050049f,1.1228053569793701f,-0.9954425692558289f,1.9596564769744873f,-0.9767731428146362f,-0.22395175695419312f,3.7520194053649902f,0.20516347885131836f,10.061981201171875f,3.0166127681732178f,2.931032657623291f,4.1449480056762695f,3.455836772918701f,2.307493209838867f,0.607994794845581f,3.6187522411346436f,5.746105670928955f,-3.936293125152588f,4.11524772644043f,1.9161262512207031f};
alignas(16) float DetectionLayer_internal_0_VALUES[] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
alignas(16) float DetectionLayer_internal_1_W[] = {0.009893767535686493f,0.017220905050635338f,-0.0012024635216221213f,0.009434585459530354f,-0.060563359409570694f,0.007632949855178595f,-0.007641699630767107f,-0.0016714476514607668f,-0.01559116318821907f,0.009935167618095875f,-0.0023531110491603613f,-0.012330306693911552f,0.004856343846768141f,-0.004861847031861544f,-0.05620179325342178f,-0.03835304081439972f,0.007459585089236498f,-0.0011968743056058884f,0.031573351472616196f,0.007761798333376646f,-0.007761784829199314f,0.01188756711781025f,-0.006136334501206875f,-0.002976055024191737f,0.0043852790258824825f,-0.04454389214515686f,0.015632303431630135f,-0.015629902482032776f,-0.018096599727869034f,-0.004229776561260223f,-0.003107405500486493f,0.002835511928424239f,-0.058843616396188736f,0.00937932264059782f,-0.00937934685498476f,0.0017428840510547161f,-0.0019035920267924666f,-0.00748895388096571f,0.0026302365586161613f,0.014855360612273216f,0.007611463312059641f,-0.007615391165018082f,0.003858498763293028f,0.006009552162140608f,-0.0008169435895979404f,0.0009604722145013511f,0.030203143134713173f,-0.06889323145151138f,0.0688895657658577f,-0.0017819841159507632f,-0.010621492750942707f,0.007522362284362316f,-0.0004656241799239069f,-0.13677029311656952f,0.004979019984602928f,-0.004979642108082771f,0.005872218869626522f,0.025368696078658104f,-0.004124392755329609f,-0.003364716423675418f,0.15920083224773407f,0.009453401900827885f,-0.009449489414691925f,0.012237755581736565f,0.010259799659252167f,-0.015239820815622807f,-0.01134317833930254f,-0.0026265322230756283f,-0.014732886105775833f,0.014736808836460114f,0.008065769448876381f,-0.013865029439330101f,-0.008326378650963306f,-0.00028104838565923274f,0.05534764751791954f,0.008945298381149769f,-0.008944071829319f,-0.02614343911409378f,0.005568666849285364f,0.0031456092838197947f,-0.004320807754993439f,-0.0494406595826149f,0.007385822478681803f,-0.007383213844150305f,-0.0338401161134243f,-0.012491673231124878f,-0.009778461419045925f,0.0010860480833798647f,0.007076955400407314f,-0.06115993112325668f,0.06116039678454399f,0.10314897447824478f,0.014416982419788837f,-0.02167186513543129f,-0.002278271596878767f,0.07354220747947693f,0.0038252603262662888f,-0.0038269597571343184f,0.016145596280694008f,-0.005576286930590868f,-0.0032648846972733736f,0.0014986925525590777f,-0.16162656247615814f,0.006021928507834673f,-0.0060192677192389965f,-0.0387926921248436f,0.018493173643946648f,0.007045007310807705f,0.007641828618943691f,-0.004038956947624683f,-0.010329535230994225f,0.01032914686948061f,0.008321902714669704f,-0.02214728482067585f,-0.008242696523666382f,-0.002312134252861142f,-0.006045095156878233f,0.004857825580984354f,-0.00485839881002903f,-0.045479580760002136f,0.035456687211990356f,0.004723221529275179f,0.0013006394729018211f,0.014012633822858334f,0.005776763428002596f,-0.0057715680450201035f,0.008313285186886787f,0.0005185924237594008f,-0.006541072856634855f,-0.011151346378028393f,-0.12283995747566223f,-0.017027776688337326f,0.017033107578754425f,0.048569776117801666f,-0.00343229784630239f,-0.012352765537798405f,0.003998000640422106f,-0.04090191423892975f,0.0043091182596981525f,-0.004310653544962406f,0.028369223698973656f,-0.013810795731842518f,-0.017387447878718376f,0.0023854896426200867f,-0.0835878923535347f,0.0011534957448020577f,-0.0011589674977585673f,-0.012649877928197384f,0.01207027304917574f,0.00793042778968811f,0.007404116448014975f,-0.12378264963626862f,-0.0532543882727623f,0.053253382444381714f,0.020479897037148476f,0.01285727508366108f,0.001997781451791525f,0.0043120295740664005f,0.0017049125162884593f,0.011517825536429882f,-0.011518998071551323f,0.01690613105893135f,0.01781170442700386f,-0.001681234105490148f,0.0043793548829853535f,-0.08201991766691208f,0.003113785060122609f,-0.00311456760391593f,-0.01141801755875349f,0.007126952987164259f,0.012467822059988976f,-0.00509305763989687f,-0.005709955468773842f,-0.05603007599711418f,0.05602340027689934f,0.007061654701828957f,-0.0025777847040444613f,0.0005384284886531532f,0.001723744091577828f,-0.004011946264654398f,0.013400621712207794f,-0.013406016863882542f,-0.002498383866623044f,0.0029276784043759108f,-0.015495539642870426f,-0.0016330232610926032f,-0.083082415163517f,0.005949772894382477f,-0.005950007122009993f,0.01213917974382639f,-0.011843450367450714f,-0.0015207077376544476f,0.00045729478006251156f,-0.06479041278362274f,-0.0592600554227829f,0.05924658477306366f,0.012953002005815506f,-0.011329090222716331f,-0.0020671309903264046f,-0.000893575488589704f,0.07462251931428909f,0.006834061350673437f,-0.006830147933214903f,0.0012518144212663174f,-0.025998501107096672f,-0.011225358583033085f,-0.0008950440096668899f,0.034291911870241165f,0.002220463240519166f,-0.002220878843218088f,0.019842464476823807f,0.0340975858271122f,0.0020753401331603527f,0.010048327967524529f,-0.08460894972085953f,-0.02834639698266983f,0.02835206687450409f,-0.014388998970389366f,0.00928959809243679f,0.0006303365807980299f,-0.001399932079948485f,-0.02481120079755783f,0.007842639461159706f,-0.007844379171729088f,0.032011955976486206f,0.021439116448163986f,-0.0228404700756073f,0.0024191259872168303f,-0.056868501007556915f,0.004573566373437643f,-0.004574798513203859f,-0.011054410599172115f,0.010373666882514954f,-0.0035430805291980505f,0.0021095655392855406f,-0.08651349693536758f,-0.037017978727817535f,0.03701067343354225f,-0.01896512322127819f,0.004553303588181734f,0.007136537693440914f,-0.0023197794798761606f,-0.047727152705192566f,0.01028240192681551f,-0.010283282957971096f,-0.03268126770853996f,0.009144828654825687f,0.007493356708437204f,0.00016461964696645737f,0.09810764342546463f,0.011609440669417381f,-0.01160331442952156f,0.008416843600571156f,-0.01753239892423153f,0.0022918586619198322f,-0.002582443878054619f,-0.1207394003868103f,0.0263195987790823f,-0.02631198801100254f,0.014375193044543266f,-0.0055968621745705605f,-0.00022881045879330486f,-0.003977673128247261f,0.03269112855195999f,0.011174914427101612f,-0.01116909645497799f,0.045662879943847656f,-0.016366388648748398f,0.0034939968027174473f,-0.0009145934018306434f,-0.026054034009575844f,0.004769019316881895f,-0.004768576007336378f,-0.04209297522902489f,-0.021765798330307007f,-0.0018491103546693921f,0.005834321025758982f,-0.004973347298800945f,0.06451509892940521f,-0.06450705230236053f,-0.009322738274931908f,-0.01106376200914383f,0.004497123416513205f,-0.0033176993019878864f,-0.04642413929104805f,0.0063993544317781925f,-0.006391618400812149f,0.00464267935603857f,-0.00980764627456665f,5.2408300689421594e-05f,0.0009252845775336027f,0.012599623762071133f,0.0052450597286224365f,-0.005241186358034611f,0.011860172264277935f,-0.004783561918884516f,0.0021175029687583447f,-0.004824331495910883f,0.17877668142318726f,-0.12068147957324982f,0.12068318575620651f,0.005082675255835056f,-0.0044646551832556725f,-0.019486434757709503f,-0.002088816137984395f,-0.08408825844526291f,0.004204889759421349f,-0.0042024957947432995f,0.02454790472984314f,-0.014907090924680233f,-0.00539387809112668f,-0.0005313016008585691f,-0.09241171926259995f,0.005103430710732937f,-0.005102512892335653f,-0.03255293890833855f,-0.006553734187036753f,0.01618606224656105f,0.01657288148999214f,0.07708102464675903f,-0.01435625459998846f,0.014357097446918488f,-0.03190522640943527f,0.022781232371926308f,0.01208171620965004f,-0.0036125632468611f,0.06286349147558212f,0.0023905872367322445f,-0.0023906873539090157f,0.00023135544324759394f,0.0074850209057331085f,-0.0021133970003575087f,0.003273487091064453f,-0.00996301881968975f,0.003018755931407213f,-0.003019534284248948f,-0.028901489451527596f,0.0248690377920866f,0.0003192519361618906f,0.004096404183655977f,-0.06336832046508789f,0.02636408619582653f,-0.02637782134115696f,0.008426540531218052f,-0.016182132065296173f,-0.0030915564857423306f,-0.002083730651065707f,0.0579572431743145f,0.016825580969452858f,-0.01682739518582821f,0.007046564016491175f,-0.0010243098950013518f,0.00683129858225584f,-0.0004660016857087612f,0.038850799202919006f,0.02186574973165989f,-0.021870529279112816f,0.02940557710826397f,-0.011295014061033726f,0.0006061081076040864f,-0.0040450082160532475f,0.008927704766392708f,-0.039421018213033676f,0.03941192105412483f,0.002402709564194083f,-0.003308257320895791f,0.004605576395988464f,0.0009435736574232578f,-0.12670399248600006f,0.005671539809554815f,-0.005678393412381411f,-0.02319554053246975f,-0.006241130642592907f,-0.00725477235391736f,0.0014519010437652469f,-0.07878202944993973f,0.006882862187922001f,-0.006884275935590267f,-0.002748401602730155f,0.008612778969109058f,-0.005841400474309921f,-0.0017155667301267385f,0.060162290930747986f,0.04065258055925369f,-0.04065868631005287f,0.032724350690841675f,0.0007621276308782399f,-0.00923894066363573f,0.0007791167590767145f,-0.15589456260204315f,0.0028717322275042534f,-0.0028720670379698277f,0.039926595985889435f,0.0017127704340964556f,-0.0011520111002027988f,0.0037636759225279093f,-0.12251145392656326f,-0.0008144503808580339f,0.0008215542766265571f,0.002239533932879567f,0.01692131534218788f,0.0037162064108997583f,0.0015861770370975137f,-0.05380566790699959f,-0.009308291599154472f,0.009299581870436668f,0.007261688355356455f,-0.017512565478682518f,-0.002055102726444602f,-0.006279292516410351f,-0.1872623860836029f,0.004938668571412563f,-0.004937124438583851f,-0.08797664195299149f,-0.004259013570845127f,0.01460069976747036f,-0.0006089666276238859f,-0.502569317817688f,0.0037933997809886932f,-0.0037933432031422853f,-0.02794918604195118f,-0.0036406833678483963f,0.006866226904094219f,-0.00222523114643991f,-0.08218049257993698f,-0.009148257784545422f,0.00915002916008234f,-0.011921929195523262f,0.00966040138155222f,0.010858179070055485f,-0.00043587503023445606f,-0.008489765226840973f,0.009206767193973064f,-0.009200986474752426f,-0.042541373521089554f,0.001086483825929463f,0.0028293603099882603f,-0.0009510164964012802f,0.04016796872019768f,0.013202921487390995f,-0.01319948025047779f,0.02490483969449997f,-0.004468926694244146f,-0.005969861056655645f,-0.007835748605430126f,0.14293740689754486f,-0.08920333534479141f,0.08919507265090942f,-0.031369030475616455f,0.003452078439295292f,0.0064842416904866695f,-0.0028786996845155954f,-0.060322366654872894f,0.011846747249364853f,-0.011849120259284973f,-0.027020815759897232f,-0.00848369300365448f,-0.0033943045418709517f,-0.0006361206760630012f,-0.021413253620266914f,0.01906074397265911f,-0.019062789157032967f,0.01461569033563137f,0.01862967386841774f,0.004647138062864542f,0.005586512386798859f,0.0037074116989970207f,0.06376032531261444f,-0.06376062333583832f,0.003497442929074168f,0.019037634134292603f,-0.0029137954115867615f,0.0009134403662756085f,-0.08385726064443588f,0.007742214482277632f,-0.007738001178950071f,-0.018818382173776627f,0.011905137449502945f,-0.0032436340115964413f,-6.0488582676043734e-05f,-0.022615978494286537f,0.003220638958737254f,-0.003218430560082197f,0.007805225905030966f,-0.018211107701063156f,-0.001931091770529747f,-0.002395821502432227f,-0.1318713277578354f,0.029804622754454613f,-0.02980518713593483f,-0.051224879920482635f,0.006107615772634745f,0.01612447015941143f,0.005540178623050451f,-0.15896816551685333f,0.007589281070977449f,-0.007590549066662788f,-0.046238820999860764f,0.0132371811196208f,0.00882092583924532f,0.0023366687819361687f,0.13740237057209015f,0.02152354083955288f,-0.021524593234062195f,-0.010778680443763733f,0.0038080348167568445f,-0.004049963783472776f,0.0014763595536351204f,0.06617258489131927f,-0.04874137043952942f,0.04874230548739433f,0.005538707599043846f,0.004026614595204592f,-0.0041197421960532665f,-0.0023310754913836718f,-0.010285725817084312f,0.008672012947499752f,-0.008671656250953674f,0.005992135498672724f,-0.009535765275359154f,0.0017006321577355266f,-0.0002613794640637934f,0.043912626802921295f,0.007571352180093527f,-0.007575079798698425f,0.004127505235373974f,0.007955349050462246f,0.003052983433008194f,-0.0011992051731795073f,0.1043737456202507f,-0.018133431673049927f,0.018126560375094414f,-0.010451408103108406f,0.0002038849488599226f,0.006437960080802441f,-0.0011414648033678532f,0.0695948600769043f,0.012286745011806488f,-0.01228407397866249f,0.005571986548602581f,-0.023478763177990913f,-0.0047533027827739716f,-0.0012062815949320793f,0.09117681533098221f,0.012778937816619873f,-0.012777039781212807f,-0.007256751414388418f,-0.008335260674357414f,0.0015878055710345507f,-0.0009170910925604403f,0.03009931370615959f,-0.027324635535478592f,0.02732563391327858f,0.007475045043975115f,0.0011581841390579939f,0.0012552485568448901f,-0.000492741062771529f,0.12441503256559372f,0.007605177816003561f,-0.007603599224239588f,0.022797977551817894f,0.005683567374944687f,0.005426027812063694f,0.0016974385362118483f,-0.08158446103334427f,0.007121832109987736f,-0.007123428862541914f,-0.00941407959908247f,0.0033772580791264772f,-0.0015406711027026176f,0.008353322744369507f,0.051855213940143585f,0.04009460285305977f,-0.040091533213853836f,-0.008295616135001183f,-0.008094005286693573f,-0.001592450775206089f,-0.002496378729119897f,0.04196600243449211f,0.0154811916872859f,-0.015484274365007877f,-0.022882094606757164f,0.014172629453241825f,0.0047868103720247746f,0.000911296927370131f,-0.12113140523433685f,0.0054924460127949715f,-0.005490931216627359f,0.013067859224975109f,0.01839274726808071f,-0.0040064239874482155f,0.009688278660178185f,-0.011140750721096992f,-0.0038883774541318417f,0.0038852570578455925f,0.02038353681564331f,0.0064711421728134155f,-0.017407920211553574f,0.004043434746563435f,0.008284104987978935f,0.004314296413213015f,-0.004314273130148649f,0.007594611961394548f,0.003051091218367219f,0.005249002017080784f,0.002040914259850979f,-0.09162969887256622f,0.0016422157641500235f,-0.0016449270769953728f,-0.004506200086325407f,-0.017463283613324165f,0.013022119179368019f,0.01039902027696371f,-0.002764173084869981f,0.012683884240686893f,-0.012673216871917248f,0.009941608645021915f,-0.0008489712490700185f,0.009293138980865479f,0.0029234362300485373f,0.06205648183822632f,0.008499758318066597f,-0.008504260331392288f,0.014467213302850723f,0.002887527458369732f,-0.0002506194286979735f,0.0008013233891688287f,0.011925332248210907f,0.008273283950984478f,-0.008267266675829887f,-0.02289184182882309f,-0.005821167025715113f,-0.014094099402427673f,-0.0076670837588608265f,-0.010204455815255642f,-0.04068190976977348f,0.04068193957209587f,-0.05708467215299606f,0.01331113651394844f,-0.0039766766130924225f,0.001703961635939777f,0.12458151578903198f,0.004589403048157692f,-0.004594974685460329f,-0.05377161502838135f,0.04761051759123802f,0.03921464830636978f,-0.062174901366233826f,0.01628841459751129f,0.003531890455633402f,-0.0035263500176370144f,-0.008406320586800575f,0.014932578429579735f,-0.005374755244702101f,-0.013409736566245556f,0.008321506902575493f,0.005590981338173151f,-0.005584746599197388f,-0.09369373321533203f,-0.006559931673109531f,0.0026048864237964153f,0.003155763726681471f,-0.016978424042463303f,0.013525611720979214f,-0.013526937924325466f,-0.013353712856769562f,-0.01418792363256216f,-0.008544457145035267f,-0.00010295228275936097f,0.06354345381259918f,0.016363395377993584f,-0.01636148802936077f,-0.0009070425294339657f,0.00035679794382303953f,0.006853053811937571f,-0.0014691193355247378f,0.05309832841157913f,-0.09819488227367401f,0.09818707406520844f,-0.007422173861414194f,-2.6200767024420202e-05f,0.00660014059394598f,-0.0013799858279526234f,-0.025663841515779495f,0.009787208400666714f,-0.009782988578081131f,-0.01676199771463871f,-0.003506417153403163f,0.009336685761809349f,-0.0013686609454452991f,-0.10543539375066757f,0.017225973308086395f,-0.017228534445166588f,0.002672302071005106f,-0.007526662200689316f,-0.001054234802722931f,-0.003250406589359045f,0.05997364968061447f,-0.07631005346775055f,0.07630903273820877f,-0.005569629371166229f,-0.01957940123975277f,-0.0017735258443281054f,-0.0035656695254147053f,0.06324312090873718f,0.012374429032206535f,-0.012369458563625813f,-0.01343595888465643f,0.0005867251893505454f,0.00636228546500206f,-0.0022781400475651026f,-0.1605575829744339f,0.007121670059859753f,-0.007125364150851965f,0.010012063197791576f,0.0018130602547898889f,-0.00363508821465075f,0.00876107532531023f,-0.024319076910614967f,-0.015138153918087482f,0.015136788599193096f,0.01790233515202999f,-0.014315342530608177f,-0.006155075505375862f,0.0008884125854820013f,-0.05628080666065216f,0.009594962000846863f,-0.009590214118361473f,-0.013354751281440258f,0.006143409293144941f,0.00108615611679852f,-4.209119651932269e-05f,0.02924225851893425f,0.009915562346577644f,-0.00991144496947527f,0.022051969543099403f,0.012289775535464287f,0.005497243255376816f,0.010007189586758614f,-0.14394141733646393f,0.028159739449620247f,-0.02816453017294407f,-5.111886639497243e-05f,-0.0029332831036299467f,-0.0038095349445939064f,-0.004670176189392805f,0.0014373159501701593f,0.01375399436801672f,-0.013753386214375496f,0.0007197559461928904f,-0.00213145324960351f,-8.247634468716569e-07f,-0.002699035219848156f,-0.06911574304103851f,0.010480798780918121f,-0.010473894886672497f,-0.010527603328227997f,-0.013025655411183834f,-0.000741298426873982f,-0.005222303792834282f,0.020840803161263466f,-0.031281448900699615f,0.031289245933294296f,0.01654270850121975f,0.013141805306077003f,0.00520758330821991f,-0.0003447909257374704f,-0.03030191920697689f,0.007404819596558809f,-0.007404274307191372f,-0.018085310235619545f,0.02706577256321907f,0.005481299944221973f,0.0037525100633502007f,0.039148785173892975f,0.006843644194304943f,-0.006845648866146803f,-0.002298544393852353f,0.01143189799040556f,-0.002373754046857357f,0.004198239650577307f,-0.1796244978904724f,-0.009713560342788696f,0.009710051119327545f,0.012340161949396133f,0.010353345423936844f,-0.0021093867253512144f,0.0035901961382478476f,-0.07735832780599594f,0.016682203859090805f,-0.016680434346199036f,0.038025252521038055f,0.004970040172338486f,-0.0025450170505791903f,-0.0017607408808544278f,-0.11098750680685043f,0.018783411011099815f,-0.018784236162900925f,0.033908095210790634f,0.011727146804332733f,-0.00505383824929595f,-0.008522153832018375f,0.09526267647743225f,-0.029480701312422752f,0.02948891371488571f,0.020860016345977783f,-0.004516470246016979f,-0.008859999477863312f,0.0021320288069546223f,-0.18742895126342773f,0.0057604932226240635f,-0.005760257598012686f,0.010809022933244705f,-0.00937911681830883f,0.002414706628769636f,-0.000983638339675963f,0.0017498437082394958f,0.012699678540229797f,-0.012699686922132969f,0.021380363032221794f,0.022058384492993355f,0.006264776922762394f,0.0011504566064104438f,-0.05927764251828194f,-0.02628943882882595f,0.026287255808711052f,-0.03774705156683922f,0.004917449783533812f,0.011073031462728977f,-0.0027747657150030136f,-0.11316090077161789f,0.0034755165688693523f,-0.0034748585894703865f,-0.050342708826065063f,0.007009763270616531f,0.006627678871154785f,0.0006339004612527788f,0.09754565358161926f,0.017990579828619957f,-0.017988070845603943f,-0.02623474970459938f,-0.03772773966193199f,-0.01970570534467697f,0.1277756243944168f,-0.009308860637247562f,0.08256175369024277f,-0.08256648480892181f,0.011226683855056763f,-0.10450175404548645f,-0.005336076486855745f,-0.0007604517741128802f,0.0009200922213494778f,0.022320076823234558f,-0.02232510782778263f,0.015900908038020134f,-0.09220544993877411f,-0.012941042892634869f,-0.00041533721378073096f,0.04914820194244385f,0.013763547874987125f,-0.013762276619672775f,-0.0023968112654983997f,0.007711739279329777f,0.0039993757382035255f,0.003437108127400279f,-0.13571037352085114f,-0.012705447152256966f,0.012709802947938442f,0.011392831802368164f,0.013920212164521217f,0.014104798436164856f,-0.0003473703400231898f,0.007203771732747555f,0.027879612520337105f,-0.027881527319550514f,0.017907986417412758f,0.0026844702661037445f,-0.005812383722513914f,0.0037866034545004368f,-0.016262367367744446f,0.013917024247348309f,-0.013912913389503956f,-0.042546819895505905f,-0.0020337908063083887f,-0.004358966369181871f,0.004361957311630249f,-0.13118882477283478f,0.03429078683257103f,-0.03428315743803978f,0.02773444913327694f,-0.008832350373268127f,0.0014748014509677887f,0.0009030248620547354f,0.16008281707763672f,0.027687588706612587f,-0.027687309309840202f,0.028676770627498627f,0.001331886276602745f,-0.009116542525589466f,0.0005845042760483921f,-0.06242697313427925f,0.023481249809265137f,-0.023478075861930847f,-0.0069241295568645f,-0.009425562806427479f,-0.033178962767124176f,-0.02487475425004959f,-0.11182290315628052f,-0.15712179243564606f,0.15711429715156555f,0.001480819541029632f,-0.012806345708668232f,-0.009985370561480522f,0.0018879709532484412f,0.29137057065963745f,0.04283119738101959f,-0.04283404350280762f,0.025515254586935043f,0.009032899513840675f,-0.008791772648692131f,-0.0050138733349740505f,0.05059918388724327f,0.02611151523888111f,-0.026109572499990463f,0.07328248023986816f,-0.014657540246844292f,-0.014617095701396465f,-0.016975831240415573f,-0.01934831216931343f,-0.047891851514577866f,0.04789652302861214f,0.0032315838616341352f,0.002755508990958333f,0.00758334482088685f,-0.002467449987307191f,-0.14602245390415192f,0.016809558495879173f,-0.01680932752788067f,0.005142807029187679f,0.012909607961773872f,-0.0019651518668979406f,7.763229223201051e-05f,-0.007106821984052658f,0.024211471900343895f,-0.024210937321186066f,0.0002877957886084914f,0.0015975097194314003f,0.002968148561194539f,-0.002763417549431324f,-0.12843871116638184f,-0.01675684191286564f,0.016750985756516457f,0.004708406049758196f,0.00814420823007822f,-0.0022172080352902412f,-0.0030578651931136847f,0.08643071353435516f,0.028335124254226685f,-0.028340239077806473f,-0.0013058865442872047f,-0.0146581856533885f,-0.020355887711048126f,0.002024098765105009f,-0.11573043465614319f,0.021472707390785217f,-0.021473353728652f,0.001130099524743855f,0.02688976563513279f,0.008307318203151226f,0.006998877972364426f,-0.0770111232995987f,0.010184332728385925f,-0.010186645202338696f,-0.028189117088913918f,0.0019550491124391556f,-0.006389050744473934f,-0.006591953337192535f,-0.2862597703933716f,0.007086635567247868f,-0.007082707714289427f,-0.030591540038585663f,-0.04469665139913559f,-0.010897130705416203f,0.007612582296133041f,-0.14586137235164642f,0.001965251285582781f,-0.0019677325617522f,-0.022417740896344185f,0.008277729153633118f,-0.005139910150319338f,0.00673320097848773f,0.03656836599111557f,-0.061661988496780396f,0.06166322901844978f,-0.19646301865577698f,-0.01777736097574234f,-0.030062207952141762f,0.0015705348923802376f,-0.10893936455249786f,0.01039747055619955f,-0.010401648469269276f,0.004255563020706177f,0.020843518897891045f,0.0002781537768896669f,0.006691639311611652f,-0.10974240303039551f,-5.860326928086579e-05f,5.8056120906258e-05f,0.017750874161720276f,-0.006822438444942236f,0.010625903494656086f,-0.009608343243598938f,-0.07871739566326141f,0.08400115370750427f,-0.08400514721870422f,0.007537174038589001f,-0.0014245440252125263f,-0.0033628700766712427f,-0.00031552842119708657f,-0.02304275520145893f,0.02491096407175064f,-0.024915821850299835f,0.00013826826761942357f,0.0038399233017116785f,0.0013589165173470974f,0.0018649855628609657f,-0.2354789525270462f,0.0024544065818190575f,-0.002452409826219082f,-0.010031587444245815f,-0.03640725836157799f,0.014328897930681705f,0.017102383077144623f,0.25216761231422424f,0.1462087631225586f,-0.14621016383171082f,0.014425314031541348f,-0.01343776099383831f,0.01080611813813448f,-0.011112698353827f,-0.27752918004989624f,0.007193028461188078f,-0.007190126925706863f,0.044500306248664856f,-0.005282606463879347f,-0.0015696535119786859f,-0.0002678673481568694f,-0.02200772985816002f,-0.0008548422483727336f,0.0008602303569205105f,-0.0052695926278829575f,0.07838091254234314f,-0.011022006161510944f,0.016639195382595062f,-0.1309642195701599f,-0.03941246494650841f,0.039417970925569534f,0.019857220351696014f,0.0858667716383934f,-0.025661462917923927f,0.00891178660094738f,-0.2014220505952835f,0.010576737113296986f,-0.010576934553682804f,0.02150815539062023f,0.03723195940256119f,-0.025673510506749153f,0.013970262371003628f,-0.1839606910943985f,0.0009186867391690612f,-0.0009129681275226176f,-0.023618750274181366f,0.00974954105913639f,0.0005886140279471874f,0.004365300759673119f,0.007498560938984156f,0.02987862005829811f,-0.029871078208088875f,-0.0015568480594083667f,0.0032775974832475185f,0.0009077573195099831f,0.0007394676213152707f,0.09910469502210617f,0.022352328523993492f,-0.02234867960214615f,0.017688779160380363f,0.002901027211919427f,-0.005101424176245928f,0.0006231315201148391f,-0.08754152804613113f,0.010041151195764542f,-0.010035702027380466f,0.0015037808334454894f,0.053163789212703705f,-0.028033358976244926f,0.01270272396504879f,-0.2226264327764511f,-0.03238290920853615f,0.032380446791648865f,0.020260700955986977f,0.051793161779642105f,0.0004411718691699207f,0.00022860597528051585f,-0.23500743508338928f,-0.00034954925649799407f,0.0003551708359736949f,-0.009317440912127495f,0.022730741649866104f,0.00405479222536087f,0.002901836996898055f,-0.07587315142154694f,-0.0007721579167991877f,0.0007741285371594131f,0.021311353892087936f,-0.06366321444511414f,0.0024529958609491587f,0.002941351616755128f,0.058608900755643845f,-0.05726049467921257f,0.057249534875154495f,-0.024241885170340538f,-0.0008736342424526811f,0.007456757128238678f,-0.005484687630087137f,-0.020435022190213203f,0.008126484230160713f,-0.008125538006424904f,-0.011989688500761986f,-0.008589542470872402f,-0.011469445191323757f,0.0010797100840136409f,-0.013430450111627579f,0.006851834710687399f,-0.0068501378409564495f,0.03125318884849548f,0.0008814859320409596f,0.0015885259490460157f,-0.0033497170079499483f,-0.1652948558330536f,-0.042224861681461334f,0.04221833497285843f,-0.014224995858967304f,-0.0004893692093901336f,0.0009921902092173696f,-0.002240479923784733f,-0.029974404722452164f,0.015179493464529514f,-0.01517895795404911f,-0.03598269820213318f,0.004816599190235138f,-0.01285979151725769f,0.0014256590511649847f,0.02104013040661812f,0.01977238617837429f,-0.01977396197617054f,-0.008905279450118542f,0.034374773502349854f,0.023816213011741638f,0.030314434319734573f,0.06291389465332031f,-0.006604218855500221f,0.006597694009542465f,-0.009367339313030243f,0.09592558443546295f,0.04725687578320503f,-0.14107494056224823f,0.13114206492900848f,0.0043356879614293575f,-0.004332307726144791f,-0.03588714450597763f,0.03897765651345253f,0.0013107267441228032f,0.004416599404066801f,0.013624733313918114f,0.0026409060228616f,-0.0026394473388791084f,0.0003549928078427911f,-0.0034344217274338007f,0.00041111710015684366f,-0.008682888932526112f,-0.052068810909986496f,-0.0839531421661377f,0.08395339548587799f,0.09455946832895279f,0.029965663328766823f,0.010068670846521854f,0.002907538553699851f,-0.23689189553260803f,0.0013976763002574444f,-0.0013978370698168874f,0.11238694936037064f,0.010524485260248184f,-0.0036779739893972874f,7.838751480448991e-05f,-0.20135971903800964f,0.0032243230380117893f,-0.003225016640499234f,-0.004737501032650471f,-0.0039965929463505745f,-0.021438293159008026f,-0.009033906273543835f,-0.13012142479419708f,0.0031999745406210423f,-0.003201252082362771f,-0.018648138269782066f,0.02229226380586624f,-0.057303741574287415f,0.011172772385179996f,0.07112564146518707f,0.009107905440032482f,-0.00910782627761364f,0.0034474236890673637f,0.004655377473682165f,-0.0172650795429945f,0.0036636937875300646f,-0.1398419737815857f,-5.3037041652714834e-05f,5.444803900900297e-05f,0.026187967509031296f,0.0033316684421151876f,-0.026522008702158928f,-0.027472207322716713f,-0.09185394644737244f,-0.048932284116744995f,0.04893931373953819f,-0.05267151817679405f,0.05628722533583641f,0.01626763306558132f,0.011370684951543808f,-0.12088143825531006f,0.0010015430161729455f,-0.0010040438501164317f,-0.03413202986121178f,0.02471228875219822f,-0.008444296196103096f,0.004958757199347019f,-0.13968762755393982f,-0.0012788567692041397f,0.0012809643521904945f,-0.006310230121016502f,0.018759608268737793f,0.00760177057236433f,0.007667997851967812f,0.02053963765501976f,0.0611330009996891f,-0.06112787500023842f,-0.01155222300440073f,0.0024398413952440023f,-0.0012305917916819453f,0.0009393140207976103f,-0.11925169825553894f,0.008560293354094028f,-0.008561164140701294f,-0.0370832122862339f,-0.015196871012449265f,0.0028692304622381926f,-0.0011231806129217148f,0.15448355674743652f,0.03328021243214607f,-0.03327547013759613f,-0.005465866066515446f,-0.0036774789914488792f,0.0030818963423371315f,0.003860732540488243f,0.3103538155555725f,-0.18719299137592316f,0.18718856573104858f,0.0029792983550578356f,0.00015829371113795787f,0.007882583886384964f,0.002649974776431918f,-0.07049150764942169f,0.011923887766897678f,-0.011927319690585136f,0.015075118280947208f,0.0047957077622413635f,0.0021888602059334517f,-0.00039688937249593437f,0.010240033268928528f,0.013994812034070492f,-0.014000789262354374f,0.043518487364053726f,0.018355848267674446f,0.00789038185030222f,0.0020682192407548428f,-0.22572307288646698f,-0.0251783300191164f,0.025191232562065125f,-0.0025915272999554873f,-0.002410984132438898f,0.002222682349383831f,-0.0003109796380158514f,0.21745565533638f,0.02587517909705639f,-0.02587563917040825f,-0.0013389445375651121f,-0.02579805813729763f,-0.0016405226197093725f,-0.0005189040675759315f,-0.016122600063681602f,0.019241731613874435f,-0.019245289266109467f,-0.004217646550387144f,-0.040380895137786865f,-0.008533000946044922f,-0.0015052069211378694f,-0.328183114528656f,-0.12186982482671738f,0.12187325209379196f,-0.005746806506067514f,0.03772552311420441f,-0.0077073834836483f,0.007205112837255001f,0.05826877802610397f,-0.0002884711720980704f,0.0002840047236531973f,0.018625278025865555f,0.021306151524186134f,-0.0033434650395065546f,-0.00031697817030362785f,-0.07575465738773346f,-0.0013653795467689633f,0.0013635150389745831f,0.0037083986680954695f,-0.018126655369997025f,-0.0035975275095552206f,0.004893186502158642f,-0.1916937232017517f,-0.006278877146542072f,0.006283851340413094f,-0.008120264858007431f,0.005765893496572971f,0.009736432693898678f,0.006829523015767336f,-0.07234722375869751f,0.04179619252681732f,-0.04179697483778f,0.02982958033680916f,-0.001421483582817018f,-0.0005402157548815012f,0.0029614269733428955f,-0.0382569320499897f,0.051062457263469696f,-0.051060598343610764f,0.03217887878417969f,-0.031135350465774536f,-0.008415195159614086f,0.0201895572245121f,0.0631999671459198f,0.056119613349437714f,-0.05612247809767723f,-0.004015871789306402f,-0.008122837170958519f,0.0012298007495701313f,-0.003511745948344469f,-0.03579146787524223f,0.01971501298248768f,-0.019716951996088028f,-0.02892584353685379f,-0.018276726827025414f,0.000374899449525401f,-0.0014849144499748945f,0.07559563219547272f,0.011399613693356514f,-0.011399781331419945f,0.02369244582951069f,0.024110591039061546f,-0.009760231710970402f,-0.004164537880569696f,-0.19816982746124268f,0.006073654629290104f,-0.006070946808904409f,-0.021277673542499542f,-0.0005372416926547885f,0.0028563777450472116f,0.003812370589002967f,0.023235993459820747f,0.029421383515000343f,-0.0294207576662302f,-0.048212647438049316f,0.002343315165489912f,-7.649663893971592e-05f,-0.000849279691465199f,-0.10823150724172592f,0.024169189855456352f,-0.024167172610759735f,0.0004818312590941787f,0.0010535971960052848f,0.004281035624444485f,0.00025859446031972766f,-0.0349721685051918f,0.027083255350589752f,-0.02708975039422512f,-0.009287577122449875f,0.01601949706673622f,-0.0025681492406874895f,-0.00011844559048768133f,0.08105384558439255f,0.02733362838625908f,-0.027332883328199387f,-0.025374967604875565f,0.030443301424384117f,-0.005239770747721195f,0.002573638455942273f,0.011998379603028297f,0.015106567181646824f,-0.015103863552212715f,0.0004031599673908204f,0.01197834126651287f,-0.002043183194473386f,0.005747513845562935f,0.13296346366405487f,0.055257271975278854f,-0.055255841463804245f,-0.015088086947798729f,0.016712088137865067f,0.0015301709063351154f,-0.003701005131006241f,-0.36827248334884644f,0.006729902233928442f,-0.00673004612326622f,-0.014137639664113522f,-0.0018638784531503916f,0.007106173317879438f,0.0015684269601479173f,-0.07892441749572754f,-0.001685837283730507f,0.0016890907427296042f,-0.0405929870903492f,0.031057192012667656f,-0.0004885790403932333f,0.022935964167118073f,-0.12517158687114716f,0.02880665846168995f,-0.028798503801226616f,-0.041617251932621f,0.02234119176864624f,-0.004568453878164291f,-0.012071612291038036f,-0.18995463848114014f,0.007909934967756271f,-0.007913011126220226f,-0.04241140931844711f,0.03712223470211029f,0.009300588630139828f,0.0007432083948515356f,-0.05027247965335846f,0.003021510783582926f,-0.0030185666400939226f,0.17695562541484833f,-0.039253171533346176f,0.01699824444949627f,0.022033987566828728f,0.05602807551622391f,0.01658572070300579f,-0.01657770946621895f,0.0883367583155632f,-0.04564374312758446f,-0.01565951481461525f,-0.0007314819376915693f,0.09504801034927368f,0.01001137588173151f,-0.010009439662098885f,-0.009611448273062706f,-0.012020763009786606f,0.015830466523766518f,0.0005403027753345668f,0.006817016284912825f,0.008413031697273254f,-0.008409987203776836f,0.04282335937023163f,-0.01742764376103878f,0.0001407219679094851f,0.019752658903598785f,-0.008361645974218845f,-0.06252031028270721f,0.06253042817115784f,0.031078459694981575f,-0.05528987571597099f,0.012366971001029015f,-0.04507744312286377f,0.17820344865322113f,0.00528895715251565f,-0.005290274973958731f,0.04933751001954079f,0.04111776128411293f,0.004238019231706858f,-0.10397698730230331f,-0.0017216004198417068f,0.0029986181762069464f,-0.0030016738455742598f,0.011283686384558678f,0.01889536716043949f,0.008189601823687553f,-0.008286828175187111f,-0.0664266049861908f,-0.02690850757062435f,0.026910604909062386f,0.22051307559013367f,0.009286382235586643f,0.012608179822564125f,0.0018353275954723358f,0.061104048043489456f,0.004384583793580532f,-0.004383453633636236f,0.05053206533193588f,-0.013453163206577301f,-0.022897666320204735f,0.001333313761278987f,0.05954551696777344f,0.008943568915128708f,-0.00894252397119999f,-0.004936607554554939f,0.02836703695356846f,0.012653415091335773f,-0.002783682197332382f,-0.10410366952419281f,-0.0328131839632988f,0.03281418979167938f,0.018690118566155434f,0.006129009649157524f,0.004880080930888653f,0.002336416859179735f,-0.22425256669521332f,0.01093984767794609f,-0.010937293991446495f,0.014110016636550426f,0.020664673298597336f,0.00046065344940871f,0.001064911251887679f,0.20201978087425232f,0.040468476712703705f,-0.04046544060111046f,0.015626152977347374f,0.025238852947950363f,-0.001407823758199811f,-0.010432576760649681f,0.061391547322273254f,-0.006376772187650204f,0.0063747987151145935f,-0.0016451057745143771f,0.026602035388350487f,0.0009781777625903487f,-0.0006596452440135181f,-0.47803786396980286f,0.010488742031157017f,-0.01049097254872322f,0.01297517865896225f,0.02789127081632614f,-0.009002041071653366f,-0.002597089856863022f,-0.06950020790100098f,0.002333696698769927f,-0.002338484628126025f,-0.0638979971408844f,0.01688794046640396f,-0.02441372536122799f,-0.00895534735172987f,-0.21287129819393158f,-0.07561561465263367f,0.0756063386797905f,0.0029016174376010895f,0.019666198641061783f,0.0003566424420569092f,-0.006035205442458391f,0.09293032437562943f,0.031943704932928085f,-0.03194454312324524f,0.01787085458636284f,0.01599913462996483f,0.00012409727787598968f,-0.001773465541191399f,-0.0030517405830323696f,0.016622431576251984f,-0.016625694930553436f,0.0003510277019813657f,0.02597728744149208f,-0.01979692466557026f,0.000608373899012804f,0.08377556502819061f,0.019422290846705437f,-0.019443705677986145f,0.0013875168515369296f,0.0004973658360540867f,-0.010129345580935478f,-0.003832243848592043f,0.1613978147506714f,0.05149586871266365f,-0.05149851366877556f,-0.02699275314807892f,0.02619415894150734f,-0.0007756893173791468f,-0.0005993344238959253f,-0.0967259630560875f,0.02156755141913891f,-0.02156234160065651f,0.01949538104236126f,-0.013150880113244057f,0.010575116612017155f,-0.0011233881814405322f,0.15931813418865204f,0.027649428695440292f,-0.02765086479485035f,-0.004997041542083025f,-0.01467274408787489f,-0.007871896028518677f,-0.0022107516415417194f,0.1461833119392395f,0.039861805737018585f,-0.039860595017671585f,0.004052591510117054f,-0.00140612933319062f,-0.0011742113856598735f,0.0003459028957877308f,-0.02858750708401203f,0.01671144738793373f,-0.016716983169317245f,0.003815109608694911f,0.01329386979341507f,-0.00460271118208766f,0.0063551124185323715f,-0.00524830911308527f,-0.01206271629780531f,0.012055602855980396f,0.015372836962342262f,0.018448270857334137f,-0.007699298672378063f,0.004370172508060932f,0.04337364807724953f,0.0377965122461319f,-0.03779149800539017f,-0.01565515249967575f,-0.0147697189822793f,-0.011781170964241028f,-0.003051087725907564f,-0.005729330703616142f,0.03003755956888199f,-0.030037302523851395f,-0.006673664785921574f,-0.020427197217941284f,0.004344439599663019f,-0.003199639031663537f,0.06385306268930435f,0.007503020577132702f,-0.007501145824790001f,0.007213157135993242f,-0.03656698018312454f,0.011562029831111431f,-0.0006337927188724279f,0.09598919749259949f,0.02774868719279766f,-0.02774723991751671f,0.015475098043680191f,-0.00323579297401011f,0.00739640137180686f,-0.0010930858552455902f,0.01705009862780571f,0.02075403556227684f,-0.020750867202878f,-0.04280601814389229f,0.0023945465218275785f,-0.011726759374141693f,0.0024522740859538317f,-0.037411171942949295f,-0.04214925318956375f,0.04214933142066002f,0.024436842650175095f,-0.005641662981361151f,0.01104583591222763f,-0.0034151438158005476f,-0.17719773948192596f,0.0030215177685022354f,-0.003017114708200097f,0.032654885202646255f,-0.023172374814748764f,0.0028010790701955557f,-0.001639212598092854f,0.19938360154628754f,0.02847382239997387f,-0.02846694551408291f,-0.007245001383125782f,0.020208682864904404f,-0.005476071033626795f,0.0010840673930943012f,-0.0664982721209526f,-0.04074101522564888f,0.04074234515428543f,-0.003552916692569852f,0.010016234591603279f,0.005144134629517794f,0.00034593656891956925f,-0.020199736580252647f,0.006608268246054649f,-0.006611438002437353f,0.021771835163235664f,-0.03928081691265106f,0.011472434736788273f,-0.0017055191565304995f,0.02887587994337082f,0.007284412160515785f,-0.007286003790795803f,-0.01394454576075077f,-0.021198151633143425f,-0.005456520244479179f,0.0007731389487162232f,-0.02679898962378502f,0.006857223343104124f,-0.006854504346847534f,0.0022608505096286535f,-0.00014214009570423514f,-0.0010775290429592133f,-0.0009699856746010482f,0.0034971782006323338f,0.011077786795794964f,-0.011076295748353004f,-0.008489650674164295f,0.009373212233185768f,0.005682945251464844f,0.0012633363949134946f,-0.030848775058984756f,0.008767723105847836f,-0.008763662539422512f,-0.006247609853744507f,0.008929627947509289f,0.005458130035549402f,0.007179741747677326f,-0.07787711918354034f,0.02350681833922863f,-0.023506145924329758f,-0.00011331443238304928f,-0.0008243140764534473f,0.004840249195694923f,0.0027475832030177116f,0.02376558631658554f,0.009357387199997902f,-0.009358211420476437f,0.0050329966470599174f,-0.0013295096578076482f,0.0061759245581924915f,-0.0004071981238666922f,0.1633419245481491f,0.01812109351158142f,-0.018119653686881065f,-0.014504180289804935f,0.017834454774856567f,-0.002772501902654767f,-0.007179658859968185f,-0.028281323611736298f,-0.031991492956876755f,0.03199552372097969f,-0.008594470098614693f,0.018412383273243904f,-0.006870402954518795f,0.002871397417038679f,0.06931480765342712f,0.012533419765532017f,-0.012534846551716328f,0.016865715384483337f,0.01456941943615675f,-0.015806026756763458f,-0.0016518817283213139f,-0.031554434448480606f,0.010791993699967861f,-0.010794531553983688f,0.004175587557256222f,0.012326614931225777f,-0.0017503080889582634f,0.0012142250780016184f,-0.04166680946946144f,0.04054496809840202f,-0.040539778769016266f,-0.006691834889352322f,0.0024783180560916662f,0.004985060542821884f,-0.0003974006976932287f,-0.09389428794384003f,0.005583035293966532f,-0.005577152594923973f,-0.0040616001933813095f,0.010967567563056946f,0.0006980627658776939f,-0.0001544767728773877f,-0.013530980795621872f,0.00996001623570919f,-0.009957859292626381f,0.006439373828470707f,0.02153784967958927f,-0.005179672967642546f,-0.004378494806587696f,0.10121902078390121f,-0.02878042869269848f,0.028771642595529556f,-0.014503736980259418f,-0.005894371308386326f,0.0016519255004823208f,-0.004250355530530214f,0.04934080317616463f,0.012084058485925198f,-0.012087174691259861f,0.007462821435183287f,-0.010629546828567982f,-0.009568201377987862f,-0.0017752567073330283f,-0.011075079441070557f,0.015641886740922928f,-0.015638219192624092f,-0.004643111024051905f,0.00014465200365521014f,-0.011023368686437607f,-0.008714377880096436f,0.0014841525116935372f,-0.007729397155344486f,0.0077294777147471905f,-0.014890876598656178f,-0.012693984434008598f,-0.004685098305344582f,-0.0001923164090840146f,-0.006463330704718828f,0.014289788901805878f,-0.01429464016109705f,-0.007667517755180597f,-0.018490158021450043f,-0.010439782403409481f,0.0017451996682211757f,-0.09104686230421066f,0.00932326354086399f,-0.00932048074901104f,-0.007088794372975826f,0.004822810646146536f,-3.574157744878903e-05f,0.003870744723826647f,0.0016125754918903112f,-0.011474856175482273f,0.011477265506982803f,0.010497627779841423f,-0.005323674995452166f,-0.0056127156130969524f,-0.0027646629605442286f,0.07024402171373367f,0.013614805415272713f,-0.013612614013254642f,-0.04214172437787056f,-0.008858838118612766f,-0.006632023490965366f,0.001035030116327107f,-0.12313172966241837f,0.006608854979276657f,-0.006605155300348997f,0.007595584262162447f,0.011951444670557976f,-0.005299564450979233f,-0.0008723617065697908f,-0.0856693834066391f,-0.06154480203986168f,0.061545081436634064f,0.008364727720618248f,-0.003367012832313776f,-0.0033616709988564253f,6.36568438494578e-05f,0.07086353003978729f,0.009904424659907818f,-0.009903843514621258f,-0.0006799681577831507f,0.003417027648538351f,-0.003431848017498851f,0.0008332736324518919f,-0.09516610205173492f,0.005518095102161169f,-0.005517216399312019f,-0.0030501403380185366f,0.004693460650742054f,0.0006500288145616651f,0.0031458537559956312f,-0.053166404366493225f,-0.03825194761157036f,0.03824983909726143f,0.0003202436491847038f,0.0026722303591668606f,0.005476942285895348f,-0.0010965699329972267f,0.03680403530597687f,0.007432335987687111f,-0.0074287764728069305f,-0.026208287104964256f,-0.015897143632173538f,-0.02845519222319126f,-0.0010835168650373816f,0.034194618463516235f,0.0023451687302440405f,-0.0023441100493073463f,-0.010578055866062641f,-0.008402261883020401f,-0.0004210321349091828f,-0.0032334881834685802f,0.008725762367248535f,-0.1109570786356926f,0.11095905303955078f,-0.01158203650265932f,0.00716801593080163f,0.0056784553453326225f,0.0012050280347466469f,-0.003210684284567833f,0.011071978136897087f,-0.011070109903812408f,-0.016914717853069305f,0.0038018585182726383f,-0.017101917415857315f,-0.0009208124247379601f,-0.016179796308279037f,0.005469437222927809f,-0.005465188063681126f,0.005975857377052307f,0.019402621313929558f,0.0023898989893496037f,0.004439052660018206f,-0.04629954323172569f,-0.013138542883098125f,0.01314310822635889f,0.0003941669419873506f,-0.007531498093158007f,0.008239878341555595f,-0.001520718913525343f,-0.02449919283390045f,0.008669859729707241f,-0.008674245327711105f,0.027049461379647255f,0.0031116234604269266f,0.0007608808227814734f,0.000370112422388047f,0.10174781084060669f,0.004589366260915995f,-0.004587480332702398f,-0.019610412418842316f,-0.012212433852255344f,-0.012751935049891472f,0.005412317346781492f,0.012747150845825672f,0.01725628972053528f,-0.017259666696190834f,0.008693506941199303f,0.003045180579647422f,-0.014318552799522877f,0.003832386340945959f,0.00960437674075365f,0.008177580311894417f,-0.00817318819463253f,-0.016292404383420944f,-0.008754810318350792f,0.012266946025192738f,0.0018000699346885085f,-0.013688866049051285f,0.002497889567166567f,-0.002498175948858261f,-0.0014813215238973498f,-0.02287081815302372f,0.0017311337869614363f,0.001707591931335628f,0.01300355140119791f,0.07773006707429886f,-0.07773030549287796f,0.020106013864278793f,-0.020276546478271484f,0.0004480987845454365f,-0.00509587861597538f,-0.004592325538396835f,0.007754970341920853f,-0.007755267899483442f,-0.009158751927316189f,0.0067335693165659904f,0.006760732736438513f,0.0013667534803971648f,0.03946933150291443f,0.007909499108791351f,-0.007908524014055729f,-0.008950051851570606f,0.009209775365889072f,0.0062127732671797276f,0.00035687972558662295f,-0.1004103496670723f,0.039017923176288605f,-0.03901445493102074f,-0.014075934886932373f,0.004213071893900633f,-0.005885247606784105f,0.00034302237327210605f,-0.04503045976161957f,0.016296107321977615f,-0.016291119158267975f,-0.04303022474050522f,-0.005781932268291712f,-0.013649236410856247f,-0.0007002514321357012f,0.10215297341346741f,0.022295109927654266f,-0.0222943015396595f,-0.0158163383603096f,0.007190991658717394f,-0.01581701450049877f,-0.010373354889452457f,-0.0018926352495327592f,0.006675331387668848f,-0.006673696916550398f,0.04862631484866142f,0.00501202791929245f,0.028550636023283005f,-0.012107313610613346f,0.048367466777563095f,0.001752842334099114f,-0.0017560877604410052f,-0.0396038293838501f,0.02799868956208229f,-0.006627698428928852f,-0.002229373436421156f,-0.054911237210035324f,0.0026595848612487316f,-0.0026600565761327744f,0.0020216703414916992f,0.01756633073091507f,0.0023498383816331625f,-0.003912430722266436f,0.06263674795627594f,-0.024839365854859352f,0.024843759834766388f,-0.004708171356469393f,0.015458764508366585f,-0.0027931153308600187f,0.00920133851468563f,-0.09699399024248123f,0.006945740897208452f,-0.006941814906895161f,-0.003620428964495659f,0.005063560791313648f,-0.018124960362911224f,-0.0004878018226008862f,0.007928194478154182f,0.0031135501340031624f,-0.0031166672706604004f,-0.025010546669363976f,0.001608175109140575f,0.0008424062980338931f,0.009271856397390366f,0.03218130022287369f,0.05913154035806656f,-0.059134695678949356f,-0.013790883123874664f,0.0009447259362787008f,0.006769831292331219f,0.0021881696302443743f,-0.014402633532881737f,0.005630809813737869f,-0.0056338137947022915f,-0.00599692715331912f,-0.010330051183700562f,0.0032818547915667295f,0.0009997530141845345f,-0.006660437677055597f,0.0017316944431513548f,-0.0017361217178404331f,-0.006540937814861536f,0.00573873333632946f,-0.0012721858220174909f,-0.0009284258121624589f,-0.011703908443450928f,-0.0009740229579620063f,0.0009691271698102355f,0.0009274790063500404f,0.010001004673540592f,-0.0037659453228116035f,0.0021481530275195837f,0.03876294195652008f,0.01233508251607418f,-0.012335077859461308f,-0.04968056455254555f,0.015085071325302124f,-0.01420432236045599f,-0.0009373657521791756f,0.012325340881943703f,0.007350232917815447f,-0.007351326290518045f,-0.014623710885643959f,0.010980661027133465f,0.011789590120315552f,0.008700965903699398f,-0.11818356066942215f,-0.032504960894584656f,0.032507918775081635f,0.02267991565167904f,-0.01160742063075304f,0.006396535784006119f,0.0015937651041895151f,0.031763721257448196f,0.015713954344391823f,-0.01571272499859333f,0.08025902509689331f,0.0036471302155405283f,0.0071736546233296394f,0.00046773863141424954f,-0.056031759828329086f,0.02626258134841919f,-0.026262080296874046f,0.017293021082878113f,0.004709156695753336f,0.003266367595642805f,0.0012643050868064165f,0.04527999833226204f,-0.025303402915596962f,0.025308476760983467f,1.6087344192783348e-05f,0.006053742486983538f,-0.0025358872953802347f,0.001152559882029891f,0.03506101295351982f,0.006851814687252045f,-0.006845733616501093f,0.029014278203248978f,0.007816283963620663f,0.011141951195895672f,-0.0007009419496171176f,0.00854428019374609f,0.008755440823733807f,-0.008752623572945595f,-0.007077532354742289f,-0.021386446431279182f,-0.011290035210549831f,-0.0030088569037616253f,0.046585433185100555f,-0.03296196460723877f,0.03295564278960228f,0.014404474757611752f,-0.015506155788898468f,-5.357423651730642e-05f,0.00010378895240137354f,-0.049128443002700806f,0.012229871936142445f,-0.01222870871424675f,0.009159635752439499f,-0.0005861909594386816f,0.006536010652780533f,0.001800781930796802f,-0.053046345710754395f,0.011966516263782978f,-0.01196740660816431f,-0.00552492868155241f,0.004119438119232655f,0.0003099595778621733f,0.0026432860177010298f,-0.004501007962971926f,0.026458563283085823f,-0.0264661256223917f,0.007267815060913563f,0.014160769991576672f,-0.0061097401194274426f,-0.003115610219538212f,-0.022893395274877548f,0.007899321615695953f,-0.007903947494924068f,-0.006484010722488165f,0.01532633788883686f,-0.0025781686417758465f,0.0002215028362115845f,-0.10653675347566605f,0.0006339395768009126f,-0.0006345541914924979f,-0.0261883195489645f,-0.01135908905416727f,-0.0003267482970841229f,-0.00019454373978078365f,-0.04354477673768997f,-0.05141178146004677f,0.051404114812612534f,-0.011998034082353115f,0.007652441971004009f,-0.0022731872741132975f,0.0029366665985435247f,-0.09658438712358475f,0.010648699477314949f,-0.010642032139003277f,-0.0371394082903862f,0.00552520714700222f,0.001964476890861988f,0.0004680190177168697f,0.09675823897123337f,0.02037379890680313f,-0.020368928089737892f,0.006647841539233923f,-0.004951152950525284f,-0.00557143846526742f,0.007721506524831057f,0.009023956023156643f,0.018992964178323746f,-0.01899835281074047f,0.0008647170616313815f,0.0008232153486460447f,-0.01184469647705555f,-0.001719248597510159f,-0.02179761789739132f,0.01044375915080309f,-0.010447132401168346f,-0.005763335153460503f,0.00954492762684822f,0.006744593381881714f,-0.0011625037295743823f,-0.0682263970375061f,0.004888313822448254f,-0.004884204361587763f,0.024554014205932617f,-0.002869043964892626f,0.00477189663797617f,-0.006869651842862368f,-0.009095974266529083f,-0.011937582865357399f,0.01193511113524437f,0.005202210508286953f,0.010628576390445232f,0.006386831868439913f,-0.0029281764291226864f,-0.14109353721141815f,0.007971972227096558f,-0.007968963123857975f,0.07285122573375702f,0.0025290115736424923f,0.014372306875884533f,0.0005163113819435239f,0.10971890389919281f,0.01245256420224905f,-0.012450032867491245f,-0.002265132963657379f,-0.005176855251193047f,-0.00012435138341970742f,-0.003445575712248683f,0.025500215590000153f,-0.03319275379180908f,0.0331912487745285f,-0.005201405845582485f,0.0013150100130587816f,0.0026543934363871813f,-0.003060681978240609f,0.038207028061151505f,0.007947005331516266f,-0.007949277758598328f,0.025730788707733154f,0.009144035167992115f,0.00975766684859991f,0.00034373727976344526f,-0.021604977548122406f,0.006868210155516863f,-0.006866241805255413f,-0.0032781639602035284f,0.003291479777544737f,-0.00454917224124074f,-0.0004213480278849602f,0.008795931935310364f,-0.0041695861145854f,0.004171921405941248f,-0.003142571309581399f,0.00029208234627731144f,-0.0028364621102809906f,-0.0041512479074299335f,0.030349142849445343f,0.014514985494315624f,-0.014518935233354568f,-0.004003031644970179f,-0.014648333191871643f,0.013789338991045952f,-0.0006548691890202463f,-0.05260598286986351f,0.00573485903441906f,-0.005732674151659012f,0.020293571054935455f,-0.005243438761681318f,-0.003645360702648759f,0.004028389696031809f,0.045656781643629074f,0.028644245117902756f,-0.028629764914512634f,0.03556749224662781f,-0.0043221632950007915f,-0.002876915503293276f,-0.0014742615167051554f,0.0018771265167742968f,0.010002263821661472f,-0.010001225396990776f,0.013733946718275547f,-0.000759743619710207f,-0.003455842612311244f,0.0002954484079964459f,0.06252633035182953f,0.005511005874723196f,-0.0055059390142560005f,-0.012400656938552856f,0.01866472326219082f,-0.0004504934186115861f,-0.003239505924284458f,-0.016985459253191948f,0.016941044479608536f,-0.016942201182246208f,-0.020250096917152405f,-0.0025969999842345715f,-0.01128986943513155f,0.002257967833429575f,0.003704447066411376f,0.0021343501284718513f,-0.0021326960995793343f,-0.03134312853217125f,-0.01802559569478035f,0.008523758500814438f,0.0023699188604950905f,-0.09050636738538742f,0.0019477258902043104f,-0.0019478591857478023f,-0.028386490419507027f,0.026223542168736458f,-0.011781999841332436f,0.01636740006506443f,-0.0680759847164154f,-0.007938927970826626f,0.007937992922961712f,0.0009900103323161602f,-0.01721172034740448f,-0.0005311188870109618f,-0.00794487539678812f,0.07910595834255219f,0.005125964991748333f,-0.0051302178762853146f,0.008980496786534786f,0.027986887842416763f,0.04746744781732559f,-0.003959094639867544f,-0.021130340173840523f,0.004200945608317852f,-0.004201284609735012f,-0.01082336250692606f,-0.002298356732353568f,0.005028048530220985f,0.0031415659468621016f,0.0008369194692932069f,-0.003919490147382021f,0.003926662728190422f,-0.022651225328445435f,0.006510427687317133f,0.005713331047445536f,0.0016035839216783643f,-0.015263472683727741f,0.005251470487564802f,-0.005251333583146334f,0.03600742295384407f,-0.0122172050178051f,-0.008886536583304405f,0.0028487476520240307f,-0.007955729961395264f,0.0023410527501255274f,-0.002343616681173444f,0.005490778014063835f,0.00288941478356719f,0.003975330851972103f,-0.003167829243466258f,-0.010288430377840996f,-0.051295921206474304f,0.05129580199718475f,-0.022331224754452705f,-0.0014932201011106372f,-0.0026240297593176365f,-0.004924645647406578f,-0.04622741416096687f,0.007864086888730526f,-0.00786600075662136f,0.014540915377438068f,0.003302511293441057f,0.00028219303931109607f,-0.00017911996110342443f,-0.2587417960166931f,0.007892254739999771f,-0.00789194367825985f,-0.012161570601165295f,-0.011565335094928741f,0.0047593568451702595f,-0.0046145557425916195f,-0.03379802405834198f,-0.04141483083367348f,0.04141329228878021f,-0.0025735716335475445f,-0.008490954525768757f,0.0008070087060332298f,-0.001274404232390225f,0.034838106483221054f,0.013341807760298252f,-0.013338427990674973f,-0.012987323105335236f,0.007370111998170614f,-0.0033840439282357693f,0.0004724515019915998f,-0.11789846420288086f,0.005250094924122095f,-0.005249354988336563f,0.0016826704377308488f,-0.013278469443321228f,0.002117234282195568f,0.0013665303122252226f,0.0018357827793806791f,-0.009791545569896698f,0.00979217141866684f,0.027630098164081573f,-0.01235300861299038f,-0.007623045239597559f,-0.0024715373292565346f,-0.005837058182805777f,0.010072963312268257f,-0.01007245946675539f,-0.04731643199920654f,0.01323272380977869f,-0.01159600354731083f,-0.0014555193483829498f,-0.03423194959759712f,0.006340210326015949f,-0.006339092738926411f,-0.0009117151494137943f,0.014921731315553188f,0.005020240321755409f,0.008660374209284782f,-0.1357005536556244f,-0.012484591454267502f,0.012490534223616123f,0.013715298846364021f,-0.002202981850132346f,-0.0027174095157533884f,-0.0009634215966798365f,0.03346037492156029f,0.014966520480811596f,-0.01496713887900114f,0.021898405626416206f,0.002805305179208517f,-0.002180426614359021f,-0.0007998701767064631f,-0.005652152001857758f,0.013676628470420837f,-0.013675498776137829f,-0.0038268554490059614f,0.002331274561583996f,0.015311897732317448f,0.0011883053230121732f,-0.05713009834289551f,0.008826234377920628f,-0.008813428692519665f,-0.013601010665297508f,0.012798678129911423f,0.0009462921880185604f,0.0012711125891655684f,-0.005952745210379362f,0.009333246387541294f,-0.00933025497943163f,-0.023553477600216866f,-0.023705504834651947f,0.0020005772821605206f,-0.00046999301412142813f,-0.0026820842176675797f,0.010611975565552711f,-0.010614503175020218f,0.001526988111436367f,-0.0028382563032209873f,0.00015309324953705072f,0.0019257906824350357f,-0.028593862429261208f,-0.017834415659308434f,0.017830025404691696f,0.010594110935926437f,0.005090117920190096f,0.0037560993805527687f,0.00295043783262372f,-0.03716599568724632f,0.012731283903121948f,-0.012725422158837318f,-0.04204969108104706f,-0.008013681508600712f,-0.014030994847416878f,-0.0008829570142552257f,-0.017217649146914482f,0.01583063416182995f,-0.01583227701485157f,0.0006687197019346058f,-0.01577199622988701f,0.001558959949761629f,-0.0073541877791285515f,0.09872595965862274f,-0.04515029117465019f,0.045140866190195084f,-0.008418840356171131f,0.01797771453857422f,0.006792609579861164f,0.0022743046283721924f,-0.0777878388762474f,0.007095518056303263f,-0.007096037268638611f,-0.013437432236969471f,-0.0008637502905912697f,0.0012262953678146005f,0.0016959315398707986f,0.05310490354895592f,0.010049847885966301f,-0.010050315409898758f,0.00930004846304655f,-0.010107439011335373f,-0.0005425633862614632f,-0.00344768981449306f,0.07769947499036789f,-0.09338672459125519f,0.09339036047458649f,0.003880054224282503f,-0.007141503971070051f,-0.0058573461137712f,-0.0016514355083927512f,-0.06591541320085526f,0.003485847497358918f,-0.003487895242869854f,0.028393903747200966f,-0.0131045738235116f,0.02203916199505329f,-0.0018888261402025819f,-0.1444387435913086f,0.008048447780311108f,-0.008041624911129475f,0.02303505875170231f,0.014223569072782993f,-0.004649647977203131f,-0.010393757373094559f,0.015184321440756321f,-0.04948705434799194f,0.04948363080620766f,0.01962415874004364f,-0.0009312782785855234f,-0.004703095182776451f,0.003484254702925682f,0.010175302624702454f,0.010004759766161442f,-0.010008689016103745f,-0.031785398721694946f,0.033590540289878845f,0.012039951048791409f,0.00023286027135327458f,0.006165423430502415f,0.011425163596868515f,-0.011426438577473164f,-9.461797162657604e-05f,-0.027497800067067146f,0.006447347346693277f,-0.0008384657558053732f,-0.02580869011580944f,0.004485992714762688f,-0.0044822124764323235f,0.004438611678779125f,-0.011655488982796669f,0.0005486942827701569f,-0.000576732330955565f,0.061784110963344574f,0.009357440285384655f,-0.009357510134577751f,-0.01220118347555399f,-0.01261886302381754f,0.0014777551405131817f,-0.0028601631056517363f,0.05029205232858658f,0.009999004192650318f,-0.009996142238378525f,-0.036371778696775436f,0.010132646188139915f,0.008005380630493164f,0.00832727923989296f,-0.107548788189888f,0.029929984360933304f,-0.02993486076593399f,-0.028134042397141457f,-0.019794434309005737f,0.012251105159521103f,0.0005822906387038529f,0.028514724224805832f,0.006342226639389992f,-0.0063429661095142365f,-0.037001240998506546f,-0.002081342041492462f,-0.0025613915640860796f,-0.0019717570394277573f,-0.04193492978811264f,0.006585579831153154f,-0.006585816387087107f,0.005313599016517401f,-0.007096738088876009f,-0.005691859405487776f,-0.017155099660158157f,-0.018795261159539223f,0.0073286425322294235f,-0.007327875588089228f,-0.015507523901760578f,0.006576945539563894f,-0.0073793912306427956f,0.0012650153366848826f,0.06701964884996414f,0.00965174101293087f,-0.00965148489922285f,0.027558708563447f,-0.0041356682777404785f,-0.008698945865035057f,-0.004000396933406591f,-0.03669233247637749f,0.0015592338750138879f,-0.001557021401822567f,-0.04065116122364998f,0.01846984587609768f,0.0018612073035910726f,-0.0041842982172966f,0.1406005471944809f,-0.02132466249167919f,0.021328238770365715f,0.01788637787103653f,-0.00011032588372472674f,0.005824795458465815f,0.0025897950399667025f,0.03590976446866989f,0.007366157602518797f,-0.007367338053882122f,-0.009161640889942646f,0.016663139685988426f,-0.007386090233922005f,0.0007683206349611282f,0.03781525418162346f,0.00964600220322609f,-0.009647689759731293f,0.0025590229779481888f,0.043576017022132874f,0.008058564737439156f,-0.0015392439672723413f,-0.006727128755301237f,-0.005841610487550497f,0.005842222366482019f,0.0054150731302797794f,0.016605710610747337f,-7.275951065821573e-05f,0.0005898509407415986f,-0.15547706186771393f,0.005278010852634907f,-0.005277642980217934f,-0.05227005481719971f,0.011160166934132576f,0.010414041578769684f,0.0031433787662535906f,-0.08191100507974625f,0.010352189652621746f,-0.010352117009460926f,0.012344064190983772f,-0.019878536462783813f,0.01306694932281971f,-0.0031102411448955536f,-0.24079838395118713f,-0.013591626659035683f,0.01359908189624548f,-0.006432654801756144f,-0.015559829771518707f,0.0018320983508601785f,-0.001000179909169674f,-0.01563393697142601f,0.004520871676504612f,-0.004522126633673906f,-0.01599191315472126f,-0.028206786140799522f,-0.006757160183042288f,-0.003047526115551591f,-0.01197807490825653f,0.006598158739507198f,-0.006600495427846909f,-0.018078474327921867f,-0.01132524386048317f,-0.002169773681089282f,-0.006691788788884878f,-0.06637071073055267f,0.037452999502420425f,-0.03745383024215698f,0.01494255568832159f,-0.0005117932450957596f,-0.0032586578745394945f,0.0018573624547570944f,-0.026005009189248085f,0.012486116960644722f,-0.012487786822021008f,-0.01896223984658718f,-0.003379116067662835f,-0.020331308245658875f,-0.00141149980481714f,-0.08929641544818878f,0.0072985147126019f,-0.007300925441086292f,-0.022598756477236748f,0.002727941144257784f,-0.006632535718381405f,-0.004752086941152811f,-0.04720675200223923f,0.0021240978967398405f,-0.0021241591311991215f,0.006403577048331499f,0.007183346897363663f,-0.005665261764079332f,0.001962631242349744f,0.0049849627539515495f,0.012253932654857635f,-0.012251796200871468f,0.016580013558268547f,-0.001764544053003192f,-0.0017109981272369623f,0.0010405160719528794f,-0.027738679200410843f,0.004273564089089632f,-0.004270809702575207f,0.017378633841872215f,0.010563232935965061f,-0.004290441051125526f,-0.000133466484840028f,0.06348446011543274f,0.0230855830013752f,-0.023079590871930122f,0.01566523313522339f,0.010411535389721394f,-0.0039807939901947975f,0.0028552385047078133f,0.0018428991315886378f,0.004538103472441435f,-0.004541364032775164f,0.00934532005339861f,-0.004611401818692684f,-0.002255306812003255f,0.0002394706680206582f,0.06225091591477394f,0.0031347167678177357f,-0.0031286291778087616f,0.050533853471279144f,-0.036237988620996475f,0.0015673610614612699f,-0.003596473252400756f,-0.07424070686101913f,-0.04041200503706932f,0.04041551798582077f,0.03601673245429993f,-0.0031884636264294386f,-0.0035325882490724325f,0.006279143039137125f,-0.1509004831314087f,0.008150985464453697f,-0.0081471623852849f,0.012904631905257702f,-0.0031159729696810246f,0.01081254705786705f,-0.0007721014553681016f,-0.09365645051002502f,0.005565543193370104f,-0.005570539273321629f,0.005196485668420792f,-0.011773752048611641f,3.0114746550680138e-05f,-0.004253948573023081f,-0.027242392301559448f,0.020604504272341728f,-0.020606666803359985f,-0.003235022770240903f,0.007486687507480383f,0.01118912361562252f,-0.0013130644802004099f,0.03326079249382019f,0.008442142978310585f,-0.008443236351013184f,0.0048341695219278336f,0.0009038591524586082f,-0.0039279880002141f,0.0007498383056372404f,-0.026646951213479042f,0.005435314495116472f,-0.005435215309262276f,-0.037124354392290115f,-0.0018206355161964893f,0.010599980130791664f,0.008739481680095196f,0.026552753522992134f,0.04267513379454613f,-0.042672667652368546f,-0.02316022664308548f,-0.003071801271289587f,0.009098305366933346f,0.0022193235345184803f,0.04593946412205696f,0.017469316720962524f,-0.017467783764004707f,-0.02100689895451069f,0.0066169449128210545f,0.010920116677880287f,-0.0011415673652663827f,-0.04826262965798378f,0.014508351683616638f,-0.014506691135466099f,-0.032263148576021194f,0.024413777515292168f,-0.0001588927407283336f,-0.0032098754309117794f,-0.20504087209701538f,-0.011404517106711864f,0.011413896456360817f,-0.0023133731447160244f,0.01594720222055912f,0.0011749421246349812f,0.0004870814736932516f,-0.030404293909668922f,0.005615122150629759f,-0.005618699826300144f,-0.0004817073931917548f,-0.017903262749314308f,0.0027324564289301634f,-0.0023613295052200556f,0.04435032606124878f,0.0024366152938455343f,-0.0024328522849828005f,0.03383585065603256f,0.013615516014397144f,-0.00506181875243783f,-0.0023695374839007854f,-0.059828855097293854f,-0.0536329448223114f,0.053639043122529984f,0.03150428831577301f,-0.00437846640124917f,-0.0005422595422714949f,-0.004118296783417463f,-0.08024075627326965f,0.00628188531845808f,-0.006280036643147469f,0.01348490547388792f,0.0013315014075487852f,-7.439941691700369e-05f,-0.00015602644998580217f,0.09647815674543381f,0.008683021180331707f,-0.008680092170834541f,0.012299233116209507f,-0.016508834436535835f,0.0021336108911782503f,-0.01746341399848461f,-0.01729525625705719f,-0.03161688894033432f,0.031617265194654465f,-0.01894073560833931f,0.024994030594825745f,0.006349252071231604f,0.0184656772762537f,-0.061435770243406296f,0.009042497724294662f,-0.00904103647917509f,-0.0037261033430695534f,0.015649501234292984f,0.0022640179377049208f,0.008105517365038395f,-0.046313606202602386f,0.01127602532505989f,-0.011274389922618866f,0.03169504553079605f,-0.013921737670898438f,-0.014340569265186787f,-0.007979425601661205f,0.03346771001815796f,-0.06135336309671402f,0.06135639548301697f,0.005600748583674431f,-0.0008391043520532548f,0.0028895726427435875f,-0.0013109479332342744f,-0.04619989171624184f,0.009179546497762203f,-0.00917528010904789f,0.019877852872014046f,0.019535526633262634f,0.004066156689077616f,-0.0009484615875408053f,-0.05729999020695686f,0.010295779444277287f,-0.010290822014212608f,-0.04638458043336868f,-0.02115478366613388f,0.00752791715785861f,-0.007894471287727356f,0.016398146748542786f,-0.040416888892650604f,0.0404101200401783f,-0.023028839379549026f,-0.016750788316130638f,0.0087986895814538f,0.00023152593348640949f,0.04648463800549507f,0.009008374996483326f,-0.009010715410113335f,-0.006325446534901857f,-0.020530326291918755f,0.0070196171291172504f,-0.0026884088292717934f,0.03817334398627281f,0.007651961408555508f,-0.007657587993890047f,0.025944750756025314f,0.02081483043730259f,0.009132764302194118f,-0.026754412800073624f,-0.06467925012111664f,-0.0127246780321002f,0.012729337438941002f,0.050836145877838135f,0.015223701484501362f,-0.004500017501413822f,0.003194072050973773f,-0.17835837602615356f,0.003962032962590456f,-0.00396527536213398f,0.049230724573135376f,-0.06565121561288834f,0.023855289444327354f,0.0022114485036581755f,-0.1322421133518219f,0.002096450887620449f,-0.002095790347084403f,0.017796283587813377f,-0.01553469616919756f,0.006003368180245161f,-0.004587350878864527f,-0.08089196681976318f,-0.003481627209112048f,0.0034795617684721947f,0.019900120794773102f,-0.028943026438355446f,-0.010373544879257679f,-4.932662704959512e-05f,-0.13739852607250214f,0.007161029614508152f,-0.007162727881222963f,0.012279966846108437f,-0.019426988437771797f,0.012569058686494827f,-0.0019091664580628276f,-0.17695491015911102f,0.007406886667013168f,-0.0074051665142178535f,-0.009014245122671127f,-0.015373745001852512f,0.001895537949167192f,-0.0033430419862270355f,-0.051815927028656006f,0.02036827802658081f,-0.020374193787574768f,0.008564845658838749f,-0.007260139100253582f,0.00360087095759809f,0.0012225600657984614f,-0.041755981743335724f,0.009067847393453121f,-0.009070026688277721f,-0.012684082612395287f,0.014924420043826103f,0.002121533965691924f,0.0009368167375214398f,-0.0582892931997776f,0.00810173712670803f,-0.008105059154331684f,0.02034134417772293f,-0.0383746400475502f,0.006958598271012306f,0.004500099457800388f,0.039218757301568985f,0.00678412988781929f,-0.006778257433325052f,-0.005250212270766497f,-0.02344728820025921f,0.014511523768305779f,-0.003296216018497944f,0.030191458761692047f,0.010564285330474377f,-0.010561826638877392f,-0.0024444626178592443f,-0.011484052054584026f,0.015070985071361065f,-0.001161521184258163f,-0.007916869595646858f,0.011979795061051846f,-0.0119743961840868f,0.04934108257293701f,0.004125535953789949f,-0.0038134471978992224f,0.0073846266604959965f,0.047339338809251785f,0.07356445491313934f,-0.0735686719417572f,0.018709179013967514f,-0.021368838846683502f,-0.0011421783128753304f,0.001409851131029427f,-0.0532071553170681f,0.007889880798757076f,-0.007889918051660061f,0.00028746016323566437f,-0.019770655781030655f,0.00841840635985136f,-3.003776873811148e-05f,-0.08876999467611313f,0.004702083766460419f,-0.004704548977315426f,-0.019252516329288483f,0.005108767654746771f,0.0028613826725631952f,-0.003822403261438012f,0.0006780257681384683f,0.010116036981344223f,-0.01011995691806078f,0.007283350452780724f,-0.03423372656106949f,-0.007670444902032614f,-0.0036586716305464506f,-0.031593646854162216f,0.0021738933864980936f,-0.0021754216868430376f,-0.030564457178115845f,0.026403430849313736f,1.8974533304572105e-06f,-3.2516349165234715e-05f,-0.15301579236984253f,0.007197993341833353f,-0.007196191698312759f,-0.021598199382424355f,0.011925378814339638f,-0.009033509530127048f,0.0009103550110012293f,0.011790843680500984f,-0.020459750667214394f,0.02046596258878708f,-0.0025860685855150223f,0.02910695970058441f,-0.009019693359732628f,0.003843765240162611f,-0.00034483845229260623f,0.00631665950641036f,-0.00631721131503582f,0.02040277235209942f,0.023006359115242958f,0.003449605079367757f,0.0020143461879342794f,-0.06557802855968475f,0.006100183818489313f,-0.006097646430134773f,0.02170676738023758f,0.018513670191168785f,0.0016175609780475497f,0.006619666703045368f,-0.06790485978126526f,0.029714902862906456f,-0.029717886820435524f,-0.027751760557293892f,0.026952289044857025f,-0.0028490768745541573f,0.002338258782401681f,0.028053859248757362f,0.007296192459762096f,-0.007299687713384628f,0.020765256136655807f,0.021849028766155243f,-0.0027303206734359264f,0.003023365279659629f,0.001742654130794108f,0.003864331403747201f,-0.0038618692196905613f,-0.025823043659329414f,0.019007772207260132f,0.00820708554238081f,0.006212293636053801f,0.014762182720005512f,0.014984113164246082f,-0.014983871951699257f,-0.01297924853861332f,-0.01975315436720848f,-0.005744664929807186f,-0.0026773938443511724f,0.043763305991888046f,0.004918819759041071f,-0.00492546521127224f,-0.008960749953985214f,0.014978333376348019f,0.00909330788999796f,-0.00024169849348254502f,0.056436531245708466f,0.0066461628302931786f,-0.006644743960350752f,-0.006003039889037609f,0.009121221490204334f,0.00037966243689879775f,-0.00492265447974205f,0.020885756239295006f,-0.007943082600831985f,0.007937916554510593f,0.017709316685795784f,-0.008606557734310627f,-0.00021291467419359833f,-0.0011788626434281468f,-0.021056320518255234f,0.007791091687977314f,-0.007793720345944166f,0.015908798202872276f,0.01221269741654396f,-0.00255577708594501f,-0.0007981128292158246f,-0.012809938751161098f,0.004285306204110384f,-0.0042848591692745686f,0.04857000336050987f,0.007279707118868828f,-0.01581529900431633f,0.004595702048391104f,-0.20342740416526794f,0.04334880784153938f,-0.04334889352321625f,0.021570399403572083f,0.007355356123298407f,-0.010043930262327194f,-0.0011738731991499662f,-0.14875422418117523f,0.0035407301038503647f,-0.003540095640346408f,-0.011389832012355328f,0.0016389871016144753f,-0.002944840816780925f,0.004830166697502136f,-0.07265742868185043f,0.00311497924849391f,-0.0031137154437601566f,-0.04609519988298416f,0.05238477140665054f,0.010655158199369907f,0.010417245328426361f,0.006594292353838682f,-0.0010579549707472324f,0.0010623132111504674f,0.018522968515753746f,-0.014470305293798447f,-0.0020856959745287895f,-0.006805041804909706f,0.026677582412958145f,0.006366179324686527f,-0.006371316500008106f,0.005046887788921595f,0.00882598478347063f,-0.0019025885267183185f,-0.0016551722073927522f,-0.01924612931907177f,0.0038674038369208574f,-0.003866779850795865f,0.007131973281502724f,-0.03108139894902706f,-0.015303839929401875f,-0.013100276701152325f,-0.018197836354374886f,-0.013510088436305523f,0.013515785336494446f,0.011872569099068642f,0.026689382269978523f,-0.022860176861286163f,0.003007245948538184f,-0.05170575529336929f,0.0033341951202601194f,-0.00333901378326118f,0.0036255940794944763f,-0.025046536698937416f,0.012099586427211761f,0.0021911095827817917f,-0.00420351279899478f,0.0060721528716385365f,-0.006075312849134207f,-0.023117776960134506f,0.009024498984217644f,0.009939349256455898f,0.0020781680941581726f,0.027447639033198357f,0.009802483953535557f,-0.00980007741600275f,-0.015312423929572105f,0.008529085665941238f,0.0017740186303853989f,0.0010988271096721292f,0.05115054175257683f,0.013350834138691425f,-0.013351261615753174f,-0.007273939438164234f,0.03463806211948395f,0.004829080309718847f,0.0003843381127808243f,0.09781379252672195f,0.013861509039998055f,-0.013859625905752182f,-0.014363191090524197f,0.015307282097637653f,0.014136670157313347f,0.0072024608962237835f,-0.09435290098190308f,-0.04901551827788353f,0.04902045428752899f,0.018868064507842064f,-0.022078435868024826f,-0.0010483494261279702f,-0.0001362233015242964f,-0.0831555724143982f,0.0040597994811832905f,-0.004056737292557955f,0.021164411678910255f,-0.0044287433847785f,-0.0035321086179465055f,0.0003523480554576963f,-0.09919080883264542f,0.003019012976437807f,-0.00302007794380188f,0.004107524175196886f,-0.015977391973137856f,0.004129375331103802f,-0.006890507880598307f,-0.03489087522029877f,-0.011034173890948296f,0.011033141985535622f,-0.005277520976960659f,0.0044046263210475445f,0.0038085852283984423f,-0.0006477723945863545f,-0.04234903305768967f,0.017890149727463722f,-0.017890704795718193f,-0.00946150440722704f,0.002156224101781845f,-0.0037489652168005705f,-0.0008124934392981231f,-0.029586801305413246f,0.014232691377401352f,-0.014232141897082329f,0.03166372701525688f,0.00041790655814111233f,0.0008105073939077556f,-0.000934082898311317f,0.06321646273136139f,0.015245039947330952f,-0.015250743366777897f,0.0030876840464770794f,0.015957960858941078f,0.0077878995798528194f,0.0011143404990434647f,-0.047219254076480865f,0.010445697233080864f,-0.01044232677668333f,0.012842127121984959f,-0.01762860082089901f,-0.00046018348075449467f,-0.0006918482831679285f,-0.021030405536293983f,0.004069505725055933f,-0.004068454727530479f,0.030127333477139473f,-0.011854084208607674f,0.008159910328686237f,0.007760487496852875f,-0.057526204735040665f,0.019328363239765167f,-0.01933342032134533f,-0.0007737306877970695f,0.023871945217251778f,0.0038503140676766634f,0.0015452614752575755f,0.011177489534020424f,0.005607608240097761f,-0.005611316300928593f,-0.043560244143009186f,-0.015595898032188416f,0.010571911931037903f,0.00015780034300405532f,0.06127258390188217f,0.005067253485321999f,-0.005073335487395525f,0.00011116089444840327f,-0.010252062231302261f,0.01009556744247675f,-0.004046787973493338f,-0.019391633570194244f,-0.012858569622039795f,0.012857118621468544f,-0.003477485151961446f,0.007327225524932146f,0.0009693533647805452f,-0.003499847138300538f,-0.03067677840590477f,0.005180343985557556f,-0.005175112280994654f,0.06725692003965378f,-0.010481832548975945f,-0.0061350553296506405f,0.0009792062919586897f,0.021745067089796066f,0.004968194290995598f,-0.004965205676853657f,0.006468815263360739f,-0.0033954624086618423f,-0.0028645654674619436f,-0.005328054539859295f,-0.050359636545181274f,-0.018123848363757133f,0.018119052052497864f,-0.004747920203953981f,0.014612751081585884f,0.005454694852232933f,-0.00016967975534498692f,-0.08800005167722702f,0.018343396484851837f,-0.018346671015024185f,0.009233559481799603f,-0.00391264958307147f,-0.004617831204086542f,0.0016704234294593334f,-0.014767931774258614f,0.01673847623169422f,-0.016741015017032623f,0.008596688508987427f,-0.0005153496167622507f,-0.0019471795530989766f,0.002043131971731782f,-0.11338844895362854f,-0.04882238805294037f,0.04882245883345604f,-0.0009437644039280713f,-0.026733126491308212f,-0.002887630369514227f,-0.0024467871990054846f,0.03357769548892975f,0.0062345596961677074f,-0.006238092668354511f,-0.03799815848469734f,-0.003827602369710803f,-0.005182880442589521f,-0.00036950502544641495f,-0.0835258737206459f,0.004057570360600948f,-0.004053684417158365f,0.04848930984735489f,-0.03412039950489998f,-0.0006994558498263359f,-0.0019131492590531707f,0.04659327492117882f,-0.03448476642370224f,0.03449048846960068f,-0.043519534170627594f,0.044531162828207016f,0.009293421171605587f,0.0017597635742276907f,-0.11396042257547379f,0.0038222691509872675f,-0.0038251637015491724f,-0.051739733666181564f,0.02140285260975361f,0.0033135267440229654f,0.0015803739661350846f,-0.008982148952782154f,0.012094898149371147f,-0.012092525139451027f,-0.024397999048233032f,0.010468456894159317f,-0.00402859365567565f,0.02600751258432865f,-0.15793465077877045f,0.014841191470623016f,-0.014835665002465248f,0.0036359222140163183f,0.03379284217953682f,-0.0017922144616022706f,0.007382647134363651f,0.1882147490978241f,0.03206966444849968f,-0.03207074850797653f,0.011517173610627651f,0.05540021136403084f,-0.008701137267053127f,0.00485643744468689f,0.18165835738182068f,0.021472692489624023f,-0.021476617082953453f,-0.048213716596364975f,-0.28195834159851074f,-0.0018467925256118178f,-0.010918418876826763f,0.013161467388272285f,0.03499005734920502f,-0.03499135747551918f,-0.04135330393910408f,-0.22189295291900635f,0.002255452563986182f,-0.004469136707484722f,-0.06498397886753082f,0.018498646095395088f,-0.018495311960577965f,0.025351738557219505f,-0.1759224534034729f,-0.015906313434243202f,-0.0021188301034271717f,-0.046591129153966904f,0.014419188722968102f,-0.014416314661502838f,0.0037697600200772285f,-0.032404910773038864f,0.0028574131429195404f,-0.00570643600076437f,-0.023326609283685684f,0.003072651568800211f,-0.00307832146063447f,0.013215528801083565f,-0.012584696523845196f,0.010407662019133568f,-0.001735515659675002f,0.0678166076540947f,0.03385091945528984f,-0.03385160490870476f,0.02407030388712883f,0.0011481360998004675f,-0.007192839402705431f,0.0013710178900510073f,-0.01455004047602415f,0.024627024307847023f,-0.02462357096374035f,-0.011043062433600426f,0.03583550453186035f,-0.02880321629345417f,-0.04732805863022804f,0.10248918831348419f,-0.021216711029410362f,0.02120247669517994f,0.002709901425987482f,0.003434985177591443f,-0.008916336111724377f,0.004717887379229069f,-0.09333096444606781f,0.015320789068937302f,-0.015317137353122234f,0.0057833073660731316f,0.07762665301561356f,0.004013176541775465f,-0.0028299710247665644f,-0.09857083112001419f,0.002633584663271904f,-0.0026318393647670746f,0.10435715317726135f,0.010420014150440693f,0.003441500710323453f,-0.005635726731270552f,0.023418281227350235f,-0.048528678715229034f,0.04852947220206261f,0.011754320934414864f,-0.005468152929097414f,0.006441403646022081f,-0.0008334803860634565f,-0.02324342168867588f,0.019822483882308006f,-0.019822362810373306f,0.024280810728669167f,-0.021718772128224373f,0.001731855678372085f,-0.0011089574545621872f,0.033718083053827286f,0.020133469253778458f,-0.02013147808611393f,-0.011632627807557583f,0.016028065234422684f,0.007059232331812382f,0.0031626794952899218f,-0.009563999250531197f,0.009691457264125347f,-0.009681673720479012f,0.010043453425168991f,-0.0034535543527454138f,-0.0006597040919587016f,0.0014264455530792475f,-0.17087793350219727f,0.02436947077512741f,-0.02436690963804722f,0.014626010321080685f,0.0019343396415933967f,-0.02153707668185234f,0.0037316724192351103f,0.12140344083309174f,0.0359867624938488f,-0.035982925444841385f,-0.012076887302100658f,0.016814900562167168f,0.005343948025256395f,0.002639003098011017f,-0.03642939776182175f,0.0017467447323724627f,-0.0017418343340978026f,-0.014538493007421494f,0.026377782225608826f,0.010563632473349571f,0.007825570181012154f,-0.03780345246195793f,0.020951954647898674f,-0.020951779559254646f,-0.02590901404619217f,0.0023086818400770426f,0.00031125007080845535f,0.0005418997607193887f,-0.014075440354645252f,0.02224479243159294f,-0.022249115630984306f,-0.22914542257785797f,-0.005433052312582731f,-0.0051034679636359215f,0.005074010230600834f,-0.015592875890433788f,0.02932579256594181f,-0.029329590499401093f,-0.11548704653978348f,0.03900814428925514f,-0.006842729169875383f,0.004049717448651791f,-0.11419479548931122f,0.012158874422311783f,-0.01215530838817358f,-0.03151378780603409f,-0.0216540340334177f,0.009031405672430992f,-0.0009696485358290374f,-0.09204703569412231f,0.002378532662987709f,-0.0023761214688420296f,-0.004675924312323332f,-0.002679749159142375f,-0.013827181421220303f,0.002348667709156871f,-0.0711384192109108f,0.03300120308995247f,-0.03300664201378822f,-0.019432784989476204f,-0.04504459351301193f,-0.008642397820949554f,-0.0070716384798288345f,-0.14992699027061462f,0.016444271430373192f,-0.016443412750959396f,-0.02391134947538376f,0.017584288492798805f,0.01742955856025219f,0.000282853317912668f,-0.07663120329380035f,0.0016350498190149665f,-0.0016314988024532795f,-0.008693934418261051f,0.013882068917155266f,0.020572295412421227f,0.004250798374414444f,-0.002699078293517232f,0.15480752289295197f,-0.15481624007225037f,-0.02426460199058056f,-0.012395518831908703f,0.00637383246794343f,0.0015050264773890376f,-0.039463166147470474f,0.007004097104072571f,-0.007002153433859348f,-0.006568391807377338f,-0.007578032091259956f,-0.004901183769106865f,-0.0002937036915682256f,0.007612472400069237f,0.0015746511053293943f,-0.0015776475192978978f,0.011221690103411674f,-0.06117350235581398f,-0.06361536681652069f,-0.035563480108976364f,-0.16457393765449524f,-0.010811179876327515f,0.010806233622133732f,0.02016720548272133f,-0.03406805172562599f,-0.06307664513587952f,0.020618146285414696f,-0.1794775426387787f,0.014672123827040195f,-0.014667067676782608f,-0.014718053862452507f,0.01999179646372795f,-0.011264835484325886f,0.012001268565654755f,-0.18939267098903656f,0.0010852479608729482f,-0.0010842665797099471f,0.02425127476453781f,0.007685557939112186f,-0.0016240052646026015f,0.004819544497877359f,0.18959614634513855f,0.1673002541065216f,-0.1673135608434677f,-0.006204306613653898f,0.006956123746931553f,0.0004399381286930293f,-0.002714830683544278f,-0.13331487774848938f,0.01500026136636734f,-0.014996564947068691f,0.004371473100036383f,-0.016441721469163895f,0.003965212497860193f,0.0014647088246420026f,-0.048179641366004944f,0.0036281831562519073f,-0.003630904946476221f,0.09722214192152023f,-0.05856041982769966f,0.012860904447734356f,-0.0038694438990205526f,-0.3566620349884033f,0.04053585231304169f,-0.040531668812036514f,0.05202167481184006f,-0.024231908842921257f,0.017980137839913368f,0.006219661794602871f,-0.2921227216720581f,0.008015882223844528f,-0.008014443330466747f,0.034852221608161926f,-0.017109746113419533f,0.009388831444084644f,0.002593729645013809f,-0.1555364429950714f,0.006291590165346861f,-0.006285392679274082f,0.010334338061511517f,0.01406133733689785f,-0.004117823205888271f,-0.006231534294784069f,0.3201843798160553f,-0.04480324685573578f,0.04481089487671852f,-0.019440053030848503f,1.469526432629209e-05f,0.0037581482902169228f,-0.002408799948170781f,0.0465652197599411f,0.005779800470918417f,-0.00577901117503643f,0.0009115997818298638f,-0.005223636515438557f,0.003056621877476573f,0.002368141897022724f,-0.051658306270837784f,0.001509565976448357f,-0.0015090665547177196f,-0.034870922565460205f,0.01590918004512787f,-0.030687134712934494f,0.00410636980086565f,-0.1406189352273941f,0.007958028465509415f,-0.007956366054713726f,-0.0282780509442091f,0.026665281504392624f,0.017714040353894234f,-0.002466215519234538f,-0.15453186631202698f,0.008203989826142788f,-0.008203224278986454f,-0.01306643895804882f,0.0014526250306516886f,-0.003958097659051418f,0.0018339711241424084f,-0.03810381889343262f,0.00837411917746067f,-0.008373801596462727f,0.01262002345174551f,0.05837330222129822f,0.02050163596868515f,-0.04120059683918953f,-0.10874918848276138f,-0.010199449025094509f,0.010199910029768944f,-0.008640667423605919f,0.0008399091893807054f,-0.006188804283738136f,0.09816746413707733f,-0.05395011231303215f,0.023368898779153824f,-0.023372311145067215f,-0.00903422012925148f,-0.025429967790842056f,-0.046096332371234894f,0.016945600509643555f,-0.07703209668397903f,0.018099483102560043f,-0.018103213980793953f,-0.0015858577098697424f,0.024007905274629593f,0.007133755832910538f,0.007778489962220192f,-0.13791128993034363f,-0.048971038311719894f,0.04896550253033638f,-0.0057160984724760056f,-0.04218684136867523f,0.0021789935417473316f,-0.0022803647443652153f,-0.12127016484737396f,0.0041805775836110115f,-0.0041793715208768845f,0.05786160007119179f,0.0073287030681967735f,-0.003353041596710682f,-0.0004051810537930578f,-0.10025714337825775f,0.0029447218403220177f,-0.002940527396276593f,0.01026071235537529f,-0.018562834709882736f,-0.011655021458864212f,-0.02038533240556717f,0.05648834630846977f,-0.025851888582110405f,0.025865942239761353f,0.0050662364810705185f,0.008027237839996815f,-0.0059115649200975895f,-0.0007536879857070744f,-0.14464537799358368f,0.017567923292517662f,-0.017569346353411674f,0.017743710428476334f,-0.0022824809420853853f,0.015069574117660522f,-0.0024846759624779224f,-0.14283432066440582f,0.004810073878616095f,-0.004807599820196629f,-0.07838071882724762f,-0.0572022907435894f,0.016517095267772675f,-0.06715039908885956f,-0.26370489597320557f,0.013285639695823193f,-0.013288795948028564f,-0.05852213501930237f,-0.06497915834188461f,-0.014896363019943237f,-0.010307691060006618f,-0.17400263249874115f,0.00787461269646883f,-0.007876669988036156f,-0.014016421511769295f,-0.07569701224565506f,0.016515007242560387f,-0.005673613399267197f,-0.18696703016757965f,0.003533362876623869f,-0.0035300166346132755f,-0.04274153336882591f,-0.017345590516924858f,0.009509056806564331f,-0.0028936052694916725f,0.051753658801317215f,-0.007328118663281202f,0.00732573913410306f,-0.018277931958436966f,0.01965581811964512f,-0.0031415841076523066f,0.002603639615699649f,-0.022410286590456963f,0.012021233327686787f,-0.01202225312590599f,-0.006825285963714123f,0.013492181897163391f,0.002664912724867463f,0.001933760242536664f,-0.05511540547013283f,0.0185558944940567f,-0.018554475158452988f,-0.003331406507641077f,0.06315019726753235f,0.005084583070129156f,-0.014829889871180058f,-0.07933960109949112f,-0.05098329484462738f,0.05098660662770271f,-0.006023440044373274f,0.005971432663500309f,0.005122645292431116f,-0.0005934518412686884f,0.07349893450737f,0.014084583148360252f,-0.014084208756685257f,-0.0022119286004453897f,-0.01696990616619587f,0.002448196057230234f,-0.002132533583790064f,0.023147763684391975f,0.011489156633615494f,-0.011487939395010471f,-0.005097899120301008f,-0.16025742888450623f,-0.000932061520870775f,0.0035134865902364254f,-0.022095095366239548f,0.049360986799001694f,-0.049352895468473434f,-0.019901365041732788f,-0.016231894493103027f,0.009660700336098671f,-0.0061579421162605286f,-0.18350760638713837f,0.011179370805621147f,-0.011179454624652863f,-0.008039438165724277f,-0.007096313405781984f,0.011161508038640022f,-0.0017092551570385695f,-0.0013262673746794462f,0.015624292194843292f,-0.015624332241714f,-0.006013231351971626f,-0.011283841915428638f,-0.00255330977961421f,0.0048917983658611774f,0.1905241310596466f,-0.004318444058299065f,0.0043167416006326675f,0.01103712897747755f,-0.011311665177345276f,-0.0016665715957060456f,0.003590129781514406f,-0.26257774233818054f,0.0025475455913692713f,-0.0025497060269117355f,0.008644144050776958f,-0.032631222158670425f,0.006349462550133467f,-0.0015706856502220035f,-0.08615034818649292f,0.0007147533469833434f,-0.0007163439295254648f,-0.018459444865584373f,-0.054461073130369186f,-0.014558084309101105f,-0.03232605755329132f,-0.061757251620292664f,0.0034020759630948305f,-0.003401531372219324f,-0.019101912155747414f,-0.000654562609270215f,-0.00850712787359953f,0.007837451994419098f,-0.2131272852420807f,0.01753196492791176f,-0.017529679462313652f,-0.0203059334307909f,0.017913684248924255f,-0.008761480450630188f,0.0002694317954592407f,-0.010370606556534767f,0.028421536087989807f,-0.028419148176908493f,-0.012888058088719845f,0.007135291583836079f,-0.023455128073692322f,-0.0027130001690238714f,0.18110200762748718f,0.002166896127164364f,-0.0021839323453605175f,0.007482772693037987f,-0.01010288204997778f,-0.01597575843334198f,-0.0009227604605257511f,-0.00763458339497447f,0.03195871040225029f,-0.03195305913686752f,0.0009934594854712486f,0.026640206575393677f,-0.000973183021415025f,0.0022294195368885994f,-0.06324413418769836f,0.016256406903266907f,-0.016253622248768806f,0.05366199091076851f,0.046422768384218216f,-0.01385511551052332f,-0.0062337713316082954f,-0.15265563130378723f,-0.00019764604803640395f,0.00020044736447744071f,0.029660863801836967f,0.0445309579372406f,-0.0049171545542776585f,0.009734127670526505f,-0.11268115043640137f,0.016830405220389366f,-0.0168340764939785f,0.00637718616053462f,0.02370908297598362f,0.0011247321963310242f,0.0008207884384319186f,-0.050633858889341354f,0.01251224149018526f,-0.01251174695789814f,0.010836037807166576f,0.014279743656516075f,0.001497189630754292f,-0.005626928526908159f,0.17756225168704987f,0.18274177610874176f,-0.1827390044927597f,0.011131998151540756f,0.04190549999475479f,-0.0028604716062545776f,-0.0024100053124129772f,-0.008521799929440022f,0.011584758758544922f,-0.01159180048853159f,0.011797125451266766f,0.004975233692675829f,0.0038145349826663733f,-0.003591950051486492f,-0.12350588291883469f,0.0027068571653217077f,-0.002701706485822797f,0.010416999459266663f,-0.03633948415517807f,-0.007105713244527578f,-0.007607095409184694f,-0.016286252066493034f,-0.019802091643214226f,0.019816290587186813f,0.01683717593550682f,-0.1382036805152893f,-0.0005063081043772399f,-0.005743259564042091f,-0.017372068017721176f,0.0041589150205254555f,-0.004158380441367626f,0.02813747711479664f,-0.04523766413331032f,-0.011309951543807983f,-0.001882688025943935f,0.00026586619787849486f,0.00022107193944975734f,-0.00021466004545800388f,-0.03426405414938927f,0.06309187412261963f,0.015096710063517094f,0.024009548127651215f,-0.38188526034355164f,0.05878045782446861f,-0.058773692697286606f,-0.0310039259493351f,-0.034287795424461365f,0.01855418272316456f,-0.009765694849193096f,-0.2514644265174866f,0.01062703225761652f,-0.01062727626413107f,-0.03346728906035423f,-0.021860187873244286f,-0.009638287127017975f,-0.002076470758765936f,-0.19804267585277557f,0.00413386058062315f,-0.004136813338845968f,0.13163045048713684f,0.10321588814258575f,0.003845270024612546f,0.02776573970913887f,0.10715479403734207f,0.026255669072270393f,-0.02625351771712303f,0.06423074007034302f,0.08130349218845367f,-0.0058687650598585606f,0.00723450118675828f,0.0024828221648931503f,0.00973458681255579f,-0.009739497676491737f,0.0012686048867180943f,0.010528749786317348f,-0.00785498321056366f,-0.0003491403767839074f,-0.06436660885810852f,0.003958523739129305f,-0.0039573353715240955f,0.03681101277470589f,-0.01918310672044754f,0.010416538454592228f,-0.0039740754291415215f,-0.12833091616630554f,-0.02950427122414112f,0.029505040496587753f,0.020643144845962524f,-0.008735804818570614f,0.0002996847906615585f,-0.00950008723884821f,-0.08611041307449341f,0.0010578607907518744f,-0.0010617136722430587f,-0.039146073162555695f,-0.045036185532808304f,0.030170535668730736f,-0.010284551419317722f,-0.06998682022094727f,0.003745944704860449f,-0.0037453570403158665f,0.017413858324289322f,-0.02839607559144497f,0.011729301884770393f,-0.026698708534240723f,-0.13942086696624756f,-0.013358090072870255f,0.013356657698750496f,0.021885868161916733f,0.013949782587587833f,0.013413017615675926f,-0.002335422905161977f,-0.08841241151094437f,0.005590285174548626f,-0.005593801848590374f,0.04348849132657051f,0.08454031497240067f,0.014477104879915714f,0.0004192029300611466f,-0.06013621762394905f,0.006100246217101812f,-0.006102792453020811f,0.0005243935738690197f,-0.013545526191592216f,-0.010284025222063065f,-0.006940547842532396f,-0.16383230686187744f,-0.020950039848685265f,0.020950384438037872f,-0.026928169652819633f,-0.04568227380514145f,0.004522411618381739f,-0.00611469941213727f,-0.14355063438415527f,0.00261623109690845f,-0.002617883961647749f,-0.006089621689170599f,-0.004756170324981213f,0.010031027719378471f,-0.0013119539944455028f,-0.41546115279197693f,0.0015316065400838852f,-0.0015341060934588313f,0.019537946209311485f,0.0015301033854484558f,-0.0046652378514409065f,0.001853344845585525f,-0.05436835438013077f,-0.00958392582833767f,0.009576619602739811f,0.010178964585065842f,0.008897315710783005f,0.0006302745896391571f,-0.0006327911396510899f,0.12263457477092743f,0.03833501413464546f,-0.038331884890794754f,0.015887480229139328f,0.020238878205418587f,-0.007539567071944475f,0.004681563004851341f,-0.15712504088878632f,0.020346749573946f,-0.020344767719507217f,-0.05550088733434677f,0.01828060857951641f,0.012368716299533844f,0.05195885896682739f,-0.19665253162384033f,0.020840272307395935f,-0.020841503515839577f,-0.009686957113444805f,-0.01255156472325325f,-0.010464036837220192f,0.004264656454324722f,0.12419328838586807f,0.030545135959982872f,-0.030540963634848595f,0.018269378691911697f,-0.018243787810206413f,-0.0028452682308852673f,-0.0017410203581675887f,-0.10801330953836441f,0.008159111253917217f,-0.008162649348378181f,-0.008990451693534851f,-0.0065178950317204f,-0.010373932309448719f,-0.008228695020079613f,-0.23074164986610413f,-0.002020186046138406f,0.0020265046041458845f,0.005894741043448448f,0.03556913882493973f,0.0011884582927450538f,0.0031076755840331316f,0.007186137605458498f,0.024670980870723724f,-0.024675564840435982f,-0.026394737884402275f,-0.013048609718680382f,-0.006086288020014763f,0.00015108133084140718f,-0.03679533675312996f,0.015291989780962467f,-0.01529502123594284f,0.017518313601613045f,0.1146586686372757f,0.004849813412874937f,-0.005509825423359871f,-0.2076953500509262f,-0.07958891242742538f,0.07959208637475967f,-0.0076098195277154446f,0.17887264490127563f,0.0031312687788158655f,0.0062399692833423615f,-0.21413832902908325f,0.013587783090770245f,-0.013586664572358131f,0.012994293123483658f,0.02691655047237873f,-0.006097013596445322f,0.003851196961477399f,-0.10662385076284409f,0.008884964510798454f,-0.008887811563909054f,0.008494147099554539f,0.010801627300679684f,0.0006918483413755894f,0.015946729108691216f,-0.12230212986469269f,-0.011430594138801098f,0.011433991603553295f,0.01736786775290966f,-0.042809732258319855f,0.0011773626320064068f,-0.0014214942930266261f,-0.11905937641859055f,0.01927284337580204f,-0.019270211458206177f,0.012269217520952225f,-0.013663182035088539f,-0.003877954790368676f,0.0006519884918816388f,0.07003387063741684f,0.01780862919986248f,-0.017813555896282196f,-0.021721266210079193f,0.026688333600759506f,0.01874859258532524f,-0.011873128823935986f,0.035445671528577805f,-0.023625267669558525f,0.023625878617167473f,-0.018439754843711853f,0.07496403902769089f,0.017622021958231926f,0.0021223435178399086f,0.043968334794044495f,0.015757428482174873f,-0.015756672248244286f,-0.002955078613013029f,0.09501853585243225f,0.0007016644813120365f,0.0015926056075841188f,0.020481793209910393f,0.010965518653392792f,-0.010963653214275837f,-0.07167140394449234f,-0.026914525777101517f,-0.00947464071214199f,0.0010451152920722961f,0.0695265606045723f,-0.026621636003255844f,0.026619652286171913f,0.030712831765413284f,0.03926205635070801f,0.012076224200427532f,0.0006343991844914854f,-0.10748739540576935f,0.0025391250383108854f,-0.002536158775910735f,-0.0019265792798250914f,0.016069570556282997f,-0.0016284629236906767f,0.0016682421555742621f,0.0006729588494636118f,0.014514608308672905f,-0.01451466791331768f,0.015537258237600327f,0.004339986946433783f,-0.0005635829293169081f,-0.0061991955153644085f,0.005300017539411783f,-0.020281357690691948f,0.020276684314012527f,0.003549523651599884f,-0.00011183995957253501f,-0.006848410237580538f,0.001234289607964456f,-0.0034863632172346115f,0.009162579663097858f,-0.009163521230220795f,0.026421301066875458f,0.006905042566359043f,0.004299171268939972f,0.000500849389936775f,0.038849543780088425f,0.011610299348831177f,-0.011607801541686058f,0.01024122815579176f,-0.01650371216237545f,0.005193398334085941f,-0.0010940690990537405f,0.010232474654912949f,0.005001660902053118f,-0.005004543345421553f,-0.01074026059359312f,-0.01891045644879341f,0.005276180803775787f,-0.0002828305005095899f,0.029981546103954315f,0.010278044268488884f,-0.010279010981321335f,0.010517211630940437f,-0.042254410684108734f,-0.004773967433720827f,-0.0006464497419074178f,0.037846144288778305f,0.009236173704266548f,-0.00923564936965704f,-0.0044325245544314384f,0.007667758036404848f,-0.010826099663972855f,0.0016229921020567417f,-0.034576792269945145f,0.02037123218178749f,-0.02037847600877285f,0.006605006288737059f,-0.008636762388050556f,0.007059352472424507f,-0.0021223274525254965f,0.03475029021501541f,0.008725584484636784f,-0.008721976540982723f,0.020638084039092064f,0.026096463203430176f,0.0018261063378304243f,-0.00023792589490767568f,0.05514546483755112f,0.00925497431308031f,-0.009253915399312973f,-0.014360526576638222f,-0.020179608836770058f,-0.0028001118917018175f,-0.0031659326050430536f,-0.03074602782726288f,0.0003303720732219517f,-0.000321077648550272f,-0.010687212459743023f,-0.01347603090107441f,0.0006232889136299491f,-0.0009132293052971363f,-0.026219990104436874f,0.008777601644396782f,-0.008776254951953888f,0.003117053769528866f,-0.007727824151515961f,0.0024339354131370783f,-0.003662527771666646f,-0.0505799762904644f,0.003164714900776744f,-0.0031662341207265854f,-0.026538779959082603f,0.007265600375831127f,0.004932329058647156f,0.00689312070608139f,0.009180722758173943f,0.05369076877832413f,-0.053690504282712936f,-0.004744002595543861f,0.00107189966365695f,0.002466848585754633f,-0.0016325420001521707f,0.01328823622316122f,0.0071784695610404015f,-0.007175431586802006f,0.037304285913705826f,-0.010216676630079746f,-0.0020739766769111156f,-7.481186185032129e-05f,-0.09250736981630325f,0.007644699886441231f,-0.007640154100954533f,0.02091793529689312f,0.0111836614087224f,-0.005830329842865467f,-0.005656112916767597f,0.0025842946488410234f,0.003632005536928773f,-0.0036208240780979395f,-0.026611579582095146f,-0.0019274578662589192f,0.0005080836708657444f,0.00019065229571424425f,-0.08385058492422104f,0.008247061632573605f,-0.008247938007116318f,0.05235135182738304f,0.005129250697791576f,0.0122534716501832f,0.001751408795826137f,0.025486692786216736f,0.020462289452552795f,-0.020459838211536407f,-0.005577461794018745f,0.0076477061957120895f,-0.0001970704906852916f,0.0035910808946937323f,0.008598336018621922f,-0.018571829423308372f,0.018561847507953644f,-0.015621844679117203f,0.011148535646498203f,0.0042149098590016365f,0.005499631632119417f,-0.013352763839066029f,0.011745641008019447f,-0.011751051992177963f,-0.00032382868812419474f,-0.016046369448304176f,0.0007109693833626807f,-0.00024386642326135188f,0.019798757508397102f,0.01596725918352604f,-0.015969863161444664f,0.020565835759043694f,0.0019101389916613698f,-0.007895996794104576f,-0.012378931045532227f,0.0050419713370501995f,-0.02026708610355854f,0.02027219720184803f,0.0066839889623224735f,-0.0019069742411375046f,0.0037360023707151413f,-0.0012052642414346337f,0.02306322194635868f,0.014812422916293144f,-0.014814822934567928f,0.0010821606265380979f,0.0035188684705644846f,0.00076003820868209f,0.0016466965898871422f,-0.03256712108850479f,0.0081970589235425f,-0.008190535940229893f,0.0031876841094344854f,-0.0059013208374381065f,-0.007458462845534086f,-0.009916734881699085f,-0.02432837337255478f,-0.03918120265007019f,0.03918541967868805f,0.011277901008725166f,-0.0009624912636354566f,-0.0018569626845419407f,-0.004347636364400387f,-0.0346314013004303f,0.01066619623452425f,-0.010666083544492722f,0.04168885946273804f,0.006776171736419201f,-0.0012204534141346812f,0.0008099875994957983f,-0.045779045671224594f,0.0046167755499482155f,-0.004614358302205801f,0.012949049472808838f,0.000445093319285661f,0.005720189772546291f,-0.006586909294128418f,-0.05458082631230354f,0.015599768608808517f,-0.01560286432504654f,0.0035775015130639076f,0.008478714153170586f,0.0012347076553851366f,0.003611380700021982f,-0.04133179783821106f,0.004632285796105862f,-0.004635604098439217f,-2.0158335246378556e-05f,0.0016539861680939794f,-0.002299821237102151f,-0.0013904690276831388f,0.018300659954547882f,0.0027983656618744135f,-0.0027968529611825943f,-0.0037447577342391014f,-0.02810516580939293f,0.005074662622064352f,0.0017012505559250712f,-0.061517056077718735f,-0.018900291994214058f,0.018903229385614395f,-0.02350621670484543f,-0.00018934148829430342f,-0.0005958246765658259f,0.005057589150965214f,-0.039260149002075195f,0.016211504116654396f,-0.016210822388529778f,-0.006425544619560242f,-0.001470970339141786f,0.0018314539920538664f,0.005959410686045885f,-0.1334764063358307f,0.007881969213485718f,-0.007876244373619556f,0.004513155203312635f,-0.006454124581068754f,-0.0017704340862110257f,-0.005529175512492657f,-0.029123874381184578f,-0.04696156457066536f,0.046954721212387085f,0.004620101768523455f,0.008494328707456589f,0.007886343635618687f,0.0006036113481968641f,0.06328184902667999f,0.008134538307785988f,-0.008135277777910233f,0.03307419270277023f,-0.0011028506560251117f,-0.0029538932722061872f,0.00044791839900426567f,0.0064608436077833176f,0.0038787692319601774f,-0.00387985585257411f,-0.02565779723227024f,0.0002837123756762594f,-0.008089237846434116f,0.00480099068954587f,-0.14024773240089417f,0.026336096227169037f,-0.026325564831495285f,-0.026823263615369797f,0.016203679144382477f,0.0008517829119227827f,0.0026585368905216455f,-0.009883643127977848f,0.01629215106368065f,-0.016291474923491478f,-0.017646241933107376f,0.010971089825034142f,0.004278254229575396f,0.0018247229745611548f,-0.04825950786471367f,0.00790216214954853f,-0.007905067875981331f,-0.006627965718507767f,0.007198613602668047f,0.0033373592887073755f,0.0151358125731349f,-0.04618116095662117f,0.05586356669664383f,-0.055862441658973694f,0.022908814251422882f,0.007055534049868584f,0.003470903728157282f,0.0005795300239697099f,-0.05671627074480057f,0.008999001234769821f,-0.009004289284348488f,0.007140612695366144f,0.0028548373375087976f,0.0062765153124928474f,-0.0024274729657918215f,-0.020237980410456657f,0.004740006290376186f,-0.004740140866488218f,-0.02168181538581848f,0.013742871582508087f,0.0016874285647645593f,0.009325955994427204f,-0.0975969061255455f,0.006194339133799076f,-0.0061896867118775845f,-0.001833815244026482f,0.00275926711037755f,0.001574793946929276f,0.003792372765019536f,0.006663115695118904f,0.015652760863304138f,-0.015652155503630638f,-0.007823694497346878f,0.01240016333758831f,0.005118735134601593f,0.0019487652461975813f,-0.018333302810788155f,0.01522925402969122f,-0.01522468589246273f,-0.016551263630390167f,0.005985737312585115f,0.0011228581424802542f,-0.023453330621123314f,-0.05120786651968956f,-0.008162031881511211f,0.008154230192303658f,0.04594632610678673f,-0.01366946566849947f,-0.004236538894474506f,0.03539230674505234f,-0.030575497075915337f,0.01493566669523716f,-0.014934751205146313f,0.03558023273944855f,-0.012849968858063221f,0.0034365449100732803f,0.004229454789310694f,-0.031555116176605225f,0.0134736904874444f,-0.013468360528349876f,-0.020368186756968498f,0.0022878365125507116f,-0.01213790476322174f,-0.009133944287896156f,-0.017232220619916916f,-0.01564660668373108f,0.015651913359761238f,-0.006171311717480421f,-0.016855698078870773f,-0.0015007156180217862f,0.0011138342088088393f,-0.07011784613132477f,0.010048804804682732f,-0.010046630166471004f,-0.034013453871011734f,-0.02062293328344822f,-0.0008892395999282598f,-0.0027193373534828424f,0.011418765410780907f,0.003770101349800825f,-0.0037718755193054676f,0.04149048030376434f,-0.0003034562396351248f,0.009968247264623642f,0.002472048858180642f,-0.08034329861402512f,0.028491491451859474f,-0.028476949781179428f,-0.004959158133715391f,-0.0003517069562803954f,-0.0020716420840471983f,0.001048401347361505f,0.0016203095437958837f,0.008196366019546986f,-0.008200136013329029f,-0.01432560384273529f,0.01545675564557314f,-0.0017522090347483754f,0.0014827897539362311f,-0.04325534403324127f,0.0025487311650067568f,-0.0025519803166389465f,0.019616486504673958f,0.013797574676573277f,0.010343236848711967f,-0.0040043191984295845f,0.054819829761981964f,-0.008556083776056767f,0.008557181805372238f,-0.004399733152240515f,0.007589440327137709f,-0.009032497182488441f,-0.0027349672745913267f,-0.06711734086275101f,0.017124366015195847f,-0.01712726429104805f,-0.01970655471086502f,-0.0023010605946183205f,0.019814427942037582f,-0.00014245882630348206f,-0.06263963133096695f,0.011576319113373756f,-0.011573757976293564f,0.01521382387727499f,-0.010390634648501873f,0.0073288907296955585f,-0.004470373038202524f,0.0029517675284296274f,-0.02811366692185402f,0.028115371242165565f,0.0007530946168117225f,0.005250199232250452f,-0.0007899418706074357f,0.001967269228771329f,-0.05341341346502304f,0.015788767486810684f,-0.01578892208635807f,0.010061944834887981f,0.0035863318480551243f,-0.004372410476207733f,0.0014584680320695043f,-0.0024688688572496176f,0.02229245752096176f,-0.022289523854851723f,0.018408440053462982f,-0.006233363412320614f,0.00958809070289135f,0.005376917775720358f,0.003861134173348546f,0.020213855430483818f,-0.020214146003127098f,0.0015218759654089808f,0.007894926704466343f,0.0024081533774733543f,1.582045115355868e-05f,0.02557150460779667f,0.011197637766599655f,-0.01120002195239067f,0.042141351848840714f,-0.011591966263949871f,0.005986493080854416f,0.002062376821413636f,0.07591595500707626f,0.010672345757484436f,-0.010673546232283115f,0.019024306908249855f,-0.006787716411054134f,0.002465894678607583f,0.0025236320216208696f,-0.009200948290526867f,0.015818003565073013f,-0.015820907428860664f,0.010027231648564339f,-0.0017430459847673774f,0.006838174071162939f,-0.000334717333316803f,0.029707401990890503f,0.00826343521475792f,-0.008265701122581959f,-0.009410989470779896f,-0.011621406301856041f,0.006639116909354925f,0.004055738914757967f,-0.034121572971343994f,0.00996382161974907f,-0.009962392970919609f,0.006124375853687525f,-0.005615778733044863f,-0.005377491004765034f,-0.01211410854011774f,-0.08167222887277603f,0.020988203585147858f,-0.020973173901438713f,-0.00520027382299304f,0.0012719279620796442f,-0.006135696079581976f,0.001149288727901876f,0.021115249022841454f,0.006616896018385887f,-0.006612819619476795f,0.0010928033152595162f,0.004572420846670866f,0.006143884267657995f,0.003291240893304348f,-0.061523668467998505f,0.0024593574926257133f,-0.002459108829498291f,-0.0200154110789299f,0.008677907288074493f,0.001014953712001443f,-0.004699212033301592f,-0.04155381768941879f,-0.029274141415953636f,0.029271671548485756f,-0.008912294171750546f,0.028265152126550674f,0.011991968378424644f,0.006954093463718891f,-0.06588929146528244f,0.006686181295663118f,-0.006688148248940706f,0.05076521635055542f,0.020999787375330925f,0.006941897328943014f,-0.001687613665126264f,-0.027992280200123787f,0.012746780179440975f,-0.01274656318128109f,0.012044570408761501f,-0.005923224613070488f,-0.006598007865250111f,-0.005822623614221811f,0.05287901684641838f,-0.0030989511869847775f,0.0031002804171293974f,0.01587342470884323f,0.020138751715421677f,-0.0004868770483881235f,-0.0013379963347688317f,-0.11325649172067642f,0.004695562645792961f,-0.004688989836722612f,-0.00023321610933635384f,0.014934943988919258f,0.003155728802084923f,0.0002886027214117348f,-0.0784321203827858f,0.005133837461471558f,-0.005136737134307623f,-7.021622150205076e-05f,-0.013758519664406776f,-0.0028967636171728373f,0.004109700210392475f,0.006933269556611776f,0.00648906547576189f,-0.006492487620562315f,0.0041259890422225f,0.004279885906726122f,0.006874323822557926f,0.001991539029404521f,-0.06873511523008347f,0.006306688766926527f,-0.00630673673003912f,0.001815014984458685f,0.01027235109359026f,0.0076508475467562675f,-0.00012470768706407398f,-0.040436334908008575f,0.005041138734668493f,-0.0050423843786120415f,0.013710567727684975f,-0.006167092360556126f,0.008357282727956772f,0.015303359366953373f,-0.0715770348906517f,0.09710268676280975f,-0.09710750728845596f,-0.0290096253156662f,0.005363013129681349f,-0.009679067879915237f,0.002771518426015973f,0.042092833667993546f,0.0034414546098560095f,-0.0034440704621374607f,-0.018282458186149597f,0.00412110285833478f,0.00622705090790987f,-0.0028713697101920843f,-0.016790492460131645f,0.0013599079102277756f,-0.0013584868283942342f,-0.0030189715325832367f,-0.01222248375415802f,-0.008084887638688087f,-0.011169502511620522f,0.034399207681417465f,-0.05224888026714325f,0.05224843695759773f,-0.007162438239902258f,-0.015281621366739273f,0.008998528122901917f,-0.0017505433643236756f,-0.01645663008093834f,0.006844526156783104f,-0.006843014154583216f,0.002978692762553692f,0.007789091672748327f,0.004893936682492495f,0.0012855665991082788f,-0.04716416820883751f,0.0028899475000798702f,-0.0028946176171302795f,-0.006359212566167116f,0.012974951416254044f,0.004500047769397497f,0.0031899004243314266f,-0.08801762014627457f,0.024547478184103966f,-0.02454639971256256f,-0.006566823460161686f,0.0020316257141530514f,0.0029403255321085453f,-0.0022717241663485765f,-0.0827566459774971f,0.010061233304440975f,-0.01006346195936203f,0.02984616532921791f,-0.015096682123839855f,4.226334931445308e-05f,-0.0037366312462836504f,0.014415603131055832f,0.007462797220796347f,-0.007463789079338312f,0.013892626389861107f,-0.009028316475450993f,-0.008781581185758114f,-0.007999058812856674f,-0.013006099499762058f,0.006140282843261957f,-0.006136523559689522f,-0.0019321085419505835f,0.002941656857728958f,-0.009139608591794968f,-0.0006689007277600467f,0.00752446660771966f,0.004458240699023008f,-0.004455750808119774f,-0.027860667556524277f,0.012693556025624275f,-0.009889738634228706f,0.0011052421759814024f,0.05571294575929642f,0.0036271216813474894f,-0.0036239016335457563f,-0.031086571514606476f,-0.027595071122050285f,0.007462484296411276f,-0.0027765631675720215f,-0.01879814639687538f,0.010799783281981945f,-0.0108058862388134f,0.02243735082447529f,0.01874818094074726f,-0.006434921640902758f,0.0005717097665183246f,-0.00803617388010025f,0.0051693241111934185f,-0.005172458477318287f,-0.002604419831186533f,-0.025760753080248833f,0.019420113414525986f,-0.00020191114163026214f,-0.03625751659274101f,0.005370340775698423f,-0.005371659062802792f,-0.021102072671055794f,0.007431197911500931f,0.011752292513847351f,0.007275556214153767f,0.0453239344060421f,0.039360933005809784f,-0.03936095908284187f,-0.008460074663162231f,0.0017046923749148846f,0.0031680380925536156f,-0.0006753006600774825f,-0.019017955288290977f,0.007633222732692957f,-0.007630759850144386f,-0.011286401189863682f,-0.010547276586294174f,0.00864279642701149f,-0.0018439277773723006f,0.0015184885123744607f,0.004012291319668293f,-0.00401122123003006f,-0.0010358860017731786f,-0.0016141922678798437f,0.0005346485995687544f,-0.001175756216980517f,-0.07630304992198944f,-0.015201651491224766f,0.01520274393260479f,-0.012897898443043232f,-0.005802058149129152f,-0.002854512073099613f,-0.0003127093950752169f,0.006727202795445919f,0.009941793978214264f,-0.009942378848791122f,-0.05430178716778755f,-0.038526203483343124f,0.002081101993098855f,-0.0004069709393661469f,-0.0343342162668705f,0.005315919406712055f,-0.005314145237207413f,-0.025156455114483833f,-0.008415228687226772f,0.0011334022274240851f,-0.01242073718458414f,0.02579558826982975f,-0.019122404977679253f,0.019111348316073418f,0.0035078865475952625f,0.010800221003592014f,0.007677716668695211f,0.004071883857250214f,-0.07554886490106583f,0.013518367893993855f,-0.013518555089831352f,-0.001433615107089281f,0.015921104699373245f,0.007054649293422699f,0.000957620213739574f,0.038994379341602325f,0.012992226518690586f,-0.012992650270462036f,0.0018929385114461184f,-0.0034230821765959263f,6.499890150735155e-05f,0.000769882695749402f,0.0023614841047674417f,0.012353897094726562f,-0.012359065935015678f,0.007618199568241835f,0.014601639471948147f,0.003687725169584155f,0.0007441660854965448f,-0.014550628140568733f,0.01101785060018301f,-0.011017099022865295f,-0.028707610443234444f,-0.00883302092552185f,-0.0007102435920387506f,-0.0044344044290483f,-0.01685030199587345f,0.005007093772292137f,-0.005005897022783756f,0.011893845163285732f,0.007491678930819035f,0.014452303759753704f,0.0016749243950471282f,-0.010967685841023922f,0.025201814249157906f,-0.025202292948961258f,0.010074867866933346f,0.02064954675734043f,0.0019427905790507793f,-0.00013104606478009373f,-0.06810209155082703f,0.00836852751672268f,-0.008367585018277168f,-0.03326951712369919f,-0.007757261861115694f,-0.0026688126381486654f,0.0016576854977756739f,-0.01706964522600174f,0.009483533911406994f,-0.009483122266829014f,-0.030265051871538162f,0.014519643038511276f,0.0069355894811451435f,-0.003829021705314517f,-0.005405612755566835f,-0.018619582056999207f,0.01861736737191677f,-0.00975124817341566f,0.014656661078333855f,-0.005284955259412527f,0.0012310222955420613f,0.020918522030115128f,0.007754791062325239f,-0.0077568418346345425f,-0.02467223070561886f,0.021297935396432877f,-0.004301420412957668f,-0.0004034982994198799f,-0.03405069559812546f,0.006431299727410078f,-0.006431097164750099f,-0.011781400069594383f,-0.006613679230213165f,-0.001085291733033955f,-0.0029300362803041935f,-0.08300717920064926f,0.0024412316270172596f,-0.002429828280583024f,-0.005684181582182646f,-0.005059960298240185f,-0.009413509629666805f,-0.001121046836487949f,0.03750203177332878f,0.014119567349553108f,-0.01411573588848114f,-0.0044945920817554f,-0.0012374176876619458f,-0.004678354598581791f,-0.00014808928244747221f,-0.1403375118970871f,0.009894954971969128f,-0.009896394796669483f,0.011554491706192493f,0.006670367904007435f,0.005558904260396957f,-0.0048592351377010345f,0.0006551191327162087f,-0.040595412254333496f,0.04059357941150665f,-0.007499225903302431f,-0.01532266940921545f,0.008176770992577076f,-9.384217264596373e-05f,0.026188133284449577f,0.008439354598522186f,-0.008436989039182663f,-0.01939423196017742f,0.023988164961338043f,0.004725498612970114f,-0.00015365735453087837f,-0.008666998706758022f,0.0073263319209218025f,-0.007322919555008411f,-0.010587864555418491f,0.00017185363685712218f,-0.007562621496617794f,-0.007542946841567755f,0.023799672722816467f,-0.034918639808893204f,0.03491606190800667f,-0.004524189978837967f,-0.0029828930273652077f,-0.0026184548623859882f,-0.0034383994061499834f,-0.07409369200468063f,0.004397460725158453f,-0.004395155236124992f,0.040117647498846054f,0.030656475573778152f,0.0033223924692720175f,6.233090243767947e-05f,-0.13437840342521667f,0.005373734515160322f,-0.00537090701982379f,0.0015523136826232076f,-0.0049135019071400166f,0.009530756622552872f,-0.00037578772753477097f,0.006511657498776913f,0.001059141242876649f,-0.0010573440231382847f,-0.03301021084189415f,-0.0005034157074987888f,-0.0019745053723454475f,0.0004927006084471941f,0.07715806365013123f,0.005061232019215822f,-0.005064160097390413f,-0.029957273975014687f,0.0031727312598377466f,0.010442362166941166f,0.002798287430778146f,0.0029835731256753206f,0.0036392263136804104f,-0.0036396514624357224f,-0.046722929924726486f,0.01598992943763733f,0.0019102055812254548f,0.007653809152543545f,0.0907939225435257f,-0.01768609508872032f,0.017688697203993797f,0.0004349700757302344f,0.020238175988197327f,-0.0021301840897649527f,0.003192748175933957f,-0.036735475063323975f,0.00799587182700634f,-0.007993850857019424f,-0.027243662625551224f,0.001524895429611206f,0.0014933234779164195f,-0.002659101039171219f,0.02292660064995289f,0.006634301505982876f,-0.0066280486062169075f,0.018174879252910614f,-0.02322212979197502f,-0.0033006840385496616f,-0.00873382855206728f,0.0040397727862000465f,-0.0348254032433033f,0.034824732691049576f,0.006213516928255558f,-0.013370383530855179f,0.0030134236440062523f,-0.0011536020319908857f,-0.027173999696969986f,0.006507395301014185f,-0.006508886814117432f,-0.02006339840590954f,-0.03974708914756775f,0.008134161122143269f,-0.0028950341511517763f,0.07836564630270004f,0.004642810672521591f,-0.004644123371690512f,0.005972741171717644f,0.005867664702236652f,0.00023987884924281389f,-0.005526619963347912f,-0.09307100623846054f,0.02241458371281624f,-0.02242346666753292f,0.01972123421728611f,0.00046483948244713247f,0.0032725671771913767f,0.0008489348110742867f,0.058158379048109055f,0.005284876096993685f,-0.005281069315969944f,0.010317381471395493f,-0.004591247998178005f,0.010300330817699432f,-0.004619647283107042f,-0.05332024022936821f,0.001325390418060124f,-0.0013281545834615827f,-0.0063380771316587925f,-0.0061909910291433334f,0.008362761698663235f,-0.0009755170904099941f,-0.003734715050086379f,-0.017633231356739998f,0.0176332276314497f,0.016705727204680443f,0.014212049543857574f,-0.009314408525824547f,-0.004273680970072746f,0.0319785512983799f,0.004918294493108988f,-0.004920920357108116f,-0.006741304881870747f,-0.005377328488975763f,-0.002506064949557185f,4.700730187323643e-07f,-0.060907404869794846f,0.0035053095780313015f,-0.003506417153403163f,0.005542065482586622f,-0.0148469852283597f,-0.00024340610252693295f,0.0006589156109839678f,0.048847753554582596f,0.0014530731132254004f,-0.001457365695387125f,-0.036820463836193085f,0.021392814815044403f,-0.003261032747104764f,0.006581942085176706f,0.026788847520947456f,0.0027544288896024227f,-0.0027508253697305918f,0.005454723257571459f,-0.011105122976005077f,0.01304218266159296f,-0.0033982405439019203f,-0.003127421485260129f,0.004721698351204395f,-0.004722090438008308f,0.0052015408873558044f,-0.001664780080318451f,0.0022426520008593798f,-0.008241103030741215f,0.02943689376115799f,-0.01926882378757f,0.019270746037364006f,-0.006057876627892256f,0.005691690370440483f,0.009724363684654236f,0.005350063554942608f,0.03437160328030586f,0.008677835576236248f,-0.008676494471728802f,-0.027349451556801796f,-0.008231021463871002f,-0.012395824305713177f,-0.0016100595239549875f,0.023321447893977165f,0.006712049711495638f,-0.006717650685459375f,0.0104471854865551f,0.014522670768201351f,-0.009295702911913395f,0.0005441283574327826f,0.020015913993120193f,-0.008507754653692245f,0.00850362703204155f,-0.017509840428829193f,0.018371587619185448f,-0.002313809935003519f,0.000610480725299567f,0.0217497106641531f,0.009446091018617153f,-0.00944878812879324f,-0.021587446331977844f,-0.01960337720811367f,-0.007739076390862465f,-0.004414486698806286f,-0.006804772652685642f,0.006617450620979071f,-0.006623485591262579f,0.020912829786539078f,-0.0034156611654907465f,-0.008971436880528927f,0.0031129869166761637f,0.11067453026771545f,0.03153349831700325f,-0.03153768554329872f,0.0025112777948379517f,-0.009276501834392548f,-0.012784232385456562f,-0.0032619875855743885f,-0.057381343096494675f,0.006998772267252207f,-0.00699998252093792f,-0.021012088283896446f,0.016689680516719818f,0.006794714368879795f,-0.0028622127138078213f,-0.09654399752616882f,0.001100523048080504f,-0.0010996588971465826f,0.002670486457645893f,-0.0025289389304816723f,-0.004329829476773739f,0.011900043115019798f,-0.0755307748913765f,0.01977209746837616f,-0.01977420412003994f,0.025421252474188805f,-0.010077615268528461f,-0.011797768995165825f,-0.005365128628909588f,0.056959956884384155f,0.005758126266300678f,-0.005758464336395264f,-0.0022212087642401457f,-0.011404285207390785f,-0.005872545298188925f,-0.007398550398647785f,-0.009341809898614883f,0.0013461256166920066f,-0.0013459180481731892f,-0.02909529209136963f,-0.02197273075580597f,-0.02052498795092106f,-0.008425557054579258f,0.021243738010525703f,-0.025758039206266403f,0.025746725499629974f,-0.0007533772150054574f,-0.014407745562493801f,-0.018371812999248505f,0.009639416821300983f,0.000655941606964916f,0.01052927877753973f,-0.010532164946198463f,-0.0209308210760355f,-0.0256651621311903f,-0.011411880142986774f,-0.0017794764135032892f,-0.05670563504099846f,0.007315181195735931f,-0.007314298301935196f,0.016194524243474007f,-0.017597490921616554f,-0.0004218672984279692f,-0.0023107286542654037f,0.009278370067477226f,0.022768106311559677f,-0.022778833284974098f,0.0065383282490074635f,0.005057825706899166f,0.007243828848004341f,0.0034248167648911476f,-0.04540540277957916f,0.006596770603209734f,-0.006601493339985609f,-0.005235808435827494f,-0.0002905042201746255f,0.0038212572690099478f,-0.0010731997899711132f,0.06747926026582718f,0.0035154882352799177f,-0.0035174544900655746f,-0.01137570571154356f,-0.004822978749871254f,0.0028806093614548445f,-0.007419797591865063f,-0.06646160781383514f,0.005589068867266178f,-0.005592972505837679f,-0.010547368787229061f,-0.0018362437840551138f,0.020528435707092285f,-0.008824232965707779f,0.08025933057069778f,0.011386891826987267f,-0.011387523263692856f,0.01741143874824047f,0.0001522033999208361f,-0.00995117612183094f,-0.005523059982806444f,0.021013963967561722f,0.006490030791610479f,-0.006492156535387039f,-0.024835743010044098f,-0.02461065724492073f,-0.004244083538651466f,0.005106781609356403f,0.023000970482826233f,0.03528021648526192f,-0.03527520224452019f,-0.021959738805890083f,-0.014515982940793037f,0.003813006915152073f,-0.004212488420307636f,-0.004998310469090939f,0.004328962881118059f,-0.004327695816755295f,0.011463452130556107f,-0.0013529114658012986f,0.0027487757615745068f,-0.00590455112978816f,-0.0010153031907975674f,0.003165462054312229f,-0.003160532796755433f,-0.000672058726195246f,0.006429634522646666f,-0.001715901424176991f,0.0036628523375838995f,-0.05585913360118866f,0.014254234731197357f,-0.014258027076721191f,-0.004288736265152693f,0.01639712043106556f,0.008587845601141453f,0.00047850245027802885f,0.04356939345598221f,0.008337438106536865f,-0.008330007083714008f,-0.028408637270331383f,-0.0005495286313816905f,0.006728183012455702f,-0.0020312450360506773f,0.025321993976831436f,0.009643207304179668f,-0.009641480632126331f,-0.01574948988854885f,0.006044128909707069f,0.002733109286054969f,0.014361262321472168f,0.031576428562402725f,-0.05725153535604477f,0.0572548508644104f,0.011610389687120914f,0.005173992365598679f,0.01382133737206459f,0.006028932053595781f,-0.04678908362984657f,0.0035407026298344135f,-0.003539949655532837f,-0.022554777562618256f,0.028023438528180122f,0.016950713470578194f,0.009447473101317883f,0.051440563052892685f,0.007367147132754326f,-0.007367386482656002f,0.04966214299201965f,-0.0009374302462674677f,0.0007770296069793403f,0.000858605548273772f,-0.06770089268684387f,0.0019501603674143553f,-0.0019515968160703778f,0.005891314707696438f,0.010705146007239819f,0.011419827118515968f,-0.004045394714921713f,0.019807782024145126f,0.007246822584420443f,-0.007246833294630051f,0.006741276476532221f,-0.028152134269475937f,0.0044801440089941025f,-0.0016956640174612403f,0.043513961136341095f,0.005050970241427422f,-0.005051448009908199f,-0.020063290372490883f,-0.005155343562364578f,0.010848766192793846f,0.0004977377830073237f,-0.0780925527215004f,0.04512627422809601f,-0.0451328381896019f,0.020187878981232643f,-0.008551659993827343f,-0.0006331696058623493f,-0.0019821501336991787f,0.025253791362047195f,0.006446155719459057f,-0.0064443196170032024f,0.018917955458164215f,-0.028278561308979988f,0.011146253906190395f,-0.004851227160543203f,0.02012733183801174f,0.0014839860377833247f,-0.0014832888264209032f,-0.02841784432530403f,0.015706444159150124f,0.009948588907718658f,-0.004727839957922697f,-0.07388904690742493f,-0.004237558227032423f,0.004229245241731405f,-0.03112848475575447f,-0.01257605291903019f,-0.001691680634394288f,-0.0023926885332912207f,-0.07411399483680725f,0.008043168112635612f,-0.008042181842029095f,0.025897573679685593f,-0.0172648373991251f,0.010115641169250011f,0.0009644777746871114f,-0.007158083841204643f,0.008445405401289463f,-0.008443820290267467f,0.017371252179145813f,0.0007139969384297729f,-0.01373512763530016f,-0.010413398034870625f,0.010481410659849644f,-0.02463662251830101f,0.024637501686811447f,0.028223421424627304f,-0.030917389318346977f,-0.007279104553163052f,-0.003828581189736724f,0.02649375982582569f,0.010509425774216652f,-0.010503708384931087f,0.010257181711494923f,-0.008822165429592133f,-0.010006221942603588f,-0.0033178897574543953f,-0.00014081603148952127f,0.01019678357988596f,-0.010199697688221931f,-0.00786976981908083f,-0.007106189150363207f,-0.0008517893147654831f,-0.0020973486825823784f,-0.0936313197016716f,0.017031554132699966f,-0.017040589824318886f,0.007021442987024784f,-0.02840987779200077f,-0.0011462927795946598f,-0.005260291509330273f,-0.001531494315713644f,0.004526084754616022f,-0.00452408054843545f,0.008082046173512936f,0.00360051728785038f,0.008900728076696396f,-0.004079743754118681f,-0.029666593298316002f,0.004727169871330261f,-0.004722226411104202f,-0.007163608446717262f,0.00779068423435092f,-0.0015772867482155561f,0.0059440420009195805f,0.02339763566851616f,0.024599457159638405f,-0.02460652031004429f,0.003077020635828376f,0.007748408243060112f,0.004431650508195162f,0.0007983750547282398f,0.013046608306467533f,0.005259606521576643f,-0.005257066804915667f,0.02396313287317753f,0.006367194931954145f,-0.0050812833942472935f,-0.002375764772295952f,-0.054009176790714264f,0.003846242558211088f,-0.0038496283814311028f,-0.010028267279267311f,0.015154633671045303f,-0.01012237835675478f,2.5090597773669288e-05f,0.008449239656329155f,-0.0008922000997699797f,0.0008928293827921152f,0.01608019508421421f,0.015413218177855015f,-0.004816326312720776f,-7.149881639634259e-06f,-0.011254964396357536f,0.0051222811453044415f,-0.005121300928294659f,0.0021116433199495077f,0.00033851549960672855f,0.0028645764105021954f,-0.0005941292038187385f,0.02809995599091053f,0.004006906412541866f,-0.004004636313766241f,-0.006292952224612236f,-0.017395900562405586f,0.002142614684998989f,0.0038774204440414906f,0.03150854632258415f,-0.0011214520782232285f,0.0011222277535125613f,-0.008351963013410568f,0.004660396836698055f,0.00021823115821462125f,0.00022886160877533257f,-0.022457679733633995f,0.003930510021746159f,-0.003927920944988728f,-0.023924274370074272f,-0.010982797481119633f,0.00935493316501379f,-0.006937961094081402f,0.06164878234267235f,0.004113746806979179f,-0.004116580355912447f,-0.007513989694416523f,0.022097252309322357f,-0.020314615219831467f,0.0035179832484573126f,-0.12040995806455612f,-0.00396195100620389f,0.003966661170125008f,0.005634746979922056f,0.013398079201579094f,-0.006662297993898392f,0.0039912015199661255f,-0.07194162905216217f,0.0023418813943862915f,-0.0023467119317501783f,0.008345961570739746f,-0.01752679981291294f,0.001009242027066648f,-0.0005628667422570288f,-0.10882139205932617f,0.0011106861056759953f,-0.0011142214061692357f,0.014343487098813057f,-0.03535538911819458f,0.0025799525901675224f,-0.0046890731900930405f,-0.024982744827866554f,0.020739419385790825f,-0.020746571943163872f,-0.009172516874969006f,-0.022282546386122704f,-0.0015937616117298603f,-0.004508997313678265f,0.011677483096718788f,0.0029746335931122303f,-0.0029800143092870712f,0.04017234593629837f,0.004471322987228632f,-0.01889338344335556f,-0.0023363251239061356f,-0.005544482264667749f,0.0015973361441865563f,-0.0015998372109606862f,0.002383067738264799f,0.005369024351239204f,-0.0008675738936290145f,0.0006310323951765895f,0.03812333196401596f,0.03406255319714546f,-0.03406652435660362f,-0.009938410483300686f,0.0010082355001941323f,0.004118390381336212f,-0.007887695915997028f,0.00823977030813694f,0.001693147118203342f,-0.0016896738670766354f,0.02633572928607464f,0.005361801944673061f,0.012068239040672779f,-0.00545910419896245f,0.011462394148111343f,0.0010358380386605859f,-0.0010406714864075184f,0.0346546396613121f,-0.0064678979106247425f,-0.0022242183331400156f,-0.005937864072620869f,-0.0077908774837851524f,0.053319305181503296f,-0.053320351988077164f,-0.008658933453261852f,0.012240687385201454f,0.0007730198558419943f,-0.0016256265807896852f,-0.0042421529069542885f,0.006820904091000557f,-0.006815091706812382f,-0.01658318191766739f,0.012204158119857311f,-0.014308635145425797f,-0.001295215217396617f,0.007788693066686392f,0.0030031315982341766f,-0.0030080871656537056f,0.021412989124655724f,-0.011605420149862766f,0.008526269346475601f,0.0036261198110878468f,-0.17029033601284027f,-0.010213389061391354f,0.010214255191385746f,0.032749664038419724f,-0.01543557271361351f,0.009111780673265457f,-0.0008050624746829271f,0.012528039515018463f,0.004190640989691019f,-0.004194278735667467f,-0.013307413086295128f,-0.002847491530701518f,-0.012081213295459747f,-0.0017352866707369685f,-0.027743106707930565f,0.0033697315957397223f,-0.003369492944329977f,0.011634266003966331f,-0.013464550487697124f,-0.005092335864901543f,-0.010163608938455582f,0.03008521907031536f,-0.021473731845617294f,0.02148001827299595f,-0.0158318392932415f,0.008325768634676933f,-0.0038869527634233236f,0.002507118508219719f,0.01962832547724247f,0.0018174843862652779f,-0.0018152579432353377f,-0.017788328230381012f,0.00017993197252508253f,-0.007691072765737772f,-0.0009717722423374653f,-0.016368405893445015f,0.0014666536590084434f,-0.0014718294842168689f,0.05332273617386818f,-0.0003795368829742074f,-0.003689215052872896f,-0.0019857976585626602f,0.021836601197719574f,0.0025799409486353397f,-0.002578481798991561f,0.01805460825562477f,0.008801784366369247f,-0.00643371045589447f,0.00328082125633955f,-0.038118451833724976f,0.001777260098606348f,-0.0017778252949938178f,-0.014308245852589607f,0.005284279119223356f,0.04703185334801674f,0.0038236654363572598f,0.0023139012046158314f,0.0033481724094599485f,-0.0033507877960801125f,-0.019318148493766785f,0.0073474436067044735f,0.0011187612544745207f,0.009866121225059032f,0.0766187533736229f,-0.007299297023564577f,0.0073066516779363155f,0.023181425407528877f,0.011713836342096329f,-0.021936040371656418f,0.0003868735220748931f,0.011727503500878811f,0.004529549274593592f,-0.004528581164777279f,0.0071233948692679405f,-0.012337915599346161f,-0.010055392049252987f,-0.004445461556315422f,0.01768307201564312f,0.004544110968708992f,-0.004541246686130762f,0.020943766459822655f,-0.03136058524250984f,-0.00434673810377717f,-0.00035914030740968883f,-0.02356012910604477f,-0.031787384301424026f,0.0317859947681427f,-0.005865596234798431f,0.008822126314043999f,0.0025975285097956657f,-0.0016464795917272568f,-0.02425660565495491f,0.007930023595690727f,-0.007928281091153622f,-0.005914718843996525f,-0.007497952785342932f,-0.002183424774557352f,0.0007307527121156454f,0.03762323036789894f,0.008954626508057117f,-0.008955294266343117f,0.026050817221403122f,-0.00468769297003746f,0.0013670077314600348f,0.003626781515777111f,0.004389932844787836f,-0.011626559309661388f,0.011634665541350842f,0.009833044372498989f,0.024005642160773277f,-0.007566235028207302f,0.006668567657470703f,-0.030998624861240387f,0.007937518879771233f,-0.007935844361782074f,-0.03157401829957962f,0.00762168737128377f,-0.009422152303159237f,0.003124146955087781f,-0.024340897798538208f,0.005085318814963102f,-0.005088537931442261f,0.007676144130527973f,0.004609955009073019f,-0.010236340574920177f,-0.00029292466933839023f,-0.042475901544094086f,0.07437609881162643f,-0.0743856206536293f,0.0043977778404951096f,-0.010883672162890434f,0.007419368252158165f,-0.003982458729296923f,-0.009635724127292633f,0.006456821225583553f,-0.00645546754822135f,-0.010523658245801926f,0.00847124494612217f,0.008501713164150715f,-0.00424790708348155f,-0.04173698276281357f,0.0011988322949036956f,-0.0011968736071139574f,0.017103036865592003f,-0.00971317384392023f,-0.0073701851069927216f,-0.00684289401397109f,-0.04414238780736923f,-0.001850759144872427f,0.0018560948083177209f,0.005648687481880188f,0.001092991791665554f,-0.009833896532654762f,-0.0035777899902313948f,-0.005617775954306126f,0.003485399764031172f,-0.0034817298874258995f,0.0044904244132339954f,-0.0003387780161574483f,-0.006157205905765295f,-0.002282508183270693f,0.0392535999417305f,0.00344565836712718f,-0.003445828566327691f,-0.015902413055300713f,0.0026404671370983124f,0.002896383637562394f,0.003638015827164054f,0.0924292504787445f,0.003052546875551343f,-0.003053938737139106f,0.0001675227249506861f,0.004182476084679365f,-0.006268692668527365f,0.003649387275800109f,-0.02568841353058815f,0.0031763878650963306f,-0.0031796968542039394f,0.025184739381074905f,0.008611204102635384f,-0.007859766483306885f,0.002074502408504486f,0.00491970544680953f,0.0041426862590014935f,-0.004147765692323446f,0.024947281926870346f,-0.0013668910833075643f,0.013630389235913754f,0.003986046649515629f,-0.1657232642173767f,-0.007531441282480955f,0.007533439435064793f,0.008189259096980095f,-0.015295297838747501f,0.002163778990507126f,0.004502411466091871f,-0.035631511360406876f,0.008401775732636452f,-0.008404103107750416f,-0.0337323397397995f,-0.018907297402620316f,-0.002244790783151984f,0.001681503839790821f,-0.0039899712428450584f,0.006331773940473795f,-0.006330104079097509f,0.012980179861187935f,0.0029715481214225292f,0.004358361475169659f,0.0021893044468015432f,-0.021050838753581047f,0.043137382715940475f,-0.043132271617650986f,-0.0029844820965081453f,0.0011012308532372117f,0.007282780017703772f,-0.002897287253290415f,-0.053342122584581375f,0.0024291968438774347f,-0.0024269968271255493f,0.008866808377206326f,-0.0021495227701961994f,-0.006126459687948227f,-0.0023134832736104727f,-0.04418642446398735f,0.002015806268900633f,-0.002020216081291437f,0.016062596812844276f,0.007203986868262291f,0.005010843276977539f,-0.005369971506297588f,0.002649034606292844f,-0.038813307881355286f,0.0388140007853508f,-0.0006109193782322109f,-0.006218024529516697f,0.00047018800978548825f,7.756675768177956e-05f,-0.11630992591381073f,0.0028954416047781706f,-0.0028961850330233574f,-0.001393600250594318f,0.006334124598652124f,-0.0034817855339497328f,0.003647393314167857f,0.026192978024482727f,0.004394928924739361f,-0.00439096987247467f,0.018650099635124207f,0.015289206057786942f,0.008365807123482227f,0.006931586656719446f,0.0005972323124296963f,-0.005492451600730419f,0.005496288184076548f,0.0049680317752063274f,-0.0059732431545853615f,-3.86714527849108e-05f,0.006268138065934181f,0.07030647248029709f,0.011881140060722828f,-0.01187986508011818f,-0.03346892073750496f,-0.006537136156111956f,0.0018969406373798847f,-0.0022305413149297237f,0.011125922203063965f,0.004342011176049709f,-0.004341082647442818f,-0.007657689042389393f,0.027844788506627083f,-0.010351906530559063f,0.004628816619515419f,0.0634775161743164f,0.007395466323941946f,-0.007391856051981449f,0.0010680567938834429f,0.02551249973475933f,-0.015923574566841125f,0.0009905379265546799f,0.03379824012517929f,0.02027946151793003f,-0.020273497328162193f,0.014938761480152607f,0.000783539901021868f,-0.0105971684679389f,0.0024522447492927313f,0.06811801344156265f,0.011493201367557049f,-0.011492287740111351f,-0.004552742000669241f,0.0008756577735766768f,0.0019332955125719309f,-0.0030440741684287786f,0.02247045934200287f,-0.02476557157933712f,0.02475924976170063f,-0.008532349951565266f,0.018234211951494217f,0.005693220067769289f,0.0029933007899671793f,-0.04963526129722595f,0.01608145795762539f,-0.01608472317457199f,0.019851207733154297f,0.027156664058566093f,-0.0010951235890388489f,-0.0033029010519385338f,0.002884510438889265f,0.007712182123214006f,-0.007713810075074434f,0.011595924384891987f,-0.03239776939153671f,-0.012321485206484795f,-0.00421080831438303f,0.004288680851459503f,0.004510670900344849f,-0.004513563588261604f,0.005330431275069714f,0.0016321559669449925f,0.005664606112986803f,0.002790694823488593f,-0.04252517223358154f,0.008388443849980831f,-0.008390023373067379f,-0.010506904684007168f,-0.019482439383864403f,-0.0005216663121245801f,-0.005112729500979185f,-0.1078367605805397f,0.001400862936861813f,-0.001400871784426272f,-0.017581241205334663f,0.007222496904432774f,0.0072921146638691425f,-0.0024649088736623526f,-0.06238371878862381f,0.02878047153353691f,-0.028783004730939865f,-0.018611624836921692f,-0.008376063778996468f,-0.0036099024582654238f,-0.0020689796656370163f,-0.0004522089730016887f,0.010076483711600304f,-0.010077283717691898f,-0.002235337859019637f,-0.0017848864663392305f,0.004594889935106039f,0.00010435353760840371f,0.003686828538775444f,0.005469825584441423f,-0.005465084686875343f,0.02620755322277546f,0.015633881092071533f,0.01422911137342453f,0.008011839352548122f,-0.061891138553619385f,0.01195721048861742f,-0.011955978348851204f,-0.018659813329577446f,0.035158731043338776f,0.004370755981653929f,0.019563201814889908f,-0.08756725490093231f,0.006377716548740864f,-0.006375169847160578f,0.025021761655807495f,0.022939594462513924f,-0.00577042531222105f,-0.0016110745491459966f,-0.13523931801319122f,0.008073810487985611f,-0.008073166944086552f,-0.001089696423150599f,-0.012398801743984222f,0.0034693286288529634f,0.005929974373430014f,-0.09225691854953766f,-0.0030221191700547934f,0.0030206318479031324f,0.03511260077357292f,0.019589077681303024f,-0.004280028399080038f,0.00828822422772646f,-0.08357003331184387f,0.010296898894011974f,-0.010297143831849098f,0.010450439527630806f,0.007361463736742735f,-0.014716844074428082f,-0.0014856079360470176f,-0.06597557663917542f,0.00818308349698782f,-0.00817782524973154f,0.008619774132966995f,-0.020735258236527443f,-0.0027487752959132195f,0.005155262537300587f,-0.01925387606024742f,0.004047396592795849f,-0.004053800832480192f,0.019212985411286354f,-0.0006966071669012308f,0.0003905741323251277f,0.0052413130179047585f,-0.02216644398868084f,0.010129746049642563f,-0.010127549059689045f,-0.009646547958254814f,0.007663227152079344f,-0.017105624079704285f,-0.0019853245466947556f,-0.028697015717625618f,0.0056396531872451305f,-0.0056410981342196465f,0.005917270667850971f,-0.01845565438270569f,-0.020677249878644943f,-0.0036581228487193584f,0.17753833532333374f,0.07123785465955734f,-0.07124301046133041f,0.003089337144047022f,-0.02205059491097927f,-0.0096647460013628f,-0.005244298372417688f,0.012042420916259289f,0.018395094200968742f,-0.018394777551293373f,-0.016904093325138092f,0.003369967918843031f,0.007819321937859058f,0.00047276701661758125f,-0.15621894598007202f,0.002938299672678113f,-0.0029388403054326773f,-0.033558011054992676f,0.01937892660498619f,0.01288202777504921f,0.007516401819884777f,-0.006773931439965963f,0.04191407561302185f,-0.04190750792622566f,-0.023589173331856728f,-0.0019491116981953382f,0.007765694055706263f,-0.004226239398121834f,-0.02852686122059822f,0.008827431127429008f,-0.008827022276818752f,0.003946367185562849f,-0.014367835596203804f,0.01008730661123991f,-0.0058592576533555984f,-0.10852258652448654f,0.0016593284672126174f,-0.0016627429286018014f,0.0434565544128418f,-0.015001066029071808f,-0.0055235824547708035f,-0.002109912922605872f,0.061138637363910675f,0.044721897691488266f,-0.04472127556800842f,-0.01426409836858511f,-0.005044872872531414f,0.007120457477867603f,2.384435174462851e-05f,-0.10311690717935562f,0.02098066732287407f,-0.020985599607229233f,-0.023151230067014694f,-0.02644825354218483f,-0.012023204006254673f,-0.007093759253621101f,-0.16968168318271637f,0.008277801796793938f,-0.008278016932308674f,0.0036925459280610085f,-0.01867823861539364f,0.0047639645636081696f,-0.0019600731320679188f,0.033874791115522385f,0.04515126347541809f,-0.04515230283141136f,0.008935605175793171f,-0.0023203373420983553f,0.00540059432387352f,0.0050282832235097885f,0.06492361426353455f,0.014170737937092781f,-0.014170102775096893f,0.02498491480946541f,-0.021161198616027832f,-0.009009284898638725f,-0.004070408642292023f,-0.0368492491543293f,0.004518433939665556f,-0.004517054185271263f,0.02051316574215889f,-0.021495286375284195f,0.0006939771119505167f,-0.017129840329289436f,0.039985112845897675f,0.021321304142475128f,-0.021331878378987312f,0.0018515713745728135f,-0.022755160927772522f,0.007930656895041466f,-0.004560249391943216f,0.12245611846446991f,0.017239848151803017f,-0.0172354057431221f,0.010657049715518951f,-0.0025217595975846052f,0.004743250086903572f,-0.007066894322633743f,-0.03840859234333038f,0.007904138416051865f,-0.007901406846940517f,-0.011240296065807343f,-0.030777527019381523f,-0.0037656640633940697f,0.0014544156147167087f,-0.08730729669332504f,0.05239209905266762f,-0.05238579958677292f,0.009015949442982674f,-0.020538918673992157f,-0.004396618809551001f,-0.0007734055980108678f,0.01764621213078499f,0.0017903228290379047f,-0.0017906116554513574f,0.006232924293726683f,0.002948490437120199f,0.006460239179432392f,0.0003651444858405739f,-0.01708405651152134f,0.0026252816896885633f,-0.0026203657034784555f,-0.013945067301392555f,-0.0343976765871048f,0.006438662763684988f,-0.0036563489120453596f,-0.08599087595939636f,0.011872980743646622f,-0.011874892748892307f,0.0009506898932158947f,-0.009780818596482277f,-0.003565447637811303f,0.007065590936690569f,-0.03841787204146385f,0.009467819705605507f,-0.009467699564993382f,-0.00523463124409318f,-0.02064557746052742f,-0.0021239486522972584f,-0.00017376132018398494f,0.06378181278705597f,0.010411775670945644f,-0.010416517965495586f,-0.004998203366994858f,0.038867928087711334f,0.008880967274308205f,-0.0005760325002484024f,-0.003851733636111021f,-0.028163744136691093f,0.028160179033875465f,-0.017969131469726562f,0.01641741767525673f,0.002886468078941107f,0.04890577495098114f,0.006153319031000137f,0.007930322550237179f,-0.007930968888103962f,0.005918292328715324f,0.02638183906674385f,-0.032713256776332855f,0.019413601607084274f,0.046423982828855515f,0.010184742510318756f,-0.010184583254158497f,-0.02265329472720623f,-0.040123939514160156f,0.0024182708002626896f,-0.008024225011467934f,0.03210454061627388f,0.042185984551906586f,-0.04218220338225365f,-0.025215908885002136f,-0.014485026709735394f,-0.012044559232890606f,-0.003910204395651817f,-0.012988119386136532f,0.008757260628044605f,-0.008756750263273716f,-0.00028822614694945514f,-0.017533835023641586f,0.004016069695353508f,-0.004920788109302521f,-0.016951261088252068f,0.004807457327842712f,-0.004808600526303053f,0.030849456787109375f,0.006205044221132994f,0.008223282173275948f,0.023361455649137497f,0.009737699292600155f,0.0734931081533432f,-0.07348938286304474f,0.00082309142453596f,-0.011956633999943733f,0.00018056829867418855f,-0.001568546867929399f,-0.017311833798885345f,0.011692201718688011f,-0.0116915637627244f,0.020518114790320396f,-0.01927863620221615f,0.002802409930154681f,-0.010140316560864449f,-0.03603360056877136f,0.0020555260125547647f,-0.0020601314026862383f,0.0036725481040775776f,-0.00046459719305858016f,-0.0018310464220121503f,-0.00492055993527174f,0.02595667541027069f,0.009576588869094849f,-0.009586215950548649f,-0.019589535892009735f,-0.027357913553714752f,-0.0043261428363621235f,-0.007542148232460022f,0.05535227805376053f,0.013649621978402138f,-0.013651927001774311f,0.008318942971527576f,-0.027200551703572273f,0.007550652604550123f,-0.0008779804338701069f,-0.009584023617208004f,0.010588129982352257f,-0.010587785392999649f,-0.022428058087825775f,0.014817269518971443f,-0.010802218690514565f,0.00030149638769216835f,-0.012030174024403095f,-0.0240084957331419f,0.02400710992515087f,0.00817620288580656f,-0.0028330408968031406f,-0.007603982463479042f,-0.0019077594624832273f,-0.07094138115644455f,0.004373581614345312f,-0.004372505936771631f,-0.002698456635698676f,-0.008996435441076756f,0.008186552673578262f,-0.0017586727626621723f,-0.07323630899190903f,0.008597321808338165f,-0.008598101325333118f,0.0023811515420675278f,0.019366450607776642f,0.007794181816279888f,-0.0020976124797016382f,-0.017178677022457123f,0.009050780907273293f,-0.009055035188794136f,-0.013597145676612854f,-0.007452666759490967f,0.007958579808473587f,-0.0021835723891854286f,-0.05580813065171242f,0.0035745357163250446f,-0.003573079127818346f,-0.0013129032449796796f,-0.0017794723389670253f,-0.0017378018237650394f,-0.0011586523614823818f,0.0077679152600467205f,0.0053296019323170185f,-0.005327548366039991f,0.015294576063752174f,0.002756982808932662f,-0.0066695138812065125f,-0.0035104267299175262f,0.07373999059200287f,-0.013521765358746052f,0.013527273200452328f,0.0002623569162096828f,0.01769474893808365f,0.00024677449255250394f,0.007906019687652588f,0.02722032181918621f,0.007326758000999689f,-0.0073332167230546474f,0.0009686804260127246f,-0.0027774805203080177f,-0.010395627468824387f,9.05960623640567e-05f,0.019667191430926323f,0.003212980693206191f,-0.0032126230653375387f,0.009856615215539932f,-0.016977204009890556f,-0.008623954840004444f,0.0001626883022254333f,0.05950882285833359f,0.039901215583086014f,-0.03990612179040909f,-0.02216973900794983f,0.009994412772357464f,-0.0031719892285764217f,0.0037316607777029276f,-0.12682877480983734f,0.003481674939393997f,-0.0034828386269509792f,-0.00824244599789381f,-0.00763000687584281f,-0.004798573907464743f,-0.0014163905289024115f,0.04903627559542656f,0.0044889808632433414f,-0.004492174368351698f,-0.02084854617714882f,-0.009128298610448837f,-0.006793434266000986f,-0.0060973744839429855f,0.03992608189582825f,-0.015246917493641376f,0.015249810181558132f,0.008527757599949837f,-0.05405125021934509f,-0.005268735811114311f,-0.012284381315112114f,-0.09381546080112457f,0.00678841769695282f,-0.0067834616638720036f,0.0031088022515177727f,-0.00781365018337965f,0.0038303653709590435f,-0.0042153215035796165f,0.004895785823464394f,0.0038870000280439854f,-0.0038849019911140203f,-0.018731698393821716f,0.088404580950737f,-0.012008240446448326f,-0.0003373882791493088f,-0.18979565799236298f,-0.020633745938539505f,0.020634939894080162f,0.02420201525092125f,0.02583841234445572f,-0.020472537726163864f,-0.0012803432764485478f,-0.09308240562677383f,0.005296522751450539f,-0.005294234957545996f,-0.03026086464524269f,-0.02436158061027527f,0.004779052454978228f,-0.0034493852872401476f,-0.1479422003030777f,0.0027186174411326647f,-0.0027163715567439795f,0.005999187473207712f,-0.006211974658071995f,0.005336891394108534f,-0.0017170142382383347f,-0.05350899323821068f,0.009884573519229889f,-0.009881232865154743f,-0.003659025300294161f,-0.0071417358703911304f,0.0054612611420452595f,-0.005179306026548147f,-0.07695730775594711f,0.012528975494205952f,-0.012527331709861755f,-0.019110316410660744f,-0.02321331761777401f,-0.01132756657898426f,-0.010656466707587242f,-0.09119223058223724f,0.004161420743912458f,-0.004160681739449501f,-0.0051827868446707726f,0.004119937773793936f,-0.004403731785714626f,0.007657153066247702f,0.010090850293636322f,0.10782286524772644f,-0.10782524943351746f,0.01675664819777012f,-0.016358699649572372f,0.005522308871150017f,-0.013316260650753975f,-0.0841173529624939f,0.0053731463849544525f,-0.005371814128011465f,0.014694501645863056f,-0.015689054504036903f,-0.004539262969046831f,-0.0071737924590706825f,-0.07532249391078949f,0.001264168182387948f,-0.0012609491823241115f,0.012389116920530796f,0.05679069086909294f,-0.008469716645777225f,1.0549989383434877e-05f,-0.12084200978279114f,0.0010108398273587227f,-0.0010110742878168821f,0.006232023239135742f,0.018210602924227715f,-0.0012846413301303983f,0.005005700513720512f,0.0866568312048912f,0.008207160048186779f,-0.00820701289921999f,-0.015073380433022976f,0.02693549543619156f,-0.007330459076911211f,-0.001101420377381146f,-0.005496049299836159f,0.002653153846040368f,-0.0026514262426644564f,-0.019224008545279503f,-0.022204311564564705f,0.014120451174676418f,0.004393709357827902f,-0.1493091732263565f,0.008157831616699696f,-0.00816378090530634f,0.013563252985477448f,0.011131659150123596f,-0.0006552130216732621f,0.0016753883101046085f,-0.01103154569864273f,0.01159273274242878f,-0.011595556512475014f,0.007062411401420832f,0.010128272697329521f,-0.015689978376030922f,0.0015614858129993081f,-0.0942242220044136f,0.004597938619554043f,-0.0045968275517225266f,-0.04064597561955452f,-0.016042783856391907f,0.006707275286316872f,0.005027913488447666f,-0.04879944771528244f,0.008509193547070026f,-0.008505011908710003f,-0.02441476285457611f,0.008528393693268299f,-0.011490367352962494f,-0.000706275983247906f,-0.02309119701385498f,0.00548104103654623f,-0.005482536740601063f,-0.015545520931482315f,0.00026522300322540104f,-0.00025279351393692195f,-0.007496536243706942f,-0.024206673726439476f,0.002486602868884802f,-0.002486191689968109f,0.005144230090081692f,0.03736681863665581f,0.009684797376394272f,0.000553680642042309f,-0.06082313135266304f,0.0030118238646537066f,-0.0030166737269610167f,-0.03387031704187393f,-0.010906703770160675f,0.015091168694198132f,-0.021701693534851074f,-0.04596840962767601f,0.0016289649065583944f,-0.0016342749586328864f,-0.028098110109567642f,0.0008133936207741499f,0.013813590630888939f,0.0014383044326677918f,-0.015511359088122845f,0.003214702010154724f,-0.003213242394849658f,-0.01206049881875515f,-0.03437052667140961f,-0.008285779505968094f,0.003979629371315241f,-0.011815212666988373f,-0.010316220112144947f,0.010316924192011356f,0.022213101387023926f,-0.03470158576965332f,0.003260636469349265f,-0.0032718495931476355f,0.0376388244330883f,0.0037411281373351812f,-0.0037382040172815323f,-0.022483941167593002f,-0.012489254586398602f,-0.0024439990520477295f,-0.0002778381749521941f,-0.026635877788066864f,0.0036462354473769665f,-0.003642717143520713f,0.003525842446833849f,-0.027462512254714966f,-0.005986078176647425f,-0.003463693428784609f,-0.0708991065621376f,-0.04986139386892319f,0.049862273037433624f,0.0029036381747573614f,-0.017233150079846382f,0.007073743734508753f,0.0017827084520831704f,-0.029759397730231285f,0.008758360520005226f,-0.008762278594076633f,0.0025140922516584396f,-0.0018775248900055885f,-0.010096781887114048f,-0.0027120134327560663f,0.048041339963674545f,0.00907948985695839f,-0.0090795261785388f,-0.022770557552576065f,-0.00017090924666263163f,-0.008617865853011608f,-0.009652412496507168f,0.03524245694279671f,0.005567737389355898f,-0.005564303603023291f,-0.0017850518925115466f,0.0018907937919721007f,4.624127905117348e-05f,0.00303008034825325f,-0.016534216701984406f,0.014178615994751453f,-0.014173349365592003f,-0.002125342143699527f,0.0162583626806736f,0.0031735412776470184f,0.002657918957993388f,-0.021996352821588516f,0.007922209799289703f,-0.00791728775948286f,-0.06859898567199707f,-0.010201199911534786f,-0.005114526487886906f,-0.011649074032902718f,-0.010061759501695633f,0.08280137181282043f,-0.08281160891056061f,0.010264654643833637f,0.0033521465957164764f,-0.015672877430915833f,-0.007492449134588242f,-0.07574064284563065f,0.013957834802567959f,-0.013962429948151112f,-0.0030994073022156954f,0.004334439989179373f,0.006054754368960857f,-0.009492751210927963f,-0.11436861008405685f,0.0016600522212684155f,-0.0016611749306321144f,0.013650001958012581f,-0.030564134940505028f,0.0020413340535014868f,-0.005222545471042395f,-0.025211142376065254f,0.02970176376402378f,-0.02970024198293686f,-0.011825764551758766f,0.006709936074912548f,-2.5680428734631278e-06f,0.0037914759013801813f,-0.04859713837504387f,0.016677917912602425f,-0.01668098382651806f,-0.018244748935103416f,-0.01419680006802082f,-0.011172289028763771f,-0.006188523955643177f,-0.06162881851196289f,0.0085834339261055f,-0.00858110748231411f,0.0016585845733061433f,-0.01525674294680357f,0.010175430215895176f,0.006871291436254978f,-0.06200786679983139f,0.007043053861707449f,-0.0070404959842562675f,0.013007366098463535f,-0.02152375876903534f,0.009088719263672829f,-0.0012155212461948395f,0.010733316652476788f,0.010019493289291859f,-0.010018840432167053f,0.007894841954112053f,-0.0039086733013391495f,0.0002284615475218743f,-0.0006672220770269632f,-0.026039717718958855f,0.006154119502753019f,-0.006154634524136782f,-0.015532885678112507f,0.0014292150735855103f,0.027117596939206123f,0.00039513426600024104f,-0.23881836235523224f,0.0015641875797882676f,-0.0015621209749951959f,-0.005508203059434891f,-0.025217566639184952f,-0.00234606652520597f,0.003206759924069047f,-0.005200875923037529f,0.008200333453714848f,-0.008194605819880962f,0.01651475578546524f,0.00816124677658081f,-0.006173681002110243f,0.0003836377873085439f,-0.009579814970493317f,0.006056583486497402f,-0.006057803984731436f,-0.02227412350475788f,-0.018474789336323738f,-0.004719400778412819f,-0.008970835246145725f,-0.11076724529266357f,0.0073096235282719135f,-0.007318044081330299f,-0.020690608769655228f,-0.005754539743065834f,0.006079646293073893f,0.0019379394361749291f,-0.08681036531925201f,0.00422586128115654f,-0.004229114390909672f,-0.0327620729804039f,-0.01030939444899559f,0.013978042639791965f,-0.0013308431953191757f,-0.024185607209801674f,0.002655962947756052f,-0.002655856544151902f,-0.026247646659612656f,0.01619785837829113f,0.004322592169046402f,-0.0026254570111632347f,0.040314748883247375f,-0.005946672521531582f,0.005947007331997156f,-0.017806431278586388f,0.008522171527147293f,-0.004476605914533138f,0.0012480728328227997f,-0.03607982397079468f,0.0017005683621391654f,-0.0017002015374600887f,0.004586242139339447f,-0.0007967998390085995f,-0.010145741514861584f,0.002852839417755604f,-0.03354942798614502f,0.0037025255151093006f,-0.003698932472616434f,-0.0119031872600317f,0.009683305397629738f,0.0033439071848988533f,0.0018027418991550803f,0.020681051537394524f,-0.008109156042337418f,0.008108069188892841f,0.015450568869709969f,-0.008226548321545124f,0.006074103061109781f,-0.007737116888165474f,0.016972705721855164f,0.003204536624252796f,-0.0032078525982797146f,0.01063018199056387f,-0.004165075719356537f,0.023554280400276184f,0.008288503624498844f,0.01797243021428585f,0.0027649288531392813f,-0.0027676806785166264f,-0.00593094527721405f,0.0002596284030005336f,-0.00022508378606289625f,0.0026555689983069897f,-0.01711921952664852f,-0.015989448875188828f,0.015988532453775406f,-0.014453071169555187f,0.010034660808742046f,0.009540244936943054f,-0.004653983283787966f,0.06551442295312881f,0.008149774745106697f,-0.008147022686898708f,0.03198697790503502f,0.011620942503213882f,0.008562585338950157f,0.00040462453034706414f,0.034115277230739594f,0.006366906221956015f,-0.006366644520312548f,0.010334881022572517f,-0.038519054651260376f,-0.011970805935561657f,-0.02781609259545803f,0.061900362372398376f,-0.004474384244531393f,0.004476439207792282f,0.0036779369693249464f,0.003027463797479868f,-0.0011041064281016588f,0.0018395186634734273f,0.03297322615981102f,0.0050940485671162605f,-0.0050939167849719524f,0.019815200939774513f,0.010203362442553043f,0.015464943833649158f,-0.0033604647032916546f,-0.009514624252915382f,0.0028232389595359564f,-0.0028222918044775724f,0.0024358101654797792f,0.017281005159020424f,-0.0035895798355340958f,-0.002565407194197178f,-0.018939441069960594f,0.006298556458204985f,-0.006301965098828077f,0.0032309116795659065f,0.008465795777738094f,0.005223031155765057f,-0.0009613525471650064f,-0.008528780192136765f,0.005412181373685598f,-0.0054132407531142235f,-0.00021169449610169977f,-0.0073101413436234f,-0.004349670372903347f,-0.001038846094161272f,-0.010378161445260048f,0.0018681208603084087f,-0.0018684694077819586f,0.00480527663603425f,-0.007497450802475214f,-0.005394510459154844f,-0.00779919046908617f,-0.005258604418486357f,0.006627150811254978f,-0.0066306572407484055f,-0.0022583291865885258f,0.006866964511573315f,-0.007099186535924673f,0.001566706458106637f,-0.05223175138235092f,0.003972557373344898f,-0.003968414850533009f,-0.02357635088264942f,0.0053309230133891106f,0.007763884030282497f,-0.0006669244612567127f,0.002955546835437417f,0.0033458250109106302f,-0.0033483055885881186f,-0.008420346304774284f,-0.007040679454803467f,-0.0034758734982460737f,-0.005485903471708298f,0.015890544280409813f,-0.02034611813724041f,0.020344611257314682f,-0.012167515233159065f,0.0037427248898893595f,-0.006786256097257137f,0.003750505158677697f,-0.023831848055124283f,0.002225391799584031f,-0.0022248949389904737f,0.011273659765720367f,0.006506059318780899f,0.007207121700048447f,0.0017508674645796418f,-0.062352657318115234f,0.005717143416404724f,-0.005718518048524857f,-0.018001941964030266f,0.013121144846081734f,-0.0057541197165846825f,-0.004350834526121616f,-0.00274782907217741f,-0.02398994378745556f,0.02398665063083172f,-0.0028912723064422607f,0.015752719715237617f,0.003818156663328409f,0.002568083116784692f,0.017834464088082314f,0.007632730528712273f,-0.007631226908415556f,-0.0080938208848238f,0.017930887639522552f,-0.023558715358376503f,0.0011593285016715527f,0.04200661927461624f,0.006248946767300367f,-0.006246226839721203f,0.004752063192427158f,0.0020485951099544764f,-0.00874236784875393f,-0.0027011348865926266f,-0.026167307049036026f,-0.007985570468008518f,0.007988016121089458f,-0.022754117846488953f,0.019186746329069138f,0.0035693570971488953f,0.0033525172621011734f,-0.04413581266999245f,0.008086384274065495f,-0.008085070177912712f,-0.0008808770217001438f,-0.005809509661048651f,-0.010492808185517788f,-0.005823985673487186f,-0.0410822331905365f,0.005569046828895807f,-0.005563165061175823f,-0.019800959154963493f,-0.024353759363293648f,-0.014895318076014519f,0.000994291272945702f,0.013992021791636944f,0.011995531618595123f,-0.01199592649936676f,-0.004810071084648371f,-0.008527918718755245f,-0.009787710383534431f,-0.003689561039209366f,0.04438347741961479f,0.009213323704898357f,-0.009211122989654541f,-0.015583195723593235f,-0.002990745473653078f,0.006003012880682945f,-0.0021647829562425613f,-0.08356118947267532f,0.002419827040284872f,-0.0024212724529206753f,0.012736593373119831f,-0.012195993214845657f,-0.006561511196196079f,-0.0006473920657299459f,-0.035549942404031754f,0.010380478575825691f,-0.010372181423008442f,-0.017419543117284775f,-0.009772430174052715f,-0.0029039422515779734f,-0.002088612411171198f,0.005854432471096516f,0.005692129954695702f,-0.005687708500772715f,0.0007925152312964201f,-0.0024135576095432043f,-0.009747237898409367f,-0.0032593347132205963f,-0.014909211546182632f,0.001262374804355204f,-0.0012684878893196583f,-0.010774143971502781f,0.0010187721345573664f,0.007554410956799984f,-0.009776988998055458f,0.06343615055084229f,-0.004646647721529007f,0.004658434074372053f,0.029338719323277473f,-0.012899765744805336f,-0.0039020006079226732f,0.0076522366143763065f,0.01608511619269848f,0.011719356290996075f,-0.011719027534127235f,0.005263148806989193f,-0.030923323705792427f,0.0030250069685280323f,-0.005684752482920885f,0.008153320290148258f,0.00797375850379467f,-0.007975311949849129f,-0.001446091802790761f,0.011333221569657326f,0.0034495550207793713f,0.0014860567171126604f,0.057509638369083405f,-0.018054278567433357f,0.018042899668216705f,-0.011614612303674221f,0.007260726299136877f,-0.0011781329521909356f,0.0030844295397400856f,-0.01272227056324482f,0.006052891258150339f,-0.006056923419237137f,0.022736357524991035f,0.001265855971723795f,0.0005209243972785771f,0.001131013035774231f,-0.07933981716632843f,0.002338763792067766f,-0.002335898345336318f,0.027571603655815125f,0.009692585095763206f,0.01534726656973362f,-0.006243417505174875f,-0.05188891291618347f,0.003070299280807376f,-0.0030647399835288525f,0.00020923007105011493f,-0.012012118473649025f,0.010347125120460987f,-0.007393110543489456f,0.030690660700201988f,0.010691334493458271f,-0.010687647387385368f,0.008181382901966572f,-0.008760146796703339f,-0.002505949232727289f,-0.007799670100212097f,0.029621947556734085f,0.005380931310355663f,-0.005380763206630945f,0.009786846116185188f,-0.011579644866287708f,-0.010900460183620453f,-0.01587412692606449f,-0.01217547245323658f,-0.024085087701678276f,0.024084685370326042f,0.0021314246114343405f,-0.005469230003654957f,0.0017458698712289333f,0.0009400804410688579f,0.015813728794455528f,0.0031666127033531666f,-0.003168008290231228f,-0.01627929136157036f,-0.0015519161242991686f,0.006205952260643244f,0.001662467373535037f,-0.04831720143556595f,0.002391096903011203f,-0.0023875758051872253f,0.0006516005378216505f,0.002232499420642853f,0.0019127094419673085f,-0.003126196563243866f,-0.035502564162015915f,-0.004128169734030962f,0.004128608852624893f,0.008077586069703102f,0.01592775247991085f,0.007028455846011639f,0.007236015517264605f,-0.05193950608372688f,0.007474314421415329f,-0.007477855775505304f,-0.0026939446106553078f,-0.02037915028631687f,-0.0004846569208893925f,0.0030724266543984413f,-0.04536891728639603f,0.00908656045794487f,-0.009085670113563538f,0.021496588364243507f,0.051112283021211624f,0.010860892944037914f,0.012346197851002216f,0.0059168897569179535f,-0.039711032062768936f,0.03970286622643471f,-0.0033120308071374893f,0.017661578953266144f,-0.02141028456389904f,0.01936100795865059f,-0.042532630264759064f,0.003109808312729001f,-0.003108772449195385f,-0.000836245424579829f,0.006185851059854031f,0.0008808476850390434f,0.01108727976679802f,-0.021008599549531937f,0.00749353738501668f,-0.007489472161978483f,0.018626756966114044f,-0.0020887909922748804f,0.010558544658124447f,0.0026229319628328085f,-0.06535238772630692f,0.0012557427398860455f,-0.0012466820189729333f,-0.01372847892343998f,-0.005415000487118959f,-0.0059143658727407455f,-0.005033858120441437f,0.0398515947163105f,0.009724784642457962f,-0.009725767187774181f,0.03265002742409706f,0.002523665316402912f,0.009955915622413158f,-0.0027472195215523243f,-0.007901621982455254f,0.004459918476641178f,-0.004455135203897953f,-0.005943184718489647f,-0.01697438210248947f,0.006824225187301636f,0.011713721789419651f,-0.015887659043073654f,0.029005639255046844f,-0.02900136634707451f,-0.01846403256058693f,-0.005335792899131775f,0.008703951723873615f,-0.006127504166215658f,-0.020597990602254868f,0.006122120656073093f,-0.00611919816583395f,-0.016475820913910866f,-0.01699250563979149f,-0.009458490647375584f,-0.009155426174402237f,0.001110403100028634f,0.001109361881390214f,-0.0011136500397697091f,-0.029429934918880463f,0.002169084968045354f,-0.009491193108260632f,0.011138961650431156f,0.08330808579921722f,-0.03238553926348686f,0.03238452598452568f,0.025574563071131706f,-0.023830486461520195f,-0.005935410037636757f,-0.003109648823738098f,0.02235635556280613f,0.009150194004178047f,-0.009143130853772163f,-0.005130423232913017f,0.0006389255286194384f,-0.010442378930747509f,-0.0009249529684893787f,0.017128558829426765f,0.008550495840609074f,-0.008550644852221012f,0.02243310958147049f,0.006773964036256075f,-0.0030357653740793467f,-0.008051177486777306f,0.05839814990758896f,-0.03258301690220833f,0.032582174986600876f,-0.021052882075309753f,0.0015592457493767142f,-0.0026683479081839323f,-0.0033433744683861732f,-0.11931674182415009f,0.006501327734440565f,-0.006501106079667807f,0.0357668399810791f,-0.013157694600522518f,0.010909016244113445f,-0.0016428804956376553f,0.0214256402105093f,0.008972803130745888f,-0.008973781019449234f,0.0017775610322132707f,0.001441279542632401f,0.005169426091015339f,0.0019585732370615005f,0.001431816490367055f,0.019249921664595604f,-0.01925285905599594f,0.009157081134617329f,-0.003240357618778944f,0.01221898477524519f,-0.0018630264094099402f,-0.012174300849437714f,0.004417790565639734f,-0.004417202901095152f,0.004079472739249468f,-0.003278815420344472f,-0.006375465542078018f,-0.0017146021127700806f,0.04470997676253319f,0.0049995663575828075f,-0.004999925848096609f,0.032276224344968796f,0.004001982510089874f,0.012491288594901562f,0.006280842702835798f,0.05692522972822189f,-0.009559976868331432f,0.00955904833972454f,-0.010820980183780193f,0.0036012709606438875f,0.0034475144930183887f,0.004753727465867996f,-0.00466996431350708f,0.006081921048462391f,-0.006085231434553862f,-0.01157767791301012f,0.024128852412104607f,-0.006680960301309824f,-0.0012165468651801348f,-0.019284171983599663f,0.0037889964878559113f,-0.003789545502513647f,0.025664154440164566f,0.003102793823927641f,-0.005269035696983337f,-0.012321569956839085f,-0.018078356981277466f,-0.0210642721503973f,0.021069256588816643f,-0.0002369763533351943f,-0.006699443329125643f,-0.004959035199135542f,-0.00017571293574292213f,-0.02625291235744953f,0.004254716448485851f,-0.004257110878825188f,-0.003727119183167815f,0.007840119302272797f,-0.002898535458371043f,-0.003122418187558651f,0.02256583794951439f,0.0032520154491066933f,-0.0032511295285075903f,0.004832432605326176f,0.010238959453999996f,-0.009756000712513924f,0.0029532290063798428f,0.005186519119888544f,-0.039951391518116f,0.03994474187493324f,-0.0010286179604008794f,-0.01801428012549877f,-0.012956502847373486f,-0.0036911587230861187f,-0.06783623993396759f,0.004574634600430727f,-0.0045734248124063015f,-0.008094800636172295f,-0.0050398120656609535f,-0.0017647905042394996f,-0.005630853585898876f,0.034399982541799545f,0.0038308228831738234f,-0.0038269227370619774f,-0.007555176969617605f,0.03955761343240738f,0.010926040820777416f,0.007075620349496603f,-0.11110822856426239f,0.020526478067040443f,-0.02053038217127323f,0.002376033691689372f,0.003967395052313805f,0.024080559611320496f,0.004298306070268154f,-0.033257897943258286f,0.0011440946254879236f,-0.0011474837083369493f,-0.004565802868455648f,0.0023609150666743517f,0.00483803590759635f,-0.005790242925286293f,-0.056968316435813904f,0.000890931289177388f,-0.0008922351407818496f,0.006280821748077869f,0.013137285597622395f,-0.0034442900214344263f,-0.005046921316534281f,-0.04777504503726959f,-0.001970825716853142f,0.0019703120924532413f,0.01429943647235632f,0.005710263270884752f,0.01973823271691799f,-0.00676288828253746f,0.00763461971655488f,0.00547840166836977f,-0.00547970412299037f,0.023478100076317787f,-0.008900550194084644f,-0.0011839836370199919f,-0.0028019449673593044f,-0.005255657248198986f,0.0030779461376369f,-0.0030834758654236794f,0.03050961159169674f,0.004799354821443558f,0.00648413086310029f,0.005823280196636915f,0.02718600258231163f,0.06647536903619766f,-0.06647319346666336f,0.01212996058166027f,-0.004363188520073891f,0.006921770051121712f,-0.004776964895427227f,-0.08055425435304642f,0.0010264706797897816f,-0.0010260925628244877f,0.0013427069643512368f,0.014916107058525085f,0.016537418588995934f,-0.0040303971618413925f,-0.021070457994937897f,0.0004970669979229569f,-0.0004912823787890375f,-0.02853924036026001f,0.02809278666973114f,-0.00182171945925802f,-0.003634893801063299f,-0.027218006551265717f,0.00111603201366961f,-0.0011138428235426545f,0.014470953494310379f,-0.006008049938827753f,-0.0118937399238348f,0.0021019368432462215f,0.003811752423644066f,0.006192425265908241f,-0.006192174274474382f,0.003893299726769328f,0.005677972454577684f,0.0027534954715520144f,-0.00395619310438633f,-0.03595408797264099f,0.0023027912247925997f,-0.0023030652664601803f,-0.011244075372815132f,-0.016868796199560165f,-0.000698155490681529f,-0.003459393512457609f,-0.08867659419775009f,-0.009805994108319283f,0.00980609655380249f,-0.005002375692129135f,-0.005408775061368942f,0.00763564370572567f,0.00028349066269584f,0.056616660207509995f,0.004538001026958227f,-0.004542665556073189f,0.0010167869040742517f,0.008358419872820377f,0.0055721416138112545f,-0.002616974525153637f,-0.009964262135326862f,0.0036012709606438875f,-0.0036026290617883205f,-0.011892416514456272f,0.006584780756384134f,0.0010486714309081435f,0.00622396357357502f,-0.004637094214558601f,-0.0002064585714833811f,0.00020527357992250472f,-0.012305092066526413f,-0.014425980858504772f,-0.005776883568614721f,-0.0012603403301909566f,0.012698999606072903f,0.0024772207252681255f,-0.0024732511956244707f,0.02344687469303608f,-0.01560164149850607f,-0.006534100975841284f,-0.004623014945536852f,0.014873676933348179f,0.0021833882201462984f,-0.00218461686745286f,-0.004607805050909519f,0.024837734177708626f,-0.006918829865753651f,0.011716428212821484f,0.03354243189096451f,-0.01996748521924019f,0.019968535751104355f,0.009809678420424461f,-0.0018623479409143329f,0.020688192918896675f,-0.005982361733913422f,-0.044905345886945724f,0.0020323924254626036f,-0.0020290084648877382f,0.02390466444194317f,0.00445098802447319f,0.0009487529750913382f,0.007295764982700348f,0.016024362295866013f,0.0029941783286631107f,-0.0029986961744725704f,0.014674477279186249f,-0.008744369260966778f,-0.0058782403357326984f,-0.0040016379207372665f,0.023091940209269524f,-0.036258697509765625f,0.03625994175672531f,0.0175542701035738f,0.013304795138537884f,-0.0016976289916783571f,0.005810255650430918f,-0.009398368187248707f,0.004032514523714781f,-0.0040294816717505455f,0.042471691966056824f,0.007538433186709881f,-0.00793266948312521f,7.876376184867695e-05f,-0.02202487736940384f,0.0037690536119043827f,-0.0037665029522031546f,-0.002081215614452958f,-0.005336428061127663f,-0.00044324598275125027f,0.003711191937327385f,-0.004740693140774965f,0.0037271485198289156f,-0.003729803254827857f,0.009651429951190948f,-0.019956201314926147f,0.0007831814000383019f,-0.000529297161847353f,0.094884492456913f,0.008744409307837486f,-0.008743515238165855f,-0.025264395400881767f,0.018161196261644363f,-0.003212702926248312f,0.0016652429476380348f,0.045244716107845306f,0.007775688543915749f,-0.007781817577779293f,-0.007603743579238653f,-0.0012721391394734383f,-0.00598186207935214f,0.004233324434608221f,0.08756069093942642f,-0.019188281148672104f,0.019190095365047455f,0.0003451156953815371f,0.012128012254834175f,-0.005843326449394226f,0.0014558621915057302f,-0.08373377472162247f,0.006692101247608662f,-0.00669286260381341f,0.023006955161690712f,0.01954982616007328f,-0.0029779537580907345f,-0.00024092374951578677f,-0.0010740876896306872f,0.004745888523757458f,-0.0047404286451637745f,0.012505083344876766f,0.011990628205239773f,-0.0007576605421490967f,0.0017116721719503403f,0.00011264199565630406f,0.08924666792154312f,-0.08924739807844162f,-0.014413547702133656f,0.0001595597277628258f,0.00380306807346642f,-0.003368217730894685f,-0.001908555394038558f,0.007340682204812765f,-0.007340336684137583f,0.027714408934116364f,0.0027106693014502525f,-0.015288680791854858f,-0.009242554195225239f,-0.016852447763085365f,0.0014147228794172406f,-0.0014118632534518838f,0.014839695766568184f,-0.00011087993334513158f,0.0019417203729972243f,-0.0012836487730965018f,-0.026280859485268593f,-0.0003619433264248073f,0.0003632469743024558f,0.007477269507944584f,-0.008759613148868084f,-0.0013499376364052296f,0.0030109556391835213f,-0.011405166238546371f,0.004842594265937805f,-0.004839891567826271f,-0.013060081750154495f,0.0028889626264572144f,-0.008522594347596169f,-0.0034004012122750282f,-0.009547990746796131f,0.00463775172829628f,-0.004637644626200199f,0.0004863023932557553f,0.006113444454967976f,0.005227482412010431f,-0.0019468676764518023f,0.005735758692026138f,-0.02428276836872101f,0.02428988367319107f,0.002960165264084935f,0.010810327716171741f,-0.0009435423999093473f,0.00421610614284873f,-0.038977302610874176f,0.0034296438097953796f,-0.003424694761633873f,-0.015225181356072426f,0.01425179373472929f,-0.0026865829713642597f,0.0007176407962106168f,0.003876158967614174f,0.003663260256871581f,-0.0036640388425439596f,-0.021498410031199455f,0.004187822807580233f,-0.014776665717363358f,0.0010373983532190323f,-0.11869607120752335f,0.012419825419783592f,-0.012417551130056381f,-0.0011899943929165602f,0.004339470993727446f,-0.0006803400465287268f,0.003480435349047184f,-0.02395743690431118f,0.00808992050588131f,-0.008089675568044186f,0.008167224936187267f,-0.005547866225242615f,0.002228386467322707f,-0.0013453952269628644f,-0.00913330353796482f,0.005779937840998173f,-0.005776068661361933f,-0.022701652720570564f,-9.166638483293355e-05f,0.001317754271440208f,-0.006766611710190773f,-0.04853195324540138f,0.0074012367986142635f,-0.007407202385365963f,-0.004714202135801315f,-0.003956819884479046f,0.0044477880001068115f,-0.00406548660248518f,-0.0025331994984298944f,0.004150933586061001f,-0.0041511571034789085f,-0.010409578680992126f,-0.0203467458486557f,0.0014518670504912734f,-0.0037023441400378942f,-0.009225389920175076f,0.0027769694570451975f,-0.002776917302981019f,-0.013839836232364178f,-0.019819149747490883f,-0.007842390798032284f,-0.007380946539342403f,-0.022819073870778084f,-0.025576520711183548f,0.025576135143637657f,0.007729832548648119f,-0.009410196915268898f,0.005967691540718079f,-0.0009618038311600685f,-0.0037569052074104548f,0.004012647550553083f,-0.004010363947600126f,-0.004448064602911472f,0.0051148575730621815f,-0.013411843217909336f,-0.0005731703713536263f,0.08451895415782928f,0.003413464641198516f,-0.0034122997894883156f};
void initclmemobjects()
{
}

void cnn(float *in_0, float *out_0)
{
  float* _buffer = (float*) calloc(153600+(16-1), sizeof(float));
  float* buffer = (float*)(((uintptr_t)_buffer+(16-1)) & ~(uintptr_t)(16-1));
  INTERNAL_CNN_STOPWATCH("OpInput (input_1)")
  {
    // OpInput
    const int H = 120; const int W = 160; const int C = 3;
    const float* in = (in_0 + 0); float* out = (buffer + 58448);
    
    COPY(out, in, H * W * C * sizeof(float), 0);
  }
  INTERNAL_CNN_STOPWATCH("OpPadding (separable_conv2d_internal_0)")
  {
    // OpPadding
    const int H = 120; const int W = 160; const int W_OUT = 161; const int C = 3;
    const int PT = 0; const int PB = 1; const int PL = 0; const int PR = 1;
    const float* in = (buffer + 58448); float* out = (buffer + 0);
    const float* buffer = separable_conv2d_internal_0_VALUES;
    
    float* out_ptr = out;
    
    // PT
    for (int i = 0; i < PT * W_OUT; i++)
    {
        for (int c = 0; c < C; c++, out_ptr++)
        {
            *out_ptr = buffer[c];
        }
    }
    // rows
    for (int h = 0; h < H; h++)
    {
        // PL
        for (int i = 0; i < PL; i++)
        {
            for (int c = 0; c < C; c++, out_ptr++)
            {
                *out_ptr = buffer[c];
            }
        }
        
        // row
        COPY(out_ptr, &in[LINEAR_3(h, 0, 0, W, C)], W * C * sizeof(float), 0);
        out_ptr += W * C;
        
        // PR
        for (int i = 0; i < PR; i++)
        {
            for (int c = 0; c < C; c++, out_ptr++)
            {
                *out_ptr = buffer[c];
            }
        }
    }
    // PB
    for (int i = 0; i < PB * W_OUT; i++)
    {
        for (int c = 0; c < C; c++, out_ptr++)
        {
            *out_ptr = buffer[c];
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpDepthwiseConvolution2D (separable_conv2d_internal_1)")
  {
    memset(buffer + 76800, 0, 57600 * sizeof(float));
    // OpDepthwiseConvolution2D
    const int H = 121; const int W = 161; const int C_IN = 3; const int C_OUT = 12; const int W_OUT = 80;
    const int SH = 2; const int SW = 2;
    const int KH = 3; const int KW = 3;
    const int DEPTH_MULTIPLIER = 4;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 76800); const float* weights_ = separable_conv2d_internal_1_W;
    
    for (int ix = 0; ix < H - KH + 1; ix += SH)
    {
        int x_out_1 = ix / SH;
        for (int jx = 0; jx < W - KW + 1; jx += SW)
        {
            int x_out_2 = jx / SW;
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    // TODO fix constraints on depth multiplier (loading of x_in)
                    int c_increment = DEPTH_MULTIPLIER % 4 == 0 ? 1 : 4 / DEPTH_MULTIPLIER;
                    int m_increment = 4 / c_increment;
                    for (int c = 0; c < C_IN; c += c_increment)
                    {
                        __m128 x_in;
                        if (c_increment == 1)
                        {
                            x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, c, W, C_IN)]);
                        }
                        else if (c_increment == 2)
                        {
                            float x0 = in_[LINEAR_3(x_1, x_2, c, W, C_IN)];
                            float x1 = in_[LINEAR_3(x_1, x_2, c+1, W, C_IN)];
                            x_in = _mm_setr_ps(x0, x0, x1, x1);
                        }
                        else if (c_increment == 4)
                        {
                            x_in = _mm_load_ps(&in_[LINEAR_3(x_1, x_2, c, W, C_IN)]);
                        }
                        for (int m = 0; m < DEPTH_MULTIPLIER; m += m_increment)
                        {
                            int c_out = c * DEPTH_MULTIPLIER + m;
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, c, m, KW, C_IN, DEPTH_MULTIPLIER)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_[LINEAR_3(x_out_1, x_out_2, c_out, W_OUT, C_OUT)]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_[LINEAR_3(x_out_1, x_out_2, c_out, W_OUT, C_OUT)], x_out);
                        }
                    }
                }
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpConvolution2D (separable_conv2d_internal_2)")
  {
    memset(buffer + 0, 0, 76800 * sizeof(float));
    // OpConvolution2D
    const int W = 80; const int C_IN = 12; const int C_OUT = 16; const int H_OUT = 60; const int W_OUT = 80;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 76800); float* out_ = (buffer + 0); const float* weights_ = separable_conv2d_internal_2_W;
    
    for (int x_out_1 = 0; x_out_1 < H_OUT; x_out_1++)
    {
        int ix = x_out_1 * SH;
        for (int x_out_2 = 0; x_out_2 < W_OUT; x_out_2++)
        {
            int jx = x_out_2 * SW;
            alignas(16) float out_tmp[C_OUT];
            COPY(&out_tmp, &out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], C_OUT * sizeof(float), 0);
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    for (int kw = 0; kw < C_IN; kw++)
                    {
                        __m128 x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, kw, W, C_IN)]);
                        for (int lw = 0; lw < C_OUT; lw += 4)
                        {
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, kw, lw, KW, C_IN, C_OUT)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_tmp[lw]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_tmp[lw], x_out);
                        }
                    }
                }
            }
            COPY(&out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], &out_tmp, C_OUT * sizeof(float), 0);
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpArithmetic<ADD> (batch_normalization)")
  {
    // OpArithmetic<ADD>
    const int H = 60; const int W = 80; const int C = 16;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 76800);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    element = _mm_add_ps(element, _mm_load_ps(&batch_normalization_A[c]));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpLeakyReLU (leaky_re_lu)")
  {
    // OpLeakyReLU
    const int H = 60; const int W = 80; const int C = 16;
    const float* in_ = (buffer + 76800); float* out_ = (buffer + 0);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    __m128 alpha = _mm_set1_ps(0.10000000149011612f);
                    element = _mm_max_ps(element, _mm_mul_ps(element, alpha));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpConvolution2D (conv2d_internal_1)")
  {
    memset(buffer + 76800, 0, 19200 * sizeof(float));
    // OpConvolution2D
    const int W = 80; const int C_IN = 16; const int C_OUT = 4; const int H_OUT = 60; const int W_OUT = 80;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 76800); const float* weights_ = conv2d_internal_1_W;
    
    for (int x_out_1 = 0; x_out_1 < H_OUT; x_out_1++)
    {
        int ix = x_out_1 * SH;
        for (int x_out_2 = 0; x_out_2 < W_OUT; x_out_2++)
        {
            int jx = x_out_2 * SW;
            alignas(16) float out_tmp[C_OUT];
            COPY(&out_tmp, &out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], C_OUT * sizeof(float), 0);
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    for (int kw = 0; kw < C_IN; kw++)
                    {
                        __m128 x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, kw, W, C_IN)]);
                        for (int lw = 0; lw < C_OUT; lw += 4)
                        {
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, kw, lw, KW, C_IN, C_OUT)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_tmp[lw]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_tmp[lw], x_out);
                        }
                    }
                }
            }
            COPY(&out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], &out_tmp, C_OUT * sizeof(float), 0);
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpArithmetic<ADD> (batch_normalization_1)")
  {
    // OpArithmetic<ADD>
    const int H = 60; const int W = 80; const int C = 4;
    const float* in_ = (buffer + 76800); float* out_ = (buffer + 19200);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    element = _mm_add_ps(element, _mm_load_ps(&batch_normalization_1_A[c]));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpLeakyReLU (leaky_re_lu_1)")
  {
    // OpLeakyReLU
    const int H = 60; const int W = 80; const int C = 4;
    const float* in_ = (buffer + 19200); float* out_ = (buffer + 0);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    __m128 alpha = _mm_set1_ps(0.10000000149011612f);
                    element = _mm_max_ps(element, _mm_mul_ps(element, alpha));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpPadding (separable_conv2d_1_internal_0)")
  {
    memset(buffer + 19200, 0, 19764 * sizeof(float));
    // OpPadding
    const int H = 60; const int H_OUT = 61; const int W = 80; const int W_OUT = 81; const int C = 4;
    const int PT = 0; const int PL = 0;
    const float* in = (buffer + 0); float* out = (buffer + 19200);
    const float* buffer = separable_conv2d_1_internal_0_VALUES;
    
    for (int h_out = 0; h_out < H_OUT; h_out++)
    {
        int h = h_out - PT;
        for (int w_out = 0; w_out < W_OUT; w_out++)
        {
            int w = w_out - PL;
            for (int c = 0; c < C; c+=4)
            {
                __m128 element = ((0 <= h) && (h < H) && (0 <= w) && (w < W)) ? _mm_load_ps(&in[LINEAR_3(h, w, c, W, C)]) : _mm_load_ps(&buffer[c]);
                _mm_store_ps(&out[LINEAR_3(h_out, w_out, c, W_OUT, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpDepthwiseConvolution2D (separable_conv2d_1_internal_1)")
  {
    memset(buffer + 0, 0, 19200 * sizeof(float));
    // OpDepthwiseConvolution2D
    const int H = 61; const int W = 81; const int C_IN = 4; const int C_OUT = 16; const int W_OUT = 40;
    const int SH = 2; const int SW = 2;
    const int KH = 3; const int KW = 3;
    const int DEPTH_MULTIPLIER = 4;
    const float* in_ = (buffer + 19200); float* out_ = (buffer + 0); const float* weights_ = separable_conv2d_1_internal_1_W;
    
    for (int ix = 0; ix < H - KH + 1; ix += SH)
    {
        int x_out_1 = ix / SH;
        for (int jx = 0; jx < W - KW + 1; jx += SW)
        {
            int x_out_2 = jx / SW;
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    // TODO fix constraints on depth multiplier (loading of x_in)
                    int c_increment = DEPTH_MULTIPLIER % 4 == 0 ? 1 : 4 / DEPTH_MULTIPLIER;
                    int m_increment = 4 / c_increment;
                    for (int c = 0; c < C_IN; c += c_increment)
                    {
                        __m128 x_in;
                        if (c_increment == 1)
                        {
                            x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, c, W, C_IN)]);
                        }
                        else if (c_increment == 2)
                        {
                            float x0 = in_[LINEAR_3(x_1, x_2, c, W, C_IN)];
                            float x1 = in_[LINEAR_3(x_1, x_2, c+1, W, C_IN)];
                            x_in = _mm_setr_ps(x0, x0, x1, x1);
                        }
                        else if (c_increment == 4)
                        {
                            x_in = _mm_load_ps(&in_[LINEAR_3(x_1, x_2, c, W, C_IN)]);
                        }
                        for (int m = 0; m < DEPTH_MULTIPLIER; m += m_increment)
                        {
                            int c_out = c * DEPTH_MULTIPLIER + m;
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, c, m, KW, C_IN, DEPTH_MULTIPLIER)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_[LINEAR_3(x_out_1, x_out_2, c_out, W_OUT, C_OUT)]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_[LINEAR_3(x_out_1, x_out_2, c_out, W_OUT, C_OUT)], x_out);
                        }
                    }
                }
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpConvolution2D (separable_conv2d_1_internal_2)")
  {
    memset(buffer + 19200, 0, 28800 * sizeof(float));
    // OpConvolution2D
    const int W = 40; const int C_IN = 16; const int C_OUT = 24; const int H_OUT = 30; const int W_OUT = 40;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 19200); const float* weights_ = separable_conv2d_1_internal_2_W;
    
    for (int x_out_1 = 0; x_out_1 < H_OUT; x_out_1++)
    {
        int ix = x_out_1 * SH;
        for (int x_out_2 = 0; x_out_2 < W_OUT; x_out_2++)
        {
            int jx = x_out_2 * SW;
            alignas(16) float out_tmp[C_OUT];
            COPY(&out_tmp, &out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], C_OUT * sizeof(float), 0);
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    for (int kw = 0; kw < C_IN; kw++)
                    {
                        __m128 x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, kw, W, C_IN)]);
                        for (int lw = 0; lw < C_OUT; lw += 4)
                        {
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, kw, lw, KW, C_IN, C_OUT)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_tmp[lw]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_tmp[lw], x_out);
                        }
                    }
                }
            }
            COPY(&out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], &out_tmp, C_OUT * sizeof(float), 0);
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpArithmetic<ADD> (batch_normalization_2)")
  {
    // OpArithmetic<ADD>
    const int H = 30; const int W = 40; const int C = 24;
    const float* in_ = (buffer + 19200); float* out_ = (buffer + 48000);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    element = _mm_add_ps(element, _mm_load_ps(&batch_normalization_2_A[c]));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpLeakyReLU (leaky_re_lu_2)")
  {
    // OpLeakyReLU
    const int H = 30; const int W = 40; const int C = 24;
    const float* in_ = (buffer + 48000); float* out_ = (buffer + 76800);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    __m128 alpha = _mm_set1_ps(0.10000000149011612f);
                    element = _mm_max_ps(element, _mm_mul_ps(element, alpha));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpConvolution2D (conv2d_1_internal_1)")
  {
    memset(buffer + 105600, 0, 9600 * sizeof(float));
    // OpConvolution2D
    const int W = 40; const int C_IN = 24; const int C_OUT = 8; const int H_OUT = 30; const int W_OUT = 40;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 76800); float* out_ = (buffer + 105600); const float* weights_ = conv2d_1_internal_1_W;
    
    for (int x_out_1 = 0; x_out_1 < H_OUT; x_out_1++)
    {
        int ix = x_out_1 * SH;
        for (int x_out_2 = 0; x_out_2 < W_OUT; x_out_2++)
        {
            int jx = x_out_2 * SW;
            alignas(16) float out_tmp[C_OUT];
            COPY(&out_tmp, &out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], C_OUT * sizeof(float), 0);
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    for (int kw = 0; kw < C_IN; kw++)
                    {
                        __m128 x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, kw, W, C_IN)]);
                        for (int lw = 0; lw < C_OUT; lw += 4)
                        {
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, kw, lw, KW, C_IN, C_OUT)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_tmp[lw]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_tmp[lw], x_out);
                        }
                    }
                }
            }
            COPY(&out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], &out_tmp, C_OUT * sizeof(float), 0);
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpArithmetic<ADD> (batch_normalization_3)")
  {
    // OpArithmetic<ADD>
    const int H = 30; const int W = 40; const int C = 8;
    const float* in_ = (buffer + 105600); float* out_ = (buffer + 0);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    element = _mm_add_ps(element, _mm_load_ps(&batch_normalization_3_A[c]));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpLeakyReLU (leaky_re_lu_3)")
  {
    // OpLeakyReLU
    const int H = 30; const int W = 40; const int C = 8;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 9600);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    __m128 alpha = _mm_set1_ps(0.10000000149011612f);
                    element = _mm_max_ps(element, _mm_mul_ps(element, alpha));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpPadding (separable_conv2d_2_internal_0)")
  {
    memset(buffer + 134400, 0, 10752 * sizeof(float));
    // OpPadding
    const int H = 30; const int H_OUT = 32; const int W = 40; const int W_OUT = 42; const int C = 8;
    const int PT = 1; const int PL = 1;
    const float* in = (buffer + 9600); float* out = (buffer + 134400);
    const float* buffer = separable_conv2d_2_internal_0_VALUES;
    
    for (int h_out = 0; h_out < H_OUT; h_out++)
    {
        int h = h_out - PT;
        for (int w_out = 0; w_out < W_OUT; w_out++)
        {
            int w = w_out - PL;
            for (int c = 0; c < C; c+=4)
            {
                __m128 element = ((0 <= h) && (h < H) && (0 <= w) && (w < W)) ? _mm_load_ps(&in[LINEAR_3(h, w, c, W, C)]) : _mm_load_ps(&buffer[c]);
                _mm_store_ps(&out[LINEAR_3(h_out, w_out, c, W_OUT, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpDepthwiseConvolution2D (separable_conv2d_2_internal_1)")
  {
    memset(buffer + 96000, 0, 38400 * sizeof(float));
    // OpDepthwiseConvolution2D
    const int H = 32; const int W = 42; const int C_IN = 8; const int C_OUT = 32; const int W_OUT = 40;
    const int SH = 1; const int SW = 1;
    const int KH = 3; const int KW = 3;
    const int DEPTH_MULTIPLIER = 4;
    const float* in_ = (buffer + 134400); float* out_ = (buffer + 96000); const float* weights_ = separable_conv2d_2_internal_1_W;
    
    for (int ix = 0; ix < H - KH + 1; ix += SH)
    {
        int x_out_1 = ix / SH;
        for (int jx = 0; jx < W - KW + 1; jx += SW)
        {
            int x_out_2 = jx / SW;
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    // TODO fix constraints on depth multiplier (loading of x_in)
                    int c_increment = DEPTH_MULTIPLIER % 4 == 0 ? 1 : 4 / DEPTH_MULTIPLIER;
                    int m_increment = 4 / c_increment;
                    for (int c = 0; c < C_IN; c += c_increment)
                    {
                        __m128 x_in;
                        if (c_increment == 1)
                        {
                            x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, c, W, C_IN)]);
                        }
                        else if (c_increment == 2)
                        {
                            float x0 = in_[LINEAR_3(x_1, x_2, c, W, C_IN)];
                            float x1 = in_[LINEAR_3(x_1, x_2, c+1, W, C_IN)];
                            x_in = _mm_setr_ps(x0, x0, x1, x1);
                        }
                        else if (c_increment == 4)
                        {
                            x_in = _mm_load_ps(&in_[LINEAR_3(x_1, x_2, c, W, C_IN)]);
                        }
                        for (int m = 0; m < DEPTH_MULTIPLIER; m += m_increment)
                        {
                            int c_out = c * DEPTH_MULTIPLIER + m;
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, c, m, KW, C_IN, DEPTH_MULTIPLIER)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_[LINEAR_3(x_out_1, x_out_2, c_out, W_OUT, C_OUT)]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_[LINEAR_3(x_out_1, x_out_2, c_out, W_OUT, C_OUT)], x_out);
                        }
                    }
                }
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpConvolution2D (separable_conv2d_2_internal_2)")
  {
    memset(buffer + 57600, 0, 38400 * sizeof(float));
    // OpConvolution2D
    const int W = 40; const int C_IN = 32; const int C_OUT = 32; const int H_OUT = 30; const int W_OUT = 40;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 96000); float* out_ = (buffer + 57600); const float* weights_ = separable_conv2d_2_internal_2_W;
    
    for (int x_out_1 = 0; x_out_1 < H_OUT; x_out_1++)
    {
        int ix = x_out_1 * SH;
        for (int x_out_2 = 0; x_out_2 < W_OUT; x_out_2++)
        {
            int jx = x_out_2 * SW;
            alignas(16) float out_tmp[C_OUT];
            COPY(&out_tmp, &out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], C_OUT * sizeof(float), 0);
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    for (int kw = 0; kw < C_IN; kw++)
                    {
                        __m128 x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, kw, W, C_IN)]);
                        for (int lw = 0; lw < C_OUT; lw += 4)
                        {
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, kw, lw, KW, C_IN, C_OUT)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_tmp[lw]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_tmp[lw], x_out);
                        }
                    }
                }
            }
            COPY(&out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], &out_tmp, C_OUT * sizeof(float), 0);
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpArithmetic<ADD> (batch_normalization_4)")
  {
    // OpArithmetic<ADD>
    const int H = 30; const int W = 40; const int C = 32;
    const float* in_ = (buffer + 57600); float* out_ = (buffer + 19200);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    element = _mm_add_ps(element, _mm_load_ps(&batch_normalization_4_A[c]));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpLeakyReLU (leaky_re_lu_4)")
  {
    // OpLeakyReLU
    const int H = 30; const int W = 40; const int C = 32;
    const float* in_ = (buffer + 19200); float* out_ = (buffer + 57600);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    __m128 alpha = _mm_set1_ps(0.10000000149011612f);
                    element = _mm_max_ps(element, _mm_mul_ps(element, alpha));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpConvolution2D (conv2d_2_internal_1)")
  {
    memset(buffer + 19200, 0, 9600 * sizeof(float));
    // OpConvolution2D
    const int W = 40; const int C_IN = 32; const int C_OUT = 8; const int H_OUT = 30; const int W_OUT = 40;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 57600); float* out_ = (buffer + 19200); const float* weights_ = conv2d_2_internal_1_W;
    
    for (int x_out_1 = 0; x_out_1 < H_OUT; x_out_1++)
    {
        int ix = x_out_1 * SH;
        for (int x_out_2 = 0; x_out_2 < W_OUT; x_out_2++)
        {
            int jx = x_out_2 * SW;
            alignas(16) float out_tmp[C_OUT];
            COPY(&out_tmp, &out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], C_OUT * sizeof(float), 0);
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    for (int kw = 0; kw < C_IN; kw++)
                    {
                        __m128 x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, kw, W, C_IN)]);
                        for (int lw = 0; lw < C_OUT; lw += 4)
                        {
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, kw, lw, KW, C_IN, C_OUT)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_tmp[lw]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_tmp[lw], x_out);
                        }
                    }
                }
            }
            COPY(&out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], &out_tmp, C_OUT * sizeof(float), 0);
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpArithmetic<ADD> (batch_normalization_5)")
  {
    // OpArithmetic<ADD>
    const int H = 30; const int W = 40; const int C = 8;
    const float* in_ = (buffer + 19200); float* out_ = (buffer + 28800);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    element = _mm_add_ps(element, _mm_load_ps(&batch_normalization_5_A[c]));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpMerge<ADD> (add)")
  {
    // OpMerge<ADD>
    const int H = 30; const int W = 40; const int C = 8;
    const float* in_1 = (buffer + 28800); const float* in_2 = (buffer + 9600); float* out_ = (buffer + 19200);
    const int VLEN = 4;
    
    for (int i = 0; i < H * W * C; i += VLEN)
    {
        __m128 i1 = _mm_load_ps(&in_1[i]);
        __m128 i2 = _mm_load_ps(&in_2[i]);
        __m128 o = _mm_add_ps(i1, i2);
        _mm_store_ps(&out_[i], o);
    }
  }
  INTERNAL_CNN_STOPWATCH("OpLeakyReLU (leaky_re_lu_5)")
  {
    // OpLeakyReLU
    const int H = 30; const int W = 40; const int C = 8;
    const float* in_ = (buffer + 19200); float* out_ = (buffer + 28800);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    __m128 alpha = _mm_set1_ps(0.10000000149011612f);
                    element = _mm_max_ps(element, _mm_mul_ps(element, alpha));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpPadding (separable_conv2d_3_internal_0)")
  {
    memset(buffer + 0, 0, 10168 * sizeof(float));
    // OpPadding
    const int H = 30; const int H_OUT = 31; const int W = 40; const int W_OUT = 41; const int C = 8;
    const int PT = 0; const int PL = 0;
    const float* in = (buffer + 28800); float* out = (buffer + 0);
    const float* buffer = separable_conv2d_3_internal_0_VALUES;
    
    for (int h_out = 0; h_out < H_OUT; h_out++)
    {
        int h = h_out - PT;
        for (int w_out = 0; w_out < W_OUT; w_out++)
        {
            int w = w_out - PL;
            for (int c = 0; c < C; c+=4)
            {
                __m128 element = ((0 <= h) && (h < H) && (0 <= w) && (w < W)) ? _mm_load_ps(&in[LINEAR_3(h, w, c, W, C)]) : _mm_load_ps(&buffer[c]);
                _mm_store_ps(&out[LINEAR_3(h_out, w_out, c, W_OUT, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpDepthwiseConvolution2D (separable_conv2d_3_internal_1)")
  {
    memset(buffer + 10176, 0, 9600 * sizeof(float));
    // OpDepthwiseConvolution2D
    const int H = 31; const int W = 41; const int C_IN = 8; const int C_OUT = 32; const int W_OUT = 20;
    const int SH = 2; const int SW = 2;
    const int KH = 3; const int KW = 3;
    const int DEPTH_MULTIPLIER = 4;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 10176); const float* weights_ = separable_conv2d_3_internal_1_W;
    
    for (int ix = 0; ix < H - KH + 1; ix += SH)
    {
        int x_out_1 = ix / SH;
        for (int jx = 0; jx < W - KW + 1; jx += SW)
        {
            int x_out_2 = jx / SW;
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    // TODO fix constraints on depth multiplier (loading of x_in)
                    int c_increment = DEPTH_MULTIPLIER % 4 == 0 ? 1 : 4 / DEPTH_MULTIPLIER;
                    int m_increment = 4 / c_increment;
                    for (int c = 0; c < C_IN; c += c_increment)
                    {
                        __m128 x_in;
                        if (c_increment == 1)
                        {
                            x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, c, W, C_IN)]);
                        }
                        else if (c_increment == 2)
                        {
                            float x0 = in_[LINEAR_3(x_1, x_2, c, W, C_IN)];
                            float x1 = in_[LINEAR_3(x_1, x_2, c+1, W, C_IN)];
                            x_in = _mm_setr_ps(x0, x0, x1, x1);
                        }
                        else if (c_increment == 4)
                        {
                            x_in = _mm_load_ps(&in_[LINEAR_3(x_1, x_2, c, W, C_IN)]);
                        }
                        for (int m = 0; m < DEPTH_MULTIPLIER; m += m_increment)
                        {
                            int c_out = c * DEPTH_MULTIPLIER + m;
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, c, m, KW, C_IN, DEPTH_MULTIPLIER)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_[LINEAR_3(x_out_1, x_out_2, c_out, W_OUT, C_OUT)]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_[LINEAR_3(x_out_1, x_out_2, c_out, W_OUT, C_OUT)], x_out);
                        }
                    }
                }
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpConvolution2D (separable_conv2d_3_internal_2)")
  {
    memset(buffer + 19776, 0, 7200 * sizeof(float));
    // OpConvolution2D
    const int W = 20; const int C_IN = 32; const int C_OUT = 24; const int H_OUT = 15; const int W_OUT = 20;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 10176); float* out_ = (buffer + 19776); const float* weights_ = separable_conv2d_3_internal_2_W;
    
    for (int x_out_1 = 0; x_out_1 < H_OUT; x_out_1++)
    {
        int ix = x_out_1 * SH;
        for (int x_out_2 = 0; x_out_2 < W_OUT; x_out_2++)
        {
            int jx = x_out_2 * SW;
            alignas(16) float out_tmp[C_OUT];
            COPY(&out_tmp, &out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], C_OUT * sizeof(float), 0);
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    for (int kw = 0; kw < C_IN; kw++)
                    {
                        __m128 x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, kw, W, C_IN)]);
                        for (int lw = 0; lw < C_OUT; lw += 4)
                        {
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, kw, lw, KW, C_IN, C_OUT)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_tmp[lw]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_tmp[lw], x_out);
                        }
                    }
                }
            }
            COPY(&out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], &out_tmp, C_OUT * sizeof(float), 0);
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpArithmetic<ADD> (batch_normalization_6)")
  {
    // OpArithmetic<ADD>
    const int H = 15; const int W = 20; const int C = 24;
    const float* in_ = (buffer + 19776); float* out_ = (buffer + 0);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    element = _mm_add_ps(element, _mm_load_ps(&batch_normalization_6_A[c]));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpLeakyReLU (leaky_re_lu_6)")
  {
    // OpLeakyReLU
    const int H = 15; const int W = 20; const int C = 24;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 7200);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    __m128 alpha = _mm_set1_ps(0.10000000149011612f);
                    element = _mm_max_ps(element, _mm_mul_ps(element, alpha));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpConvolution2D (conv2d_3_internal_1)")
  {
    memset(buffer + 14400, 0, 4800 * sizeof(float));
    // OpConvolution2D
    const int W = 20; const int C_IN = 24; const int C_OUT = 16; const int H_OUT = 15; const int W_OUT = 20;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 7200); float* out_ = (buffer + 14400); const float* weights_ = conv2d_3_internal_1_W;
    
    for (int x_out_1 = 0; x_out_1 < H_OUT; x_out_1++)
    {
        int ix = x_out_1 * SH;
        for (int x_out_2 = 0; x_out_2 < W_OUT; x_out_2++)
        {
            int jx = x_out_2 * SW;
            alignas(16) float out_tmp[C_OUT];
            COPY(&out_tmp, &out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], C_OUT * sizeof(float), 0);
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    for (int kw = 0; kw < C_IN; kw++)
                    {
                        __m128 x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, kw, W, C_IN)]);
                        for (int lw = 0; lw < C_OUT; lw += 4)
                        {
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, kw, lw, KW, C_IN, C_OUT)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_tmp[lw]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_tmp[lw], x_out);
                        }
                    }
                }
            }
            COPY(&out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], &out_tmp, C_OUT * sizeof(float), 0);
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpArithmetic<ADD> (batch_normalization_7)")
  {
    // OpArithmetic<ADD>
    const int H = 15; const int W = 20; const int C = 16;
    const float* in_ = (buffer + 14400); float* out_ = (buffer + 19200);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    element = _mm_add_ps(element, _mm_load_ps(&batch_normalization_7_A[c]));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpLeakyReLU (leaky_re_lu_7)")
  {
    // OpLeakyReLU
    const int H = 15; const int W = 20; const int C = 16;
    const float* in_ = (buffer + 19200); float* out_ = (buffer + 24000);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    __m128 alpha = _mm_set1_ps(0.10000000149011612f);
                    element = _mm_max_ps(element, _mm_mul_ps(element, alpha));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpPadding (separable_conv2d_4_internal_0)")
  {
    memset(buffer + 28800, 0, 5984 * sizeof(float));
    // OpPadding
    const int H = 15; const int H_OUT = 17; const int W = 20; const int W_OUT = 22; const int C = 16;
    const int PT = 1; const int PL = 1;
    const float* in = (buffer + 24000); float* out = (buffer + 28800);
    const float* buffer = separable_conv2d_4_internal_0_VALUES;
    
    for (int h_out = 0; h_out < H_OUT; h_out++)
    {
        int h = h_out - PT;
        for (int w_out = 0; w_out < W_OUT; w_out++)
        {
            int w = w_out - PL;
            for (int c = 0; c < C; c+=4)
            {
                __m128 element = ((0 <= h) && (h < H) && (0 <= w) && (w < W)) ? _mm_load_ps(&in[LINEAR_3(h, w, c, W, C)]) : _mm_load_ps(&buffer[c]);
                _mm_store_ps(&out[LINEAR_3(h_out, w_out, c, W_OUT, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpDepthwiseConvolution2D (separable_conv2d_4_internal_1)")
  {
    memset(buffer + 34784, 0, 19200 * sizeof(float));
    // OpDepthwiseConvolution2D
    const int H = 17; const int W = 22; const int C_IN = 16; const int C_OUT = 64; const int W_OUT = 20;
    const int SH = 1; const int SW = 1;
    const int KH = 3; const int KW = 3;
    const int DEPTH_MULTIPLIER = 4;
    const float* in_ = (buffer + 28800); float* out_ = (buffer + 34784); const float* weights_ = separable_conv2d_4_internal_1_W;
    
    for (int ix = 0; ix < H - KH + 1; ix += SH)
    {
        int x_out_1 = ix / SH;
        for (int jx = 0; jx < W - KW + 1; jx += SW)
        {
            int x_out_2 = jx / SW;
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    // TODO fix constraints on depth multiplier (loading of x_in)
                    int c_increment = DEPTH_MULTIPLIER % 4 == 0 ? 1 : 4 / DEPTH_MULTIPLIER;
                    int m_increment = 4 / c_increment;
                    for (int c = 0; c < C_IN; c += c_increment)
                    {
                        __m128 x_in;
                        if (c_increment == 1)
                        {
                            x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, c, W, C_IN)]);
                        }
                        else if (c_increment == 2)
                        {
                            float x0 = in_[LINEAR_3(x_1, x_2, c, W, C_IN)];
                            float x1 = in_[LINEAR_3(x_1, x_2, c+1, W, C_IN)];
                            x_in = _mm_setr_ps(x0, x0, x1, x1);
                        }
                        else if (c_increment == 4)
                        {
                            x_in = _mm_load_ps(&in_[LINEAR_3(x_1, x_2, c, W, C_IN)]);
                        }
                        for (int m = 0; m < DEPTH_MULTIPLIER; m += m_increment)
                        {
                            int c_out = c * DEPTH_MULTIPLIER + m;
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, c, m, KW, C_IN, DEPTH_MULTIPLIER)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_[LINEAR_3(x_out_1, x_out_2, c_out, W_OUT, C_OUT)]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_[LINEAR_3(x_out_1, x_out_2, c_out, W_OUT, C_OUT)], x_out);
                        }
                    }
                }
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpConvolution2D (separable_conv2d_4_internal_2)")
  {
    memset(buffer + 53984, 0, 9600 * sizeof(float));
    // OpConvolution2D
    const int W = 20; const int C_IN = 64; const int C_OUT = 32; const int H_OUT = 15; const int W_OUT = 20;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 34784); float* out_ = (buffer + 53984); const float* weights_ = separable_conv2d_4_internal_2_W;
    
    for (int x_out_1 = 0; x_out_1 < H_OUT; x_out_1++)
    {
        int ix = x_out_1 * SH;
        for (int x_out_2 = 0; x_out_2 < W_OUT; x_out_2++)
        {
            int jx = x_out_2 * SW;
            alignas(16) float out_tmp[C_OUT];
            COPY(&out_tmp, &out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], C_OUT * sizeof(float), 0);
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    for (int kw = 0; kw < C_IN; kw++)
                    {
                        __m128 x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, kw, W, C_IN)]);
                        for (int lw = 0; lw < C_OUT; lw += 4)
                        {
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, kw, lw, KW, C_IN, C_OUT)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_tmp[lw]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_tmp[lw], x_out);
                        }
                    }
                }
            }
            COPY(&out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], &out_tmp, C_OUT * sizeof(float), 0);
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpArithmetic<ADD> (batch_normalization_8)")
  {
    // OpArithmetic<ADD>
    const int H = 15; const int W = 20; const int C = 32;
    const float* in_ = (buffer + 53984); float* out_ = (buffer + 9600);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    element = _mm_add_ps(element, _mm_load_ps(&batch_normalization_8_A[c]));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpLeakyReLU (leaky_re_lu_8)")
  {
    // OpLeakyReLU
    const int H = 15; const int W = 20; const int C = 32;
    const float* in_ = (buffer + 9600); float* out_ = (buffer + 0);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    __m128 alpha = _mm_set1_ps(0.10000000149011612f);
                    element = _mm_max_ps(element, _mm_mul_ps(element, alpha));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpConvolution2D (conv2d_4_internal_1)")
  {
    memset(buffer + 9600, 0, 4800 * sizeof(float));
    // OpConvolution2D
    const int W = 20; const int C_IN = 32; const int C_OUT = 16; const int H_OUT = 15; const int W_OUT = 20;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 9600); const float* weights_ = conv2d_4_internal_1_W;
    
    for (int x_out_1 = 0; x_out_1 < H_OUT; x_out_1++)
    {
        int ix = x_out_1 * SH;
        for (int x_out_2 = 0; x_out_2 < W_OUT; x_out_2++)
        {
            int jx = x_out_2 * SW;
            alignas(16) float out_tmp[C_OUT];
            COPY(&out_tmp, &out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], C_OUT * sizeof(float), 0);
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    for (int kw = 0; kw < C_IN; kw++)
                    {
                        __m128 x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, kw, W, C_IN)]);
                        for (int lw = 0; lw < C_OUT; lw += 4)
                        {
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, kw, lw, KW, C_IN, C_OUT)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_tmp[lw]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_tmp[lw], x_out);
                        }
                    }
                }
            }
            COPY(&out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], &out_tmp, C_OUT * sizeof(float), 0);
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpArithmetic<ADD> (batch_normalization_9)")
  {
    // OpArithmetic<ADD>
    const int H = 15; const int W = 20; const int C = 16;
    const float* in_ = (buffer + 9600); float* out_ = (buffer + 14400);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    element = _mm_add_ps(element, _mm_load_ps(&batch_normalization_9_A[c]));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpMerge<ADD> (add_1)")
  {
    // OpMerge<ADD>
    const int H = 15; const int W = 20; const int C = 16;
    const float* in_1 = (buffer + 14400); const float* in_2 = (buffer + 24000); float* out_ = (buffer + 28800);
    const int VLEN = 4;
    
    for (int i = 0; i < H * W * C; i += VLEN)
    {
        __m128 i1 = _mm_load_ps(&in_1[i]);
        __m128 i2 = _mm_load_ps(&in_2[i]);
        __m128 o = _mm_add_ps(i1, i2);
        _mm_store_ps(&out_[i], o);
    }
  }
  INTERNAL_CNN_STOPWATCH("OpLeakyReLU (leaky_re_lu_9)")
  {
    // OpLeakyReLU
    const int H = 15; const int W = 20; const int C = 16;
    const float* in_ = (buffer + 28800); float* out_ = (buffer + 5712);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    __m128 alpha = _mm_set1_ps(0.10000000149011612f);
                    element = _mm_max_ps(element, _mm_mul_ps(element, alpha));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpPadding (separable_conv2d_5_internal_0)")
  {
    memset(buffer + 0, 0, 5712 * sizeof(float));
    // OpPadding
    const int H = 15; const int H_OUT = 17; const int W = 20; const int W_OUT = 21; const int C = 16;
    const int PT = 1; const int PL = 0;
    const float* in = (buffer + 5712); float* out = (buffer + 0);
    const float* buffer = separable_conv2d_5_internal_0_VALUES;
    
    for (int h_out = 0; h_out < H_OUT; h_out++)
    {
        int h = h_out - PT;
        for (int w_out = 0; w_out < W_OUT; w_out++)
        {
            int w = w_out - PL;
            for (int c = 0; c < C; c+=4)
            {
                __m128 element = ((0 <= h) && (h < H) && (0 <= w) && (w < W)) ? _mm_load_ps(&in[LINEAR_3(h, w, c, W, C)]) : _mm_load_ps(&buffer[c]);
                _mm_store_ps(&out[LINEAR_3(h_out, w_out, c, W_OUT, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpDepthwiseConvolution2D (separable_conv2d_5_internal_1)")
  {
    memset(buffer + 5712, 0, 5120 * sizeof(float));
    // OpDepthwiseConvolution2D
    const int H = 17; const int W = 21; const int C_IN = 16; const int C_OUT = 64; const int W_OUT = 10;
    const int SH = 2; const int SW = 2;
    const int KH = 3; const int KW = 3;
    const int DEPTH_MULTIPLIER = 4;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 5712); const float* weights_ = separable_conv2d_5_internal_1_W;
    
    for (int ix = 0; ix < H - KH + 1; ix += SH)
    {
        int x_out_1 = ix / SH;
        for (int jx = 0; jx < W - KW + 1; jx += SW)
        {
            int x_out_2 = jx / SW;
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    // TODO fix constraints on depth multiplier (loading of x_in)
                    int c_increment = DEPTH_MULTIPLIER % 4 == 0 ? 1 : 4 / DEPTH_MULTIPLIER;
                    int m_increment = 4 / c_increment;
                    for (int c = 0; c < C_IN; c += c_increment)
                    {
                        __m128 x_in;
                        if (c_increment == 1)
                        {
                            x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, c, W, C_IN)]);
                        }
                        else if (c_increment == 2)
                        {
                            float x0 = in_[LINEAR_3(x_1, x_2, c, W, C_IN)];
                            float x1 = in_[LINEAR_3(x_1, x_2, c+1, W, C_IN)];
                            x_in = _mm_setr_ps(x0, x0, x1, x1);
                        }
                        else if (c_increment == 4)
                        {
                            x_in = _mm_load_ps(&in_[LINEAR_3(x_1, x_2, c, W, C_IN)]);
                        }
                        for (int m = 0; m < DEPTH_MULTIPLIER; m += m_increment)
                        {
                            int c_out = c * DEPTH_MULTIPLIER + m;
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, c, m, KW, C_IN, DEPTH_MULTIPLIER)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_[LINEAR_3(x_out_1, x_out_2, c_out, W_OUT, C_OUT)]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_[LINEAR_3(x_out_1, x_out_2, c_out, W_OUT, C_OUT)], x_out);
                        }
                    }
                }
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpConvolution2D (separable_conv2d_5_internal_2)")
  {
    memset(buffer + 10832, 0, 3200 * sizeof(float));
    // OpConvolution2D
    const int W = 10; const int C_IN = 64; const int C_OUT = 40; const int H_OUT = 8; const int W_OUT = 10;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 5712); float* out_ = (buffer + 10832); const float* weights_ = separable_conv2d_5_internal_2_W;
    
    for (int x_out_1 = 0; x_out_1 < H_OUT; x_out_1++)
    {
        int ix = x_out_1 * SH;
        for (int x_out_2 = 0; x_out_2 < W_OUT; x_out_2++)
        {
            int jx = x_out_2 * SW;
            alignas(16) float out_tmp[C_OUT];
            COPY(&out_tmp, &out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], C_OUT * sizeof(float), 0);
            for (int iw = 0; iw < KH; iw++)
            {
                int x_1 = ix + iw;
                for (int jw = 0; jw < KW; jw++)
                {
                    int x_2 = jx + jw;
                    for (int kw = 0; kw < C_IN; kw++)
                    {
                        __m128 x_in = _mm_load_ps1(&in_[LINEAR_3(x_1, x_2, kw, W, C_IN)]);
                        for (int lw = 0; lw < C_OUT; lw += 4)
                        {
                            __m128 w = _mm_load_ps(&weights_[LINEAR_4(iw, jw, kw, lw, KW, C_IN, C_OUT)]);
                            __m128 y = _mm_mul_ps(x_in, w);
                            __m128 x_out = _mm_load_ps(&out_tmp[lw]);
                            x_out = _mm_add_ps(x_out, y);
                            _mm_store_ps(&out_tmp[lw], x_out);
                        }
                    }
                }
            }
            COPY(&out_[LINEAR_3(x_out_1, x_out_2, 0, W_OUT, C_OUT)], &out_tmp, C_OUT * sizeof(float), 0);
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpArithmetic<ADD> (batch_normalization_10)")
  {
    // OpArithmetic<ADD>
    const int H = 8; const int W = 10; const int C = 40;
    const float* in_ = (buffer + 10832); float* out_ = (buffer + 14032);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    element = _mm_add_ps(element, _mm_load_ps(&batch_normalization_10_A[c]));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpLeakyReLU (leaky_re_lu_10)")
  {
    // OpLeakyReLU
    const int H = 8; const int W = 10; const int C = 40;
    const float* in_ = (buffer + 14032); float* out_ = (buffer + 0);
    
    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            for (int c = 0; c < C; c+= 4)
            {
                __m128 element = _mm_load_ps(&in_[LINEAR_3(h, w, c, W, C)]);
                // inner
                {
                    __m128 alpha = _mm_set1_ps(0.10000000149011612f);
                    element = _mm_max_ps(element, _mm_mul_ps(element, alpha));
                }
                _mm_store_ps(&out_[LINEAR_3(h, w, c, W, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpPadding (DetectionLayer_internal_0)")
  {
    memset(buffer + 3200, 0, 4800 * sizeof(float));
    // OpPadding
    const int H = 8; const int H_OUT = 10; const int W = 10; const int W_OUT = 12; const int C = 40;
    const int PT = 1; const int PL = 1;
    const float* in = (buffer + 0); float* out = (buffer + 3200);
    const float* buffer = DetectionLayer_internal_0_VALUES;
    
    for (int h_out = 0; h_out < H_OUT; h_out++)
    {
        int h = h_out - PT;
        for (int w_out = 0; w_out < W_OUT; w_out++)
        {
            int w = w_out - PL;
            for (int c = 0; c < C; c+=4)
            {
                __m128 element = ((0 <= h) && (h < H) && (0 <= w) && (w < W)) ? _mm_load_ps(&in[LINEAR_3(h, w, c, W, C)]) : _mm_load_ps(&buffer[c]);
                _mm_store_ps(&out[LINEAR_3(h_out, w_out, c, W_OUT, C)], element);
            }
        }
    }
  }
  INTERNAL_CNN_STOPWATCH("OpConvolution2D (DetectionLayer_internal_1)")
  {
    memset(out_0 + 0, 0, 1680 * sizeof(float));
    // OpConvolution2D
    const int H = 10; const int W = 12; const int C_IN = 40; const int C_OUT = 21; const int W_OUT = 10; const int H_OUT = 8;
    const int SH = 1; const int SW = 1;
    const int KH = 3; const int KW = 3;
    const float* in_ = (buffer + 3200); float* out_ = (out_0 + 0); const float* weights_ = DetectionLayer_internal_1_W;
    
    #if HEAP
        float* _patch_buffer = (float*) calloc((H_OUT * W_OUT * KH * KW * C_IN)+(16-1), sizeof(float));
        float* patch_buffer = (float*) (((uintptr_t)_patch_buffer+(16-1)) & ~(uintptr_t)(16-1));
    #else
        alignas(16) float patch_buffer[H_OUT * W_OUT * KH * KW * C_IN];
    #endif
    
    
    
    const int M = H_OUT * W_OUT;
    const int N = C_OUT;
    const int K = KH * KW * C_IN;
    
    #if HEAP
    float* a = patch_buffer;
    #else
    float* a = &patch_buffer[0];
    #endif
    
    const float* b = weights_;
    float* c = out_;
    int lda = K;
    int ldb = N;
    int ldc = N;
    
    IM2ROW(H, W, H_OUT, W_OUT, KH, KW, SH, SW, C_IN, in_, patch_buffer, 0); 
    GEMM(M, N, K, a, lda, b, ldb, c, ldc, 13);
    
    #if HEAP
        free(_patch_buffer);
    #endif
  }
  free(_buffer);
}

#ifdef CNN_TEST

#include <cstdlib>
#include <cstdio>

int main()
{
    printf("START ...\n");
    const int IN_DIM = 57600;
    const int OUT_DIM = 1680;
    const int NUM_RUNS = 1000;
    
    printf("ALLOCATION ...\n");
    
#if HEAP
    float* _in = (float*) calloc(IN_DIM+(16-1), sizeof(float));
    float* in = (float*) (((uintptr_t)_in+(16-1)) & ~(uintptr_t)(16-1));
    float* _out = (float*) calloc(OUT_DIM+(16-1), sizeof(float));
    float* out = (float*) (((uintptr_t)_out+(16-1)) & ~(uintptr_t)(16-1));
#else
    alignas(16) float in[IN_DIM];
    alignas(16) float out[OUT_DIM];
#endif

    // read image
    printf("READ FILE ...\n");
    FILE *f = fopen("img_0.bin", "r");

    if (f == NULL)
    {
        for (size_t i = 0; i < IN_DIM; i++)
        {
            in[i] = rand() % 256;
        }
    }
    else
    {
#if HEAP
        fread(in, sizeof(float), IN_DIM, f);
#else
        fread(&in, sizeof(float), IN_DIM, f);
#endif
    }
    
    printf("RUN ...\n");
    // warm-up run
    cnn(in, out);
    cnn(in, out);
    cnn(in, out);
    
    for (int run = 0; run < NUM_RUNS; run++)
    {
        // reset out
        for (size_t i = 0; i < OUT_DIM; i++)
        {
            out[i] = 0.0f;
        }

        // run function
        CNN_STOPWATCH("__dcg_cnn")
        {
            cnn(in, out);
        }
    }
    
    printf("timings:");
    for (int i = 0; i < NUM_RUNS; i++)
    {
        printf(" %" PRIu64 "", stopwatch_timings["__dcg_cnn"][i]);
    }
    printf("\n");

    for ( auto &pair : stopwatch_timings )
    {
        printf("%s:", pair.first.c_str());
        for (auto const& timing: pair.second)
        {
            printf(" %" PRIu64, timing);
        }
        printf("\n");
    }

    uint64_t total_elapsed = total_time(stopwatch_timings["__dcg_cnn"]);
    
    double sd = 0.0;
    double mv = total_elapsed/NUM_RUNS;
    for( int i = 0; i < NUM_RUNS; i++ )
    {
      sd += (stopwatch_timings["__dcg_cnn"][i] - mv)*(stopwatch_timings["__dcg_cnn"][i] - mv);
    }
    sd /= NUM_RUNS;
    sd = sqrt( sd );
    
    printf("OUT_DIM: %d\n", OUT_DIM);
    printf("values:");
    for (int i = 0; i < OUT_DIM; i++)
    {
        printf(" %f", out[i]);
    }
    printf("\ntime: %" PRIu64 "nano seconds\nruns: %d\naverage: %" PRIu64 " +- %5.f nano seconds/run\n", total_elapsed, NUM_RUNS, total_elapsed/NUM_RUNS, sd);
    printf("\nFINISHED\n");
    
#if HEAP
    free(_in);
    free(_out);
#endif

    return 0;
}

#endif