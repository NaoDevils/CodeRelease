// includes
#include <emmintrin.h>
#include <math.h>

#define YOLO 1
#define HEAP 1

#if YOLO

#include <vector>

// dimensions
const unsigned int input_height = 60;
const unsigned int input_width = 80;
const unsigned int input_channel = 3;
const unsigned int output_height = 4;
const unsigned int output_width = 5;
const unsigned int output_channel = 24;
const unsigned int num_of_boxes = 3;
const unsigned int num_of_classes = 3;
const unsigned int num_of_coords = 4;
const std::vector<float> anchors = {0.76f, 0.64f, 1.40f,1.58f, 2.62f,3.12f};
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
alignas(16) float separable_conv2d_internal_1_W[] = {-0.0884430855512619f,0.07102608680725098f,-0.17189069092273712f,-0.033695291727781296f,0.08333118259906769f,-0.11440744251012802f,0.004617802798748016f,0.03782990202307701f,-0.15786947309970856f,-0.12171530723571777f,0.06655845791101456f,0.048753559589385986f,-0.014749948866665363f,0.12888751924037933f,0.016430532559752464f,-0.12760667502880096f,0.12005562335252762f,-0.18259279429912567f,-0.06519055366516113f,0.018729086965322495f,-0.08880224823951721f,0.11727511882781982f,0.10097263008356094f,0.0926857590675354f,-0.14200665056705475f,0.05769019573926926f,-0.018502965569496155f,-0.13977009057998657f,0.10728660970926285f,-0.08052562177181244f,0.04746608808636665f,0.038992989808321f,-0.13976195454597473f,0.06724026799201965f,0.0694136917591095f,-0.022867795079946518f,0.058877646923065186f,0.11542835831642151f,-0.0029831959400326014f,0.20886440575122833f,-0.003748641349375248f,0.0926402360200882f,-0.09466247260570526f,0.11791213601827621f,0.09715176373720169f,-0.10883981734514236f,0.1251918226480484f,0.06621597707271576f,0.0709710344672203f,0.1524370163679123f,0.14928922057151794f,0.07481847703456879f,0.014551679603755474f,0.07495227456092834f,-0.138861283659935f,0.13635312020778656f,0.1595984697341919f,0.07347511500120163f,0.13734905421733856f,0.11086267232894897f,0.09233977645635605f,0.07244625687599182f,0.0638984739780426f,-0.015632735565304756f,0.10444579273462296f,0.08749856054782867f,0.013290775939822197f,0.05009995028376579f,0.07919761538505554f,0.07351333647966385f,0.016308190301060677f,-0.14335541427135468f,-0.15174245834350586f,-0.12342570722103119f,-0.031661663204431534f,0.05409504100680351f,-0.09937141835689545f,-0.09724762290716171f,0.056624650955200195f,0.03701050579547882f,-0.02428232505917549f,-0.03706446662545204f,0.031499553471803665f,-0.029499337077140808f,-0.06963176280260086f,-0.14028532803058624f,-0.025575101375579834f,0.10932845622301102f,-0.15546347200870514f,0.05674112215638161f,0.030603649094700813f,0.07751846313476562f,0.08281635493040085f,0.06127959117293358f,0.012890827842056751f,-0.06539766490459442f,-0.030670886859297752f,-0.052976809442043304f,0.05444052815437317f,0.024736810475587845f,-0.08825932443141937f,0.11347868293523788f,0.15773728489875793f,0.056672584265470505f,0.07305062562227249f,-0.010836753994226456f,-0.04946190491318703f,-0.08829855173826218f};
alignas(16) float separable_conv2d_internal_2_W[] = {0.20043183863162994f,0.6864765286445618f,0.13260924816131592f,-4.729432106018066f,4.471002101898193f,-1.785215139389038f,0.3688417971134186f,0.4838082194328308f,0.15168781578540802f,0.28535589575767517f,-1.7669641971588135f,-4.164175510406494f,3.187680959701538f,-1.6699525117874146f,1.4714471101760864f,-3.224738359451294f,1.1254935264587402f,-0.976254940032959f,0.8105969429016113f,5.850522518157959f,-1.8210101127624512f,-0.17602503299713135f,0.7750086784362793f,-0.5339586734771729f,0.5426502823829651f,-1.600872278213501f,1.543526530265808f,-2.2756900787353516f,4.920286655426025f,-1.8696857690811157f,-1.8641972541809082f,3.0368778705596924f,0.7180300354957581f,0.33902814984321594f,-0.7343684434890747f,0.3593616485595703f,-1.3996520042419434f,-1.6699165105819702f,1.8385396003723145f,0.5853985548019409f,-1.3332247734069824f,3.249603271484375f,0.005679191555827856f,-3.2172703742980957f,0.6668393611907959f,-1.803523302078247f,-1.3015879392623901f,-0.2805171608924866f,-0.7632545232772827f,0.41095170378685f,-0.6450679898262024f,2.8609459400177f,-2.468933582305908f,2.0939910411834717f,-5.55720329284668f,0.29997923970222473f,-1.891724944114685f,3.3142080307006836f,0.1966904103755951f,-3.5282418727874756f,-0.6429268717765808f,-0.05496234446763992f,0.6642404794692993f,1.1769514083862305f,-3.1879076957702637f,-1.516969084739685f,0.1179373636841774f,-0.6126309633255005f,0.09690447151660919f,-0.4574715495109558f,-0.7217994928359985f,-1.3016822338104248f,-5.415219783782959f,-3.245906352996826f,-0.1024329885840416f,-2.4804019927978516f,3.6473493576049805f,-0.5616130828857422f,0.8350094556808472f,-0.8504964113235474f,0.7775452733039856f,2.2729110717773438f,0.14148670434951782f,-0.2605746388435364f,1.1254359483718872f,0.9711810946464539f,0.8602816462516785f,-1.2278093099594116f,6.63167142868042f,3.1420352458953857f,-1.481830358505249f,-4.357367038726807f,0.0008083522552624345f,-0.10091899335384369f,0.7062768340110779f,1.4190088510513306f,0.8986549973487854f,0.5171427130699158f,-1.8850094079971313f,-0.7930817604064941f,1.343705415725708f,-0.05409996584057808f,2.437490224838257f,-2.897854804992676f,3.558767795562744f,-2.241769790649414f,0.03749435395002365f,1.4309009313583374f,-1.268850326538086f,0.725845217704773f,-1.1819714307785034f,1.7243932485580444f,-2.6218788623809814f,0.009984134696424007f,-2.615048408508301f,0.003418571315705776f,0.1732863038778305f,-1.3725025653839111f,-0.5937944650650024f,-2.499042272567749f,0.21244880557060242f,0.9379751682281494f,-7.184476375579834f,-0.6283828020095825f,-1.4521046876907349f,-1.1501495838165283f,-1.1108392477035522f,-2.998030662536621f,0.6337074637413025f,-1.0792702436447144f,1.1700718402862549f,0.2454129159450531f,2.9427762031555176f,-4.3072590827941895f,-2.393397331237793f,3.2725393772125244f,-3.331996440887451f,3.6120285987854004f,0.9978429079055786f,-2.0912177562713623f,1.864952802658081f,-1.532426357269287f,-1.5724408626556396f,-0.1452275812625885f,0.169851616024971f,0.19116777181625366f,0.014378373511135578f,-1.1418975591659546f,0.7130493521690369f,-1.6481612920761108f,3.6572771072387695f,3.205197334289551f,0.4625907838344574f,0.2078193575143814f,0.720805823802948f,1.1593741178512573f,0.1650901436805725f,-1.1026331186294556f,-3.2810006141662598f,-1.44349205493927f,0.6852923035621643f,-3.303107976913452f,-1.5818978548049927f,-6.072955131530762f,3.4959716796875f,1.5015085935592651f,-0.30304431915283203f,4.208858489990234f,1.2941808700561523f,-0.957578182220459f,4.362705230712891f,-0.03255860134959221f,0.10022296756505966f,-1.1715553998947144f,-1.0422441959381104f,2.155123472213745f,2.275383472442627f,-2.6855483055114746f,0.46899741888046265f,-1.180797815322876f,-0.2845727205276489f,3.378725528717041f,-0.40122610330581665f,-1.951186180114746f,0.5081520676612854f,-1.6926093101501465f,-1.114762783050537f,-0.8194169998168945f,0.6587628126144409f,-2.293916702270508f,1.7169477939605713f,-0.23768040537834167f};
alignas(16) float batch_normalization_A[] = {0.8635789752006531f,0.5247016549110413f,1.4155455827713013f,-0.4579620361328125f,-0.046245917677879333f,0.21679559350013733f,0.6176183819770813f,-0.08438841998577118f,0.26874586939811707f,-0.06627240777015686f,1.063916563987732f,0.05948343873023987f,0.02647668495774269f,1.2017874717712402f,0.8461903929710388f,0.008880577981472015f};
alignas(16) float conv2d_internal_1_W[] = {0.08012130111455917f,0.4832216203212738f,0.05338801071047783f,0.2104957550764084f,-0.7547124624252319f,-0.11653774231672287f,-0.10176682472229004f,-0.6227943301200867f,0.09076356887817383f,0.7883397340774536f,0.4005697965621948f,0.10972845554351807f,-0.22471560537815094f,-0.07990559935569763f,0.030585946515202522f,-1.225128173828125f,-0.5325846672058105f,0.17248277366161346f,0.4528442323207855f,-0.3754325807094574f,0.7380236983299255f,0.1783793866634369f,-0.6251382827758789f,-0.47323891520500183f,-0.8406429290771484f,-0.07291228324174881f,-0.9181187152862549f,0.19928498566150665f,0.9484058022499084f,-0.27942052483558655f,0.0454840213060379f,0.2830258905887604f,0.39126697182655334f,0.4794563949108124f,0.010216775350272655f,0.64216548204422f,-0.6086586713790894f,0.01699444092810154f,-1.339053750038147f,0.2223903089761734f,0.1369289606809616f,0.5615753531455994f,0.22115109860897064f,0.8541993498802185f,-0.49862632155418396f,0.09700748324394226f,-1.0854225158691406f,0.33104005455970764f,-0.1771765649318695f,0.4018617272377014f,-1.2395631074905396f,0.40963214635849f,-0.6261389851570129f,0.21340274810791016f,0.14529351890087128f,-0.4039100408554077f,0.4094037711620331f,0.04660645127296448f,0.27992621064186096f,-0.9095547199249268f,0.429002970457077f,0.06122719869017601f,-0.19943509995937347f,0.7510515451431274f};
alignas(16) float batch_normalization_1_A[] = {0.5539381504058838f,0.1267862319946289f,1.0557823181152344f,0.19319409132003784f};
alignas(16) float separable_conv2d_1_internal_0_VALUES[] = {0.0f,0.0f,0.0f,0.0f};
alignas(16) float separable_conv2d_1_internal_1_W[] = {-0.1353314369916916f,0.10196460783481598f,-0.06543825566768646f,-0.03494260832667351f,0.11509816348552704f,0.09667158871889114f,-0.07756936550140381f,-0.15584950149059296f,0.06079130992293358f,-0.1121378019452095f,-0.05494485795497894f,0.04945266619324684f,-0.0647812932729721f,0.006602134555578232f,-0.003242660081014037f,0.029232610017061234f,-0.0896972194314003f,0.08381238579750061f,0.0004475994501262903f,-0.15575672686100006f,0.027407385408878326f,0.09174317121505737f,-0.03204898536205292f,-0.09787828475236893f,0.002594405086711049f,-0.13384230434894562f,-0.041594527661800385f,-0.004645694978535175f,-0.13479949533939362f,0.05077529698610306f,0.04926783964037895f,0.09136804193258286f,0.010015182197093964f,0.013389925472438335f,0.028249815106391907f,-0.09775202721357346f,-0.13865530490875244f,0.07027223706245422f,0.13559558987617493f,-0.11186179518699646f,-0.045782361179590225f,-0.07723181694746017f,0.0186216589063406f,-0.019577454775571823f,-0.04669853672385216f,0.1500522494316101f,0.06981173157691956f,0.022173607721924782f,-0.08134160190820694f,0.06711458414793015f,-0.0172431617975235f,0.177463099360466f,0.1328841745853424f,-0.13573259115219116f,0.01627071015536785f,-0.07925041019916534f,0.15643282234668732f,0.01599922403693199f,-0.03591937944293022f,0.0058456650003790855f,-0.02378600277006626f,-0.1442958563566208f,0.0798659548163414f,-0.01529791858047247f,-0.09178943186998367f,-0.012392599135637283f,0.15202145278453827f,0.135381817817688f,-0.008431614376604557f,-0.16105009615421295f,-0.09201457351446152f,0.0034036387223750353f,0.15520381927490234f,0.03240988403558731f,0.1141667291522026f,-0.038797102868556976f,0.0012820780975744128f,-0.07893173396587372f,0.12783357501029968f,-0.07897496968507767f,-0.00997191946953535f,-0.056426532566547394f,0.12235736101865768f,0.03758283331990242f,-0.21350516378879547f,-0.08560439199209213f,-0.027975261211395264f,0.0014679944142699242f,-0.00046777515672147274f,0.022452566772699356f,0.13642708957195282f,-0.06745920330286026f,0.05335056781768799f,0.10771767795085907f,0.05591805651783943f,-0.14651571214199066f,0.009043315425515175f,-0.08343079686164856f,-0.11410947144031525f,0.017197588458657265f,0.12356791645288467f,0.0730806514620781f,-0.03518639877438545f,0.02763228677213192f,0.010358692146837711f,0.02944350615143776f,-0.09731949865818024f,-0.05569464713335037f,0.09418083727359772f,-0.08100118488073349f,0.002565266564488411f,0.127218097448349f,-0.00109188468195498f,-0.17257188260555267f,-0.08256293088197708f,-0.04030994325876236f,-0.003533259266987443f,0.004753605928272009f,-0.10962074995040894f,0.16164104640483856f,-0.050769295543432236f,0.024174895137548447f,-0.05722341313958168f,-0.13795439898967743f,0.13937385380268097f,-0.013075385242700577f,-0.010043205693364143f,0.09057380259037018f,-0.013312586583197117f,-0.06722744554281235f,0.010727396234869957f,-0.05263740196824074f,-0.1290050446987152f,0.0445798859000206f,-0.026542356237769127f,0.19380000233650208f,-0.07622810453176498f,-0.008523805066943169f,-0.027670949697494507f,-0.04524961858987808f,0.04720078036189079f,0.03604111447930336f,-0.039844006299972534f,-0.06471085548400879f};
alignas(16) float separable_conv2d_1_internal_2_W[] = {-0.18675442039966583f,-0.7594151496887207f,0.13986419141292572f,0.3015216588973999f,-2.3792271614074707f,1.6556384563446045f,-2.5755224227905273f,-1.3547477722167969f,-0.9333784580230713f,-0.05686267837882042f,0.2440395951271057f,-0.29961952567100525f,-0.16711120307445526f,-0.33556804060935974f,2.9102635383605957f,-1.771820306777954f,0.8206219673156738f,-0.5111240744590759f,1.070256233215332f,3.4744720458984375f,1.511655330657959f,0.1635781079530716f,-0.644557535648346f,0.46545350551605225f,0.10532126575708389f,-0.504347026348114f,-1.1742123365402222f,0.38010647892951965f,-1.7320420742034912f,-0.7936625480651855f,0.3192594349384308f,-1.2717351913452148f,-0.53005051612854f,0.9849063158035278f,0.04538758099079132f,0.1941044181585312f,-0.06228528916835785f,2.1108367443084717f,-1.2200450897216797f,-0.3461226522922516f,-1.6606448888778687f,2.0277531147003174f,0.9015659689903259f,1.093493103981018f,-1.5225133895874023f,-0.36033204197883606f,2.1442947387695312f,-3.126882314682007f,-0.2175908386707306f,-1.3472715616226196f,0.18185102939605713f,0.41373348236083984f,0.45469093322753906f,2.4090254306793213f,0.5835212469100952f,0.47967368364334106f,-0.6641505360603333f,0.33925652503967285f,2.761772871017456f,1.3348383903503418f,-1.5590894222259521f,-0.27539026737213135f,-2.448671817779541f,0.1811847686767578f,-1.0500776767730713f,0.142685204744339f,-1.4665640592575073f,-0.18594452738761902f,1.549789547920227f,0.5055315494537354f,-0.01748180203139782f,-1.6654833555221558f,2.543975591659546f,1.683250069618225f,0.7241335511207581f,-0.11496781557798386f,0.7275431156158447f,-1.7889800071716309f,-0.8772178888320923f,-0.02766173519194126f,-0.6337113976478577f,-1.4013234376907349f,3.7728121280670166f,-0.42754271626472473f,-1.6368459463119507f,0.5869626998901367f,1.0534968376159668f,0.9871006011962891f,1.2672686576843262f,-0.8024876713752747f,-0.7034115791320801f,-0.9585533142089844f,-0.35376909375190735f,-0.7667788863182068f,0.1274736374616623f,-2.0177268981933594f,-0.6285969614982605f,-1.1693847179412842f,1.2163931131362915f,-0.6366583704948425f,-1.1185282468795776f,1.2765264511108398f,0.43238624930381775f,-1.822045922279358f,1.6476081609725952f,2.748439311981201f,0.35603418946266174f,2.0716636180877686f,-0.009470763616263866f,3.07930850982666f,0.12473881244659424f,0.8311547040939331f,1.1985228061676025f,-0.10316038876771927f,-2.465592861175537f,-1.5050721168518066f,0.817406415939331f,-0.6543265581130981f,-2.971466302871704f,0.29711058735847473f,0.18067997694015503f,-1.132128119468689f,-0.7519075274467468f,0.02020505629479885f,0.37678155303001404f,-0.19634772837162018f,-0.23896263539791107f,-0.28715407848358154f,0.6762341856956482f,1.5572266578674316f,2.4012274742126465f,0.8074157238006592f,-0.7729071974754333f,-0.3100499212741852f,-0.14554737508296967f,-0.13214386999607086f,1.0809333324432373f,-2.9446446895599365f,0.7733811736106873f,1.140955924987793f,-1.4930977821350098f,-3.082244873046875f,0.5471177697181702f,1.6436866521835327f,1.8683772087097168f,-2.461184501647949f,-0.49919697642326355f,-2.401553153991699f,1.8455108404159546f,-0.31093600392341614f,0.2488955706357956f,0.2137894332408905f,3.211219549179077f,0.06705213338136673f,-0.46753090620040894f,-0.6512815952301025f,0.7894930839538574f,0.013066406361758709f,0.40226271748542786f,0.36666616797447205f,-0.9438372254371643f,-0.31559011340141296f,1.2149752378463745f,-1.183847427368164f,-0.09495557844638824f,0.20654340088367462f,2.5103626251220703f,0.029436351731419563f,0.35943952202796936f,1.2737178802490234f,-0.4777660667896271f,2.204577684402466f,0.9780387878417969f,0.3575618267059326f,0.8960353136062622f,0.050015244632959366f,0.33852919936180115f,1.370617151260376f,1.5548079013824463f,-0.08850632607936859f,-1.8525673151016235f,-2.8630194664001465f,0.6521296501159668f,0.5122818946838379f,-2.253532648086548f,-0.2565394937992096f,-0.09415668994188309f,-0.7772957682609558f,1.5600953102111816f,-1.2444390058517456f,-0.8666260838508606f,0.6832718253135681f,0.31338194012641907f,1.3907722234725952f,-1.7612206935882568f,-0.14341850578784943f,0.3505188822746277f,0.6353243589401245f,2.1017239093780518f,-1.4040390253067017f,0.9379248023033142f,1.0963947772979736f,0.23184619843959808f,-1.3631932735443115f,2.0348308086395264f,0.18287615478038788f,0.42073532938957214f,0.5206913352012634f,1.337701439857483f,-1.748672366142273f,-1.698582649230957f,1.064036250114441f,1.152255892753601f,0.7971847653388977f,1.1082936525344849f,0.5437206625938416f,-1.9856711626052856f,-0.6614692211151123f,-0.15882940590381622f,1.0103787183761597f,0.5323837995529175f,0.08162326365709305f,-1.1398680210113525f,-0.3649834096431732f,0.0714496448636055f,1.2894973754882812f,-0.839566171169281f,0.8147850632667542f,2.823763847351074f,-0.2588466703891754f,0.699863076210022f,1.0148204565048218f,-1.5320115089416504f,0.762346088886261f,0.29492831230163574f,-0.3770458996295929f,-0.7773191928863525f,-0.5130881071090698f,-0.1984020173549652f,-1.7910703420639038f,0.6649505496025085f,-0.8253679871559143f,-1.471483826637268f,-0.19759085774421692f,-1.1242752075195312f,-1.1541088819503784f,-0.46015286445617676f,1.3461946249008179f,0.3728095591068268f,-0.46006566286087036f,-0.14607468247413635f,1.2168375253677368f,2.6215505599975586f,0.13845504820346832f,0.3571147322654724f,-0.3495059609413147f,-0.5898745059967041f,-1.6499179601669312f,-1.8119606971740723f,-0.8381258249282837f,-0.09095388650894165f,0.8289241790771484f,-0.982442319393158f,-0.3170316815376282f,1.188125491142273f,-1.039601445198059f,-1.7733023166656494f,2.433570146560669f,0.3650442957878113f,0.6009701490402222f,-1.3119418621063232f,-0.3707926869392395f,-0.8681150674819946f,-0.8354036211967468f,-0.17034292221069336f,1.270053744316101f,-0.3005989193916321f,-0.6666816473007202f,-0.9295266270637512f,0.7719560861587524f,2.913288116455078f,-1.1969807147979736f,1.4929183721542358f,-0.12827767431735992f,-0.21333584189414978f,0.4909771680831909f,-1.8370239734649658f,0.06371964514255524f,1.3560967445373535f,0.32974034547805786f,-1.1246724128723145f,-0.18410630524158478f,-1.188624382019043f,0.9342899322509766f,-0.7446557283401489f,-0.7701770663261414f,0.34112146496772766f,-0.23146629333496094f,-0.3017773926258087f,-0.3732958137989044f,-0.3014301061630249f,-0.1499331146478653f,0.6871960759162903f,1.0974992513656616f,0.7620768547058105f,-1.997903823852539f,0.5370734930038452f,-0.9163922071456909f,2.1551289558410645f,-0.7449110746383667f,-1.9032015800476074f,1.5677433013916016f,-2.798046112060547f,-0.4002586901187897f,-1.617430567741394f,0.16301122307777405f,1.394369125366211f,2.6294312477111816f,-0.4503626227378845f,-2.182241201400757f,-0.4408842921257019f,1.2971923351287842f,-0.5730848908424377f,-1.0225615501403809f,1.4005892276763916f,-0.43986156582832336f,-0.9054315090179443f,0.40829142928123474f,0.34779539704322815f,0.1350191831588745f,0.9644317626953125f,1.8022900819778442f,-0.9161345362663269f,0.8432113528251648f,0.46700283885002136f,-0.6960490345954895f,-1.1625232696533203f,-1.2863754034042358f,1.3424214124679565f,0.6409430503845215f,-3.2296063899993896f,0.8228482604026794f,0.8786506652832031f,0.22584857046604156f,1.2779290676116943f,-0.8246848583221436f,0.6048899292945862f,1.434126377105713f,-1.090590000152588f,-0.3974522650241852f,-0.9772728085517883f,1.1901816129684448f,1.74004328250885f,0.8840908408164978f,1.4634549617767334f,1.5088895559310913f,-0.9444355368614197f,-1.1420180797576904f,-0.613838255405426f,-0.4135836064815521f,0.3563368320465088f,-0.3255416452884674f,-0.1978723704814911f,0.561485230922699f,0.713725745677948f,-0.6465432047843933f,-0.6249448657035828f,1.8297672271728516f,0.31390640139579773f,-0.5007661581039429f,-2.2128565311431885f,0.5275672078132629f,2.615112543106079f,-0.765275776386261f,1.4775617122650146f,-0.36624160408973694f,-0.39054250717163086f,-0.2227778434753418f,-0.722446620464325f,-0.9928331971168518f,-0.5111605525016785f,-2.258284091949463f,-1.3651254177093506f,-0.7859408855438232f};
alignas(16) float batch_normalization_2_A[] = {0.1807873398065567f,-1.1068925857543945f,-0.7804926633834839f,0.2973366379737854f,0.4306294620037079f,0.10381114482879639f,-1.5699888467788696f,-0.25903934240341187f,0.3263375461101532f,-0.16762438416481018f,-0.3600577116012573f,0.2943260669708252f,0.2318718135356903f,0.05281832814216614f,0.5499204397201538f,0.24290573596954346f,-0.4076674282550812f,-0.26344817876815796f,0.8095517158508301f,-0.3024790287017822f,0.4497032165527344f,-0.4858855605125427f,-0.39444059133529663f,-0.5763674974441528f};
alignas(16) float conv2d_1_internal_1_W[] = {0.20734094083309174f,0.3235519230365753f,-0.28108862042427063f,-0.037736356258392334f,0.11625885963439941f,0.011262031272053719f,0.16189393401145935f,-0.37210583686828613f,0.16081957519054413f,0.15072962641716003f,0.25940293073654175f,-0.17711225152015686f,-0.0532408244907856f,-0.026478545740246773f,0.11499617993831635f,0.3913763165473938f,0.057213544845581055f,-0.1780775785446167f,-0.24178163707256317f,0.15734431147575378f,0.10209237039089203f,0.004260062240064144f,0.038838811218738556f,-0.15116353332996368f,-0.03448237106204033f,0.20997999608516693f,0.12675593793392181f,0.2945356070995331f,-0.11500036716461182f,0.18275047838687897f,-0.06069299206137657f,-0.20929555594921112f,-0.13638179004192352f,-0.18495553731918335f,0.08590719848871231f,0.09920242428779602f,0.19013291597366333f,-0.13526129722595215f,-0.13340216875076294f,-0.21449093520641327f,0.09256106615066528f,0.24512849748134613f,-0.07163139432668686f,-0.2779949903488159f,-0.02345811016857624f,0.2575005292892456f,-0.16480369865894318f,0.155550017952919f,0.08429131656885147f,-0.3377162218093872f,0.30985721945762634f,-0.13849492371082306f,0.22626163065433502f,-0.019918568432331085f,-0.06833896785974503f,-0.1209636852145195f,-0.12318002432584763f,0.6615762710571289f,0.2785736620426178f,-0.20009808242321014f,0.1114235371351242f,0.11997190117835999f,0.12040398269891739f,0.2922760844230652f,-0.040015120059251785f,0.1648893505334854f,-0.23754212260246277f,-0.09854435920715332f,0.10987736284732819f,0.11640812456607819f,-0.2632008492946625f,-0.19904597103595734f,0.11137275397777557f,-0.5223572850227356f,-0.06736309081315994f,-0.2085418701171875f,0.166991725564003f,-0.07620116323232651f,0.3508989214897156f,0.012744488194584846f,0.1748168170452118f,-0.09500916302204132f,0.18961133062839508f,0.16713738441467285f,-0.18196043372154236f,0.07111863791942596f,0.3821244239807129f,-0.031917523592710495f,0.20255260169506073f,0.0276719331741333f,-0.4518432021141052f,0.14005646109580994f,0.019493944942951202f,0.09969838708639145f,-0.14113886654376984f,0.0906568095088005f,0.23983095586299896f,-0.15515924990177155f,0.09172666817903519f,0.33096086978912354f,-0.11878905445337296f,0.08122561872005463f,0.23484192788600922f,-0.13868802785873413f,-0.2244703769683838f,0.9609673619270325f,0.4021594822406769f,0.02329644188284874f,-0.1383059322834015f,-0.08474660664796829f,-0.019729413092136383f,0.0911540687084198f,0.30197522044181824f,0.08174792677164078f,-0.08278870582580566f,-0.22639687359333038f,-0.025217333808541298f,-0.032087262719869614f,0.1878959834575653f,-0.18118387460708618f,-0.22433331608772278f,-0.20890693366527557f,-0.08256261795759201f,0.06145840510725975f,0.2372235804796219f,0.22934983670711517f,0.07280942052602768f,-0.04927930608391762f,-0.10937929153442383f,-0.9546071887016296f,-0.12880200147628784f,0.12021100521087646f,-0.08172933012247086f,0.04042699560523033f,0.07872230559587479f,0.2861941456794739f,0.23983275890350342f,0.13175331056118011f,0.0804823562502861f,0.43145278096199036f,-0.13954408466815948f,0.13763675093650818f,0.31740662455558777f,-0.09352660179138184f,0.029435668140649796f,-0.43700483441352844f,-0.05928909778594971f,0.10818450897932053f,-0.07013475149869919f,0.0639752745628357f,0.11105348914861679f,-0.30272406339645386f,0.5399328470230103f,0.2063458114862442f,-0.4347838759422302f,0.0017741266638040543f,0.031156448647379875f,-0.1294257640838623f,-0.16665436327457428f,-0.017220724374055862f,-0.003951642662286758f,-0.6710652709007263f,-0.3388645648956299f,-0.09236990660429001f,0.024016540497541428f,-0.10281741619110107f,0.25620028376579285f,0.03259238973259926f,0.25685903429985046f,-0.16332405805587769f,0.11142290383577347f,-0.04850931465625763f,0.2760188579559326f,-0.07009382545948029f,-0.13370820879936218f,0.4818611145019531f,0.07906108349561691f,-0.3377216160297394f,0.41755592823028564f,-0.16105841100215912f,-0.12838827073574066f,0.033589769154787064f,-0.2662128210067749f,-0.15358375012874603f,0.08118970692157745f,0.3750187158584595f,0.10835763067007065f,0.20589853823184967f,-0.02042790874838829f,0.02352682501077652f,0.362291544675827f,0.04950555041432381f};
alignas(16) float batch_normalization_3_A[] = {-0.013449341058731079f,0.01585150882601738f,0.12135317921638489f,0.554092288017273f,0.5614051818847656f,0.2772301435470581f,0.25052663683891296f,0.6035758256912231f};
alignas(16) float separable_conv2d_2_internal_0_VALUES[] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
alignas(16) float separable_conv2d_2_internal_1_W[] = {-0.0543765053153038f,0.01334695890545845f,0.012214219197630882f,0.08829402178525925f,-0.018486887216567993f,-0.1017267033457756f,-0.07984574139118195f,-0.008629269897937775f,0.06841856986284256f,-0.06744503229856491f,0.023887475952506065f,0.0750437006354332f,0.055925823748111725f,0.06444208323955536f,0.04610517993569374f,-0.08050867170095444f,0.155278280377388f,0.05639263615012169f,0.03282152861356735f,-0.08932353556156158f,0.0611567497253418f,-0.1320761740207672f,-0.05713807791471481f,0.17496079206466675f,-0.05848373472690582f,-0.05177190527319908f,-0.0014624196337535977f,0.07144588977098465f,0.08254481852054596f,-0.013751250691711903f,0.0655633881688118f,-0.028526954352855682f,-0.13985708355903625f,0.02983817271888256f,0.05065998062491417f,-0.012828832492232323f,0.06073522940278053f,-0.02686145529150963f,0.07336322963237762f,-0.04446794465184212f,-0.06829367578029633f,-0.050116244703531265f,-0.1620679795742035f,-0.035968806594610214f,-0.0051293992437422276f,0.1349698007106781f,-0.15445208549499512f,-0.01681983657181263f,-0.14300869405269623f,-0.09434561431407928f,0.08761566877365112f,0.06501022726297379f,-0.06450293958187103f,-0.023116732016205788f,-0.09340564906597137f,-0.0628432109951973f,0.12702828645706177f,-0.19198930263519287f,0.015070920810103416f,-0.021288389340043068f,0.0754443034529686f,-0.08421323448419571f,-0.11234009265899658f,0.005200044717639685f,0.023724490776658058f,0.10494031012058258f,0.055974774062633514f,-0.09040556102991104f,-0.060939569026231766f,-0.0018095319392159581f,0.03899576514959335f,0.009619032964110374f,-0.11572735011577606f,0.019273612648248672f,0.046107858419418335f,0.1027728021144867f,-0.15184512734413147f,-0.00436325604096055f,0.09917280077934265f,-0.09183689951896667f,0.0753106027841568f,0.10562559217214584f,0.18116578459739685f,0.09885995090007782f,0.078405000269413f,0.04209583252668381f,0.04770578071475029f,-0.14116394519805908f,-0.09165376424789429f,0.03017621673643589f,-0.09116321802139282f,-0.10836099088191986f,0.09734823554754257f,0.0016978309722617269f,0.003936105873435736f,0.1742784082889557f,-0.06877172738313675f,-0.07210000604391098f,-0.013543067499995232f,0.06951847672462463f,-0.051633648574352264f,-0.018733473494648933f,-0.012175400741398335f,-0.08365710079669952f,0.060730647295713425f,-0.09275375306606293f,0.083164744079113f,0.033404283225536346f,0.04290022328495979f,-0.02665066532790661f,0.06746672093868256f,-0.1237831711769104f,0.10446194559335709f,0.060319896787405014f,0.03438819572329521f,-0.11921084672212601f,-0.15848645567893982f,-0.11111786961555481f,-0.10628096759319305f,0.02090180665254593f,-0.12546788156032562f,-0.04662006348371506f,-0.011858821846544743f,0.09667334705591202f,0.01248207688331604f,-0.011549691669642925f,0.17818042635917664f,-0.044510092586278915f,0.022615129128098488f,-0.1403140127658844f,0.0773378387093544f,0.05380333214998245f,0.141206294298172f,0.019443806260824203f,0.06468163430690765f,-0.06055406853556633f,0.021660514175891876f,-0.09328088909387589f,-0.07601769268512726f,-0.07747629284858704f,0.08724129945039749f,0.0983840674161911f,-0.011832138523459435f,0.16550499200820923f,-0.0017305135261267424f,-0.14511369168758392f,-0.03889220207929611f,0.056013576686382294f,-0.0035697559360414743f,0.009426779113709927f,-0.06295856088399887f,-0.1951405107975006f,-0.041507914662361145f,-0.00543938297778368f,-0.0012283558025956154f,-0.017674321308732033f,-0.0039912075735628605f,0.03821929916739464f,-0.03970843553543091f,-0.045038290321826935f,0.08403147011995316f,0.06128276512026787f,0.04250528663396835f,0.01578882522881031f,-0.06518351286649704f,-0.014148827642202377f,0.03474527969956398f,-0.11910498887300491f,0.053862083703279495f,-0.006167065352201462f,-0.032903436571359634f,0.1308012455701828f,-0.12373112142086029f,-0.08056086301803589f,0.034805990755558014f,-0.015978610143065453f,0.0999484434723854f,-0.01346677541732788f,-0.05669472739100456f,0.08810222148895264f,0.0405023992061615f,0.138871431350708f,-0.04333933815360069f,0.005377351772040129f,-0.021303366869688034f,-0.0363510400056839f,-0.035354889929294586f,-0.11558278650045395f,0.030110254883766174f,0.012073577381670475f,-0.037087053060531616f,-0.12944631278514862f,-0.06925544887781143f,-0.06244130805134773f,-0.09319557994604111f,-0.17370811104774475f,0.0020557392854243517f,-0.053640663623809814f,-0.05461100488901138f,0.011241022497415543f,-0.07686344534158707f,-0.04962462559342384f,0.0269668847322464f,-0.011666795238852501f,0.030659813433885574f,-0.005387929733842611f,0.006727372761815786f,-0.10159468650817871f,-0.11426064372062683f,-0.013999953866004944f,0.1618003249168396f,-0.06029415503144264f,-0.1296946257352829f,0.04830542951822281f,0.14573822915554047f,0.028770338743925095f,-0.127131387591362f,0.002153217326849699f,0.013357161544263363f,0.08790412545204163f,-0.08925709128379822f,-0.11332102864980698f,0.11435622721910477f,0.04444480687379837f,0.018546240404248238f,0.025967096909880638f,-0.02980760484933853f,0.020026633515954018f,0.022820832207798958f,-0.023964669555425644f,-0.04067439213395119f,-0.07827402651309967f,-0.028924325481057167f,-0.0022680487018078566f,0.007780961226671934f,0.07628001272678375f,0.0190725140273571f,-0.010950937867164612f,0.12796761095523834f,0.09634040296077728f,0.005960988346487284f,-0.02089213952422142f,-0.01042077224701643f,-0.11156603693962097f,-0.056580688804388046f,0.05470866337418556f,0.09206774085760117f,0.006046769674867392f,0.040688104927539825f,0.020095597952604294f,0.0492602176964283f,-0.040158502757549286f,-0.03361951932311058f,-0.05722915381193161f,-0.06739113479852676f,0.03364429622888565f,-0.03769393637776375f,0.09055516123771667f,-0.09686418622732162f,0.1073843389749527f,0.026882058009505272f,0.12787899374961853f,-0.10431354492902756f,-0.0617305189371109f,0.09204012155532837f,0.10498592257499695f,-0.057240474969148636f,0.019448844715952873f,-0.036773551255464554f,0.02657228335738182f,-0.00742469122633338f,0.017096642404794693f,0.05806770175695419f,-0.06691837310791016f,-0.09858784079551697f,-0.03933137282729149f,0.05963224917650223f,0.12995678186416626f,-0.09141392260789871f,0.1915532499551773f,0.015696212649345398f,-0.01944391056895256f,0.17493143677711487f,-0.07198190689086914f,-0.08616092801094055f,-0.05448484048247337f,-0.07614962011575699f,-0.10184311866760254f};
alignas(16) float separable_conv2d_2_internal_2_W[] = {2.4819626808166504f,-2.111316442489624f,2.43887996673584f,-0.4318809509277344f,-0.23636052012443542f,-0.6569046378135681f,-1.9099137783050537f,0.43900400400161743f,0.26532861590385437f,0.4416011571884155f,-1.5030004978179932f,0.22895389795303345f,-0.09121943265199661f,0.0321245938539505f,1.3113698959350586f,-1.6063969135284424f,-0.22040796279907227f,-0.955112636089325f,-0.5982388854026794f,1.7655930519104004f,-2.614259719848633f,-1.0789070129394531f,-0.004768972285091877f,0.8834589123725891f,-0.921716570854187f,-1.261428952217102f,1.0020650625228882f,-1.5100122690200806f,0.5645030736923218f,0.30955591797828674f,0.6722754240036011f,-0.6211487650871277f,-0.33421772718429565f,-0.8992042541503906f,-0.2329210489988327f,1.3315229415893555f,0.9731089472770691f,-2.0019495487213135f,-0.8300622701644897f,0.12723632156848907f,-1.6992894411087036f,0.5312778353691101f,-1.023459553718567f,1.6948399543762207f,0.6574389934539795f,0.022207792848348618f,0.38534072041511536f,1.1750580072402954f,0.043846555054187775f,0.8485726118087769f,-0.9041284322738647f,1.1280099153518677f,-0.9748678803443909f,-2.0357353687286377f,-3.366276264190674f,1.3931058645248413f,1.6581337451934814f,-0.10186007618904114f,1.1213072538375854f,2.2178139686584473f,-0.44839179515838623f,-1.2749167680740356f,1.0694031715393066f,0.8837944269180298f,0.08492256700992584f,-1.630958914756775f,1.8792375326156616f,0.7015188336372375f,-2.411618947982788f,-2.508774518966675f,-0.022843562066555023f,-0.08571825176477432f,-0.7687273621559143f,-0.7190855145454407f,0.8858797550201416f,-0.39483609795570374f,-0.10405408591032028f,0.1819387823343277f,0.22918806970119476f,0.461366206407547f,-0.5286800861358643f,0.8495845794677734f,-0.8460204005241394f,0.22041024267673492f,0.918776273727417f,-0.6435959339141846f,1.5070595741271973f,1.1967053413391113f,0.6322490572929382f,-0.5410581827163696f,-0.12577320635318756f,0.05014379695057869f,-0.9252722263336182f,-1.3041625022888184f,1.2770628929138184f,-1.9295415878295898f,1.4338228702545166f,-0.31871867179870605f,1.3168572187423706f,-0.3309696912765503f,-0.8019403219223022f,0.02361142449080944f,-0.029055386781692505f,-0.022034255787730217f,0.3842718005180359f,-1.7045789957046509f,0.6095473170280457f,-0.650870680809021f,0.539039134979248f,1.17677903175354f,-1.0360349416732788f,-1.5395491123199463f,-0.6595869660377502f,-1.8032816648483276f,-0.4509686529636383f,-0.35257062315940857f,2.820390224456787f,-0.05399397015571594f,-1.3450714349746704f,-0.04703478142619133f,1.6560124158859253f,-2.2053580284118652f,-0.6349915266036987f,0.6636660099029541f,-1.2189213037490845f,-0.3696765899658203f,-0.11923868954181671f,-0.6178060173988342f,-1.0097399950027466f,1.025604486465454f,-0.45920896530151367f,-0.8053620457649231f,-0.3799518048763275f,-2.3409364223480225f,1.6419475078582764f,-0.022806493565440178f,0.7292605638504028f,-0.847575306892395f,0.884046733379364f,0.6606674790382385f,-1.7822433710098267f,0.21365247666835785f,-1.7201930284500122f,-0.508724570274353f,1.0822923183441162f,-1.5031746625900269f,0.7707802057266235f,-0.26420316100120544f,-0.7816521525382996f,-0.31864023208618164f,-0.19267596304416656f,-0.21952491998672485f,-0.2557666003704071f,0.26159539818763733f,-1.3760300874710083f,-0.14647932350635529f,0.8325031995773315f,-0.18648727238178253f,0.3668592870235443f,-0.011748096905648708f,0.08799261599779129f,0.4981308579444885f,2.2489593029022217f,0.5655332803726196f,0.06943762302398682f,-0.750984251499176f,-1.5615242719650269f,-1.7490512132644653f,-0.01116811390966177f,-0.33972233533859253f,0.33282652497291565f,-0.08760231733322144f,-0.2591128647327423f,0.6577610969543457f,0.7897317409515381f,2.2311861515045166f,-1.1844180822372437f,-0.28266409039497375f,0.054108165204524994f,-0.8994839191436768f,-0.5919249057769775f,-0.701336681842804f,-1.2808387279510498f,-0.4719894230365753f,-0.8356165885925293f,1.5315022468566895f,1.1039093732833862f,0.004344751127064228f,0.7167086601257324f,-0.3663747012615204f,-0.4262242913246155f,-1.418722152709961f,-1.8922536373138428f,-1.256331443786621f,0.21818697452545166f,-1.2304130792617798f,-1.2722320556640625f,1.3217631578445435f,-0.057235926389694214f,-0.3003973364830017f,0.6263697147369385f,-0.007753837388008833f,0.8909574747085571f,-0.06109361723065376f,-0.9627249836921692f,0.4682050049304962f,-1.3386725187301636f,0.9287078976631165f,1.0208898782730103f,0.3499876856803894f,1.1568095684051514f,-0.6291812658309937f,0.008404821157455444f,-0.8570855855941772f,-0.516153872013092f,-0.2673153877258301f,-0.1437257081270218f,-0.07831032574176788f,-0.779297411441803f,1.0864583253860474f,-0.1763584017753601f,1.3292018175125122f,-2.5080299377441406f,1.0231757164001465f,-0.9245893359184265f,-1.58034348487854f,0.8614568710327148f,1.6480293273925781f,-0.11956685781478882f,1.2028777599334717f,-0.7641828656196594f,0.45044630765914917f,0.3875757157802582f,-0.06297089904546738f,0.03939557448029518f,-1.2515195608139038f,5.070519924163818f,1.180748462677002f,0.1930374652147293f,-1.1164159774780273f,-0.05960603803396225f,-2.2429072856903076f,4.0675764083862305f,1.1359994411468506f,-0.027848640456795692f,-1.017541527748108f,-1.0388869047164917f,0.6905445456504822f,-0.70681232213974f,-1.1100764274597168f,1.1228501796722412f,0.8013190627098083f,-0.2571712136268616f,1.6190117597579956f,0.3166486620903015f,-0.4597167670726776f,-0.5091120600700378f,-0.37805986404418945f,-0.9207186698913574f,0.035905156284570694f,-0.015852157026529312f,-0.9799998998641968f,0.48539161682128906f,0.05407227575778961f,1.2685481309890747f,-0.8253850936889648f,0.32619211077690125f,-0.24144084751605988f,0.6253820061683655f,-1.0131040811538696f,-0.6843516230583191f,1.045048475265503f,-0.18690192699432373f,1.313652515411377f,0.9357303380966187f,-0.6527140736579895f,-2.597073554992676f,0.9738174080848694f,0.5673969984054565f,-0.07971824705600739f,0.06553575396537781f,1.0488090515136719f,1.366112470626831f,1.3802601099014282f,-1.7524895668029785f,-0.8066327571868896f,0.6367951035499573f,0.6231579184532166f,3.132077932357788f,0.453801691532135f,-2.6667847633361816f,0.31309282779693604f,-0.4492093324661255f,-0.6843135952949524f,-0.26440879702568054f,-0.14714817702770233f,-0.11055941879749298f,1.8254835605621338f,1.0825116634368896f,0.10079769045114517f,0.10049363225698471f,0.3797262907028198f,-1.0864031314849854f,1.433694839477539f,-2.4760994911193848f,0.6294935345649719f,1.4652280807495117f,-1.3774633407592773f,1.2438781261444092f,1.7611479759216309f,0.44900625944137573f,1.928959608078003f,1.5468393564224243f,-0.026782510802149773f,0.6835402250289917f,-0.629056990146637f,-0.030552059412002563f,-0.1907871961593628f,0.34788593649864197f,1.186604380607605f,-1.5579334497451782f,-0.057105984538793564f,-2.4414868354797363f,-0.2553524076938629f,-0.8387698531150818f,1.7225722074508667f,0.6858920454978943f,0.756835401058197f,2.2315924167633057f,2.245380401611328f,-0.8125717043876648f,0.5746617913246155f,1.1302059888839722f,1.3770374059677124f,-1.7678768634796143f,0.12627772986888885f,0.26404762268066406f,1.829208493232727f,-0.18439790606498718f,0.17275173962116241f,0.3939901292324066f,-1.8266500234603882f,-0.17351022362709045f,-0.39631879329681396f,0.7528381943702698f,0.39997419714927673f,-0.45446932315826416f,-0.6470096111297607f,-1.0606502294540405f,0.4125547707080841f,0.28160402178764343f,-0.9255560040473938f,1.3141882419586182f,-0.5363149046897888f,1.4477484226226807f,0.9358789324760437f,0.1874794214963913f,0.6058194637298584f,-0.31357342004776f,0.2446317970752716f,2.419135570526123f,-1.7333279848098755f,-0.0759170651435852f,0.12014780938625336f,-0.7133331894874573f,0.42724496126174927f,-1.6868765354156494f,0.49294742941856384f,-0.3146803081035614f,-1.1344895362854004f,3.271145820617676f,0.12568046152591705f,-0.6656543612480164f,-0.3771364092826843f,-4.018775463104248f,-0.7336164712905884f,-1.8052830696105957f,0.5594456791877747f,-1.1084494590759277f,0.9579286575317383f,-0.8769547343254089f,-0.19008931517601013f,-0.7509019374847412f,-0.3160471022129059f,0.7161955237388611f,0.8531343936920166f,-1.3367702960968018f,0.11402872204780579f,-0.2310630977153778f,1.4858627319335938f,-0.4771982431411743f,0.05587766692042351f,2.1460750102996826f,0.5254104733467102f,1.0139579772949219f,1.7132781744003296f,0.7955467700958252f,-1.6178886890411377f,0.5175871253013611f,2.0709502696990967f,1.92642080783844f,0.9661192893981934f,0.42498138546943665f,0.08568049222230911f,0.40764322876930237f,-0.45915165543556213f,2.502593994140625f,0.5250242948532104f,0.9111415147781372f,-0.2581130564212799f,2.0399980545043945f,2.1665759086608887f,-0.8405941128730774f,0.1839969903230667f,1.1173553466796875f,2.155410051345825f,-0.6371303200721741f,0.2924259901046753f,-0.308660626411438f,1.3162202835083008f,-0.7392868399620056f,0.19350434839725494f,-1.252290964126587f,0.5325372219085693f,2.7351772785186768f,-0.8381032943725586f,-0.7648427486419678f,0.3115708529949188f,0.324886292219162f,-1.2045965194702148f,-1.0776365995407104f,-0.22476695477962494f,-1.1127828359603882f,0.6273971796035767f,0.3769312798976898f,-2.372256278991699f,-0.24128764867782593f,-2.0163490772247314f,1.2048920392990112f,-0.31702569127082825f,0.68698650598526f,-1.1805001497268677f,0.6145451664924622f,-3.291707992553711f,-0.15830768644809723f,-0.4862615764141083f,-0.1148407980799675f,-0.4253723621368408f,-1.230818748474121f,0.642807126045227f,0.007254964672029018f,1.4661189317703247f,1.173767328262329f,0.6968635320663452f,-0.6063142418861389f,1.9064216613769531f,-1.8302489519119263f,-0.19429926574230194f,-1.7818351984024048f,0.7989482283592224f,2.024789333343506f,-0.34412485361099243f,-3.059338092803955f,0.10319482535123825f,-0.6614995002746582f,0.2136087715625763f,-0.45982974767684937f,0.608289361000061f,-2.8377184867858887f,0.7676216959953308f,-1.5342905521392822f,-1.0134694576263428f,0.41548511385917664f,-0.24282032251358032f,-1.75166916847229f,0.6955306529998779f,0.9406816959381104f,-0.058385927230119705f,0.2923148572444916f,1.733177900314331f,-0.2251083105802536f,-0.39940139651298523f,0.7798540592193604f,-0.7776285409927368f,-0.19601377844810486f,-1.2052197456359863f,2.6707401275634766f,1.2911837100982666f,1.5733027458190918f,0.22617678344249725f,0.49920105934143066f,-0.45357024669647217f,-0.4225192070007324f,-0.1820744425058365f,1.827479600906372f,0.8189831972122192f,-0.9213781952857971f,0.7358074188232422f,-1.7606770992279053f,-1.2017607688903809f,-0.6463732719421387f,1.473232626914978f,-0.4455479383468628f,2.6956143379211426f,-0.5252562165260315f,1.5182214975357056f,0.6804697513580322f,2.1211538314819336f,-0.31251490116119385f,1.8279495239257812f,-1.3474072217941284f,2.5937159061431885f,-1.4066812992095947f,0.3015013337135315f,-0.9831823110580444f,0.2887194752693176f,2.2537569999694824f,-1.982358455657959f,-2.2078146934509277f,1.0564883947372437f,1.6803113222122192f,-0.12117383629083633f,-0.2202196717262268f,0.2206459790468216f,-0.6649414300918579f,-0.3950282335281372f,2.409527540206909f,-1.2752387523651123f,-1.207961082458496f,-1.076337218284607f,0.16702745854854584f,-0.1155608594417572f,-1.3942619562149048f,-2.357293128967285f,-2.18681001663208f,-1.0337178707122803f,1.7495557069778442f,-0.005466392729431391f,-1.8758262395858765f,2.853835344314575f,-1.5173481702804565f,1.1889642477035522f,-0.0660187229514122f,-1.680694341659546f,0.363336443901062f,2.2183523178100586f,0.46795645356178284f,-1.5152482986450195f,1.356208324432373f,-0.6398983597755432f,0.5238864421844482f,0.004342443775385618f,0.8190701603889465f,-0.5150106549263f,-1.3359683752059937f,-1.759390115737915f,1.0759705305099487f,3.1428048610687256f,2.023726463317871f,0.326369971036911f,1.6072118282318115f,1.3642075061798096f,0.35213083028793335f,-2.6878058910369873f,0.17996740341186523f,-0.8122718334197998f,0.8372750282287598f,-0.16408662497997284f,-0.6068595051765442f,0.2707754969596863f,-0.5195499062538147f,0.10310906916856766f,0.25244128704071045f,-0.2827296555042267f,1.3354332447052002f,-1.3949815034866333f,2.352799415588379f,2.4069724082946777f,1.4089754819869995f,-2.096554756164551f,1.1502338647842407f,-0.8065094351768494f,-2.2478854656219482f,0.18462367355823517f,-1.818781852722168f,0.39738908410072327f,-1.627217173576355f,-1.3550198078155518f,-2.691995620727539f,0.3083110749721527f,0.826305091381073f,1.4594485759735107f,1.141706109046936f,-2.587660074234009f,0.4764363765716553f,1.7055070400238037f,3.3801002502441406f,-1.0222452878952026f,-3.0679125785827637f,-2.001768112182617f,-0.26175057888031006f,-1.8462202548980713f,0.3704088628292084f,0.3864113986492157f,-1.029532790184021f,0.07875389605760574f,0.300691157579422f,1.0524307489395142f,1.752266526222229f,-1.1196815967559814f,-0.9916599988937378f,0.9152239561080933f,0.9345836639404297f,-0.9258922934532166f,-0.9479134678840637f,-1.1301461458206177f,0.4876202642917633f,-1.582200527191162f,-0.3520777225494385f,-1.5708245038986206f,1.6102477312088013f,0.8153449892997742f,0.11341787874698639f,0.6882153153419495f,0.14372582733631134f,1.8261293172836304f,-0.020491791889071465f,1.099184513092041f,0.8943491578102112f,-0.9090085625648499f,3.1812386512756348f,0.9320513606071472f,0.23712295293807983f,-1.53611421585083f,-0.7131375670433044f,1.6086055040359497f,3.5342423915863037f,0.7490654587745667f,-2.3271191120147705f,0.8752614259719849f,-0.551142692565918f,1.8264933824539185f,1.2181613445281982f,-1.1918367147445679f,-0.5539288520812988f,0.5569620132446289f,0.24633750319480896f,-1.6082193851470947f,-0.07153462618589401f,2.897545576095581f,-2.785818338394165f,-1.5625934600830078f,0.7113265991210938f,1.7737390995025635f,0.5081732273101807f,-0.6282455325126648f,-1.0050320625305176f,-0.338034063577652f,1.046899676322937f,-0.8111796975135803f,1.6559034585952759f,2.6259822845458984f,0.4029500484466553f,1.1132233142852783f,0.3349580466747284f,0.7714546918869019f,-0.7625781893730164f,0.8840692639350891f,1.5634710788726807f,-0.5455216765403748f,-1.9426573514938354f,1.8878189325332642f,1.8938957452774048f,-0.8768823742866516f,1.0929549932479858f,1.4622535705566406f,0.3886689245700836f,-2.578460216522217f,-0.2822395861148834f,1.4972126483917236f,0.715314507484436f,1.7155752182006836f,0.19479645788669586f,-1.6268551349639893f,-1.9346462488174438f,2.4315924644470215f,1.649289846420288f,-0.7315232753753662f,-1.4387502670288086f,-3.547239303588867f,-1.4070130586624146f,0.22279797494411469f,-1.288789987564087f,-2.2974603176116943f,1.3315078020095825f,3.0316131114959717f,1.252385139465332f,-1.7345771789550781f,0.897758960723877f,-0.9613736867904663f,0.5114290118217468f,0.3182455599308014f,-1.373213291168213f,-3.7820496559143066f,2.3509628772735596f,1.0895204544067383f,1.2403603792190552f,1.1783427000045776f,1.3085081577301025f,-1.4111733436584473f,1.0843185186386108f,-1.195738434791565f,-0.7764936089515686f,3.5519421100616455f,-1.8803247213363647f,1.9193100929260254f,1.6524063348770142f,0.9238660335540771f,-0.03785258159041405f,1.2355221509933472f,1.7830066680908203f,-0.9569810032844543f,0.678729236125946f,0.15262115001678467f,0.09307710826396942f,0.2625333368778229f,0.890073299407959f,0.33398374915122986f,-1.3255813121795654f,0.1027982085943222f,2.477109432220459f,1.050318717956543f,-1.8075971603393555f,-0.9824278950691223f,1.5343996286392212f,-1.140160322189331f,-0.8959351181983948f,-1.7384148836135864f,-0.19065403938293457f,1.3872525691986084f,1.249602198600769f,0.7522141933441162f,0.10511694848537445f,-1.2679742574691772f,0.24280387163162231f,1.9461150169372559f,-0.7044697403907776f,-0.06040739640593529f,-1.3626325130462646f,-2.5647308826446533f,0.04431462660431862f,-1.6851704120635986f,0.6821728944778442f,0.9966353178024292f,0.7310643792152405f,1.2785389423370361f,1.689997911453247f,-1.4682923555374146f,-0.003859227290377021f,-0.24507033824920654f,0.09597305208444595f,4.516735553741455f,-0.41783255338668823f,-2.6723034381866455f,-0.6281611919403076f,-0.5847184062004089f,-0.344657838344574f,-3.5593247413635254f,0.8497494459152222f,1.4388874769210815f,0.5711674094200134f,0.7644138336181641f,0.5011810064315796f,1.8931005001068115f,1.644801139831543f,-1.2678821086883545f,0.709842324256897f,1.0619428157806396f,-2.34869647026062f,1.3830647468566895f,0.8829596638679504f,-0.9228840470314026f,0.1949843317270279f,0.27393990755081177f,0.3212474286556244f,0.9679129123687744f,-1.1004770994186401f,-1.8216320276260376f,-0.5510002374649048f,-0.055797968059778214f,1.117026686668396f,2.3889331817626953f,-0.3876092731952667f,0.8209916949272156f,0.715170681476593f,-1.786211609840393f,1.3464287519454956f,-0.4755670428276062f,-1.2318729162216187f,2.29781436920166f,0.748175859451294f,1.1232821941375732f,0.02984163910150528f,-1.756923794746399f,0.1499088853597641f,-0.1810513585805893f,1.5725464820861816f,-1.9619083404541016f,1.0753235816955566f,1.3511784076690674f,-1.696892261505127f,0.33269229531288147f,-1.9650635719299316f,-1.518283724784851f,-1.400069236755371f,-0.8230807185173035f,-0.48736628890037537f,-0.30933111906051636f,0.0427413173019886f,0.0878138542175293f,-0.02197284996509552f,-0.6896201968193054f,-1.9806526899337769f,-0.1831693947315216f,1.676924467086792f,1.0791462659835815f,-3.3999640941619873f,2.6640865802764893f,-0.14670538902282715f,-0.32813596725463867f,-0.774732768535614f,0.8502143025398254f,1.8314930200576782f,2.487074136734009f,0.5429370403289795f,1.1913360357284546f,1.1895240545272827f,-1.330007791519165f,-1.3568949699401855f,-1.971016526222229f,-0.6980892419815063f,-0.42617714405059814f,-0.47945335507392883f,0.0639493316411972f,0.18781477212905884f,0.08974052220582962f,0.17591363191604614f,1.562923550605774f,-0.29234468936920166f,0.5045677423477173f,1.9988223314285278f,-1.763621211051941f,0.3082877993583679f,-1.4328500032424927f,-1.0333607196807861f,-0.2687695622444153f,-0.3470788300037384f,-0.2919508218765259f,-0.6029711961746216f,-1.011518955230713f,-0.1127331405878067f,0.617191731929779f,0.6279710531234741f,-0.4947280287742615f,1.2511000633239746f,0.2144457995891571f,-0.24620510637760162f,1.7060749530792236f,-0.06562678515911102f,0.5444629192352295f,-0.6482828259468079f,-0.15395165979862213f,2.0070033073425293f,0.07289710640907288f,-0.8905959129333496f,-1.7006821632385254f,-1.543819785118103f,-1.5119677782058716f,-0.850482165813446f,-1.3783327341079712f,1.954518437385559f,0.5221894383430481f,-0.7815559506416321f,-0.2780840992927551f,1.150942087173462f,2.2107412815093994f,-0.5052920579910278f,0.07252125442028046f,-0.9795432686805725f,-2.5484237670898438f,-2.3273158073425293f,1.796671986579895f,-1.3103492259979248f,0.8435393571853638f,0.9423667192459106f,-0.23038452863693237f,3.6759204864501953f,0.07130987197160721f,-0.13317713141441345f,0.8875678777694702f,1.5166003704071045f,-0.958467960357666f,1.5100135803222656f,-0.29823750257492065f,0.3678029775619507f,0.9245356917381287f,0.7491635680198669f,-0.24521367251873016f,-1.3737494945526123f,0.6075634360313416f,1.62697434425354f,-0.7024423480033875f,0.16578803956508636f,1.0040696859359741f,-1.483562707901001f,-1.1084429025650024f,-0.7987481355667114f,2.3205621242523193f,0.6552507877349854f,-0.4122219979763031f,-0.2146889567375183f,1.9386168718338013f,-0.7173460721969604f,0.29049691557884216f,0.7216547727584839f,-1.9364254474639893f,-0.19357572495937347f,-0.48431476950645447f,-0.2856452167034149f,-2.254819393157959f,0.2528669834136963f,1.8089754581451416f,-1.1742147207260132f,0.2788794934749603f,-2.2664453983306885f,-0.46947139501571655f,-1.4014986753463745f,-1.737257719039917f,0.7481361031532288f,-0.7183914184570312f,0.42027169466018677f,0.4952571392059326f,0.3846296966075897f,2.5864243507385254f,-1.4511109590530396f,0.10881330072879791f,-0.9138850569725037f,-0.011691602878272533f,-0.5046843886375427f,1.1015409231185913f,2.316762685775757f,0.6702207326889038f,0.9394569396972656f,1.624356746673584f,-1.9784495830535889f,-0.7226434946060181f,-0.986548900604248f,-2.1642487049102783f,0.10793806612491608f,1.215057134628296f,0.2671431601047516f,-1.715173363685608f,-0.37608394026756287f,0.9914429187774658f,0.1271596997976303f,-0.4615250527858734f,-0.09766251593828201f,-0.6748059988021851f,-0.7587789297103882f,-2.632922887802124f,0.7368954420089722f,-0.21118046343326569f,-2.1528215408325195f,-1.3144184350967407f,1.0042856931686401f,-3.070420742034912f,0.08713331073522568f,-2.4864182472229004f,-1.7762047052383423f,1.0392069816589355f,-0.5914485454559326f,1.45246422290802f,-1.193177580833435f,-2.351524829864502f,-0.5898820757865906f,0.16032813489437103f,-0.7645265460014343f,0.9645717144012451f,1.5971659421920776f,2.6681971549987793f,0.07472927123308182f,-0.5191619396209717f,-0.21785588562488556f,0.43060681223869324f,1.2107387781143188f,0.10460438579320908f,0.29756176471710205f,0.27754008769989014f,-1.0926440954208374f,-1.0083376169204712f,0.5583615899085999f,-0.4788235127925873f,0.8426440954208374f,-0.25406184792518616f,1.2863119840621948f,0.6490442752838135f,-1.8230688571929932f,-1.0565450191497803f,-2.388401508331299f,1.6805850267410278f,-1.0038750171661377f,-1.287018895149231f,-0.6502172946929932f,2.0488107204437256f,-0.482536256313324f,0.9361154437065125f,2.768927574157715f,0.6261517405509949f,-0.47865691781044006f,-0.6790534853935242f,1.3588669300079346f,1.540905237197876f,-0.33388185501098633f,-0.2637496888637543f};
alignas(16) float batch_normalization_4_A[] = {0.08643117547035217f,-0.7197518348693848f,0.1716439425945282f,0.1538207232952118f,-0.7127913236618042f,0.46817052364349365f,0.4581816494464874f,-1.6354225873947144f,0.13127365708351135f,0.6018862128257751f,-0.7794498205184937f,-0.07406474649906158f,0.9375619292259216f,-0.09255336225032806f,-0.2046114206314087f,0.15719763934612274f,-0.07226341962814331f,1.0153172016143799f,-0.5033321380615234f,-0.6542841792106628f,-0.2502657473087311f,0.9001913666725159f,1.2850457429885864f,1.4429693222045898f,-0.006922195665538311f,0.28036001324653625f,-0.35826826095581055f,-0.10542657971382141f,0.15151327848434448f,-0.24982404708862305f,-0.27248498797416687f,0.35769277811050415f};
alignas(16) float conv2d_2_internal_1_W[] = {-0.341446191072464f,-0.13438904285430908f,0.9726850390434265f,0.17179907858371735f,0.295221745967865f,-0.8008561134338379f,0.40268978476524353f,0.17556141316890717f,-0.34713542461395264f,0.3707357347011566f,-0.27195480465888977f,0.20741860568523407f,-0.2466258853673935f,-0.6727361083030701f,-0.9063344597816467f,-0.4770875573158264f,0.6377209424972534f,0.5129608511924744f,-0.5409179329872131f,0.5802541375160217f,0.2584439218044281f,-0.3979607820510864f,0.477120965719223f,-0.3400157690048218f,0.027005057781934738f,0.7591758966445923f,-0.347927987575531f,-0.8787458539009094f,-0.6195445656776428f,0.1306658834218979f,0.852494478225708f,-0.05627728998661041f,0.14698146283626556f,-0.17320503294467926f,0.9928070902824402f,0.539827823638916f,-0.5635107159614563f,0.38086390495300293f,0.49767717719078064f,-0.28128084540367126f,-0.02599940076470375f,0.48389139771461487f,0.3190460801124573f,0.6828569769859314f,-0.1613469123840332f,-0.04215491563081741f,0.7178090810775757f,-0.9111343622207642f,0.6513116359710693f,-0.7765440940856934f,-0.33046236634254456f,-0.32094934582710266f,0.5684894919395447f,1.1202343702316284f,-0.10436971485614777f,0.2866196036338806f,-0.5009759068489075f,0.9307345747947693f,-0.3025103807449341f,-0.5426816940307617f,-0.6525282859802246f,-0.8101829886436462f,0.19875149428844452f,-0.16091980040073395f,-0.0409449003636837f,0.5145431756973267f,-0.35682135820388794f,0.052792176604270935f,0.39579588174819946f,0.5644106268882751f,-0.7970029711723328f,0.09060356765985489f,0.6251004934310913f,-0.10335177183151245f,0.3608117997646332f,-1.0462870597839355f,-0.0881253331899643f,-0.9826253056526184f,0.0804399773478508f,-0.34381064772605896f,0.13670486211776733f,0.5855644345283508f,0.7424989938735962f,-0.8944708108901978f,-0.34229031205177307f,-0.2776264548301697f,0.5906844139099121f,-0.8557826280593872f,-0.2537294626235962f,-0.6885368824005127f,0.23044663667678833f,-0.36306899785995483f,0.9487941861152649f,0.20851077139377594f,-0.6052219271659851f,-0.47448164224624634f,-0.12173191457986832f,-0.26523932814598083f,0.3434879779815674f,-0.3145788311958313f,-0.2090100795030594f,-1.1046925783157349f,0.24625101685523987f,-0.25239554047584534f,0.246669739484787f,-1.1689629554748535f,0.70644611120224f,-0.7507252097129822f,1.0689291954040527f,0.048167984932661057f,-0.0866101011633873f,-0.49482858180999756f,0.28525981307029724f,0.35117125511169434f,-0.051524221897125244f,-0.379465788602829f,0.3145832121372223f,0.4672817885875702f,-1.0572755336761475f,0.30342304706573486f,-0.6204210519790649f,-1.563339352607727f,0.0838705375790596f,0.005229237023741007f,0.4532981514930725f,0.8577542901039124f,-0.030626947060227394f,-0.09371427446603775f,0.5108707547187805f,-1.6507563591003418f,0.005916593596339226f,0.40054553747177124f,0.9983484745025635f,-0.33421483635902405f,0.6894177198410034f,0.17021766304969788f,0.8618959188461304f,0.6042351126670837f,-1.0516282320022583f,0.3449264168739319f,-0.27214518189430237f,0.29941701889038086f,0.5304666757583618f,0.9285227656364441f,-0.44481515884399414f,-0.718681275844574f,0.6025442481040955f,0.14580781757831573f,-0.14360597729682922f,0.23529838025569916f,-1.0182781219482422f,-0.360116571187973f,0.18939611315727234f,0.9505027532577515f,0.10573692619800568f,0.3853358030319214f,-0.31248706579208374f,0.07807506620883942f,-1.1267744302749634f,0.3918938636779785f,-0.7249256372451782f,0.279239684343338f,1.090807318687439f,-0.6656158566474915f,0.2257375568151474f,-0.5664457082748413f,0.6416255831718445f,-0.21315932273864746f,0.30128130316734314f,-1.6577706336975098f,-0.4632825255393982f,0.2371695190668106f,-0.05482646822929382f,0.17834550142288208f,-0.7160024642944336f,0.5004715323448181f,-1.1202478408813477f,1.4814379215240479f,-0.3955550193786621f,0.5736152529716492f,-0.07802941650152206f,0.21982866525650024f,0.07144147157669067f,1.2112222909927368f,1.572806477546692f,-0.3816397488117218f,0.45242705941200256f,0.26450982689857483f,-0.5796884298324585f,0.6202146410942078f,-0.23565338551998138f,0.2839750051498413f,-0.09407613426446915f,-0.06109710410237312f,0.27677565813064575f,0.47058993577957153f,-0.2288770079612732f,-0.6218494772911072f,-0.31333523988723755f,0.6351628303527832f,-0.3188096284866333f,-0.6359145045280457f,0.3417891561985016f,-0.5180537104606628f,0.6618003845214844f,-0.7236838340759277f,0.880081295967102f,-0.0675707682967186f,-0.4581965506076813f,0.9999143481254578f,-0.5384446978569031f,-0.20620930194854736f,-0.08472632616758347f,0.5253497362136841f,0.2866578996181488f,-1.0442243814468384f,-0.20709112286567688f,0.5296941995620728f,-0.48311564326286316f,-0.48228734731674194f,0.2131468951702118f,-0.8189176917076111f,0.515981137752533f,0.2981019914150238f,-0.34182044863700867f,-0.34347110986709595f,0.31645041704177856f,0.4056602120399475f,0.41760802268981934f,0.7082815766334534f,-0.4309837520122528f,-0.17202860116958618f,-0.604999840259552f,-0.5325969457626343f,0.9689300060272217f,-0.7763242125511169f,-0.030486468225717545f,0.04327904433012009f,0.10198105126619339f,0.710014820098877f,-0.25901591777801514f,0.8753735423088074f,0.22817642986774445f,-0.24195364117622375f,1.2213997840881348f,0.38288238644599915f,0.5333847403526306f,0.03767739608883858f,0.6758114099502563f,0.9857218861579895f,1.007346510887146f,-0.21398481726646423f,-0.015533925965428352f,0.4682389795780182f,0.7064847350120544f,-1.5953923463821411f};
alignas(16) float batch_normalization_5_A[] = {0.04103386402130127f,0.4065544307231903f,-0.2666769325733185f,0.8184987902641296f,0.04569077491760254f,-0.21212399005889893f,0.417671263217926f,0.18433359265327454f};
alignas(16) float separable_conv2d_3_internal_0_VALUES[] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
alignas(16) float separable_conv2d_3_internal_1_W[] = {-0.002399261575192213f,0.026364034041762352f,0.004967386834323406f,-0.14067640900611877f,0.05287771299481392f,-0.07231485843658447f,0.07879193872213364f,0.07305396348237991f,-0.0345233716070652f,0.038068246096372604f,0.12774808704853058f,-0.04941666126251221f,-0.07794341444969177f,0.1450042426586151f,0.012061195448040962f,0.061327144503593445f,0.1082112118601799f,-0.04672882705926895f,-0.07016053050756454f,0.17985624074935913f,0.12045518308877945f,0.1015600860118866f,-0.07778659462928772f,0.007357545662671328f,0.008341056294739246f,-0.0004288231721147895f,0.0658288523554802f,-0.08567932993173599f,0.04940781742334366f,0.001250254805199802f,-0.13134247064590454f,-0.020515872165560722f,-0.05863819643855095f,-0.025725113227963448f,-0.03598713129758835f,-0.13313409686088562f,0.0024944969918578863f,0.03839094936847687f,0.07252130657434464f,0.0223713256418705f,0.06784647703170776f,0.08413880318403244f,0.013372080400586128f,-0.07491415739059448f,-0.1155739277601242f,0.06056033819913864f,0.08077901601791382f,0.005216020159423351f,-0.055292416363954544f,-0.01708730310201645f,-0.02263256534934044f,0.0856713131070137f,0.0636870488524437f,0.05417892336845398f,-0.19191573560237885f,0.10683048516511917f,0.04158962890505791f,0.03304724767804146f,0.12048666924238205f,-0.10598579049110413f,0.01744866743683815f,-0.055267516523599625f,-0.1326049417257309f,0.04711448401212692f,-0.12267380207777023f,-0.022536523640155792f,-0.11313702911138535f,0.010126124136149883f,0.01958048902451992f,0.11497332900762558f,0.06270971149206161f,-0.00656320434063673f,0.07582243531942368f,0.06883355230093002f,0.003496826160699129f,-0.036903392523527145f,-0.020524227991700172f,-0.05118659511208534f,0.07701867818832397f,0.03900061547756195f,0.06846613436937332f,0.04390113055706024f,0.03202541917562485f,-0.020839860662817955f,-0.011666272766888142f,0.02801363915205002f,-0.06464628130197525f,0.008683444000780582f,0.1642206311225891f,-0.04482497274875641f,0.02013346739113331f,0.036295026540756226f,0.05704038217663765f,-0.10963989049196243f,-0.03905269503593445f,0.06351540237665176f,-0.00999092124402523f,-0.004725603852421045f,-0.016507692635059357f,-0.03600000962615013f,0.08176682144403458f,-0.06576648354530334f,-0.006430847570300102f,-0.05423643812537193f,-0.06606430560350418f,0.016949858516454697f,0.05217677727341652f,-0.013398684561252594f,0.024737566709518433f,0.06180761009454727f,-0.019770614802837372f,0.12968315184116364f,0.0043319957330822945f,0.026513589546084404f,-0.10592080652713776f,-0.06989835947751999f,0.10691335052251816f,-0.058626800775527954f,-0.09491270035505295f,-0.04832039773464203f,0.02591017819941044f,0.07412846386432648f,-0.025017881765961647f,-0.05265675112605095f,0.02887680009007454f,0.1354272961616516f,-0.0006726800929754972f,0.06598133593797684f,0.04010472819209099f,0.006814479362219572f,-0.1155071035027504f,-0.03917716443538666f,0.07854994386434555f,-0.01265213917940855f,-0.06871464103460312f,-0.07719650119543076f,-0.0466904416680336f,0.0650385394692421f,-0.01935218647122383f,-0.05495496466755867f,0.030925646424293518f,0.019455911591649055f,0.04521184787154198f,0.08438605815172195f,-0.07089744508266449f,-0.045981515198946f,-0.013534896075725555f,-0.07186777889728546f,0.0013163959374651313f,-0.13117212057113647f,-0.12918074429035187f,-0.10134650766849518f,0.08008886873722076f,0.04573977366089821f,-0.04110737144947052f,-0.045599404722452164f,0.020572081208229065f,0.04767002910375595f,-0.0036217719316482544f,0.09160629659891129f,-0.09175118803977966f,-0.004919120110571384f,-0.06940291821956635f,0.02905449829995632f,0.07043015211820602f,0.06388946622610092f,0.007035327143967152f,-0.08233888447284698f,0.026710860431194305f,0.0002805646799970418f,0.005632173269987106f,-0.08122394233942032f,0.08295171707868576f,-0.03764381632208824f,0.06692064553499222f,0.04407105594873428f,0.14799806475639343f,0.01764059066772461f,0.035987965762615204f,-0.07884576171636581f,-0.009116631001234055f,-0.04070625826716423f,-0.0798214003443718f,-0.06449788063764572f,0.12494780868291855f,0.03660672530531883f,-0.08331526070833206f,0.022640671581029892f,-0.054400306195020676f,-0.06698011606931686f,0.025631407275795937f,0.13356833159923553f,0.041946738958358765f,-0.11066768318414688f,-0.014506453648209572f,0.0575011670589447f,0.048659808933734894f,0.012751409783959389f,0.02164202556014061f,-0.008907676674425602f,-0.03539328649640083f,0.08994364738464355f,-0.07023582607507706f,-0.036040786653757095f,0.0685092881321907f,0.1165931224822998f,-0.011721955612301826f,-0.08397886157035828f,0.03630252555012703f,-0.024981940165162086f,-0.09276250749826431f,-0.10967689007520676f,-0.03633131459355354f,0.09128706157207489f,0.016842244192957878f,-0.17595788836479187f,0.0024921332951635122f,0.02687699720263481f,-0.09104312211275101f,-0.050721123814582825f,0.07145524024963379f,0.03351059928536415f,0.015149886719882488f,0.014325669035315514f,0.07695598155260086f,-0.1469753533601761f,0.0581015944480896f,0.011515882797539234f,0.05910571664571762f,-0.015348701737821102f,-0.08666639775037766f,0.10850603878498077f,-0.06314333528280258f,-0.01837097853422165f,0.02955746464431286f,-0.03921491652727127f,0.06764964759349823f,-0.009776336140930653f,0.048437222838401794f,-0.08597228676080704f,-0.020810073241591454f,-0.13141517341136932f,0.02668030560016632f,-0.043219469487667084f,-0.1484529972076416f,-0.012703991495072842f,-0.0465129055082798f,-0.08685743808746338f,-0.006405949126929045f,-0.09045977145433426f,-0.07142340391874313f,-0.05361170321702957f,0.13464392721652985f,-0.011295117437839508f,0.07211016118526459f,0.03857063502073288f,-0.037016429007053375f,-0.019796904176473618f,0.164045050740242f,0.008413843810558319f,0.03553229197859764f,0.0413707010447979f,-0.0338834784924984f,0.09828729182481766f,-0.0075919488444924355f,-0.0979810506105423f,-0.05381358414888382f,-0.11776315420866013f,0.03922443836927414f,-0.005012029316276312f,0.10783609002828598f,-0.04636464640498161f,0.021616419777274132f,-0.11469060927629471f,0.0926092192530632f,0.028903014957904816f,-0.1137564480304718f,0.07421831041574478f,-0.059621602296829224f,-0.033978015184402466f,-0.024527421221137047f,-0.15205897390842438f,0.01702274940907955f,-0.06709656119346619f,0.035417236387729645f,0.028144383803009987f,0.101647287607193f,-0.003471638774499297f};
alignas(16) float separable_conv2d_3_internal_2_W[] = {0.419147789478302f,-0.061999596655368805f,1.0784837007522583f,-0.020877722650766373f,-0.311381459236145f,-0.7229763269424438f,-0.15814615786075592f,0.05389939248561859f,0.4218025207519531f,-0.22491520643234253f,-0.07396122068166733f,-0.38922297954559326f,-0.07506181299686432f,-0.013042572885751724f,-0.11976593732833862f,-0.14129087328910828f,0.022759975865483284f,0.751371443271637f,0.41865673661231995f,-1.3696805238723755f,-0.09846427291631699f,0.2145531326532364f,-0.031588710844516754f,-0.049846649169921875f,0.3544970452785492f,0.042738158255815506f,-0.6135923266410828f,0.48936957120895386f,0.46313950419425964f,0.4205531179904938f,-0.6657696962356567f,0.2930431067943573f,-0.33993038535118103f,0.27046534419059753f,-0.10182907432317734f,-0.22562605142593384f,-0.449535608291626f,0.4769798517227173f,-0.387399822473526f,0.3619953989982605f,-1.0558351278305054f,1.1966662406921387f,-0.41899076104164124f,-0.07309985160827637f,-0.5472670793533325f,0.7131256461143494f,0.0491141639649868f,0.2114633321762085f,-0.0006339317769743502f,-0.3679686486721039f,1.1346606016159058f,-0.8428131937980652f,0.7646881341934204f,-0.2489650994539261f,-1.3586739301681519f,-0.12125590443611145f,0.7124946713447571f,0.4908503293991089f,0.6358619928359985f,0.03454020246863365f,-0.02997693419456482f,-0.7672507166862488f,0.6947670578956604f,-0.08249472826719284f,-0.17263202369213104f,0.047048419713974f,-0.14325106143951416f,-0.6419191956520081f,-0.8641772270202637f,-0.015319213271141052f,0.18162624537944794f,-0.13506820797920227f,-0.47913873195648193f,0.39251717925071716f,0.1850474327802658f,-1.368539810180664f,-0.21683171391487122f,-0.04173213616013527f,-0.4167921841144562f,0.11338871717453003f,-0.7787506580352783f,0.9165348410606384f,0.27470648288726807f,-0.2958737313747406f,-1.1914315223693848f,-0.48781412839889526f,-0.08998464792966843f,-0.387306272983551f,-0.09393852204084396f,0.5563562512397766f,0.23670735955238342f,-0.29109418392181396f,-0.3825397491455078f,0.5562368631362915f,-0.28327676653862f,-0.4374055862426758f,0.026881393045186996f,-0.7710821032524109f,-0.8338867425918579f,-0.15156729519367218f,0.6427901983261108f,-0.05311323702335358f,-0.5250663757324219f,-0.9127981662750244f,-0.20244799554347992f,-0.32385188341140747f,0.4902030825614929f,-0.0767766609787941f,0.4039020836353302f,0.22768796980381012f,-1.1774375438690186f,0.40289536118507385f,0.7916286587715149f,-0.1784524768590927f,-0.14505153894424438f,0.47043105959892273f,0.062157321721315384f,-0.07002273947000504f,-0.8721629977226257f,0.30279865860939026f,-0.4636286497116089f,-0.19669568538665771f,0.00015407353930640966f,-1.280332088470459f,0.30121076107025146f,-0.1768457144498825f,1.1427123546600342f,0.48567888140678406f,-0.22362275421619415f,-0.6669136881828308f,0.07346061617136002f,0.39231744408607483f,0.5201014280319214f,0.32954901456832886f,0.2734435796737671f,0.4478786885738373f,-0.13983476161956787f,-0.010540992021560669f,-0.12148115783929825f,-0.20122620463371277f,0.5966992378234863f,0.013172229751944542f,-0.506619393825531f,-0.08405082672834396f,-0.31377899646759033f,0.01106028351932764f,-0.5240634679794312f,-0.29886195063591003f,-0.1999303251504898f,-0.10762736201286316f,-0.2361402064561844f,-0.04873793572187424f,0.5524118542671204f,-0.5175589919090271f,0.24895860254764557f,0.6720194220542908f,0.2972372770309448f,-0.8022002577781677f,0.048020027577877045f,0.4359849691390991f,-0.09033829718828201f,-0.03583243489265442f,0.35275474190711975f,-0.7669106125831604f,0.9625309705734253f,0.4683517515659332f,0.3424682319164276f,-0.25905841588974f,-0.9779976606369019f,-0.1098119392991066f,-0.35290804505348206f,-0.4222399592399597f,-0.5010399222373962f,-0.6091607809066772f,0.05816105380654335f,-0.07697977125644684f,-0.058815013617277145f,-0.26173219084739685f,0.37663280963897705f,-0.9554030299186707f,0.6978445649147034f,0.09353135526180267f,0.08836477249860764f,0.035161785781383514f,-0.02924729324877262f,0.8409658670425415f,0.09396420419216156f,0.16920940577983856f,-0.9746301174163818f,0.11879243701696396f,0.6034599542617798f,0.2720828950405121f,0.3827264904975891f,-0.4614485204219818f,0.20793220400810242f,0.5218136310577393f,-0.4380965530872345f,-0.16115538775920868f,-0.07169187813997269f,0.7879377007484436f,-0.4647272229194641f,-0.47907283902168274f,0.3352513313293457f,0.05456734448671341f,-0.46888473629951477f,-0.0854855552315712f,-0.11302637308835983f,0.3950670063495636f,0.7983700037002563f,0.20205003023147583f,-0.11547715216875076f,-0.9576391577720642f,-0.2597159445285797f,-0.25956904888153076f,-0.029753295704722404f,0.2495545744895935f,0.40436387062072754f,0.11823747307062149f,-0.24685770273208618f,0.9311484694480896f,-0.9962456822395325f,0.3425374925136566f,0.5691664218902588f,-0.28597211837768555f,-0.05165814608335495f,-0.9801615476608276f,-0.470641553401947f,0.7679279446601868f,0.12480480968952179f,-0.9204040765762329f,-0.36693075299263f,-0.7393296957015991f,0.021171558648347855f,-0.582209587097168f,0.13866977393627167f,-0.3391161561012268f,-0.24807755649089813f,0.3826252818107605f,0.1769433170557022f,0.08116541802883148f,-0.05066656321287155f,0.03861906751990318f,-0.148073211312294f,0.7197352051734924f,-0.0833057165145874f,0.08464332669973373f,0.17533919215202332f,-0.17355544865131378f,0.32769227027893066f,0.06697927415370941f,-0.8743752241134644f,0.3135126233100891f,0.38975927233695984f,-0.2577027678489685f,-0.49002861976623535f,0.0767694041132927f,-0.12865719199180603f,-0.5763993263244629f,0.08341757208108902f,-0.894135594367981f,0.05421019718050957f,0.08973319828510284f,-0.7022841572761536f,0.06614464521408081f,0.9498262405395508f,0.207223579287529f,0.46366897225379944f,1.0917682647705078f,-0.012137585319578648f,-0.1680164635181427f,0.034107621759176254f,-0.4716583788394928f,1.1977840662002563f,0.7570047974586487f,0.9157769680023193f,-0.3833433985710144f,-0.688157320022583f,1.1805223226547241f,0.2594378888607025f,0.7368957996368408f,-0.04052659496665001f,0.7492865920066833f,-0.7424027919769287f,-0.15432600677013397f,0.9539304375648499f,-0.5214121341705322f,-0.21627181768417358f,0.11498245596885681f,-0.3257882595062256f,0.16213136911392212f,-0.07644490897655487f,-0.7135739326477051f,0.8556203842163086f,0.6062910556793213f,0.2766740918159485f,0.43746593594551086f,-0.03284168615937233f,0.36427655816078186f,0.3196510076522827f,0.40163177251815796f,0.4257040321826935f,0.6071204543113708f,-0.4458610415458679f,0.8106970191001892f,0.40844470262527466f,-0.10734786838293076f,0.0669199600815773f,1.1258151531219482f,-0.8893470764160156f,0.56973797082901f,0.07763989269733429f,-0.462631493806839f,-0.7164669036865234f,0.05254792049527168f,-0.4157371520996094f,0.4493996202945709f,-0.9694018363952637f,-1.0911880731582642f,-1.3978514671325684f,-0.29284340143203735f,-0.054827187210321426f,0.19994114339351654f,-0.34275415539741516f,0.23805402219295502f,-0.299839586019516f,-0.6030729413032532f,-0.19824106991291046f,-0.09500103443861008f,0.7324860692024231f,0.03535907715559006f,0.33708375692367554f,0.5390077233314514f,-1.024757742881775f,0.25588878989219666f,0.6475339531898499f,-0.06406515091657639f,-0.5537332892417908f,0.40279895067214966f,1.0276626348495483f,-1.3004807233810425f,0.2810467779636383f,0.25256916880607605f,-0.528085470199585f,0.07910918444395065f,0.37154003977775574f,-0.6748767495155334f,0.06930550932884216f,0.006218243855983019f,0.06618356704711914f,0.21427512168884277f,-0.015275495126843452f,0.10332516580820084f,0.9460119605064392f,0.46571823954582214f,-0.3210221827030182f,-0.12718525528907776f,-1.6378222703933716f,0.8110767006874084f,0.44881758093833923f,0.7437285780906677f,-0.6234971284866333f,-0.3231405019760132f,-0.27298110723495483f,0.7935175895690918f,0.12013648450374603f,-0.5665650367736816f,-0.3265698552131653f,0.2750750184059143f,-0.07650261372327805f,0.821749746799469f,0.5610150098800659f,0.13783301413059235f,-0.1513512283563614f,0.04777292162179947f,-0.14451007544994354f,-0.9156851172447205f,0.29792892932891846f,0.0648040622472763f,0.22649671137332916f,-0.38595765829086304f,-0.045208320021629333f,0.8349863886833191f,-0.034461889415979385f,1.3711916208267212f,-0.18357789516448975f,0.14475421607494354f,-1.5736619234085083f,-1.092552661895752f,-1.3117363452911377f,0.2130115032196045f,0.31665143370628357f,-1.5131155252456665f,-0.16904787719249725f,-0.5658970475196838f,0.09976642578840256f,0.33626535534858704f,-0.05824602022767067f,-0.13748712837696075f,0.33932480216026306f,-0.002096186624839902f,-0.049136482179164886f,-0.15586581826210022f,0.16925862431526184f,-0.4535505473613739f,0.05756369233131409f,-0.4104292392730713f,0.4094538390636444f,0.1415276676416397f,-0.1105203852057457f,0.26980775594711304f,-0.8861529231071472f,0.8178533911705017f,-0.400764137506485f,0.42787647247314453f,0.35882648825645447f,0.8270921111106873f,0.4311530590057373f,0.25699782371520996f,-0.25917789340019226f,0.03611643239855766f,0.2508048713207245f,-0.05237000808119774f,0.14368292689323425f,0.2839038670063019f,0.45760640501976013f,-0.516631543636322f,-0.8828863501548767f,-0.25857654213905334f,-0.8059766888618469f,1.1320465803146362f,-0.6263585090637207f,-0.37184688448905945f,0.5086579918861389f,-0.08086207509040833f,0.48777663707733154f,-0.6322726011276245f,-1.2985013723373413f,-0.9423994421958923f,0.3981223404407501f,-0.3271213173866272f,-0.17283768951892853f,0.11545870453119278f,0.04233495518565178f,-0.3278065621852875f,-0.05092509090900421f,1.2101473808288574f,0.01706801727414131f,-0.4883556365966797f,-0.835166335105896f,0.3695284128189087f,-0.6711539626121521f,-0.1955035775899887f,-0.327012836933136f,0.7580747008323669f,-0.42858198285102844f,-0.10001340508460999f,-0.37123119831085205f,0.2430158257484436f,-1.1256372928619385f,-0.24686603248119354f,1.0218180418014526f,-0.7866530418395996f,-1.0786563158035278f,0.5464679002761841f,-0.04762575402855873f,-0.5732612013816833f,-0.9988019466400146f,-0.42954063415527344f,0.6498590707778931f,-0.043073125183582306f,0.03882927820086479f,0.3359687626361847f,-0.4207160770893097f,0.6001020669937134f,-0.3267931342124939f,0.6525745391845703f,-0.8714025616645813f,0.1929691731929779f,0.30946096777915955f,-0.1506551206111908f,-1.474021553993225f,0.5495914816856384f,0.18130607903003693f,0.7410098910331726f,-0.8715946078300476f,0.6454034447669983f,-0.41460320353507996f,0.10050775855779648f,0.7435814142227173f,-0.4381006956100464f,0.297196626663208f,0.16238725185394287f,-0.3447258770465851f,-0.08686738461256027f,-0.38451722264289856f,0.831091046333313f,0.010173408314585686f,-0.9834464192390442f,-0.1293630450963974f,-0.18925140798091888f,0.32222235202789307f,0.01331207063049078f,-0.15061472356319427f,1.020777702331543f,0.10831623524427414f,0.5940604209899902f,-0.3582913279533386f,-1.1519675254821777f,-0.9104044437408447f,0.09497185796499252f,-0.47681909799575806f,-0.586609423160553f,-0.02973756194114685f,-0.07676710933446884f,-0.16224980354309082f,0.4641452431678772f,-0.1458437740802765f,0.6031321287155151f,-0.31094810366630554f,-0.5645474791526794f,-0.08148010075092316f,-0.016479354351758957f,0.526231586933136f,-0.03685596585273743f,0.48775312304496765f,-0.9019927978515625f,-0.3499926030635834f,0.43046027421951294f,0.666521430015564f,-1.3522236347198486f,-0.5400540828704834f,-0.9370620250701904f,-0.12137926369905472f,1.4093999862670898f,-0.01105824951082468f,-0.512106716632843f,1.4442880153656006f,1.281062126159668f,-0.7235128879547119f,-0.20309066772460938f,-0.6919283270835876f,0.7141647934913635f,0.9847156405448914f,-0.9920486807823181f,-0.20100276172161102f,-0.29265716671943665f,0.04666343703866005f,1.3366748094558716f,0.7774012088775635f,0.6641412973403931f,-0.459673672914505f,-0.13562233746051788f,-1.2727937698364258f,-0.8917622566223145f,0.0681249350309372f,-0.2289939522743225f,0.09439486265182495f,0.22686590254306793f,0.5162136554718018f,0.44441142678260803f,0.3190915584564209f,0.5312031507492065f,0.25955453515052795f,-0.5514153242111206f,-0.3341459631919861f,0.7118397951126099f,0.6054700613021851f,-0.426426500082016f,-0.8012773394584656f,-0.1703653633594513f,0.046827081590890884f,-0.6411393880844116f,-0.3380737900733948f,-0.5465600490570068f,-1.11488676071167f,1.06255304813385f,0.05164320766925812f,-0.8833640217781067f,-0.6629822254180908f,-1.3382477760314941f,-0.1565658003091812f,-0.21459388732910156f,0.5908514261245728f,-0.9943070411682129f,0.4752797782421112f,0.3842070996761322f,0.9792065620422363f,-0.983851969242096f,-0.16977711021900177f,0.27619579434394836f,-0.6984020471572876f,-0.02958347275853157f,-0.0722988098859787f,-1.2746474742889404f,0.8504561185836792f,-0.20880678296089172f,-0.2919204533100128f,0.6433437466621399f,-0.2570357322692871f,0.13102415204048157f,-0.32745417952537537f,-0.1424628645181656f,0.9584648609161377f,0.7798396348953247f,0.10725425183773041f,0.16466158628463745f,-1.0828750133514404f,-0.7557595372200012f,0.022322507575154305f,-0.0288480743765831f,-0.008717015385627747f,-0.12283448874950409f,0.635585367679596f,0.07166941463947296f,0.6946116089820862f,0.15941846370697021f,-0.4753651022911072f,-0.11191485822200775f,-0.0847993791103363f,-0.020220894366502762f,0.5585983395576477f,0.6811420321464539f,-0.33371731638908386f,0.6214871406555176f,0.3187306225299835f,0.15842361748218536f,-0.3070838749408722f,0.1734369695186615f,0.39107751846313477f,-0.6998541355133057f,-0.010181610472500324f,0.005113009363412857f,0.4428184926509857f,-0.8036510348320007f,0.5710378885269165f,0.2704660892486572f,0.9827274084091187f,0.9342772960662842f,0.4568612575531006f,-0.09419381618499756f,-0.6396105885505676f,0.31633296608924866f,-0.11616544425487518f,-0.03354884311556816f,0.050097040832042694f,-0.500287652015686f,-0.6916850209236145f,-0.05807457119226456f,-0.3754984140396118f,0.09964503347873688f,-0.2928231954574585f,0.04808689281344414f,1.5532138347625732f,0.21717767417430878f,-0.37714827060699463f,0.6870523691177368f,-0.7763425707817078f,0.7615166306495667f,-0.024734539911150932f,1.0238087177276611f,0.042485561221838f,0.6363743543624878f,0.6941838264465332f,-0.6285298466682434f,0.8428401947021484f,0.22396637499332428f,-0.33579495549201965f,-0.23342013359069824f,0.9256460070610046f,0.2151768058538437f,-0.1628854125738144f,0.3107295036315918f,-0.9440630078315735f,-0.15486696362495422f,-0.489212304353714f,-0.28539106249809265f,0.009993808344006538f,-0.022347068414092064f,-0.053123604506254196f,0.3292914032936096f,-0.08220700919628143f,-0.39622101187705994f,0.08744166791439056f,0.410778671503067f,-1.0876320600509644f,-0.26883241534233093f,0.21076516807079315f,0.38376903533935547f,-0.7741504907608032f,-0.5740035176277161f,-0.2770248353481293f,-0.8778526782989502f,-0.2738915681838989f,-0.46341562271118164f,0.21901392936706543f,0.33656826615333557f,0.22677023708820343f,0.5270676016807556f,0.22854194045066833f,0.05812099575996399f,0.5008333325386047f,0.7612602114677429f,0.8167506456375122f,0.43372008204460144f,0.053507935255765915f,0.09201672673225403f,0.7009100914001465f,-0.3608332574367523f,-0.06562500447034836f,0.6936282515525818f,0.026865674182772636f,-0.3814067542552948f,-1.0127770900726318f,0.21112219989299774f,-0.22792284190654755f,-0.8246350288391113f,0.1379466950893402f,0.7066747546195984f,-0.40544262528419495f,0.07334613054990768f,0.23677808046340942f,-0.18343865871429443f,-0.42603304982185364f,0.6940643191337585f,-0.6153205633163452f,-0.3265403211116791f,-0.3997204005718231f,0.11303278058767319f,-0.14709609746932983f,1.0718638896942139f,0.7671644687652588f,-0.5095505118370056f,-0.24545817077159882f,-0.807305097579956f,-0.538526177406311f,-0.4803426265716553f,0.010164258070290089f,0.4463258981704712f,0.15918388962745667f,-0.9620784521102905f,0.025140918791294098f,1.0223854780197144f,0.4024907946586609f,0.0972990021109581f,0.42691507935523987f,-0.6866060495376587f,0.06802517175674438f,-0.8780121803283691f,0.10776378214359283f,0.9990794062614441f,-0.03154274821281433f,0.03542518615722656f,-0.8226076364517212f,0.4021219313144684f,-0.5328401923179626f,0.33251646161079407f,-0.24879397451877594f,0.27551883459091187f,0.6190096735954285f,0.3874554932117462f,-0.4981039762496948f,-0.4469703137874603f,-0.07078012824058533f,-0.5735845565795898f,-0.007004039827734232f,0.6825802326202393f,1.578015923500061f,0.372245192527771f};
alignas(16) float batch_normalization_6_A[] = {0.3243119716644287f,0.7290039658546448f,0.5136919021606445f,1.053989291191101f,0.7742409706115723f,-0.023383542895317078f,-0.7344841957092285f,1.5411560535430908f,0.7964208722114563f,1.373447060585022f,-0.2426302433013916f,-0.04586007073521614f,-0.1734738051891327f,1.2054047584533691f,1.246718406677246f,-0.2558481991291046f,-0.11110090464353561f,1.8392884731292725f,-1.1577484607696533f,0.07501949369907379f,0.8422512412071228f,-0.32306644320487976f,-1.358952283859253f,-0.7546368837356567f};
alignas(16) float conv2d_3_internal_1_W[] = {0.07430511713027954f,0.19776643812656403f,-0.39733630418777466f,-0.08300165086984634f,-0.29222625494003296f,0.25443416833877563f,0.1777559518814087f,-0.1070953905582428f,-0.1808779090642929f,-0.05473696440458298f,0.18097464740276337f,-0.15959405899047852f,-0.11281288415193558f,-0.14781948924064636f,-0.15389777719974518f,-0.018556497991085052f,0.04740032181143761f,0.36500585079193115f,-0.024953899905085564f,0.037884898483753204f,-0.08407377451658249f,-0.29500389099121094f,0.01626460812985897f,0.2510354220867157f,0.053194187581539154f,-0.17917227745056152f,0.15050488710403442f,0.22681398689746857f,-0.6307203769683838f,-0.4601898193359375f,0.05433488264679909f,-0.008688918314874172f,0.3134367763996124f,-0.15136361122131348f,0.2173890769481659f,-0.27370116114616394f,-0.08938692510128021f,-0.01098680216819048f,-0.11024298518896103f,0.10752380639314651f,0.22371070086956024f,-0.2758345901966095f,0.027658779174089432f,0.0013801950262859464f,0.13506706058979034f,0.041218653321266174f,-0.1673145741224289f,-0.05096704512834549f,-0.25601622462272644f,-0.10411082208156586f,0.2491157352924347f,-0.12169491499662399f,-0.08495062589645386f,-0.026777222752571106f,-0.17858809232711792f,-0.08560297638177872f,0.323202908039093f,-0.017382636666297913f,0.07680768519639969f,-0.36658716201782227f,0.1279059648513794f,-0.4961693286895752f,0.16008292138576508f,0.11518307775259018f,0.21773332357406616f,-0.06411392986774445f,0.14159032702445984f,0.05653543770313263f,-0.24724863469600677f,0.21372276544570923f,-0.30784356594085693f,-0.04045969992876053f,0.022524872794747353f,-0.11943063139915466f,0.020650314167141914f,0.042698416858911514f,-0.4219328463077545f,-0.0279634241014719f,-0.023107219487428665f,-0.07519087940454483f,0.004950604867190123f,0.03399675711989403f,-0.07504037767648697f,0.059050701558589935f,-0.17876847088336945f,-0.3436611592769623f,-0.16201907396316528f,0.07873595505952835f,-0.07018280774354935f,0.2992074191570282f,-0.023338429629802704f,0.03638630732893944f,0.4618668854236603f,0.13658341765403748f,-0.3164463937282562f,-0.17486295104026794f,-0.01919800601899624f,-0.1844773143529892f,-0.37209802865982056f,-0.2350611388683319f,-0.05783437564969063f,0.10327168554067612f,-0.06260555982589722f,-0.1506347507238388f,-0.18444490432739258f,0.019702740013599396f,0.08763238787651062f,-0.1191459372639656f,-0.05311071127653122f,-0.3788834810256958f,-0.1395258903503418f,0.23845501244068146f,-0.10679837316274643f,0.3987438678741455f,-0.3780079483985901f,-0.09759580343961716f,0.12664197385311127f,-0.09671668708324432f,-0.31281208992004395f,0.042065732181072235f,-0.0032805989030748606f,-0.0023115917574614286f,0.005267612636089325f,0.13726504147052765f,0.09591862559318542f,0.09337291121482849f,0.04155661165714264f,0.033442672342061996f,-0.20843805372714996f,-0.05550748482346535f,-0.09189623594284058f,0.221607506275177f,0.14572854340076447f,-0.48207828402519226f,-0.08395887911319733f,-0.11864249408245087f,-0.06964424252510071f,-0.022863844409585f,-0.15372678637504578f,-0.044401854276657104f,-0.14104454219341278f,0.07964077591896057f,-0.34614497423171997f,-0.014501933939754963f,0.054997432976961136f,0.17672233283519745f,-0.44308531284332275f,-0.115089051425457f,0.2723383605480194f,0.5347112417221069f,-0.2622527778148651f,-0.07569100707769394f,-0.15945343673229218f,0.058338411152362823f,-0.01883118972182274f,-0.1355031132698059f,0.24738354980945587f,0.14750097692012787f,0.005637983325868845f,-0.19104348123073578f,0.04696275666356087f,0.2025928795337677f,0.24565528333187103f,-0.17367278039455414f,-0.15254420042037964f,0.46704673767089844f,0.2564902603626251f,-0.1258111447095871f,0.11133396625518799f,0.21344046294689178f,0.01906064711511135f,0.17242048680782318f,0.2721247673034668f,0.07876414060592651f,0.08821681886911392f,-0.2212415635585785f,0.09531469643115997f,-0.020984921604394913f,0.0810616984963417f,0.22762779891490936f,-0.024090755730867386f,0.3393997848033905f,-0.036277417093515396f,0.0474553219974041f,-0.12154268473386765f,-0.41458016633987427f,-0.19233745336532593f,0.1532999873161316f,0.18552091717720032f,-0.2432163655757904f,0.23599448800086975f,0.12601324915885925f,0.23425650596618652f,0.10221447050571442f,-0.6114968061447144f,-0.10695943981409073f,-0.04586299508810043f,-0.10979608446359634f,-0.03668825328350067f,-0.13044102489948273f,0.057341866195201874f,-0.1678844690322876f,0.028932776302099228f,-0.3601260483264923f,0.2133622020483017f,-0.07214311510324478f,0.21098913252353668f,0.01272963173687458f,0.01610124669969082f,0.042469967156648636f,0.1877518743276596f,0.1006549820303917f,0.32803940773010254f,0.18865250051021576f,-0.06613562256097794f,0.03090844303369522f,0.10271607339382172f,-0.2738538384437561f,0.02320481278002262f,0.22035720944404602f,0.6585140824317932f,-0.08139591664075851f,-0.17819590866565704f,-0.1351361721754074f,-0.03562720865011215f,0.26370611786842346f,-0.1870269924402237f,-0.4864758551120758f,0.08865566551685333f,-0.6405752897262573f,0.02115096151828766f,-0.10765524208545685f,0.04289547726511955f,-0.09674651175737381f,-0.12873955070972443f,0.3085246980190277f,-0.24781616032123566f,-0.2579757273197174f,0.018862897530198097f,-0.1327110230922699f,0.0329788401722908f,0.08521958440542221f,0.3002850115299225f,0.29129114747047424f,0.0371817946434021f,0.04050228372216225f,0.08714123070240021f,-0.2673981487751007f,-0.1597548872232437f,-0.12909835577011108f,0.21918922662734985f,0.04278864338994026f,0.2662183344364166f,-0.07442707568407059f,-0.026754090562462807f,0.35368409752845764f,-0.013878166675567627f,0.08514580130577087f,-0.3966265320777893f,0.34509581327438354f,0.21325401961803436f,0.302836149930954f,0.10409731417894363f,-0.03185715153813362f,0.4193938970565796f,-0.26226159930229187f,0.18778619170188904f,0.10611361265182495f,-0.0808010995388031f,-0.12909764051437378f,-0.09586015343666077f,-0.27037686109542847f,-0.0366920568048954f,0.07947447150945663f,0.318374365568161f,0.2912295460700989f,-0.04569590464234352f,-0.21169236302375793f,-0.03380715101957321f,0.12879545986652374f,-0.12817907333374023f,0.015879541635513306f,0.09263801574707031f,-0.24881896376609802f,-0.36691901087760925f,0.05439405515789986f,-0.3126959502696991f,-0.18374067544937134f,0.11380232125520706f,-0.32586362957954407f,-0.27546972036361694f,-0.44937944412231445f,-0.08638542145490646f,-0.3817485272884369f,-0.032349344342947006f,0.16914059221744537f,-0.0636029914021492f,0.2687455415725708f,0.07323895394802094f,0.20168417692184448f,-0.2800935208797455f,-0.05517417564988136f,0.04574527218937874f,-0.11553344875574112f,0.022960379719734192f,-0.2530783414840698f,0.027908941730856895f,-0.2129126638174057f,-0.00016658782260492444f,0.20943233370780945f,-0.014223205856978893f,-0.223643958568573f,0.0682702511548996f,0.48027321696281433f,-0.18597151339054108f,0.08406928181648254f,0.17088840901851654f,-0.037247415632009506f,-0.05548003315925598f,0.11804986000061035f,0.24626469612121582f,0.21254600584506989f,0.3766588866710663f,-0.1464514285326004f,0.18111304938793182f,0.1701221913099289f,-0.09112358093261719f,-0.04864087328314781f,0.03139033541083336f,0.10231166332960129f,-0.11578339338302612f,-0.18093706667423248f,-0.22107595205307007f,-0.2619531452655792f,-0.20080387592315674f,-0.20150528848171234f,-0.1536368429660797f,-0.10520540922880173f,-0.047089703381061554f,0.14691385626792908f,-0.4696325659751892f,0.1174318715929985f,0.2696126401424408f,0.28879523277282715f,-0.10521800071001053f,-0.05842779576778412f,-0.028779540210962296f,0.029984712600708008f,0.06223299726843834f,0.2827244997024536f,-0.22648252546787262f,0.16374090313911438f,-0.08802472800016403f,0.04434827342629433f,0.026492204517126083f,-0.2684538960456848f,-0.30024465918540955f,0.1726297289133072f,-0.023119984194636345f,0.23864831030368805f,-0.15048177540302277f,-0.13006505370140076f,0.11935305595397949f,0.30518898367881775f,0.13305914402008057f,-0.46526652574539185f,-0.23956048488616943f,-0.06375177949666977f,-0.030982591211795807f,0.17321844398975372f,-0.11887707561254501f,-0.07510748505592346f,-0.2745437026023865f,-0.21903476119041443f,0.16600985825061798f,-0.11111698299646378f,0.21245715022087097f,-0.27716198563575745f,-0.12878486514091492f,-0.14485134184360504f,-0.019466452300548553f,0.0936245247721672f,-0.16887959837913513f,0.37172871828079224f};
alignas(16) float batch_normalization_7_A[] = {0.22971756756305695f,-0.16255289316177368f,0.42725974321365356f,0.28215113282203674f,0.2078244388103485f,0.11388906836509705f,0.41315537691116333f,0.21372677385807037f,0.27697277069091797f,0.3703034222126007f,0.03761696070432663f,-0.11220814287662506f,-0.23752674460411072f,0.7152029871940613f,0.8767150640487671f,0.36709731817245483f};
alignas(16) float separable_conv2d_4_internal_0_VALUES[] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
alignas(16) float separable_conv2d_4_internal_1_W[] = {-0.15952973067760468f,0.03779768943786621f,-0.14462780952453613f,0.05114029347896576f,0.01161690428853035f,-0.06124561280012131f,0.12948812544345856f,0.1241699606180191f,0.16243566572666168f,-0.0928490161895752f,-0.03957713767886162f,-0.07264097034931183f,-0.038613732904195786f,0.07693648338317871f,0.08470945060253143f,0.0044779181480407715f,0.008060584776103497f,0.15541768074035645f,0.04234086722135544f,0.04550861939787865f,0.0186964999884367f,-0.1549849510192871f,-0.06467720866203308f,0.04125013202428818f,-0.02282162755727768f,-0.1194620430469513f,0.03566284850239754f,-0.029316268861293793f,0.04149433225393295f,-0.044685158878564835f,-0.031953033059835434f,0.12022766470909119f,0.015531599521636963f,0.15284988284111023f,0.09358559548854828f,0.003933338914066553f,0.03182811290025711f,-0.14840352535247803f,-0.03610086813569069f,-0.02183908224105835f,-0.04346057400107384f,-0.03303205221891403f,0.10273388028144836f,0.08206091821193695f,0.040577854961156845f,0.10632806271314621f,-0.05162516236305237f,0.11974061280488968f,0.06587748974561691f,-0.03361876308917999f,-0.12983432412147522f,-0.037332095205783844f,-0.11692024767398834f,0.1204969733953476f,0.0040629091672599316f,0.08958178758621216f,-0.010791681706905365f,0.11776971817016602f,0.09690562635660172f,-0.00765409879386425f,0.011965778656303883f,0.13886652886867523f,-0.038330480456352234f,-0.07992307841777802f,-0.021598298102617264f,0.06973424553871155f,-0.0041708736680448055f,0.032145582139492035f,-0.007043818011879921f,0.1205265074968338f,0.008433496579527855f,-0.024584956467151642f,-0.060691554099321365f,-0.036180704832077026f,-0.02567129395902157f,-0.1182246282696724f,-0.10715688019990921f,0.004751912783831358f,0.04552587866783142f,0.0065946998074650764f,-0.1499456912279129f,-0.13011428713798523f,0.10305453091859818f,0.03110099956393242f,0.08478894084692001f,0.029376477003097534f,0.06366722285747528f,0.010176603682339191f,-0.11013425886631012f,0.06997675448656082f,-0.11762920022010803f,-0.00042402115650475025f,-0.06256048381328583f,-0.09193525463342667f,-0.08809150010347366f,0.07896552979946136f,0.06307841837406158f,0.05853186175227165f,0.019296158105134964f,0.08411840349435806f,-0.1139044240117073f,0.020984644070267677f,0.05962398648262024f,0.06594672799110413f,0.10902764648199081f,-0.1561228334903717f,0.029654715210199356f,-0.0764131247997284f,-0.08297637850046158f,0.032011840492486954f,0.01310181524604559f,-0.021077236160635948f,0.08088681101799011f,-0.047470059245824814f,-0.018934551626443863f,0.03992599621415138f,-0.05130676180124283f,-0.047438718378543854f,0.02327686734497547f,-0.038943447172641754f,-0.09243754297494888f,-0.062324803322553635f,0.10668396949768066f,0.0172869972884655f,0.01835082285106182f,0.09691793471574783f,0.019489942118525505f,0.052048515528440475f,0.04711945354938507f,0.0861138179898262f,0.022886380553245544f,-0.030040323734283447f,-0.0989954024553299f,-0.017326032742857933f,0.08096875995397568f,-0.006995157804340124f,-0.054169971495866776f,0.05284566804766655f,0.08006570488214493f,-0.04841257631778717f,-0.0036134845577180386f,0.06799791753292084f,-0.04051409289240837f,-0.1679983139038086f,-0.0013113805325701833f,0.004553697537630796f,-0.06707631051540375f,0.09235593676567078f,-0.05061451345682144f,0.08752386271953583f,0.004844789393246174f,0.09253982454538345f,-0.08743825554847717f,0.09123819321393967f,0.10393764078617096f,-0.010881882160902023f,0.12582182884216309f,-0.020865853875875473f,-0.027962852269411087f,0.1120140552520752f,0.12541456520557404f,0.003802270395681262f,-0.03304046392440796f,-0.11481507122516632f,0.05961214751005173f,-0.03450119122862816f,0.12291848659515381f,-0.02460094913840294f,0.021851932629942894f,-0.051856301724910736f,0.02712470293045044f,-0.01984722726047039f,-0.028951941058039665f,0.01655655726790428f,0.06532096862792969f,0.07063209265470505f,-0.026190096512436867f,-0.09274259209632874f,0.010585427284240723f,-0.04089520499110222f,-0.04492275044322014f,-0.13992656767368317f,0.06981079280376434f,0.05782917141914368f,0.0021719883661717176f,-0.07202543318271637f,0.0701860785484314f,0.11243540793657303f,-0.043668631464242935f,0.03394925966858864f,0.016721520572900772f,-0.07241673022508621f,0.0035282126627862453f,0.0024046902544796467f,-0.056942619383335114f,-0.14106056094169617f,0.08024177700281143f,-0.04903140291571617f,-0.002285016467794776f,0.05811385437846184f,0.06352472305297852f,0.13365523517131805f,0.024687480181455612f,-0.02910729870200157f,0.00806969590485096f,0.054299477487802505f,-0.06746454536914825f,0.12215591967105865f,0.013240694999694824f,-0.0011968656908720732f,-0.05207878351211548f,-0.053091950714588165f,0.05333016440272331f,-0.003572422079741955f,0.03983483836054802f,-0.04079800099134445f,-0.03694163262844086f,-0.038259416818618774f,0.12480731308460236f,-0.07361187785863876f,-0.011741641908884048f,-0.09537404775619507f,0.1216241866350174f,0.07141901552677155f,-0.020633641630411148f,-0.05768680199980736f,0.1558554768562317f,-0.005266731139272451f,-0.012954236008226871f,-0.03888682276010513f,0.031243566423654556f,-0.12140344083309174f,-0.08093147724866867f,-0.059415433555841446f,-0.04637714475393295f,0.12748128175735474f,0.04354401305317879f,0.023464307188987732f,0.004255332052707672f,-0.051239073276519775f,-0.06358873844146729f,-0.07043733447790146f,0.01325408648699522f,-0.062220193445682526f,-0.011267408728599548f,0.017973748967051506f,-0.10077481716871262f,0.030737485736608505f,-0.0054401070810854435f,-0.07232913374900818f,0.006790188141167164f,-0.12901069223880768f,-0.0710141584277153f,0.004270079545676708f,-0.04740016162395477f,-0.09701868891716003f,-0.0007731571095064282f,-0.08910411596298218f,-0.030251221731305122f,-0.05735649913549423f,-0.09725990146398544f,0.06054810434579849f,0.046609196811914444f,-0.001406416529789567f,-0.0020731231197714806f,0.004156153183430433f,-0.050151631236076355f,-0.04172036424279213f,0.021539853885769844f,-0.013483205810189247f,-0.062450915575027466f,-0.007203984074294567f,-0.04520927369594574f,-0.11864282190799713f,-0.029091818258166313f,-0.10874325037002563f,-0.08331585675477982f,0.09518138319253922f,-0.02480035275220871f,-0.0778273493051529f,0.039658378809690475f,-0.07847759872674942f,-0.10887883603572845f,-0.04532255604863167f,-0.1296393722295761f,0.06123991310596466f,0.00615698890760541f,0.04115055128931999f,0.03228186070919037f,-0.05253603309392929f,-0.011315521784126759f,0.06350301951169968f,0.06502220034599304f,0.12364697456359863f,-0.018233640119433403f,-0.02929735742509365f,-0.11073404550552368f,0.0036601684987545013f,0.12325183302164078f,0.05095222592353821f,-0.05760450288653374f,0.03720507025718689f,0.009942682459950447f,-0.07442136853933334f,0.1272086352109909f,0.03040388785302639f,-0.0313694141805172f,0.027452869340777397f,-0.06235051155090332f,-0.02285856008529663f,-0.026684662327170372f,-0.12314700335264206f,0.04848439618945122f,-0.11013367027044296f,-0.05082231014966965f,-0.0010009041288867593f,0.14449281990528107f,-0.02970016561448574f,0.06419961899518967f,-0.06262730807065964f,0.022036481648683548f,0.041627634316682816f,-0.022885622456669807f,0.011638752184808254f,-0.03763660043478012f,-0.10663337260484695f,0.010755961760878563f,-0.045515164732933044f,0.0009111797553487122f,0.06344151496887207f,-0.03584396839141846f,-0.03277542069554329f,-0.036919064819812775f,0.047783028334379196f,-0.062274109572172165f,-0.0005050906911492348f,0.004292049445211887f,0.09050136059522629f,-0.09946700185537338f,0.06704702228307724f,0.004217721056193113f,-0.029776940122246742f,0.11321043968200684f,0.07395677268505096f,-0.040584906935691833f,-0.08046367764472961f,0.06400879472494125f,-0.06741844117641449f,0.02446180395781994f,0.12438113242387772f,0.03825205937027931f,0.031097495928406715f,0.015858119353652f,-0.04801824688911438f,-0.012480738572776318f,-0.007678051944822073f,0.0809723511338234f,-0.03240250423550606f,0.06763438135385513f,0.022945065051317215f,-0.04491531476378441f,0.09872067719697952f,0.05680219084024429f,-0.14971907436847687f,0.037449102848768234f,-0.14591731131076813f,0.0785028487443924f,0.08582589030265808f,-0.12321116775274277f,0.02391885779798031f,-0.05096864700317383f,-0.07697048783302307f,0.017864834517240524f,0.04826102405786514f,-0.057219065725803375f,-0.008076541125774384f,0.03968670219182968f,0.09319474548101425f,0.03804052621126175f,0.08165352791547775f,0.0516989640891552f,-0.09264258295297623f,-0.12876303493976593f,-0.07853735983371735f,0.11148180067539215f,0.05866091698408127f,-0.1162128746509552f,0.0473189614713192f,-0.04391276836395264f,0.033940766006708145f,-0.06582184135913849f,0.08268069475889206f,0.015211127698421478f,0.022882649675011635f,0.09817740321159363f,-0.010088280774652958f,-0.03651060163974762f,-0.06532600522041321f,-0.06328690052032471f,-0.0013752214144915342f,0.17787832021713257f,-0.09033776819705963f,0.041436586529016495f,0.09655717760324478f,0.06777632981538773f,0.0720781609416008f,-0.09219206869602203f,0.0840706005692482f,-0.06046885997056961f,-0.07469189912080765f,0.0202778447419405f,0.13828015327453613f,0.06521861255168915f,-0.004191170912235975f,-0.019315112382173538f,0.0017110953340306878f,0.09401150047779083f,-0.022862903773784637f,-0.06812865287065506f,0.037384990602731705f,0.057684727013111115f,0.07038448005914688f,-0.08075671643018723f,0.01950773410499096f,-0.05899585783481598f,-0.08672846853733063f,-0.11729684472084045f,-0.07604049891233444f,0.08846377581357956f,0.02866736799478531f,0.07846518605947495f,-0.055977653712034225f,-0.10058825463056564f,0.037535160779953f,-0.1056433767080307f,0.056365884840488434f,0.04218406602740288f,-0.027297046035528183f,-0.014179307967424393f,0.04753457382321358f,-0.01666676066815853f,0.017026856541633606f,0.023420922458171844f,0.0008230344974435866f,-0.09835705161094666f,0.018973488360643387f,-0.09710419178009033f,-0.008477844297885895f,0.023833326995372772f,-0.09742648899555206f,-0.08848520368337631f,0.10446696728467941f,-0.06279777735471725f,-0.016492336988449097f,-0.026817696169018745f,0.02299131453037262f,-0.0018075682455673814f,-0.08977705240249634f,0.10296317934989929f,-0.007599323056638241f,0.09233805537223816f,0.0007744812755845487f,-0.017695730552077293f,-0.008262775838375092f,0.03481084853410721f,0.0557500459253788f,0.06798390299081802f,-0.10776230692863464f,-0.0818624198436737f,-0.01886449009180069f,0.014260259456932545f,0.010932372882962227f,0.0994245782494545f,0.025506259873509407f,-0.0031926643569022417f,-0.0012068523792549968f,0.0021723804529756308f,0.11717663705348969f,-0.030823690816760063f,0.009480353444814682f,-0.011744886636734009f,-0.002056154888123274f,-0.042048390954732895f,0.09005622565746307f,-0.0032589323818683624f,-0.08449888974428177f,-0.07757814973592758f,0.035356733947992325f,-0.09641727060079575f,-0.020338892936706543f,-0.15070585906505585f,-0.0595320500433445f,-0.019863536581397057f,0.058520715683698654f,0.019058380275964737f,0.06669282913208008f,0.02158513106405735f,-0.07929977774620056f,0.07086010277271271f,0.03226364031434059f,0.02795993536710739f,-0.04011645168066025f,-0.06541266292333603f,-0.04427572339773178f,0.09528745710849762f,-0.03686664253473282f,-0.031310368329286575f,0.007088950835168362f,0.036224570125341415f,0.04459502920508385f,-0.025987321510910988f,0.08225592225790024f,0.022284744307398796f,0.06476093083620071f,0.05632155016064644f,0.045915327966213226f,-0.020336436107754707f,-0.05466480925679207f,-0.06797169893980026f,0.031291376799345016f,0.02853723242878914f,-0.009106521494686604f,0.07611725479364395f,0.01110067404806614f,-0.07530135661363602f,-0.13847781717777252f,0.024268869310617447f,0.06271719932556152f,0.01294784713536501f,0.03402084484696388f,0.08832824230194092f,-0.007255067583173513f,-0.05649062618613243f,-0.04784725606441498f,-0.02557550184428692f,0.03213771805167198f,0.0035005295649170876f,0.007445395924150944f,0.031049270182847977f,0.144907608628273f,0.0590038001537323f,-0.05277705192565918f,0.07641777396202087f,-0.03235624358057976f,-0.02262023091316223f,-0.053971003741025925f,0.05133025348186493f,-0.005327229388058186f,0.044335685670375824f,-0.007930750958621502f,-0.012503868900239468f,0.051561277359724045f,-0.02511575073003769f,-0.026571180671453476f,-0.003405120223760605f,-0.036604538559913635f,0.0034740373957902193f,0.02432764694094658f,0.09697864949703217f,-0.01141562033444643f,0.006520181894302368f,0.005780531093478203f,-0.027215292677283287f,0.07594906538724899f,-0.0135662741959095f,0.057431913912296295f,0.09265735745429993f,-0.05938812345266342f,0.12480635195970535f,0.06443548947572708f,0.03685591742396355f,-0.03177080303430557f,-0.01947121135890484f,0.02649330347776413f,-0.05073625594377518f,0.05579864978790283f};
alignas(16) float separable_conv2d_4_internal_2_W[] = {-1.630974531173706f,1.5265934467315674f,1.411393404006958f,-0.13917340338230133f,-0.5762366056442261f,-2.6757619380950928f,-0.03803650662302971f,-0.8843799829483032f,-0.0976717546582222f,-0.32291755080223083f,0.25140878558158875f,-0.2660784125328064f,0.8174243569374084f,1.793729543685913f,-0.028903264552354813f,-0.9797027111053467f,-1.6527245044708252f,0.6993075609207153f,-0.33525949716567993f,-1.1551088094711304f,-0.1967521607875824f,-0.8007462620735168f,0.6571430563926697f,-0.16740834712982178f,0.3369975984096527f,1.2892564535140991f,1.2420538663864136f,-1.1064810752868652f,0.5056620240211487f,-2.207231044769287f,0.6329571008682251f,-0.8204216957092285f,-1.8768259286880493f,-2.84580397605896f,-0.7645960450172424f,0.7592712640762329f,1.337512493133545f,0.8021437525749207f,-0.31908878684043884f,-0.2822534739971161f,0.8448747992515564f,0.4937232434749603f,-1.5556620359420776f,-0.04235285148024559f,0.985011100769043f,0.6519654393196106f,0.13975706696510315f,-1.9583522081375122f,0.21871601045131683f,-0.21186229586601257f,1.1807842254638672f,0.7672948241233826f,1.117000937461853f,-1.9648622274398804f,1.1231411695480347f,0.8964925408363342f,0.2291254848241806f,-0.590606153011322f,0.2583339214324951f,1.1114519834518433f,0.4351293444633484f,-1.1634654998779297f,0.47578704357147217f,-0.8569433689117432f,1.6951969861984253f,0.5872541666030884f,2.8665037155151367f,1.7020914554595947f,-0.7935989499092102f,-0.0887509286403656f,-1.0883235931396484f,-0.5096178650856018f,-0.04789934307336807f,0.8881382346153259f,-0.9232471585273743f,1.8910083770751953f,-1.6854768991470337f,1.0548903942108154f,-0.8016524314880371f,0.5924926996231079f,0.6372219324111938f,1.3988054990768433f,0.39591503143310547f,-0.05533856153488159f,1.4763067960739136f,1.5419721603393555f,-1.2254642248153687f,2.4359030723571777f,-0.32948827743530273f,0.7790043354034424f,3.156257152557373f,-0.01422610692679882f,2.252030849456787f,-1.0340189933776855f,-1.4157006740570068f,1.2585468292236328f,-0.6541531682014465f,-0.29180675745010376f,0.4631822407245636f,0.2843473255634308f,1.5323346853256226f,0.22299928963184357f,1.8658424615859985f,2.0741076469421387f,-0.40826573967933655f,-1.2464994192123413f,0.06635230034589767f,1.2684335708618164f,1.165454387664795f,-0.8930866122245789f,-0.4947482645511627f,0.0881597176194191f,-1.1520930528640747f,1.5557448863983154f,-0.4683314859867096f,-1.307075023651123f,0.381961464881897f,2.1225335597991943f,-0.9180226922035217f,0.8370177149772644f,2.0853567123413086f,-0.5258809924125671f,-0.265411376953125f,-0.19254198670387268f,-0.1294759064912796f,-0.13857105374336243f,-0.06618772447109222f,-0.21968865394592285f,0.19415417313575745f,-0.599321722984314f,0.6520662903785706f,0.18591533601284027f,0.9201542139053345f,-0.5749552845954895f,1.89424467086792f,-0.2242969572544098f,-1.2840962409973145f,0.24253855645656586f,2.7992472648620605f,0.03637891635298729f,1.1146167516708374f,0.8128954768180847f,-0.7231137156486511f,0.37583863735198975f,0.6345574259757996f,1.793055772781372f,1.80392324924469f,-1.246085524559021f,-1.1726897954940796f,0.6548007130622864f,-1.4184906482696533f,1.650691032409668f,-0.8101062774658203f,-0.535687267780304f,0.026457859203219414f,-1.2953753471374512f,0.9680708646774292f,-1.0617536306381226f,0.9259440302848816f,0.44738849997520447f,-0.15428312122821808f,0.31310248374938965f,0.35957154631614685f,0.9212666749954224f,1.4628210067749023f,0.7270539999008179f,0.8103886842727661f,0.5166355967521667f,-0.026023970916867256f,-0.4836564064025879f,-0.5687826871871948f,1.1775202751159668f,-0.788997232913971f,-0.11324062198400497f,-0.18378716707229614f,0.7806243896484375f,0.661755383014679f,-0.43776705861091614f,0.7418246269226074f,0.9755613803863525f,-1.6474655866622925f,2.7648844718933105f,1.1695313453674316f,-1.2714080810546875f,0.08834238350391388f,1.1586066484451294f,-0.3139983117580414f,1.7775954008102417f,0.6252366900444031f,-1.307092547416687f,0.5286788940429688f,1.0219080448150635f,1.1604152917861938f,-0.8600754141807556f,0.6900959014892578f,1.7442079782485962f,-0.5556864738464355f,0.6015906929969788f,0.6322504281997681f,-0.06937618553638458f,-2.3745129108428955f,-0.08229190111160278f,-0.9215704202651978f,0.07036277651786804f,-2.6973824501037598f,-0.9460974335670471f,0.6124626994132996f,-0.20540906488895416f,-0.04762765392661095f,-0.40518873929977417f,-1.3202650547027588f,-1.2446175813674927f,0.15922552347183228f,1.6519917249679565f,-0.24017730355262756f,0.5418264865875244f,-0.0987640991806984f,-0.022493628785014153f,0.3106197118759155f,0.7053862810134888f,0.03726824000477791f,2.0775678157806396f,0.2898026406764984f,0.22594529390335083f,0.4151972830295563f,-0.6112782955169678f,-1.1038371324539185f,0.4602998197078705f,-0.9258342981338501f,2.0510106086730957f,-0.025341706350445747f,0.0762866884469986f,-0.920504093170166f,-1.7724835872650146f,0.8684713840484619f,1.0793620347976685f,-1.2875490188598633f,1.364551305770874f,-1.0720691680908203f,-0.5442225933074951f,0.13270889222621918f,0.3446856141090393f,-0.3378836214542389f,1.1715240478515625f,-0.053401026874780655f,-1.4397121667861938f,0.5533657670021057f,0.1239546611905098f,-1.3226537704467773f,-0.6544773578643799f,-0.3417130410671234f,-0.4676571190357208f,-0.8971186280250549f,-0.1823592483997345f,0.41544464230537415f,-2.636510133743286f,0.03044300340116024f,0.4390103816986084f,0.9714603424072266f,0.8590320944786072f,-0.9077242016792297f,-1.146626591682434f,-0.5611408352851868f,0.8712000250816345f,-0.20411404967308044f,-1.7115507125854492f,-0.5261321663856506f,0.2650271952152252f,-0.16249743103981018f,-0.9521705508232117f,0.48301780223846436f,1.0915195941925049f,0.14286938309669495f,0.02991553209722042f,-0.16785402595996857f,-0.6756609082221985f,2.960737705230713f,-0.6330258250236511f,-0.4107252359390259f,-0.16884736716747284f,0.20566536486148834f,0.3989454209804535f,-0.042364347726106644f,1.859827995300293f,1.0365837812423706f,0.266679584980011f,-1.8961747884750366f,0.3393002450466156f,-0.38283947110176086f,0.05633018910884857f,0.07685969769954681f,-0.9024363160133362f,0.8664063811302185f,0.7310633063316345f,-1.151163935661316f,2.1072866916656494f,0.3180121183395386f,1.8808050155639648f,-0.6274366974830627f,-1.4845749139785767f,-0.0462072491645813f,-1.8057271242141724f,1.1728382110595703f,-1.302070140838623f,0.11938989907503128f,1.6214377880096436f,-0.3943901062011719f,0.013648380525410175f,-0.46643730998039246f,-1.8875759840011597f,-0.08002182096242905f,-0.1840522736310959f,0.7689042091369629f,-1.075534462928772f,-0.39480361342430115f,-1.2116022109985352f,1.4087364673614502f,-1.8276594877243042f,0.5449329614639282f,-0.6680905222892761f,0.3150719106197357f,3.1249823570251465f,-1.6397706270217896f,0.31691983342170715f,-0.23067037761211395f,-0.09130843728780746f,0.7960419058799744f,-1.2846488952636719f,-1.4893383979797363f,0.8882851004600525f,1.220646858215332f,-0.15150609612464905f,0.7496384382247925f,-2.0820770263671875f,0.9417929649353027f,0.6674900054931641f,0.11615511029958725f,0.607419490814209f,0.07631603628396988f,-0.3073844909667969f,-0.9677509665489197f,0.5109058022499084f,-0.18831531703472137f,0.03796757385134697f,0.32689064741134644f,0.1808420866727829f,0.05764147266745567f,-0.4828021824359894f,-0.8333434462547302f,-1.7393524646759033f,-0.9428040385246277f,-1.062311053276062f,-0.814705491065979f,1.2066138982772827f,0.31279900670051575f,-2.1238701343536377f,-0.09224159270524979f,0.21477928757667542f,-0.4349478483200073f,0.9031741619110107f,-1.0832970142364502f,-1.2859246730804443f,0.543918788433075f,1.7922102212905884f,-1.260202407836914f,1.4250410795211792f,0.06658230721950531f,-1.1128593683242798f,-0.6180729269981384f,0.4162738025188446f,0.2810995280742645f,-1.26778244972229f,0.9097927808761597f,0.01072755828499794f,-0.06890279054641724f,0.4594550132751465f,-1.2369418144226074f,-1.6440619230270386f,-0.674513041973114f,-0.4250435531139374f,-1.4316753149032593f,0.6869925856590271f,1.7367255687713623f,-1.4301888942718506f,0.4146992266178131f,-0.5391411781311035f,-0.6028324365615845f,0.1142086386680603f,-0.7078689336776733f,0.5750251412391663f,-0.27158302068710327f,0.4557369351387024f,-1.3972961902618408f,1.9271221160888672f,0.14191067218780518f,-1.2474435567855835f,0.4390926957130432f,-0.5672457814216614f,-0.13182030618190765f,1.4377449750900269f,-1.6133910417556763f,2.8634841442108154f,0.8174819350242615f,1.2303016185760498f,-1.1414794921875f,-0.8139389753341675f,0.9197658896446228f,0.6796665787696838f,-0.29555970430374146f,0.8537198305130005f,-0.4440139830112457f,0.5788964033126831f,-0.974668562412262f,0.9352195262908936f,0.5047963261604309f,-0.8926388621330261f,1.4806416034698486f,-0.470826119184494f,-0.34214845299720764f,0.8691549897193909f,0.10179542005062103f,-0.566453218460083f,0.6961645483970642f,0.36014434695243835f,2.6776180267333984f,-0.9767394065856934f,-0.29863521456718445f,-0.5325940847396851f,1.0083181858062744f,-1.1964776515960693f,-2.3408758640289307f,-1.4145451784133911f,1.1651184558868408f,-0.33268046379089355f,-0.2614734172821045f,0.9000992774963379f,0.2751074433326721f,-1.498690128326416f,-0.24183955788612366f,1.1434974670410156f,-0.21780157089233398f,1.0547229051589966f,0.9951139092445374f,-0.6532005071640015f,0.43660202622413635f,-1.2348257303237915f,0.43002378940582275f,0.957038164138794f,0.6615258455276489f,-0.10033563524484634f,-0.4763373136520386f,0.5333157777786255f,1.0734272003173828f,0.7060922980308533f,0.7341223955154419f,-0.8449286818504333f,-0.6636316180229187f,-0.6284040212631226f,0.4912334084510803f,-1.9405184984207153f,0.3830716013908386f,0.3299688696861267f,-1.0347017049789429f,1.36125648021698f,-0.35345199704170227f,-1.3283030986785889f,-1.7717509269714355f,-0.8450788259506226f,-2.270522356033325f,-0.9036338329315186f,-0.23444028198719025f,-1.459797739982605f,-0.2809450030326843f,-0.17188309133052826f,0.512791097164154f,-0.11582313477993011f,1.346301794052124f,-0.33796921372413635f,-0.6841039061546326f,-0.6974780559539795f,0.1979144811630249f,-1.1780521869659424f,1.1298210620880127f,0.6457290649414062f,-0.7137949466705322f,0.8808467388153076f,-0.6453636288642883f,-3.2827234268188477f,0.4019497036933899f,0.024682193994522095f,0.8810972571372986f,1.349668025970459f,-0.34636542201042175f,0.08018418401479721f,-0.2902595102787018f,-2.01010799407959f,0.8783561587333679f,3.417480230331421f,-0.3638048470020294f,-1.1882314682006836f,-0.14534065127372742f,-0.8023611903190613f,0.8001462817192078f,-0.1542542576789856f,-0.2685483992099762f,-0.7278752326965332f,-0.5119239687919617f,0.14295367896556854f,1.2036998271942139f,1.7034897804260254f,-0.38943973183631897f,-0.4026773273944855f,0.9472910165786743f,0.5018123388290405f,-2.7729134559631348f,0.9370930790901184f,0.18853677809238434f,-0.13793349266052246f,-0.28873303532600403f,1.076635479927063f,-0.5196840763092041f,3.7569355964660645f,3.0713987350463867f,0.5915839076042175f,0.5167722105979919f,-0.6304009556770325f,-1.3059204816818237f,0.8570913672447205f,-0.15117210149765015f,-1.1217164993286133f,-0.1424795389175415f,-1.8158046007156372f,-1.2514715194702148f,1.6238754987716675f,1.450339436531067f,1.9754225015640259f,-0.4113476872444153f,2.2752768993377686f,0.14015686511993408f,1.3799203634262085f,1.3152021169662476f,-0.3760358393192291f,-1.0961347818374634f,-0.5795503854751587f,-0.48380839824676514f,-1.5529330968856812f,1.1819154024124146f,-0.30041182041168213f,-0.1350906491279602f,0.9420107007026672f,0.5651288628578186f,-0.5902886390686035f,-0.7796686291694641f,-0.34495413303375244f,-1.1010327339172363f,-0.3136094808578491f,1.0825666189193726f,-0.2599036693572998f,-0.7109820246696472f,0.06580241024494171f,1.4952232837677002f,0.1091848686337471f,3.5909650325775146f,0.5508501529693604f,-1.123164176940918f,0.6015400886535645f,-0.20637862384319305f,1.7472362518310547f,-0.3952980935573578f,0.8632411956787109f,0.39081043004989624f,0.46130046248435974f,1.920116901397705f,-0.05640176311135292f,-0.046627167612314224f,-0.2439238578081131f,-0.3644445836544037f,0.48116588592529297f,-0.364400714635849f,0.3510593771934509f,-0.13665957748889923f,-0.49214550852775574f,-0.7305433750152588f,-1.4035619497299194f,-0.05114186927676201f,2.932316780090332f,0.23292627930641174f,1.6195117235183716f,-0.8636681437492371f,-1.919717788696289f,0.3127884566783905f,1.0005600452423096f,-0.2878057062625885f,0.6147800087928772f,0.08468315750360489f,0.27546438574790955f,0.07753288000822067f,-0.5122869610786438f,-1.0616577863693237f,-1.3017797470092773f,0.6952798366546631f,1.1546880006790161f,0.28836941719055176f,-0.42414700984954834f,0.0022233284544199705f,-0.19292952120304108f,-2.225846529006958f,-0.3094031512737274f,-0.9434629082679749f,-0.7906802296638489f,-1.4460139274597168f,-0.9670694470405579f,0.8040679097175598f,-0.4441642165184021f,0.45511874556541443f,1.190647840499878f,-0.7802042961120605f,-0.3760530650615692f,-0.7269925475120544f,-1.6127375364303589f,-1.5564879179000854f,-1.5383328199386597f,1.2892111539840698f,0.0419904887676239f,-0.17264704406261444f,0.3436189293861389f,0.19417575001716614f,1.197943091392517f,-0.4751988649368286f,-0.6667280197143555f,-0.5277745127677917f,1.0556647777557373f,-1.4416728019714355f,-0.2139887660741806f,-0.8097192645072937f,-1.6300212144851685f,1.6983647346496582f,1.294913411140442f,-0.030431807041168213f,0.8675681352615356f,-1.2606209516525269f,-0.29585912823677063f,-0.32668861746788025f,0.9357742071151733f,-1.1099599599838257f,0.20798197388648987f,-0.10331495106220245f,0.8822363615036011f,-1.2299119234085083f,-0.9223390817642212f,-1.1724456548690796f,-0.3839166760444641f,0.8207448124885559f,-0.8258603811264038f,-0.30979958176612854f,2.171544313430786f,1.0456300973892212f,0.0565168634057045f,1.0603748559951782f,1.3983827829360962f,-1.4507266283035278f,0.04375099390745163f,0.7613601684570312f,-0.644924521446228f,-0.6328496336936951f,0.4249955713748932f,-0.09718911349773407f,0.10502935200929642f,0.2639261782169342f,-1.2200878858566284f,-0.07576433569192886f,-0.8973284363746643f,-0.3200445771217346f,0.057270508259534836f,-0.5357240438461304f,-0.30308377742767334f,-0.500306248664856f,1.9631805419921875f,-0.6648301482200623f,-0.2761240005493164f,1.0339690446853638f,-0.3512411117553711f,-0.6549983024597168f,0.05645707622170448f,-1.397685170173645f,0.696300208568573f,-0.34571754932403564f,-1.4608044624328613f,-0.2518278956413269f,-0.9481328725814819f,0.2376699596643448f,-0.680420458316803f,-1.2000857591629028f,2.5150580406188965f,-0.5556454658508301f,-1.439229130744934f,1.0685173273086548f,-0.3329240679740906f,0.9589519500732422f,-1.4135500192642212f,-1.6741044521331787f,2.697861671447754f,-0.35643962025642395f,0.33106568455696106f,0.2578805685043335f,0.19140374660491943f,0.05751960724592209f,-0.7734544277191162f,-0.410337895154953f,0.6785826683044434f,-0.34319043159484863f,-1.2521756887435913f,0.03524886816740036f,-0.7881155610084534f,0.21612833440303802f,-0.46581822633743286f,-0.0059743341989815235f,-0.7219445705413818f,-1.5026435852050781f,-1.2124632596969604f,-0.26753726601600647f,0.6462425589561462f,1.3328198194503784f,-0.06198139488697052f,0.179564967751503f,0.10909130424261093f,1.146553635597229f,-1.4902582168579102f,-0.34457916021347046f,0.7687023878097534f,-1.0544995069503784f,-1.1522765159606934f,-2.1097018718719482f,0.6993871331214905f,2.1665797233581543f,0.5538344979286194f,-2.2293648719787598f,0.06167524307966232f,0.7011634707450867f,-1.5657020807266235f,-0.13213837146759033f,1.0932785272598267f,-0.45462697744369507f,-1.021440863609314f,0.29501354694366455f,-2.6676740646362305f,0.5355422496795654f,-0.608570396900177f,-0.04635868966579437f,0.6557791233062744f,-0.5157209038734436f,0.2071085125207901f,-0.7960059642791748f,-0.343923419713974f,0.5938382148742676f,-0.3163410723209381f,1.2415223121643066f,-0.8815426230430603f,-0.7983628511428833f,-1.1112103462219238f,-0.6093415021896362f,-1.4078595638275146f,0.732797384262085f,2.025481700897217f,0.5835331678390503f,-0.4359804689884186f,-0.5806230902671814f,0.14794588088989258f,-1.6105402708053589f,1.5018904209136963f,0.7914559245109558f,0.1576336920261383f,-0.5528729557991028f,0.35878074169158936f,0.6717920899391174f,0.9651156663894653f,1.3168073892593384f,0.9687100052833557f,-1.3707815408706665f,-0.741127610206604f,-2.094308853149414f,-0.07482555508613586f,-2.1243643760681152f,0.14084503054618835f,-0.4524240493774414f,-0.5599290728569031f,-1.27223539352417f,0.03255167976021767f,-1.1407045125961304f,0.4562625288963318f,1.7926791906356812f,0.6626617908477783f,1.3175119161605835f,-0.13196583092212677f,-0.743721067905426f,0.12283008545637131f,2.6888623237609863f,-1.3704440593719482f,1.096549153327942f,0.05698995292186737f,1.9376296997070312f,-1.403610348701477f,2.91511607170105f,0.7215492725372314f,0.32173770666122437f,0.3269864022731781f,-1.6407400369644165f,1.8806337118148804f,-0.7905166745185852f,1.9410607814788818f,-0.38429000973701477f,1.739796757698059f,0.11071659624576569f,-0.5578349828720093f,-1.3966891765594482f,0.9931953549385071f,0.12083596736192703f,-1.119937539100647f,1.1569610834121704f,-1.255689024925232f,-0.8284446001052856f,-0.7789045572280884f,0.8396943807601929f,0.6592835187911987f,-0.7762505412101746f,0.07665093243122101f,0.5355038642883301f,1.330856204032898f,-0.9771091938018799f,0.6867303252220154f,2.155921220779419f,-0.6703723073005676f,-0.13871923089027405f,-0.12890616059303284f,1.7173620462417603f,0.6137951016426086f,-0.8129093050956726f,-0.48122692108154297f,0.9553536176681519f,-2.090550184249878f,-1.276633381843567f,-0.9728832244873047f,-0.7788587212562561f,0.5572273135185242f,-0.8669276833534241f,1.4514567852020264f,0.6041662693023682f,-0.8715957403182983f,0.6566265821456909f,-0.35442492365837097f,-1.9534802436828613f,0.14539188146591187f,1.1624068021774292f,-1.3272678852081299f,-3.7432868480682373f,0.5986197590827942f,-0.10584727674722672f,0.3948514759540558f,-0.32438087463378906f,1.6463886499404907f,0.4740917980670929f,1.5766022205352783f,-1.2096710205078125f,1.0615670680999756f,-1.3492897748947144f,0.28981372714042664f,0.3958738446235657f,2.678384304046631f,0.4895554482936859f,1.3052397966384888f,0.4216163158416748f,-2.488093614578247f,-0.06541988253593445f,-1.1713378429412842f,0.3909022808074951f,-0.6610774397850037f,0.44140928983688354f,1.576289176940918f,-0.5046615600585938f,1.2668046951293945f,-0.37830838561058044f,-0.03482319042086601f,0.7565082311630249f,-0.9087958335876465f,0.6789819598197937f,-1.7877154350280762f,0.27885499596595764f,0.35343292355537415f,1.3279865980148315f,0.11716760694980621f,-1.3958089351654053f,-0.12792415916919708f,0.7268514633178711f,0.14876043796539307f,1.8716492652893066f,1.455806016921997f,-1.3294792175292969f,-2.329399824142456f,-1.3920276165008545f,-0.8372510671615601f,0.2524315416812897f,-0.6925874948501587f,1.2288575172424316f,0.41985416412353516f,0.5885820984840393f,0.2662945091724396f,-0.47000497579574585f,0.5659362077713013f,0.9441270232200623f,0.2289642095565796f,-1.264565348625183f,-0.27670350670814514f,-0.7474277019500732f,0.4136803150177002f,1.4291958808898926f,-0.8011531829833984f,0.05008536949753761f,0.03842703253030777f,-2.372786045074463f,-0.4649326205253601f,0.42141759395599365f,-1.05365788936615f,-0.42824897170066833f,1.6863161325454712f,-0.24962550401687622f,-0.7980019450187683f,0.30737996101379395f,-2.016227960586548f,-0.949831485748291f,-0.39631491899490356f,1.4361215829849243f,-1.0767807960510254f,1.1546647548675537f,1.4437267780303955f,-1.694633960723877f,1.6305773258209229f,1.2212979793548584f,-1.5525656938552856f,-0.5972793698310852f,0.38299816846847534f,0.7732217907905579f,-0.5580474138259888f,0.15525908768177032f,0.12849654257297516f,0.08134614676237106f,1.3661483526229858f,0.24050088226795197f,-0.5220624804496765f,-1.7362514734268188f,0.06188877299427986f,0.31645646691322327f,-1.2220325469970703f,-1.137510895729065f,-0.2274753600358963f,0.4803028106689453f,0.08553995192050934f,1.6629685163497925f,2.8365864753723145f,2.084420680999756f,-0.650897741317749f,-0.4924730360507965f,0.1458488255739212f,-0.14300811290740967f,-1.9421579837799072f,-0.5017433166503906f,0.6834090352058411f,-0.4323655366897583f,0.05956564471125603f,0.26074111461639404f,-1.3513070344924927f,0.8901072144508362f,-0.46946096420288086f,-0.5256887078285217f,-0.7699822783470154f,-0.8414955139160156f,-1.6318243741989136f,0.9800238609313965f,-1.7840200662612915f,1.4255027770996094f,-0.196116104722023f,-0.23450328409671783f,-0.8830403685569763f,-0.40835675597190857f,-0.9659604430198669f,-0.44821763038635254f,0.19555574655532837f,-0.05551241338253021f,1.5828986167907715f,0.4127323627471924f,0.805126428604126f,-0.2609803080558777f,1.7429431676864624f,1.0895956754684448f,-1.4957560300827026f,0.41808998584747314f,1.4815067052841187f,0.18012066185474396f,-1.9143375158309937f,0.5729184150695801f,-1.422263503074646f,1.4977695941925049f,0.9443099498748779f,-1.6649754047393799f,-0.4631813168525696f,-1.2518244981765747f,1.5581971406936646f,-0.6079233884811401f,0.9787799715995789f,-1.4138908386230469f,0.3230573534965515f,-2.9350759983062744f,0.7664175629615784f,-1.247719645500183f,-0.6406036615371704f,-0.8380113840103149f,0.8770043253898621f,-1.0945038795471191f,-0.75715571641922f,-1.6688346862792969f,1.859920859336853f,-0.2586114704608917f,0.8289657831192017f,-0.9760748744010925f,1.3331217765808105f,-1.869055986404419f,-0.13890312612056732f,-1.250659465789795f,0.1676565408706665f,0.3945501148700714f,-0.675550103187561f,-0.9292380809783936f,-1.9476193189620972f,1.645490050315857f,0.1725563406944275f,0.9536638855934143f,2.5894813537597656f,-0.5509234070777893f,0.4500579535961151f,1.2741528749465942f,-0.22959601879119873f,0.5947820544242859f,1.3617274761199951f,-2.2768239974975586f,-0.11506573855876923f,-0.5868954062461853f,-0.8405930399894714f,0.43449562788009644f,1.7289012670516968f,-1.0501152276992798f,-0.07134192436933517f,1.0727673768997192f,-0.8657787442207336f,0.3216242790222168f,-1.294527292251587f,-0.31056392192840576f,-1.5489288568496704f,0.13962046802043915f,1.151332974433899f,-0.6252493858337402f,0.301717072725296f,0.0014747012173756957f,-0.6994140148162842f,1.7839993238449097f,-0.2881630063056946f,2.131399154663086f,-0.5523581504821777f,2.797313690185547f,0.8360855579376221f,0.808743417263031f,-1.2769290208816528f,-1.704321265220642f,2.8971335887908936f,0.6984013319015503f,0.12035982310771942f,0.501411497592926f,-0.5439088940620422f,-0.45026788115501404f,0.35170531272888184f,1.0380666255950928f,-1.513520359992981f,-0.6213877201080322f,1.8182165622711182f,-0.758699893951416f,-0.18708691000938416f,-0.10642987489700317f,0.04332937300205231f,-0.7346760034561157f,0.6322576403617859f,-0.9449805021286011f,-0.909390926361084f,-1.4343743324279785f,-0.19948402047157288f,-0.033811286091804504f,-0.6656661033630371f,0.795276939868927f,-0.4523388743400574f,-0.03734462708234787f,0.3850216865539551f,0.6749634742736816f,-1.1305769681930542f,-0.33960092067718506f,1.326831340789795f,0.2090417742729187f,-0.8693652153015137f,-0.019779356196522713f,-0.9410842657089233f,-1.338707447052002f,0.14680853486061096f,0.5334003567695618f,0.9454014301300049f,-0.1416368931531906f,-2.2291157245635986f,-1.2876248359680176f,0.5840775370597839f,-2.363852024078369f,-0.06593005359172821f,0.9069058895111084f,-2.0508100986480713f,-1.9759912490844727f,0.9307206273078918f,0.5278058052062988f,-0.12275520712137222f,0.4745910167694092f,1.4855281114578247f,-0.026266828179359436f,1.099946141242981f,-0.08117303252220154f,-1.2654387950897217f,-2.0728001594543457f,0.08892153948545456f,0.7475523948669434f,0.04422776401042938f,-1.1540058851242065f,0.753757894039154f,2.86281156539917f,0.6926382184028625f,1.3542652130126953f,0.2865142524242401f,-1.2590528726577759f,0.5285171270370483f,-2.02626895904541f,0.6698327660560608f,1.8602790832519531f,-0.9908022284507751f,0.034619856625795364f,-0.22091907262802124f,0.02527048997581005f,-0.7541291117668152f,0.12390773743391037f,0.6848129630088806f,-0.19654551148414612f,2.4632136821746826f,-0.9078574180603027f,-0.0067704892717301846f,-0.16447588801383972f,1.5572036504745483f,-1.546546459197998f,0.04235158488154411f,-0.41553908586502075f,-2.504180431365967f,-0.03883764147758484f,0.2953188121318817f,0.644672155380249f,0.7677621245384216f,-0.2012675553560257f,1.7210100889205933f,0.25247085094451904f,0.6452300548553467f,-0.38538041710853577f,-0.842379629611969f,-0.4397237300872803f,-0.9711544513702393f,0.9306158423423767f,0.07553552836179733f,1.0040370225906372f,-0.6638365983963013f,-0.13943494856357574f,0.6656526327133179f,-0.5322849154472351f,0.42349478602409363f,-2.1998274326324463f,0.9052444100379944f,1.8483017683029175f,0.29905369877815247f,0.7080462574958801f,1.185766577720642f,-0.8768372535705566f,0.760583758354187f,0.32641106843948364f,-0.22300654649734497f,-0.23300008475780487f,1.98005211353302f,0.5509459972381592f,-1.0781471729278564f,0.3495485186576843f,-0.08222586661577225f,0.8894127011299133f,1.38327157497406f,1.0824211835861206f,1.024527907371521f,0.884548544883728f,0.07115980982780457f,0.03770419955253601f,0.020456496626138687f,-1.0428107976913452f,-0.9074733257293701f,0.6090823411941528f,-0.21544979512691498f,1.4619266986846924f,-0.4635932743549347f,0.8450746536254883f,-0.2620767056941986f,-0.5745557546615601f,-0.9823703169822693f,0.2548162639141083f,2.575385332107544f,-2.614264965057373f,0.1380075216293335f,-1.3402819633483887f,0.4623430669307709f,-1.4701905250549316f,-0.5392919778823853f,-0.433795303106308f,0.8937816619873047f,-1.6889275312423706f,-2.5795485973358154f,0.3729562759399414f,0.8730759024620056f,-0.7318193912506104f,-1.6778769493103027f,0.38491395115852356f,0.04971448704600334f,0.7358903884887695f,2.5139384269714355f,1.2952579259872437f,0.7413753271102905f,-1.0095853805541992f,0.7270404696464539f,0.5206758975982666f,-0.4310327172279358f,-0.6217920780181885f,-1.6815131902694702f,0.08788257092237473f,2.6858155727386475f,-1.8501229286193848f,1.1049622297286987f,-0.10757723450660706f,-1.2499463558197021f,0.6914932131767273f,0.11228231340646744f,1.3405333757400513f,0.672940194606781f,-1.9301512241363525f,0.6989908218383789f,0.8164222240447998f,-0.19201940298080444f,0.9525041580200195f,-0.3259885609149933f,-0.3250323534011841f,-0.16937939822673798f,-0.05238009989261627f,0.8387156128883362f,-1.0704288482666016f,-0.7319015264511108f,0.6889588236808777f,-0.12600159645080566f,0.9464280009269714f,0.3520755171775818f,0.02019459381699562f,0.9292787909507751f,0.7057912349700928f,0.6284950375556946f,0.27168917655944824f,-0.3078639805316925f,1.6953543424606323f,1.5617754459381104f,0.9421350955963135f,-1.0084145069122314f,0.0014193824026733637f,-0.05874774605035782f,-0.3680221736431122f,-0.1075301319360733f,0.37435591220855713f,-0.276750773191452f,0.25486326217651367f,0.8212080001831055f,-0.5533828139305115f,-1.5808998346328735f,1.2296463251113892f,-1.4341193437576294f,0.3271428942680359f,1.9921540021896362f,1.0384337902069092f,1.9470723867416382f,-1.222396969795227f,1.736872911453247f,2.3387351036071777f,0.011387352831661701f,0.9421340823173523f,-1.5473260879516602f,-0.6997211575508118f,-2.0264124870300293f,1.3882384300231934f,0.5250329971313477f,-0.6643905639648438f,1.3569505214691162f,4.994646072387695f,-0.1482471078634262f,-1.1799286603927612f,-0.401117205619812f,0.5065149068832397f,-1.7710891962051392f,-1.0351313352584839f,3.2857489585876465f,-0.279713898897171f,1.3038136959075928f,-1.1415935754776f,-0.9446572661399841f,-1.3098746538162231f,-0.20439329743385315f,2.029047966003418f,0.25595560669898987f,0.9988648295402527f,0.24551020562648773f,-2.7551450729370117f,-0.09923440217971802f,0.8984602689743042f,0.08624539524316788f,-0.9085845351219177f,-0.5277369618415833f,-1.2913094758987427f,-1.585619568824768f,2.8445966243743896f,0.2493039071559906f,-1.0088739395141602f,0.5217040777206421f,0.5398391485214233f,-0.22766432166099548f,2.0259883403778076f,-0.12053738534450531f,-1.1824017763137817f,1.1131128072738647f,1.5172417163848877f,-1.2717033624649048f,1.049655556678772f,1.7386442422866821f,-1.6732887029647827f,-1.6458741426467896f,0.6407695412635803f,2.924954652786255f,-0.41300830245018005f,1.2063934803009033f,1.5035215616226196f,-2.2746050357818604f,-0.5738024115562439f,-0.7770748138427734f,1.4526487588882446f,0.4005143344402313f,-0.2545435428619385f,-0.27897003293037415f,-0.5499287843704224f,-0.3997313678264618f,0.11936602741479874f,1.4352916479110718f,1.6283930540084839f,0.5262079834938049f,0.5050731897354126f,-0.02830806002020836f,-3.571762800216675f,2.9925825595855713f,-0.593949556350708f,0.36833348870277405f,-0.4208644926548004f,1.6308534145355225f,0.613275408744812f,-0.03638794273138046f,2.097604513168335f,-0.6768361926078796f,0.9410219192504883f,-1.2209492921829224f,1.135444164276123f,-1.2420636415481567f,-0.1254175752401352f,2.340583324432373f,0.007763226516544819f,-0.22858549654483795f,0.39078259468078613f,-0.7705473303794861f,-1.017773151397705f,0.298534631729126f,2.349973440170288f,0.8000704050064087f,2.5758159160614014f,-1.9013816118240356f,-1.668879508972168f,1.0118870735168457f,0.18720503151416779f,1.1493110656738281f,0.2111297845840454f,-1.478495478630066f,1.4094656705856323f,1.8202341794967651f,0.16810646653175354f,1.297280192375183f,-1.1401499509811401f,-1.9352047443389893f,-1.522018313407898f,-0.3813411593437195f,0.23052279651165009f,1.2553681135177612f,0.1530347764492035f,-0.09529052674770355f,-0.7272340655326843f,-1.0679214000701904f,0.47707533836364746f,-0.3388400971889496f,-0.4895890951156616f,-0.18853963911533356f,-1.199964165687561f,-0.6563532948493958f,0.9833365082740784f,0.3185252547264099f,0.032084766775369644f,-0.6947255730628967f,-0.10933377593755722f,0.43491822481155396f,1.4319140911102295f,0.2972373962402344f,2.0923655033111572f,-0.3475908637046814f,1.310921311378479f,-1.5124350786209106f,0.1364310085773468f,-0.32419732213020325f,0.2111256718635559f,-1.0756685733795166f,-1.4286329746246338f,-0.5151419639587402f,0.501664936542511f,0.5659027695655823f,-1.0112651586532593f,-2.0447897911071777f,-0.35999733209609985f,-0.5171621441841125f,1.3445446491241455f,0.3372036814689636f,1.174587607383728f,-2.567584276199341f,-0.14156419038772583f,-0.9155403971672058f,0.7216899991035461f,-0.597821056842804f,1.100205898284912f,0.1896505057811737f,1.9074783325195312f,0.15516166388988495f,-0.6321099400520325f,-2.0703530311584473f,-0.42079025506973267f,-0.7486847639083862f,1.4626071453094482f,-0.39729398488998413f,-1.6028803586959839f,-1.2104684114456177f,1.0937482118606567f,0.995397686958313f,-0.8197928071022034f,0.02123110182583332f,1.107080101966858f,-0.9763189554214478f,0.04317135736346245f,1.0855897665023804f,-0.7625353932380676f,-0.5818153619766235f,0.215140700340271f,-0.41188135743141174f,0.2277413308620453f,0.5564324855804443f,0.17371216416358948f,-0.6551352143287659f,1.2496949434280396f,-2.115906238555908f,-0.11677049100399017f,1.3921231031417847f,-0.07337921857833862f,-2.891545295715332f,-1.5494319200515747f,0.9985384941101074f,-0.621611475944519f,-1.1446919441223145f,-1.64154851436615f,0.003645162098109722f,0.590508759021759f,0.10894130915403366f,1.1827547550201416f,0.6881694197654724f,-1.0971696376800537f,2.175351619720459f,-1.8573657274246216f,-0.1192694902420044f,-3.6227216720581055f,0.41254687309265137f,0.9973989129066467f,-0.7219632863998413f,1.3813142776489258f,-2.0754942893981934f,0.6566239595413208f,0.418697327375412f,1.253435730934143f,-0.11425947397947311f,-0.19139482080936432f,-0.11443344503641129f,-1.1876356601715088f,-1.449568510055542f,-2.0894103050231934f,0.35928788781166077f,-0.903497040271759f,0.04465658962726593f,-1.425491452217102f,-1.0014333724975586f,1.060333013534546f,-2.297635555267334f,-1.3053842782974243f,0.051378704607486725f,-0.2130119353532791f,1.7249419689178467f,0.250926673412323f,-0.5058154463768005f,-0.4890441298484802f,0.5088837146759033f,0.6260073781013489f,0.4820159077644348f,-1.5178016424179077f,-0.5379753708839417f,-1.2255069017410278f,0.5270748138427734f,0.3077406585216522f,-0.9175942540168762f,0.08279333263635635f,0.49918466806411743f,0.4013134241104126f,-1.0264016389846802f,-0.2274608612060547f,-1.726405143737793f,0.06196824461221695f,-3.963779926300049f,1.1605653762817383f,-1.262897253036499f,0.9838938117027283f,-1.6371972560882568f,0.18996892869472504f,-0.22707442939281464f,-0.27495092153549194f,-0.5927217602729797f,2.068917989730835f,0.18927522003650665f,0.12209716439247131f,-1.2465900182724f,1.462017297744751f,1.7258514165878296f,2.1901307106018066f,0.3683038651943207f,1.367993950843811f,0.57856285572052f,-0.4133842885494232f,-0.28022170066833496f,-0.45024919509887695f,0.8466467261314392f,0.21426916122436523f,-0.8159845471382141f,0.030432021245360374f,0.7491520643234253f,0.8110830187797546f,-0.06877002120018005f,-0.3059714436531067f,1.3560600280761719f,-0.42414143681526184f,1.9422695636749268f,-0.08806569874286652f,1.5702940225601196f,0.9806006550788879f,-1.0644221305847168f,-1.3289639949798584f,-0.5236015915870667f,-0.5843896865844727f,0.5036380887031555f,-0.28897568583488464f,0.7779680490493774f,0.016488192602992058f,-0.823943018913269f,-0.5619749426841736f,0.7639733552932739f,0.3092685639858246f,0.3423632085323334f,-0.027882996946573257f,-1.2863653898239136f,-1.1213570833206177f,0.10850866138935089f,-0.646366536617279f,0.8203988671302795f,1.1528807878494263f,1.4782119989395142f,-0.4814014434814453f,1.269269347190857f,0.09769803285598755f,1.7629154920578003f,-0.18575844168663025f,0.0363864041864872f,-0.5400307774543762f,-1.2944707870483398f,1.7388801574707031f,0.2730674743652344f,1.5664947032928467f,0.6115381121635437f,-1.3646492958068848f,-0.4816740155220032f,0.668936550617218f,-0.5912403464317322f,-0.7940545678138733f,0.6101510524749756f,-0.009941724129021168f,0.3812825679779053f,0.7256529331207275f,2.2345783710479736f,-0.9479853510856628f,0.03962510824203491f,0.5125587582588196f,-0.10986649990081787f,0.5391301512718201f,-1.8083888292312622f,-0.36982446908950806f,0.8370636105537415f,-2.6002016067504883f,-0.7902308702468872f,-0.6165396571159363f,-0.49472281336784363f,0.1790379285812378f,1.5084354877471924f,-1.7989866733551025f,0.21215865015983582f,1.0400984287261963f,-0.19540047645568848f,-2.3358042240142822f,0.10973995923995972f,-0.44465047121047974f,0.46392253041267395f,1.3627082109451294f,1.5997670888900757f,-0.31115177273750305f,-0.5557456612586975f,0.07682377845048904f,0.37422704696655273f,-0.0762898325920105f,0.31200507283210754f,0.38536763191223145f,2.0686423778533936f,-0.6645202040672302f,-1.969874620437622f,0.4318414032459259f,1.469077229499817f,0.36483490467071533f,1.8399955034255981f,-0.5259129405021667f,0.7091758251190186f,0.030315613374114037f,1.6991491317749023f,-1.174354910850525f,-0.6192551851272583f,0.22804000973701477f,-0.7903926968574524f,1.3314025402069092f,-0.5327875018119812f,0.7035664916038513f,0.5116162300109863f,0.8014394640922546f,1.033012866973877f,0.5550866723060608f,1.2208850383758545f,0.3074093759059906f,1.2056818008422852f,-0.36815640330314636f,2.623826503753662f,-0.7439674735069275f,-0.43257537484169006f,1.020642876625061f,0.30094584822654724f,0.9848268628120422f,1.188188910484314f,-0.9032374024391174f,-0.6291894316673279f,-1.607364535331726f,0.42301011085510254f,0.821162223815918f,-0.9361718893051147f,0.8970841765403748f,-0.44083836674690247f,-0.8536504507064819f,-0.010068037547171116f,-0.5038336515426636f,-0.42468246817588806f,-0.7633004188537598f,1.068559169769287f,-0.7952914834022522f,-1.0569496154785156f,-1.616264820098877f,-0.768677294254303f,0.935364305973053f,1.0847020149230957f,-0.669205904006958f,1.2657248973846436f,0.5132566094398499f,-0.9263386726379395f,-1.2049871683120728f,-0.3054569661617279f,1.0714678764343262f,-1.5532927513122559f,-0.021551864221692085f,-0.11496306955814362f,-1.5829010009765625f,-2.6785364151000977f,1.062864065170288f,0.6178959608078003f,-1.4160959720611572f,0.6749258637428284f,2.002744436264038f,-0.08092573285102844f,-0.3720163404941559f,-0.2386963665485382f,1.2998807430267334f,-0.045511264353990555f,-0.03719594329595566f,0.5319620966911316f,0.9070133566856384f,-1.0669751167297363f,0.12780600786209106f,-0.16945983469486237f,-1.3637059926986694f,1.4207555055618286f,0.7387524247169495f,-2.2047934532165527f,-0.2520926594734192f,-0.34573429822921753f,-0.8151741623878479f,2.4319562911987305f,0.866479218006134f,0.6280125379562378f,0.32854264974594116f,1.4333298206329346f,-0.8506702780723572f,1.8114320039749146f,-0.5628202557563782f,-0.08072107285261154f,0.043507665395736694f,1.6479026079177856f,0.2747277319431305f,0.30906739830970764f,1.9693307876586914f,0.4901144504547119f,-0.4491022527217865f,0.18362046778202057f,0.3626183271408081f,-0.6820165514945984f,-0.3997882008552551f,1.7644871473312378f,1.7545924186706543f,-0.0032010520808398724f,-0.03125150501728058f,-0.0019124961690977216f,-0.7033900022506714f,1.6132924556732178f,0.9809855818748474f,1.913985013961792f,0.359355092048645f,-2.4568123817443848f,-1.083416223526001f,-0.8127052187919617f,0.1709938794374466f,0.6591188907623291f,0.02930375374853611f,0.05777359753847122f,-0.5012857913970947f,-0.8069295883178711f,-0.8890954256057739f,-2.0717520713806152f,0.4381219744682312f,0.059268124401569366f,0.5626673698425293f,-0.8057920932769775f,0.7631927728652954f,1.233093500137329f,0.2885158360004425f,-0.533180296421051f,-2.629587173461914f,-1.6441900730133057f,-0.974803626537323f,0.5080596208572388f,1.0678569078445435f,-1.0223509073257446f,-1.0995124578475952f,1.148068904876709f,1.0034371614456177f,2.155555248260498f,1.4384829998016357f,-0.05066565424203873f,-0.9296148419380188f,-0.5617964267730713f,1.4825245141983032f,-0.3421403169631958f,1.399376392364502f,-0.2985934615135193f,-1.3081543445587158f,0.20553356409072876f,0.4815302789211273f,-1.4606376886367798f,-1.1638144254684448f,-1.9260265827178955f,0.712680995464325f,-0.7138856649398804f,-0.6962532997131348f,0.4522978961467743f,0.4071761667728424f,0.7704771161079407f,2.30735445022583f,0.40932804346084595f,-0.2270374894142151f,2.2445995807647705f,1.1662789583206177f,-0.6348301768302917f,-0.8931717872619629f,-0.2803592085838318f,1.3735586404800415f,0.6336981058120728f,1.4856348037719727f,0.3075218200683594f,1.2210097312927246f,0.8550035953521729f,-1.0078305006027222f,-1.5568331480026245f,-1.702909231185913f,0.06473034620285034f,-0.4049573242664337f,-0.10452310740947723f,0.3320680558681488f,-0.9437150359153748f,1.3107326030731201f,-2.6251988410949707f,-0.6332289576530457f,-0.6867905259132385f,0.5667579174041748f,-1.6785459518432617f,0.22318877279758453f,-1.0696852207183838f,0.12050002813339233f,0.806024432182312f,-0.49312829971313477f,-1.0790531635284424f,2.6858179569244385f,-2.729773998260498f,2.1259260177612305f,1.15151047706604f,1.6617579460144043f,0.43240123987197876f,0.594704806804657f,-0.8392196893692017f,0.08340467512607574f,1.63368558883667f,1.0944851636886597f,1.0040868520736694f,1.3145886659622192f,-1.385730266571045f,-1.8741588592529297f,-2.5836021900177f,-0.43554365634918213f,0.4837324917316437f,1.2871934175491333f,0.3993145525455475f,0.8679110407829285f,-1.1287733316421509f,0.47382107377052307f,0.2665417194366455f,-1.3349181413650513f,0.7729313373565674f,-0.6398123502731323f,1.4223002195358276f,0.47503840923309326f,0.21417474746704102f,0.5343824028968811f,-0.1966671347618103f,-0.6226057410240173f,-0.061017703264951706f,0.6959181427955627f,1.7264984846115112f,0.31148579716682434f,-0.34152352809906006f,0.47010982036590576f,-0.6047739386558533f,-1.377835988998413f,0.17468301951885223f,-0.4074559807777405f,0.8646432757377625f,2.994428873062134f,-0.9478296637535095f,-0.4231865406036377f,0.43354564905166626f,-0.6809535026550293f,0.7620941996574402f,0.5434111952781677f,-0.6488415598869324f,2.207772731781006f,-0.10441956669092178f,-1.4786988496780396f,-0.36289018392562866f,-0.4140909016132355f,-0.879117488861084f,1.8730491399765015f,2.059922456741333f,0.9126044511795044f,-0.25397732853889465f,0.7309975028038025f,-0.39066505432128906f,0.6776297092437744f,-0.027958473190665245f,0.42506593465805054f,-2.0171825885772705f,0.7502687573432922f,1.8361274003982544f,0.02402033470571041f,1.4407316446304321f,-0.49049699306488037f,2.4429492950439453f,0.21200652420520782f,0.14018477499485016f,-1.1117877960205078f,0.14430376887321472f,-0.8589625954627991f,-0.42198848724365234f,-1.297855019569397f,-2.2116637229919434f,0.8205487132072449f,0.5412188768386841f,-0.07075420022010803f,2.1893670558929443f,-0.713729202747345f,1.0245686769485474f,-1.6527382135391235f,0.7479806542396545f,0.8648442625999451f,-0.9202672839164734f,0.7782939076423645f,0.5289826989173889f,-0.13668309152126312f,-0.806769609451294f,0.5048906207084656f,-0.4869341254234314f,1.0324811935424805f,-0.23023411631584167f,-0.9341028928756714f,-1.0191806554794312f,-0.1080290749669075f,0.17960387468338013f,1.1287531852722168f,1.487612247467041f,1.5318762063980103f,1.909412145614624f,0.15096767246723175f,0.19431501626968384f,-1.2489413022994995f,0.5509757399559021f,0.2817719876766205f,0.49972590804100037f,-3.161163330078125f,-0.6786826848983765f,-0.38585442304611206f,-1.5332715511322021f,0.26021549105644226f,-0.11310989409685135f,0.14284908771514893f,-0.5352863669395447f,0.2790553867816925f,-2.218024730682373f,-2.8155603408813477f,-1.7700344324111938f,-1.5088751316070557f,-0.11264436691999435f,-1.239003300666809f,0.8185913562774658f,-2.2408087253570557f,-0.8237045407295227f,-1.4761648178100586f,-1.7548408508300781f,1.4327881336212158f,1.69144868850708f,0.6726519465446472f,0.822871744632721f,-0.5639218688011169f,1.1850700378417969f,0.5544999241828918f,0.464884489774704f,1.7744386196136475f,-0.7203508019447327f,-0.6785759329795837f,-0.41518306732177734f,1.4214141368865967f,-0.8335444927215576f,-0.8691821098327637f,1.2176731824874878f,0.6125086545944214f,1.6175997257232666f,-1.610172152519226f,0.5688914656639099f,-0.4320448637008667f,0.3454733192920685f,-0.009405218064785004f,1.861645221710205f,0.1134076938033104f,-1.2495958805084229f,-0.3049474358558655f,0.3689897060394287f,-0.8720254302024841f,1.9052239656448364f,-1.828892469406128f,0.531408965587616f,0.28060415387153625f,0.24533292651176453f,-0.6413555145263672f,1.7047454118728638f,0.05561106652021408f,1.236993670463562f,1.301370620727539f,-0.9309923648834229f,1.7768981456756592f,1.6320618391036987f,-0.7346504330635071f,1.2532542943954468f,0.8544823527336121f,-1.2182923555374146f,-0.009568984620273113f,3.1160905361175537f,-0.6132407784461975f,-0.42478838562965393f,1.542183518409729f,-2.565136194229126f,-0.23791369795799255f,0.6454435586929321f,-0.1391819268465042f,0.7752273678779602f,-0.6562255024909973f,-0.4811801314353943f};
alignas(16) float batch_normalization_8_A[] = {-0.027106046676635742f,1.1145867109298706f,0.26178401708602905f,-0.05873005464673042f,-0.40778499841690063f,-0.4089793562889099f,0.17722921073436737f,0.8513494729995728f,-0.362763911485672f,-0.44112342596054077f,0.053377121686935425f,0.02475319802761078f,0.090097576379776f,0.11851856112480164f,-0.4646701216697693f,0.35359472036361694f,0.34442567825317383f,0.15887439250946045f,1.0218936204910278f,-0.6092986464500427f,-0.39047715067863464f,0.5566542744636536f,-0.09708614647388458f,0.36393117904663086f,0.16839908063411713f,-0.24416683614253998f,0.7791377305984497f,-0.16028019785881042f,0.39228397607803345f,-0.5130518078804016f,0.1284460425376892f,-0.3078066408634186f};
alignas(16) float conv2d_4_internal_1_W[] = {-0.5495547652244568f,0.08827636390924454f,0.9595202803611755f,1.179256796836853f,-0.20829136669635773f,-1.994078516960144f,-1.8252297639846802f,0.6170768141746521f,-0.09387797117233276f,0.9778481125831604f,0.7511487603187561f,-0.2529546320438385f,-0.06665759533643723f,0.04650628939270973f,0.7995564341545105f,0.004516520071774721f,0.15775658190250397f,-0.3903629779815674f,-0.22930331528186798f,0.8534806370735168f,-1.0904003381729126f,-1.029885172843933f,2.557260274887085f,-0.4805282950401306f,0.3032746911048889f,1.1227320432662964f,0.05203447863459587f,-0.10932666063308716f,-0.5051159858703613f,0.6809855699539185f,-0.40086033940315247f,-1.4958791732788086f,-0.8854008913040161f,0.5790826082229614f,0.9259213209152222f,0.5982491970062256f,-0.6199505925178528f,0.9364700317382812f,0.9843881130218506f,-1.058496117591858f,0.05246947705745697f,0.5524083375930786f,-1.1291134357452393f,-0.6525983214378357f,0.2279055416584015f,-1.0383473634719849f,-0.19188855588436127f,-0.41806647181510925f,-1.3517950773239136f,-1.200310230255127f,-0.5523784756660461f,-0.5427539348602295f,-0.17786185443401337f,0.5885871052742004f,1.3740097284317017f,-1.0570905208587646f,0.4399990439414978f,-1.0498316287994385f,0.6519782543182373f,0.8612268567085266f,0.5066407918930054f,-1.2651571035385132f,-0.8896191120147705f,-0.5443257093429565f,-0.6640299558639526f,-0.05831468850374222f,-1.64122474193573f,0.15844233334064484f,1.4712553024291992f,0.6029887199401855f,0.34527692198753357f,-0.32310545444488525f,1.3806244134902954f,-2.3183181285858154f,-0.10021437704563141f,0.7978218793869019f,-0.09778860211372375f,0.018738673999905586f,-0.3517918884754181f,0.5577914714813232f,0.11440517008304596f,1.4120748043060303f,-1.471941590309143f,0.016801847144961357f,1.3288978338241577f,-0.050049491226673126f,0.9806168079376221f,0.03206968680024147f,0.7043125033378601f,1.599475622177124f,0.22184611856937408f,-1.60374915599823f,0.29431378841400146f,-2.0228514671325684f,-0.4340192973613739f,-1.4157830476760864f,0.046701475977897644f,1.4421908855438232f,0.5009492635726929f,-1.0591360330581665f,1.1345949172973633f,-0.47608017921447754f,-0.32169923186302185f,0.05733400583267212f,-0.8376668095588684f,-0.5222532153129578f,0.35010504722595215f,0.22102557122707367f,-1.8270705938339233f,-0.8345415592193604f,-0.43837669491767883f,-1.2705388069152832f,-0.5469933152198792f,0.11155872792005539f,1.2708834409713745f,-0.04118332266807556f,-0.12098685652017593f,1.5895686149597168f,0.7445785999298096f,-0.6754841208457947f,0.4432578980922699f,0.4459784924983978f,-0.4816552698612213f,-0.2941962778568268f,-1.0961989164352417f,-2.5014407634735107f,-0.08704990148544312f,0.2862894535064697f,-0.29502150416374207f,-0.06971198320388794f,0.3808874487876892f,1.462708830833435f,-0.4148173928260803f,-0.4491385817527771f,0.6719251275062561f,0.8681918978691101f,0.8951886892318726f,-0.586585521697998f,-0.33781349658966064f,0.27868396043777466f,0.5787423253059387f,-1.5448524951934814f,-1.1200984716415405f,0.428519606590271f,1.56434166431427f,0.32070380449295044f,-0.9561066627502441f,1.7943819761276245f,-0.9730521440505981f,0.542691171169281f,0.5643094182014465f,1.7823331356048584f,-0.08038777112960815f,-0.6450923681259155f,0.07130099833011627f,-0.6845033168792725f,1.5658080577850342f,0.27302905917167664f,0.18736208975315094f,-0.9168381690979004f,-1.0004414319992065f,-0.11031178385019302f,0.6979702115058899f,-0.9794228076934814f,-1.0689024925231934f,0.3107301890850067f,-1.4761028289794922f,-0.28204572200775146f,-0.6316632032394409f,0.8346064686775208f,-0.40408870577812195f,-1.564469575881958f,-0.2994978427886963f,0.0032136065419763327f,-0.6909140944480896f,1.5492709875106812f,1.0853813886642456f,-0.14249372482299805f,1.1912473440170288f,-0.5977888703346252f,1.708953857421875f,-0.14089404046535492f,-0.4850935935974121f,-0.8665021061897278f,0.4383447468280792f,0.42241233587265015f,0.02060188539326191f,-0.7573102712631226f,0.17321933805942535f,1.8125338554382324f,0.8234654068946838f,0.007267134729772806f,0.9650879502296448f,-1.0892744064331055f,0.9861104488372803f,-0.4429458677768707f,0.08999928086996078f,0.9890462160110474f,1.1023141145706177f,1.7251496315002441f,-0.006805381737649441f,1.227614402770996f,0.2026820182800293f,-0.6888874769210815f,0.16049370169639587f,-1.8754558563232422f,0.8515576124191284f,-0.042980317026376724f,0.2952722907066345f,0.26905933022499084f,0.365325391292572f,0.6032377481460571f,0.480863094329834f,-0.050579193979501724f,-0.3498765528202057f,0.5399994850158691f,1.3880698680877686f,0.07979337126016617f,0.04308752715587616f,-1.5089985132217407f,-1.1182537078857422f,-2.1490275859832764f,-1.244002342224121f,0.4831773340702057f,-0.21977859735488892f,0.38344183564186096f,0.8837953805923462f,-1.4745303392410278f,-0.4503341615200043f,0.19843840599060059f,0.3071787655353546f,0.42105454206466675f,-0.09186529368162155f,1.2041504383087158f,-0.6123234033584595f,0.04225732386112213f,1.7007306814193726f,-1.4375001192092896f,-1.1085020303726196f,-0.7559415102005005f,-1.3718833923339844f,-0.40438175201416016f,1.1775906085968018f,1.2727054357528687f,0.5659869313240051f,-1.0040299892425537f,-0.21983692049980164f,0.21390476822853088f,0.3833356201648712f,-0.6666890382766724f,0.059174779802560806f,-0.4294702112674713f,1.2534747123718262f,0.4360910952091217f,1.2982321977615356f,-0.11967843770980835f,1.0688940286636353f,0.8842142820358276f,0.4813399612903595f,-0.33687177300453186f,1.1438355445861816f,0.22977186739444733f,0.7619417309761047f,-0.40534451603889465f,-0.6141559481620789f,0.724619448184967f,0.7484862208366394f,-1.1433656215667725f,1.7742284536361694f,0.5038570165634155f,-0.7270546555519104f,1.1988037824630737f,-0.5782279372215271f,0.3535800278186798f,-0.528640627861023f,0.4505285918712616f,-0.767387330532074f,-1.5887980461120605f,-0.975302517414093f,1.4352519512176514f,-0.6057296991348267f,-0.05115267261862755f,0.5993708372116089f,0.6024541854858398f,1.3206150531768799f,0.20648731291294098f,1.1707143783569336f,0.15229180455207825f,0.05746348574757576f,-0.31607377529144287f,0.08503596484661102f,1.6171672344207764f,0.3006448447704315f,1.628265142440796f,1.2047070264816284f,-1.3414777517318726f,-0.6311999559402466f,-0.10814090818166733f,-0.39180564880371094f,0.264008104801178f,0.3407128155231476f,-0.6264017224311829f,1.3668005466461182f,-0.929449200630188f,-0.8653945922851562f,1.1979644298553467f,-2.3757827281951904f,-0.47358930110931396f,-2.418138265609741f,0.2053573578596115f,0.9550119042396545f,-0.8623887896537781f,0.4260425865650177f,0.20673847198486328f,0.6167595982551575f,-1.2581125497817993f,-1.345259428024292f,1.320618748664856f,0.8255565762519836f,-0.014396212995052338f,1.3536301851272583f,0.009138244204223156f,-0.4111378490924835f,-1.417517066001892f,0.26450759172439575f,-0.8168500065803528f,1.1560372114181519f,-0.4868868887424469f,-0.08647657185792923f,-0.4770312011241913f,0.26160094141960144f,-1.7019912004470825f,-0.17167650163173676f,0.7111284136772156f,1.0241365432739258f,0.5370477437973022f,-0.04127621278166771f,-0.5939601063728333f,-0.7007126212120056f,-1.235559344291687f,-0.3278629183769226f,0.18669885396957397f,1.0776463747024536f,1.5743663311004639f,0.6523972153663635f,-0.002702980302274227f,-0.7505940794944763f,1.0684548616409302f,0.8120880126953125f,0.5149155259132385f,1.1643110513687134f,0.06769563257694244f,-1.4226499795913696f,0.01828852854669094f,0.045034259557724f,-0.9964288473129272f,0.7145246863365173f,0.14428049325942993f,-0.8457536697387695f,-0.877446711063385f,-0.495326966047287f,-1.5669965744018555f,1.02727210521698f,-0.1663721203804016f,-1.9281686544418335f,0.11332376301288605f,1.2314687967300415f,0.38899073004722595f,0.28757062554359436f,0.2669886350631714f,0.08669428527355194f,-0.45314252376556396f,-0.016621660441160202f,2.4157257080078125f,-1.4639400243759155f,0.025036228820681572f,-0.11460937559604645f,0.960353434085846f,-0.15551559627056122f,-0.07935982197523117f,0.6564611792564392f,1.8407784700393677f,-0.34022170305252075f,-0.059230007231235504f,0.07866077870130539f,2.119553804397583f,0.5438913702964783f,-0.21595589816570282f,0.32866770029067993f,0.5328648686408997f,1.2628713846206665f,0.8185927867889404f,-0.6176700592041016f,-0.9196295738220215f,0.17132414877414703f,-0.7447543144226074f,-0.27285417914390564f,0.7392719984054565f,-0.4261101186275482f,-0.2173735350370407f,0.9755373597145081f,-0.4615398049354553f,1.0663740634918213f,-1.1075592041015625f,-0.10698848217725754f,0.989544153213501f,0.6976580023765564f,1.4316776990890503f,0.15961605310440063f,-0.742303729057312f,1.09288489818573f,-0.4872070550918579f,-0.2559284567832947f,0.24416640400886536f,-1.2394667863845825f,-0.9929952621459961f,0.994649350643158f,-0.3120747208595276f,0.46003660559654236f,-1.5515408515930176f,0.04449860379099846f,-2.156294584274292f,-0.6104418635368347f,-0.5457209944725037f,0.8751130700111389f,0.9633719325065613f,0.18950927257537842f,0.43444928526878357f,-1.4678019285202026f,-0.08959091454744339f,-0.9805027842521667f,-1.1332992315292358f,-0.7897589206695557f,-0.006876806728541851f,-1.0022389888763428f,1.0157870054244995f,-0.38023117184638977f,-0.7701719999313354f,-1.016012191772461f,-0.1852794587612152f,-0.5703645944595337f,0.6430736780166626f,0.8153977990150452f,-0.041748251765966415f,-0.06984017789363861f,-0.012999641709029675f,-2.013320207595825f,0.052862439304590225f,0.08824873715639114f,-0.6918822526931763f,0.3356422781944275f,-1.6465022563934326f,-0.3083253502845764f,-0.47960296273231506f,-0.23902934789657593f,-0.41997984051704407f,-0.33781784772872925f,0.5920739769935608f,-0.7231955528259277f,-0.3798774480819702f,-0.7316527962684631f,1.8130398988723755f,1.2197425365447998f,0.4703536033630371f,2.004422187805176f,0.6869490146636963f,1.4867678880691528f,-1.6175944805145264f,-0.8375113010406494f,-0.6877923607826233f,-0.17682524025440216f,-0.6581389904022217f,0.11620721966028214f,-0.8137093782424927f,0.4479621648788452f,-1.364580750465393f,-1.126213550567627f,-0.21898186206817627f,-0.2538015842437744f,0.6760547161102295f,-0.2627030611038208f,0.655341625213623f,1.4545997381210327f,-0.32623451948165894f,-0.6864230632781982f,-1.1246525049209595f,-1.1196261644363403f,-1.5265640020370483f,0.8210833668708801f,-0.21813471615314484f,-0.3711852431297302f,0.3210268020629883f,0.4583855867385864f,2.059414863586426f,-0.2234325408935547f,-1.436331868171692f,1.0055437088012695f,-0.7964109778404236f,0.4860649108886719f,2.166708469390869f,-0.7362135052680969f,1.0279083251953125f,-0.2995017170906067f,-1.504675030708313f,-0.168874129652977f,0.1173662394285202f,0.1778520941734314f,1.5795248746871948f,-0.48612070083618164f,0.2265946865081787f,0.9553796052932739f,0.42746278643608093f};
alignas(16) float batch_normalization_9_A[] = {-0.35937055945396423f,-0.6057433485984802f,-1.5584752559661865f,0.7772523164749146f,-0.4958341717720032f,-0.1985667645931244f,-1.601257085800171f,1.5715248584747314f,-0.35255441069602966f,0.07914847135543823f,-0.5286636352539062f,1.8122937679290771f,1.0890158414840698f,0.46450304985046387f,1.2030268907546997f,1.4009193181991577f};
alignas(16) float separable_conv2d_5_internal_0_VALUES[] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
alignas(16) float separable_conv2d_5_internal_1_W[] = {-0.025021379813551903f,0.0933736190199852f,-0.12115798145532608f,-0.0009356762166135013f,-0.025679055601358414f,0.12598741054534912f,-0.04851049929857254f,0.042395755648612976f,-0.0974099412560463f,-0.08737484365701675f,-0.0667291060090065f,-0.048442550003528595f,-0.05007233843207359f,-0.018436511978507042f,-0.18405945599079132f,-0.0331512987613678f,-0.005734529811888933f,-0.09207940101623535f,0.07591966539621353f,0.10119751840829849f,-0.07361071556806564f,0.04386475682258606f,0.07208282500505447f,0.09731616079807281f,-0.09014563262462616f,-0.05208621546626091f,0.0371900349855423f,0.08577486872673035f,0.07868997752666473f,-0.05434826388955116f,-0.1019689291715622f,-0.02669902704656124f,-0.09946492314338684f,0.12638725340366364f,0.03851480409502983f,-0.03955472633242607f,0.0854400023818016f,-0.04578502103686333f,-0.08198436349630356f,-0.10544852912425995f,0.089118592441082f,-0.17948906123638153f,0.02988247014582157f,-0.08240775018930435f,0.08316579461097717f,0.03811786696314812f,-0.04934321343898773f,0.09407011419534683f,0.032398223876953125f,-0.08911221474409103f,0.09172538667917252f,-0.05309204012155533f,-0.03617561236023903f,0.035391490906476974f,-0.1446027159690857f,-0.06830330193042755f,-0.00011304165673209354f,-0.04760336875915527f,0.14843426644802094f,0.10023345053195953f,0.10814071446657181f,0.10328666865825653f,-0.019506260752677917f,-0.03868340700864792f,0.06263139843940735f,0.04795320704579353f,-0.08867542445659637f,-0.06516998261213303f,-0.042368341237306595f,0.09277059882879257f,0.005455653183162212f,0.01360922772437334f,-0.10999337583780289f,-0.0356520377099514f,-0.012018335051834583f,-0.10474172979593277f,-0.03001120500266552f,-0.06440052390098572f,-0.05427924916148186f,-0.07907602936029434f,0.10417450219392776f,-0.06585537642240524f,0.08347112685441971f,-0.0009324628626927733f,-0.10650935024023056f,0.08007669448852539f,-0.04298654943704605f,0.09442035108804703f,0.006293918937444687f,-0.08637810498476028f,0.00286534009501338f,0.1455942541360855f,0.046386756002902985f,0.0053787329234182835f,0.026218675076961517f,-0.11205966770648956f,0.05608675256371498f,0.11341340094804764f,-0.09770910441875458f,-0.022809959948062897f,0.004206619691103697f,-0.1320328265428543f,0.03522925078868866f,-0.019866757094860077f,0.13831670582294464f,-0.0033711278811097145f,0.01705157943069935f,0.045796651393175125f,-0.00025142915546894073f,-0.03340201452374458f,-0.0924399271607399f,0.057260893285274506f,0.016269829124212265f,0.0734349712729454f,-0.04934614151716232f,-0.022231116890907288f,0.09105277061462402f,0.10543393343687057f,0.03554437309503555f,0.06188236549496651f,0.029811792075634003f,0.04458220675587654f,0.03633943572640419f,0.11920962482690811f,-0.007296903058886528f,0.0932004526257515f,0.01972048357129097f,0.07842367142438889f,0.002298052189871669f,0.026848267763853073f,-0.037063442170619965f,-0.010125831700861454f,-0.017892252653837204f,0.07119626551866531f,-0.055326227098703384f,-0.13823121786117554f,0.014431233517825603f,0.040331222116947174f,0.13675634562969208f,-0.10831021517515182f,0.07250732183456421f,0.008245584554970264f,0.07942156493663788f,0.025208203122019768f,0.05023019760847092f,0.0631508007645607f,0.04156965762376785f,-0.04572119191288948f,-0.1027398332953453f,-0.055194832384586334f,-0.0947556421160698f,-0.05177823826670647f,-0.09078983217477798f,-0.031450290232896805f,-0.030291540548205376f,-0.03308138623833656f,-0.06804510205984116f,0.05010363832116127f,-0.04599207267165184f,-0.1153806820511818f,0.024944094941020012f,0.12393427640199661f,0.02724963054060936f,-0.035742759704589844f,-0.0953301191329956f,-0.029536640271544456f,0.03298332542181015f,0.013002955354750156f,-0.017101556062698364f,-0.06225767731666565f,-0.0821845605969429f,-0.004758073482662439f,-0.08226712793111801f,0.0411544106900692f,-0.03666363283991814f,-0.03515370562672615f,-0.07953516393899918f,0.06454618275165558f,-0.03111935220658779f,0.040796760469675064f,0.11400392651557922f,-0.026986505836248398f,-0.06909753382205963f,-0.07468857616186142f,0.1804715394973755f,-0.018276434391736984f,-0.039873335510492325f,0.06405165791511536f,0.027730604633688927f,0.06747513264417648f,-0.053684793412685394f,-0.1413927525281906f,-7.741560693830252e-05f,-0.08564218133687973f,-0.028241034597158432f,-0.08228582888841629f,0.12418612092733383f,0.09638039767742157f,-0.02602561190724373f,0.07372388243675232f,-0.026542600244283676f,-0.0068259406834840775f,0.09982715547084808f,0.07297398895025253f,-0.05428101867437363f,-0.07778865844011307f,-0.06228630989789963f,0.12015888094902039f,-0.002340478589758277f,-0.14238688349723816f,-0.011913175694644451f,-0.01297969650477171f,-0.02916623465716839f,-0.06274620443582535f,0.06408637017011642f,0.09415993839502335f,0.028432747349143028f,0.045484788715839386f,0.11762847006320953f,0.010975322686135769f,-0.07871290296316147f,-0.028628434985876083f,-0.09850902855396271f,-0.02856363169848919f,-0.0764337033033371f,-0.040554679930210114f,-0.08602239936590195f,-0.06625966727733612f,0.09011461585760117f,-0.027293594554066658f,0.14314989745616913f,-0.04813206195831299f,0.10653694719076157f,0.050059974193573f,-0.020500602200627327f,0.031249875202775f,0.10341502726078033f,-0.11236177384853363f,-0.08959472924470901f,-0.030800150707364082f,-0.10599077492952347f,0.02641354687511921f,0.1404321789741516f,0.06521609425544739f,-0.09121355414390564f,0.07891090214252472f,0.01665632613003254f,-0.030829858034849167f,0.0008201824966818094f,0.0637466311454773f,-0.019633008167147636f,0.07013782858848572f,0.13293148577213287f,-0.050293177366256714f,-0.025197958573698997f,-0.002522775437682867f,-0.051192738115787506f,0.07027240842580795f,0.14990125596523285f,-0.13826501369476318f,-0.06023399159312248f,-0.06845554709434509f,-0.07911095023155212f,0.13111001253128052f,-0.08376950770616531f,-0.03633158653974533f,-0.02524438127875328f,0.04889008402824402f,-0.17441900074481964f,-0.041612833738327026f,0.05873838812112808f,0.040025290101766586f,0.053182072937488556f,-0.09592832624912262f,0.009949074126780033f,-0.0927649736404419f,-0.07082175463438034f,-0.06946997344493866f,0.04582523554563522f,0.08164838701486588f,-0.0015823256690055132f,0.11360485851764679f,0.10264565795660019f,0.07741636782884598f,0.02466464973986149f,-0.0505121573805809f,0.05625107139348984f,-0.08567368984222412f,-0.01734616421163082f,-0.036528538912534714f,-0.019023682922124863f,-0.16151678562164307f,-0.0025191493332386017f,-0.021906264126300812f,-0.017021220177412033f,0.026201259344816208f,0.037470027804374695f,0.05534626170992851f,-0.016500001773238182f,-0.0635947659611702f,-0.0008065333822742105f,-0.02408755198121071f,-0.10637281090021133f,-0.09266582131385803f,0.0014417215716093779f,0.07705719769001007f,0.07177182286977768f,-0.059014059603214264f,-0.034423649311065674f,0.08491116762161255f,-0.04188237711787224f,0.08050016313791275f,-0.08573772758245468f,0.0640747994184494f,-0.06775527447462082f,0.07295050472021103f,0.03917897492647171f,0.010270635597407818f,0.12131598591804504f,0.03268427029252052f,0.0708015114068985f,0.03852676600217819f,0.09556876868009567f,-0.0967659130692482f,-0.0480080209672451f,0.02155621163547039f,-0.05927343666553497f,-0.0469701774418354f,-0.0857096016407013f,-0.023321250453591347f,0.0168947521597147f,0.04245142638683319f,0.01319591049104929f,-0.05950981751084328f,0.11405492573976517f,-0.0802990049123764f,0.04751448333263397f,0.05510096624493599f,0.012884608469903469f,-0.053239647299051285f,-0.09290195256471634f,-0.09809496253728867f,-0.07211251556873322f,-0.006352487485855818f,-0.11683657765388489f,0.039507437497377396f,-0.008057190105319023f,0.013100084848701954f,0.007154074031859636f,0.06427338719367981f,-0.046141672879457474f,-0.04185926541686058f,0.12665525078773499f,0.02311830408871174f,0.038397274911403656f,-0.09021038562059402f,-0.12035086005926132f,-0.0565941296517849f,-0.009592810645699501f,-0.0025630055461078882f,0.0002270905824843794f,0.007005695253610611f,-0.13006891310214996f,-0.022873111069202423f,-0.02826046198606491f,0.024071384221315384f,-0.07753919810056686f,-0.03774923458695412f,0.02054579369723797f,0.03339430317282677f,-0.018160918727517128f,-0.05701062083244324f,0.0030665616504848003f,0.05984548479318619f,-0.06429387629032135f,0.021555308252573013f,0.0019393934635445476f,-0.05298911780118942f,-0.11534959822893143f,0.01748114638030529f,-0.053014032542705536f,0.09366490691900253f,-0.029160795733332634f,-0.05723349377512932f,0.030764596536755562f,0.014245015569031239f,-0.05932486802339554f,-0.06454291939735413f,0.15216541290283203f,-0.030104009434580803f,-0.07689543813467026f,0.007765078451484442f,-0.01087719015777111f,-0.1135612428188324f,0.09677021950483322f,0.02168823964893818f,-0.002548798220232129f,-0.0848291888833046f,-0.12054947018623352f,0.04241306334733963f,0.10435687750577927f,0.029400760307908058f,-0.03340239077806473f,0.07327302545309067f,0.026166953146457672f,-0.01525951735675335f,-0.1258556991815567f,0.03889399394392967f,0.014768281951546669f,0.015371451154351234f,-0.18292580544948578f,0.041210711002349854f,-0.07812425494194031f,-0.04178491234779358f,0.006715601775795221f,0.024883998557925224f,-0.015850303694605827f,0.06376484036445618f,-0.04083938151597977f,0.0024043782614171505f,-0.040971789509058f,0.0633283331990242f,0.06086813658475876f,-0.11833833158016205f,0.013626948930323124f,-0.02832873910665512f,-0.017954103648662567f,0.11973533034324646f,0.08596785366535187f,-0.014883173629641533f,0.07464814186096191f,-0.11436596512794495f,-0.15248340368270874f,-0.07095621526241302f,-0.047497399151325226f,-0.06490494310855865f,-0.014448797330260277f,0.02479696087539196f,-0.04208245873451233f,0.11503444612026215f,0.0008582990267314017f,0.08028857409954071f,0.01720448024570942f,0.005293673370033503f,0.09718028455972672f,-0.01587197743356228f,-0.08067277818918228f,0.018481044098734856f,-0.0027171242982149124f,0.16302801668643951f,0.014386770315468311f,0.0564163401722908f,0.019852502271533012f,-0.03350743278861046f,-0.09433645009994507f,-0.05493856221437454f,0.10193470865488052f,-0.14209142327308655f,0.04816487804055214f,0.019408093765378f,-0.002951253205537796f,-0.10661487281322479f,0.04159052297472954f,-0.030552254989743233f,0.036619316786527634f,-0.0036975154653191566f,-0.09144052118062973f,0.047735560685396194f,0.09040965884923935f,-0.035508573055267334f,-0.007265113294124603f,0.11686746776103973f,-0.016620725393295288f,0.14043255150318146f,-0.09960444271564484f,0.06202676147222519f,-0.10115734487771988f,-0.08743616938591003f,0.07047145068645477f,0.02643943950533867f,-0.007546171545982361f,0.044100042432546616f,-0.07663566619157791f,0.05755013972520828f,-0.029014984145760536f,-0.02232074737548828f,-0.08166588842868805f,-0.04024776443839073f,0.021509043872356415f,-0.03499823063611984f,0.05249074101448059f,0.11353850364685059f,0.06649612635374069f,0.10141853988170624f,0.040691010653972626f,-0.03466655686497688f,0.005223134532570839f,-0.0376083180308342f,0.035040706396102905f,-0.11773788183927536f,-0.06953658908605576f,-0.045944105833768845f,-0.090083546936512f,0.09872731566429138f,0.04180042818188667f,0.06165548413991928f,0.03733263909816742f,-0.04347917065024376f,0.00262358202598989f,0.055724672973155975f,0.02267659269273281f,0.08566980063915253f,0.11317113041877747f,0.06197613477706909f,-0.007179229520261288f,0.03670758754014969f,-0.005531684961169958f,-0.050466980785131454f,-0.03929949924349785f,-0.05412604287266731f,0.006887241266667843f,-0.0954979881644249f,-0.011607123538851738f,0.004861908499151468f,0.13822205364704132f,-0.06867368519306183f,0.015188165940344334f,-0.038569893687963486f,-0.01153318677097559f,-0.0023095032665878534f,-0.0019981602672487497f,0.010075986385345459f,0.011739622801542282f,-0.039761364459991455f,-0.004747206810861826f,0.026294294744729996f,-0.02741508185863495f,0.1421622484922409f,0.005971800070255995f,-0.04544941335916519f,-0.02635725773870945f,0.11441275477409363f,-0.05245916545391083f,0.0016986258560791612f,-0.024660153314471245f,0.006412564776837826f,0.08597307652235031f,-0.016646750271320343f,-0.029186224564909935f,-0.049935974180698395f,-0.06907419115304947f,-0.050150513648986816f,-0.044962286949157715f,-0.10320493578910828f,-0.10210685431957245f,-0.008667104877531528f,0.07179158180952072f,0.09914441406726837f,-0.023619599640369415f,0.010351601988077164f,-0.03821442648768425f,0.07868707180023193f,-0.04127197712659836f,-0.07764934748411179f,-0.02963978610932827f,-0.030490800738334656f,-0.055560093373060226f,0.06166867911815643f,0.07874996960163116f,0.05640651658177376f,-0.010900064371526241f,-0.05636313557624817f,-0.0498986542224884f,0.019779009744524956f,-0.13621123135089874f,0.03363920375704765f};
alignas(16) float separable_conv2d_5_internal_2_W[] = {0.5048772692680359f,1.7382426261901855f,0.4075077474117279f,-0.088250532746315f,-0.3534296452999115f,1.109460711479187f,0.5787704586982727f,-1.0598567724227905f,-0.4432809054851532f,0.5410486459732056f,1.052991509437561f,-0.9130055904388428f,-0.6277876496315002f,0.758272647857666f,-0.09527759999036789f,0.6417738795280457f,-2.025949239730835f,0.23069944977760315f,-0.10907460749149323f,0.6233645677566528f,-1.135371446609497f,0.5866292119026184f,0.44432875514030457f,0.6695668697357178f,-0.6973147392272949f,0.47615599632263184f,0.06570399552583694f,0.4165245592594147f,-1.6186254024505615f,-2.0961201190948486f,0.13755373656749725f,-3.2444679737091064f,0.27774402499198914f,1.1748491525650024f,-0.3899081349372864f,-0.028745045885443687f,-0.7770465016365051f,0.9704984426498413f,-0.2062753140926361f,0.7242711186408997f,0.5618708729743958f,1.748416781425476f,0.5537087917327881f,0.6352136731147766f,-0.35939329862594604f,-0.4866389334201813f,0.8658958673477173f,3.5233154296875f,-1.178995132446289f,-1.3896499872207642f,0.38831934332847595f,-0.2620525658130646f,2.1930925846099854f,0.3998042047023773f,0.8410940170288086f,-0.2980811893939972f,-3.3231122493743896f,1.8645620346069336f,1.465916395187378f,-1.105558156967163f,-1.9408748149871826f,-1.629994511604309f,1.4917867183685303f,1.0317301750183105f,-0.22827082872390747f,0.05468663573265076f,0.7816762328147888f,-0.5611151456832886f,-3.0220179557800293f,-0.03340088576078415f,0.835858941078186f,-0.6932273507118225f,-0.278925359249115f,-0.313508003950119f,-1.951085090637207f,-2.2981138229370117f,0.4034487307071686f,2.2342095375061035f,-2.6325838565826416f,1.6008665561676025f,-1.3050458431243896f,0.7810645699501038f,0.933142364025116f,-1.4400449991226196f,1.5623019933700562f,-0.6188889145851135f,1.2341901063919067f,-1.3063422441482544f,-1.972513198852539f,-0.2081049233675003f,-0.03563149645924568f,-1.0454567670822144f,-0.5792748332023621f,-0.8826676607131958f,-0.5708673596382141f,2.783168077468872f,-2.747321128845215f,-0.5282006859779358f,0.4784092307090759f,0.7694312334060669f,0.5235605239868164f,-3.1713836193084717f,0.6455166339874268f,-0.176595076918602f,1.4237704277038574f,0.34204792976379395f,0.9368235468864441f,1.929444432258606f,2.41141939163208f,1.1466692686080933f,-1.1158653497695923f,-0.46749430894851685f,-1.665422797203064f,0.6586175560951233f,0.10198018699884415f,2.0156917572021484f,-1.6875090599060059f,-1.7639665603637695f,-1.5508928298950195f,0.3582054078578949f,1.5886797904968262f,-0.6479190587997437f,-0.9712759852409363f,1.9645609855651855f,1.4678528308868408f,-0.6249018907546997f,-1.178263783454895f,-1.1139929294586182f,-0.8538286685943604f,2.795952081680298f,-2.180717945098877f,0.9967296719551086f,2.664621114730835f,-1.1815874576568604f,1.272865891456604f,-0.6897781491279602f,-0.5927006006240845f,-1.1634750366210938f,-0.49034383893013f,-0.7692076563835144f,4.004227161407471f,-0.39061373472213745f,-0.1930716335773468f,0.5229460000991821f,2.0044186115264893f,-0.44654712080955505f,-1.880897045135498f,0.6780076622962952f,-0.17819930613040924f,-2.100055456161499f,0.4006320834159851f,0.24330191314220428f,0.7435317635536194f,-0.0015948524232953787f,1.0799938440322876f,0.5820719599723816f,1.3486722707748413f,0.05239560827612877f,-0.7238573431968689f,0.6144594550132751f,1.8838860988616943f,-1.1129021644592285f,-0.5384948253631592f,-2.251466989517212f,-1.5316319465637207f,-0.17916788160800934f,0.5922936797142029f,-1.6129374504089355f,-0.3715687096118927f,-0.97847580909729f,-2.5053255558013916f,1.5747286081314087f,0.9949885010719299f,-0.09978730231523514f,0.7356669306755066f,-0.21073058247566223f,-1.2669161558151245f,1.8141266107559204f,-0.5998058319091797f,-1.4217910766601562f,-1.113110065460205f,0.4141691327095032f,0.46736887097358704f,1.2921181917190552f,2.0093324184417725f,-1.5114548206329346f,-1.4822980165481567f,0.2610069215297699f,1.1043790578842163f,-1.1969407796859741f,-0.6330389380455017f,-2.2134697437286377f,1.7599060535430908f,3.8246326446533203f,0.1910853236913681f,0.6171957850456238f,-0.04855010285973549f,0.6490797996520996f,1.663914442062378f,-0.4169577658176422f,-2.393181324005127f,-2.142101764678955f,2.6207633018493652f,1.3049745559692383f,-1.135115146636963f,-1.6757241487503052f,-2.5045721530914307f,0.6367810964584351f,-0.08912622183561325f,-0.44862210750579834f,1.6996756792068481f,-0.8383389711380005f,-0.37158772349357605f,-0.4028220772743225f,-0.1727905124425888f,-2.9641225337982178f,-0.485042929649353f,0.3929295241832733f,0.3405494689941406f,-0.9077396988868713f,0.675307035446167f,-0.19479715824127197f,-0.4344239830970764f,-1.4944673776626587f,-0.8451968431472778f,-1.7751668691635132f,-1.6247509717941284f,-1.1033263206481934f,-0.2127380222082138f,-0.6048151850700378f,2.696275472640991f,-2.4082956314086914f,-2.3155646324157715f,-0.1962611973285675f,0.12840349972248077f,-0.9851007461547852f,0.051678627729415894f,0.18184113502502441f,0.6385003924369812f,-0.2106492966413498f,1.5161187648773193f,0.5511079430580139f,2.8225789070129395f,0.2359369993209839f,0.7683385610580444f,1.9502837657928467f,0.5317015051841736f,2.8848483562469482f,1.5789495706558228f,0.21052847802639008f,0.21787291765213013f,1.6057671308517456f,2.4674336910247803f,-1.4877597093582153f,0.7713442444801331f,-1.727905035018921f,-0.6795412302017212f,-2.4924752712249756f,3.1733131408691406f,0.10553412139415741f,1.5492830276489258f,-0.27531978487968445f,-0.9645293951034546f,1.7961864471435547f,0.9205582141876221f,1.0734493732452393f,1.4687862396240234f,-1.9278064966201782f,2.4379971027374268f,-0.24706479907035828f,-1.5113041400909424f,2.3717458248138428f,1.5098031759262085f,0.9552524089813232f,0.12437909841537476f,-0.8495570421218872f,1.049027919769287f,0.048208389431238174f,-1.2756425142288208f,2.44960618019104f,-1.2028318643569946f,-1.4957355260849f,0.3369600176811218f,-0.8069557547569275f,0.39519283175468445f,2.2259507179260254f,0.6437201499938965f,0.5549702048301697f,-0.11257967352867126f,-0.9474810361862183f,0.3610284924507141f,0.9073130488395691f,-4.030120372772217f,-0.48519396781921387f,-1.4241682291030884f,-1.1005349159240723f,-0.3602388799190521f,0.7231995463371277f,-0.5329625606536865f,0.6090232133865356f,-3.240753650665283f,0.4667644202709198f,-1.6644654273986816f,1.62630295753479f,2.4657092094421387f,-0.21580860018730164f,1.49690842628479f,-2.0419723987579346f,-0.6245994567871094f,-0.13184064626693726f,-0.5005527138710022f,-0.14619497954845428f,-1.3356043100357056f,3.4054999351501465f,0.7276288866996765f,-0.20713385939598083f,1.4111247062683105f,0.1509818285703659f,1.5404150485992432f,0.33321261405944824f,-1.4776597023010254f,1.9250905513763428f,0.3551233112812042f,-1.1648688316345215f,-0.10586441308259964f,1.7072291374206543f,0.07436398416757584f,1.080717921257019f,0.9059544801712036f,0.1041731983423233f,-0.9659388065338135f,-1.1263606548309326f,-3.677828073501587f,0.49585071206092834f,-0.6708920001983643f,-0.5156315565109253f,-0.11740709096193314f,-0.6851766109466553f,0.5312384963035583f,-2.3301055431365967f,-1.3987460136413574f,-1.2913786172866821f,-0.4609125852584839f,-0.17972758412361145f,-0.8470062017440796f,0.7034717202186584f,0.4498211741447449f,-1.805484652519226f,1.0613999366760254f,-1.320820689201355f,-1.8943960666656494f,0.4887629747390747f,0.4854487180709839f,-0.9784176349639893f,3.257347345352173f,-0.8642997145652771f,1.0981138944625854f,-0.8931015729904175f,-0.015468680299818516f,2.6995954513549805f,-1.7845481634140015f,0.727201521396637f,2.6812734603881836f,0.856071949005127f,0.23525018990039825f,2.0267107486724854f,1.2264212369918823f,0.6437869668006897f,1.3214149475097656f,-2.6915903091430664f,0.12739142775535583f,0.0892586037516594f,1.0557917356491089f,0.44840726256370544f,0.05215439572930336f,0.5058522820472717f,-0.2846126854419708f,-0.505377471446991f,-1.631388545036316f,-0.8385325074195862f,0.8650213479995728f,0.391144335269928f,0.5855598449707031f,0.08371298760175705f,2.0636537075042725f,0.48169851303100586f,2.204252004623413f,3.6150598526000977f,-0.650900661945343f,0.3033115565776825f,-2.995008945465088f,2.3269834518432617f,-1.67079496383667f,0.24315932393074036f,-0.13677062094211578f,0.36071425676345825f,-0.5344938039779663f,3.4641661643981934f,1.2169928550720215f,1.4921258687973022f,0.9338715672492981f,1.6088831424713135f,-1.1164387464523315f,2.10233473777771f,1.0357776880264282f,0.9760457277297974f,-2.4743599891662598f,-0.9424089193344116f,0.23302054405212402f,1.1793051958084106f,0.8440702557563782f,-0.186442032456398f,0.7127044796943665f,-1.30267333984375f,-0.05602048709988594f,-0.682136595249176f,1.5605500936508179f,-0.3872312307357788f,-0.12172509729862213f,-2.432844638824463f,-0.462632954120636f,0.20636120438575745f,0.1859809309244156f,-0.25415685772895813f,0.31105396151542664f,0.14742273092269897f,0.23606443405151367f,-0.19308064877986908f,-0.3663501739501953f,0.6273019313812256f,-0.039221394807100296f,-0.2378363013267517f,2.3977203369140625f,0.1986083835363388f,0.8443253040313721f,-1.7754545211791992f,-0.20037966966629028f,-2.115159034729004f,-1.2295284271240234f,1.7357152700424194f,1.96908438205719f,-1.7847148180007935f,-0.4117918908596039f,-0.16387271881103516f,0.03436807543039322f,0.5867795944213867f,0.7155421376228333f,-0.31050679087638855f,-0.3039862811565399f,0.20118792355060577f,-0.2509442865848541f,-2.8166356086730957f,-0.40439656376838684f,-1.4316825866699219f,0.8630065321922302f,1.5773259401321411f,0.4345996379852295f,1.5695164203643799f,-1.1791205406188965f,-1.1407384872436523f,0.10300561785697937f,-2.3015027046203613f,-0.29183903336524963f,-1.3811780214309692f,-0.7714290022850037f,0.5950912833213806f,0.9229661822319031f,-0.6923258900642395f,0.5408746004104614f,-1.111387848854065f,0.5763923525810242f,-0.5839274525642395f,-1.2234166860580444f,-0.4885654151439667f,1.0020831823349f,-3.7001991271972656f,-1.288503885269165f,0.782535195350647f,1.1845632791519165f,0.29263415932655334f,1.7359987497329712f,-1.559367060661316f,2.8593087196350098f,-1.0913151502609253f,0.6381263136863708f,0.16040228307247162f,0.9585986137390137f,-0.06676288694143295f,-2.11214017868042f,1.9579788446426392f,-1.383852481842041f,-1.2697139978408813f,-0.7153846025466919f,1.1791294813156128f,1.3490852117538452f,-1.325934886932373f,-1.9843087196350098f,-1.2928935289382935f,0.5527184009552002f,-0.4718131422996521f,-2.346848964691162f,0.2176254838705063f,1.7737817764282227f,-0.7050303220748901f,1.4613385200500488f,0.8678267002105713f,-0.40856876969337463f,-1.9534918069839478f,0.32693949341773987f,0.8292192220687866f,2.111912488937378f,0.5171996355056763f,2.420649766921997f,-0.38305431604385376f,0.045997533947229385f,1.1037451028823853f,0.19894850254058838f,0.3816179037094116f,2.7304775714874268f,0.7631997466087341f,1.4142405986785889f,-0.4101788103580475f,0.7687193155288696f,1.4259330034255981f,0.1675514280796051f,-0.24114759266376495f,-0.41536951065063477f,1.3569368124008179f,-3.1404120922088623f,-1.4073749780654907f,1.3826173543930054f,0.21228651702404022f,-0.8414311408996582f,-0.20802372694015503f,0.7816598415374756f,-1.2795771360397339f,-2.425321578979492f,2.479032278060913f,1.4338330030441284f,-1.2370991706848145f,-0.3341042995452881f,-0.7532086372375488f,1.3653398752212524f,0.030683057382702827f,-2.131985664367676f,-0.5947455167770386f,-0.04340943694114685f,0.19426114857196808f,-1.2458447217941284f,-0.04437915235757828f,0.9428550601005554f,2.284749746322632f,-0.35181349515914917f,-0.6781859993934631f,-1.5905956029891968f,0.9029750227928162f,0.36106982827186584f,-2.2737185955047607f,0.7881835699081421f,2.299168348312378f,1.1134164333343506f,-0.6810310482978821f,0.7823745012283325f,1.6011658906936646f,-0.1812482476234436f,-2.4329097270965576f,0.17116299271583557f,0.2960328161716461f,1.5890588760375977f,3.636951446533203f,0.6352639198303223f,1.1664445400238037f,1.2854644060134888f,0.6915148496627808f,1.02070951461792f,-3.572178840637207f,-2.538374185562134f,-3.3212735652923584f,-0.49911338090896606f,-0.9106399416923523f,-0.07493748515844345f,-2.621046304702759f,-0.24266861379146576f,1.960976481437683f,-1.4238669872283936f,0.9397186636924744f,-0.8862112760543823f,1.5265002250671387f,0.553503692150116f,0.40409621596336365f,0.20315095782279968f,-0.8692059516906738f,0.15829181671142578f,2.235882043838501f,-0.3437938392162323f,-0.31461620330810547f,0.9198603630065918f,-0.16478972136974335f,1.6109025478363037f,0.841559112071991f,-0.31886544823646545f,1.4778368473052979f,-0.035975586622953415f,-2.2193892002105713f,-0.8406220078468323f,-0.13875043392181396f,-1.6160831451416016f,0.8189692497253418f,-0.47243401408195496f,-1.5038089752197266f,2.187520742416382f,0.35610082745552063f,0.5743789672851562f,-1.076192021369934f,1.466495156288147f,0.37620189785957336f,0.48030999302864075f,-0.473917156457901f,1.3633382320404053f,2.313138246536255f,0.8213364481925964f,-0.06420350074768066f,0.4283921718597412f,-0.2856929898262024f,-0.7897531390190125f,-0.11833912879228592f,-0.06211486831307411f,1.2468183040618896f,-1.088843822479248f,-0.9127181172370911f,-1.347821831703186f,1.1794323921203613f,0.9624733328819275f,-0.9818911552429199f,0.7265936732292175f,-0.07840538024902344f,-0.6242746114730835f,0.4227491319179535f,-0.5083812475204468f,0.4992070496082306f,-1.4669914245605469f,0.2562127411365509f,0.3621523380279541f,-0.33273857831954956f,1.0905954837799072f,-0.8279059529304504f,-0.14801724255084991f,1.1259303092956543f,-0.3436514735221863f,0.3471996486186981f,-0.9708760380744934f,-0.6147895455360413f,0.8744673728942871f,1.3475935459136963f,-0.027413075789809227f,-0.4827187657356262f,-2.316016674041748f,1.2227107286453247f,-1.8424626588821411f,0.8992468118667603f,1.4362353086471558f,0.037287984043359756f,-1.2788647413253784f,-1.036995768547058f,-0.4104834794998169f,-0.2878062129020691f,-1.131345272064209f,-0.6888310313224792f,-0.5416653752326965f,-1.2332689762115479f,0.9142865538597107f,1.6532021760940552f,0.7005264163017273f,-0.5247263312339783f,-0.37360596656799316f,-3.2714083194732666f,-2.6247787475585938f,0.8683927655220032f,1.0747730731964111f,-0.7419224977493286f,-2.586540460586548f,-1.409127950668335f,-0.29882481694221497f,0.9448767304420471f,0.2937870919704437f,2.7959117889404297f,2.06595778465271f,1.0236117839813232f,-0.811748743057251f,0.582487165927887f,-0.9266070127487183f,1.2984797954559326f,-1.2375409603118896f,1.2089860439300537f,-0.09932804852724075f,2.0760700702667236f,0.18397273123264313f,-1.0344966650009155f,0.437375545501709f,0.19798187911510468f,0.158390074968338f,1.739998698234558f,-0.030142154544591904f,0.5370904207229614f,0.841293215751648f,-0.5422562956809998f,1.4824535846710205f,0.27918902039527893f,0.249969482421875f,0.5622153878211975f,-1.4114288091659546f,-0.33458244800567627f,0.8891441822052002f,-1.8042629957199097f,-1.0457167625427246f,1.826540231704712f,2.2904257774353027f,2.9540886878967285f,1.828476905822754f,-0.10378967970609665f,1.2533526420593262f,0.13194020092487335f,0.039536673575639725f,-1.6753287315368652f,-0.13641725480556488f,-0.6962326169013977f,-2.0844547748565674f,-1.2238126993179321f,0.44826024770736694f,-0.1108192652463913f,0.2876344323158264f,0.7227776050567627f,2.3124616146087646f,0.6379067897796631f,-0.5989519953727722f,-0.5126609206199646f,-0.11339502781629562f,-1.6438223123550415f,0.9452461004257202f,-1.1392589807510376f,-1.1117422580718994f,0.45032191276550293f,0.19088228046894073f,-0.8974567651748657f,-1.6016628742218018f,-0.9230382442474365f,-0.22620435059070587f,-3.1328461170196533f,-0.23491980135440826f,-0.07598021626472473f,-2.5944175720214844f,1.6517460346221924f,0.23580078780651093f,-2.2028589248657227f,0.7663795351982117f,0.6315111517906189f,-2.3798818588256836f,-2.081641435623169f,0.7445374727249146f,0.8482441902160645f,0.8716559410095215f,-1.330425500869751f,0.23595890402793884f,1.063382863998413f,0.0040075830183923244f,-2.076465129852295f,-1.3066325187683105f,2.428819417953491f,-0.025780199095606804f,0.06902444362640381f,-0.6162111759185791f,1.2296355962753296f,0.30954277515411377f,-0.937627911567688f,-0.6450285315513611f,-0.39270511269569397f,0.5596690773963928f,-0.5293797254562378f,0.2375577688217163f,2.4242489337921143f,-2.0740365982055664f,-1.2475652694702148f,-0.562133252620697f,-0.46761563420295715f,-0.7138792276382446f,-0.41846659779548645f,-0.15989534556865692f,2.1495938301086426f,-1.0380158424377441f,-0.9929221272468567f,0.7366330027580261f,-0.06334766000509262f,-0.7263140082359314f,0.3544861674308777f,-0.6419603824615479f,-0.7462416291236877f,-0.692952573299408f,2.219841718673706f,2.395702362060547f,0.9065694212913513f,0.965340256690979f,-1.316758632659912f,-0.8236132264137268f,3.1627612113952637f,-0.29531529545783997f,0.732248842716217f,-1.1302800178527832f,-0.1502886712551117f,-1.4145199060440063f,-2.5394210815429688f,0.06469234079122543f,0.6838144063949585f,1.211388111114502f,2.1619038581848145f,1.5025825500488281f,0.7120748162269592f,0.10252314060926437f,2.1474480628967285f,-0.6120508313179016f,-1.176591396331787f,-1.9356715679168701f,0.09931488335132599f,-0.8364925980567932f,0.6217472553253174f,1.0517938137054443f,0.9304336309432983f,1.960854172706604f,2.0769097805023193f,-3.699936866760254f,-0.05687114968895912f,1.3050206899642944f,-0.14706780016422272f,-0.4355623424053192f,0.0283679086714983f,1.4077950716018677f,-1.1079425811767578f,0.5139343738555908f,0.6284661293029785f,-0.8244279026985168f,0.10316600650548935f,0.3447498381137848f,-1.3278025388717651f,0.26845890283584595f,0.49806854128837585f,-0.9361953139305115f,0.3969278335571289f,-2.0728206634521484f,2.2726972103118896f,0.930428683757782f,-0.10555025190114975f,0.5621238946914673f,0.8805652260780334f,-1.5860841274261475f,-0.04590244218707085f,-0.05936773121356964f,0.5021916031837463f,2.8192837238311768f,-0.7028370499610901f,-1.1082016229629517f,-0.46358931064605713f,-1.1738741397857666f,0.2360004484653473f,2.8537309169769287f,1.4797133207321167f,1.4693351984024048f,0.21734566986560822f,0.2931555509567261f,-1.029098629951477f,-0.8173521161079407f,-0.3517835736274719f,1.970054030418396f,-1.726035237312317f,0.025539854541420937f,0.4536212980747223f,1.412269115447998f,0.7399903535842896f,-0.8661982417106628f,0.5232188701629639f,-0.46200379729270935f,1.5442906618118286f,1.0759050846099854f,-2.8613548278808594f,1.8841520547866821f,0.5834178328514099f,0.4742843806743622f,2.9991722106933594f,-1.3250423669815063f,-0.28334861993789673f,-0.025784967467188835f,0.30944544076919556f,0.6485716700553894f,-1.0524122714996338f,-1.15669584274292f,-0.33564215898513794f,1.3918653726577759f,3.5271847248077393f,-0.33687397837638855f,0.7780298590660095f,-1.2216919660568237f,-0.26853159070014954f,-1.1437504291534424f,0.44153204560279846f,0.007487786468118429f,0.9911266565322876f,-2.0439908504486084f,1.2984168529510498f,0.35385826230049133f,0.7187745571136475f,-0.10692622512578964f,-0.6671226024627686f,0.4263223707675934f,-1.6204843521118164f,0.8871776461601257f,-2.072800397872925f,1.2941527366638184f,1.4323302507400513f,-0.2756531536579132f,0.45941323041915894f,-0.5367972254753113f,1.1920888423919678f,3.6240742206573486f,-1.850576400756836f,4.826727308682166e-05f,-0.06491218507289886f,0.61821049451828f,-0.9282599091529846f,-1.8294497728347778f,-1.0904302597045898f,-1.67671537399292f,1.6469533443450928f,0.27910029888153076f,-1.1639991998672485f,-0.7647432684898376f,1.1984384059906006f,-1.2324086427688599f,-0.28316211700439453f,-0.07624132931232452f,1.7789363861083984f,-1.599911093711853f,0.856467604637146f,2.6159420013427734f,-2.0322816371917725f,-0.44716691970825195f,0.06582346558570862f,-1.0714439153671265f,-0.8638128638267517f,-1.785490870475769f,-2.040379762649536f,2.8375329971313477f,-2.0537500381469727f,-1.0685187578201294f,0.8078057765960693f,1.8808090686798096f,0.22887732088565826f,-1.1108975410461426f,-1.2372419834136963f,0.22406256198883057f,0.24106061458587646f,-0.11961162835359573f,2.0787787437438965f,-0.11769498139619827f,0.24651168286800385f,-0.07095254212617874f,-0.8719712495803833f,0.8073496222496033f,0.0035199951380491257f,-1.5808277130126953f,-1.1262116432189941f,-0.546570360660553f,-0.7580028772354126f,-1.021552562713623f,0.003236937802284956f,0.05144113302230835f,0.0842643454670906f,-0.7436034083366394f,0.3795759379863739f,-1.3621443510055542f,-0.5653916597366333f,-1.5064064264297485f,-1.1603291034698486f,0.7259700298309326f,2.0522661209106445f,3.602391242980957f,0.947396993637085f,1.1860369443893433f,0.6211631894111633f,1.7127561569213867f,-0.9140222072601318f,1.9346935749053955f,0.12976062297821045f,-1.7275739908218384f,-0.31862887740135193f,-0.4866708219051361f,-0.04892770200967789f,0.30803433060646057f,0.4662710130214691f,-0.1433287262916565f,-0.4546881318092346f,0.7887839674949646f,-0.5117461085319519f,-0.6007419228553772f,0.22030384838581085f,1.2524775266647339f,-0.9620169401168823f,-0.5028437972068787f,-0.011730145663022995f,-2.504476308822632f,1.4839967489242554f,1.6039561033248901f,0.26654666662216187f,-1.3830375671386719f,-1.6890345811843872f,-1.6853286027908325f,1.0553003549575806f,-0.4051113724708557f,1.0961527824401855f,-2.442016839981079f,0.8042796850204468f,-0.11801190674304962f,1.8067615032196045f,-1.1283848285675049f,-0.9474571347236633f,-1.4777085781097412f,-0.455171674489975f,2.565114974975586f,0.06757998466491699f,-2.631965160369873f,1.3831182718276978f,0.9672272801399231f,-1.176276445388794f,0.3197500705718994f,-2.7778990268707275f,-2.902207612991333f,-0.31339335441589355f,0.6017233729362488f,0.49385619163513184f,0.7256348133087158f,1.2148802280426025f,1.468837857246399f,2.2338149547576904f,-0.24794349074363708f,-0.4644036889076233f,0.47446438670158386f,0.8587716817855835f,-0.6256842017173767f,0.5384566187858582f,0.7726974487304688f,0.9210312962532043f,-2.4386518001556396f,-0.6015543341636658f,-0.9663498401641846f,-1.2096558809280396f,-3.3884830474853516f,-0.8109520673751831f,1.4029474258422852f,0.29784247279167175f,-0.3391871750354767f,1.1727235317230225f,-1.546400547027588f,-0.7178347706794739f,1.1631169319152832f,1.2969857454299927f,0.6497972011566162f,1.4658279418945312f,0.07387612760066986f,1.0040515661239624f,-2.553020715713501f,-4.0172343254089355f,-1.0112358331680298f,-0.691952109336853f,2.158108949661255f,1.6771183013916016f,1.9638679027557373f,-0.19424696266651154f,0.535986065864563f,1.7481611967086792f,1.05055570602417f,-0.9454368948936462f,-0.3224552869796753f,-0.598038911819458f,1.1413911581039429f,0.4288150668144226f,0.778110921382904f,0.054243482649326324f,-0.18214695155620575f,1.5678446292877197f,-0.16082929074764252f,-2.1629016399383545f,0.743661105632782f,3.985943555831909f,1.8105485439300537f,-0.5347939133644104f,0.20120321214199066f,1.2537516355514526f,-0.8345087766647339f,-0.029392564669251442f,0.5120130777359009f,0.8890120387077332f,-1.7491568326950073f,1.60444176197052f,-2.1299455165863037f,-1.5661793947219849f,-1.4135593175888062f,1.214066982269287f,0.10605897754430771f,0.8703531622886658f,-1.3488022089004517f,-0.7751810550689697f,-1.443610429763794f,0.3456823527812958f,0.9282270669937134f,1.9438718557357788f,1.0099999904632568f,-0.13643953204154968f,-2.807826519012451f,-1.5181760787963867f,-1.0992002487182617f,-1.1767979860305786f,0.608227550983429f,0.7551780939102173f,-0.790959894657135f,0.3935863971710205f,-1.736196517944336f,0.33647817373275757f,0.27479833364486694f,-0.7316173911094666f,0.7379469275474548f,-1.5065463781356812f,3.1526248455047607f,2.3786303997039795f,-2.081467866897583f,-0.6018892526626587f,0.33246326446533203f,-1.7549694776535034f,-0.8868935704231262f,0.5341901183128357f,-1.7919297218322754f,0.5830273032188416f,0.4485706090927124f,2.1539247035980225f,1.4308762550354004f,-0.5586742162704468f,1.3415817022323608f,0.3136586546897888f,1.2065296173095703f,-0.5249568223953247f,-0.07938636094331741f,-0.7779722809791565f,0.08603142201900482f,-1.5313323736190796f,0.2634788155555725f,-1.4271831512451172f,0.6709486246109009f,0.13084255158901215f,0.9797662496566772f,0.8256807327270508f,-0.016838625073432922f,-0.6194220185279846f,0.6840758323669434f,-2.6896743774414062f,0.014496425166726112f,-0.9368176460266113f,0.37177374958992004f,0.08156288415193558f,-1.7412645816802979f,0.06432108581066132f,-2.3161158561706543f,-0.9149886965751648f,-0.9012859463691711f,1.6689205169677734f,-1.1829488277435303f,1.4373279809951782f,-0.5295313596725464f,-0.7553181052207947f,1.2416882514953613f,2.003572463989258f,2.029110908508301f,2.196242332458496f,-1.8517619371414185f,1.5958510637283325f,0.5870757102966309f,2.2738664150238037f,-0.3739442527294159f,0.7484645843505859f,-0.42295700311660767f,-1.3969173431396484f,0.4687383770942688f,-0.9852034449577332f,1.0749883651733398f,-0.840084433555603f,-0.7514966726303101f,0.005737611558288336f,-0.5683007836341858f,0.15369336307048798f,0.044207461178302765f,1.1647870540618896f,1.3198225498199463f,0.38207772374153137f,0.737723708152771f,0.7739190459251404f,0.021428076550364494f,-1.181348204612732f,-1.4300353527069092f,-1.0594841241836548f,-1.4167211055755615f,-1.9380779266357422f,2.206441640853882f,-0.992128312587738f,-0.6739637851715088f,1.4734611511230469f,0.5205383896827698f,0.39553335309028625f,2.482922077178955f,2.549018144607544f,0.3192918300628662f,-0.8002685308456421f,-2.0757040977478027f,0.8321025967597961f,0.08034957200288773f,-0.5796035528182983f,0.5448570847511292f,-0.7324485182762146f,-0.8187222480773926f,-0.6173725128173828f,0.9773167371749878f,0.7150527238845825f,1.1566541194915771f,-0.7869284749031067f,-0.473447322845459f,-0.46752169728279114f,-0.28387874364852905f,-1.0326461791992188f,-0.3816928267478943f,0.9332020878791809f,-0.5729718208312988f,-0.7626715898513794f,0.4801725447177887f,1.3228299617767334f,-0.5556281208992004f,0.07883331179618835f,0.3657396137714386f,2.134197950363159f,-2.124645471572876f,1.138598918914795f,0.1586138755083084f,-2.101083755493164f,1.5048383474349976f,-1.9689218997955322f,-2.01414155960083f,0.8041145205497742f,2.065559148788452f,0.8261798620223999f,-0.976762056350708f,-0.08098234236240387f,1.0319629907608032f,0.031261250376701355f,-1.6175731420516968f,-1.1113685369491577f,-0.7817158102989197f,2.1236612796783447f,0.8841006755828857f,0.5331234931945801f,-0.6735841631889343f,0.9242456555366516f,-0.16158035397529602f,0.06237011030316353f,2.6160926818847656f,-0.32881078124046326f,-1.6796265840530396f,2.222259044647217f,1.2404074668884277f,-1.793717384338379f,0.6036821603775024f,-1.496897578239441f,-0.5928653478622437f,-0.1287601888179779f,-0.7442003488540649f,-0.1389160454273224f,-0.411455899477005f,0.6055302023887634f,-1.8976943492889404f,0.7660620808601379f,0.12517715990543365f,1.412169337272644f,-0.18901734054088593f,-0.3710211515426636f,-0.36054447293281555f,-0.2783922255039215f,1.2445379495620728f,-0.9066398739814758f,1.338468313217163f,0.9481795430183411f,-0.4209407866001129f,1.5649816989898682f,0.5397570729255676f,-0.39615312218666077f,-0.4163653552532196f,-1.411528468132019f,3.0437686443328857f,0.19502763450145721f,0.14976347982883453f,0.53302401304245f,-0.039437610656023026f,-1.998169183731079f,1.2039506435394287f,-0.7007102370262146f,-3.4607207775115967f,0.39310893416404724f,1.209704875946045f,-0.738774299621582f,-0.2597709894180298f,0.7651737332344055f,0.8376129865646362f,1.6329303979873657f,-0.707012951374054f,-0.8928752541542053f,-1.2061141729354858f,-0.5295154452323914f,-1.1337631940841675f,-1.5770946741104126f,-1.6880215406417847f,-0.052283838391304016f,0.03822212293744087f,-0.9795040488243103f,-1.915262222290039f,0.7158104181289673f,0.7302238941192627f,-1.0619566440582275f,0.8271670937538147f,0.6826069951057434f,1.9559152126312256f,-0.3509191572666168f,-0.8526828289031982f,0.048790447413921356f,-0.8890692591667175f,1.412391185760498f,-0.23403319716453552f,-0.8019153475761414f,-1.7699296474456787f,-0.2684345245361328f,-1.544363021850586f,-0.06202182546257973f,-2.1677677631378174f,-0.17888425290584564f,0.29630905389785767f,0.8604685068130493f,1.2711353302001953f,2.4922778606414795f,0.8093973994255066f,-1.253433346748352f,-0.32443076372146606f,-0.029384354129433632f,-2.097907304763794f,3.707117795944214f,1.0987459421157837f,-2.3891334533691406f,-3.1337804794311523f,0.8667729496955872f,2.3230438232421875f,-0.02244442328810692f,0.8546546697616577f,-0.5832293629646301f,0.864029586315155f,-1.0820389986038208f,-1.4252712726593018f,0.1874798834323883f,0.46228930354118347f,1.3953408002853394f,-0.5832997560501099f,-1.3193950653076172f,0.010047073476016521f,0.26079005002975464f,-2.0061118602752686f,-0.4128429889678955f,0.9256482720375061f,-1.512691617012024f,-1.72117280960083f,-0.47976240515708923f,0.6193070411682129f,-0.3882644772529602f,2.792499542236328f,0.5485286712646484f,0.938592255115509f,0.1652715802192688f,0.040005289018154144f,0.33677998185157776f,-2.3931190967559814f,-0.09825563430786133f,0.5553252100944519f,-0.31924885511398315f,0.5643541812896729f,-1.1304607391357422f,1.52943754196167f,0.35225799679756165f,0.8461647033691406f,-2.150668144226074f,-0.8618703484535217f,1.5061713457107544f,-0.3730612099170685f,-1.8034801483154297f,-0.862841784954071f,-1.5338081121444702f,1.9073066711425781f,1.5723589658737183f,0.36930978298187256f,0.07460945099592209f,0.5694260001182556f,-2.147603750228882f,-1.1874504089355469f,-1.1980006694793701f,-0.26806917786598206f,1.3254956007003784f,0.4846498668193817f,-0.3869568705558777f,0.7356695532798767f,1.614227056503296f,0.547521710395813f,-2.9457194805145264f,0.9180450439453125f,-0.06933744251728058f,3.344294309616089f,0.7579075694084167f,1.5495078563690186f,0.027504751458764076f,-0.8079804182052612f,-0.5504769086837769f,1.1138437986373901f,0.9619492292404175f,-0.6138226389884949f,1.6601454019546509f,0.26596248149871826f,0.5801243782043457f,2.7087483406066895f,-1.6541695594787598f,-2.431136131286621f,-3.0835931301116943f,0.03653092309832573f,0.48491171002388f,0.4306263327598572f,0.4343881905078888f,0.04155135527253151f,1.3562015295028687f,-1.347458004951477f,0.24392619729042053f,0.32742932438850403f,-0.9768484234809875f,2.2708332538604736f,-1.1883306503295898f,0.24132373929023743f,0.25575506687164307f,0.8412376046180725f,0.9293384552001953f,2.526681423187256f,2.4018442630767822f,0.7777249813079834f,0.25700125098228455f,-1.4442715644836426f,0.04796353355050087f,-0.3596566319465637f,-0.34292763471603394f,0.09133966267108917f,-0.6866466999053955f,0.9304687976837158f,0.7075328230857849f,1.1282305717468262f,0.3826764225959778f,-1.9986027479171753f,1.4736425876617432f,-0.002622363157570362f,1.9426747560501099f,-0.6602279543876648f,-1.9231592416763306f,0.2608144283294678f,2.83268141746521f,-0.9670118093490601f,0.41436415910720825f,-0.4692610800266266f,-1.3884943723678589f,-0.4737849533557892f,-1.2706596851348877f,1.3616019487380981f,1.677936315536499f,0.6525779962539673f,-0.635306715965271f,1.8232345581054688f,-1.0990006923675537f,-0.4856162369251251f,-0.5811066627502441f,1.937056303024292f,-0.302300363779068f,0.4832129180431366f,1.5493263006210327f,-0.36656737327575684f,-2.222961902618408f,0.35489732027053833f,0.5576257109642029f,-1.1228506565093994f,-1.4053592681884766f,-0.19995979964733124f,-2.3972103595733643f,-0.6588816046714783f,0.15966670215129852f,0.6922832131385803f,-0.7674115300178528f,1.3328204154968262f,0.5213620662689209f,-0.09292569756507874f,-1.631592869758606f,0.48966410756111145f,0.5902044177055359f,0.2636670172214508f,-0.16251994669437408f,-2.0837528705596924f,0.13999761641025543f,1.4268929958343506f,1.2276241779327393f,1.4407100677490234f,-2.0060110092163086f,-0.8260054588317871f,0.3830951750278473f,-3.4445531368255615f,0.6034727692604065f,-0.19017811119556427f,-0.7007163166999817f,0.3799077868461609f,-0.8793696165084839f,-0.036488715559244156f,-0.436345636844635f,1.480672836303711f,1.108066201210022f,0.09597186744213104f,1.417812705039978f,2.078991651535034f,-0.192658469080925f,-0.12604056298732758f,1.1467525959014893f,-0.5203048586845398f,0.08432541787624359f,0.41079553961753845f,-0.8310220241546631f,1.3117259740829468f,-2.7157537937164307f,0.6729490160942078f,0.11510632932186127f,-2.5611279010772705f,-0.32453590631484985f,0.20383884012699127f,1.8388829231262207f,0.2945817708969116f,-0.16436654329299927f,0.01453148853033781f,1.2946206331253052f,-1.4862412214279175f,-0.3221687078475952f,0.7731771469116211f,-0.7100803256034851f,-0.9351962804794312f,-1.002205729484558f,-0.07406001538038254f,0.916131854057312f,-0.021658044308423996f,2.7248213291168213f,1.051680088043213f,-0.5411649346351624f,-0.6179669499397278f,1.5811693668365479f,0.3056904375553131f,-1.9070243835449219f,0.8074924945831299f,0.9228144288063049f,1.4883129596710205f,0.42257484793663025f,-0.6350616812705994f,0.3757561147212982f,-1.1527262926101685f,-1.7994519472122192f,0.627510130405426f,-0.4822330176830292f,0.05379209294915199f,-0.4928484261035919f,-3.1962268352508545f,-0.7945910692214966f,1.4976434707641602f,-2.461737632751465f,0.816801905632019f,0.6909804940223694f,-0.2235841155052185f,1.1962544918060303f,-1.1382861137390137f,-1.1714274883270264f,1.6852781772613525f,1.491031527519226f,0.848511278629303f,-0.6387881636619568f,-0.8115203976631165f,0.6841213703155518f,1.326933741569519f,-0.7005462050437927f,-0.8349214792251587f,0.4596758484840393f,-0.9568848013877869f,1.3109718561172485f,1.5263762474060059f,-0.4248138964176178f,1.2645879983901978f,0.1355753242969513f,0.05962640419602394f,0.4281875789165497f,-1.0629371404647827f,-2.1632587909698486f,-0.7552924156188965f,-0.2821667790412903f,0.19496388733386993f,-1.3971017599105835f,-2.029813528060913f,-0.4109722077846527f,-0.3957632780075073f,-0.049734652042388916f,1.3163111209869385f,2.019676446914673f,-1.8228496313095093f,-0.33704468607902527f,-3.611797571182251f,1.0089373588562012f,0.4855029881000519f,-1.3248165845870972f,0.3416573405265808f,3.6855995655059814f,-0.38860079646110535f,-1.6340587139129639f,1.7527422904968262f,0.06884028762578964f,0.30410847067832947f,-3.061812400817871f,2.014169692993164f,0.6599101424217224f,-2.2883288860321045f,0.25147294998168945f,-0.7479835748672485f,0.5263285040855408f,-1.7113173007965088f,0.10430777072906494f,0.9604803919792175f,1.4939888715744019f,0.8829688429832458f,-0.9647164344787598f,-0.25886157155036926f,1.342548131942749f,1.7613167762756348f,-0.2687091827392578f,0.8742779493331909f,0.7950122356414795f,1.9682822227478027f,-0.4097060561180115f,3.391298294067383f,-1.4922491312026978f,0.5115921497344971f,-0.4104025363922119f,-0.1451367288827896f,1.4455032348632812f,1.110144853591919f,0.6452412605285645f,2.042088508605957f,0.12937597930431366f,-1.797791600227356f,1.1742196083068848f,0.20856131613254547f,1.4432278871536255f,0.5737220644950867f,-0.7430907487869263f,0.7588465213775635f,-1.3878332376480103f,-1.354861855506897f,0.14106667041778564f,-2.233414888381958f,0.17307215929031372f,-0.1141023337841034f,1.6459769010543823f,-0.03880377486348152f,-1.01809561252594f,1.1961028575897217f,1.3080976009368896f,1.7048667669296265f,1.6429635286331177f,-0.0965733677148819f,-1.3017240762710571f,0.02725234627723694f,-0.33464235067367554f,-2.1121561527252197f,-0.9640832543373108f,1.5839546918869019f,1.7124972343444824f,2.0523505210876465f,0.19618278741836548f,0.6701011061668396f,0.20657800137996674f,0.7172114849090576f,0.1008712574839592f,-2.4088993072509766f,0.27590394020080566f,1.2513307332992554f,-0.25416699051856995f,-2.1856329441070557f,-1.8070223331451416f,-0.5199803113937378f,1.2163355350494385f,-0.22021056711673737f,0.3582645654678345f,0.06984101980924606f,1.094200849533081f,-0.5312546491622925f,0.41352519392967224f,-3.3277456760406494f,-1.125860571861267f,0.3538070023059845f,2.411101818084717f,-0.8485537171363831f,-0.3094191551208496f,0.053264468908309937f,1.1137629747390747f,1.7532596588134766f,1.536186695098877f,-0.7702553272247314f,0.3115822672843933f,-2.2230679988861084f,-0.16197283565998077f,-1.8590967655181885f,-1.0843067169189453f,-0.3689402937889099f,0.3305700123310089f,0.7554129362106323f,0.7317404747009277f,2.077772617340088f,-1.7685078382492065f,-0.598054826259613f,-1.7624428272247314f,0.03566787391901016f,0.28482183814048767f,1.2842204570770264f,-0.892346978187561f,-1.4967221021652222f,-0.5748531222343445f,-0.5664356350898743f,-2.5559253692626953f,0.04361217841506004f,0.3664633631706238f,1.9950246810913086f,-0.14396342635154724f,1.1840680837631226f,-2.272684097290039f,0.32565778493881226f,-1.1070772409439087f,1.022294044494629f,0.6654731035232544f,0.0396634079515934f,-1.698358178138733f,0.03974942862987518f,-0.5353782773017883f,0.9074622392654419f,-0.3483501076698303f,-1.8284101486206055f,0.46614769101142883f,-0.4624548554420471f,-0.35393786430358887f,-0.14430412650108337f,-0.20848649740219116f,1.922969937324524f,-2.159404754638672f,0.32437288761138916f,-0.8804805874824524f,0.4785494804382324f,1.4765238761901855f,-0.3811607360839844f,-0.39076605439186096f,1.463389277458191f,0.06478315591812134f,0.6352197527885437f,2.38149356842041f,0.7912635207176208f,1.0410405397415161f,-1.4695274829864502f,-0.9861952662467957f,2.6098270416259766f,1.748329758644104f,-3.2988827228546143f,-1.2243566513061523f,1.2174392938613892f,-0.8387000560760498f,-0.28772106766700745f,-1.0955181121826172f,-2.5993800163269043f,-2.3214006423950195f,0.749771773815155f,-0.6855919361114502f,0.8990245461463928f,1.0089147090911865f,-1.9951186180114746f,1.9632558822631836f,0.07430505752563477f,-0.6501110196113586f,1.6328566074371338f,-0.12160812318325043f,-0.18078427016735077f,0.7516845464706421f,0.7268210053443909f,-0.7042796015739441f,-1.597645878791809f,2.269071578979492f,1.2728006839752197f,0.30056923627853394f,-1.7000408172607422f,1.0392813682556152f,-2.395418643951416f,-0.20709188282489777f,0.45235323905944824f,1.1439487934112549f,-0.7962487936019897f,-0.42735999822616577f,0.5795366168022156f,-1.080965280532837f,0.018864039331674576f,-0.16042770445346832f,1.0227887630462646f,-0.009136267006397247f,-1.5552817583084106f,-0.05314100161194801f,0.10971952229738235f,3.239389419555664f,-1.07399320602417f,0.145951047539711f,0.020650416612625122f,1.1214635372161865f,-0.22940902411937714f,-0.4252494275569916f,-0.1667465716600418f,-0.6254997849464417f,-0.7414543628692627f,0.08973410725593567f,-1.8906874656677246f,-1.1413705348968506f,1.7629481554031372f,-0.10440926253795624f,0.4538145661354065f,-2.972548484802246f,2.0168843269348145f,-1.773961067199707f,-0.45259013772010803f,-0.8117359280586243f,-1.2427140474319458f,-0.865013599395752f,0.33303460478782654f,-0.8273495435714722f,-1.3432167768478394f,0.9720630049705505f,0.282889187335968f,-0.5550892949104309f,-1.7060540914535522f,2.4938790798187256f,-0.47115394473075867f,-1.2224512100219727f,2.8452680110931396f,1.1337212324142456f,1.7524973154067993f,-1.9857555627822876f,-0.7492623925209045f,1.779400110244751f,0.37399283051490784f,0.9263566732406616f,-0.18502222001552582f,0.6078020930290222f,0.6822755932807922f,2.741807699203491f,1.4139008522033691f,-0.4234389364719391f,0.7032010555267334f,0.24181881546974182f,-0.7641986012458801f,-2.8788721561431885f,-0.7207019329071045f,-0.7623312473297119f,-1.3763447999954224f,2.079594373703003f,-1.6516813039779663f,-0.28175899386405945f,3.0951898097991943f,-0.2315029501914978f,-1.8446874618530273f,0.3504844903945923f,0.4431678354740143f,1.0384286642074585f,0.7345515489578247f,-1.6008905172348022f,2.640817880630493f,-0.5592171549797058f,0.27622830867767334f,-1.0898686647415161f,0.7896619439125061f,-0.6900898218154907f,0.17913521826267242f,-0.4728565514087677f,1.7296485900878906f,-1.24086594581604f,0.16073694825172424f,-1.2190775871276855f,1.6653258800506592f,-0.21737085282802582f,0.9269912242889404f,2.5091567039489746f,-0.586818516254425f,0.35623225569725037f,-0.7146664261817932f,-0.8210869431495667f,-0.9450688362121582f,1.5671207904815674f,-1.3473049402236938f,-0.06509687006473541f,0.5889977216720581f,0.3889349699020386f,-1.2628587484359741f,-0.4482460916042328f,-2.2179012298583984f,-0.1822832077741623f,0.07314366847276688f,-1.254369854927063f,0.35873135924339294f,1.3452898263931274f,0.16070975363254547f,-0.719952404499054f,0.1923927217721939f,1.5120298862457275f,-1.8287153244018555f,-0.4459390342235565f,-0.5135272741317749f,0.7235718369483948f,1.0121443271636963f,-1.236098051071167f,1.5233792066574097f,1.3356796503067017f,-0.4257904291152954f,2.092862367630005f,2.053394317626953f,1.5035604238510132f,1.0623526573181152f,2.4204490184783936f,-1.3172398805618286f,0.8031095266342163f,-2.852837085723877f,-1.465437650680542f,1.1735774278640747f,0.32155898213386536f,-0.7766427993774414f,-0.9759544730186462f,0.4245295524597168f,-1.0467305183410645f,-0.8317660093307495f,-1.239750623703003f,-0.09092089533805847f,-0.9908668994903564f,1.9451984167099f,0.13040651381015778f,3.353395700454712f,-0.8529180288314819f,1.1632843017578125f,-0.6791913509368896f,1.47727632522583f,0.7949761152267456f,0.6847853064537048f,-0.3334057331085205f,0.4590572714805603f,1.856117606163025f,-1.3257555961608887f,-0.6252825260162354f,-1.2535977363586426f,0.11964421719312668f,-0.49482104182243347f,-0.9166067838668823f,1.0101368427276611f,1.2671658992767334f,1.531758189201355f,-0.5184155702590942f,0.42092353105545044f,0.6076880693435669f,-1.5135077238082886f,-1.5877255201339722f,1.339834451675415f,-0.036353327333927155f,-0.43044888973236084f,-0.5703428387641907f,-0.3252864181995392f,0.1276664286851883f,0.5247217416763306f,0.6925730109214783f,-1.114270806312561f,-2.5706353187561035f,-0.2848176658153534f,-0.3845808804035187f,-0.3231682777404785f,-0.17686942219734192f,-0.30544355511665344f,-1.980782151222229f,0.2365058958530426f,-0.7050562500953674f,-0.8497467637062073f,-1.0259387493133545f,-0.7708958983421326f,0.049190178513526917f,-0.6842179894447327f,-1.313005805015564f,-2.471853017807007f,1.2500358819961548f,-0.7736682295799255f,-0.2898094952106476f,-2.0058491230010986f,2.4727022647857666f,-0.07931248843669891f,2.03383469581604f,-1.444225788116455f,-0.253136545419693f,0.3289845585823059f,0.05770653113722801f,-0.4980907142162323f,-1.8934292793273926f,0.5868033766746521f,0.16759082674980164f,0.6419074535369873f,0.6792100667953491f,0.28677085041999817f,-0.6359307765960693f,-1.08527672290802f,0.7839562892913818f,1.1338365077972412f,-0.4501141607761383f,-0.06857520341873169f,0.47395119071006775f,1.948498010635376f,0.7850701808929443f,0.07041646540164948f,0.8407975435256958f,0.679695188999176f,2.944704294204712f,1.8124970197677612f,1.4735453128814697f,-3.337334632873535f,-2.7236146926879883f,0.4249354600906372f,0.38256320357322693f,1.1575651168823242f,-1.1266621351242065f,-2.2346389293670654f,0.8039405345916748f,-0.9908658862113953f,-0.2966100573539734f,2.0281240940093994f,-1.7347166538238525f,-1.0585864782333374f,-1.3274405002593994f,-0.2844184637069702f,1.3544522523880005f,-0.8739485144615173f,1.3045943975448608f,1.6222645044326782f,-2.100630760192871f,0.4425620436668396f,1.4122081995010376f,-0.05937052518129349f,-0.9672968983650208f,0.5318191051483154f,-2.482858896255493f,1.6984894275665283f,0.044868115335702896f,-0.2853488028049469f,0.10096491873264313f,0.3123226463794708f,-0.3745303153991699f,-1.4566996097564697f,-1.0305569171905518f,-0.5739386081695557f,-0.4024588465690613f,-0.24655984342098236f,0.3242122530937195f,0.6324417591094971f,0.005632258951663971f,1.3113161325454712f,0.20008999109268188f,0.8430207967758179f,0.19590459764003754f,-1.7259334325790405f,-2.1521170139312744f,0.08098974823951721f,2.5562081336975098f,-0.19729194045066833f,0.4136350750923157f,1.6165403127670288f,1.5261400938034058f,-1.388520359992981f,-0.733588457107544f,0.0018298117211088538f,-1.8530561923980713f,-0.48888370394706726f,-2.0691123008728027f,0.1558954417705536f,1.2976632118225098f,-0.9435921907424927f,0.055737949907779694f,-0.038067977875471115f,-0.2717089354991913f,0.5260874032974243f,-1.635429859161377f,0.8563708662986755f,0.3537559509277344f,0.32603907585144043f,1.2030612230300903f,-0.27807536721229553f,0.8350478410720825f,3.336231231689453f,-0.4620303809642792f,0.030315104871988297f,0.08128595352172852f,0.017704004421830177f,-2.2033467292785645f,-2.036048412322998f,-1.6159045696258545f,-0.11926429718732834f,0.2126196324825287f,0.033561788499355316f,-0.23311199247837067f,-0.01792454905807972f,0.06997104734182358f,1.3900305032730103f,-0.4511719048023224f,-0.13783079385757446f,-0.27774858474731445f,1.1621216535568237f,-0.6660826206207275f,1.6098964214324951f,-0.034752365201711655f,1.5011509656906128f,-0.8558352589607239f,-0.2180950939655304f,-1.5574285984039307f,2.462735414505005f,0.13316339254379272f,0.7182590365409851f,0.1728094518184662f,-0.9390541315078735f,-0.16069698333740234f,2.1404404640197754f,2.3438825607299805f,-2.0533947944641113f,1.373549461364746f,0.216539666056633f,0.8868544101715088f,-0.6018487215042114f,0.922137439250946f,1.1836386919021606f,1.6596771478652954f,0.6922217607498169f,0.676250159740448f,0.8846907615661621f,0.26916876435279846f,1.0098687410354614f,-0.1941453218460083f,-1.492204189300537f,3.016193151473999f,-0.9044790267944336f,1.9221556186676025f,0.07363063842058182f,2.269749879837036f,-1.1675243377685547f,2.178778886795044f,-0.45217904448509216f,0.1475573480129242f,2.3955299854278564f,-0.7695152163505554f,0.41341185569763184f,0.09582589566707611f,-0.6623649001121521f,-0.9396027326583862f,0.11837787926197052f,-1.1644684076309204f,-0.7678231000900269f,2.8651790618896484f,0.12226394563913345f,0.7427355647087097f,-0.41537386178970337f,0.7668340802192688f,-1.3731765747070312f,-3.465501546859741f,0.48794418573379517f,0.11471938341856003f,-1.7861849069595337f,-2.079822301864624f,-0.12454318255186081f,-1.4574470520019531f,-0.16782312095165253f,0.0631457045674324f,0.14034046232700348f,0.8339475989341736f,-0.37896549701690674f,1.1964257955551147f,-1.1340724229812622f,1.2943131923675537f,0.7723396420478821f,-1.1939465999603271f,-0.5927640199661255f,-1.1824595928192139f,-0.013964677229523659f,-0.07029017806053162f,1.5979853868484497f,0.5291420817375183f,0.836152195930481f,0.3695424199104309f,1.9666471481323242f,0.0006275943014770746f,0.8007241487503052f,-2.0710971355438232f,0.1893787384033203f,0.48237523436546326f,-1.424339771270752f,3.179549217224121f,2.1722099781036377f,1.1731570959091187f,0.6738256812095642f,-0.4589552879333496f,-0.1311810314655304f,-0.28025028109550476f,-1.1141079664230347f,1.1990203857421875f,-1.3913944959640503f,0.4957706928253174f,1.8563851118087769f,-0.2817835211753845f,-0.25683119893074036f,1.47149658203125f,-3.5361928939819336f,0.17268769443035126f,-0.9932469129562378f,-1.0660338401794434f,1.4153351783752441f,0.05398394167423248f,-0.12697260081768036f,-1.3390833139419556f,0.04388841986656189f,1.003740668296814f,0.6900317668914795f,1.593342661857605f,-0.21285229921340942f,0.3634893000125885f,-1.8065507411956787f,-3.61342191696167f,-0.4601881802082062f,0.263204962015152f,2.8884027004241943f,1.4447078704833984f,-0.19858835637569427f,0.6289577484130859f,-0.8923155069351196f,1.1640398502349854f,0.2881985306739807f,-0.38567858934402466f,-0.52126145362854f,1.2443716526031494f,1.9575896263122559f,0.1966816931962967f,0.3724127411842346f,-0.7231525778770447f,-0.36710572242736816f,-1.4144623279571533f,-0.2870405614376068f,-0.08472801744937897f,1.5076404809951782f,-0.000616810517385602f,-1.6442420482635498f,-1.0443624258041382f,-0.44605112075805664f,-0.19794391095638275f,0.03873076289892197f,0.4867536127567291f,-2.686649799346924f,2.267821788787842f,-1.2501378059387207f,0.6808936595916748f,0.902949869632721f,-1.0291517972946167f,-0.18166814744472504f,0.22616641223430634f,-1.744217872619629f,1.842510461807251f,-0.7534796595573425f,0.4627218246459961f,-0.1365012228488922f,-0.26395341753959656f,-0.4013799726963043f,-0.6115627884864807f,-0.49269524216651917f,-1.7762247323989868f,-1.2157942056655884f,1.209256649017334f,-0.7389706969261169f,-1.6400742530822754f,-0.09121713042259216f,1.1893800497055054f,-1.0907725095748901f,-1.80224609375f,-0.15778414905071259f,0.8627282381057739f,1.3723416328430176f,1.1748725175857544f,1.832442045211792f,-1.5716214179992676f,-0.593174934387207f,-0.11010317504405975f,0.5131648778915405f,0.09686962515115738f,0.1702563464641571f,-0.2936040461063385f,-1.4142355918884277f,2.2365589141845703f,-1.4281322956085205f,-0.18354220688343048f,1.3533060550689697f,-0.7369424104690552f,-2.133650064468384f,-1.8703112602233887f,0.9354501962661743f,-2.9916300773620605f,0.7588923573493958f,1.3607152700424194f,0.3759441375732422f,-0.6939588785171509f,0.6881765723228455f,0.9664920568466187f,1.2628389596939087f,0.43628180027008057f,0.6559639573097229f,-3.261408805847168f,-0.5303704738616943f,-2.0209085941314697f,-0.7241206169128418f,-0.7227454781532288f,-1.1574842929840088f,-1.190820336341858f,0.8159317374229431f,-1.6574565172195435f,1.4725651741027832f,1.768425703048706f,0.7171012163162231f,-0.3407194912433624f,-0.7768516540527344f,-0.1878131479024887f,0.6329103112220764f,-1.5978957414627075f,-1.0761187076568604f,0.7005725502967834f,0.3157508671283722f,-2.354567050933838f,0.8426262736320496f,1.0415269136428833f,2.0983269214630127f,-0.1047183945775032f,0.5541873574256897f,0.2602027356624603f,-1.9987565279006958f,-0.8037563562393188f,-0.21128469705581665f,-0.7834020853042603f,-1.0184614658355713f,-0.7829369306564331f,-1.0888035297393799f,-1.1159592866897583f,2.864647150039673f,-0.15526823699474335f,4.124043941497803f,-1.551577091217041f,-1.8918112516403198f,-0.6334543228149414f,1.2134939432144165f,-0.5915223956108093f,-0.7661207914352417f,-0.7380077838897705f,1.6868929862976074f,-0.5029944777488708f,0.3072289824485779f,-1.7094931602478027f,-1.5281262397766113f,0.6718414425849915f,2.7349586486816406f,0.6285378932952881f,0.7855784296989441f,-1.4588127136230469f,1.1282312870025635f,1.2281293869018555f,-0.2739318013191223f,1.0629338026046753f,0.17278462648391724f,1.633079171180725f,1.2779207229614258f,0.07336409389972687f,-0.10037484019994736f,-0.2193693369626999f,-1.4114118814468384f,3.42918062210083f,1.0870195627212524f,0.032494619488716125f,2.1580049991607666f,-1.7053782939910889f,-2.256145477294922f,-1.5959707498550415f,0.48382678627967834f,-0.031099257990717888f,-0.975823163986206f,0.441705584526062f,-0.6921299695968628f,-3.5927650928497314f,0.6378128528594971f,-0.7935693860054016f,0.13879457116127014f,0.7545985579490662f,-2.3599636554718018f,-1.4633132219314575f,-2.114457130432129f,0.24978157877922058f,0.9559211134910583f,-1.455048680305481f,0.31326088309288025f,-1.6637994050979614f,1.1593425273895264f,-1.4090850353240967f,-0.10179212689399719f,1.0190105438232422f,-0.8109318614006042f,-0.3592907190322876f,-0.3774549663066864f,-0.736160397529602f,-0.3612559735774994f,-0.8093797564506531f,0.14901061356067657f,-0.19491977989673615f,2.13582706451416f,-0.097320556640625f,-0.32455816864967346f,0.5856330990791321f,1.3609933853149414f,-0.12758968770503998f,0.6288007497787476f,1.6567144393920898f,0.09537584334611893f,-0.018769241869449615f,0.1152244508266449f,-3.521432876586914f,-2.4247496128082275f,1.7888941764831543f,-0.8732839226722717f,1.2050831317901611f,0.9945700764656067f,0.8580647706985474f,-1.459096074104309f,1.8472506999969482f,0.4673179090023041f,-2.609541177749634f,3.1766624450683594f,1.4796829223632812f,0.21585646271705627f,-0.29152974486351013f,-0.8262408375740051f,-1.109898567199707f,-0.008680524304509163f,-1.0128748416900635f,-1.1107326745986938f,0.07597511261701584f,-1.6030910015106201f,-0.944293737411499f,2.119807243347168f,-1.1812317371368408f,-1.7603204250335693f,-0.8700015544891357f,0.7323456406593323f,-0.6211661696434021f,0.47788092494010925f,0.5899319052696228f,1.16060471534729f,-0.11941657215356827f,0.2103489488363266f,-0.17503617703914642f,1.5158694982528687f,0.5740020275115967f,-1.4755361080169678f,-0.4504924416542053f,2.9511046409606934f,0.809075653553009f,-0.4818092882633209f,0.127915620803833f,1.304402470588684f,0.9620115756988525f,-0.5105637311935425f,-1.5921399593353271f,-0.49365735054016113f,-0.6475855708122253f,2.1108007431030273f,-0.25353190302848816f,-3.1472103595733643f,-1.5392333269119263f,-0.5872166752815247f,-0.7481462359428406f,-1.1815003156661987f,1.3820087909698486f,-0.9944525361061096f,-1.7673767805099487f,-0.9898318648338318f,-0.8343415856361389f,0.29798638820648193f,-0.8731775879859924f,-2.251075506210327f,-0.657707929611206f,-0.23786522448062897f,2.0118651390075684f,-0.2155364602804184f,-0.2088382989168167f,-0.7103456854820251f,1.350704312324524f,0.8687248826026917f,-2.1925082206726074f,-0.8690149784088135f,-1.1091008186340332f,0.22691220045089722f,-1.8193020820617676f,-0.18679317831993103f,-0.22560739517211914f,1.5989717245101929f,1.6385166645050049f,-0.7648690342903137f,-1.5483909845352173f,2.1870648860931396f,0.3781871497631073f,0.49940693378448486f,0.21253971755504608f,0.4895530939102173f,1.0209972858428955f,-0.06777607649564743f,-0.6057259440422058f,0.613145112991333f,1.1354838609695435f,0.5760965943336487f,-0.48698779940605164f,-1.1662547588348389f,0.6675967574119568f,1.6059833765029907f,-1.3394536972045898f,1.6993484497070312f,0.48398852348327637f,-1.1093544960021973f,-2.206050395965576f,-2.264080047607422f,-0.6211138367652893f,-1.3890596628189087f,-0.35437914729118347f,-1.8440486192703247f,1.3329976797103882f,0.5011066794395447f,0.5848383903503418f};
alignas(16) float batch_normalization_10_A[] = {3.523482084274292f,2.177792549133301f,0.5418968796730042f,2.4669837951660156f,2.910290479660034f,1.5670442581176758f,3.1095714569091797f,0.8662702441215515f,-0.16609913110733032f,2.318248987197876f,0.3262211084365845f,3.8249332904815674f,1.3323478698730469f,0.4410874545574188f,3.8999385833740234f,-0.5204768776893616f,-0.5736913681030273f,-3.566403865814209f,2.7618935108184814f,0.06502717733383179f,4.171037197113037f,2.219714403152466f,4.520316123962402f,1.9080820083618164f,4.034090518951416f,4.108101844787598f,0.2558993697166443f,-0.9489204287528992f,2.451896905899048f,3.386479377746582f,-0.583057165145874f,3.3418071269989014f,1.7143867015838623f,-1.124367594718933f,0.5045118927955627f,2.66032338142395f,4.339940547943115f,-1.7800588607788086f,-1.5757684707641602f,1.5006158351898193f};
alignas(16) float DetectionLayer_internal_0_VALUES[] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
alignas(16) float DetectionLayer_internal_1_W[] = {-0.0107040423899889f,-0.016565384343266487f,-0.01137953344732523f,0.004735283553600311f,-0.04331386461853981f,-0.03676794469356537f,0.027624987065792084f,0.008854375220835209f,-0.015587995760142803f,0.001698087202385068f,0.006393707823008299f,-0.0018733240431174636f,0.08360472321510315f,-0.006483103614300489f,-0.008047712035477161f,0.013910296373069286f,-0.023978130891919136f,0.01490433793514967f,0.005936172790825367f,-0.0002525719173718244f,0.014882197603583336f,0.011547067202627659f,-0.005758565850555897f,-0.005801489111036062f,0.010501168668270111f,0.001019444316625595f,-0.00762854004278779f,0.014875641092658043f,-0.044596996158361435f,-0.059919510036706924f,0.08152947574853897f,-0.021256767213344574f,0.005851855967193842f,0.005376364104449749f,0.0019778823480010033f,0.00295875477604568f,0.08263540267944336f,-0.06460180133581161f,0.1144600436091423f,-0.049731045961380005f,-0.07564689218997955f,-0.011358719319105148f,0.013764138333499432f,0.0003283209225628525f,0.05788438767194748f,0.016705293208360672f,-0.008347876369953156f,-0.008367038331925869f,0.008817282505333424f,0.013303606770932674f,-0.007814528420567513f,-0.0007798532024025917f,0.06851125508546829f,-0.04942023381590843f,0.13326317071914673f,-0.08488040417432785f,-0.007901511155068874f,0.008470514789223671f,0.005662305746227503f,0.0023979831021279097f,0.036992743611335754f,0.01760789379477501f,0.004671160131692886f,-0.022219505161046982f,0.009674923494458199f,0.022656377404928207f,-0.00354931247420609f,0.0020435662008821964f,0.11731071770191193f,0.023669065907597542f,-0.01182203833013773f,-0.011838415637612343f,0.0006017611594870687f,-0.020300792530179024f,0.004575391300022602f,-0.00227946019731462f,0.017177201807498932f,0.1435420960187912f,-0.13948321342468262f,-0.004064315464347601f,0.00961411464959383f,-0.005623583681881428f,0.0029085837304592133f,-0.003787974128499627f,0.05186768248677254f,0.03076276183128357f,-0.03251141309738159f,0.0014189602807164192f,-0.0032162582501769066f,-0.011988633312284946f,0.00733364000916481f,-0.001356721157208085f,0.1125551387667656f,0.010808243416249752f,-0.0053876363672316074f,-0.005400307476520538f,-0.010760444216430187f,0.041575293987989426f,0.014255237765610218f,0.01570245437324047f,-0.03919048607349396f,-0.05742053687572479f,0.06726203113794327f,-0.010610979050397873f,0.004476556554436684f,-0.002988590393215418f,0.0008772705332376063f,-0.0069985296577215195f,0.09040666371583939f,-0.026042042300105095f,0.04580876603722572f,-0.019832022488117218f,-0.01736300438642502f,0.006817359011620283f,-0.002687314758077264f,-0.0014626635238528252f,-0.11007893830537796f,0.00639860425144434f,-0.00320675247348845f,-0.0031825238838791847f,0.03119952417910099f,0.060246728360652924f,0.0029824862722307444f,0.011973576620221138f,0.1928635537624359f,-0.04941761866211891f,0.09891295433044434f,-0.0496189184486866f,0.0487261638045311f,0.002361839171499014f,-0.00434174295514822f,0.002536541083827615f,-0.1469775140285492f,-0.059880055487155914f,0.07941219210624695f,-0.019517073407769203f,0.0018775237258523703f,0.008433309383690357f,-0.006721160374581814f,-0.0012797100935131311f,-0.13392294943332672f,0.012082762084901333f,-0.0060701267793774605f,-0.006063283886760473f,-0.0018074715044349432f,-0.02162241004407406f,-0.004552246071398258f,-0.017579413950443268f,0.04802658036351204f,-0.02796953357756138f,-0.014691151678562164f,0.042739253491163254f,-0.01757984422147274f,-0.01236364059150219f,0.0029553978238254786f,-0.00010289264901075512f,0.05748933181166649f,-0.013588838279247284f,-0.02441912703216076f,0.03773621842265129f,0.005464079789817333f,0.02349683828651905f,1.0086732800118625e-06f,0.001289471285417676f,-0.022445620968937874f,0.011200476437807083f,-0.005620149429887533f,-0.005606148391962051f,0.02942824549973011f,-0.017860669642686844f,-0.005842185113579035f,-0.007006440311670303f,0.20024076104164124f,-0.04177260771393776f,0.08243632316589355f,-0.04067963361740112f,-0.019136663526296616f,-0.02016572281718254f,-0.003425504779443145f,-0.003982018679380417f,-0.13828355073928833f,-0.01851637288928032f,0.031573742628097534f,-0.013217002153396606f,-0.013965568505227566f,-0.03830743581056595f,0.002786678494885564f,-0.0021873097866773605f,0.044610004872083664f,0.020095281302928925f,-0.010044998489320278f,-0.010048705153167248f,-0.006119505502283573f,-0.0014468827284872532f,0.0033938447013497353f,0.005885089747607708f,0.0120328888297081f,-0.0728321224451065f,0.05857216566801071f,0.014221280813217163f,-0.04291980713605881f,0.007309066131711006f,0.003692333120852709f,-0.003524038242176175f,-0.17118078470230103f,-0.0376802422106266f,0.05272097513079643f,-0.015176111832261086f,-0.03155449405312538f,0.004194771870970726f,0.005610000342130661f,0.0014350914862006903f,0.17722967267036438f,0.03219980746507645f,-0.01608656533062458f,-0.016067197546362877f,-0.023789560422301292f,-0.014728616923093796f,-0.0011412282474339008f,0.00017154419037979096f,-0.0700649842619896f,0.05540085956454277f,-0.03995497152209282f,-0.01548097375780344f,0.008524110540747643f,0.0025334369856864214f,-0.0006047048955224454f,0.004302665591239929f,0.0300068948417902f,0.03826881945133209f,-0.04436373710632324f,0.00594665389508009f,0.034634120762348175f,0.014455893076956272f,-0.008184242062270641f,0.0008524084696546197f,0.14241406321525574f,0.021641315892338753f,-0.010803338140249252f,-0.010816853493452072f,0.027423759922385216f,0.009768268093466759f,0.01105987187474966f,0.005010152235627174f,0.024952372536063194f,-0.010091125033795834f,-0.029163246974349022f,0.03927517682313919f,0.004208932165056467f,-0.00893351249396801f,-0.001806528540328145f,-0.008524379692971706f,-0.02767685241997242f,-0.028745178133249283f,-0.04298411309719086f,0.07195180654525757f,0.05739707872271538f,0.0016555546317249537f,-0.0019334895769134164f,0.0008673164411447942f,-0.03650495409965515f,0.0012445769971236587f,-0.0006238995119929314f,-0.0005940687842667103f,0.018749890848994255f,0.0038390960544347763f,-0.012942995876073837f,0.011371279135346413f,-0.0973113551735878f,0.024050956591963768f,-0.01579882949590683f,-0.008238117210566998f,0.0036184017080813646f,0.027840258553624153f,0.01098692137748003f,0.013595872558653355f,-0.06942429393529892f,0.014047052711248398f,-0.004837934859097004f,-0.009128265082836151f,-0.03736359253525734f,0.01360581535845995f,-0.0029654453974217176f,-0.0017233288381248713f,0.06398528814315796f,0.018668772652745247f,-0.009294639341533184f,-0.00931349117308855f,0.021758677437901497f,0.029376652091741562f,0.005551270674914122f,0.0017042288091033697f,-0.027600163593888283f,-0.04699374735355377f,-0.05867205560207367f,0.10539724677801132f,0.005840468220412731f,-0.0019287476316094398f,0.0019347352208569646f,0.002668233122676611f,0.003231834154576063f,0.01778910681605339f,-0.06605589389801025f,0.048456646502017975f,0.0010477823670953512f,0.018111111596226692f,0.00024167717492673546f,0.0031955661252141f,-0.022647669538855553f,0.008834927342832088f,-0.0044203586876392365f,-0.004380448721349239f,-0.02725091204047203f,0.008700955659151077f,0.004435812588781118f,0.0159298162907362f,-0.09911956638097763f,-0.04342121630907059f,0.046912286430597305f,-0.003781593171879649f,0.009035232476890087f,0.0036941557191312313f,-0.0030061989091336727f,0.0012255290057510138f,0.022043246775865555f,-0.0005006471765227616f,-0.004351923242211342f,0.004653693176805973f,0.007904229685664177f,-0.033602241426706314f,-0.0004897629260085523f,-0.0023024610709398985f,-0.11623500287532806f,0.0065648616291582584f,-0.0032978011295199394f,-0.0032964153215289116f,0.008635801263153553f,-0.013614608906209469f,0.022827528417110443f,0.01558716781437397f,0.09226322174072266f,0.05647696927189827f,-0.0854417160153389f,0.02901950292289257f,0.030011624097824097f,0.020297862589359283f,-0.006841517984867096f,0.006203122902661562f,0.013346622698009014f,-0.0004162814875598997f,-0.014786955900490284f,0.015140105970203876f,0.0429663322865963f,-0.0004374183772597462f,-0.012226525694131851f,0.0030895692761987448f,-0.014688311144709587f,0.005126527510583401f,-0.002540468005463481f,-0.002545164432376623f,-0.026995275169610977f,-0.008476722985506058f,0.001816507545299828f,0.012847993522882462f,-0.06119156628847122f,0.04386548697948456f,0.07623772323131561f,-0.12066193670034409f,-0.010826129466295242f,0.005904970690608025f,0.0037111700512468815f,-0.005279043223708868f,0.06050935387611389f,-0.05536385625600815f,0.1104109063744545f,-0.05532580986618996f,-0.08028344064950943f,0.01291702315211296f,0.009433585219085217f,0.005208607297390699f,-0.3219623863697052f,0.010290549136698246f,-0.0051556420512497425f,-0.005157192703336477f,0.03765179589390755f,-0.015289917588233948f,-0.0101337730884552f,-0.028983620926737785f,-0.160921111702919f,-0.051389116793870926f,0.07164042443037033f,-0.02052326872944832f,-0.11026103794574738f,0.009855803102254868f,0.013534881174564362f,0.010024012066423893f,-0.3788081705570221f,-0.0066742985509335995f,0.00936543382704258f,-0.0025855882558971643f,-0.06318491697311401f,0.00823846086859703f,0.004301487933844328f,0.0021782140247523785f,-0.12448619306087494f,0.012193543836474419f,-0.006085485219955444f,-0.006130532827228308f,0.00831294059753418f,-0.0009934973204508424f,-0.0042230673134326935f,-0.003123971400782466f,-0.11191661655902863f,0.041768889874219894f,0.005845855921506882f,-0.047494933009147644f,0.008747855201363564f,-0.008769512176513672f,0.004332874435931444f,0.0038154558278620243f,-0.0927504301071167f,0.003258984535932541f,-0.003067098557949066f,-0.0002620222221594304f,-0.0063693942502141f,0.023136405274271965f,-0.006406671833246946f,0.0026652272790670395f,-0.10046208649873734f,0.001570800319314003f,-0.0007938710041344166f,-0.0007667518802918494f,0.004203838296234608f,-0.020208321511745453f,0.006213798653334379f,-0.008790127001702785f,0.03427629917860031f,-0.036687903106212616f,0.00525486096739769f,0.03139937296509743f,0.03553697466850281f,-0.007471168413758278f,-0.004363230895251036f,-0.005033901892602444f,-0.1407667100429535f,-0.03377511724829674f,0.001665923511609435f,0.03224732354283333f,0.06568310409784317f,-0.04841606318950653f,0.00019081388018094003f,-0.005361975636333227f,0.048621684312820435f,0.0039060451090335846f,-0.0019673167262226343f,-0.001974179409444332f,0.023891828954219818f,-0.028953729197382927f,0.0003556242154445499f,-0.01031232625246048f,-0.014136360958218575f,-0.011176543310284615f,0.0009620705968700349f,0.009926111437380314f,0.03201552480459213f,-0.011519867926836014f,-0.015298016369342804f,0.005986079107969999f,-0.008087018504738808f,-0.01249717827886343f,0.004197897855192423f,0.00811181589961052f,-0.05379751697182655f,-0.01257357932627201f,0.000872574164532125f,-0.00044125181739218533f,-0.10175515711307526f,0.00011728179379133508f,-4.819094465347007e-05f,-6.927150388946757e-05f,-0.04058072715997696f,0.004687105771154165f,0.004595738369971514f,0.01181003637611866f,0.04307739809155464f,-0.0498352088034153f,-0.05785574018955231f,0.10812943428754807f,-0.00024403177667409182f,-0.009196380153298378f,-0.003982229623943567f,-0.010150101035833359f,-0.011437215842306614f,-0.07998229563236237f,-0.012899787165224552f,0.09310959279537201f,0.013393388129770756f,-0.012083026580512524f,-0.00038882545777596533f,-0.0015864745946601033f,0.07088816165924072f,0.011890561319887638f,-0.0059603299014270306f,-0.005933665204793215f,0.013284510932862759f,0.014207035303115845f,-0.0008035225910134614f,0.015473924577236176f,-0.025596745312213898f,0.021977178752422333f,0.01344467606395483f,-0.03568838909268379f,-0.02086814120411873f,-0.017589667811989784f,-0.003669065423309803f,-0.000606931687798351f,-0.10080477595329285f,-0.07570390403270721f,0.05755298212170601f,0.01813030056655407f,0.005269227083772421f,-0.011519157327711582f,0.005327051971107721f,-0.0025366235058754683f,0.009830805473029613f,0.01003190316259861f,-0.005040663294494152f,-0.0050489641726017f,-0.03569529205560684f,-0.0071522886864840984f,0.0018280858639627695f,-0.0046273632906377316f,-0.21900653839111328f,-0.035363227128982544f,0.004800518974661827f,0.030903257429599762f,-0.0100614158436656f,-0.01921968348324299f,0.0029292157851159573f,0.009868931025266647f,0.06336919218301773f,-0.12212315946817398f,0.14267414808273315f,-0.020632628351449966f,-0.008249519392848015f,-0.002394655253738165f,0.0009694721084088087f,0.00041515514021739364f,-0.029188139364123344f,0.004135066643357277f,-0.0020386571995913982f,-0.0020717093721032143f,0.015899373218417168f,-0.0003470805531833321f,-0.0025385688059031963f,-0.009432438760995865f,0.06748082488775253f,0.01947573758661747f,-0.03051747940480709f,0.0113792410120368f,-0.023668507114052773f,-0.006188816390931606f,-0.00044361225445754826f,-0.007608687039464712f,-0.02013065665960312f,0.07651405036449432f,-0.08781886100769043f,0.010323108173906803f,-0.013571145944297314f,-0.039033859968185425f,0.0035814254079014063f,-0.00366193731315434f,0.026766164228320122f,0.014828067272901535f,-0.007413134910166264f,-0.007418760564178228f,0.0029657750856131315f,0.010759233497083187f,0.008351500146090984f,0.0037793696392327547f,-0.005215284880250692f,-0.03140672668814659f,0.07493486255407333f,-0.04329102486371994f,0.035495296120643616f,0.021607765927910805f,-0.0076014194637537f,-0.01499977707862854f,0.13877959549427032f,0.001663382863625884f,0.05316658318042755f,-0.055154457688331604f,0.0006736537907272577f,0.00417668279260397f,0.015549293719232082f,0.0025682251434773207f,-0.017665548250079155f,0.018525579944252968f,-0.0092643853276968f,-0.009256318211555481f,0.01350813265889883f,-0.004168393090367317f,0.011404409073293209f,0.010438057594001293f,0.07982159405946732f,-0.00940610095858574f,-0.007140607573091984f,0.01615637168288231f,-0.008826241828501225f,-0.03931576386094093f,-0.0058316695503890514f,-0.004043974913656712f,-0.03415779396891594f,-0.03884836286306381f,0.04277998208999634f,-0.004795775283128023f,-0.0384165495634079f,-0.010122577659785748f,0.0007053282461129129f,0.0015138263115659356f,-0.0036742526572197676f,0.0041140909306705f,-0.002080374164506793f,-0.002051315037533641f,0.0379478745162487f,-0.03839913383126259f,-0.006263787392526865f,0.0016447054222226143f,-0.13022185862064362f,0.01417296752333641f,0.03434326872229576f,-0.04815210774540901f,0.0011517747770994902f,-0.0059486133977770805f,-0.0044723679311573505f,-0.004095830954611301f,0.0721428245306015f,-0.03468210622668266f,0.05611461028456688f,-0.021592548117041588f,-0.010859073139727116f,-0.01453559473156929f,-0.008189293555915356f,-0.0015042450977489352f,0.042575109750032425f,0.013918058015406132f,-0.006970956455916166f,-0.006956552620977163f,-0.003702242858707905f,0.0158437117934227f,-0.008893704041838646f,-0.0075226156041026115f,0.11938906461000443f,0.062279507517814636f,0.026086485013365746f,-0.08909591287374496f,-0.019403787329792976f,0.002635116223245859f,-0.00041346397483721375f,0.0038009348791092634f,-0.08935532718896866f,-0.028386715799570084f,0.08431687951087952f,-0.056642334908246994f,0.019935613498091698f,-0.006399722304195166f,0.001533480011858046f,-3.78400509362109e-05f,-0.10251984000205994f,0.007106958422809839f,-0.0035393633879721165f,-0.0035607293248176575f,-0.013285077176988125f,-0.004061792977154255f,-0.00010218602983513847f,0.0024467906914651394f,0.010938921011984348f,-0.0773165225982666f,0.08167198300361633f,-0.004410085268318653f,-0.034684792160987854f,0.00868319720029831f,0.008743253536522388f,0.010818514972925186f,-0.1804281771183014f,-0.025036830455064774f,0.03846189007163048f,-0.013380808755755424f,-0.03670208528637886f,-0.002930499380454421f,0.0035306767094880342f,4.0198792703449726e-05f,-0.00983384344726801f,0.03422863781452179f,-0.01712978444993496f,-0.017134329304099083f,0.01683778502047062f,0.0036421071272343397f,-0.0014087300514802337f,0.018850058317184448f,-0.08547411859035492f,-0.06786718219518661f,0.07682333141565323f,-0.010002870112657547f,0.007364198565483093f,0.00736263720318675f,0.0017626022454351187f,0.006553513463586569f,0.21856459975242615f,-0.04046754166483879f,0.025903111323714256f,0.01441744714975357f,-0.001579121919348836f,0.0070904023014009f,-0.006796308793127537f,-0.0017700052121654153f,-0.10869760811328888f,0.006163831800222397f,-0.0030843436252325773f,-0.0030690659768879414f,0.019390083849430084f,-0.014229667373001575f,-0.016407925635576248f,0.009555388242006302f,-0.11813414841890335f,0.11668076366186142f,-0.10569361597299576f,-0.01093288417905569f,0.014636049047112465f,0.00798637792468071f,0.0010630546603351831f,-0.0032160882838070393f,-0.01190275326371193f,0.03876476362347603f,-0.03539564087986946f,-0.0034358922857791185f,0.05160563439130783f,0.024737564846873283f,-0.012940564192831516f,0.0017701403703540564f,-0.1966032087802887f,0.0032013568561524153f,-0.0016256127273663878f,-0.0015791341429576278f,-0.03356301411986351f,-0.021996667608618736f,0.011137144640088081f,0.003548332955688238f,0.009358685463666916f,-0.1371610462665558f,0.07135636359453201f,0.06452631950378418f,-0.007643234450370073f,0.005501674488186836f,-0.0029745264910161495f,0.001330205355770886f,0.200385183095932f,-0.09832801669836044f,0.03318661078810692f,0.06502177566289902f,0.010865635238587856f,0.013433480635285378f,0.0035866631660610437f,-0.0011316004674881697f,-0.11932454258203506f,0.008312520571053028f,-0.004131882451474667f,-0.004169599153101444f,0.01260421983897686f,0.007115642540156841f,0.005333948414772749f,0.009495360776782036f,-0.05700956657528877f,0.02395852468907833f,-0.04618135094642639f,0.022631138563156128f,0.01091044582426548f,-0.006420629099011421f,-0.002451240085065365f,0.003604183904826641f,-0.12658686935901642f,-0.02253860794007778f,-0.023979637771844864f,0.046076297760009766f,-0.007293686270713806f,0.0014255208661779761f,0.0023084317799657583f,0.0038562193512916565f,-0.21223101019859314f,0.0062597328796982765f,-0.003129704622551799f,-0.0031406537164002657f,0.0337735079228878f,0.04072577878832817f,-0.0015694759786128998f,0.02646736055612564f,-0.0023764122743159533f,0.05988195165991783f,0.01045991014689207f,-0.0698576495051384f,-0.010458861477673054f,-0.013612119480967522f,0.0035695391707122326f,0.009150062687695026f,0.1296120584011078f,0.10178937017917633f,-0.03320401534438133f,-0.06848400831222534f,0.003782212035730481f,-0.020473076030611992f,-0.007459322921931744f,-0.0032840182539075613f,0.09163690358400345f,0.03750431165099144f,-0.01877622865140438f,-0.018761321902275085f,0.07077337056398392f,-0.07090935856103897f,-0.007735654711723328f,-0.0005204710178077221f,-0.007824729196727276f,0.03153834491968155f,-0.01569945551455021f,-0.015722934156656265f,0.015450488775968552f,-0.004589412361383438f,-0.005841995123773813f,0.00976125244051218f,-0.10525064170360565f,0.005855944473296404f,-0.013152699917554855f,0.007290339097380638f,-0.01980428956449032f,0.054176703095436096f,-0.008768634870648384f,0.004757589660584927f,-0.011939763091504574f,0.01664106361567974f,-0.008328565396368504f,-0.008308897726237774f,-0.026776092126965523f,0.009579653851687908f,0.009673953987658024f,-0.005255024414509535f,-0.03673892468214035f,0.03462725877761841f,0.0140857994556427f,-0.04802951589226723f,0.0020072462502866983f,-0.0003099444729741663f,0.0044898102059960365f,-0.00047189940232783556f,-0.01869138889014721f,0.026033854112029076f,-0.019851386547088623f,-0.006859187036752701f,0.017193825915455818f,0.053777992725372314f,-0.007316105533391237f,0.002995467511937022f,0.124186672270298f,0.010586857795715332f,-0.005280705634504557f,-0.0053200009278953075f,-0.02337488904595375f,-0.03711983188986778f,-0.003108531702309847f,-0.007535264827311039f,-0.14128796756267548f,0.0074924868531525135f,-0.00511470902711153f,-0.00243106740526855f,-0.08731067180633545f,-0.01386339496821165f,0.00906310323625803f,0.005685714073479176f,0.05635495483875275f,0.0014836983755230904f,0.050215668976306915f,-0.05213858559727669f,-0.008069242350757122f,0.020869813859462738f,0.0038744499906897545f,0.0017571205971762538f,0.07281482964754105f,0.02360367961227894f,-0.011822131462395191f,-0.01179112121462822f,0.02343282848596573f,0.0021300024818629026f,-0.0016220887191593647f,0.01667471043765545f,0.048507705330848694f,0.07432498782873154f,-0.01813431642949581f,-0.05556047335267067f,-0.0028855593409389257f,-0.014819271862506866f,2.688082167878747e-05f,-0.0011102933203801513f,-0.0801713615655899f,0.041170597076416016f,-0.01182151585817337f,-0.029463503509759903f,-0.05126109719276428f,-0.005762977059930563f,0.0006176464376039803f,9.029168722918257e-05f,-0.058297764509916306f,0.024024764075875282f,-0.012027674354612827f,-0.012030957266688347f,-0.03094474785029888f,0.0178590789437294f,0.005990026518702507f,0.012533042579889297f,-0.20945461094379425f,0.022706424817442894f,-0.004833699204027653f,-0.017926976084709167f,-0.018874088302254677f,-0.0029714484699070454f,-0.004437844734638929f,0.004937429912388325f,-0.28245189785957336f,0.028068218380212784f,-0.015891246497631073f,-0.012227228842675686f,-0.009477349929511547f,-0.020444391295313835f,-0.0011483879061415792f,0.0008952650241553783f,0.1119547113776207f,0.02360200136899948f,-0.01177242025732994f,-0.011785377748310566f,0.0022741376888006926f,-0.008229711093008518f,-0.011747116222977638f,-0.009971179068088531f,-0.054125573486089706f,-0.08272527158260345f,0.1149330884218216f,-0.03213439881801605f,0.02398417517542839f,-0.007584066595882177f,-0.004185759928077459f,0.006886940449476242f,-0.07888956367969513f,-0.007262962404638529f,0.01670895516872406f,-0.009870639070868492f,0.022949788719415665f,-0.009278576821088791f,-0.005079591181129217f,0.0008087219321168959f,0.21159031987190247f,0.014567147940397263f,-0.007307765539735556f,-0.007275850977748632f,0.0028597149066627026f,0.03790678083896637f,-0.0030959879513829947f,0.012576252222061157f,-0.1501239538192749f,0.005772455129772425f,-0.007927305065095425f,0.002144590485841036f,0.018349863588809967f,0.023947473615407944f,0.0009402282885275781f,0.0034679726231843233f,-0.0007989240111783147f,0.02531726099550724f,-0.08716510981321335f,0.06151244789361954f,0.00758358184248209f,0.019461248070001602f,-0.010711077600717545f,0.006482758093625307f,-0.07409361004829407f,0.005032313521951437f,-0.002499341731891036f,-0.0025072810240089893f,-0.00033485176390968263f,0.052538082003593445f,0.001075178966857493f,-0.010444995015859604f,-0.1837388426065445f,0.027410585433244705f,-0.041139766573905945f,0.013893535360693932f,0.005607550498098135f,-0.0007195186917670071f,0.0031501874327659607f,-0.0011841350933536887f,0.06483007222414017f,0.07993577420711517f,-0.05254175141453743f,-0.027681177482008934f,0.010265756398439407f,-0.02003568597137928f,-0.003999698907136917f,-0.0001774722768459469f,-0.07791679352521896f,0.033793263137340546f,-0.016886776313185692f,-0.016893498599529266f,-0.027673594653606415f,-0.002254233928397298f,-0.014092983677983284f,-0.006304316688328981f,0.18323498964309692f,-0.051372870802879333f,0.03307673707604408f,0.017928700894117355f,-0.025440162047743797f,-0.02631494775414467f,-0.004341453313827515f,0.0008452464244328439f,-0.0906061977148056f,-0.04952552542090416f,0.03591885790228844f,0.013543413020670414f,-0.027423374354839325f,0.016834912821650505f,-0.0036607009824365377f,0.00022499499027617276f,0.005965378601104021f,0.013585293665528297f,-0.0068008615635335445f,-0.006802957039326429f,0.011529247276484966f,0.05418866127729416f,0.017101945355534554f,0.009462731890380383f,-0.03027670830488205f,0.09854313731193542f,-0.059240613132715225f,-0.03970644995570183f,0.041312094777822495f,0.004186728969216347f,0.005360062699764967f,-0.014056330546736717f,0.05910012871026993f,0.042487744241952896f,-0.01809876412153244f,-0.024699855595827103f,0.06671017408370972f,0.028604499995708466f,0.006298963911831379f,0.0012990521499887109f,-0.0965583398938179f,0.011310126632452011f,-0.0056271422654390335f,-0.005635012872517109f,-0.056423503905534744f,-0.08824491500854492f,0.02269548922777176f,0.035592544823884964f,0.020410962402820587f,0.04937347397208214f,-0.11599353700876236f,0.06695160269737244f,-0.06292473524808884f,-0.16380305588245392f,-0.006111270748078823f,-0.006955968216061592f,0.061564866453409195f,0.04261564090847969f,-0.02969275787472725f,-0.012934288941323757f,-0.03153727576136589f,-0.1174691841006279f,-0.012264327146112919f,-0.003272311296314001f,-0.14653687179088593f,0.019462987780570984f,-0.009709319099783897f,-0.00971921719610691f,-0.024203257635235786f,0.032013192772865295f,-0.0019482985371723771f,0.013328615576028824f,-0.05217308551073074f,-0.010659839026629925f,0.029473869130015373f,-0.01905830018222332f,0.0285743847489357f,0.020289992913603783f,-0.004517524968832731f,-0.00268316431902349f,-0.0490143708884716f,-0.002401150995865464f,0.01922743208706379f,-0.016863111406564713f,-0.02367735095322132f,-0.004920112434774637f,0.0024585917126387358f,0.001973729347810149f,0.08105584979057312f,0.03697977215051651f,-0.018494615331292152f,-0.018490860238671303f,0.0362621545791626f,0.03878544643521309f,0.020559193566441536f,-0.0013193641789257526f,-0.0581488162279129f,0.0011973008513450623f,5.4559961427003145e-05f,-0.0006411431240849197f,-0.009792551398277283f,0.012090298347175121f,-0.0003129509568680078f,0.0070278095081448555f,-0.12635061144828796f,-0.01859320141375065f,0.0014241444878280163f,0.01773327775299549f,0.06675500422716141f,0.009131916798651218f,-0.00027668432448990643f,0.0011506867595016956f,0.017703361809253693f,0.011737944558262825f,-0.005859802942723036f,-0.005852186121046543f,0.02257673628628254f,-0.07768271118402481f,0.013278098776936531f,-0.00624103145673871f,-0.1180267333984375f,-0.021612346172332764f,0.056585218757390976f,-0.03536541387438774f,0.018207654356956482f,-0.12266948819160461f,0.003615268971771002f,-0.002606645692139864f,-0.008696332573890686f,0.05771202966570854f,-0.004320303909480572f,-0.05332766845822334f,0.0185684971511364f,-0.1278039515018463f,-0.001639531459659338f,-0.007407423108816147f,0.005999650340527296f,0.019564148038625717f,-0.009795735590159893f,-0.009802146814763546f,-0.00332345487549901f,0.044684503227472305f,0.012764799408614635f,-0.007517511956393719f,-0.011218279600143433f,-0.004593432880938053f,0.04541472718119621f,-0.040714919567108154f,0.035901252180337906f,0.027020538225769997f,0.0024085536133497953f,0.0026833657175302505f,-0.08441942185163498f,0.030221890658140182f,-0.01247375924140215f,-0.017899056896567345f,0.06462927907705307f,0.032338228076696396f,-0.002702639903873205f,0.0038932303432375193f,0.012573710642755032f,0.01754666119813919f,-0.008790155872702599f,-0.008760420605540276f,-0.003554676892235875f,-0.023947130888700485f,-0.003973052371293306f,-0.0017036248464137316f,0.04432405158877373f,0.051895104348659515f,-0.006518819369375706f,-0.04504374414682388f,-0.011248648166656494f,0.012667386792600155f,-0.0024432549253106117f,0.005951708648353815f,-0.2931959629058838f,-0.0057457322254776955f,-0.011404245160520077f,0.017339365556836128f,0.022950464859604836f,0.009736945852637291f,0.004617585800588131f,0.0009830533526837826f,-0.2644394636154175f,0.0003876048431266099f,-0.00017802423099055886f,-0.00018764100968837738f,-0.00354342395439744f,0.030645130202174187f,0.01742672175168991f,0.026364177465438843f,0.11130741238594055f,0.152833491563797f,-0.11637862026691437f,-0.037344593554735184f,-0.058115214109420776f,0.03505687415599823f,-0.01118407491594553f,-0.011854706332087517f,0.23918521404266357f,0.08886177092790604f,-0.07519359141588211f,-0.014006786979734898f,-0.02846481278538704f,0.056742604821920395f,0.005024741403758526f,0.0005510091432370245f,-0.3407477140426636f,0.007538861595094204f,-0.0037737740203738213f,-0.0037713933270424604f,-0.013501768000423908f,0.017719920724630356f,-0.01549497339874506f,-0.009713700972497463f,-0.15234853327274323f,0.04596798121929169f,-0.020795952528715134f,-0.02566446177661419f,0.014626015909016132f,-0.017949260771274567f,0.005591674242168665f,0.00435318099334836f,-0.08903853595256805f,0.10781347751617432f,-0.0945919081568718f,-0.013125800527632236f,0.014640514738857746f,0.003762112930417061f,-0.00055655837059021f,0.000926857115700841f,-0.02239288203418255f,0.028280610218644142f,-0.014140927232801914f,-0.014115756377577782f,-0.006991521455347538f,-0.014647357165813446f,-0.00710889371111989f,0.013549106195569038f,-0.14088669419288635f,0.06315149366855621f,-0.028631385415792465f,-0.034805577248334885f,0.014159290120005608f,-0.0033918889239430428f,0.0026368005201220512f,0.007165845483541489f,0.14011815190315247f,0.13777612149715424f,-0.1089608371257782f,-0.028736209496855736f,-0.043497756123542786f,0.004051652736961842f,-0.007315752096474171f,0.0016540397191420197f,-0.27091458439826965f,0.017152180895209312f,-0.00858236476778984f,-0.00857308879494667f,0.0019861548207700253f,0.025410909205675125f,-0.0012130887480452657f,-0.0011581939179450274f,0.1204354465007782f,-0.052753008902072906f,0.06003163382411003f,-0.007465239614248276f,-0.002306719310581684f,-0.003931546583771706f,-0.008818201720714569f,-0.00523828249424696f,-0.18754065036773682f,-0.04980264976620674f,0.008487301878631115f,0.04127936437726021f,0.03040517307817936f,-0.014824146404862404f,-0.0019945562817156315f,-0.00210156780667603f,-0.1160631999373436f,0.0014429902657866478f,-0.0007193866185843945f,-0.0007565142004750669f,0.021058261394500732f,0.07718554139137268f,-0.009957258589565754f,-0.01200328953564167f,-0.055366385728120804f,0.05797518044710159f,-0.022257762029767036f,-0.03672080859541893f,-0.028716403990983963f,0.07578491419553757f,0.003357944078743458f,0.001297767274081707f,-0.1678932160139084f,-0.031992800533771515f,0.024927208200097084f,0.006784398108720779f,-0.013688134960830212f,0.036607690155506134f,0.0002098808909067884f,0.0084708072245121f,-0.21824921667575836f,0.002730483654886484f,-0.0013834851561114192f,-0.0013791273813694715f,0.029109353199601173f,0.00471778167411685f,0.009779873304069042f,0.002836448373273015f,-0.09222932904958725f,0.012798253446817398f,-0.004407064523547888f,-0.008524518460035324f,-0.0035076220519840717f,0.010204371996223927f,-0.00369030493311584f,-0.0008567746262997389f,-0.28434520959854126f,-0.03818664327263832f,0.0826234444975853f,-0.04460051655769348f,-0.029232997447252274f,0.032115109264850616f,-0.003433351870626211f,0.0031037696171551943f,0.13366390764713287f,0.05852170288562775f,-0.029259629547595978f,-0.02925361879169941f,-0.0036671741399914026f,-0.014222036115825176f,-0.008314121514558792f,0.0022273575887084007f,-0.15247516334056854f,0.031001854687929153f,-0.014980314299464226f,-0.015871252864599228f,-0.018862903118133545f,-0.003313993103802204f,0.0017821058863773942f,0.0044315410777926445f,0.07095188647508621f,0.0369722880423069f,0.0396120585501194f,-0.0768028050661087f,0.01531955599784851f,-0.0051032789051532745f,-0.0033008581958711147f,7.488272240152583e-05f,0.0914435163140297f,0.04466322436928749f,-0.02233225479722023f,-0.022352883592247963f,0.001712606637738645f,0.010807611048221588f,0.0016684811562299728f,-0.007568799424916506f,-0.06024514511227608f,-0.054295316338539124f,0.0884416252374649f,-0.033810876309871674f,-0.002504153875634074f,0.012267637066543102f,0.0016811216482892632f,0.0016192725161090493f,-0.09191649407148361f,-0.11429910361766815f,0.08517825603485107f,0.02910679392516613f,0.018142223358154297f,-0.0023489214945584536f,-0.0005100563284941018f,0.0011348434491083026f,-0.2770552337169647f,0.00889820046722889f,-0.004434350412338972f,-0.004457272123545408f,0.014869097620248795f,0.0026540779508650303f,-0.013190456666052341f,-0.000122426834423095f,-0.12180349230766296f,-0.051809392869472504f,0.03915369138121605f,0.012086642906069756f,-0.05567080155014992f,0.01800801046192646f,0.013469729572534561f,0.006387326400727034f,-0.02383279986679554f,-0.005604283884167671f,-0.02377820573747158f,0.02843286469578743f,-0.07186891883611679f,0.024626558646559715f,0.0033575037959963083f,0.001979856053367257f,0.09604639559984207f,0.011991468258202076f,-0.006021655630320311f,-0.006009194068610668f,-0.001209275797009468f,0.01703660748898983f,-0.007376255001872778f,0.010084833949804306f,0.058844007551670074f,-0.07803554832935333f,0.06061769649386406f,0.01736476831138134f,0.0010751498630270362f,-0.031046299263834953f,-0.006746724247932434f,0.0016464412910863757f,-0.21503499150276184f,-0.03524624556303024f,-0.006212748121470213f,0.04168232902884483f,0.0019027034286409616f,0.026951484382152557f,0.0010380980093032122f,0.0013953939778730273f,-0.06174027919769287f,0.013890787959098816f,-0.006959398742765188f,-0.006925149820744991f,0.03967422991991043f,-0.007992781698703766f,0.011467033065855503f,0.006069325841963291f,0.09693168103694916f,-0.06122562289237976f,-0.03317604959011078f,0.09439822286367416f,0.006526044104248285f,-0.025550197809934616f,-0.0006871449295431376f,-0.006586072966456413f,0.12431279569864273f,-0.06547858566045761f,-0.0710073783993721f,0.13691551983356476f,-0.019056444987654686f,-0.0027380480896681547f,0.0028281419072300196f,0.001333048683591187f,0.03205818310379982f,0.01854025013744831f,-0.009253880940377712f,-0.009260294027626514f,-0.012031013146042824f,-0.011610782705247402f,0.0010807608487084508f,0.0013064030790701509f,-0.2869706451892853f,-0.004794040694832802f,0.012165773659944534f,-0.007464701309800148f,0.0039795017801225185f,0.007302149198949337f,-0.0005047644954174757f,-1.2632959624170326e-05f,0.3344636559486389f,-0.0595933273434639f,0.04643746465444565f,0.012122387997806072f,-0.013780287466943264f,-0.00326717016287148f,0.00035745042259804904f,0.00024542774190194905f,-0.1699894815683365f,0.01331557147204876f,-0.00663520535454154f,-0.006662466563284397f,-0.0015396655071526766f,-0.009543768130242825f,0.015767209231853485f,-0.0014311171835288405f,-0.06879207491874695f,0.01869380660355091f,0.002471327083185315f,-0.02121216431260109f,-0.0008652749238535762f,0.014497571624815464f,-0.0025599112268537283f,0.0024335538037121296f,-0.28563445806503296f,-0.000532343634404242f,-0.0020761077757924795f,0.0025125986430794f,-0.0036753860767930746f,0.004437093157321215f,0.006829168181866407f,-4.349396112957038e-05f,0.05737340450286865f,0.0105888145044446f,-0.005286739207804203f,-0.005313192028552294f,-0.02921903133392334f,0.023640474304556847f,-0.03380701690912247f,-0.0027277281042188406f,-0.1942943036556244f,0.038527075201272964f,0.006146179977804422f,-0.04506060481071472f,0.10160165280103683f,-0.025763703510165215f,0.022248106077313423f,0.0007811484974808991f,-0.12839503586292267f,0.027306942269206047f,-0.035841453820466995f,0.008525896817445755f,0.00854810792952776f,-0.047205641865730286f,-0.0014032162725925446f,-0.005022665020078421f,-0.019505873322486877f,0.0228520929813385f,-0.011426182463765144f,-0.011456789448857307f,-0.005108605604618788f,0.006263575982302427f,-0.00637761689722538f,0.010136737488210201f,0.0038089656736701727f,-0.029216758906841278f,0.04376128315925598f,-0.01484895870089531f,-0.05093996226787567f,0.010225794278085232f,-0.008965931832790375f,-0.008332534693181515f,0.10439570993185043f,-0.039599064737558365f,0.06398873031139374f,-0.024538207799196243f,-0.02108496055006981f,-0.021012110635638237f,0.007246979046612978f,-0.0023707025684416294f,0.07848943769931793f,0.024908361956477165f,-0.012479040771722794f,-0.01248338632285595f,-0.0462677888572216f,0.060274746268987656f,-0.12017606198787689f,0.08049838244915009f,0.11738016456365585f,0.14654697477817535f,-0.015040542930364609f,-0.13269571959972382f,-0.019879838451743126f,0.034709904342889786f,-0.07871776074171066f,0.027437888085842133f,0.1061183363199234f,0.1460970789194107f,-0.08989279717206955f,-0.056556276977062225f,-0.02631106972694397f,-0.019327007234096527f,0.013061520643532276f,-0.010493863373994827f,-0.2013251781463623f,0.0023540365509688854f,-0.0012122736079618335f,-0.0011915649520233274f,-0.0009008478955365717f,-0.0262469295412302f,0.02136407047510147f,0.02324788086116314f,-0.3661736249923706f,0.05833642929792404f,0.0014998259721323848f,-0.060228776186704636f,0.017025906592607498f,-0.004622220527380705f,-0.001530397217720747f,0.004596870858222246f,0.15485158562660217f,0.12758147716522217f,0.01526222936809063f,-0.14389793574810028f,0.009074901230633259f,-0.016893533989787102f,-0.012413332238793373f,2.2725635062670335e-05f,-0.027798118069767952f,0.06370271742343903f,-0.03183958679437637f,-0.03183659911155701f,0.014214612543582916f,0.0034301127307116985f,0.0032066188286989927f,-0.0024606690276414156f,-0.18507176637649536f,0.025372127071022987f,0.033361997455358505f,-0.05891582369804382f,0.017377609387040138f,-0.002004070207476616f,0.0038637907709926367f,0.0058957478031516075f,0.1426333636045456f,0.006080095190554857f,0.11176571995019913f,-0.11841049790382385f,0.003941184841096401f,0.009007412008941174f,-0.003811161033809185f,-0.0013788013020530343f,0.03141162544488907f,0.02540413849055767f,-0.012685068883001804f,-0.012686924077570438f,0.0033389637246727943f,0.005783244967460632f,0.005971319507807493f,0.002694201422855258f,0.029844937846064568f,-0.04384693130850792f,0.039212971925735474f,0.0042283860966563225f,-0.007643586955964565f,0.017703702673316002f,-0.0032470393925905228f,-0.0013013007119297981f,-0.11431373655796051f,-0.00490660360082984f,0.00037344693555496633f,0.004470900632441044f,0.016744565218687057f,-0.01858639530837536f,0.0010881623020395637f,0.0004263182054273784f,0.07205484807491302f,0.04747774079442024f,-0.02372838370501995f,-0.023730803281068802f,0.048990968614816666f,-0.0013677733950316906f,-0.014172418974339962f,0.004017870407551527f,-0.06341977417469025f,0.0544566810131073f,-0.02911742404103279f,-0.02576277405023575f,0.11971605569124222f,0.036857862025499344f,-0.0024389808531850576f,0.008685262873768806f,-0.16879041492938995f,-0.052683789283037186f,0.030468693003058434f,0.022143971174955368f,0.01622953452169895f,-0.005163492169231176f,0.007063139695674181f,0.0019904111977666616f,-0.20375807583332062f,-0.0006172567955218256f,0.0003452709352131933f,0.00034167469129897654f,-0.017709430307149887f,0.031347282230854034f,0.0033430405892431736f,0.015092941001057625f,-0.006132348440587521f,0.056860413402318954f,-0.0076944404281675816f,-0.04770231619477272f,-0.019455337896943092f,-0.0012521734461188316f,-0.0037619443610310555f,0.0017710363026708364f,-0.17227178812026978f,-0.10247809439897537f,0.12808986008167267f,-0.026970986276865005f,-0.007185123860836029f,-9.907245839713141e-05f,0.029760664328932762f,0.004921038169413805f,-0.21135634183883667f,-0.003213742980733514f,0.001579473027959466f,0.0015940847806632519f,0.039769675582647324f,-0.027383429929614067f,-0.010648618452250957f,0.008085428737103939f,0.08556131273508072f,-0.05604923143982887f,-0.01668512634932995f,0.07332219928503036f,0.05929199978709221f,-0.009165880270302296f,-0.011586945503950119f,-0.005054572597146034f,-0.06652242690324783f,-0.04412107169628143f,-0.033032603561878204f,0.07826044410467148f,0.016536064445972443f,-0.0057162148877978325f,0.017140507698059082f,-0.0011273566633462906f,-0.14207397401332855f,0.005743251647800207f,-0.002869961317628622f,-0.00288335420191288f,0.012272924184799194f,-0.0051130857318639755f,0.010041647590696812f,-0.0031346501782536507f,-0.20594757795333862f,-0.036161623895168304f,0.0056373123079538345f,0.03023725375533104f,-0.004720001947134733f,0.02124398574233055f,0.0021553009282797575f,0.00575528759509325f,0.12591610848903656f,-0.04155227914452553f,-0.03388134762644768f,0.07441342622041702f,-0.010645337402820587f,0.013282054103910923f,-0.00449890224263072f,0.00018616819579619914f,0.23945820331573486f,0.021955110132694244f,-0.010949486866593361f,-0.01096432190388441f,0.002186486031860113f,0.020860062912106514f,0.005532700102776289f,0.008817164227366447f,-0.005553852766752243f,0.04369422793388367f,0.07513949275016785f,-0.11849150061607361f,0.02877570502460003f,-0.02581843174993992f,-0.0024963784962892532f,-0.001228060806170106f,-0.029747484251856804f,0.03314410150051117f,0.02452528476715088f,-0.05754604563117027f,0.03365344926714897f,-0.028748175129294395f,-0.018089352175593376f,-0.0008511586929671466f,-0.22968345880508423f,0.011569762602448463f,-0.005791237112134695f,-0.0058112372644245625f,-0.024634571745991707f,0.03740990534424782f,0.002410071436315775f,-0.07251552492380142f,-0.15891033411026f,-0.018571725115180016f,0.02517932839691639f,-0.0066522578708827496f,0.010488109663128853f,-0.0683295801281929f,0.009960554540157318f,0.020121268928050995f,-0.3522043228149414f,0.018876584246754646f,-0.0010722383158281446f,-0.017861057072877884f,0.043759286403656006f,-0.0011587642366066575f,0.0014873223844915628f,0.00048061320558190346f,-0.01633167825639248f,0.03616930916905403f,-0.01810779795050621f,-0.018087897449731827f,0.01865651272237301f,-0.012358004227280617f,0.015866324305534363f,-0.007407476659864187f,-0.02516186609864235f,0.05722327157855034f,-0.0020329721737653017f,-0.05550443381071091f,0.020604722201824188f,0.006515092682093382f,0.0027905949391424656f,0.0031307379249483347f,-0.12602631747722626f,0.12386056035757065f,-0.059684645384550095f,-0.06478278338909149f,-0.00016096557374112308f,0.044002942740917206f,-0.025127921253442764f,0.0007566524436697364f,0.03978366032242775f,0.0348440557718277f,-0.017404181882739067f,-0.017402945086359978f,0.010516875423491001f,-0.08940452337265015f,0.0028036702424287796f,0.010767892003059387f,0.0287840086966753f,0.004327611066401005f,0.009649029932916164f,-0.013930624350905418f,0.03731473535299301f,-0.023005830124020576f,-0.005818978417664766f,-0.00038698469870723784f,0.07688593864440918f,0.05583111569285393f,-0.009548624977469444f,-0.04685976356267929f,0.019860247150063515f,0.033406566828489304f,0.005243844818323851f,0.001555591356009245f,0.07766290009021759f,0.026676775887608528f,-0.013344747014343739f,-0.013322794809937477f,0.0072362395003438f,-0.005893611814826727f,0.0057146670296788216f,0.0035766777582466602f,0.12673485279083252f,0.015227828174829483f,0.04596990719437599f,-0.06176353991031647f,-0.009897161275148392f,0.008592010475695133f,-0.0013229688629508018f,0.0011518684914335608f,0.02089528925716877f,0.10632200539112091f,-0.020345840603113174f,-0.08841078728437424f,-0.015470423735678196f,-0.00022531088325195014f,0.0013085177633911371f,0.00023145301383920014f,0.10930448770523071f,0.067625030875206f,-0.03380102291703224f,-0.03380219265818596f,0.02266887202858925f,-0.04154960438609123f,0.003063417039811611f,0.005538148805499077f,0.016032420098781586f,-0.051406294107437134f,0.13111987709999084f,-0.08028537780046463f,-0.012838291004300117f,0.022112848237156868f,-0.0034144676756113768f,-0.0034370957873761654f,0.1683282107114792f,-0.09664929658174515f,0.09827408939599991f,-0.0016053165309131145f,0.010479140095412731f,0.004354636184871197f,-0.004183614626526833f,0.0032611326314508915f,-0.24233639240264893f,0.015675783157348633f,-0.007846571505069733f,-0.00782837439328432f,0.011085924692451954f,-0.0178231168538332f,-0.009068166837096214f,0.0031160516664385796f,-0.04061991721391678f,0.04586682841181755f,-0.05658123642206192f,0.0115016745403409f,-0.0030189594253897667f,-0.002402836224064231f,0.004605583380907774f,0.03680915758013725f,0.13499286770820618f,0.16172835230827332f,-0.12895165383815765f,-0.03283289447426796f,-0.00961923599243164f,-0.003991378005594015f,-0.007788771763443947f,0.001170452102087438f,0.15326033532619476f,0.04840974137187004f,-0.024189816787838936f,-0.024210603907704353f,-0.025525439530611038f,0.017164021730422974f,-0.003999621607363224f,0.0099015012383461f,-0.07350493967533112f,-0.0033513535745441914f,-0.05967763438820839f,0.060754116624593735f,-0.0035210030619055033f,0.011268917471170425f,-0.0005795699544250965f,0.00308950780890882f,-0.0026554507203400135f,-0.02360207587480545f,-0.010454455390572548f,0.032834719866514206f,0.016313819214701653f,0.018422376364469528f,0.001939979731105268f,0.0017039843369275331f,-0.00571794668212533f,0.0082862488925457f,-0.004140194039791822f,-0.004144235514104366f,-0.014227120205760002f,0.010998803190886974f,-0.001861001830548048f,0.0076368716545403f,-0.1906486600637436f,-0.01734509877860546f,0.038818906992673874f,-0.021662434563040733f,-0.01096939854323864f,0.008167543448507786f,0.004719969350844622f,0.0030583636835217476f,-0.036843933165073395f,0.08763208240270615f,-0.04524868726730347f,-0.04267947003245354f,0.00265847728587687f,0.00789943803101778f,-0.0026039069052785635f,-0.0011733260471373796f,0.03594449907541275f,0.029701899737119675f,-0.014851455576717854f,-0.014853165484964848f,-0.03082326613366604f,0.006411934271454811f,-0.004519264679402113f,0.003460736246779561f,0.04707847908139229f,-0.07920417189598083f,0.05981767177581787f,0.019099436700344086f,0.026823315769433975f,-0.013292805291712284f,-0.001337756053544581f,0.0007539783837273717f,-0.021879302337765694f,-0.10727384686470032f,0.10854172706604004f,-0.0011424242984503508f,0.009767808951437473f,0.027467213571071625f,0.011097438633441925f,0.0004179812385700643f,0.03090207651257515f,0.019016264006495476f,-0.009485221467912197f,-0.009501115418970585f,-0.02123415842652321f,-0.010081762447953224f,-0.007575764786452055f,0.012112613767385483f,-0.09244775027036667f,0.030521513894200325f,-0.02245686948299408f,-0.007920246571302414f,-0.012488821521401405f,0.005868743173778057f,-0.0030491393990814686f,0.0062831430695950985f,-0.08044817298650742f,-0.0011583119630813599f,0.01744137704372406f,-0.016631603240966797f,0.010648558847606182f,0.006744053680449724f,0.005900747608393431f,-0.0006303845439106226f,0.11488877981901169f,0.012903692200779915f,-0.006465981248766184f,-0.006463228724896908f,0.0158418957144022f,-0.04996345937252045f,0.005509685724973679f,0.014728862792253494f,0.023481491953134537f,-0.017166096717119217f,-0.025224782526493073f,0.04227569326758385f,0.04927360266447067f,-0.029620490968227386f,0.007677243556827307f,-0.00029028847347944975f,0.004164921585470438f,0.013100915588438511f,-0.03874253109097481f,0.025912592187523842f,0.06887447834014893f,-0.043200984597206116f,0.00674251327291131f,-0.0033013327047228813f,0.05680068954825401f,0.025683121755719185f,-0.01283282320946455f,-0.012828024104237556f,0.0059695676900446415f,-0.007140479050576687f,0.005678302608430386f,0.004467342048883438f,0.058775100857019424f,-0.05137691646814346f,0.07630432397127151f,-0.024928806349635124f,-0.04243454337120056f,0.001041637733578682f,-0.008027364499866962f,0.0022957834880799055f,-0.07201436161994934f,-0.03816140443086624f,0.053582675755023956f,-0.01541338860988617f,0.06031191349029541f,0.014379722066223621f,0.006635179277509451f,0.0006596510647796094f,-0.1989550143480301f,0.0066202739253640175f,-0.0032958541996777058f,-0.0033068705815821886f,-0.019316215068101883f,-0.0038656038232147694f,-0.008793985471129417f,0.019546959549188614f,-0.055444180965423584f,-0.04043125361204147f,0.042109016329050064f,-0.0018132937839254737f,-0.021270623430609703f,-0.010856490582227707f,-0.0055784196592867374f,-0.006102693732827902f,0.06941059231758118f,-0.01305419486016035f,0.00546595174819231f,0.00754350982606411f,0.0086312061175704f,0.027351073920726776f,0.0039539635181427f,0.003316768677905202f,-0.10198624432086945f,0.0037954270374029875f,-0.001870370702818036f,-0.0018869591876864433f,-0.013336213305592537f,-0.01633194088935852f,0.00018003534933086485f,-0.00602828711271286f,0.019534869119524956f,-0.047445278614759445f,0.06432279199361801f,-0.016792410984635353f,-0.010120834223926067f,0.011301916092634201f,0.000588598137255758f,-0.0001435244339518249f,0.02663234993815422f,0.0002727116225287318f,0.03164543956518173f,-0.031877726316452026f,0.014967106282711029f,-0.04104260355234146f,0.0031428104266524315f,-0.00024086647317744792f,-0.027058715000748634f,0.014163248240947723f,-0.007069048471748829f,-0.0070768180303275585f,0.024016762152314186f,0.0070847705937922f,0.00517653301358223f,-0.013924486935138702f,0.07058122754096985f,-0.012124053202569485f,0.043955691158771515f,-0.03184065595269203f,-0.007392451167106628f,0.0033957194536924362f,0.004230603575706482f,0.001173815573565662f,0.051883939653635025f,-0.01748049072921276f,0.05064072087407112f,-0.033331550657749176f,-0.056592464447021484f,-0.002027907408773899f,2.388732355029788e-05f,0.00023299142776522785f,0.0034496034495532513f,0.008403493091464043f,-0.004193127155303955f,-0.004215200897306204f,0.022170929238200188f,-0.016191523522138596f,-0.005815424490720034f,-0.0012038308195769787f,0.046920809894800186f,0.024297382682561874f,0.019428757950663567f,-0.043707262724637985f,-0.009438238106667995f,0.0022730915807187557f,-0.0007469299016520381f,0.0016412807162851095f,0.00571979908272624f,0.051747750490903854f,-0.05755726248025894f,0.005490511655807495f,-0.028045373037457466f,0.006331740412861109f,-0.004394511226564646f,0.00024805113207548857f,0.011241335421800613f,0.00014841087977401912f,-8.284734940389171e-05f,-7.863037899369374e-05f,0.0044275554828345776f,0.04691790044307709f,0.002201726892963052f,0.015203949995338917f,-0.045233335345983505f,-0.02675144374370575f,-0.025005703791975975f,0.052028123289346695f,0.036344002932310104f,0.019849427044391632f,0.0010004111099988222f,-0.0019562174566090107f,0.12277677655220032f,0.037443339824676514f,-0.07001061737537384f,0.03240378201007843f,-0.008392179384827614f,0.04816018417477608f,-0.012922865338623524f,-0.0018663447117432952f,-0.010056623257696629f,0.01427572499960661f,-0.007110084407031536f,-0.007126720622181892f,0.0036163353361189365f,-0.006006637122482061f,-0.012430168688297272f,0.017799770459532738f,-0.03031226620078087f,-0.04580220952630043f,0.03251343220472336f,0.013236166909337044f,-0.010486015118658543f,-0.0024823409039527178f,-0.007081098388880491f,0.013796553947031498f,-0.06570208072662354f,-0.03620074316859245f,-0.0014190863585099578f,0.03786129876971245f,0.03162567317485809f,0.007401295006275177f,-0.0005967803299427032f,0.002799771726131439f,-0.018246637657284737f,0.016348153352737427f,-0.008176666684448719f,-0.008176335133612156f,-0.018045620992779732f,0.006981596350669861f,0.0017891523893922567f,-0.0062248725444078445f,-0.08778375387191772f,-0.0409509539604187f,-0.0012441544095054269f,0.04223167896270752f,0.02163175866007805f,-0.005906570702791214f,0.004413594491779804f,0.0005708645912818611f,-0.17445889115333557f,-0.0007643375429324806f,-0.015596821904182434f,0.01666582003235817f,0.030748264864087105f,-0.006804580334573984f,-0.003860584693029523f,-0.0006586936069652438f,0.21321995556354523f,0.024546371772885323f,-0.012275557033717632f,-0.012262911535799503f,0.011560998857021332f,0.015981178730726242f,0.008310516364872456f,-0.0058283028192818165f,-0.027175089344382286f,-0.007177623454481363f,0.039797618985176086f,-0.033071376383304596f,-0.01683914288878441f,0.009060142561793327f,-0.0015278365463018417f,0.004139818251132965f,-0.06842357665300369f,0.017318326979875565f,-0.01569604128599167f,-0.0016889018006622791f,-0.05264480039477348f,0.010341554880142212f,0.0022915545850992203f,-0.0010335036786273122f,0.05291619524359703f,0.001392867648974061f,-0.0006931629031896591f,-0.0006703123217448592f,-0.003274682443588972f,0.007373463362455368f,0.00603512953966856f,-0.010068842209875584f,-0.06106746569275856f,-0.0021379394456744194f,-0.014601864852011204f,0.016423821449279785f,-0.018244801089167595f,-0.004572401754558086f,0.0023492267355322838f,0.00023659593716729432f,0.025389209389686584f,-0.02476007305085659f,-0.0025955874007195234f,0.026687903329730034f,-0.06184975430369377f,0.0018215809250250459f,-0.020192906260490417f,0.0030694021843373775f,-0.09767190366983414f,0.012663941830396652f,-0.006317947991192341f,-0.006310658995062113f,-0.001050414051860571f,-0.0032355342991650105f,0.004317176528275013f,-0.011305378749966621f,-0.13062800467014313f,-0.005122417584061623f,0.07394200563430786f,-0.06887026876211166f,0.007749779615551233f,0.002851292723789811f,0.0019590735901147127f,-0.0032097846269607544f,0.06099000573158264f,-0.009566825814545155f,0.04433998465538025f,-0.03472689539194107f,0.05419478192925453f,-0.019795870408415794f,0.007089583203196526f,-0.0012236512266099453f,-0.17309433221817017f,0.0199448075145483f,-0.00995964091271162f,-0.009985187090933323f,-0.05020066350698471f,0.019831981509923935f,-0.0066339364275336266f,0.007902901619672775f,0.035796068608760834f,0.04506295919418335f,-0.010727031156420708f,-0.0342925526201725f,-0.00950551312416792f,-0.004936513025313616f,0.0012470177607610822f,-0.00394504563882947f,0.11800112575292587f,0.039705000817775726f,-0.007876948453485966f,-0.03223056718707085f,-0.011421014554798603f,0.015167509205639362f,-0.00021229461708571762f,-0.0006149187101982534f,0.07782398164272308f,0.016317075118422508f,-0.00817150715738535f,-0.008133837953209877f,-0.018335135653614998f,-0.019098550081253052f,-0.017604032531380653f,0.002625764347612858f,0.033371999859809875f,0.05298054590821266f,-0.001804724452085793f,-0.052265577018260956f,-0.0006942551699467003f,-0.017575640231370926f,-0.001739146769978106f,0.007667201105505228f,-0.08325561881065369f,0.015472257509827614f,0.009523020125925541f,-0.025343600660562515f,-0.03642906993627548f,0.006851754616945982f,-0.001987425610423088f,0.001470957649871707f,0.04391489923000336f,0.011321150697767735f,-0.005677930545061827f,-0.005654247011989355f,-0.01988685317337513f,0.013180332258343697f,-0.015164037235081196f,-2.885217145376373e-05f,0.01870650053024292f,-0.03407686948776245f,0.06631304323673248f,-0.03258298337459564f,0.009170729666948318f,0.021867141127586365f,0.0012836080277338624f,-0.006835836451500654f,-0.07699143886566162f,0.01644059829413891f,-0.0013534859754145145f,-0.015081766061484814f,0.060335803776979446f,0.041456639766693115f,0.012502564117312431f,0.001816485309973359f,0.05777406692504883f,0.015444204211235046f,-0.00773392291739583f,-0.007691122125834227f,-0.014594361186027527f,-0.005599002819508314f,-0.004854694474488497f,0.024925120174884796f,0.0019275068771094084f,-0.0013041376369073987f,-0.0088124368339777f,0.01015895139425993f,-0.036436885595321655f,0.010089126415550709f,-0.007512354291975498f,0.005838631186634302f,0.043490368872880936f,0.025582056492567062f,-0.02029358595609665f,-0.005573366768658161f,-0.031089814379811287f,0.00043783552246168256f,0.004217681474983692f,-0.000696716655511409f,-0.07762259244918823f,0.004697165451943874f,-0.002337465761229396f,-0.0023628482595086098f,-0.006922441069036722f,0.01953701674938202f,-0.0031592659652233124f,0.001616113935597241f,0.027275264263153076f,-0.02783435769379139f,-0.004379800986498594f,0.03232842683792114f,-7.393120904453099e-05f,-0.0028787609189748764f,0.0015023407759144902f,0.0009098776499740779f,-0.1213880106806755f,-0.04589173197746277f,0.0036037631798535585f,0.0423213355243206f,0.019106106832623482f,0.0037959858309477568f,0.002455370733514428f,0.000818734522908926f,0.056908540427684784f,0.01137951947748661f,-0.005675621796399355f,-0.00567402271553874f,-0.019995050504803658f,0.004415543749928474f,-0.004233970772475004f,-0.003927309066057205f,0.07732008397579193f,0.03605176880955696f,-0.01409982331097126f,-0.02195008471608162f,0.006095941178500652f,0.012576199136674404f,0.0017884137341752648f,-7.034980808384717e-05f,0.01015016995370388f,-0.023386618122458458f,0.040201589465141296f,-0.016842691227793694f,0.007075989618897438f,0.01788441836833954f,0.005364054813981056f,0.002554784994572401f,0.019833533093333244f,0.011406926438212395f,-0.005721105262637138f,-0.0057173618115484715f,-0.014277574606239796f,0.01945408433675766f,0.010875963605940342f,0.005640272982418537f,-0.07415316998958588f,0.03836202621459961f,-0.0021192405838519335f,-0.03632116690278053f,-0.02869868464767933f,0.01517315674573183f,-0.0013409939128905535f,0.005509640090167522f,0.13311952352523804f,0.03256193920969963f,-0.021992577239871025f,-0.010585435666143894f,0.008867363445460796f,0.015641435980796814f,-0.0012430230854079127f,0.00010750121145974845f,-0.015380922704935074f,0.01769895851612091f,-0.008823757991194725f,-0.008824638091027737f,-0.02248900942504406f,-0.0051757777109742165f,0.003563752630725503f,-0.008910105563700199f,0.023427266627550125f,-0.017406770959496498f,-0.01033941749483347f,0.027815764769911766f,-0.002929002745077014f,0.010465417988598347f,-0.004897082224488258f,-0.006750852335244417f,-0.04834507778286934f,-0.021679768338799477f,-0.006326678209006786f,0.02777654118835926f,-0.004237514454871416f,-0.02727154828608036f,0.01211594045162201f,-0.0015964291524142027f,-0.13792157173156738f,0.006569083780050278f,-0.00328721571713686f,-0.0033052414655685425f,0.004094155039638281f,0.0027336401399224997f,0.01180688850581646f,0.001077415538020432f,0.03375457599759102f,-0.045714303851127625f,0.04188484698534012f,0.0038473361637443304f,0.013707976788282394f,-0.01485253032296896f,0.0017784630181267858f,-0.0016906489618122578f,0.03799368441104889f,-0.0690290778875351f,0.07252375781536102f,-0.0030507587362080812f,0.043194372206926346f,0.009674989618360996f,0.005904555786401033f,0.001572644803673029f,-0.10244441777467728f,0.011786142364144325f,-0.005859598983079195f,-0.005861256737262011f,-0.07080142945051193f,-0.025429466739296913f,0.007636004593223333f,0.012580430135130882f,-0.011336897499859333f,-0.006137234158813953f,0.01253560185432434f,-0.0062670581974089146f,-0.05761419236660004f,0.009323176927864552f,-0.011543681845068932f,-0.00789951253682375f,0.007488703355193138f,-0.029471304267644882f,0.028664108365774155f,0.0007824372150935233f,0.0010228027822449803f,0.021050790324807167f,0.02545444294810295f,0.0014668768271803856f,-0.1616712063550949f,0.0022418026346713305f,-0.0011275438591837883f,-0.0011236502323299646f,-0.013161441311240196f,0.009059990756213665f,-0.014808673411607742f,0.0039008327294141054f,0.010684020817279816f,0.010230009444057941f,-0.020610041916370392f,0.010308938100934029f,-0.016447491943836212f,0.0152512202039361f,0.0007014535367488861f,0.007904903031885624f,-0.09930247068405151f,0.029095545411109924f,-0.006166676990687847f,-0.023100629448890686f,-0.020770208910107613f,-0.002724494319409132f,-0.0056752716191112995f,0.00029487331630662084f,0.07347039878368378f,0.030651012435555458f,-0.015306076966226101f,-0.015330447815358639f,-0.010344048030674458f,-0.014008563943207264f,-0.009944521822035313f,0.001798622077330947f,-0.005226511508226395f,0.04568808525800705f,-0.013125767931342125f,-0.03244071826338768f,0.0044046794064342976f,-0.0012696211924776435f,-0.0014844820834696293f,0.003613552777096629f,-0.06827943027019501f,0.004471306223422289f,0.035889703780412674f,-0.040603987872600555f,0.004695413634181023f,0.00564691424369812f,0.0028657300863415003f,0.00040943751810118556f,-0.042277995496988297f,0.017174450680613518f,-0.008573531173169613f,-0.00858023390173912f,-0.0009398867259733379f,0.03946521505713463f,-0.003446368733420968f,0.008147685788571835f,0.027751678600907326f,-0.12358928471803665f,0.13393741846084595f,-0.01022467203438282f,-0.02443503960967064f,0.017961010336875916f,-0.004985033068805933f,0.0028395121917128563f,-0.03041897341609001f,-0.09230844676494598f,0.131129652261734f,-0.03899747505784035f,0.007213535252958536f,0.0068673426285386086f,0.0004693068331107497f,0.0028426814824342728f,0.02999732829630375f,0.017740080133080482f,-0.008847021497786045f,-0.008863795548677444f,-0.04535181447863579f,0.010093140415847301f,-0.00015360229008365422f,0.0042070248164236546f,0.019404197111725807f,0.00032158091198652983f,-0.011634787544608116f,0.011318588629364967f,0.004953110124915838f,-0.01083127036690712f,-0.0029649771749973297f,0.0037346454337239265f,0.04031345620751381f,0.009423828683793545f,-0.0456879697740078f,0.03622370958328247f,-0.01464792899787426f,-0.016346313059329987f,-0.004369903355836868f,0.0011324932565912604f,0.08026663959026337f,0.016445033252239227f,-0.008234028704464436f,-0.008225466124713421f,-0.005861813668161631f,-0.017011234536767006f,-0.002751045860350132f,0.010860205627977848f,-0.044781725853681564f,0.06687720119953156f,-0.04854601249098778f,-0.018772704526782036f,-0.014257138594985008f,-0.00029536260990425944f,-0.00798805058002472f,-0.0005178697174414992f,0.0933016911149025f,0.024693721905350685f,-0.056300099939107895f,0.03108043409883976f,-0.0651816800236702f,-0.02530886046588421f,-0.025839466601610184f,-0.002311252523213625f,-0.14315272867679596f,0.0073929280042648315f,-0.003712698817253113f,-0.0037179572973400354f,-0.001769771333783865f,-0.04550859332084656f,-0.0009754908387549222f,-0.0038074066396802664f,-0.02178478240966797f,-0.0147907929494977f,-0.012568894773721695f,0.02738255262374878f,-0.03020215593278408f,-0.0034007704816758633f,-0.0016639487585052848f,0.005743290297687054f,0.1160329058766365f,-0.012647008523344994f,0.009582136757671833f,0.0028134535532444715f,0.01235844474285841f,0.030047623440623283f,0.012155805714428425f,0.004167784936726093f,0.11414706707000732f,0.013157828710973263f,-0.006571560632437468f,-0.006591259501874447f,-0.01306235883384943f,0.006611136253923178f,0.00010073106386698782f,0.0006595178856514394f,-0.12465368211269379f,0.048574741929769516f,-0.031897831708192825f,-0.016496725380420685f,0.00026351161068305373f,-0.006281019654124975f,0.007359969429671764f,-0.002827555639669299f,-0.07511946558952332f,-0.004572077188640833f,-0.0021791085600852966f,0.006753852590918541f,0.016218198463320732f,-0.007969865575432777f,0.0006915517151355743f,0.0009160504559986293f,-0.14450310170650482f,0.017077138647437096f,-0.008564088493585587f,-0.008527715690433979f,0.02319331467151642f,0.020641544833779335f,0.0013202190166339278f,0.008265996351838112f,-0.05630364269018173f,-0.07350877672433853f,0.10768058896064758f,-0.034324005246162415f,-0.002563446294516325f,0.014437742531299591f,0.000896702753379941f,-0.001893478911370039f,0.09497449547052383f,-0.04881959781050682f,0.07487589120864868f,-0.025687498971819878f,0.012616807594895363f,0.015365509316325188f,0.001074540545232594f,0.00279459566809237f,-0.1718577742576599f,0.0014454976189881563f,-0.0006955158314667642f,-0.000705093378201127f,-0.03028021939098835f,-0.03515278920531273f,-0.004847428295761347f,0.01023833267390728f,-0.015431712381541729f,-0.014465189538896084f,-0.029993660748004913f,0.04356399551033974f,-0.020702945068478584f,-0.014141062274575233f,-0.004076680168509483f,0.00016870796389412135f,-0.06734270602464676f,-0.04016042873263359f,0.011421558447182178f,0.02798612229526043f,0.006960956379771233f,0.027945049107074738f,-0.0043991124257445335f,0.00430664699524641f,-0.08161868900060654f,0.01061391830444336f,-0.005308529362082481f,-0.0053249625489115715f,0.014579483307898045f,0.009116949513554573f,-0.0022872467525303364f,0.00740734301507473f,-0.05177467316389084f,0.04796648770570755f,-0.0252918042242527f,-0.022750575095415115f,-0.01849050633609295f,0.013742664828896523f,0.0005311825661920011f,-0.0030887669418007135f,-0.04203604534268379f,0.023940332233905792f,-0.04198848456144333f,0.018228303641080856f,-0.030696211382746696f,0.023081541061401367f,-0.00913860835134983f,0.0005595609545707703f,0.18799498677253723f,0.012233439832925797f,-0.006089060567319393f,-0.006098470184952021f,0.007595118135213852f,0.016253294423222542f,-0.008760970085859299f,0.010995060205459595f,-0.0019199054222553968f,0.01063062809407711f,-0.0009208105620928109f,-0.0097949905321002f,-0.004882518667727709f,-0.016154704615473747f,0.002566666342318058f,0.005554807372391224f,0.016282597556710243f,-0.006649462040513754f,0.007056724280118942f,-0.00010869325342355296f,-0.0043342225253582f,0.02147446945309639f,7.286396430572495e-05f,0.002032018266618252f,-0.11217319220304489f,0.0064090038649737835f,-0.0032257267739623785f,-0.0031902468763291836f,0.009570633061230183f,-0.006462056189775467f,-0.0012867104960605502f,-0.023661425337195396f,-0.017917009070515633f,-0.019136756658554077f,0.04594326391816139f,-0.026849620044231415f,0.014696980826556683f,-0.004737359005957842f,0.0036511735524982214f,-0.0036825735587626696f,0.024259502068161964f,-0.014763959683477879f,0.061875004321336746f,-0.047855399549007416f,0.0017933498602360487f,-0.013080762699246407f,0.0008461151155643165f,0.0006642651278525591f,0.03921428322792053f,0.02210909128189087f,-0.011052326299250126f,-0.011053625494241714f,-0.03021896816790104f,-0.001973524456843734f,-0.005156880710273981f,0.012910943478345871f,-0.03277759626507759f,-0.013498795218765736f,0.023570913821458817f,-0.010433613322675228f,0.015093039721250534f,-0.005151047836989164f,-0.0015185031807050109f,-0.0003617070324253291f,0.026987284421920776f,-0.07210184633731842f,0.055934805423021317f,0.01595611311495304f,0.037723150104284286f,-0.015037649311125278f,-0.0012953928671777248f,0.001252024550922215f,-0.055609334260225296f,0.010400375351309776f,-0.00519930012524128f,-0.005211695097386837f,0.00046170002315193415f,-0.002174551598727703f,-0.010842250660061836f,0.009256748482584953f,0.011172524653375149f,-0.0401095412671566f,0.00023176120885182172f,0.03985638543963432f,0.008569695986807346f,-0.0007717423723079264f,-0.0019196073990315199f,0.005212041549384594f,-0.04246130958199501f,-0.007228106260299683f,0.0005268073291517794f,0.006730567663908005f,0.021942270919680595f,-0.004868675488978624f,-0.0037812343798577785f,0.0014312129933387041f,0.08996079862117767f,0.02139618806540966f,-0.010700724087655544f,-0.010695204138755798f,-0.04062346741557121f,0.0024673256557434797f,0.022384177893400192f,0.004048496019095182f,-0.1918226033449173f,0.11427846550941467f,-0.06318359822034836f,-0.05130951851606369f,-0.027582772076129913f,-0.016224903985857964f,0.0133031215518713f,-0.007904690690338612f,0.23472867906093597f,0.08800366520881653f,-0.07731622457504272f,-0.010594609193503857f,-0.021644527092576027f,0.00890862476080656f,0.00021996092982590199f,0.0022044593933969736f,-0.08174747228622437f,0.005200599320232868f,-0.002611711388453841f,-0.002606319496408105f,-0.026211028918623924f,0.015114068053662777f,0.0129158990457654f,0.004573614802211523f,-0.06347957998514175f,-0.19692464172840118f,0.21162301301956177f,-0.014929724857211113f,-0.02693818137049675f,0.020715348422527313f,0.0026637776754796505f,-0.0003820972051471472f,-0.012882625684142113f,-0.03924525901675224f,0.08248590677976608f,-0.043718401342630386f,-0.03951513022184372f,0.017472757026553154f,0.002410668646916747f,0.0016189604066312313f,0.029312381520867348f,0.014241322875022888f,-0.007151307538151741f,-0.007140276487916708f,0.013002840802073479f,0.011465315707027912f,-0.009540345519781113f,-0.0006795913795940578f,-0.08216837793588638f,0.060857534408569336f,-0.0404171422123909f,-0.020390616729855537f,0.029570546001195908f,-0.004202983807772398f,-0.0030711048748344183f,0.0016777589917182922f,-0.03311573714017868f,0.030918490141630173f,0.001293691573664546f,-0.032740913331508636f,-0.02654305286705494f,0.009049619548022747f,-0.0049976021982729435f,-0.0017949294997379184f,-0.07946360111236572f,0.009881329722702503f,-0.004929907154291868f,-0.004913307726383209f,0.013874968513846397f,0.06724073737859726f,-0.010655238293111324f,0.005107181612402201f,-0.21033602952957153f,-0.006975103169679642f,-0.013450403697788715f,0.020271126180887222f,0.009710271842777729f,0.04049370065331459f,0.005556041374802589f,-0.0035483702085912228f,-0.04534038528800011f,0.003840951481834054f,-0.0012316473294049501f,-0.002488931640982628f,-0.04070526361465454f,-0.03302605077624321f,0.008348476141691208f,-0.0015757879009470344f,-0.1619367003440857f,0.002628176473081112f,-0.0013005323708057404f,-0.001328627928160131f,0.024477917701005936f,-0.0045211464166641235f,0.005424524657428265f,-0.0018045909237116575f,-0.027950644493103027f,-0.009886960498988628f,0.01972353644669056f,-0.009625714272260666f,-0.003216036129742861f,0.027889857068657875f,0.00409986125305295f,0.008212605491280556f,-0.10517682880163193f,0.002459264826029539f,0.004364578519016504f,-0.0067048994824290276f,0.004857178777456284f,-0.007547755725681782f,0.00909036211669445f,-0.0020968252792954445f,0.11807243525981903f,0.010048151016235352f,-0.005004594102501869f,-0.0050157844088971615f,0.010012284852564335f,-0.011572579853236675f,0.0012451851507648826f,-0.026017969474196434f,-0.060615457594394684f,-0.08569831401109695f,0.05062834918498993f,0.035026196390390396f,0.03099398873746395f,0.0005045087309554219f,0.0024836657103151083f,-0.012804485857486725f,0.07631023973226547f,-0.03531309589743614f,0.07139316946268082f,-0.03609450161457062f,0.011476618237793446f,-0.006174755282700062f,-0.0044648009352386f,0.0028017060831189156f,-0.12076159566640854f,0.01679525338113308f,-0.008361552841961384f,-0.008396124467253685f,0.012256624177098274f,-0.00487477844581008f,-0.00953644048422575f,-0.007742345333099365f,0.15809176862239838f,-0.11227718740701675f,0.02859187312424183f,0.08368384838104248f,2.193766886193771e-05f,0.01210109330713749f,0.0007856469601392746f,0.006134395021945238f,-0.13662691414356232f,-0.07289929687976837f,0.013172727078199387f,0.05976531282067299f,-0.014260297641158104f,0.02006123587489128f,-0.004060117062181234f,0.0013268928742036223f,0.003848898457363248f,0.01216186210513115f,-0.00605821842327714f,-0.006064846180379391f,-0.04241088032722473f,0.01344643160700798f,0.005032196640968323f,-0.0067687551490962505f,-0.007600628305226564f,0.008806396275758743f,0.07861851155757904f,-0.0874195545911789f,-0.01521643903106451f,0.02202725224196911f,0.002037585945799947f,0.005816627759486437f,0.017613621428608894f,0.010736336000263691f,0.03328196704387665f,-0.04358181729912758f,-0.014664031565189362f,-0.007932893000543118f,0.000983881065621972f,-0.0003013968525920063f,-0.010896677151322365f,0.017565011978149414f,-0.008770981803536415f,-0.00877617672085762f,-0.05109167844057083f,0.013421899639070034f,0.012693369761109352f,-0.004214510787278414f,0.04584972932934761f,0.06911355257034302f,0.0008173813694156706f,-0.07044427841901779f,-0.017679236829280853f,0.008738288655877113f,-0.0009904124308377504f,-0.0039217243902385235f,-0.014145822264254093f,0.005319343879818916f,0.027673672884702682f,-0.032943688333034515f,-0.03484716638922691f,0.04569898545742035f,0.002973460126668215f,0.0012498480500653386f,-0.1354963332414627f,0.012634794227778912f,-0.006334348581731319f,-0.006308403797447681f,-0.02219216898083687f,0.012153666466474533f,0.0008325902745127678f,-0.006806364748626947f,-0.14362554252147675f,0.026881277561187744f,-0.07246172428131104f,0.04528512433171272f,0.007183182518929243f,0.003568114712834358f,-0.0035354678984731436f,0.00016175802738871425f,0.09624224156141281f,0.03987916558980942f,-0.027144882827997208f,-0.012876360677182674f,0.014083131216466427f,-0.009161131456494331f,0.0008446171996183693f,-0.0006343147251754999f,-0.1538124829530716f,0.008643625304102898f,-0.0043027992360293865f,-0.004313868470489979f,0.041913215070962906f,0.016020936891436577f,0.0009665230754762888f,0.0016884974902495742f,0.03939348831772804f,-0.054000094532966614f,0.12811847031116486f,-0.07443861663341522f,0.005715091247111559f,0.009186417795717716f,0.0007559973746538162f,-0.004643951542675495f,-0.12051496654748917f,-0.03813750296831131f,0.0872703343629837f,-0.04989839717745781f,0.030188225209712982f,-0.04739959537982941f,-0.0011379417264834046f,-0.004419670440256596f,0.016477219760417938f,0.006161353085190058f,-0.003104710951447487f,-0.0030698913615196943f,0.061412639915943146f,-0.048201024532318115f,-0.010095075704157352f,-0.008851108141243458f,-0.279745876789093f,0.028554951772093773f,0.002623962005600333f,-0.031121356412768364f,0.026097053661942482f,-0.03627403452992439f,0.001556840375997126f,0.006652923300862312f,-0.2134217768907547f,-0.03211846202611923f,0.02296505682170391f,0.009220178239047527f,-0.000692848872859031f,-0.021912185475230217f,0.0033874791115522385f,-5.132688966114074e-05f,-0.10929384082555771f,0.006396972108632326f,-0.003199921455234289f,-0.003181948559358716f,-0.014587046578526497f,-0.008489429019391537f,0.010563047602772713f,-0.008190794847905636f,-0.07965882867574692f,0.022942520678043365f,0.04325846955180168f,-0.06576090306043625f,-0.014764846302568913f,-0.01910739578306675f,0.005663839168846607f,-0.000629372545517981f,-0.005964060313999653f,-0.01307880599051714f,-0.0258847177028656f,0.03895726427435875f,-0.016174612566828728f,0.012293827719986439f,0.0031387442722916603f,-0.0004463638470042497f,-0.07212284207344055f,0.004257052671164274f,-0.0021098412107676268f,-0.002109646098688245f,-0.06065288931131363f,-0.003223329782485962f,0.010413140058517456f,-0.0004345784545876086f,0.07855380326509476f,0.06543461978435516f,0.023607591167092323f,-0.0891314297914505f,-0.01991509087383747f,0.013901203870773315f,-0.0014432900352403522f,-0.0006486514466814697f,0.12378544360399246f,-0.029007285833358765f,0.07186558842658997f,-0.0431709922850132f,-0.02259393408894539f,-0.035647161304950714f,-0.004150100518018007f,-0.003118228865787387f,-0.0019020605832338333f,0.012992557138204575f,-0.006461364682763815f,-0.006482645403593779f,0.05145853012800217f,0.01993413269519806f,0.0023055642377585173f,0.009939121082425117f,0.03771863877773285f,0.1238633245229721f,-0.10820546746253967f,-0.016120364889502525f,0.041610706597566605f,-0.005684992764145136f,-0.008531062863767147f,-0.007695917505770922f,-0.09720583260059357f,-0.026554062962532043f,0.022669466212391853f,0.003760271705687046f,-0.002210743259638548f,0.042145129293203354f,0.01363089494407177f,0.003549126908183098f,-0.05999289080500603f,0.005069809965789318f,-0.0025572129525244236f,-0.002560667460784316f,-0.0031611756421625614f,0.02377183362841606f,7.50000326661393e-05f,-0.008019337430596352f,-0.10379493981599808f,-0.088180772960186f,0.06608206033706665f,0.022068049758672714f,-0.01127932220697403f,0.012207908555865288f,0.0006564208888448775f,-0.004173548426479101f,-0.07465780526399612f,-0.008912533521652222f,0.01575932651758194f,-0.006834808271378279f,0.013495114631950855f,-0.02751631662249565f,-0.004101952537894249f,-0.002804851159453392f,0.06414342671632767f,0.01498157624155283f,-0.007492118049412966f,-0.007509197108447552f,0.016396915540099144f,-0.021276406943798065f,0.00978029053658247f,0.002403030637651682f,-0.16212867200374603f,0.02788996882736683f,0.029860004782676697f,-0.057345326989889145f,0.026595471426844597f,-0.03812215104699135f,-0.002577810548245907f,0.006044334266334772f,-0.2343500405550003f,-0.014239349402487278f,0.01445717178285122f,-0.00016605609562247992f,-0.0033486203756183386f,-0.0021739176008850336f,-0.0014845746336504817f,-0.002883093198761344f,-0.0995430052280426f,0.006908320356160402f,-0.0034824293106794357f,-0.003437374485656619f,-0.00875171273946762f,0.010455146431922913f,0.0010446811793372035f,-0.0038625397719442844f,0.02313346602022648f,-0.013473782688379288f,0.07131478190422058f,-0.058182891458272934f,0.0009879113640636206f,-0.013555008918046951f,-6.208763807080686e-05f,-0.0008932395721785724f,-0.04096017777919769f,0.10108986496925354f,-0.033370062708854675f,-0.06978018581867218f,-0.01598367653787136f,0.011859736405313015f,0.00567596685141325f,-0.0024613693822175264f,-0.1062941625714302f,0.012504679150879383f,-0.0062629650346934795f,-0.006268318276852369f,0.008638866245746613f,0.025022059679031372f,0.01334424875676632f,0.014352173544466496f,0.04442396014928818f,-0.04795055463910103f,0.05734818056225777f,-0.009143673814833164f,0.0005051203188486397f,-0.011416327208280563f,-0.00430251331999898f,-0.008290384896099567f,0.07884789258241653f,-0.05884973704814911f,0.03150612488389015f,0.027760455384850502f,0.006217059679329395f,-0.028613828122615814f,0.008310312405228615f,-0.002568560652434826f,-0.02716202288866043f,0.008008735254406929f,-0.003978978842496872f,-0.003999819979071617f,0.02178378216922283f,-0.013641328550875187f,-0.018378518521785736f,-0.0045896125957369804f,0.07994715124368668f,0.02144004963338375f,-0.08062919229269028f,0.058616507798433304f,0.05180051922798157f,-0.021652260795235634f,-0.001457700738683343f,-0.007040825672447681f,-0.16444343328475952f,0.018092093989253044f,-0.03872212395071983f,0.0207882858812809f,0.022000322118401527f,-0.013639821670949459f,0.0017780698835849762f,-0.0008111541974358261f,0.03454672545194626f,0.0028806293848901987f,-0.0014441257808357477f,-0.0014281722251325846f,-0.04177413880825043f,0.01654793508350849f,0.0001539442891953513f,-0.004243945237249136f,0.009972053579986095f,0.0174600537866354f,-0.18089425563812256f,0.16274847090244293f,0.01120423711836338f,0.009853732772171497f,-0.0015893061645328999f,-0.00332658807747066f,0.039276961237192154f,-0.04357898235321045f,-0.07024016976356506f,0.11398371309041977f,0.0158395878970623f,0.03418030962347984f,-0.0010263153817504644f,0.002433562884107232f,0.008569871075451374f,0.010569407604634762f,-0.005296255461871624f,-0.005292546935379505f,0.021073216572403908f,0.0021584804635494947f,-0.0025509910192340612f,-0.0016357004642486572f,-0.17212101817131042f,0.032460350543260574f,-0.03428567200899124f,0.0017197293927893043f,-0.02393920160830021f,0.017631052061915398f,-6.385544111253694e-05f,-0.0027273346204310656f,0.0920114740729332f,-0.03808203712105751f,0.02147742360830307f,0.016506405547261238f,-0.022855328395962715f,-0.004408611450344324f,0.0030891727656126022f,0.001121710753068328f,0.040937140583992004f,0.008861924521625042f,-0.004436136223375797f,-0.004431704059243202f,0.009257586672902107f,0.01000428106635809f,-0.0032363436184823513f,-0.0059372237883508205f,-0.16440768539905548f,-0.04499830678105354f,-0.0011943667195737362f,0.046214476227760315f,-0.004427238367497921f,0.012155978009104729f,-0.0009147860109806061f,-0.0016594487242400646f,0.1313035488128662f,-0.019797103479504585f,-0.033298131078481674f,0.05329832807183266f,-0.014440533705055714f,-0.05133812502026558f,0.009635274298489094f,-0.004944339860230684f,-0.0008325704839080572f,0.004846824333071709f,-0.0024380211252719164f,-0.002408599480986595f,0.010840371251106262f,0.00248183635994792f,0.0036833141930401325f,0.017472170293331146f,-0.06600227952003479f,0.04106811434030533f,-0.028316428884863853f,-0.013464470393955708f,0.022116471081972122f,-0.017700351774692535f,-0.005001511890441179f,-0.00852213241159916f,0.003311638254672289f,0.004265560302883387f,-0.04286688566207886f,0.039039622992277145f,0.004855500068515539f,0.05841784551739693f,-0.004924828186631203f,0.0006726373103447258f,-0.02219001017510891f,0.0071414620615541935f,-0.0035749063827097416f,-0.003593890927731991f,0.0712393969297409f,0.012841449119150639f,0.00610464857891202f,-0.009427906014025211f,-0.09677616506814957f,0.03255891054868698f,-0.03834526985883713f,0.005976252723485231f,0.05710241198539734f,-0.045184649527072906f,-0.005215201992541552f,-0.006276057101786137f,-0.050599124282598495f,-0.01129655260592699f,0.03146805241703987f,-0.02026228979229927f,-0.03462306410074234f,-0.001904042437672615f,0.007530150003731251f,-0.0027706350665539503f,-0.13943786919116974f,0.012139689177274704f,-0.006060651503503323f,-0.006057743914425373f,0.011324337683618069f,-0.0001702459849184379f,-0.001795052201487124f,-0.008458370342850685f,0.0712033212184906f,0.024926774203777313f,0.003562199417501688f,-0.028412405401468277f,0.028673984110355377f,0.023683086037635803f,0.00033964822068810463f,-7.819394522812217e-05f,-0.025625739246606827f,-0.0188972819596529f,0.04026244580745697f,-0.02178981341421604f,-0.005742636509239674f,-0.004420835059136152f,-0.005381191149353981f,-0.0024851681664586067f,0.11279521882534027f,0.01085162628442049f,-0.005411146674305201f,-0.005401614122092724f,-0.034048136323690414f,0.004812515340745449f,-0.006595318205654621f,-0.016633128747344017f,0.029646487906575203f,-0.14382639527320862f,0.13035210967063904f,0.01328226923942566f,0.03080766275525093f,0.02281256392598152f,-0.013500913046300411f,-0.004047035239636898f,-0.0098236333578825f,-0.04695974662899971f,0.04628439247608185f,0.0007796894060447812f,-0.045674294233322144f,0.02421705052256584f,0.003190236631780863f,0.002465337049216032f,0.0015825963346287608f,0.008695762604475021f,-0.004344842862337828f,-0.004332877229899168f,0.0424925796687603f,0.0045141857117414474f,-0.0044073197059333324f,0.005054511595517397f,-0.07609108090400696f,0.12454820424318314f,-0.0820242241024971f,-0.04253874719142914f,-0.009829961694777012f,0.000551035744138062f,0.005400793626904488f,-0.0010657450184226036f,0.09900961816310883f,0.005171145312488079f,0.017497727647423744f,-0.022653458639979362f,-0.0013510409044101834f,-0.012623107992112637f,0.0018430843483656645f,-0.0033018190879374743f,0.07702846080064774f,0.011150027625262737f,-0.005575935821980238f,-0.005571091081947088f,-0.001919014030136168f,-0.0010207428131252527f,0.004333375953137875f,0.0035206256434321404f,-0.10104911029338837f,-0.059321966022253036f,-0.048556819558143616f,0.10668843984603882f,-0.014397535473108292f,-0.000236380219575949f,0.004991027060896158f,-0.007605601102113724f,-0.044320181012153625f,0.00720817968249321f,-0.025835931301116943f,0.018720682710409164f,-0.03491941839456558f,0.005842445883899927f,0.0017504487186670303f,-0.0016200768295675516f,-0.02855542115867138f,0.0203043594956398f,-0.010155503638088703f,-0.010173297487199306f,0.05264352262020111f,0.0041068196296691895f,0.0065082972869277f,0.006479648873209953f,-0.009269843809306622f,0.04682430252432823f,-0.035972271114587784f,-0.01087156031280756f,-0.0041751801036298275f,0.00044751609675586224f,0.006006654817610979f,-0.008876227773725986f,-0.09063226729631424f,-0.0015027931658551097f,-0.020188109949231148f,0.021730778738856316f,-0.004449247848242521f,-0.00551064545288682f,-0.00046295870561152697f,0.002554708393290639f,-0.11006581783294678f,0.007623685989528894f,-0.0037998610641807318f,-0.0038444986566901207f,0.0016549648717045784f,-0.01053270697593689f,0.00804843008518219f,-0.002035645768046379f,-0.027478501200675964f,0.07869353145360947f,-0.031180089339613914f,-0.048671264201402664f,-0.019841348752379417f,0.0018722774693742394f,-0.0010544118704274297f,-0.0071773892268538475f,-0.03104768879711628f,0.040862224996089935f,0.021762501448392868f,-0.06461072713136673f,0.04396488517522812f,-0.05060983821749687f,-0.0032281430903822184f,-0.004621637985110283f,-0.045512087643146515f,0.0033950915094465017f,-0.0016956721665337682f,-0.0017204907489940524f,-0.05981486663222313f,-0.016588350757956505f,0.006107216235250235f,-0.00639969389885664f,0.030491827055811882f,-0.013903111219406128f,0.025566376745700836f,-0.011847191490232944f,-0.02446122094988823f,-0.012384768575429916f,0.0021388940513134003f,-0.011670362204313278f,-0.109995037317276f,-0.044698528945446014f,0.055572863668203354f,-0.011079944670200348f,-0.0066475640051066875f,-0.023772509768605232f,0.004042047541588545f,-0.003739047097042203f,-0.05886714160442352f,0.016648799180984497f,-0.008331768214702606f,-0.008336653932929039f,-0.03683064505457878f,0.009594331495463848f,0.004724784288555384f,-0.0008087196620181203f,-0.03939623758196831f,0.020642558112740517f,-0.049302373081445694f,0.029097463935613632f,-0.005034842528402805f,-0.004104660823941231f,0.0026890893932431936f,0.005016183014959097f,-0.03612297773361206f,0.0033468299079686403f,-0.053448379039764404f,0.05019180476665497f,-0.01794469729065895f,-0.008085260167717934f,-0.0037286144215613604f,-0.0014169055502861738f,0.02364145964384079f,0.006312737241387367f,-0.003168269293382764f,-0.003168104449287057f,-0.028197024017572403f,-0.022530043497681618f,0.008906771428883076f,-0.005544030573219061f,0.013852075673639774f,0.05509497597813606f,0.02983144298195839f,-0.08559970557689667f,0.004482591059058905f,0.004778028931468725f,0.0045252954587340355f,0.0029509803280234337f,-0.02331065945327282f,0.06588243693113327f,-0.02413049153983593f,-0.04169044643640518f,0.008119151927530766f,0.0033871023915708065f,-0.001554945483803749f,0.0018793238559737802f,-0.06153357774019241f,0.023796986788511276f,-0.011906814761459827f,-0.011903179809451103f,0.07425630837678909f,-0.021125439554452896f,-0.019312452524900436f,-0.00182899902574718f,0.05977146327495575f,0.036805953830480576f,-0.023531043902039528f,-0.01304897852241993f,-0.023348448798060417f,-0.007246206980198622f,-0.0012791971676051617f,-0.0044538648799061775f,-0.01496290322393179f,-0.010641488246619701f,0.02259579487144947f,-0.012228628620505333f,-0.03589387238025665f,0.002000860869884491f,0.004232116509228945f,0.0015762411057949066f,-0.033133022487163544f,0.01289178803563118f,-0.006445091683417559f,-0.00643258448690176f,-0.04498116672039032f,0.06865061074495316f,0.018666017800569534f,-0.031315773725509644f,0.024546517059206963f,-0.0048734028823673725f,-0.028807135298848152f,0.033908162266016006f,0.022881783545017242f,5.6572775065433234e-05f,-0.0017650291556492448f,-0.014379080384969711f,-0.1086069643497467f,-0.019096331670880318f,-0.04773883894085884f,0.06472412496805191f,0.009767219424247742f,0.048091500997543335f,0.004914557095617056f,0.0012172014685347676f,-0.08117088675498962f,0.005914525128901005f,-0.0029603317379951477f,-0.0029510383028537035f,0.00972792599350214f,0.05522514134645462f,-0.0007920423522591591f,-0.0008185553015209734f,-0.054000090807676315f,-0.08852687478065491f,0.057801809161901474f,0.03101271577179432f,-0.07879697531461716f,0.08256207406520844f,0.0013684089062735438f,-0.0031933276914060116f,0.07066237181425095f,-0.018503718078136444f,-0.018003525212407112f,0.03631268069148064f,-0.012574750930070877f,0.009206438437104225f,-0.001140875043347478f,-0.0006801876006647944f,0.025492237880825996f,0.008513585664331913f,-0.004260778892785311f,-0.00425335019826889f,0.0019716816022992134f,-0.005541632417589426f,-0.0043639251962304115f,0.006223815027624369f,-0.05156635493040085f,-0.035560544580221176f,0.017849937081336975f,0.01815837062895298f,-0.027983408421278f,0.029024947434663773f,0.0066084121353924274f,-0.006428574677556753f,-0.11333778500556946f,-0.0337318517267704f,0.03627108410000801f,-0.0025220566894859076f,0.022827042266726494f,-0.020061226561665535f,-0.0025547773111611605f,0.0003810882626567036f,-0.12010267376899719f,0.005536508746445179f,-0.00276617961935699f,-0.0027488875202834606f,-0.006501649506390095f,-0.02653152495622635f,0.014423774555325508f,0.00017785251839086413f,-0.007200576364994049f,0.09080559760332108f,-0.0807747170329094f,-0.010109273716807365f,0.01556725986301899f,0.001301450189203024f,0.001921272836625576f,-0.008127315901219845f,-0.06304799765348434f,0.03520138934254646f,-0.013658391311764717f,-0.021619178354740143f,-0.012141743674874306f,-0.016779309138655663f,0.004391190595924854f,0.0005700453766621649f,-0.027298059314489365f,0.016735028475522995f,-0.008371463976800442f,-0.008345181122422218f,0.024399280548095703f,0.011585744097828865f,-5.656065695802681e-05f,0.004810457117855549f,0.010241546668112278f,-0.0010701988358050585f,-0.007617712952196598f,0.008910630829632282f,0.04120204225182533f,-0.02142462506890297f,-0.009877855889499187f,0.004470686428248882f,0.05262324586510658f,-0.004971448332071304f,-0.02278895303606987f,0.027555979788303375f,0.043585702776908875f,0.04328375309705734f,-0.004195980727672577f,0.0034722439013421535f,-0.0970095694065094f,0.0044244504533708096f,-0.002186431083828211f,-0.0022229680325835943f,0.010843748226761818f,-0.04101010411977768f,0.0010383370099589229f,-0.007576663978397846f,0.16022033989429474f,0.012650885619223118f,-0.03640235960483551f,0.02370738424360752f,0.006599106825888157f,-0.02797752618789673f,0.0011550132185220718f,-0.004020253196358681f,-0.19819775223731995f,0.035905852913856506f,-0.02496376261115074f,-0.011787253431975842f,-0.010815629735589027f,-0.03205179423093796f,-0.0028758684638887644f,-0.0036358314100652933f,-0.08859355002641678f,0.0091431038454175f,-0.004545407835394144f,-0.004569063428789377f,0.040727823972702026f,0.016960810869932175f,0.008628848008811474f,0.0029367110691964626f,-0.03727911412715912f,-0.027826977893710136f,0.05299966782331467f,-0.025269296020269394f,0.007429819088429213f,-0.0021138323936611414f,-0.0037909222301095724f,0.004965700674802065f,-0.03091885708272457f,0.030643543228507042f,-0.015952643007040024f,-0.014691065065562725f,-0.001820852281525731f,-0.002436872338876128f,0.0051171667873859406f,-0.0018058748682960868f,0.0638863816857338f,0.0342637337744236f,-0.017162399366497993f,-0.01714983955025673f,-0.019790755584836006f,0.0015464604366570711f,-0.0007324509206227958f,-0.001134454389102757f,0.02550813928246498f,-0.02715473435819149f,0.08462104946374893f,-0.05745604634284973f,-0.041815564036369324f,-0.022223273292183876f,-0.0010814403649419546f,-0.009244688786566257f,0.10879047960042953f,-0.059530649334192276f,0.0675390362739563f,-0.00789192970842123f,-0.002378051169216633f,0.008635744452476501f,-0.006431345362216234f,0.0006227574194781482f,-0.055087052285671234f,0.011622360907495022f,-0.005830441601574421f,-0.005819886457175016f,-0.10858076810836792f,0.030750960111618042f,-0.021928098052740097f,0.001077347667887807f,-0.26712292432785034f,-0.05500730127096176f,0.05110283941030502f,0.004126057494431734f,0.02084517478942871f,0.03733498975634575f,0.005348459351807833f,-0.0010189510649070144f,-0.09476888179779053f,-0.018066413700580597f,0.01507585495710373f,0.00300096208229661f,-0.018293948844075203f,-0.0013375666458159685f,-0.007959390990436077f,0.004336059559136629f,-0.18705634772777557f,0.0010386024368926883f,-0.0005004557897336781f,-0.0004930712748318911f,-0.06381937861442566f,0.06632784754037857f,0.019717808812856674f,0.011672679334878922f,0.11496718972921371f,0.09829528629779816f,-0.07709939777851105f,-0.021372973918914795f,-0.023269586265087128f,0.07106390595436096f,-0.009285593405365944f,-0.0033015848603099585f,0.04100077226758003f,0.03442201390862465f,-0.023178773000836372f,-0.011257094331085682f,-0.02262929640710354f,0.02330227941274643f,-0.010697678662836552f,0.005680266302078962f,0.025509381666779518f,0.042601097375154495f,-0.021289890632033348f,-0.021307386457920074f,-0.018434638157486916f,0.03270138055086136f,-0.007373816333711147f,-0.01963346265256405f,0.16297443211078644f,0.05076919123530388f,-0.013656701892614365f,-0.03732326626777649f,0.017743539065122604f,0.013408348895609379f,-0.004081363789737225f,0.010330861434340477f,-0.24630588293075562f,0.016324209049344063f,-0.009310105815529823f,-0.006869886070489883f,0.015448309481143951f,0.010390790179371834f,-0.0043857102282345295f,0.00023762758064549416f,0.05417885631322861f,0.031699005514383316f,-0.01587560959160328f,-0.015848586335778236f,0.09932389855384827f,-0.019927330315113068f,-0.030323626473546028f,-0.011022305116057396f,-0.3492455780506134f,-0.06608547270298004f,0.042322464287281036f,0.02461526170372963f,0.001181236351840198f,-0.012341953814029694f,-0.0039833164773881435f,-0.013316025026142597f,-0.2472294121980667f,0.007229106035083532f,-0.012610544450581074f,0.0052970959804952145f,-0.029113130643963814f,0.017532717436552048f,0.006677961442619562f,0.0019188917940482497f,-0.085416778922081f,0.007426510564982891f,-0.003719440894201398f,-0.003724446054548025f,0.030920403078198433f,-0.012393258512020111f,0.0051811812445521355f,-0.0015606756787747145f,0.0311978030949831f,0.05058354139328003f,0.03017636016011238f,-0.08136872202157974f,-0.014751799404621124f,0.030578726902604103f,-0.0006390633643604815f,0.0027676261961460114f,0.10782695561647415f,0.0650549903512001f,-0.008941683918237686f,-0.055461421608924866f,-0.005963122006505728f,-0.008794433437287807f,-0.0068213860504329205f,0.0008371236035600305f,0.12769530713558197f,0.034061744809150696f,-0.017042234539985657f,-0.017012184485793114f,-0.028875986114144325f,-0.03769960626959801f,-0.004463432356715202f,-0.024999679997563362f,0.19639280438423157f,0.0727878287434578f,-0.020122433081269264f,-0.053090814501047134f,0.01469175424426794f,-0.020854724571108818f,0.0027639446780085564f,-0.00695781921967864f,-0.10938939452171326f,0.0016546053811907768f,0.03457438200712204f,-0.03607553243637085f,0.01270182617008686f,0.006038181949406862f,0.0037766543682664633f,0.001449899049475789f,0.0022102822549641132f,0.02459975704550743f,-0.012269021943211555f,-0.01229025237262249f,-0.006211814470589161f,0.021028902381658554f,0.0008317400352098048f,-0.002704672748222947f,0.16345132887363434f,-0.026719510555267334f,-0.09049747884273529f,0.11702707409858704f,-0.004269582685083151f,-0.0014412467135116458f,-0.003575578099116683f,-0.00409666495397687f,-0.292587548494339f,-0.013368699699640274f,-0.0012876504333689809f,0.014510019682347775f,0.024133741855621338f,-0.012008302845060825f,0.010499512776732445f,-0.0007594689377583563f,-0.2007664144039154f,0.00549453217536211f,-0.0027401738334447145f,-0.0027467268519103527f,-0.06114906072616577f,0.023570671677589417f,0.008922620676457882f,0.012065665796399117f,-0.03593086823821068f,-0.05912930145859718f,0.03659156337380409f,0.02317354828119278f,-0.014942965470254421f,-0.00044496561167761683f,-0.008808275684714317f,-0.010051346383988857f,-0.10469762980937958f,-0.03414098918437958f,0.025145838037133217f,0.009853780269622803f,0.019382933154702187f,0.0296690221875906f,-0.0021501227747648954f,0.0015544205671176314f,-0.14136898517608643f,0.0015673604793846607f,-0.0007760431035421789f,-0.0008025772986002266f,0.02345147356390953f,-0.08643443882465363f,-0.00713470671325922f,-0.000369424931704998f,-0.2732272744178772f,-0.006908779498189688f,0.006650784518569708f,0.00017094491340685636f,0.0151821905747056f,-0.14571654796600342f,-0.00808644387871027f,-0.003124357433989644f,-0.28583526611328125f,0.007327874191105366f,-0.020209411159157753f,0.012723113410174847f,-0.004043673165142536f,-0.07280895859003067f,-0.004728357773274183f,-0.001724660280160606f,-0.17702196538448334f,0.0068467953242361546f,-0.003443856490775943f,-0.0034421433228999376f,0.007706261705607176f,-0.009607178159058094f,0.003132862038910389f,-0.005459682084619999f,0.06759533286094666f,0.08031918108463287f,-0.11237679421901703f,0.03405163064599037f,0.015101675875484943f,-0.005087828263640404f,0.004271821118891239f,-0.005794811528176069f,0.0900949016213417f,0.07198233157396317f,-0.04760279878973961f,-0.025044230744242668f,0.020306067541241646f,-0.02119281142950058f,-0.0005685520591214299f,0.00037909415550529957f,-0.08860421925783157f,0.01784929633140564f,-0.008920054882764816f,-0.008919310756027699f,0.028692036867141724f,0.03628266230225563f,0.0029008304700255394f,0.024382861331105232f,-0.31208306550979614f,0.004892346914857626f,-0.006015722639858723f,0.0014698704471811652f,0.08427462726831436f,0.006189660634845495f,0.004919843748211861f,0.004750370979309082f,-0.2842162549495697f,-0.002113706199452281f,0.02173924818634987f,-0.019752992317080498f,0.0017922839615494013f,-0.050949372351169586f,0.0030419216491281986f,-0.006095169577747583f,-0.12788084149360657f,0.004426322877407074f,-0.0022111497819423676f,-0.00222689937800169f,-0.0746661126613617f,-0.047130659222602844f,-0.030550256371498108f,-0.03377775475382805f,-0.2001727819442749f,0.0038178269751369953f,0.006454599555581808f,-0.010324778966605663f,-0.016226287931203842f,0.008178941905498505f,0.022594181820750237f,0.010721253231167793f,-0.31333523988723755f,0.005998409818857908f,-0.011991961859166622f,0.005973593331873417f,-0.050964441150426865f,-0.02260504849255085f,-0.007051934488117695f,0.0017805701354518533f,-0.23189777135849f,0.0026707896031439304f,-0.0013600894017145038f,-0.0013180183013901114f,-0.019774844869971275f,-0.0028918583411723375f,0.0049509271048009396f,0.005575687158852816f,-0.12770280241966248f,-0.011662484146654606f,-0.006353633012622595f,0.01810944266617298f,-0.000719588715583086f,-0.011318389326334f,-0.0008835929911583662f,0.0017214680556207895f,-0.061524227261543274f,0.0059049976989626884f,-0.0010427769739180803f,-0.004945310764014721f,0.009661585092544556f,-0.024994898587465286f,0.0027334149926900864f,-0.000970801105722785f,-0.016169734299182892f,0.021034469828009605f,-0.010505402460694313f,-0.010499611496925354f,0.01828712411224842f,-0.015640724450349808f,-0.004174928646534681f,-0.009187599644064903f,-0.002961356658488512f,0.08023867011070251f,-0.04197605699300766f,-0.0382804311811924f,-0.012447211891412735f,-0.013628987595438957f,0.0013858609599992633f,-0.005812354851514101f,-0.0008365433313883841f,0.02125941962003708f,-0.03933589532971382f,0.018743136897683144f,-0.020523453131318092f,0.02566157840192318f,-0.003033280838280916f,0.002191282343119383f,-0.034336816519498825f,0.022532865405082703f,-0.01125198882073164f,-0.011280824430286884f,0.0010308650089427829f,0.02389249950647354f,-0.004065461456775665f,0.005299905315041542f,0.017764762043952942f,0.008681794628500938f,0.03297068551182747f,-0.04007433354854584f,-0.013035619631409645f,0.007431432604789734f,-0.0016158211510628462f,-0.006247321609407663f,-0.09507367759943008f,0.10189647227525711f,0.014482651837170124f,-0.11740302294492722f,0.022308867424726486f,-6.30819340585731e-05f,-0.00251353089697659f,-0.002036509569734335f,0.04937461018562317f,0.033735550940036774f,-0.016885297372937202f,-0.01688193529844284f,0.040567830204963684f,-0.008137552998960018f,0.019484197720885277f,0.008250306360423565f,-0.2054869681596756f,0.07707125693559647f,-0.016763662919402122f,-0.06056457385420799f,0.03294488787651062f,-0.03329310938715935f,-0.000846200215164572f,-0.00318560260348022f,-0.12614524364471436f,0.03905067592859268f,-0.03105916827917099f,-0.008081814274191856f,-0.019673440605401993f,-0.018325984477996826f,-0.0051965597085654736f,-0.00041432291618548334f,-0.08482320606708527f,0.006952979601919651f,-0.0034925867803394794f,-0.0034823508467525244f,-0.021025190129876137f,-0.004712366499006748f,-0.014894253574311733f,-0.01679682545363903f,-0.32661062479019165f,0.028093982487916946f,-0.0670655220746994f,0.03874538838863373f,-0.0016775265103206038f,-0.003315731417387724f,-0.007668225094676018f,-0.019494228065013885f,-0.15109339356422424f,0.0558517649769783f,-0.03976821526885033f,-0.016227519139647484f,0.017022302374243736f,0.010578891262412071f,0.004326851572841406f,0.0013096180045977235f,-0.13399963080883026f,0.003724826266989112f,-0.001864174148067832f,-0.0018629248952493072f,0.017072118818759918f,0.025576351210474968f,-0.0009397637913934886f,-0.0037707542069256306f,0.057779811322689056f,-0.12872278690338135f,-0.18870596587657928f,0.315837562084198f,-0.00924250390380621f,0.02252945490181446f,-0.0017428521532565355f,-0.00040933905984275043f,-0.06532055139541626f,-0.07580620050430298f,-0.06477382779121399f,0.14081914722919464f,-0.033269256353378296f,0.02068648301064968f,-0.0026987073943018913f,0.001923463656567037f,-0.20767222344875336f,0.008541708812117577f,-0.0042500062845647335f,-0.004254618659615517f,-0.006135857198387384f,0.003648810088634491f,0.003010217100381851f,-0.011392244137823582f,-0.005128358956426382f,0.01505371555685997f,0.0036636083386838436f,-0.01838308945298195f,0.020672975108027458f,-0.0011524423025548458f,-0.001004637568257749f,0.00012626653187908232f,-0.0008760302443988621f,-0.023059649392962456f,0.02717508375644684f,-0.004146641585975885f,0.01485541183501482f,0.009769139811396599f,0.01097754668444395f,0.0015185691881924868f,-0.07035485655069351f,0.007411795202642679f,-0.003690920304507017f,-0.0036880485713481903f,-0.019857969135046005f,-0.1087544709444046f,-0.007834861986339092f,-0.05498707666993141f,0.012353452853858471f,-0.009267766028642654f,-0.004562105052173138f,0.014087996445596218f,-0.017875447869300842f,-0.044468969106674194f,0.01363156083971262f,0.0005394110339693725f,-0.11778195947408676f,0.06774245947599411f,-0.08088941127061844f,0.013452197425067425f,-0.0019874442368745804f,-0.03218751773238182f,-0.0031974592711776495f,-0.003076619701460004f,-0.03386785462498665f,0.005941341631114483f,-0.002978722332045436f,-0.0030082990415394306f,-0.002257463289424777f,-0.008999488316476345f,0.020240670070052147f,-0.0028539467602968216f,-0.4157344698905945f,0.03134005144238472f,-0.021405404433608055f,-0.009968838654458523f,0.03559790551662445f,0.0013702678261324763f,0.0009429427445866168f,0.012773714028298855f,-0.11701076477766037f,0.04812367632985115f,-0.03279300034046173f,-0.015337868593633175f,-0.018497154116630554f,0.050734903663396835f,-0.005704935174435377f,0.0032561058178544044f,-0.03864706680178642f,0.020526226609945297f,-0.010270013473927975f,-0.010257557965815067f,-0.0044601717963814735f,0.06129768490791321f,-0.0717429593205452f,-0.0003304474812466651f,-0.18766525387763977f,0.0013033176073804498f,-0.011711942963302135f,0.01030154712498188f,-0.016728481277823448f,-0.0006149637047201395f,0.0011846086708828807f,0.0019624601118266582f,-0.12450180947780609f,-0.012763218022882938f,0.024460390210151672f,-0.01145278662443161f,-0.003275842871516943f,0.009111748076975346f,-0.004311754368245602f,0.0013242175336927176f,-0.05433563515543938f,0.01634872332215309f,-0.008173237554728985f,-0.008155587129294872f,0.030469870194792747f,0.01968088559806347f,-0.04315388575196266f,-0.005976032931357622f,0.17330147325992584f,0.10025133192539215f,-0.10975988209247589f,0.010134050622582436f,-0.01837664097547531f,0.04888314753770828f,-0.04280323162674904f,0.03278031572699547f,-0.11340758204460144f,0.12849098443984985f,-0.09958471357822418f,-0.02907772734761238f,0.048655446618795395f,-0.018990760669112206f,-0.048118337988853455f,0.006178673822432756f,-0.09058889001607895f,0.01645679771900177f,-0.008260884322226048f,-0.008247876539826393f,0.0035560736432671547f,0.008185525424778461f,-0.011375928297638893f,0.006172870751470327f,-0.1712280958890915f,0.022038273513317108f,0.002675673458725214f,-0.02394087053835392f,0.001729475916363299f,0.016513695940375328f,0.0068823229521512985f,0.009138964116573334f,-0.15232853591442108f,0.022769851610064507f,0.0013158161891624331f,-0.024199778214097023f,0.009007344953715801f,0.008669594302773476f,-0.00296968687325716f,0.00419194670394063f,0.007808046881109476f,0.0263554397970438f,-0.013210274279117584f,-0.01320168748497963f,-0.049082428216934204f,-0.008167879655957222f,-0.007725255563855171f,0.004782667849212885f,-0.08499199151992798f,0.07075496762990952f,-0.06796061247587204f,-0.002347448607906699f,-0.013250639662146568f,0.006850717589259148f,0.00396868959069252f,0.00600857101380825f,-0.15744173526763916f,0.0047021773643791676f,0.027635062113404274f,-0.031880274415016174f,-0.004950571805238724f,0.0028152000159025192f,0.0027766271959990263f,-0.0010469210101291537f,0.011740787886083126f,0.019531073048710823f,-0.00976784247905016f,-0.009752269834280014f,0.01221044734120369f,-0.010989582166075706f,-0.003686476033180952f,-0.010378053411841393f,-0.006491472478955984f,-0.08684487640857697f,-0.09375055879354477f,0.18007352948188782f,0.005353052169084549f,0.00715246656909585f,9.776624210644513e-05f,0.006506054662168026f,0.02346787601709366f,0.022012166678905487f,-0.11409584432840347f,0.09225776046514511f,-0.00854916125535965f,-0.0002924927102867514f,-0.001004316727630794f,0.0025553698651492596f,-0.07128749787807465f,0.020854897797107697f,-0.010418820194900036f,-0.010412339121103287f,-0.034822627902030945f,0.05972987040877342f,0.04010673984885216f,0.009806862100958824f,-0.27452847361564636f,0.022698083892464638f,0.006626174785196781f,-0.029363473877310753f,-0.11886280030012131f,0.022461723536252975f,0.004462151322513819f,-0.0030505256727337837f,-0.19609031081199646f,0.006221664138138294f,-0.01082450058311224f,0.004578642081469297f,-0.0500485859811306f,-0.042549196630716324f,0.007512621581554413f,-0.0005178785067982972f,-0.05046003684401512f,0.007345013320446014f,-0.003709119511768222f,-0.0036755260080099106f,-0.01656504161655903f,-0.010656215250492096f,-0.003294573863968253f,-0.0018589210230857134f,-0.016232352703809738f,0.045763466507196426f,0.09623856097459793f,-0.14470279216766357f,0.03292237967252731f,-0.014394046738743782f,-0.005387214012444019f,-0.0003465202171355486f,-0.07325559109449387f,-0.029327617958188057f,0.09455083310604095f,-0.06630904227495193f,-0.02871922217309475f,-0.04507709667086601f,0.005413271952420473f,-0.002825875300914049f,-0.17268776893615723f,0.004037800244987011f,-0.001999480649828911f,-0.001997630810365081f,0.07398887723684311f,-0.008963963948190212f,0.0053209527395665646f,-0.015757087618112564f,0.11407666653394699f,-0.034228838980197906f,0.06930126994848251f,-0.03558792173862457f,0.05366797000169754f,-0.0020997123792767525f,0.003562026424333453f,-0.002960351761430502f,-0.02339944988489151f,-0.04178608953952789f,0.011991471983492374f,0.029347244650125504f,0.002562162932008505f,-0.02468058653175831f,0.0010344370966777205f,-0.001146651222370565f,-0.020441347733139992f,0.020217586308717728f,-0.010105624794960022f,-0.010111137293279171f,-0.006621254608035088f,-0.018055925145745277f,-0.011162898503243923f,-0.013117588125169277f,0.11125712096691132f,-0.18891866505146027f,0.1165822446346283f,0.07309278845787048f,-0.011435502208769321f,-0.006881115026772022f,0.005182634573429823f,0.0015483250608667731f,-0.13874945044517517f,0.0010733854724094272f,-0.049897558987140656f,0.048656806349754333f,-0.014368385076522827f,0.020482124760746956f,-2.762052281468641e-05f,0.0010692563373595476f,0.04184794798493385f,0.006619623396545649f,-0.003312080865725875f,-0.003305121324956417f,0.05241032689809799f,-0.01962527260184288f,0.021027393639087677f,-0.015642881393432617f,-0.17868369817733765f,0.04031004384160042f,-0.11415668576955795f,0.07350519299507141f,0.03555108979344368f,0.017394516617059708f,0.0020073617342859507f,0.0021610066760331392f,0.001251746085472405f,0.05621366202831268f,-0.034510307013988495f,-0.021975718438625336f,0.012959127314388752f,0.019212186336517334f,-0.0019043313805013895f,0.0013038305332884192f,0.014346469193696976f,0.017058616504073143f,-0.008549249731004238f,-0.008548477664589882f,0.008722270838916302f,-0.022628536447882652f,0.0035880268551409245f,0.009406810626387596f,-0.2547619044780731f,-0.036772534251213074f,0.06298809498548508f,-0.02590014412999153f,0.016556911170482635f,-0.014338997192680836f,0.001508739311248064f,-0.00013795658014714718f,0.0802709236741066f,-0.030572757124900818f,0.07043977081775665f,-0.03985115885734558f,0.020097453147172928f,-0.01044008880853653f,0.0009790565818548203f,0.0006903441390022635f,0.010689436458051205f,0.03387533128261566f,-0.016927091404795647f,-0.01691899262368679f,0.015265768393874168f,0.13143384456634521f,0.009716635569930077f,-0.026731569319963455f,0.024057980626821518f,0.04812535271048546f,0.049424972385168076f,-0.0978826954960823f,0.004648853093385696f,0.12977585196495056f,0.010076913982629776f,0.010013401508331299f,-0.14369139075279236f,0.13604889810085297f,-0.09959785640239716f,-0.03638561815023422f,-0.03411544859409332f,0.14463792741298676f,-0.013184905052185059f,0.001649712212383747f,-0.10547561198472977f,0.028516223654150963f,-0.014273682609200478f,-0.014266632497310638f,0.03785344213247299f,0.15610241889953613f,0.00981774739921093f,0.025809671729803085f,0.03377138823270798f,0.039580363780260086f,0.030722077935934067f,-0.07054157555103302f,0.04124641418457031f,0.19274039566516876f,-0.0034996680915355682f,-0.013644878752529621f,0.1742255836725235f,0.005102378316223621f,0.026063119992613792f,-0.030993737280368805f,0.03199194371700287f,0.06428086757659912f,0.005873058922588825f,0.0009213550365529954f,-0.0542166605591774f,0.01892496831715107f,-0.009447847492992878f,-0.009476616978645325f,-0.005004535894840956f,-0.02074173279106617f,0.017314855009317398f,-0.0007499019266106188f,-0.2509117126464844f,0.018205758184194565f,-0.005313935689628124f,-0.012699251063168049f,-0.0010760911973193288f,0.051617708057165146f,0.008385039865970612f,0.005081030540168285f,-0.46402043104171753f,-0.019829941913485527f,0.03749428316950798f,-0.017958644777536392f,-0.009527676738798618f,0.001172432443127036f,0.001445441390387714f,0.0028120295610278845f,-0.34552159905433655f,0.0028547977562993765f,-0.001429122406989336f,-0.0014443137915804982f,-0.014693653210997581f,0.02034577913582325f,0.003328988328576088f,0.006520538590848446f,-0.15895895659923553f,0.016953185200691223f,0.008326281793415546f,-0.02515161968767643f,-0.005208236165344715f,-0.018320458009839058f,0.004757223650813103f,0.001423228532075882f,-0.12706205248832703f,-0.013273078948259354f,0.052517395466566086f,-0.039945583790540695f,-0.038685474544763565f,-0.061786673963069916f,-0.004464232362806797f,-0.003394502215087414f,0.038784727454185486f,0.01717158779501915f,-0.00857973750680685f,-0.00859263725578785f,0.009531013667583466f,0.00468214089050889f,-0.0009037221316248178f,-0.0009252745076082647f,0.009657202288508415f,-0.07829654216766357f,0.06605736911296844f,0.01159406267106533f,-0.005308153107762337f,0.00020093620696570724f,-0.005024699494242668f,0.00744983833283186f,-0.004194458480924368f,0.0484008863568306f,-0.04764796793460846f,-0.0010986041743308306f,0.0008652237011119723f,0.0504588857293129f,-0.004946428816765547f,0.0018553102854639292f,-0.025321880355477333f,0.021448103711009026f,-0.01069246232509613f,-0.010707670822739601f,0.0016099864151328802f,-0.006330064032226801f,0.0014867183053866029f,0.010876416228711605f,-0.18120728433132172f,-0.006351771764457226f,0.04616987332701683f,-0.039660483598709106f,1.0053892765427008e-05f,-0.015163627453148365f,-0.004340827465057373f,-0.003225386608392f,0.04215424135327339f,0.026851119473576546f,-0.008221149444580078f,-0.01868264563381672f,-0.02556309476494789f,-0.016647370532155037f,0.0055925073102116585f,-0.0035567202139645815f,-0.03131730109453201f,0.005666698794811964f,-0.0028266985900700092f,-0.0028131951112300158f,-0.01073633972555399f,0.003944880329072475f,0.0017808645498007536f,0.009057361632585526f,-0.08360417187213898f,-0.07387471199035645f,0.05129729211330414f,0.022532202303409576f,0.01432438101619482f,0.008824681863188744f,-0.0005789296119473875f,0.0008444586419500411f,-0.06078111752867699f,0.0016170465387403965f,0.019712520763278008f,-0.021533988416194916f,0.018794681876897812f,0.02278795652091503f,0.004963192623108625f,0.0002654541749507189f,0.07658596336841583f,0.023998569697141647f,-0.012020651251077652f,-0.012027065269649029f,0.007361601106822491f,-0.005863400641828775f,0.0018703786190599203f,-0.0037903760094195604f,0.1581890881061554f,-0.0743163451552391f,0.18150660395622253f,-0.1077154353260994f,0.021876011043787003f,0.008931060321629047f,0.00023161288117989898f,-0.0025042586494237185f,0.03780444711446762f,-0.09529980272054672f,0.12978826463222504f,-0.03456361964344978f,0.023246703669428825f,-0.010495735332369804f,0.0036741995718330145f,0.0012769545428454876f,-0.0008079120889306068f,0.019185755401849747f,-0.0095903305336833f,-0.009611522778868675f,0.0038126513827592134f,-0.004331398755311966f,-0.004957099910825491f,0.007896391674876213f,0.07667763531208038f,-0.020515942946076393f,0.054043568670749664f,-0.034456461668014526f,-0.0017188491765409708f,-0.011552236042916775f,-0.0045847040601074696f,-0.00024111421953421086f,-0.026491589844226837f,-0.04836316034197807f,0.04002154991030693f,0.008274566382169724f,-0.01873541995882988f,-0.028039585798978806f,-0.008892991580069065f,-0.005981852300465107f,0.0554928258061409f,0.00957640539854765f,-0.004818187095224857f,-0.004802373703569174f,0.04555855318903923f,0.01981431059539318f,0.021522685885429382f,0.000397175521356985f,-0.07039017230272293f,0.032911136746406555f,-0.019590578973293304f,-0.01381685584783554f,0.023267148062586784f,0.018014617264270782f,0.011609960347414017f,0.0003943823976442218f,0.06180889531970024f,0.019269075244665146f,-0.01951960101723671f,-0.0002786446420941502f,0.06835507601499557f,0.036558978259563446f,0.0010957771446555853f,0.0008174848626367748f,-0.022176364436745644f,0.023739688098430634f,-0.011864079162478447f,-0.011854985728859901f,-0.0191531702876091f,0.0038528949953615665f,-0.0029420575592666864f,0.006747294683009386f,0.06696043908596039f,-0.05303814634680748f,0.05780176818370819f,-0.00484087411314249f,-0.011859504505991936f,-0.021550646051764488f,-0.01006025169044733f,-0.0011140262940898538f,-0.06026364862918854f,-0.023210586979985237f,0.010606713593006134f,0.012552469968795776f,-0.010366030968725681f,-0.02329300343990326f,0.005310877226293087f,-0.0019557925406843424f,-0.07063841819763184f,0.015683205798268318f,-0.007839292287826538f,-0.007844223640859127f,-0.0507197268307209f,0.014415860176086426f,-0.007246247958391905f,0.00955780129879713f,-0.056275103241205215f,0.05077946186065674f,-0.10836100578308105f,0.0572647824883461f,-0.011940481141209602f,-0.025995781645178795f,0.003151374403387308f,-0.012575549073517323f,0.004156091250479221f,-0.015456363558769226f,-0.008085043169558048f,0.023485396057367325f,-0.014621815644204617f,-0.026114750653505325f,-0.005881279706954956f,-0.001566138002090156f,-0.015390205197036266f,0.009446436539292336f,-0.004740644712001085f,-0.004723007325083017f,0.03313489630818367f,0.012669430114328861f,0.0060190060175955296f,0.013399448245763779f,0.06579700112342834f,0.0029438408091664314f,0.06000668555498123f,-0.06314921379089355f,0.02808316797018051f,-0.009739149361848831f,0.0030463910661637783f,-0.0005925404257141054f,-0.0676855742931366f,0.017847595736384392f,-0.008086904883384705f,-0.009244110435247421f,0.019091377034783363f,0.002822510665282607f,-0.00036733271554112434f,-0.0016783602768555284f,-0.029973937198519707f,0.013662355951964855f,-0.006814112886786461f,-0.006840965244919062f,0.007937435060739517f,0.013950219377875328f,-0.0005416481290012598f,-0.00790267065167427f,-0.06214280426502228f,0.0799562931060791f,-0.04467428848147392f,-0.035654086619615555f,-0.003816604381427169f,-0.0034282905980944633f,-0.00336728198453784f,-0.00907030887901783f,0.0428909957408905f,0.003424027469009161f,0.023201242089271545f,-0.026662200689315796f,0.013073998503386974f,0.0034562258515506983f,0.0039945002645254135f,-0.0012512619141489267f,0.04778072610497475f,0.008078699000179768f,-0.004050047602504492f,-0.004037660080939531f,0.024017201736569405f,-0.010626554489135742f,0.004888350609689951f,-0.005788196809589863f,-0.05237158387899399f,0.023348666727542877f,-0.0464039072394371f,0.02314240112900734f,-0.030030613765120506f,0.002231896622106433f,0.0007896830793470144f,0.0007079122588038445f,-0.024377278983592987f,0.029577242210507393f,-0.02219952642917633f,-0.007364812307059765f,-0.005002200603485107f,-0.009989188052713871f,-0.005001363344490528f,0.002592455828562379f,-0.03971610218286514f,0.006144314538687468f,-0.0030398096423596144f,-0.0030608687084168196f,-0.0020535618532449007f,-0.017816508188843727f,-0.016024217009544373f,-0.008364574983716011f,-0.028866074979305267f,-0.0029891948215663433f,0.0014163970481604338f,0.0020831190049648285f,0.03598565608263016f,-0.004225226119160652f,0.0006329382886178792f,-0.01108043733984232f,0.008772804401814938f,-0.018314488232135773f,0.009797188453376293f,0.008659557439386845f,0.002887526759877801f,-0.014718946069478989f,0.002831661608070135f,0.0017373854061588645f,-0.12812809646129608f,0.00420868955552578f,-0.0020962010603398085f,-0.002107798121869564f,-0.01886986754834652f,0.000780520960688591f,-0.005310970824211836f,-0.00031205674167722464f,-0.1061519905924797f,-0.0196661539375782f,-0.007947924546897411f,0.02740885131061077f,-0.03422396630048752f,-0.015775982290506363f,-0.007338108494877815f,0.0025891936384141445f,-0.1202084943652153f,-0.059308718889951706f,0.036975711584091187f,0.022304097190499306f,0.01217286754399538f,-0.0092927822843194f,-0.0005520388367585838f,0.002644079038873315f,-0.03957151994109154f,0.011215218342840672f,-0.005619758740067482f,-0.005627416539937258f,-0.05236407741904259f,-0.01260537002235651f,-0.010613619349896908f,-0.006198165472596884f,0.10621032863855362f,-0.03262389823794365f,0.05955837294459343f,-0.026957431808114052f,-0.002553540049120784f,-0.017696255818009377f,-0.0034697121009230614f,-0.007108122576028109f,-0.041105445474386215f,0.008662942796945572f,-0.008605960756540298f,-5.9572044847300276e-05f,0.022258276119828224f,0.000691587571054697f,-0.0005937839159741998f,-0.0006557638407684863f,-0.11687891185283661f,0.005952910054475069f,-0.0030040403362363577f,-0.003003103658556938f,0.0018125656060874462f,0.02682492509484291f,-0.012386568821966648f,0.00468238303437829f,-0.0312851257622242f,0.022224893793463707f,0.011244668625295162f,-0.03344488516449928f,0.012730040587484837f,0.0031145624816417694f,-0.00119424844160676f,-0.003879984375089407f,0.012365078553557396f,0.02239053323864937f,-0.013535205274820328f,-0.008777758106589317f,-0.004724305123090744f,0.001540709170512855f,0.008257835172116756f,0.002503702649846673f,-0.0791873037815094f,0.005959509406238794f,-0.002994708949699998f,-0.0029608483891934156f,-0.03466517850756645f,0.029568281024694443f,-0.002049370901659131f,-0.003529931418597698f,0.10491932183504105f,0.03628744184970856f,-0.028257910162210464f,-0.008027113974094391f,-0.04691130667924881f,0.024542730301618576f,-0.008550355210900307f,-0.009572670795023441f,-0.1595226228237152f,-0.018405957147479057f,-0.013417035341262817f,0.032135915011167526f,0.037647880613803864f,0.022823741659522057f,0.00351845845580101f,0.0034722951240837574f,0.033254772424697876f,0.011169475503265858f,-0.005571634043008089f,-0.005598040297627449f,0.005253777839243412f,0.01470942236483097f,-0.0030206895899027586f,0.0006027960334904492f,-0.010270556434988976f,-0.09775539487600327f,0.03415753319859505f,0.0634964108467102f,-0.011836291290819645f,-0.004762724041938782f,0.0006319001549854875f,0.001377890002913773f,-0.012495496310293674f,-0.030825743451714516f,0.00912484060972929f,0.02149464376270771f,0.02975027821958065f,-0.022974051535129547f,0.0020971931517124176f,-0.0004903228837065399f,0.0036972605157643557f,0.01733647845685482f,-0.008696286007761955f,-0.008650196716189384f,0.011313664726912975f,0.009536148980259895f,0.018243977800011635f,0.010671695694327354f,-0.05911564081907272f,0.0825214684009552f,-0.02639908157289028f,-0.0572747066617012f,0.012420409359037876f,0.020969707518815994f,0.003180590458214283f,-0.005302890669554472f,0.13536715507507324f,0.033537864685058594f,-0.01208151038736105f,-0.021755706518888474f,-0.004182324279099703f,0.014653882943093777f,-0.0014888780424371362f,0.00233148573897779f,-0.032755136489868164f,0.010099126026034355f,-0.005072098225355148f,-0.005072017200291157f,0.0011028499575331807f,0.0007082720985636115f,-0.011593789793550968f,-0.004358477890491486f,0.04861287400126457f,0.03016957826912403f,-0.00013834274432156235f,-0.029312532395124435f,0.004989195615053177f,0.0009498620056547225f,6.548520468641073e-05f,0.0008454438066110015f,-0.05958237126469612f,0.04779858887195587f,-0.0205951277166605f,-0.027126535773277283f,0.026654159650206566f,0.031068405136466026f,0.0007558703073300421f,0.003979661036282778f,-0.05602032318711281f,0.01192131545394659f,-0.005976355168968439f,-0.0059523265808820724f,-0.053519491106271744f,-0.0027399284299463034f,0.003983015660196543f,0.007595384959131479f,-0.003016476985067129f,0.07113584876060486f,-0.003872143803164363f,-0.0677691102027893f,-0.0018765124259516597f,0.010047337040305138f,-0.001997519051656127f,0.002184197073802352f,0.01640055701136589f,0.009685039520263672f,0.0019370217341929674f,-0.01175012718886137f,0.014559361152350903f,-0.015760328620672226f,0.006387345027178526f,0.000821357942186296f,-0.018392227590084076f,0.01120972540229559f,-0.005589141510426998f,-0.005629433784633875f,-0.03464967757463455f,0.0016783066093921661f,-0.0015664080856367946f,-0.010660859756171703f,-0.03140927851200104f,-0.0003539128811098635f,-0.032264284789562225f,0.032363999634981155f,-0.005310948472470045f,0.002070730086416006f,0.0012842891737818718f,-0.0007884156657382846f,-0.026114413514733315f,0.028355417773127556f,-0.038345180451869965f,0.010007458738982677f,-0.007904100231826305f,0.0035975149367004633f,0.007275674492120743f,-0.001775909448042512f,-0.045361410826444626f,0.004226495046168566f,-0.0021437089890241623f,-0.002132538938894868f,0.02476034313440323f,-0.028203362599015236f,0.005975064821541309f,-0.0028878997545689344f,0.04361909255385399f,0.05453968048095703f,-0.08329389989376068f,0.0287447739392519f,0.004529787227511406f,-0.01111418567597866f,0.0019444177160039544f,-0.002541452180594206f,0.012545616365969181f,0.008128960616886616f,-0.017485206946730614f,0.0090394988656044f,0.0011048780288547277f,-0.002856297418475151f,0.0065004779025912285f,-0.0007669366314075887f,0.051371559500694275f,0.008739574812352657f,-0.004361606203019619f,-0.0043871100060641766f,-0.03301886469125748f,-0.016120102256536484f,-0.011587727814912796f,-0.007203896064311266f,-0.15738196671009064f,-0.03360282629728317f,0.03944752737879753f,-0.005822711158543825f,0.0061808242462575436f,-0.008391637355089188f,0.00792929157614708f,0.0031820645090192556f,-0.009680014103651047f,-0.004062832333147526f,-0.005675937049090862f,0.009673204272985458f,0.025843027979135513f,0.01051853597164154f,0.007734907791018486f,0.0018995641730725765f,-0.02325470931828022f,0.010092492215335369f,-0.005041713360697031f,-0.005043837707489729f,-0.002922645304352045f,-0.03965970501303673f,-0.0036606115754693747f,-0.0034589844290167093f,0.04218606650829315f,0.021042997017502785f,-0.07348565757274628f,0.05276080593466759f,0.01203442644327879f,0.008337336592376232f,0.008399885147809982f,-0.0015423157019540668f,0.050824131816625595f,0.03242801874876022f,-0.04974615201354027f,0.017171710729599f,-0.024037716910243034f,-0.022208716720342636f,0.00531601021066308f,-0.001561073469929397f,-0.07123661786317825f,0.011604312807321548f,-0.005782268941402435f,-0.005790640600025654f,-0.0172920860350132f,0.003619409864768386f,-0.0014511349145323038f,-0.009112119674682617f,0.0900755524635315f,-0.027651304379105568f,-0.016722416505217552f,0.044282231479883194f,0.01121610775589943f,0.012609350495040417f,0.0026944256387650967f,-0.0019418991869315505f,-0.04040570929646492f,-0.00408914452418685f,-0.020667146891355515f,0.024821897968649864f,0.03017517365515232f,9.279157529817894e-05f,0.006946716923266649f,0.0024044730234891176f,0.03336465358734131f,0.011930376291275024f,-0.005987694952636957f,-0.005968501791357994f,-0.023372478783130646f,0.003217010758817196f,-0.0022760520223528147f,-0.0026926356367766857f,-0.0014578503323718905f,0.027263179421424866f,-0.0404185950756073f,0.013116417452692986f,0.009669788181781769f,-0.02189922146499157f,-0.00012720628001261503f,-0.00854406412690878f,0.08470457047224045f,-0.06819310039281845f,0.05726619064807892f,0.010971331968903542f,0.02910085767507553f,-0.009163696318864822f,-0.00329489354044199f,-0.0006760601536370814f,-0.022956885397434235f,0.016246115788817406f,-0.008150555193424225f,-0.008121225982904434f,-0.022064264863729477f,-0.021899158135056496f,-0.0026198462583124638f,-0.020526431500911713f,0.01564720645546913f,-0.023075049743056297f,-0.032226476818323135f,0.055562105029821396f,-0.014824457466602325f,-0.007341291289776564f,3.504308551782742e-06f,-0.0033336514607071877f,0.014294610358774662f,-0.038437750190496445f,0.0010619882959872484f,0.03744243085384369f,0.04889532923698425f,-0.010294841602444649f,0.00826332625001669f,0.0004458889306988567f,-0.03507968783378601f,0.007470269687473774f,-0.003715141676366329f,-0.003751882119104266f,-0.017462730407714844f,-0.014303695410490036f,-0.009035169146955013f,-0.006074125878512859f,-0.010065228678286076f,-0.00848526693880558f,0.03662675619125366f,-0.027886386960744858f,-0.025654304772615433f,-0.0037390084471553564f,-0.01132019143551588f,0.005963183008134365f,0.011106686666607857f,0.0378720685839653f,-0.021159494295716286f,-0.01672075316309929f,-0.006493373308330774f,0.031236127018928528f,-0.004553136415779591f,0.00468999519944191f,0.00230355910025537f,0.010697129182517529f,-0.005344447214156389f,-0.005334880668669939f,-0.008375749923288822f,-0.016107797622680664f,-0.00642960611730814f,-0.006825712975114584f,0.015362927690148354f,0.05068764090538025f,-0.05320950970053673f,0.0024602306075394154f,0.03050852380692959f,-0.004207601770758629f,0.004668915178626776f,0.005211011040955782f,0.037052951753139496f,0.06064847856760025f,-0.03131799399852753f,-0.029599037021398544f,0.010193738155066967f,-0.0036045012529939413f,0.007676732260733843f,-0.0013738761190325022f,-0.050274305045604706f,0.010492652654647827f,-0.005258372984826565f,-0.005261310376226902f,-0.024431372061371803f,-0.010795917361974716f,-0.0006946012144908309f,-0.0018402310088276863f,-0.0692092552781105f,-0.07798607647418976f,0.027879182249307632f,0.05010869726538658f,0.016587145626544952f,0.014520427212119102f,0.0014590697828680277f,-0.0016845373902469873f,-0.07622329890727997f,-0.024974633008241653f,0.004651323892176151f,0.020260967314243317f,-0.007211277261376381f,0.02178226225078106f,0.0012958853039890528f,0.0036482298746705055f,-0.09277014434337616f,0.010674838908016682f,-0.005328286439180374f,-0.005340101197361946f,0.011768561787903309f,0.006401945371180773f,0.0069022211246192455f,0.01030520349740982f,-0.07375749945640564f,0.003998562227934599f,-0.0069695706479251385f,0.0029251486994326115f,0.024590766057372093f,0.029469599947333336f,-0.0022223335690796375f,0.0005331884603947401f,0.020702671259641647f,-0.025784961879253387f,0.002428032224997878f,0.02328699827194214f,-0.020897503942251205f,-0.01085282675921917f,0.0010588454315438867f,0.0021291172597557306f,0.06105886772274971f,0.023667430505156517f,-0.011852063238620758f,-0.011865057982504368f,0.017397193238139153f,0.008862687274813652f,0.007730826269835234f,-0.0022940505295991898f,0.036121245473623276f,0.08905480057001114f,-0.022811755537986755f,-0.06758996099233627f,-0.02400318905711174f,0.007683494593948126f,-0.006751726381480694f,0.0019241997506469488f,-0.0025131211150437593f,0.08306289464235306f,-0.05323437228798866f,-0.03047693334519863f,-0.012772362679243088f,0.0011092289350926876f,-0.005339048337191343f,-0.00019308808259665966f,0.036718569695949554f,0.007113058585673571f,-0.003572880057618022f,-0.0035745245404541492f,0.0003883681492879987f,0.0004443148209247738f,-0.004867881536483765f,-0.00981999933719635f,-0.10460745543241501f,0.04646269604563713f,0.014268174767494202f,-0.06049056723713875f,-0.043906319886446f,-0.0030835685320198536f,-0.002348093083128333f,-0.008411307819187641f,-0.03720350190997124f,0.014096827246248722f,0.03037334978580475f,-0.0452081635594368f,-0.0016883260104805231f,0.010729477740824223f,0.005652233958244324f,0.0025959028862416744f,-0.052074916660785675f,0.018539229407906532f,-0.009249147959053516f,-0.009245878085494041f,-0.005952701438218355f,-0.009555100463330746f,0.0037223149556666613f,-0.006277799140661955f,0.003444711910560727f,0.03513805940747261f,-0.04396669194102287f,0.00881093181669712f,0.014947262592613697f,0.002109054010361433f,0.002788502722978592f,0.0057985554449260235f,0.024874789640307426f,0.042558107525110245f,-0.047490641474723816f,0.004828186705708504f,0.06444929540157318f,0.014399084262549877f,0.0022100224159657955f,-0.00015954083937685937f,2.5653178454376757e-05f,0.009841048158705235f,-0.004919157363474369f,-0.004930401686578989f,-0.039666805416345596f,-0.021545816212892532f,0.0030433116480708122f,0.0014125085435807705f,-0.12147256731987f,-0.015739694237709045f,-0.016265302896499634f,0.03256600350141525f,-0.05393045395612717f,0.027082659304142f,-0.007251167669892311f,0.00459917401894927f,-0.06076192110776901f,-0.03683053329586983f,0.03851862996816635f,-0.001732020522467792f,0.006644018925726414f,-8.47667470225133e-05f,0.002546205185353756f,-0.002382703823968768f,0.02478276938199997f,0.005250842310488224f,-0.0026394701562821865f,-0.0026142599526792765f,-0.0070716808550059795f,-0.015901867300271988f,-0.002618497470393777f,-0.007334951777011156f,-0.04172874987125397f,-0.022092802450060844f,0.01917368918657303f,0.002606143243610859f,0.02034260891377926f,0.010824033990502357f,0.004382089711725712f,-0.001710701733827591f,-0.1101539209485054f,-0.04443245381116867f,0.054263677448034286f,-0.010142937302589417f,0.022888589650392532f,-0.014557627029716969f,0.012699693441390991f,-0.0011970296036452055f,-0.03450725972652435f,0.010690445080399513f,-0.005356630776077509f,-0.005311437416821718f,0.03940820321440697f,0.012060252018272877f,0.008305427618324757f,-0.01810406893491745f,0.006288914941251278f,-0.058741189539432526f,0.019422704353928566f,0.03949363902211189f,-0.027754439041018486f,0.003821392310783267f,-0.0054722740314900875f,0.0008401909726671875f,-0.05416664108633995f,-0.02990402653813362f,-0.026088697835803032f,0.05584599822759628f,-0.025472883135080338f,0.027144458144903183f,-0.0008732455316931009f,-0.000537388666998595f,0.06361115723848343f,0.012298437766730785f,-0.006135594565421343f,-0.006141477730125189f,-0.0054891835898160934f,0.006020619533956051f,-0.0077846599742770195f,-0.008197381161153316f,0.021372806280851364f,-0.07497011870145798f,0.058991093188524246f,0.015566161833703518f,0.008463531732559204f,0.036609288305044174f,0.0017965736333280802f,0.007718442473560572f,-0.06477877497673035f,-0.04117729887366295f,0.008261156268417835f,0.03304489701986313f,-0.0037433006800711155f,-0.009071364998817444f,0.002598464023321867f,0.00017636746633797884f,0.008090443909168243f,0.009098239243030548f,-0.004549125209450722f,-0.0045121763832867146f,-0.015032054856419563f,-0.005887851119041443f,0.005254557356238365f,0.00551198422908783f,-0.132829487323761f,0.031242255121469498f,0.013128383085131645f,-0.04425414651632309f,-0.008841958828270435f,0.02644532173871994f,0.00358711089938879f,0.0034885958302766085f,-0.08198747783899307f,-0.01753237470984459f,0.04239736497402191f,-0.024932773783802986f,-0.010323399677872658f,0.0005739895277656615f,-0.0009832404321059585f,-2.4825332729960792e-05f,-0.03553973510861397f,0.003637905465438962f,-0.0017968956381082535f,-0.0017964498838409781f,0.007206019014120102f,0.018937621265649796f,0.002859422704204917f,0.017399508506059647f,0.00714702345430851f,-0.005525995511561632f,0.015629306435585022f,-0.009982475079596043f,0.0208575539290905f,-0.0007000433397479355f,-0.0007511094445362687f,-0.0047836946323513985f,-0.019635679200291634f,0.013166814111173153f,-0.007367535028606653f,-0.005646087694913149f,0.012177525088191032f,-0.002751393476501107f,0.008499229326844215f,0.0008080309489741921f,0.07057537138462067f,0.01593927852809429f,-0.00796931516379118f,-0.007951682433485985f,0.024082908406853676f,-0.00013389009109232575f,-0.005873737391084433f,-0.005178928840905428f,0.06917398422956467f,-0.05782720446586609f,0.07846149802207947f,-0.02103765308856964f,0.010693426243960857f,0.002808238612487912f,-0.0025373129174113274f,0.00033830286702141166f,-0.08741205185651779f,0.0020617491099983454f,-0.016721706837415695f,0.01465828437358141f,0.016959836706519127f,0.033875953406095505f,-0.00574675714597106f,0.0034734371583908796f,0.03928300365805626f,0.01327431295067072f,-0.006633753888309002f,-0.006646461319178343f,0.02883281372487545f,-0.022676512598991394f,-0.00547281838953495f,-0.003274049609899521f,-0.1587742269039154f,-0.02440732531249523f,0.04458514228463173f,-0.020842744037508965f,-0.02189810201525688f,0.0027627984527498484f,0.00307256611995399f,-0.0027387652080506086f,-0.02497691474854946f,-0.004901057109236717f,0.029773391783237457f,-0.02519892528653145f,-0.010210422798991203f,0.027132784947752953f,-0.006870218552649021f,0.0004423287755344063f,0.037433743476867676f,0.0014708212111145258f,-0.0007082386873662472f,-0.0007253795629367232f,0.01504242792725563f,-0.0018113133264705539f,-0.00990438275039196f,-0.00286792847327888f,0.1035185158252716f,-0.031487297266721725f,-0.06294410675764084f,0.09483139216899872f,0.014267596416175365f,-0.005417726933956146f,0.0044074710458517075f,-0.005040932446718216f,-0.049726661294698715f,0.03021208569407463f,-0.015086297877132893f,-0.015154722146689892f,0.03593427315354347f,0.006747383624315262f,-0.0015678084455430508f,-0.00034002296160906553f,-0.10785400122404099f,0.0016456098528578877f,-0.0008201490854844451f,-0.0008248803205788136f,-0.05031690001487732f,-0.016274159774184227f,0.0018465050961822271f,0.0021286599803715944f,-0.06737260520458221f,0.029607674106955528f,0.10210169106721878f,-0.13226483762264252f,-0.008298578672111034f,0.024709483608603477f,0.008178766816854477f,0.00024489860516041517f,-0.06388410180807114f,-0.030375218018889427f,0.04314858466386795f,-0.012792853638529778f,-0.03805987164378166f,0.0461326502263546f,0.0038866454269737005f,0.00030015938682481647f,0.04748421162366867f,0.0027401787228882313f,-0.0013830760726705194f,-0.0013679461553692818f,-0.009804020635783672f,0.022121496498584747f,0.008196168579161167f,0.012088784016668797f,-0.05769285187125206f,-0.06371599435806274f,0.14363570511341095f,-0.07986488938331604f,-0.0029951492324471474f,-0.0023016794584691525f,-0.007721009198576212f,0.005424818955361843f,0.03836919367313385f,-0.03621037304401398f,0.05671093240380287f,-0.02067389525473118f,0.015385616570711136f,-0.00966090988367796f,-0.0054514165967702866f,-0.0013610556488856673f,-0.014484147541224957f,0.0020876091439276934f,-0.0010359117295593023f,-0.0010348582873120904f,-0.004920720122754574f,0.011833912692964077f,-0.007032569032162428f,0.013921445235610008f,-0.013340193778276443f,-0.034749191254377365f,0.0294629093259573f,0.0047317505814135075f,-0.013641424477100372f,0.0035724821500480175f,0.005906569771468639f,-0.002498875604942441f,0.013862798921763897f,-0.02361544966697693f,0.03045024536550045f,-0.006733970250934362f,-0.026935895904898643f,0.004596247803419828f,0.002246070420369506f,0.0002954109222628176f,-0.025557218119502068f,0.0016352928942069411f,-0.0008049212046898901f,-0.0007980141090229154f,-0.014137226156890392f,0.020485641434788704f,0.0073753539472818375f,-0.010742433369159698f,0.03964244946837425f,-0.10715058445930481f,0.08986017853021622f,0.01747993193566799f,0.017868097871541977f,-0.01694752462208271f,-0.0025281559210270643f,-0.0007774748373776674f,0.01859678141772747f,-0.055900752544403076f,0.0732225775718689f,-0.017287030816078186f,0.041276540607213974f,0.012139779515564442f,-0.0006393283838406205f,0.004137656185775995f,0.056587789207696915f,0.003348915372043848f,-0.001660075387917459f,-0.001654032152146101f,-0.002985503524541855f,-0.007361630909144878f,0.00407324219122529f,-0.0009202679502777755f,-0.08787412196397781f,-0.003643017029389739f,0.05446695536375046f,-0.050776079297065735f,-0.010799120180308819f,0.004565728362649679f,0.014352130703628063f,0.0022910749539732933f,-0.05858175456523895f,-0.0072702993638813496f,-6.30567956250161e-05f,0.007210818585008383f,-0.02719857543706894f,-0.010247773490846157f,0.0016169209266081452f,-7.753114914521575e-05f,0.051451750099658966f,0.003292543115094304f,-0.001641258131712675f,-0.001665731193497777f,-0.0030627415981143713f,0.004121955018490553f,0.0059814960695803165f,0.009912763722240925f,-0.00028315858799032867f,-0.012765439227223396f,0.08110924065113068f,-0.07040641456842422f,-0.012332966551184654f,0.00888463482260704f,-0.0064664315432310104f,-0.007056714966893196f,0.05451785773038864f,0.042696427553892136f,-0.017461733892560005f,-0.025337453931570053f,-0.0006343824788928032f,-0.0016531272558495402f,0.007214858662337065f,-0.009058615192770958f,-0.006854629144072533f,0.0023447535932064056f,-0.0011827114503830671f,-0.0011880217352882028f,0.0493423268198967f,-0.028345266357064247f,-0.01792781427502632f,-0.019707443192601204f,0.2252579778432846f,0.044021621346473694f,-0.04713593050837517f,0.003057680791243911f,-0.013218083418905735f,-0.001045997953042388f,-0.011972334235906601f,0.001019722898490727f,-0.0788675993680954f,0.00688337255269289f,0.011286535300314426f,-0.01815246418118477f,-0.01533922366797924f,-0.007124846335500479f,-0.0032956679351627827f,-6.59633005852811e-05f,-0.11600267887115479f,0.0004759253060910851f,-0.00022117269691079855f,-0.00022993260063230991f,-0.02331383340060711f,0.010955773293972015f,0.002814796520397067f,-0.011090734042227268f,0.06782769411802292f,-0.05552786588668823f,-0.022393733263015747f,0.0778452679514885f,-0.031030721962451935f,-0.0054871924221515656f,0.005329312290996313f,0.005080878268927336f,-0.04214758798480034f,0.007433638907968998f,-0.028754588216543198f,0.02163381688296795f,-0.023203235119581223f,-0.0008299981709569693f,-0.000442151736933738f,-0.001658025081269443f,0.04665353149175644f,0.0031104611698538065f,-0.0015507920179516077f,-0.0015610854607075453f,0.032541174441576004f,-0.004046169575303793f,-0.012403099797666073f,-0.019381986930966377f,0.0014314627042040229f,0.008811204694211483f,-0.0034205138217657804f,-0.005281980149447918f,0.02900453470647335f,-0.006953294854611158f,0.0008372105075977743f,0.000715336762368679f,-0.057535432279109955f,0.00949559360742569f,-0.010243714787065983f,0.0008003205293789506f,-0.007538992445915937f,0.021506642922759056f,-0.0039051277562975883f,0.003193699289113283f,0.011271484196186066f,0.002748216036707163f,-0.0013593914918601513f,-0.001346926437690854f,0.007690875791013241f,0.008776752278208733f,0.0027779003139585257f,0.009413071908056736f,0.004823790397495031f,0.003518369048833847f,0.0482158437371254f,-0.05187288299202919f,-0.027904527261853218f,-0.00976157933473587f,0.0022527759429067373f,-0.0063790371641516685f,-0.019464876502752304f,-0.020633762702345848f,0.04567757621407509f,-0.0252691637724638f,0.012887198477983475f,0.00344220083206892f,-0.0025724414736032486f,-0.003540490521118045f,-0.071156807243824f,0.002531399019062519f,-0.0012687238631770015f,-0.0012959358282387257f,0.010339749045670033f,-0.025267284363508224f,-0.001143903355114162f,-0.014601032249629498f,-0.05156354978680611f,-0.016051875427365303f,-0.05421307310461998f,0.06992914527654648f,0.011417442001402378f,-0.0012191772693768144f,0.006093095522373915f,0.0005449947784654796f,0.10479079931974411f,0.0178308617323637f,-0.007786844857037067f,-0.010049345903098583f,0.017161257565021515f,-0.03495774045586586f,0.0046792724169790745f,-0.004933140240609646f,-0.03499447554349899f,0.0014958977699279785f,-0.0007635435904376209f,-0.0007566987769678235f,0.0481787845492363f,0.013725338503718376f,-0.012857990339398384f,0.00801665335893631f,-0.0840512290596962f,0.006209539715200663f,0.004601700697094202f,-0.010769493877887726f,0.014163810759782791f,0.018012672662734985f,-0.0001635125809116289f,-0.0038970177993178368f,0.0681920200586319f,0.004769358783960342f,0.004986178129911423f,-0.009666558355093002f,-0.010218179784715176f,-0.005150551907718182f,-0.0016958791529759765f,-0.0022228513844311237f,-0.041922036558389664f,0.003842828329652548f,-0.0019384556217119098f,-0.0019442843040451407f,-0.02181200310587883f,-0.01740458235144615f,0.009631718508899212f,-0.0062396046705543995f,0.007624633144587278f,0.06285318732261658f,0.01143566519021988f,-0.07465465366840363f,0.002219327026978135f,-0.03219548612833023f,-0.004385609645396471f,0.009595558047294617f,-0.01936592534184456f,0.005999219138175249f,0.006758871953934431f,-0.012933392077684402f,0.05439933016896248f,-0.0006267317803576589f,0.006736425217241049f,-0.0035400553606450558f,-0.06708090752363205f,0.001305375131778419f,-0.000637456018012017f,-0.0006490601226687431f,-0.010045582428574562f,-0.042938679456710815f,-0.004329622723162174f,-0.009222504682838917f,-0.00959835946559906f,0.01714502088725567f,-0.048427119851112366f,0.03149859234690666f,-0.013179254718124866f,-0.007754433434456587f,0.005987111013382673f,0.0016687215538695455f,-0.006736879236996174f,0.008526249788701534f,0.0035462614614516497f,-0.011825699359178543f,0.0207778699696064f,-0.015102613717317581f,-0.0033677725587040186f,-0.010295835323631763f,0.04923499375581741f,0.0034520465414971113f,-0.0017381219659000635f,-0.0017478934023529291f,0.0012516622664406896f,-0.020398631691932678f,-0.0023044354747980833f,-0.0007966459961608052f,0.16999788582324982f,-0.0361550897359848f,-0.08535774052143097f,0.12055149674415588f,0.02558917924761772f,0.013071809895336628f,-0.0014258641749620438f,0.002801465103402734f,0.019898340106010437f,0.028556615114212036f,-0.0004309682408347726f,-0.028482794761657715f,0.0011652561370283365f,-0.0075603206641972065f,0.002705896506085992f,-0.003147999057546258f,-0.021010419353842735f,0.001154581899754703f,-0.0005806011613458395f,-0.0005893561174161732f,0.0006640258361585438f,0.011737009510397911f,0.0008434266201220453f,0.002161146840080619f,-0.22139059007167816f,0.16938108205795288f,-0.16294027864933014f,-0.007055986672639847f,-0.024022888392210007f,-0.006653710268437862f,0.008785847574472427f,-9.007925109472126e-05f,0.008196195587515831f,0.07414177805185318f,-0.048977624624967575f,-0.025287512689828873f,-0.006519602611660957f,-0.007508512120693922f,0.0024239113554358482f,1.804118619475048e-05f,0.02286454103887081f,0.0029798278119415045f,-0.00149914575740695f,-0.00148078054189682f,-0.036535486578941345f,-0.016833504661917686f,0.0036876860540360212f,0.004610258154571056f,0.08122273534536362f,-0.057664286345243454f,0.10876298695802689f,-0.0513201579451561f,0.021017951890826225f,-0.016623007133603096f,0.0064442353323102f,0.008320641703903675f,-0.001598113332875073f,0.029833633452653885f,-0.015146440826356411f,-0.014754879288375378f,-0.006690470967441797f,0.003337942296639085f,0.0006802466814406216f,-0.008388145826756954f,0.05237048864364624f,0.0033854811917990446f,-0.0017143215518444777f,-0.0016969800926744938f,-0.08456943184137344f,0.013330280780792236f,0.0122431805357337f,0.0007958875503391027f,0.17040768265724182f,0.02968771941959858f,-0.1237226128578186f,0.09327840059995651f,-0.004176125396043062f,0.03958149626851082f,-0.009192593395709991f,0.0021206107921898365f,-0.11249826103448868f,0.04142415151000023f,-0.05126561224460602f,0.009927721694111824f,-0.03330144286155701f,0.011728635057806969f,0.003214139025658369f,0.002338063670322299f,0.05461956933140755f,0.0020310741383582354f,-0.0010195282520726323f,-0.0010335395345464349f,0.00514985853806138f,0.012897775508463383f,-0.0015027940971776843f,-0.0007755416445434093f,0.0055935862474143505f,0.05666094273328781f,-0.051231857389211655f,-0.005676465108990669f,-0.020239075645804405f,-0.0013609338784590364f,-0.0003122792695648968f,0.000122585566714406f,-0.018540197983384132f,0.01568642444908619f,-0.03632486239075661f,0.020749589428305626f,-0.01885957457125187f,0.0211333055049181f,-0.003946271724998951f,-0.0010034817969426513f,0.023066941648721695f,0.0026566009037196636f,-0.001325169112533331f,-0.001333483960479498f,0.016235660761594772f,-0.002876157406717539f,-9.879180288407952e-05f,-0.01635822094976902f,-0.01975303515791893f,-0.07873514294624329f,0.0472901314496994f,0.031148860231041908f,0.0005107831093482673f,0.0033809286542236805f,0.004670922178775072f,-0.0009564566425979137f,0.03062959387898445f,-0.002879659179598093f,0.02803153172135353f,-0.025791844353079796f,-0.00401617307215929f,0.012381217442452908f,-0.0012093447148799896f,-0.001982218585908413f,0.00944390520453453f,0.003631897736340761f,-0.0018394538201391697f,-0.0018179294420406222f,-0.015503891743719578f,-0.0014163823798298836f,0.015406015329062939f,0.004397221375256777f,-0.03589264675974846f,0.011214101687073708f,0.012827844358980656f,-0.024461645632982254f,-0.016185786575078964f,-0.001413908670656383f,0.004700636491179466f,-0.0010807330254465342f,-0.016811655834317207f,0.0699513852596283f,-0.05026838928461075f,-0.019778000190854073f,-0.016340509057044983f,0.012110741809010506f,0.003035634756088257f,-0.0033076622057706118f,-0.01830589398741722f,0.0020501192193478346f,-0.001006227801553905f,-0.0010289112105965614f,0.04724409058690071f,-0.011787845753133297f,-0.010811745189130306f,0.018875591456890106f,0.05602671578526497f,-0.0028756032697856426f,0.037077099084854126f,-0.03452558442950249f,0.007148250006139278f,-0.007852782495319843f,0.003004638245329261f,-0.0024397128727287054f,0.005260446108877659f,0.018566973507404327f,-9.016579133458436e-05f,-0.018736938014626503f,0.02821728214621544f,-0.033976588398218155f,0.007504279259592295f,-0.002780931070446968f,0.0029535831417888403f,0.0018412377685308456f,-0.0009562179329805076f,-0.0009168785763904452f,-0.002541216090321541f,0.0012321984395384789f,-0.0038707945495843887f,-0.0005494561628438532f,-0.015382747165858746f,0.028528867289423943f,-0.05105205252766609f,0.022508028894662857f,0.013122940436005592f,0.0004924838431179523f,-0.015979330986738205f,0.0045873792842030525f,-0.03112286701798439f,0.006328405346721411f,0.006475921720266342f,-0.01284786220639944f,0.002284783637151122f,-0.024189293384552002f,7.14245397830382e-05f,-0.0025367410853505135f,-0.05177215486764908f,0.0028743017464876175f,-0.0014247787185013294f,-0.0014236054848879576f,-0.0021383799612522125f,0.003539134282618761f,-0.004124357830733061f,0.01721358485519886f,0.006433251313865185f,-0.034116435796022415f,0.0018211150309070945f,0.032467592507600784f,0.005651412066072226f,-0.009616576135158539f,0.0013802163302898407f,0.0005136785912327468f,0.0701286643743515f,-0.017016928642988205f,0.01856805942952633f,-0.001560763455927372f,-0.011798813939094543f,0.02121211588382721f,0.010045754723250866f,-0.000730721338186413f,0.015361165627837181f,0.0020083098206669092f,-0.0010104497196152806f,-0.0009911010274663568f,-0.005570768844336271f,0.0005032895714975893f,0.011507460847496986f,0.013372641056776047f,-0.06106652319431305f,0.00794832780957222f,0.025303035974502563f,-0.032871589064598083f,0.00730179576203227f,0.016251377761363983f,-0.00255455169826746f,0.006101503968238831f,0.0333009697496891f,0.008836519904434681f,0.01648944802582264f,-0.025618700310587883f,-0.04167007654905319f,-0.0037143202498555183f,0.008051550015807152f,-0.005001724697649479f,0.03324592858552933f,0.0019339476712048054f,-0.0009450106881558895f,-0.0009862647857517004f,-0.007628914434462786f,0.008435704745352268f,0.011906136758625507f,0.0006441919831559062f,0.1779169738292694f,0.18241700530052185f,-0.11005392670631409f,-0.07278730720281601f,-0.022475657984614372f,-0.02284177765250206f,0.010157446376979351f,-0.007461930625140667f,-0.06047441065311432f,-0.0016220399411395192f,0.010690701194107533f,-0.00899329874664545f,0.009163480252027512f,0.012059849686920643f,-0.0011677030706778169f,8.454023918602616e-05f,-0.06406357139348984f,0.001064174808561802f,-0.0005260725738480687f,-0.0005404045805335045f,0.01968107745051384f,-0.004539704881608486f,0.0022184995468705893f,0.0032596031669527292f,-0.01688488945364952f,-0.07928682863712311f,0.008086200803518295f,0.07250478118658066f,0.022868502885103226f,0.018814396113157272f,0.0008927175076678395f,-0.000499967485666275f,-0.0788995698094368f,-0.020841779187321663f,-0.004859458189457655f,0.025843676179647446f,0.0018761103274300694f,0.036098018288612366f,0.005848870612680912f,-0.0037801177240908146f,-0.026677800342440605f,0.0021783753763884306f,-0.001092026475816965f,-0.0010674174409359694f,0.012067344039678574f,-0.020056286826729774f,-0.008035131730139256f,-0.010308004915714264f,-0.029180405661463737f,0.004874991253018379f,-0.011304355226457119f,0.006990230642259121f,-0.006586018484085798f,0.006547739263623953f,-0.001559908501803875f,0.006140279117971659f,-0.01133812591433525f,-0.00205812300555408f,0.0015326642896980047f,0.0004413925635162741f,-0.008952958509325981f,0.005405409727245569f,-0.0019216681830585003f,0.0033148936927318573f,0.03737649321556091f,0.0035071310121566057f,-0.0017663320759311318f,-0.0017546871677041054f,0.020397895947098732f,-0.02017265371978283f,0.0021359685342758894f,-0.0015588407404720783f,0.07726649194955826f,0.06809929758310318f,-0.0641055554151535f,-0.004046401008963585f,0.026930812746286392f,0.00021352063049562275f,0.0039680213667452335f,-0.007985491305589676f,-0.019072875380516052f,0.03614699840545654f,-0.021976890042424202f,-0.014253041706979275f,0.02206462435424328f,0.018224157392978668f,0.0006801399285905063f,-0.004189864732325077f,0.03710542991757393f,0.003118055174127221f,-0.001566449529491365f,-0.0015783198177814484f,0.0037267254665493965f,0.025342468172311783f,-0.0019280415726825595f,0.007852680049836636f,0.04932370409369469f,0.031850751489400864f,0.07840193808078766f,-0.11055691540241241f,0.002225373638793826f,0.0044764322228729725f,-0.007090092170983553f,-0.004272800870239735f,-0.037877943366765976f,-0.025416530668735504f,0.04261551424860954f,-0.017494747415184975f,-0.0014738874742761254f,0.020829951390624046f,-0.003668777411803603f,0.0026212481316179037f,-0.005416462197899818f,0.0038576715160161257f,-0.0019443340133875608f,-0.0019029391696676612f,-0.0034316624514758587f,-0.02857029251754284f,-0.01076630037277937f,-0.01043025590479374f,-0.011899294331669807f,-0.038682736456394196f,0.06509815901517868f,-0.026222676038742065f,-0.005657882429659367f,-0.00590905174612999f,0.0007732446538284421f,0.00023984310973901302f,-0.07239454984664917f,0.052582744508981705f,-0.029583007097244263f,-0.023183060809969902f,0.023766808211803436f,-0.0206038448959589f,0.0032950195018202066f,-0.011378736235201359f,-0.011481870897114277f,0.002227801363915205f,-0.0010980729712173343f,-0.001091760117560625f,-0.030778566375374794f,0.0059844208881258965f,0.00849607028067112f,0.019430657848715782f,0.043874435126781464f,0.16799432039260864f,-0.013045157305896282f,-0.15519395470619202f,-0.011406167410314083f,0.011498762294650078f,0.0037404499016702175f,-0.0015473677776753902f,0.04504004865884781f,0.029893435537815094f,-0.02033541165292263f,-0.009440585039556026f,0.0006890526856295764f,0.00964426714926958f,0.0031853565014898777f,0.0017505355644971132f,-0.05251658335328102f,0.002250870456919074f,-0.0011198197025805712f,-0.0011188804637640715f,-0.00840387586504221f,0.015938786789774895f,0.011935291811823845f,0.00505031319335103f,0.01412054430693388f,-0.03591204807162285f,0.05214771628379822f,-0.01628747209906578f,-0.026352781802415848f,-0.00976584292948246f,0.005804064217954874f,0.00377820897847414f,0.054724279791116714f,0.009841464459896088f,0.01814732700586319f,-0.028231080621480942f,-0.020933955907821655f,0.002221138449385762f,0.006702933460474014f,-0.005779915489256382f,0.025174077600240707f,0.0030814148485660553f,-0.0015254957834258676f,-0.0015500116860494018f,-0.016304846853017807f,-0.016135569661855698f,-0.007295374292880297f,0.001678544096648693f,0.049023427069187164f,0.015934037044644356f,-0.0223010815680027f,0.006405466701835394f,-0.012323330156505108f,0.01531135942786932f,0.007776312064379454f,0.007516557816416025f,-0.13118547201156616f,-0.02424701303243637f,0.02844308689236641f,-0.004143259488046169f,-0.013971813954412937f,-0.0166503693908453f,0.0027297830674797297f,-0.0004438186588231474f,0.07230957597494125f,0.001983694266527891f,-0.0009948250371962786f,-0.0009701337548904121f,-0.0007397414883598685f,-0.02976200170814991f,-0.004852571990340948f,-0.007321025710552931f,0.008296169340610504f,-0.04155987128615379f,0.03646356612443924f,0.005040165036916733f,-0.009737816639244556f,-0.007109490223228931f,-9.495590347796679e-06f,-0.0015749133890494704f,-0.08058222383260727f,0.0027484367601573467f,0.00376685569062829f,-0.006532398518174887f,0.011473114602267742f,-0.017416829243302345f,-0.006429409608244896f,0.0008685425273142755f,0.003910294733941555f,0.0006925420602783561f,-0.00036631186958402395f,-0.0003481658932287246f,0.05604977160692215f,-0.008632481098175049f,-0.024312812834978104f,-0.021885517984628677f,-0.11169300973415375f,0.021429577842354774f,-0.06365393102169037f,0.04212093725800514f,-0.007380698341876268f,-0.01371911633759737f,0.00044672703370451927f,-0.004892558790743351f,0.041096318513154984f,0.0033002649433910847f,0.005471677519381046f,-0.00880101416260004f,0.006600162945687771f,0.02517635002732277f,0.0013842338230460882f,0.0007032467983663082f,-0.03981222212314606f,0.0012929189251735806f,-0.0006144776125438511f,-0.0006185357924550772f,-0.0087583614513278f,0.01551802083849907f,0.006413572002202272f,0.007408104836940765f,-0.07347266376018524f,0.06392674893140793f,-0.047948528081178665f,-0.01569228433072567f,0.010670383460819721f,0.003003556514158845f,-0.001716245198622346f,0.003199091413989663f,-0.09629683941602707f,0.003507905174046755f,-0.008629333227872849f,0.005263184662908316f,0.005506443791091442f,0.009118674322962761f,-0.0027099160943180323f,-0.008875714614987373f,0.02506105788052082f,0.0029671897646039724f,-0.001462776679545641f,-0.001472157659009099f,-0.02771514654159546f,-0.015478143468499184f,0.0046453033573925495f,0.007570622023195028f,-0.08891106396913528f,-0.0030455379746854305f,0.0020048737060278654f,0.0009923287434503436f,-0.022871803492307663f,-0.013050394132733345f,0.0025579535868018866f,0.002820187248289585f,-0.0428047738969326f,0.007207827176898718f,-0.003876010188832879f,-0.0031775650568306446f,0.025561096146702766f,0.005022789351642132f,-0.008085867390036583f,-0.0004969695582985878f,0.10279706865549088f,0.0014261846663430333f,-0.0006794522050768137f,-0.0006990052061155438f,-0.013561597093939781f,0.019710328429937363f,0.0043851714581251144f,-0.0009733711485750973f,-0.08035480976104736f,-0.08989465236663818f,0.016025297343730927f,0.07322133332490921f,-0.008486072532832623f,0.01393464021384716f,0.005398860666900873f,0.0006645626854151487f,0.018167663365602493f,0.02554941177368164f,0.0033398421946913004f,-0.029019970446825027f,-0.014510011300444603f,0.017856214195489883f,-0.0017396730836480856f,-0.0070203146897256374f,-0.08842120319604874f,0.002530235331505537f,-0.0012562568299472332f,-0.0012817449169233441f,0.015255749225616455f,-0.004045803099870682f,0.015965761616826057f,-0.0036652185954153538f,-0.09114281833171844f,-0.04385589808225632f,-0.020751602947711945f,0.06411497294902802f,-9.914517431752756e-05f,0.0016050083795562387f,0.0013067113468423486f,0.0044782646000385284f,-0.07364990562200546f,0.019232794642448425f,-0.0012686963891610503f,-0.017979232594370842f,-0.013035262934863567f,0.01333568338304758f,0.004220020025968552f,-0.0006110426038503647f,-0.07527560740709305f,0.0022809538058936596f,-0.001167706330306828f,-0.0011553367367014289f,0.023471076041460037f,-0.01857609860599041f,0.004774802830070257f,0.000755112967453897f,0.0455273799598217f,-0.009277758188545704f,0.10549788177013397f,-0.09613814204931259f,0.002036591526120901f,0.009825050830841064f,-0.005149361677467823f,0.0038316810969263315f,0.003279041964560747f,0.004326421767473221f,0.01887417584657669f,-0.02307427115738392f,0.01878308318555355f,0.030032113194465637f,-0.0031228107400238514f,-0.0011254391865804791f,0.013590680435299873f,0.0038371291011571884f,-0.0019372175447642803f,-0.0019178843358531594f,-0.00023242073075380176f,-0.017629647627472878f,-0.007985792122781277f,0.01489983219653368f,-0.04667602479457855f,-0.06467512249946594f,0.11091545224189758f,-0.046316299587488174f,0.016927551478147507f,-0.021422967314720154f,0.00195839605294168f,-0.0003370010817889124f,-0.007847657427191734f,-0.009123707190155983f,0.035257935523986816f,-0.02649291232228279f,-0.014257676899433136f,-0.007581820245832205f,0.003283923491835594f,-0.007980960421264172f,-0.06293120235204697f,0.002548172138631344f,-0.0012385263107717037f,-0.0012422020081430674f,-0.03321418538689613f,0.03144390508532524f,-0.00022190791787579656f,0.021526481956243515f,-0.06922876089811325f,0.0436830148100853f,0.010103181935846806f,-0.05657769739627838f,0.006673287134617567f,-0.006541100330650806f,0.00308514223434031f,-0.004223906900733709f,-0.04030590131878853f,0.0480467826128006f,-0.03342602401971817f,-0.014570314437150955f,0.018260829150676727f,0.014826394617557526f,-0.0007229268085211515f,-0.0012256084010004997f,-0.08383629471063614f,0.0022571261506527662f,-0.001115463674068451f,-0.0011603356106206775f,-0.008821401745080948f,-0.0043281689286231995f,-0.007551091257482767f,-0.0041764662601053715f,0.14373397827148438f,-0.025379978120326996f,0.019352436065673828f,0.005715034436434507f,-0.017015263438224792f,-0.034880418330430984f,-0.011306611821055412f,0.0015666020335629582f,0.0744951143860817f,0.03087054193019867f,-0.019174931570887566f,-0.011657618917524815f,0.01221954170614481f,-0.017626119777560234f,-0.006812746170908213f,0.009795095771551132f,-0.07877208292484283f,0.0026554656215012074f,-0.0013075342867523432f,-0.0012959789019078016f,-0.0069405697286129f,-0.024062860757112503f,0.014222963713109493f,0.0006607124814763665f,-0.10167764872312546f,0.048847079277038574f,-0.005397197790443897f,-0.043038927018642426f,0.005766000133007765f,0.013740616850554943f,-0.0027337614446878433f,-0.008281465619802475f,0.0694182962179184f,0.055596791207790375f,-0.06651090830564499f,0.010948283597826958f,0.033473048359155655f,-0.03225739300251007f,0.0062815104611217976f,-0.0029145865701138973f,-0.025885900482535362f,0.002928126836195588f,-0.0014273402048274875f,-0.0014312502462416887f,0.028168238699436188f,0.03512517735362053f,-0.0014503269921988249f,0.007827266119420528f,-0.04319438710808754f,-0.06614512950181961f,0.12186340987682343f,-0.058066871017217636f,0.019091187044978142f,0.031779009848833084f,0.0010647292947396636f,0.003979039844125509f,0.06449372321367264f,0.030670223757624626f,-0.0024678674526512623f,-0.028352651745080948f,0.010139514692127705f,0.0011890657478943467f,0.008501310832798481f,-0.006714202929288149f,-0.020227909088134766f,0.0025708272587507963f,-0.001274009933695197f,-0.0013027595123276114f,-0.051985736936330795f,-0.02376195602118969f,-0.025864645838737488f,-0.015714263543486595f,0.14551420509815216f,0.01985536515712738f,-0.009872711263597012f,-0.00780255114659667f,-0.025061795487999916f,0.005466856993734837f,-0.003623414319008589f,0.0015609757974743843f,-0.05862562730908394f,-0.012959137558937073f,0.023389695212244987f,-0.009943748824298382f,-0.010535224340856075f,-0.0076986211352050304f,-0.01712845265865326f,-0.003276427276432514f,-0.05850231647491455f,0.0015740636736154556f,-0.0007922033546492457f,-0.0007687160396017134f,0.0010170750319957733f,-0.025802182033658028f,-0.015424808487296104f,-0.021527240052819252f,0.015266851522028446f,-0.11465009301900864f,-0.08596011251211166f,0.20052170753479004f,0.03132091835141182f,-0.029501065611839294f,0.005932330619543791f,-0.0005332197179086506f,-0.00011838671343866736f,0.007797359488904476f,-0.037253476679325104f,0.0293880607932806f,0.012856378220021725f,0.010994615033268929f,-0.002443186240270734f,-0.0009318035445176065f,-0.05115572363138199f,0.0035475734621286392f,-0.0017662507016211748f,-0.0017744656652212143f,-0.020633729174733162f,0.036993466317653656f,-0.015454067848622799f,0.011012000031769276f,-0.0331018790602684f,0.008364622481167316f,0.046714335680007935f,-0.054909683763980865f,-0.009232517331838608f,-0.012570151127874851f,-0.0066640120930969715f,-0.0007686049211770296f,-0.023861316964030266f,-0.02212490141391754f,0.026801584288477898f,-0.004665694199502468f,0.005590029526501894f,0.006198214367032051f,-0.0065579889342188835f,0.003974048886448145f,-0.0025576904881745577f,0.0025308714248239994f,-0.0012824231525883079f,-0.0012859813868999481f,-0.021890876814723015f,0.015679839998483658f,-0.006233294494450092f,0.01068209484219551f,0.05600574240088463f,0.021561138331890106f,0.0006327448645606637f,-0.02186700329184532f,-0.02584150619804859f,0.013218546286225319f,-0.002750735729932785f,-0.007422777358442545f,-0.0049959877505898476f,0.016867294907569885f,-0.0017850894946604967f,-0.015212503261864185f,0.001836938550695777f,0.024498099461197853f,0.0030329350847750902f,0.0017594146775081754f,-0.10482456535100937f,0.00400997931137681f,-0.002019117586314678f,-0.0020160593558102846f,0.009995497763156891f,-0.0009709830628708005f,-0.013949304819107056f,0.008664741180837154f,-0.020710354670882225f,0.18056155741214752f,-0.15976707637310028f,-0.021115217357873917f,0.03382332995533943f,0.014902119524776936f,-0.00475752679631114f,-0.0014082761481404305f,0.05891671031713486f,0.014877155423164368f,-0.0037985146045684814f,-0.011085545644164085f,-0.014383853413164616f,0.0061294641345739365f,-0.00024408000172115862f,-0.004234006628394127f,-0.022680139169096947f,0.0029986202716827393f,-0.0014743161154910922f,-0.0014871291350573301f,0.021338699385523796f,-0.013005968183279037f,0.013034644536674023f,-0.0033969327341765165f,0.07762260735034943f,0.045875515788793564f,-0.0529400035738945f,0.007226597052067518f,0.010545567609369755f,-0.03250205144286156f,-0.0002103899314533919f,-0.008200587704777718f,0.01019248552620411f,0.050181008875370026f,-0.05155596137046814f,0.001503723324276507f,0.02167961187660694f,-0.019329611212015152f,0.004055046942085028f,-0.0026517335791140795f,-0.007774573750793934f,0.0035854291636496782f,-0.0017900366801768541f,-0.001775972661562264f,0.038876961916685104f,-0.014335299842059612f,0.011347867548465729f,0.0028831660747528076f,-0.03519134223461151f,0.0370483435690403f,0.030869746580719948f,-0.06787612289190292f,0.005414291750639677f,-0.059339992702007294f,0.004954774864017963f,-0.016296278685331345f,-0.024089597165584564f,0.026435475796461105f,-0.005859029944986105f,-0.020822543650865555f,-0.03328733146190643f,-0.008694611489772797f,-0.00200604647397995f,-0.0005817569908685982f,-0.11104567348957062f,0.0016342788003385067f,-0.0008054185309447348f,-0.0008037869702093303f,0.000656809366773814f,-0.01583799347281456f,-0.004934145603328943f,0.007247352972626686f,0.037623628973960876f,0.08410577476024628f,-0.06276430189609528f,-0.02104230411350727f,0.012554490938782692f,-0.00881801825016737f,0.0002761345240287483f,-0.0017290128162130713f,0.05163167789578438f,-0.02994048036634922f,0.0408509336411953f,-0.010698395781219006f,0.012758061289787292f,0.01090923510491848f,-0.00043897496652789414f,-0.007957554422318935f,-0.053082823753356934f,0.0031041959300637245f,-0.0015555607387796044f,-0.0015554494457319379f,0.023494193330407143f,-0.015940004959702492f,0.012730898335576057f,0.0038453522138297558f,-0.15016621351242065f,-0.014336206018924713f,-0.010155814699828625f,0.024497367441654205f,0.002908958587795496f,-0.005545307416468859f,0.002912722062319517f,-0.004876837134361267f,0.01791583187878132f,0.02355334535241127f,-0.002410339657217264f,-0.021229898557066917f,-0.013567212969064713f,0.016709236428141594f,0.005635092034935951f,-0.0009647111291997135f,-0.0447719544172287f,0.0019107499392703176f,-0.0009819237748160958f,-0.0009603484068065882f,0.003985599614679813f,-0.02086087316274643f,-4.271799843991175e-05f,0.006188202183693647f,-0.13129356503486633f,0.12498999387025833f,-0.0678521916270256f,-0.05739812180399895f,-0.0135033605620265f,0.016831718385219574f,-0.0030813661869615316f,-0.004660635255277157f,0.003436769125983119f,0.06419146806001663f,-0.03517854958772659f,-0.029039256274700165f,-0.01815948262810707f,0.017261777073144913f,-0.00017438035865779966f,-0.005306891165673733f,-0.003479881677776575f,0.0033764576073735952f,-0.0016814230475574732f,-0.0016878977185115218f,-0.004388011526316404f,-0.0013408631784841418f,-0.0034548225812613964f,-0.012006384320557117f,0.0876062884926796f,-0.07296451181173325f,0.09173984080553055f,-0.018379850313067436f,0.007046896498650312f,0.03656858950853348f,0.015039682388305664f,0.00015392160275951028f,-0.024993961676955223f,0.053117651492357254f,-0.041943516582250595f,-0.011095672845840454f,0.007828261703252792f,0.006792635656893253f,-0.010892878286540508f,-0.001468004542402923f,0.02331295609474182f,0.003894360503181815f,-0.0019628636073321104f,-0.0019427931401878595f,0.02749728225171566f,0.010536233894526958f,0.014559835195541382f,-0.0027925290632992983f,-0.035134267061948776f,0.11512112617492676f,-0.13844603300094604f,0.023759178817272186f,0.004811042919754982f,0.021677041426301003f,0.0022782422602176666f,-0.0029720296151936054f,-0.14607824385166168f,0.012748781591653824f,-0.029761716723442078f,0.01710866205394268f,-0.0069397068582475185f,-0.009346948936581612f,0.0018495647236704826f,-0.003709133481606841f,-0.0726855918765068f,0.002734266221523285f,-0.0013479853514581919f,-0.0013820744352415204f,0.0058820839039981365f,0.005355777218937874f,-0.0029313901904970407f,0.0011936784721910954f,0.0274422038346529f,0.09733866155147552f,-0.03997868299484253f,-0.058789368718862534f,-0.0050999438390135765f,-0.006344088353216648f,-0.0031830642838031054f,0.003674575360491872f,0.11611785739660263f,0.03161490708589554f,-0.03189258277416229f,0.0002627400972414762f,0.04435887187719345f,0.00022886339866090566f,0.0003061452298425138f,-0.0009197721956297755f,0.007179041393101215f,0.0034621930681169033f,-0.0017058216035366058f,-0.0017136139795184135f,0.001933403662405908f,0.005880333017557859f,0.000489821657538414f,-0.01393178291618824f,-0.050230011343955994f,-0.050274841487407684f,0.02094161882996559f,0.029583726078271866f,0.0159305427223444f,0.012226511724293232f,0.00844893790781498f,-0.002129946369677782f,-0.04474285989999771f,0.03509477153420448f,-0.020431702956557274f,-0.015064341947436333f,0.01737082004547119f,0.0005106765893287957f,0.0010181062389165163f,-0.002357770223170519f,0.014464540407061577f,0.00502071063965559f,-0.0025058952160179615f,-0.0025384908076375723f,0.005575860384851694f,-0.000502595619764179f,0.00866012368351221f,-0.0028525011148303747f,-0.007074909284710884f,0.06476961076259613f,-0.06585495173931122f,0.0002234479907201603f,0.010493801906704903f,0.012751479633152485f,0.006270963698625565f,-0.0017254806589335203f,0.1111016497015953f,0.06710550934076309f,-0.03977588191628456f,-0.02745075896382332f,0.015134509652853012f,0.02883780188858509f,0.0029463728424161673f,-0.002409270266070962f,-0.029018031433224678f,0.0018824049038812518f,-0.0009433218510821462f,-0.0009458625572733581f,-0.05165645852684975f,-0.011260269209742546f,-0.00811233650892973f,0.0023570782504975796f,-0.06350226700305939f,0.1198972538113594f,-0.05953997001051903f,-0.06207713857293129f,-0.018702853471040726f,-0.029852693900465965f,0.0014617206761613488f,0.002113972557708621f,-0.009896713308990002f,0.07996079325675964f,-0.06363938003778458f,-0.01633630134165287f,-0.019379088655114174f,-0.0038191925268620253f,-0.017305469140410423f,-0.0035559427924454212f,-0.08805963397026062f,0.002175857312977314f,-0.0010926430113613605f,-0.0010826434008777142f,0.019534973427653313f,-0.012372422032058239f,-0.013390987180173397f,-0.004895999561995268f,-0.06918416172266006f,-0.036668624728918076f,-0.1197432205080986f,0.15689383447170258f,0.034354161471128464f,-0.013102441094815731f,-0.0008813962340354919f,0.0023505077697336674f,0.1269645243883133f,0.013966712169349194f,-0.011499235406517982f,-0.0022906456142663956f,-0.015797976404428482f,0.008110061287879944f,0.0007842606282792985f,-0.004812553524971008f,-0.08060484379529953f,0.004071794915944338f,-0.002011407632380724f,-0.002029509749263525f,0.001108313794247806f,-0.04993654042482376f,-0.03273307904601097f,-0.007453246507793665f,-0.08095695078372955f,-0.01802460476756096f,0.015392517670989037f,0.00140871643088758f,-0.029483012855052948f,-0.015258867293596268f,-0.014678188599646091f,-0.0046905833296477795f,-0.03604384511709213f,0.017795689404010773f,-0.022581199184060097f,0.004885809030383825f,0.013492612168192863f,-0.026648540049791336f,0.01570078171789646f,-0.019377322867512703f,-0.09223555028438568f,0.0023923844564706087f,-0.001178413163870573f,-0.0011835861951112747f,-0.006304861977696419f,-0.020972074940800667f,-0.025445863604545593f,0.007783391047269106f,0.0036001955159008503f,0.018482277169823647f,0.022627057507634163f,-0.041457559913396835f,0.017076030373573303f,-0.03174407407641411f,-0.00292154960334301f,0.00198551919311285f,-0.09569592773914337f,0.05543132126331329f,-0.035569269210100174f,-0.019891999661922455f,0.004401931539177895f,-0.0022316770628094673f,0.0029133684001863003f,0.0004986217827536166f,-0.08907593786716461f,0.0017013546312227845f,-0.0008439365192316473f,-0.0008480029064230621f,0.01718244142830372f,-0.007593488786369562f,-0.0026113928761333227f,-0.009898747317492962f,0.1408052295446396f,0.17895691096782684f,-0.12695574760437012f,-0.05182920768857002f,0.024205438792705536f,0.012842214666306973f,3.3307605917798355e-07f,-0.002337175887078047f,0.03260388225317001f,-0.01575443521142006f,0.029825923964381218f,-0.013946983963251114f,0.006697867065668106f,0.00203324924223125f,0.0020727599039673805f,0.0016143341781571507f,-0.027712447568774223f,0.0012973848497495055f,-0.0006398565019480884f,-0.0006304661510512233f,0.02569631300866604f,0.003071411047130823f,-0.0035242969170212746f,-0.009852897375822067f,0.021681759506464005f,-0.12388052046298981f,0.003519717138260603f,0.1204088032245636f,-0.010855733416974545f,-0.0328202024102211f,-0.006969758775085211f,-0.0035930608864873648f,-0.09457533061504364f,-0.012339010834693909f,-0.009295501746237278f,0.021670838817954063f,-0.03350827470421791f,-0.005489268805831671f,0.0031144313979893923f,-0.0035689615178853273f,-0.0026046005077660084f,0.0026350785046815872f,-0.001299749012105167f,-0.0013265782035887241f,-0.0004815793363377452f,-0.015537253580987453f,-0.00045399737427942455f,-0.025469783693552017f,-0.031749285757541656f,0.04377754405140877f,-0.014584813266992569f,-0.02915860526263714f,-0.005733147729188204f,-0.017314324155449867f,-0.0037135183811187744f,0.009425346739590168f,-0.033529601991176605f,0.028089813888072968f,-0.022512763738632202f,-0.00569992745295167f,-0.0016779816942289472f,-0.004566402640193701f,-0.011242127977311611f,-0.0021177702583372593f,0.020351871848106384f,0.0047911955043673515f,-0.0023739307653158903f,-0.0024020757991820574f,0.006022067274898291f,0.001722061773762107f,-0.003061849158257246f,0.004991598427295685f,-0.09419423341751099f,0.001775685464963317f,-0.023069068789482117f,0.021444132551550865f,-0.046410273760557175f,-0.011197018437087536f,-0.002823777962476015f,-0.0020847998093813658f,0.07109395414590836f,0.04837298020720482f,-0.02253543771803379f,-0.02593228965997696f,-0.009933088906109333f,0.027382221072912216f,-4.2752595618367195e-05f,0.007012791000306606f,-0.048607904464006424f,0.002870206255465746f,-0.0014425985282287002f,-0.0014222979079931974f,0.04540782421827316f,0.016697270795702934f,-0.0038782123010605574f,0.0008272588020190597f,-0.020435843616724014f,-0.012928823940455914f,0.08178019523620605f,-0.06810879707336426f,0.0017968547763302922f,0.004345159512013197f,0.0005062979180365801f,-0.004867130424827337f,-0.031016001477837563f,-0.008620209991931915f,0.024437636137008667f,-0.015785133466124535f,-0.00025516413734294474f,-0.021925590932369232f,-0.004018639679998159f,-0.007430320605635643f,0.04544826224446297f,0.004509184509515762f,-0.002247182885184884f,-0.0022365860641002655f,-0.0005318275070749223f,0.010804159566760063f,-0.014265513978898525f,-0.0028265516739338636f,0.03025290183722973f,-0.048735108226537704f,-0.013733063824474812f,0.0626690462231636f,0.006524753291159868f,0.007078241556882858f,-0.0011933555360883474f,0.0039980411529541016f,0.10711711645126343f,0.073630191385746f,-0.03876258805394173f,-0.035172056406736374f,-0.05340664088726044f,-0.017297817394137383f,0.00506601994857192f,-0.007235203869640827f,-0.058766260743141174f,0.0022159970831125975f,-0.0011083617573603988f,-0.0011189555516466498f,0.028292451053857803f,0.007886532694101334f,0.002320151077583432f,0.009118017740547657f,0.021736662834882736f,-0.09385685622692108f,0.04050802066922188f,0.05324247106909752f,-0.000567678187508136f,-0.00471071433275938f,0.002995719900354743f,-0.009391439147293568f,0.071495421230793f,-0.0028873663395643234f,-0.016249315813183784f,0.019217444583773613f,0.014891999773681164f,0.035757239907979965f,0.006013781763613224f,0.003470327705144882f,-0.10109526664018631f,0.0032235323451459408f,-0.001595025765709579f,-0.0016132246237248182f,-0.03168239817023277f,0.0005084951990284026f,0.00031012107501737773f,-0.004792509600520134f,0.08741637319326401f,-0.05308501049876213f,0.007401091046631336f,0.04627695679664612f,0.01610218733549118f,-0.0006233828025870025f,-0.00023606372997164726f,-0.008358754217624664f,-0.059609413146972656f,0.0008263082709163427f,0.02702980302274227f,-0.028001397848129272f,0.013762572780251503f,0.022076498717069626f,-0.0016231206245720387f,-0.000523971626535058f,-0.1123022586107254f,0.002482987241819501f,-0.0012448043562471867f,-0.0012234678724780679f,0.012859076261520386f,-0.015367561019957066f,-0.0009418812696821988f,0.0051746428944170475f,-0.13261781632900238f,0.03901779279112816f,-0.05934242904186249f,0.020565088838338852f,0.007735233288258314f,-0.010775060392916203f,0.006199738476425409f,-0.002877570688724518f,-0.08174022287130356f,0.015850884839892387f,-0.013709460385143757f,-0.0020691989921033382f,0.005004233680665493f,0.0004469345440156758f,-0.003155599581077695f,-0.004125551786273718f,-0.08497696369886398f,0.0025110247079283f,-0.00125915149692446f,-0.001258286414667964f,0.0014656963758170605f,-0.01329330075532198f,0.010332474485039711f,-0.001209558336995542f,-0.039621371775865555f,-0.06954801082611084f,0.021109972149133682f,0.04821450635790825f,0.004986686632037163f,0.018190711736679077f,-0.005521672777831554f,0.007412082049995661f,-0.02086801826953888f,0.025675464421510696f,-0.0145459845662117f,-0.011181739158928394f,-0.01614275574684143f,-0.00423429487273097f,0.004744437988847494f,0.004894369747489691f,-0.017794711515307426f,0.001777426339685917f,-0.0008976159151643515f,-0.0009043666068464518f,-0.024415403604507446f,-0.034700457006692886f,0.008299639448523521f,-0.02238030545413494f,-0.13699914515018463f,0.0647333562374115f,0.012863703072071075f,-0.07783565670251846f,-0.025858188048005104f,-0.012860242277383804f,0.0018665425013750792f,-0.00837707333266735f,0.0069823418743908405f,0.017749277874827385f,-0.011925442144274712f,-0.005896884016692638f,-0.008211120031774044f,-0.012603942304849625f,-0.010878805071115494f,-0.005170576740056276f,-0.031035827472805977f,0.0014438832877203822f,-0.0007106594275683165f,-0.0007321617449633777f,-0.017334403470158577f,-3.846374966087751e-06f,-0.00460390979424119f,0.00040316287777386606f,-0.046946413815021515f,0.011464513838291168f,-0.03398185223340988f,0.02284269966185093f,0.0018373150378465652f,0.019055208191275597f,-0.006433457601815462f,0.008611381985247135f,0.0397469587624073f,0.028654055669903755f,-0.03193047270178795f,0.0033120689913630486f,0.010214539244771004f,0.004830657970160246f,0.001257738214917481f,-0.007095648907124996f,-0.03842140734195709f,0.004522539209574461f,-0.0022467004600912333f,-0.002261233050376177f,0.04823236167430878f,0.006469023879617453f,0.006893250625580549f,0.008721877820789814f,-0.11302217841148376f,0.031788330525159836f,0.021183986216783524f,-0.05336468666791916f,-0.00999525748193264f,0.010429940186440945f,-0.004083389416337013f,-0.005108729470521212f,-0.04991665482521057f,0.017238005995750427f,-0.002707475796341896f,-0.014507238753139973f,0.0286194309592247f,-0.01620769128203392f,0.00944359041750431f,-0.005516205448657274f,-0.06373966485261917f,0.0012936982093378901f,-0.000666279171127826f,-0.0006470878724940121f,0.008017083629965782f,-0.005982252303510904f,-0.003124681767076254f,0.004734334070235491f,-0.004753447137773037f,-0.038314931094646454f,0.050328657031059265f,-0.012199996039271355f,0.006608725991100073f,0.009783413261175156f,-0.0011696661822497845f,-0.0041506849229335785f,0.06672318279743195f,-0.004392852075397968f,0.015229262411594391f,-0.010843202471733093f,-0.01554790884256363f,-0.04115154966711998f,0.0031191916204988956f,-0.008625312708318233f,-0.011656945571303368f,0.0019255942897871137f,-0.0009943832410499454f,-0.0009735409403219819f,-0.019905388355255127f,-0.006336902268230915f,0.0028907470405101776f,-0.0009075469570234418f,-0.05990041792392731f,-0.04321398586034775f,0.007422277703881264f,0.03485901281237602f,0.021718645468354225f,-0.0008122030412778258f,-0.003645313438028097f,0.0007074698223732412f,-0.08554957807064056f,0.023871062323451042f,-0.011544390581548214f,-0.012236732989549637f,-0.00029397098114714026f,0.022060852497816086f,0.0025264425203204155f,0.0024732681922614574f,-0.10042977333068848f,0.0020021721720695496f,-0.000980654265731573f,-0.0009875489631667733f,0.035391949117183685f,0.017290383577346802f,0.0057799010537564754f,0.007946106605231762f,-0.02076672948896885f,0.015259272418916225f,0.11895324289798737f,-0.13549251854419708f,-0.0056382580660283566f,0.0032300478778779507f,0.0011973453219980001f,-0.003997333813458681f,0.006385886576026678f,-0.00179484230466187f,0.02016756869852543f,-0.018321244046092033f,-0.011433621868491173f,0.00917899515479803f,0.0062179649248719215f,-0.0009321380057372153f,-0.06700186431407928f,0.0028988595586270094f,-0.001435091602616012f,-0.0014339280314743519f,-0.023366285488009453f,-0.011527154594659805f,0.010224913246929646f,-0.004489682614803314f,0.003009829903021455f,-0.08381258696317673f,0.0969444289803505f,-0.01318728644400835f,-0.0029359101317822933f,0.01884712278842926f,-0.007996606640517712f,-0.0045655108988285065f,0.024003256112337112f,-0.016975397244095802f,0.041081979870796204f,-0.02439800836145878f,0.011396332643926144f,-0.006274432875216007f,-0.0032207889016717672f,-0.004660513252019882f,-0.004490018356591463f,0.0029997604433447123f,-0.001513521303422749f,-0.001494710799306631f,0.018722957000136375f,0.019502397626638412f,0.017561834305524826f,0.0019016218138858676f,-0.005935186054557562f,-0.0009007438202388585f,0.0010798913426697254f,-0.0006645826506428421f,0.03486073389649391f,-0.007797228638082743f,0.005062546581029892f,-0.0004324831534177065f,0.06109091266989708f,0.030151521787047386f,-0.020647922530770302f,-0.009434161707758904f,0.016673261299729347f,-0.012717395089566708f,-0.009198975749313831f,-0.006205609999597073f,0.03091452084481716f,0.0023341865744441748f,-0.001145935500971973f,-0.0011758412001654506f,0.034920163452625275f,0.019037922844290733f,0.0008357964688912034f,0.0009258684003725648f,0.00424208166077733f,-0.10584054887294769f,0.048803288489580154f,0.05662955716252327f,-0.030275166034698486f,0.0026531293988227844f,-0.00855288002640009f,-0.007396599277853966f,0.005309156607836485f,-0.026382451876997948f,0.033064328134059906f,-0.006683073937892914f,-0.0488814078271389f,0.013901717029511929f,-0.012452609837055206f,0.005740640684962273f,0.06451601535081863f,0.0038463848177343607f,-0.001955564133822918f,-0.001932667102664709f,-0.05591987073421478f,0.005381043069064617f,0.010182422585785389f,-0.0004311708325985819f,-0.09291242063045502f,0.07204917818307877f,-0.05597860366106033f,-0.016094550490379333f,0.017632272094488144f,0.004091558046638966f,0.0012842703144997358f,-0.005118940025568008f,0.11029348522424698f,0.02651458978652954f,-0.029446734115481377f,0.0027522214222699404f,0.009015172719955444f,0.015913572162389755f,-0.0030253110453486443f,-0.0010511802975088358f,0.016990890726447105f,0.0037685709539800882f,-0.0019093463197350502f,-0.0018722187960520387f,-0.04476017504930496f,-0.02612358145415783f,-0.0007563754916191101f,-0.005585115402936935f,-0.016015933826565742f,-0.028264254331588745f,0.0730382651090622f,-0.04540582373738289f,0.01753445342183113f,-0.010656912811100483f,0.0053380336612463f,-0.004510950762778521f,-0.0028795492835342884f,0.0024394281208515167f,0.012755123898386955f,-0.015344920568168163f,0.01206178218126297f,-0.045341987162828445f,-0.00018033257219940424f,-0.006614773068577051f,-0.027382126078009605f,0.001943142618983984f,-0.0009660262148827314f,-0.0009745123097673059f,-0.024348074570298195f,0.005873134359717369f,-0.007117387373000383f,-0.0010861529735848308f,0.01697014458477497f,-0.009375377558171749f,0.04682598635554314f,-0.03661860525608063f,-0.0053399959579110146f,0.0069647664204239845f,-0.0007955433102324605f,-0.0005870248423889279f,-0.018303023651242256f,0.00806682463735342f,0.0030728555284440517f,-0.010858929716050625f,0.005532287061214447f,2.686773541427101e-06f,-0.006842262111604214f,-0.0021043799351900816f,-0.05896037444472313f,0.0014593737432733178f,-0.000695160124450922f,-0.0007317292038351297f,0.0010287973564118147f,-0.010729792527854443f,-0.005458127707242966f,-0.009179960936307907f,0.024255016818642616f,-0.0877859890460968f,-0.022525304928421974f,0.10997997969388962f,0.008552046492695808f,0.010562512092292309f,-0.004136987496167421f,0.00018576376896817237f,0.006752952933311462f,-0.006731257773935795f,-0.01681927777826786f,0.02374662272632122f,0.016456589102745056f,0.04151839017868042f,-0.006673409137874842f,0.004960206337273121f,-0.022989600896835327f,0.004035432357341051f,-0.001998623600229621f,-0.0019946866668760777f,-0.01618845760822296f,0.004741881042718887f,-0.002295989776030183f,-0.0015970421954989433f,0.004785211756825447f,-0.013388419523835182f,0.04995042085647583f,-0.036164045333862305f,-0.014399687759578228f,0.022267445921897888f,0.0008490309119224548f,1.4093320714891888e-05f,-0.021507415920495987f,0.019377367570996284f,-0.016428077593445778f,-0.002818154636770487f,-0.021351179108023643f,0.0009214083547703922f,-0.00010930187272606418f,0.003501099767163396f,0.02038135565817356f,0.002675976138561964f,-0.0013300984865054488f,-0.0012987398076802492f,0.015213727951049805f,0.0010040983324870467f,0.003940350376069546f,0.0020220857113599777f,-0.1437036246061325f,-0.036736905574798584f,0.04539765417575836f,-0.008256402797996998f,-0.0067248097620904446f,0.0003087820950895548f,-0.004245249088853598f,0.0022434007842093706f,0.021318066865205765f,0.028159184381365776f,-0.004166484344750643f,-0.024236740544438362f,-0.01740756630897522f,0.012688611634075642f,0.0061480384320020676f,0.001120098284445703f,-0.013402154669165611f,0.0035666353069245815f,-0.0017966770101338625f,-0.0017922859406098723f,0.0008939628605730832f,-0.0055664824321866035f,-0.00984345842152834f,-0.007878120057284832f,-0.06564316898584366f,-0.03825719654560089f,0.009087740443646908f,0.02967964857816696f,-0.00369553011842072f,0.0006269778241403401f,0.002272534416988492f,-0.00015015696408227086f,-0.03319587558507919f,0.04361792653799057f,-0.030757268890738487f,-0.012806335464119911f,0.012623557820916176f,0.021333947777748108f,-0.002234508516266942f,0.0006278334767557681f,0.002362227300181985f,0.0018340861424803734f,-0.0009252811432816088f,-0.0009040120639838278f,0.009755878709256649f,-0.0012822874123230577f,0.006432718131691217f,-0.011649135500192642f,0.0048352493904531f,0.06240120157599449f,-0.03296232968568802f,-0.02993476577103138f,0.005291495472192764f,0.00153674790635705f,0.006492914166301489f,0.00800715759396553f,-0.09392980486154556f,0.033921435475349426f,-0.0271921269595623f,-0.006725870072841644f,-0.01006641797721386f,0.024250494316220284f,0.01954636536538601f,0.0023507841397076845f,0.017713356763124466f,0.0029197626281529665f,-0.0014727023662999272f,-0.0014503034763038158f,0.008284538052976131f,-0.0025783844757825136f,0.003660239279270172f,0.0019779978320002556f,0.050141558051109314f,0.04463503509759903f,0.04944172501564026f,-0.0942406877875328f,-0.0027559443842619658f,-0.015348374843597412f,0.005558009725064039f,-0.0003621999640017748f,0.027095530182123184f,0.03225462883710861f,-0.011235591024160385f,-0.0211886428296566f,0.004281019326299429f,-0.05011894553899765f,-0.010908987373113632f,-0.00854380615055561f,-0.058768998831510544f,0.002538398141041398f,-0.0012342133559286594f,-0.001264787744730711f,-0.008744092658162117f,-0.01480684895068407f,0.007372144144028425f,0.0022438245359808207f,-0.05108596384525299f,0.006505582015961409f,-0.010679231025278568f,0.004353462252765894f,0.0073768761940300465f,0.0013230436015874147f,-0.0025661271065473557f,-6.52374656056054e-05f,0.04830510541796684f,-0.013054793700575829f,0.018526563420891762f,-0.004996641539037228f,-0.0028301486745476723f,-0.009433883242309093f,0.007081047631800175f,-0.0071424636989831924f,0.06218970566987991f,0.003975518047809601f,-0.0019963125232607126f,-0.002012569224461913f,0.005117300897836685f,-0.00418109493330121f,0.01377106923609972f,0.0008312358404509723f,0.02759314887225628f,-0.007467913441359997f,0.021485095843672752f,-0.01358650904148817f,0.0030218965839594603f,0.008191410452127457f,-0.004010012838989496f,0.0063376291655004025f,0.012140905484557152f,-0.011413976550102234f,0.029153412207961082f,-0.017954906448721886f,0.0013443150091916323f,-0.0006315432256087661f,0.009393121115863323f,-0.00669105863198638f,-0.005924113094806671f,0.0015670130960643291f,-0.0007965161348693073f,-0.0007475115125998855f,0.0043773590587079525f,0.014896598644554615f,-0.0012915462721139193f,0.011499017477035522f,-0.07554677128791809f,0.06660915911197662f,-0.04341627657413483f,-0.02302847057580948f,0.013474144041538239f,-0.007885979488492012f,0.00019340698781888932f,-0.002793353982269764f,-0.02897740714251995f,0.03002600185573101f,-0.01282167062163353f,-0.017253652215003967f,0.004514778032898903f,0.007733955048024654f,-0.004277398809790611f,-0.004601235967129469f,-0.03852508217096329f,0.0018685850081965327f,-0.000954155926592648f,-0.000929658766835928f,0.03823261335492134f,-0.015678396448493004f,0.0055604479275643826f,0.0026565156877040863f,-0.0034073740243911743f,-0.0010852091945707798f,0.07709485292434692f,-0.07673519104719162f,-0.004929978400468826f,0.014018218033015728f,-0.00634648697450757f,0.00026182291912846267f,-0.07051745057106018f,0.028592847287654877f,-0.01896085776388645f,-0.009610000066459179f,0.0024188831448554993f,-0.010797311551868916f,-0.0021446067839860916f,-0.004805789794772863f,-0.0005263467319309711f,0.0026976631488651037f,-0.001345044351182878f,-0.0013411445543169975f,0.030760476365685463f,0.0023975223302841187f,-0.009162441827356815f,0.00883561372756958f,-0.025402622297406197f,0.01233868207782507f,-0.04010945186018944f,0.02714226394891739f,-0.00649447413161397f,-0.017810549587011337f,0.004140123259276152f,-0.012477035634219646f,-0.05350900813937187f,0.008760147728025913f,-0.021164001896977425f,0.012387706898152828f,-0.002833361504599452f,0.00355089851655066f,-0.0008280836627818644f,-4.745373735204339e-05f,0.05906077101826668f,0.0028662667609751225f,-0.0014346230309456587f,-0.001434443867765367f,-0.012284670025110245f,-0.002079159952700138f,0.01197861135005951f,0.009198655374348164f,-0.0863599181175232f,0.11928462237119675f,-0.0777309238910675f,-0.042024269700050354f,0.00528842443600297f,-0.010450365953147411f,-0.002900384832173586f,-0.007671772502362728f,0.0607636533677578f,0.021527543663978577f,-0.0223274864256382f,0.0007804752676747739f,0.011983539909124374f,-0.005491511430591345f,-0.005682995077222586f,-9.806353045860305e-05f,-0.07606589049100876f,0.002736541209742427f,-0.0013584111584350467f,-0.0013722300063818693f,-0.02750963531434536f,-0.0002330337738385424f,0.0021865542512387037f,-0.007864371873438358f,0.0005888977320864797f,-0.0030346668791025877f,0.006868547294288874f,-0.003972681704908609f,0.01589089073240757f,-0.02944302000105381f,0.006094976793974638f,-0.007288549095392227f,0.03633512556552887f,-0.00021828139142598957f,0.02385888434946537f,-0.024113556370139122f,0.03016578033566475f,-0.006438249256461859f,0.0010652825003489852f,-0.003579127136617899f,-0.004222193267196417f,0.0032094253692775965f,-0.0016230421606451273f,-0.0015904523897916079f,0.04363775625824928f,0.004514490254223347f,0.0025214424822479486f,0.008795489557087421f,-0.06491716206073761f,0.080134816467762f,-0.007863648235797882f,-0.07205288112163544f,0.016159527003765106f,0.02054365538060665f,0.006494925823062658f,0.0001366643700748682f,-0.07066871970891953f,0.036777906119823456f,-0.01866430789232254f,-0.018252430483698845f,-0.010824787430465221f,0.016219526529312134f,0.005923237185925245f,0.0054534547962248325f,-0.024113576859235764f,0.0025859472807496786f,-0.0012817011447623372f,-0.0012649900745600462f,0.017030252143740654f,-0.004950319416821003f,0.00499718077480793f,0.010140215046703815f,0.0967923104763031f,0.007273384369909763f,-0.0738334208726883f,0.06629757583141327f,0.021875087171792984f,-0.0015985574573278427f,0.010730714537203312f,0.014204010367393494f,0.021563472226262093f,0.0519871711730957f,-0.03603976219892502f,-0.01588222198188305f,-0.02211199700832367f,-0.01292105857282877f,0.007991061545908451f,-9.189949196297675e-06f,0.016792045906186104f,0.002853947225958109f,-0.001432410441339016f,-0.0014290311373770237f,-0.004448601510375738f,0.009333177469670773f,-0.0017603231826797128f,0.016071589663624763f,0.14944495260715485f,-0.055047210305929184f,-0.07130005210638046f,0.12662893533706665f,-0.03408823534846306f,0.018566319718956947f,-0.0012383193243294954f,0.00011658298899419606f,-0.05262783169746399f,-0.0007391656399704516f,0.011084460653364658f,-0.010410131886601448f,-0.002489590086042881f,0.010638399049639702f,-0.012879026122391224f,-0.002650580834597349f,0.029910320416092873f,0.004440339282155037f,-0.002218051813542843f,-0.0022220038808882236f,-0.0021437073592096567f,0.016092853620648384f,0.011105606332421303f,0.007938245311379433f,0.08100154250860214f,-0.02699938416481018f,0.03942013531923294f,-0.012430788949131966f,0.0075330063700675964f,0.001093076542019844f,0.00640070391818881f,0.0042311400175094604f,0.05980553478002548f,0.0028020639438182116f,-0.009088902734220028f,0.006252511404454708f,0.01866578310728073f,0.00029608988552354276f,0.015008406713604927f,-0.007654559798538685f,0.004755789879709482f,0.0034372371155768633f,-0.0017419335199519992f,-0.001710337121039629f,0.012974056415259838f,0.0026921769604086876f,0.005781413987278938f,0.012825251556932926f,0.025374623015522957f,0.06426984071731567f,-0.052499640733003616f,-0.011689605191349983f,-0.019812950864434242f,0.0029584344010800123f,2.939209934993414e-06f,-0.0013122365344315767f,0.00025613702018745244f,0.029097747057676315f,-0.009967966936528683f,-0.01917787455022335f,-0.005920147057622671f,-0.01529501099139452f,0.009236391633749008f,-0.0025890416000038385f,0.02556898072361946f,0.0016416743164882064f,-0.0008028468582779169f,-0.000827551877591759f,-0.013849552720785141f,0.00851349625736475f,-0.0015036518452689052f,0.002251638565212488f,0.09676960855722427f,0.13800778985023499f,-0.04882810264825821f,-0.08950311690568924f,0.005235748365521431f,0.017094438895583153f,0.0029488313011825085f,-0.0036786922719329596f,-0.025454320013523102f,0.021124351769685745f,-0.008737867698073387f,-0.012354535982012749f,-0.0013534222962334752f,-0.004639649298042059f,-0.002564889146015048f,-0.0034497790038585663f,-0.062475819140672684f,0.0010211063781753182f,-0.0005085565499030054f,-0.0005266188527457416f,-0.035704195499420166f,0.004256270360201597f,-0.006263913121074438f,0.018411312252283096f,-0.0595143623650074f,-0.10431879013776779f,0.008685505948960781f,0.09595033526420593f,-0.013474829494953156f,-0.010740146972239017f,0.0035052725579589605f,0.00064961111638695f,-0.09008649736642838f,-0.012548454105854034f,-0.0008820173679850996f,0.013452778570353985f,-0.019851237535476685f,0.008194345980882645f,0.009988011792302132f,-0.005588260479271412f,0.010475678369402885f,0.0030551934614777565f,-0.001526127103716135f,-0.0015419978881254792f,-0.03377581760287285f,0.02618686482310295f,-0.005411219783127308f,-0.001134447637014091f,0.022204207256436348f,0.09080681204795837f,-0.0372115820646286f,-0.05427781865000725f,-0.0051953778602182865f,-0.01570749841630459f,-0.0043370891362428665f,0.004556730855256319f,0.08117415010929108f,0.047133706510066986f,-0.03871319070458412f,-0.008313038386404514f,-0.01964353211224079f,-0.007880249992012978f,-0.01638896018266678f,-0.00649175513535738f,-0.0057586743496358395f,0.0044602383859455585f,-0.0022142406087368727f,-0.0022377907298505306f,-0.008134376257658005f,-0.008616738952696323f,0.019635939970612526f,0.0023522693663835526f,-0.05612812936306f,0.10062151402235031f,-0.028841668739914894f,-0.07229726016521454f,-0.002229623030871153f,0.010554380714893341f,-0.0029301128815859556f,0.004208523780107498f,0.0549875907599926f,0.043803539127111435f,-0.02116294391453266f,-0.022785095497965813f,-0.011460274457931519f,0.02033093571662903f,0.0022764599416404963f,0.0002400414814474061f,-0.004810837563127279f,0.0019965472165495157f,-0.0009703697869554162f,-0.0009899827418848872f,-0.022180577740073204f,-0.008090823888778687f,-0.0031458812300115824f,-0.004960731603205204f,0.020049558952450752f,0.010518413037061691f,0.11536385864019394f,-0.12624013423919678f,0.002736038062721491f,-0.0035234184470027685f,0.0043229032307863235f,0.0018146393122151494f,-0.08364513516426086f,-0.005518232472240925f,0.01813463307917118f,-0.012656611390411854f,-0.016262829303741455f,-0.01955593004822731f,-0.008571641519665718f,-0.002664325525984168f,-0.049417704343795776f,0.0031355591490864754f,-0.001554569462314248f,-0.001577979070134461f,-0.008666967041790485f,-0.0019934626761823893f,-0.009242020547389984f,-0.012457008473575115f,-0.04116668552160263f,0.03825907036662102f,-0.04151145741343498f,0.0032029119320213795f,-0.007878453470766544f,-0.015526346862316132f,0.004938782192766666f,0.0059225005097687244f,-0.038815584033727646f,0.05983704328536987f,-0.035778023302555084f,-0.024121565744280815f,0.0031112199649214745f,-0.0010472633875906467f,0.006656731478869915f,-0.008324801921844482f,-0.03763904049992561f,0.0015912246890366077f,-0.0007987984572537243f,-0.0008193267276510596f,0.008192201144993305f,-0.009004306979477406f,-0.0002573105739429593f,-0.018963929265737534f,0.05968848988413811f,-0.02067751996219158f,-0.018758753314614296f,0.03939767926931381f,0.01550378743559122f,-0.04098990187048912f,0.0006849049823358655f,-0.0005837423959746957f,-0.046649497002363205f,0.01612287014722824f,-0.009567687287926674f,-0.006504029035568237f,-0.03843134641647339f,-0.010764098726212978f,-0.005584198050200939f,-0.003579929005354643f,0.05079592391848564f,0.0028218983206897974f,-0.0013827583752572536f,-0.0013891333946958184f,0.04010647162795067f,0.03396952152252197f,0.001993577927350998f,0.009745782241225243f,0.0886673852801323f,-0.027881035581231117f,-0.026720352470874786f,0.0546397902071476f,0.004852666519582272f,-0.009148964658379555f,0.006902821362018585f,-0.004519079811871052f,0.020263785496354103f,0.0168622937053442f,0.0027657656464725733f,-0.01972927339375019f,0.045659538358449936f,-0.021418869495391846f,-0.0004806529905181378f,-0.00491840485483408f,0.02010554075241089f,0.0019118066411465406f,-0.0009617157629691064f,-0.0009459183784201741f,-0.02176872454583645f,0.0016665189759805799f,0.0035960234235972166f,-0.010511381551623344f,0.00463278591632843f,0.023327112197875977f,-0.02941819466650486f,0.006161488592624664f,0.011080139316618443f,0.00850930716842413f,0.00012553345004562289f,0.009891858324408531f,0.0043513718992471695f,0.01913781650364399f,-0.008901065215468407f,-0.010146167129278183f,0.0002929906768258661f,-0.022388847544789314f,0.000599958817474544f,0.0002808667777571827f,0.07526018470525742f,0.003468910465016961f,-0.0017227897187694907f,-0.0017356151947751641f,-0.006436042487621307f,-0.004195785149931908f,-0.028474746271967888f,-0.006398058030754328f,0.037085697054862976f,-0.027194291353225708f,-0.0335308238863945f,0.06085927411913872f,-0.01890229992568493f,-0.0168210007250309f,-0.006569149903953075f,-0.00851169042289257f,-0.04516809806227684f,-0.010042970068752766f,0.022224193438887596f,-0.0121764512732625f,0.002049453789368272f,-0.0167425274848938f,-0.003944538068026304f,-0.0003307466395199299f,0.056243982166051865f,0.002391629619523883f,-0.0011981758289039135f,-0.001222918275743723f,0.010325278155505657f,0.014836069196462631f,0.0023111789487302303f,0.0037313250359147787f,-0.015684111043810844f,-0.07334666699171066f,0.030574552714824677f,0.04287828505039215f,0.01699059270322323f,-0.00545648904517293f,0.0010729747591540217f,0.0038290058728307486f,-0.02872314490377903f,0.005361796822398901f,0.004346283618360758f,-0.009739376604557037f,0.04032779112458229f,-0.01687779277563095f,0.002197663299739361f,-0.006714862771332264f,0.0037330843042582273f,0.0017714843852445483f,-0.0008760458440519869f,-0.0009157589520327747f,-0.005075747612863779f,0.021582847461104393f,0.0025628271978348494f,-0.0010295563843101263f,-0.08266853541135788f,-0.007603662088513374f,-0.04074598848819733f,0.0488317497074604f,0.00760645791888237f,0.005133687052875757f,0.005366870202124119f,-0.004415379837155342f,0.033519260585308075f,0.028649989515542984f,-0.01657487079501152f,-0.012064055539667606f,-0.006170919165015221f,0.018062224611639977f,0.0009807521710172296f,-0.0033006910234689713f,-0.01568036526441574f,0.003522297367453575f,-0.001756834564730525f,-0.0017715641297399998f,-0.04801129177212715f,-0.014439471065998077f,-0.027224840596318245f,-0.009047270752489567f,0.06951770186424255f,-0.0668117105960846f,0.0484142079949379f,0.019163187593221664f,-0.005495102610439062f,-0.02239101752638817f,-0.007106224540621042f,-0.002152610570192337f,-0.10639993101358414f,0.006929066497832537f,0.0008617894491180778f,-0.007752344477921724f,0.006149204447865486f,-0.014057585969567299f,-0.013090165331959724f,-0.006071706302464008f,0.03497534617781639f,0.0014426871202886105f,-0.0007077668560668826f,-0.0007107389974407852f};
void initclmemobjects()
{
}

void cnn(float *in_0, float *out_0)
{
  float* _buffer = (float*) calloc(38400+(16-1), sizeof(float));
  float* buffer = (float*)(((uintptr_t)_buffer+(16-1)) & ~(uintptr_t)(16-1));
  INTERNAL_CNN_STOPWATCH("OpInput (input_1)")
  {
    // OpInput
    const int H = 60; const int W = 80; const int C = 3;
    const float* in = (in_0 + 0); float* out = (buffer + 0);
    
    COPY(out, in, H * W * C * sizeof(float), 0);
  }
  INTERNAL_CNN_STOPWATCH("OpPadding (separable_conv2d_internal_0)")
  {
    // OpPadding
    const int H = 60; const int W = 80; const int W_OUT = 81; const int C = 3;
    const int PT = 0; const int PB = 1; const int PL = 0; const int PR = 1;
    const float* in = (buffer + 0); float* out = (buffer + 14400);
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
    memset(buffer + 0, 0, 14400 * sizeof(float));
    // OpDepthwiseConvolution2D
    const int H = 61; const int W = 81; const int C_IN = 3; const int C_OUT = 12; const int W_OUT = 40;
    const int SH = 2; const int SW = 2;
    const int KH = 3; const int KW = 3;
    const int DEPTH_MULTIPLIER = 4;
    const float* in_ = (buffer + 14400); float* out_ = (buffer + 0); const float* weights_ = separable_conv2d_internal_1_W;
    
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
    memset(buffer + 19200, 0, 19200 * sizeof(float));
    // OpConvolution2D
    const int W = 40; const int C_IN = 12; const int C_OUT = 16; const int H_OUT = 30; const int W_OUT = 40;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 19200); const float* weights_ = separable_conv2d_internal_2_W;
    
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
    const int H = 30; const int W = 40; const int C = 16;
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
    const int H = 30; const int W = 40; const int C = 16;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 19200);
    
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
    memset(buffer + 0, 0, 4800 * sizeof(float));
    // OpConvolution2D
    const int W = 40; const int C_IN = 16; const int C_OUT = 4; const int H_OUT = 30; const int W_OUT = 40;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 19200); float* out_ = (buffer + 0); const float* weights_ = conv2d_internal_1_W;
    
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
    const int H = 30; const int W = 40; const int C = 4;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 4800);
    
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
    const int H = 30; const int W = 40; const int C = 4;
    const float* in_ = (buffer + 4800); float* out_ = (buffer + 0);
    
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
    memset(buffer + 4800, 0, 5084 * sizeof(float));
    // OpPadding
    const int H = 30; const int H_OUT = 31; const int W = 40; const int W_OUT = 41; const int C = 4;
    const int PT = 0; const int PL = 0;
    const float* in = (buffer + 0); float* out = (buffer + 4800);
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
    memset(buffer + 0, 0, 4800 * sizeof(float));
    // OpDepthwiseConvolution2D
    const int H = 31; const int W = 41; const int C_IN = 4; const int C_OUT = 16; const int W_OUT = 20;
    const int SH = 2; const int SW = 2;
    const int KH = 3; const int KW = 3;
    const int DEPTH_MULTIPLIER = 4;
    const float* in_ = (buffer + 4800); float* out_ = (buffer + 0); const float* weights_ = separable_conv2d_1_internal_1_W;
    
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
    memset(buffer + 14416, 0, 7200 * sizeof(float));
    // OpConvolution2D
    const int W = 20; const int C_IN = 16; const int C_OUT = 24; const int H_OUT = 15; const int W_OUT = 20;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 14416); const float* weights_ = separable_conv2d_1_internal_2_W;
    
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
    const int H = 15; const int W = 20; const int C = 24;
    const float* in_ = (buffer + 14416); float* out_ = (buffer + 7216);
    
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
    const int H = 15; const int W = 20; const int C = 24;
    const float* in_ = (buffer + 7216); float* out_ = (buffer + 0);
    
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
    memset(buffer + 7200, 0, 2400 * sizeof(float));
    // OpConvolution2D
    const int W = 20; const int C_IN = 24; const int C_OUT = 8; const int H_OUT = 15; const int W_OUT = 20;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 7200); const float* weights_ = conv2d_1_internal_1_W;
    
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
    const int H = 15; const int W = 20; const int C = 8;
    const float* in_ = (buffer + 7200); float* out_ = (buffer + 0);
    
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
    const int H = 15; const int W = 20; const int C = 8;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 22192);
    
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
    memset(buffer + 0, 0, 2992 * sizeof(float));
    // OpPadding
    const int H = 15; const int H_OUT = 17; const int W = 20; const int W_OUT = 22; const int C = 8;
    const int PT = 1; const int PL = 1;
    const float* in = (buffer + 22192); float* out = (buffer + 0);
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
    memset(buffer + 2992, 0, 9600 * sizeof(float));
    // OpDepthwiseConvolution2D
    const int H = 17; const int W = 22; const int C_IN = 8; const int C_OUT = 32; const int W_OUT = 20;
    const int SH = 1; const int SW = 1;
    const int KH = 3; const int KW = 3;
    const int DEPTH_MULTIPLIER = 4;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 2992); const float* weights_ = separable_conv2d_2_internal_1_W;
    
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
    memset(buffer + 12592, 0, 9600 * sizeof(float));
    // OpConvolution2D
    const int W = 20; const int C_IN = 32; const int C_OUT = 32; const int H_OUT = 15; const int W_OUT = 20;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 2992); float* out_ = (buffer + 12592); const float* weights_ = separable_conv2d_2_internal_2_W;
    
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
    const int H = 15; const int W = 20; const int C = 32;
    const float* in_ = (buffer + 12592); float* out_ = (buffer + 304);
    
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
    const int H = 15; const int W = 20; const int C = 32;
    const float* in_ = (buffer + 304); float* out_ = (buffer + 9904);
    
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
    memset(buffer + 2448, 0, 2400 * sizeof(float));
    // OpConvolution2D
    const int W = 20; const int C_IN = 32; const int C_OUT = 8; const int H_OUT = 15; const int W_OUT = 20;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 9904); float* out_ = (buffer + 2448); const float* weights_ = conv2d_2_internal_1_W;
    
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
    const int H = 15; const int W = 20; const int C = 8;
    const float* in_ = (buffer + 2448); float* out_ = (buffer + 48);
    
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
    const int H = 15; const int W = 20; const int C = 8;
    const float* in_1 = (buffer + 48); const float* in_2 = (buffer + 22192); float* out_ = (buffer + 2448);
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
    const int H = 15; const int W = 20; const int C = 8;
    const float* in_ = (buffer + 2448); float* out_ = (buffer + 0);
    
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
    memset(buffer + 2560, 0, 2856 * sizeof(float));
    // OpPadding
    const int H = 15; const int H_OUT = 17; const int W = 20; const int W_OUT = 21; const int C = 8;
    const int PT = 1; const int PL = 0;
    const float* in = (buffer + 0); float* out = (buffer + 2560);
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
    memset(buffer + 0, 0, 2560 * sizeof(float));
    // OpDepthwiseConvolution2D
    const int H = 17; const int W = 21; const int C_IN = 8; const int C_OUT = 32; const int W_OUT = 10;
    const int SH = 2; const int SW = 2;
    const int KH = 3; const int KW = 3;
    const int DEPTH_MULTIPLIER = 4;
    const float* in_ = (buffer + 2560); float* out_ = (buffer + 0); const float* weights_ = separable_conv2d_3_internal_1_W;
    
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
    memset(buffer + 2560, 0, 1920 * sizeof(float));
    // OpConvolution2D
    const int W = 10; const int C_IN = 32; const int C_OUT = 24; const int H_OUT = 8; const int W_OUT = 10;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 2560); const float* weights_ = separable_conv2d_3_internal_2_W;
    
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
    const int H = 8; const int W = 10; const int C = 24;
    const float* in_ = (buffer + 2560); float* out_ = (buffer + 0);
    
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
    const int H = 8; const int W = 10; const int C = 24;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 1920);
    
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
    memset(buffer + 0, 0, 1280 * sizeof(float));
    // OpConvolution2D
    const int W = 10; const int C_IN = 24; const int C_OUT = 16; const int H_OUT = 8; const int W_OUT = 10;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 1920); float* out_ = (buffer + 0); const float* weights_ = conv2d_3_internal_1_W;
    
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
    const int H = 8; const int W = 10; const int C = 16;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 1280);
    
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
    const int H = 8; const int W = 10; const int C = 16;
    const float* in_ = (buffer + 1280); float* out_ = (buffer + 7696);
    
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
    memset(buffer + 0, 0, 1920 * sizeof(float));
    // OpPadding
    const int H = 8; const int H_OUT = 10; const int W = 10; const int W_OUT = 12; const int C = 16;
    const int PT = 1; const int PL = 1;
    const float* in = (buffer + 7696); float* out = (buffer + 0);
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
    memset(buffer + 2576, 0, 5120 * sizeof(float));
    // OpDepthwiseConvolution2D
    const int H = 10; const int W = 12; const int C_IN = 16; const int C_OUT = 64; const int W_OUT = 10;
    const int SH = 1; const int SW = 1;
    const int KH = 3; const int KW = 3;
    const int DEPTH_MULTIPLIER = 4;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 2576); const float* weights_ = separable_conv2d_4_internal_1_W;
    
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
    memset(buffer + 16, 0, 2560 * sizeof(float));
    // OpConvolution2D
    const int W = 10; const int C_IN = 64; const int C_OUT = 32; const int H_OUT = 8; const int W_OUT = 10;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 2576); float* out_ = (buffer + 16); const float* weights_ = separable_conv2d_4_internal_2_W;
    
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
    const int H = 8; const int W = 10; const int C = 32;
    const float* in_ = (buffer + 16); float* out_ = (buffer + 3888);
    
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
    const int H = 8; const int W = 10; const int C = 32;
    const float* in_ = (buffer + 3888); float* out_ = (buffer + 1328);
    
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
    memset(buffer + 0, 0, 1280 * sizeof(float));
    // OpConvolution2D
    const int W = 10; const int C_IN = 32; const int C_OUT = 16; const int H_OUT = 8; const int W_OUT = 10;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 1328); float* out_ = (buffer + 0); const float* weights_ = conv2d_4_internal_1_W;
    
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
    const int H = 8; const int W = 10; const int C = 16;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 1280);
    
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
    const int H = 8; const int W = 10; const int C = 16;
    const float* in_1 = (buffer + 1280); const float* in_2 = (buffer + 7696); float* out_ = (buffer + 0);
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
    const int H = 8; const int W = 10; const int C = 16;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 2544);
    
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
    memset(buffer + 0, 0, 1584 * sizeof(float));
    // OpPadding
    const int H = 8; const int H_OUT = 9; const int W = 10; const int W_OUT = 11; const int C = 16;
    const int PT = 0; const int PL = 0;
    const float* in = (buffer + 2544); float* out = (buffer + 0);
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
    memset(buffer + 1584, 0, 1280 * sizeof(float));
    // OpDepthwiseConvolution2D
    const int H = 9; const int W = 11; const int C_IN = 16; const int C_OUT = 64; const int W_OUT = 5;
    const int SH = 2; const int SW = 2;
    const int KH = 3; const int KW = 3;
    const int DEPTH_MULTIPLIER = 4;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 1584); const float* weights_ = separable_conv2d_5_internal_1_W;
    
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
    memset(buffer + 0, 0, 800 * sizeof(float));
    // OpConvolution2D
    const int W = 5; const int C_IN = 64; const int C_OUT = 40; const int H_OUT = 4; const int W_OUT = 5;
    const int SH = 1; const int SW = 1;
    const int KH = 1; const int KW = 1;
    const float* in_ = (buffer + 1584); float* out_ = (buffer + 0); const float* weights_ = separable_conv2d_5_internal_2_W;
    
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
    const int H = 4; const int W = 5; const int C = 40;
    const float* in_ = (buffer + 0); float* out_ = (buffer + 800);
    
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
    const int H = 4; const int W = 5; const int C = 40;
    const float* in_ = (buffer + 800); float* out_ = (buffer + 1680);
    
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
    memset(buffer + 0, 0, 1680 * sizeof(float));
    // OpPadding
    const int H = 4; const int H_OUT = 6; const int W = 5; const int W_OUT = 7; const int C = 40;
    const int PT = 1; const int PL = 1;
    const float* in = (buffer + 1680); float* out = (buffer + 0);
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
    memset(out_0 + 0, 0, 480 * sizeof(float));
    // OpConvolution2D
    const int W = 7; const int C_IN = 40; const int C_OUT = 24; const int H_OUT = 4; const int W_OUT = 5;
    const int SH = 1; const int SW = 1;
    const int KH = 3; const int KW = 3;
    const float* in_ = (buffer + 0); float* out_ = (out_0 + 0); const float* weights_ = DetectionLayer_internal_1_W;
    
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
  free(_buffer);
}

#ifdef CNN_TEST

#include <cstdlib>
#include <cstdio>

int main()
{
    printf("START ...\n");
    const int IN_DIM = 14400;
    const int OUT_DIM = 480;
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