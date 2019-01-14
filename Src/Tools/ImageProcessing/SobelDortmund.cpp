// TODO: check this warning
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <tmmintrin.h>
#ifdef __clang__
#pragma clang diagnostic pop
#endif
#include "SobelDortmund.h"

#include "Representations/Infrastructure/Image.h"
#include "Tools/Debugging/Stopwatch.h"


stdVector2D<unsigned char> SobelDortmund::sobelSSEAnyYUVImageFull(const Image &YUVImage, Vector2i start, Vector2i end, int width, int height,
                                                                  Direction dir, bool returnFullArray)
{
  unsigned char *image_ptr = (unsigned char *)YUVImage.image;

  switchStartEnd(start, end);
  int rectWidth = end(0) - start(0) + 1;
  int rectHeight = end(1) - start(1) + 1;

  stdVector2D<unsigned char> targetData(returnFullArray ? width : rectWidth, returnFullArray ? height : rectHeight);

  unsigned char *target = targetData.data();

  // Calculate the pointers to the first element of the first 3 rows using pointer arithmetic
  unsigned char *row_0_ptr = image_ptr + start(1) * width * 2;
  unsigned char *row_1_ptr = row_0_ptr + 2 * width;
  unsigned char *row_2_ptr = row_0_ptr + 4 * width;


  // Main Loop, we will start at (1,1) because Sobel needs a 3x3 surrounding, which is
  // not available for edge pixels
  // Every row will be looped through, but only every 14th column, because with 8 bit SSE
  // it is possible to store 16 8-bit values in one SSE register, which will give 14 results (minus edge pixels included!)

  for (int y = start(1) + 1; y < end(1); y++)
  {
    for (int x = start(0) + 1; x < end(0); x += 14)
    {
      // Now we need to load 16 Y values per row into 1 register
      // This is done with SSE unpack, which works as in the following example:
      // Pointer a: a0, a1, a3, ... , a15 | Pointer b: b0, b1, b3, ... , b15 (each value ai/bi has the size 1 Byte)
      // _mm_unpacklo_epi8(a,b) will result in a register with the 16 8-Bit values: a0, b0, a1, b1, ... , a7, b7
      // _mm_unpackhi_epi8(a,b) will result in a register with the 16 8-Bit values: a8, b8, a9, b9, ... , a15, b15
      // We want to use these unpack operations to deinterlace our YUV422 array, so that we will have two registers
      // one containing all the Y values in correct order and the other one containing all U and V values

      // --------------------------------------------------------------------------------------------------------

      // The following commands will deinterlace the Y values for the first row:
      // Let the YUV422 array be: Y0, U0, Y1, V0, Y2, U1, Y3, V1, Y4, U2, Y5, V2, ...
      // Unpack the first 16 Values with the second 16 values into a (lo)
      // This will result in a = Y0, Y8, U0, U4, Y1, Y9, V0, V4, ...
      // likewise b will be (hi): Y5, Y12, U2, U6, Y5, Y13, V2, V6, ...

      // The pointer will need an offset of 2 * (x-1)
      // -1: because we started this loop with x = 1
      // 2 * : because only every second value is a Y value

      __m128i loadA = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_0_ptr + 2 * (x - 1)));
      __m128i loadB = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_0_ptr + 16 + 2 * (x - 1)));

      __m128i a = _mm_unpacklo_epi8(loadA, loadB);
      __m128i b = _mm_unpackhi_epi8(loadA, loadB);

      // We will unpack again, resulting in 4 continous Y values per register
      // i.e. c = Y0, Y4, Y8, Y12, U0, U2, U4, U6, ...
      //      d = Y2, Y6, Y10, Y14, U1, U3, U5, U7, ...
      __m128i c = _mm_unpacklo_epi8(a, b);
      __m128i d = _mm_unpackhi_epi8(a, b);

      // Next unpack will result in 8 continous Y values per register
      // i.e. a = Y0, Y2, Y4, ... , Y14, U0, U1, ...
      //      b = Y1, Y3, Y5, ... , Y15, V0, V1, ...
      a = _mm_unpacklo_epi8(c, d);
      b = _mm_unpackhi_epi8(c, d);

      // For the last unpack we only need the lower 8 values from each register, because these are the Y values
      // row_0 = Y0, Y1, Y2, Y3, ... , Y15
      // The unpackhi would give: U0, V0, U1, V1, ... but since we do not want to use the color values, this won't be calculated
      __m128i row_0 = _mm_unpacklo_epi8(a, b);

      // Since the Sobel operator needs addition of the 8-bit values, the largest value could be getting up to 4*255
      // So we will neglect the 2 least significant bits by shifting each 8-Bit value 2 bits to the right (dividing by 4)
      // This function is not available in SSE, so I implemented my own using 16-Bit operations
      // row_0 = _mm_srli_epi8(row_0, 2);

      // Same as above for second row
      loadA = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_1_ptr + 2 * (x - 1)));
      loadB = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_1_ptr + 16 + 2 * (x - 1)));
      a = _mm_unpacklo_epi8(loadA, loadB);
      b = _mm_unpackhi_epi8(loadA, loadB);
      c = _mm_unpacklo_epi8(a, b);
      d = _mm_unpackhi_epi8(a, b);
      a = _mm_unpacklo_epi8(c, d);
      b = _mm_unpackhi_epi8(c, d);
      __m128i row_1 = _mm_unpacklo_epi8(a, b);

      // row_1 = _mm_srli_epi8(row_1, 2);

      // Same as above for third row
      loadA = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_2_ptr + 2 * (x - 1)));
      loadB = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_2_ptr + 16 + 2 * (x - 1)));
      a = _mm_unpacklo_epi8(loadA, loadB);
      b = _mm_unpackhi_epi8(loadA, loadB);
      c = _mm_unpacklo_epi8(a, b);
      d = _mm_unpackhi_epi8(a, b);
      a = _mm_unpacklo_epi8(c, d);
      b = _mm_unpackhi_epi8(c, d);
      __m128i row_2 = _mm_unpacklo_epi8(a, b);

      // row_2 = _mm_srli_epi8(row_2, 2);

      // --------------------------------------------------------------------------------------------------------

      // Now the actual calculations are performed
      // Since sobel needs a 3 by 3 surrounding of a pixel, we shift the rows to the right, so that
      // the correct values will be added
      // for example:
      // Y0 | Y1 | Y2
      // Y3 | xx | Y4
      // Y5 | Y6 | Y7
      // To calculate the result xx we need to calculate gx = Y0 + Y3 * 2 + Y5 - Y2 - Y4 * 2 - Y7
      // To get all these values in the first 8 bit values of a register, the shifts are done
      // i.e. row_0 is shifted 2 Bytes to the right, to get Y2 in the first 8-bit of the register
      __m128i row_0_shifts_0 = row_0;
      __m128i row_0_shifts_1 = _mm_srli_si128(row_0, 1);
      __m128i row_0_shifts_2 = _mm_srli_si128(row_0, 2);
      __m128i row_1_shifts_0 = row_1;
      __m128i row_1_shifts_2 = _mm_srli_si128(row_1, 2);
      __m128i row_2_shifts_0 = row_2;
      __m128i row_2_shifts_1 = _mm_srli_si128(row_2, 1);
      __m128i row_2_shifts_2 = _mm_srli_si128(row_2, 2);

      // Multiply by 2 (see above, Y3 and Y4 need to be multiplied by 2)
      row_1_shifts_0 = _mm_srli_epi8(row_1_shifts_0, 1);
      row_1_shifts_2 = _mm_srli_epi8(row_1_shifts_2, 1);

      row_0_shifts_0 = _mm_srli_epi8(row_0_shifts_0, 2);
      row_2_shifts_0 = _mm_srli_epi8(row_2_shifts_0, 2);
      row_2_shifts_2 = _mm_srli_epi8(row_2_shifts_2, 2);
      row_0_shifts_2 = _mm_srli_epi8(row_0_shifts_2, 2);

      __m128i gx;
      if (dir == Uni || dir == Horizontal)
      {
        // Calculate positiv and negativ sum for X direction
        __m128i gx_pos = _mm_adds_epu8(row_0_shifts_0, _mm_adds_epu8(row_2_shifts_0, row_1_shifts_0));
        __m128i gx_neg = _mm_adds_epu8(row_0_shifts_2, _mm_adds_epu8(row_2_shifts_2, row_1_shifts_2));

        // Calulate the absolute difference between the positive and negative sum in X direction
        gx = _mm_subs_epu8(_mm_max_epu8(gx_pos, gx_neg), _mm_min_epu8(gx_pos, gx_neg));
      }

      __m128i gy;
      if (dir == Uni || dir == Vertical)
      {
        // Same for gy
        // Multiply by 2 (see above)
        row_0_shifts_1 = _mm_srli_epi8(row_0_shifts_1, 1);
        row_2_shifts_1 = _mm_srli_epi8(row_2_shifts_1, 1);

        __m128i gy_pos = _mm_adds_epu8(row_0_shifts_0, _mm_adds_epu8(row_0_shifts_1, row_0_shifts_2));
        __m128i gy_neg = _mm_adds_epu8(row_2_shifts_1, _mm_adds_epu8(row_2_shifts_0, row_2_shifts_2));

        gy = _mm_subs_epu8(_mm_max_epu8(gy_pos, gy_neg), _mm_min_epu8(gy_pos, gy_neg));
      }

      __m128i result;
      if (dir == Uni)
      {
        // The result should be calculated as sqrt( pow(gx,2) + pow(gy,2) )
        // Since this is really slow and not easily done in 8-Bit because of massive overflow
        // we gonna approximate this with the "Alpha max plus beta min"-algorithm
        // with alpha = 1 and beta = 1/4 (since this is only a bitshift for the smaller value)
        // see for example http://en.wikipedia.org/wiki/Alpha_max_plus_beta_min_algorithm
        // or http://www.dspguru.com/dsp/tricks/magnitude-estimator
        __m128i mins = _mm_min_epu8(gx, gy);
        __m128i maxs = _mm_max_epu8(gx, gy);
        mins = _mm_srli_epi8(mins, 2);
        result = _mm_adds_epu8(mins, maxs);
      }
      else if (dir == Horizontal)
      {
        result = gx;
      }
      else
      {
        result = gy;
      }

      // Store the result
      if (returnFullArray)
      {
        _mm_storeu_si128(reinterpret_cast<__m128i *>(target + x + y * width), result);
      }
      else
      {
        _mm_storeu_si128(reinterpret_cast<__m128i *>(target + x - start(0) + (y - start(1)) * rectWidth), result);
      }

      // Increase the row pointers
      row_0_ptr += 2 * width;
      row_1_ptr += 2 * width;
      row_2_ptr += 2 * width;
    }
  }


  if (returnFullArray)
  {
    // top edge above rectangle
    for (int i = 0; i <= start(1); ++i)
    {
      for (int j = 0; j < width; ++j)
      {
        target[i * width + j] = 0;
      }
    }

    // bottom edge below rectangle
    for (int i = end(1) - 1; i < height; ++i)
    {
      for (int j = 0; j < width; ++j)
      {
        target[i * width + j] = 0;
      }
    }

    // Left edge, but without top and lower edge
    for (int j = 0; j <= start(0); ++j)
    {
      for (int i = start(1) + 1; i <= end(1); ++i)
      {
        target[i * width + j] = 0;
      }
    }

    // right edge, but without top and lower edge
    for (int j = end(0); j < width; ++j)
    {
      for (int i = start(1) + 1; i <= end(1); ++i)
      {
        target[i * width + j] = 0;
      }
    }
  }
  else
  {
    for (int i = 0; i < rectHeight; ++i)
    {
      target[i * (rectWidth)] = 0;          // Set left edge to 0
      target[(i + 1) * (rectWidth)-1] = 0;  // Set right edge to 0
    }
    for (int j = 0; j < rectWidth; ++j)
    {
      target[j] = 0;                                   // Set top edge to 0
      target[(rectHeight - 1) * (rectWidth) + j] = 0;  // Set bottom edge to 0
    }
  }


  return targetData;
}

stdVector2D<unsigned char> SobelDortmund::sobelSSEAnyYUVImageQuarter(const Image &YUVImage, Vector2i start, Vector2i end, int width, int height,
                                                                     Direction dir, bool returnFullArray)
{
  unsigned char *image_ptr = (unsigned char *)YUVImage.image;


  switchStartEnd(start, end);
  int rectWidth = end(0) - start(0) + 1;
  int rectHeight = end(1) - start(1) + 1;

  stdVector2D<unsigned char> targetData(returnFullArray ? width : rectWidth, returnFullArray ? height : rectHeight);

  unsigned char *target = targetData.data();

  // Calculate the pointers to the first element of the first 3 rows using pointer arithmetic
  unsigned char *row_0_ptr = image_ptr + start(1) * width * 8;
  unsigned char *row_1_ptr = row_0_ptr + 8 * width;
  unsigned char *row_2_ptr = row_0_ptr + 16 * width;

  int i = 1;
  int j = 1;

  if (returnFullArray)
  {
    i = start(1) + 1;
  }


  // Main Loop, we will start at (1,1) because Sobel needs a 3x3 surrounding, which is
  // not available for edge pixels
  // Every row will be looped through, but only every 14th column, because with 8 bit SSE
  // it is possible to store 16 8-bit values in one SSE register, which will give 14 results (minus edge pixels included!)
  for (int y = start(1) + 1; y < end(1); y++)
  {
    if (returnFullArray)
    {
      j = start(0) + 1;
    }
    else
    {
      j = 1;
    }
    for (int x = start(0) * 2 + 1; x <= end(0) * 2; x += 28)
    {
      // Now we need to load 16 Y values per row into 1 register
      // This is done with SSE unpack, which works as in the following example:
      // Pointer a: a0, a1, a3, ... , a15 | Pointer b: b0, b1, b3, ... , b15 (each value ai/bi has the size 1 Byte)
      // _mm_unpacklo_epi8(a,b) will result in a register with the 16 8-Bit values: a0, b0, a1, b1, ... , a7, b7
      // _mm_unpackhi_epi8(a,b) will result in a register with the 16 8-Bit values: a8, b8, a9, b9, ... , a15, b15

      // We want to use these unpack operations to deinterlace our YUV422 array, so that we will have two registers
      // one containing all the Y values in correct order and the other one containing all U and V values

      // --------------------------------------------------------------------------------------------------------

      // The following commands will deinterlace the Y values for the first row:
      // Let the YUV422 array be: Y0, U0, Y1, V0, Y2, U1, Y3, V1, Y4, U2, Y5, V2, ...
      // Unpack the first 16 Values with the second 16 values into a (lo)
      // This will result in a = Y0, Y8, U0, U4, Y1, Y9, V0, V4, ...
      // likewise b will be (hi): Y5, Y12, U2, U6, Y5, Y13, V2, V6, ...

      // The pointer will need an offset of 2 * (x-1)
      // -1: because we started this loop with x = 1
      // 2 * : because only every second value is a Y value

      __m128i loadA = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_0_ptr + 2 * (x - 1)));
      __m128i loadB = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_0_ptr + 16 + 2 * (x - 1)));
      __m128i loadC = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_0_ptr + 32 + 2 * (x - 1)));
      __m128i loadD = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_0_ptr + 48 + 2 * (x - 1)));

      __m128i a = _mm_unpacklo_epi8(loadA, loadB);
      __m128i b = _mm_unpackhi_epi8(loadA, loadB);

      // We will unpack again, resulting in 4 continous Y values per register
      // i.e. c = Y0, Y4, Y8, Y12, U0, U2, U4, U6, ...
      //      d = Y2, Y6, Y10, Y14, U1, U3, U5, U7, ...
      __m128i c = _mm_unpacklo_epi8(a, b);
      __m128i d = _mm_unpackhi_epi8(a, b);

      // Next unpack will result in 8 continous Y values per register
      // i.e. a = Y0, Y2, Y4, ... , Y14, U0, U1, ...
      //      b = Y1, Y3, Y5, ... , Y15, V0, V1, ...
      a = _mm_unpacklo_epi8(c, d);
      b = _mm_unpackhi_epi8(c, d);

      // For the last unpack we only need the lower 8 values from each register, because these are the Y values
      // row_0 = Y0, Y1, Y2, Y3, ... , Y15
      // The unpackhi would give: U0, V0, U1, V1, ... but since we do not want to use the color values, this won't be calculated
      __m128i row_0_part_1 = _mm_unpacklo_epi8(a, b);

      a = _mm_unpacklo_epi8(loadC, loadD);
      b = _mm_unpackhi_epi8(loadC, loadD);
      c = _mm_unpacklo_epi8(a, b);
      d = _mm_unpackhi_epi8(a, b);
      a = _mm_unpacklo_epi8(c, d);
      b = _mm_unpackhi_epi8(c, d);
      __m128i row_0_part_2 = _mm_unpacklo_epi8(a, b);

      a = _mm_unpacklo_epi8(row_0_part_1, row_0_part_2);
      b = _mm_unpackhi_epi8(row_0_part_1, row_0_part_2);

      c = _mm_unpacklo_epi8(a, b);
      d = _mm_unpackhi_epi8(a, b);

      a = _mm_unpacklo_epi8(c, d);
      b = _mm_unpackhi_epi8(c, d);

      __m128i row_0 = _mm_unpacklo_epi8(a, b);

      // Same as above for second row
      loadA = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_1_ptr + 2 * (x - 1)));
      loadB = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_1_ptr + 16 + 2 * (x - 1)));
      loadC = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_1_ptr + 32 + 2 * (x - 1)));
      loadD = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_1_ptr + 48 + 2 * (x - 1)));
      a = _mm_unpacklo_epi8(loadA, loadB);
      b = _mm_unpackhi_epi8(loadA, loadB);
      c = _mm_unpacklo_epi8(a, b);
      d = _mm_unpackhi_epi8(a, b);
      a = _mm_unpacklo_epi8(c, d);
      b = _mm_unpackhi_epi8(c, d);
      __m128i row_1_part_1 = _mm_unpacklo_epi8(a, b);

      a = _mm_unpacklo_epi8(loadC, loadD);
      b = _mm_unpackhi_epi8(loadC, loadD);
      c = _mm_unpacklo_epi8(a, b);
      d = _mm_unpackhi_epi8(a, b);
      a = _mm_unpacklo_epi8(c, d);
      b = _mm_unpackhi_epi8(c, d);
      __m128i row_1_part_2 = _mm_unpacklo_epi8(a, b);

      a = _mm_unpacklo_epi8(row_1_part_1, row_1_part_2);
      b = _mm_unpackhi_epi8(row_1_part_1, row_1_part_2);

      c = _mm_unpacklo_epi8(a, b);
      d = _mm_unpackhi_epi8(a, b);

      a = _mm_unpacklo_epi8(c, d);
      b = _mm_unpackhi_epi8(c, d);

      __m128i row_1 = _mm_unpacklo_epi8(a, b);


      // Same as above for third row
      loadA = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_2_ptr + 2 * (x - 1)));
      loadB = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_2_ptr + 16 + 2 * (x - 1)));
      loadC = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_2_ptr + 32 + 2 * (x - 1)));
      loadD = _mm_loadu_si128(reinterpret_cast<__m128i *>(row_2_ptr + 48 + 2 * (x - 1)));
      a = _mm_unpacklo_epi8(loadA, loadB);
      b = _mm_unpackhi_epi8(loadA, loadB);
      c = _mm_unpacklo_epi8(a, b);
      d = _mm_unpackhi_epi8(a, b);
      a = _mm_unpacklo_epi8(c, d);
      b = _mm_unpackhi_epi8(c, d);
      __m128i row_2_part_1 = _mm_unpacklo_epi8(a, b);

      a = _mm_unpacklo_epi8(loadC, loadD);
      b = _mm_unpackhi_epi8(loadC, loadD);
      c = _mm_unpacklo_epi8(a, b);
      d = _mm_unpackhi_epi8(a, b);
      a = _mm_unpacklo_epi8(c, d);
      b = _mm_unpackhi_epi8(c, d);
      __m128i row_2_part_2 = _mm_unpacklo_epi8(a, b);

      a = _mm_unpacklo_epi8(row_2_part_1, row_2_part_2);
      b = _mm_unpackhi_epi8(row_2_part_1, row_2_part_2);

      c = _mm_unpacklo_epi8(a, b);
      d = _mm_unpackhi_epi8(a, b);

      a = _mm_unpacklo_epi8(c, d);
      b = _mm_unpackhi_epi8(c, d);

      __m128i row_2 = _mm_unpacklo_epi8(a, b);

      // --------------------------------------------------------------------------------------------------------

      // Now the actual calculations are performed
      // Since sobel needs a 3 by 3 surrounding of a pixel, we shift the rows to the right, so that
      // the correct values will be added
      // for example:
      // Y0 | Y1 | Y2
      // Y3 | xx | Y4
      // Y5 | Y6 | Y7
      // To calculate the result xx we need to calculate gx = Y0 + Y3 * 2 + Y5 - Y2 - Y4 * 2 - Y7
      // To get all these values in the first 8 bit values of a register, the shifts are done
      // i.e. row_0 is shifted 2 Bytes to the right, to get Y2 in the first 8-bit of the register
      __m128i row_0_shifts_0 = row_0;
      __m128i row_0_shifts_1 = _mm_srli_si128(row_0, 1);
      __m128i row_0_shifts_2 = _mm_srli_si128(row_0, 2);
      __m128i row_1_shifts_0 = row_1;
      __m128i row_1_shifts_2 = _mm_srli_si128(row_1, 2);
      __m128i row_2_shifts_0 = row_2;
      __m128i row_2_shifts_1 = _mm_srli_si128(row_2, 1);
      __m128i row_2_shifts_2 = _mm_srli_si128(row_2, 2);

      // Multiply by 2 (see above, Y3 and Y4 need to be multiplied by 2)
      row_1_shifts_0 = _mm_srli_epi8(row_1_shifts_0, 1);
      row_1_shifts_2 = _mm_srli_epi8(row_1_shifts_2, 1);

      row_0_shifts_0 = _mm_srli_epi8(row_0_shifts_0, 2);
      row_2_shifts_0 = _mm_srli_epi8(row_2_shifts_0, 2);
      row_2_shifts_2 = _mm_srli_epi8(row_2_shifts_2, 2);
      row_0_shifts_2 = _mm_srli_epi8(row_0_shifts_2, 2);


      __m128i gx;
      if (dir == Uni || dir == Horizontal)
      {
        // Calculate positiv and negativ sum for X direction
        __m128i gx_pos = _mm_adds_epu8(row_0_shifts_0, _mm_adds_epu8(row_2_shifts_0, row_1_shifts_0));
        __m128i gx_neg = _mm_adds_epu8(row_0_shifts_2, _mm_adds_epu8(row_2_shifts_2, row_1_shifts_2));

        // Calulate the absolute difference between the positive and negative sum in X direction
        gx = _mm_subs_epu8(_mm_max_epu8(gx_pos, gx_neg), _mm_min_epu8(gx_pos, gx_neg));
      }

      __m128i gy;
      if (dir == Uni || dir == Vertical)
      {
        // Same for gy
        // Multiply by 2 (see above)
        row_0_shifts_1 = _mm_srli_epi8(row_0_shifts_1, 1);
        row_2_shifts_1 = _mm_srli_epi8(row_2_shifts_1, 1);

        __m128i gy_pos = _mm_adds_epu8(row_0_shifts_0, _mm_adds_epu8(row_0_shifts_1, row_0_shifts_2));
        __m128i gy_neg = _mm_adds_epu8(row_2_shifts_1, _mm_adds_epu8(row_2_shifts_0, row_2_shifts_2));

        gy = _mm_subs_epu8(_mm_max_epu8(gy_pos, gy_neg), _mm_min_epu8(gy_pos, gy_neg));
      }

      __m128i result;
      if (dir == Uni)
      {
        // The result should be calculated as sqrt( pow(gx,2) + pow(gy,2) )
        // Since this is really slow and not easily done in 8-Bit because of massive overflow
        // we gonna approximate this with the "Alpha max plus beta min"-algorithm
        // with alpha = 1 and beta = 1/4 (since this is only a bitshift for the smaller value)
        // see for example http://en.wikipedia.org/wiki/Alpha_max_plus_beta_min_algorithm
        // or http://www.dspguru.com/dsp/tricks/magnitude-estimator
        __m128i mins = _mm_min_epu8(gx, gy);
        __m128i maxs = _mm_max_epu8(gx, gy);
        mins = _mm_srli_epi8(mins, 2);
        result = _mm_adds_epu8(mins, maxs);
      }
      else if (dir == Horizontal)
      {
        result = gx;
      }
      else
      {
        result = gy;
      }

      // Store the result
      if (returnFullArray)
      {
        _mm_storeu_si128(reinterpret_cast<__m128i *>(target + j + i * width), result);
      }
      else
      {
        _mm_storeu_si128(reinterpret_cast<__m128i *>(target + j + i * rectWidth), result);
      }

      j += 14;
    }
    i++;
    // Increase the row pointers
    row_0_ptr += 8 * width;
    row_1_ptr += 8 * width;
    row_2_ptr += 8 * width;
  }


  if (returnFullArray)
  {
    // top edge above rectangle
    for (int i = 0; i <= start(1); ++i)
    {
      for (int j = 0; j < rectWidth; ++j)
      {
        target[i * width + j] = 0;
      }
    }

    // bottom edge below rectangle
    for (int i = end(1); i < height; ++i)
    {
      for (int j = 0; j < width; ++j)
      {
        target[i * width + j] = 0;
      }
    }

    // Left edge, but without top and lower edge
    for (int j = 0; j <= start(0); ++j)
    {
      for (int i = start(1) + 1; i <= end(1); ++i)
      {
        target[i * width + j] = 0;
      }
    }

    // right edge, but without top and lower edge
    for (int j = end(0); j < width; ++j)
    {
      for (int i = start(1) + 1; i <= end(1); ++i)
      {
        target[i * width + j] = 0;
      }
    }
  }
  else
  {
    for (int i = 0; i < rectHeight; ++i)
    {
      target[i * (rectWidth)] = 0;          // Set left edge to 0
      target[(i + 1) * (rectWidth)-1] = 0;  // Set right edge to 0
    }
    for (int j = 0; j < rectWidth; ++j)
    {
      target[j] = 0;                                   // Set top edge to 0
      target[(rectHeight - 1) * (rectWidth) + j] = 0;  // Set bottom edge to 0
    }
  }

  return targetData;
}


void SobelDortmund::switchStartEnd(Vector2i &start, Vector2i &end)
{
  Vector2i bufferStart = start;
  // Switch start and end to that it is always a top left and a bottom right corner
  if (start(0) > end(0))
  {
    start(0) = end(0);
    end(0) = bufferStart(0);
  }

  if (start(1) > end(1))
  {
    start(1) = end(1);
    end(1) = bufferStart(1);
  }
}
