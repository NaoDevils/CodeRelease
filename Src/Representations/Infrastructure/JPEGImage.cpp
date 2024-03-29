/**
 * @file JPEGImage.cpp
 *
 * Implementation of struct JPEGImage
 */

#include "Tools/SIMD.h"
#include "Platform/BHAssert.h"
#include "JPEGImage.h"
#include "Tools/SSE.h"
#include "Platform/SystemCall.h"
#include <cstddef>

JPEGImage::JPEGImage(const Image& image)
{
  *this = image;
}

JPEGImage& JPEGImage::operator=(const Image& src)
{
  fromImage(src);
  return *this;
}

void JPEGImage::fromImage(const Image& src, int quality)
{
  setResolution(src.width, src.height);
  timeStamp = src.timeStamp;

  jpeg_compress_struct cInfo;
  jpeg_error_mgr jem;
  cInfo.err = jpeg_std_error(&jem);
  jpeg_create_compress(&cInfo);

  if (!cInfo.dest)
    cInfo.dest = (jpeg_destination_mgr*)(*cInfo.mem->alloc_small)((j_common_ptr)&cInfo, JPOOL_PERMANENT, sizeof(jpeg_destination_mgr));
  cInfo.dest->init_destination = onDestIgnore;
  cInfo.dest->empty_output_buffer = onDestEmpty;
  cInfo.dest->term_destination = onDestIgnore;
  cInfo.dest->next_output_byte = reinterpret_cast<JOCTET*>(image);
  ASSERT(!isReference);
  cInfo.dest->free_in_buffer = maxResolutionWidth * maxResolutionHeight * 2 * sizeof(Pixel);

  cInfo.image_width = width;
  cInfo.image_height = height;
  cInfo.input_components = 3;
  cInfo.in_color_space = JCS_YCbCr;
  cInfo.jpeg_color_space = JCS_YCbCr;

  jpeg_set_defaults(&cInfo);

  cInfo.raw_data_in = true;

  // disable chroma subsampling
  cInfo.comp_info[0].h_samp_factor = 1;
  cInfo.comp_info[0].v_samp_factor = 1;

  cInfo.dct_method = JDCT_FASTEST;
  jpeg_set_quality(&cInfo, quality, true);

  jpeg_start_compress(&cInfo, true);

  JPEGRawData data;
  while (cInfo.next_scanline < cInfo.image_height)
  {
    toJPEGData(src, data.rows.data(), cInfo.next_scanline, data.y_plane.size());
    jpeg_write_raw_data(&cInfo, data.rows.data(), static_cast<JDIMENSION>(data.y_plane.size()));
  }

  jpeg_finish_compress(&cInfo);
  size = static_cast<unsigned>(cInfo.dest->next_output_byte - reinterpret_cast<JOCTET*>(image));
  jpeg_destroy_compress(&cInfo);
}

void JPEGImage::toImage(Image& dest) const
{
  dest.setResolution(width, height);
  dest.timeStamp = timeStamp;
  dest.imageSource = ImageSource::jpegImage;

  jpeg_decompress_struct cInfo;
  jpeg_error_mgr jem;
  cInfo.err = jpeg_std_error(&jem);

  jpeg_create_decompress(&cInfo);

  if (!cInfo.src)
    cInfo.src = (jpeg_source_mgr*)(*cInfo.mem->alloc_small)((j_common_ptr)&cInfo, JPOOL_PERMANENT, sizeof(jpeg_source_mgr));
  cInfo.src->init_source = onSrcIgnore;
  cInfo.src->fill_input_buffer = onSrcEmpty;
  cInfo.src->skip_input_data = onSrcSkip;
  cInfo.src->resync_to_restart = jpeg_resync_to_restart;
  cInfo.src->term_source = onSrcIgnore;
  cInfo.src->bytes_in_buffer = size;
  cInfo.src->next_input_byte = (const JOCTET*)(*this)[0];

  jpeg_read_header(&cInfo, true);
  if (cInfo.num_components == 3 && cInfo.jpeg_color_space == JCS_YCbCr) // new JPEG-compression
  {
    cInfo.raw_data_out = true;
    jpeg_start_decompress(&cInfo);

    JPEGRawData data;
    while (cInfo.output_scanline < cInfo.image_height)
    {
      jpeg_read_raw_data(&cInfo, data.rows.data(), static_cast<JDIMENSION>(data.y_plane.size()));
      fromJPEGData(data.rows.data(), dest, cInfo.output_scanline - data.y_plane.size(), data.y_plane.size());
    }
  }
  else if (cInfo.num_components == 1 && cInfo.jpeg_color_space == JCS_GRAYSCALE) // old JPEG-compression
  {
    jpeg_start_decompress(&cInfo);
    std::vector<unsigned char> aiboAlignedImage(width * height * 3);

    // setup rows
    while (cInfo.output_scanline < cInfo.output_height)
    {
      JSAMPROW rowPointer = &aiboAlignedImage[cInfo.output_scanline * cInfo.output_width];
      jpeg_read_scanlines(&cInfo, &rowPointer, 1);
    }

    fromAiboAlignment(aiboAlignedImage.data(), reinterpret_cast<unsigned char*>(dest.image));
  }
  else
  {
    ASSERT(false);
  }

  // finish decompress
  jpeg_finish_decompress(&cInfo);
  jpeg_destroy_decompress(&cInfo);
}

boolean JPEGImage::onDestEmpty(j_compress_ptr)
{
  ASSERT(false);
  return false;
}

void JPEGImage::onDestIgnore(j_compress_ptr) {}

void JPEGImage::onSrcSkip(j_decompress_ptr, long) {}

boolean JPEGImage::onSrcEmpty(j_decompress_ptr)
{
  ASSERT(false);
  return false;
}

void JPEGImage::onSrcIgnore(j_decompress_ptr) {}

void JPEGImage::fromJPEGData(const JSAMPIMAGE src, Image& dst, size_t yOffset, size_t ySize)
{
  ASSERT(dst.width % 16 == 0);
  ASSERT(reinterpret_cast<size_t>(dst.image) % 16 == 0);

  __m128i *pDst, *pDstLineEnd;
  const __m128i *pSrcY, *pSrcCb, *pSrcCr;

  __m128i mY, mCb, mCr;
  __m128i mLowYCb, mHighYCb, mLowYCr, mHighYCr;
  __m128i p0, p1, p2, p3;

  for (size_t y = 0; y < ySize; ++y)
  {
    pDst = reinterpret_cast<__m128i*>(dst.image + (y + yOffset) * dst.widthStep);
    pDstLineEnd = reinterpret_cast<__m128i*>(dst.image + (y + yOffset) * dst.widthStep + dst.width);
    pSrcY = reinterpret_cast<__m128i*>(src[0][y]);
    pSrcCb = reinterpret_cast<__m128i*>(src[1][y]);
    pSrcCr = reinterpret_cast<__m128i*>(src[2][y]);
    for (; pDst < pDstLineEnd; pDst += 4, ++pSrcY, ++pSrcCb, ++pSrcCr)
    {
      mY = _mm_load_si128(pSrcY);
      mCb = _mm_load_si128(pSrcCb);
      mCr = _mm_load_si128(pSrcCr);

      mLowYCb = _mm_unpacklo_epi8(mY, mCb); // y1 cb1 y2 cb2 y3 cb3 y4 cb4 y5 cb5 y6 cb6 y7 cb7 y8 cb8
      mHighYCb = _mm_unpackhi_epi8(mY, mCb); // y9 cb9 y10 cb10 y11 cb11 y12 cb12 y13 cb13 y14 cb14 y15 cb15 y16 cb16
      mLowYCr = _mm_unpacklo_epi8(mY, mCr); // y1 cr1 y2 cr2 y3 cr3 y4 cr4 y5 cr5 y6 cr6 y7 cr7 y8 cr8
      mHighYCr = _mm_unpackhi_epi8(mY, mCr); // y9 cr9 y10 cr10 y11 cr11 y12 cr12 y13 cr13 y14 cr14 y15 cr15 y16 cr16

      p0 = _mm_unpacklo_epi16(mLowYCb, mLowYCr); // y1 cb1 y1 cr1 y2 cb2 y2 cr2 y3 cb3 y3 cr3 y4 cb4 y4 cr4
      p1 = _mm_unpackhi_epi16(mLowYCb, mLowYCr); // y5 cb5 y5 cr5 y6 cb6 y6 cr6 y7 cb7 y7 cr7 y8 cb8 y8 cr8
      p2 = _mm_unpacklo_epi16(mHighYCb, mHighYCr); // y9 cb9 y9 cr9 y10 cb10 y10 cr10 y11 cb11 y11 cr11 y12 cb12 y12 cr12
      p3 = _mm_unpackhi_epi16(mHighYCb, mHighYCr); // y12 cb12 y12 cr12 y13 cb13 y13 cr13 y14 cb14 y14 cr14 y15 cb15 y15 cr15

      _mm_store_si128(pDst, p0);
      _mm_store_si128(pDst + 1, p1);
      _mm_store_si128(pDst + 2, p2);
      _mm_store_si128(pDst + 3, p3);
    }
  }
}

void JPEGImage::toJPEGData(const Image& src, JSAMPIMAGE dst, size_t yOffset, size_t ySize)
{
  ASSERT(src.width % 16 == 0);
  ASSERT(reinterpret_cast<size_t>(src.image) % 16 == 0);

  const std::array<unsigned char, 16> mask = {offsetof(Image::Pixel, cb),
      offsetof(Image::Pixel, cb) + 4,
      offsetof(Image::Pixel, cb) + 8,
      offsetof(Image::Pixel, cb) + 12,
      offsetof(Image::Pixel, y),
      offsetof(Image::Pixel, y) + 4,
      offsetof(Image::Pixel, y) + 8,
      offsetof(Image::Pixel, y) + 12,
      offsetof(Image::Pixel, cr),
      offsetof(Image::Pixel, cr) + 4,
      offsetof(Image::Pixel, cr) + 8,
      offsetof(Image::Pixel, cr) + 12,
      0xFF,
      0xFF,
      0xFF,
      0xFF};

  const __m128i mMask = _mm_loadu_si128(reinterpret_cast<const __m128i*>(&mask));

  const __m128i *pSrc, *pSrcLineEnd;
  __m128i *pDstY, *pDstCb, *pDstCr;

  __m128i p0, p1, p2, p3;
  __m128i mLowCbY, mHighCbY, mLowCr, mHighCr;
  __m128i mY, mCb, mCr;

  for (size_t y = 0; y < ySize; ++y)
  {
    pSrc = reinterpret_cast<__m128i*>(src.image + (y + yOffset) * src.widthStep);
    pSrcLineEnd = reinterpret_cast<__m128i*>(src.image + (y + yOffset) * src.widthStep + src.width);
    pDstY = reinterpret_cast<__m128i*>(dst[0][y]);
    pDstCb = reinterpret_cast<__m128i*>(dst[1][y]);
    pDstCr = reinterpret_cast<__m128i*>(dst[2][y]);
    for (; pSrc < pSrcLineEnd; pSrc += 4, ++pDstY, ++pDstCb, ++pDstCr)
    {
      p0 = _mm_load_si128(pSrc); // yPadd1 cb1 y1 cr1 yPadd2 cb2 y2 cr2 yPadd3 cb3 y3 cr3 yPadd4 cb4 y4 cr4
      p1 = _mm_load_si128(pSrc + 1); // yPadd5 cb5 y5 cr5 yPadd6 cb6 y6 cr6 yPadd7 cb7 y7 cr7 yPadd8 cb8 y8 cr8
      p2 = _mm_load_si128(pSrc + 2); // yPadd9 cb9 y9 cr9 yPadd10 cb10 y10 cr10 yPadd11 cb11 y11 cr11 yPadd12 cb12 y12 cr12
      p3 = _mm_load_si128(pSrc + 3); // yPadd13 cb13 y13 cr13 yPadd14 cb14 y14 cr14 yPadd15 cb15 y15 cr15 yPadd16 cb16 y16 cr16

      p0 = _mm_shuffle_epi8(p0, mMask); // cb1 cb2 cb3 cb4 y1 y2 y3 y4 cr1 cr2 cr3 cr4 0 0 0 0
      p1 = _mm_shuffle_epi8(p1, mMask); // cb5 cb6 cb7 cb8 y5 y6 y7 y8 cr5 cr6 cr7 cr8 0 0 0 0
      p2 = _mm_shuffle_epi8(p2, mMask); // cb9 cb10 cb11 cb12 y9 y10 y11 y12 cr9 cr10 cr11 cr12 0 0 0 0
      p3 = _mm_shuffle_epi8(p3, mMask); // cb13 cb14 cb15 cb16 y13 y14 y15 y16 cr13 cr14 cr15 cr16 0 0 0 0

      mLowCbY = _mm_unpacklo_epi32(p0, p1); // cb1 cb2 cb3 cb4 cb5 cb6 cb7 cb8 y1 y2 y3 y4 y5 y6 y7 y8
      mHighCbY = _mm_unpacklo_epi32(p2, p3); // cb9 cb10 cb11 cb12 cb13 cb14 cb15 cb16 y9 y10 y11 y12 y13 y14 y15 y16
      mLowCr = _mm_unpackhi_epi32(p0, p1); // cr1 cr2 cr3 cr4 cr5 cr6 cr7 cr8 0 0 0 0 0 0 0 0
      mHighCr = _mm_unpackhi_epi32(p2, p3); // cr9 cr10 cr11 cr12 cr13 cr14 cr15 cr16 0 0 0 0 0 0 0 0

      mY = _mm_unpackhi_epi64(mLowCbY, mHighCbY);
      mCb = _mm_unpacklo_epi64(mLowCbY, mHighCbY);
      mCr = _mm_unpacklo_epi64(mLowCr, mHighCr);

      _mm_store_si128(pDstY, mY);
      _mm_store_si128(pDstCb, mCb);
      _mm_store_si128(pDstCr, mCr);
    }
  }
}

void JPEGImage::fromAiboAlignment(const unsigned char* src, unsigned char* dst) const
{
  const int resolutionWidth(this->width);
  for (int y = 0; y < height; y++)
  {
    const unsigned char* pSrc = src + y * resolutionWidth * 3;
    unsigned char* pDst = dst + y * widthStep * 4;
    for (int x = 0; x < resolutionWidth; x++)
    {
      pDst[1] = pSrc[0];
      pDst[0] = pDst[2] = pSrc[resolutionWidth];
      pDst[3] = pSrc[2 * resolutionWidth];
      pSrc++;
      pDst += 4;
    }
  }
}

void JPEGImage::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(width);
  STREAM(height);
  STREAM(timeStamp);
  timeStamp &= ~(1 << 31); // remove legacy isFullSize bit

  STREAM(size);
  if (in)
  {
    widthStep = 2 * width;
    in->read((*this)[0], size);
  }
  else
    out->write((*this)[0], size);
  STREAM_REGISTER_FINISH;
}
