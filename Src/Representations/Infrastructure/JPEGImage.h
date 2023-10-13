/**
 * @file JPEGImage.h
 *
 * Declaration of struct JPEGImage
 */

#pragma once

#include "Representations/Infrastructure/Image.h"

#ifdef WINDOWS

// INT32 and FAR conflict with any other header files...
#define INT32 _INT32
#undef FAR

// "boolean" conflicts with "rpcndr.h", so we force "jpeglib.h" not to define boolean
#ifdef __RPCNDR_H__
#define HAVE_BOOLEAN
#endif

#include <jpeglib.h>

#undef INT32
#undef FAR

#else

extern "C"
{
#include <jpeglib.h>
}

#endif

/**
 * Definition of a struct for JPEG-compressed images.
 */
struct JPEGImage : public Image
{
private:
  unsigned size; /**< The size of the compressed image. */

public:
  JPEGImage() = default;

  /**
   * Constructs a JPEG image from an image.
   * @param src The image used as template.
   */
  JPEGImage(const Image& src);

  /**
   * Assignment operator.
   * @param src The image used as template.
   * @return The resulting JPEG image.
   */
  JPEGImage& operator=(const Image& src);

  /**
   * Compress image.
   * @param src The uncompressed image.
   * @param quality The JPEG quality (0...100).
   */
  void fromImage(const Image& src, int quality = 85);

  /**
   * Uncompress image.
   * @param dest Will receive the uncompressed image.
   */
  void toImage(Image& dest) const;

private:
  //!@name Handlers for JPEG-compression
  //!@{
  static boolean onDestEmpty(j_compress_ptr);
  static void onDestIgnore(j_compress_ptr);

  static void onSrcSkip(j_decompress_ptr cInfo, long numBytes);
  static boolean onSrcEmpty(j_decompress_ptr);
  static void onSrcIgnore(j_decompress_ptr);
  //!@}

  struct JPEGRawData
  {
    // since we are not using chroma subsampling, we can directly use DCTSIZE here
    alignas(16) std::array<std::array<JSAMPLE, maxResolutionWidth>, DCTSIZE> y_plane;
    alignas(16) std::array<std::array<JSAMPLE, maxResolutionWidth>, DCTSIZE> u_plane;
    alignas(16) std::array<std::array<JSAMPLE, maxResolutionWidth>, DCTSIZE> v_plane;

    std::array<JSAMPROW, DCTSIZE> y_rows;
    std::array<JSAMPROW, DCTSIZE> u_rows;
    std::array<JSAMPROW, DCTSIZE> v_rows;

    std::array<JSAMPARRAY, 3> rows;

    JPEGRawData()
    {
      for (size_t i = 0; i < y_rows.size(); ++i)
        y_rows[i] = y_plane[i].data();
      for (size_t i = 0; i < u_rows.size(); ++i)
        u_rows[i] = u_plane[i].data();
      for (size_t i = 0; i < v_rows.size(); ++i)
        v_rows[i] = v_plane[i].data();

      rows = {y_rows.data(), u_rows.data(), v_rows.data()};
    }
  };

  /**
   * Convert image from Aibo's alignment (one channel per line) to Nao's alignment (YUV422)
   * destination is asserted to be allocated
   * @param src Pointer to the source image in Aibo's alignment
   * @param dst Pointer to the destination image
   */
  void fromAiboAlignment(const unsigned char* src, unsigned char* dst) const;

  static void toJPEGData(const Image& src, JSAMPIMAGE dst, size_t yOffset, size_t ySize);
  static void fromJPEGData(const JSAMPIMAGE src, Image& dst, size_t yOffset, size_t ySize);

  void serialize(In* in, Out* out);
};

struct JPEGImageUpper : JPEGImage
{
};
