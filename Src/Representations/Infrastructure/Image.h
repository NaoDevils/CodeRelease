/**
 * @file Image.h
 *
 * Declaration of struct Image
 */

#pragma once
#include "Tools/SIMD.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/ColorModelConversions.h"
#include <type_traits>

// TODO: check this warning
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <tmmintrin.h>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

ENUM(ImageSource,
  naoProviderV6,
  thumbnail,
  jpegImage,
  lowFrameRateImage,
  sequenceImage,
  yoloInput
);

/**
 * Platform independent definition of an image struct
 */
struct Image : public Streamable
{
public:
  /**
   * The union defines a pixel in YCbCr space.
   */

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4201) // nonstandard extension used : nameless struct/union
#endif
  union Pixel
  {
    unsigned color; /**< Representation as single machine word. */
    unsigned char channels[4]; /**< Representation as an array of channels. */
    struct
    {
      unsigned char yCbCrPadding; /**< Ignore. */
      unsigned char cb; /**< Cb channel. */
      unsigned char y; /**< Y channel. */
      unsigned char cr; /**< Cr channel. */
    };
    struct
    {
      unsigned char r; /**< R channel. */
      unsigned char g; /**< G channel. */
      unsigned char b; /**< B channel. */
      unsigned char rgbPadding; /**< Ignore. */
    };
    struct
    {
      unsigned char h; /**< H channel. */
      unsigned char s; /**< S channel. */
      unsigned char i; /**< I channel. */
      unsigned char hsiPadding; /**< Ignore. */
    };
  };
#ifdef _MSC_VER
#pragma warning(pop)
#endif

  static constexpr int maxResolutionWidth = 640;
  static constexpr int maxResolutionHeight = 480;

  int width; /**< The width of the image in pixel */
  int height; /**< The height of the image in pixel */
  int widthStep; /**< The Distance between the first pixels of subsequent lines. */
  unsigned timeStamp = 0; /**< The time stamp of this image. */
  bool isReference = false; /**< States whether this struct holds the image, or only a reference to an image stored elsewhere. */
  ImageSource imageSource = ImageSource::naoProviderV6;
  Pixel* image; /**< The image. Please note that the second half of each row must be ignored. */

  /**
   * @param initialize Whether to initialize the image in gray or not
   */
  Image(bool initialize = true, int width = maxResolutionWidth, int height = maxResolutionHeight);

  /**
   * Copy constructor.
   * @param other The image this is copied from.
   */
  Image(const Image& other);

  Image(Image&& other) noexcept;

  /** destructs an image */
  virtual ~Image();

  /**
   * Assignment operator.
   * @param other The image this is copied from.
   * @return This image.
   */
  Image& operator=(const Image& other);

  Image& operator=(Image&& other) noexcept;

  Pixel* operator[](const int y)
  {
    return image + y * widthStep;
  }
  const Pixel* operator[](const int y) const
  {
    return image + y * widthStep;
  }

  Pixel getFullSizePixel(const int y, const int x) const
  {
    Image::Pixel p = *(image + y * width + x / 2);
    if (!(x & 1))
      p.y = p.yCbCrPadding;
    return p;
  }

  /**
   * The method sets an external image.
   * @param buffer The image buffer.
   */
  void setImage(unsigned char* buffer);
  void setImage(Pixel* image);

  /**
   * @brief Copies over an Image via SSE.
   * @param [in] other The image to be copied.
   * @param [in] halfResolution If every second row should be left out or not.
  */
  void setImageBySSECopy(const Image& other, bool halfResolution = true);

  /** 
   * Converts an YCbCr image into an RGB image.
   * @param ycbcrImage The given YCbCr image
   */
  void convertFromYCbCrToRGB(const Image& ycbcrImage);

  /**
   * Converts an RGB image into an YCbCr image.
   * @param rgbImage The given RGB image
   */
  void convertFromRGBToYCbCr(const Image& rgbImage);

  /**
   * Converts an YCbCr image into a HSI image.
   * @param ycrcbImage The given YCbCr image
   */
  void convertFromYCbCrToHSI(const Image& ycrcbImage);

  /**
   * Converts a HSI image into an YCbCr image.
   * @param hsiImage The given HSI image
   */
  void convertFromHSIToYCbCr(const Image& hsiImage);

  /**
   * Sets the new resolution of the image including the widthStep.
   */
  void setResolution(int newWidth, int newHeight);

  /**
   * Calculates the distance between the first three bytes of two colors
   * @param a The first color
   * @param b The second color
   * @return The distance
   */
  static float getColorDistance(const Image::Pixel& a, const Image::Pixel& b);

  bool isOutOfImage(int x, int y, int distance) const
  {
    return x >= width - distance || x < distance || y >= height - distance || y < distance;
  }

  bool isOutOfImage(float x, float y, int distance) const
  {
    return x >= width - distance || x < distance || y >= height - distance || y < distance;
  }

  bool isOutOfBounds(int xStart, int xOffset, int yStart, int yOffset) const;

  bool projectIntoImage(int& x, int& y, int sizeX, int sizeY) const;
  bool projectIntoImage(Vector2i& postion, float radius) const;

  /**
   * Returns the upper left, lower right position and size within the camera image
   * of a given patch's center position and size.
   * @param center The patch center position
   * @param size The patch size
   * @return The upper left, lower right position and size in this order
   */
  std::array<Vector2i, 3> projectIntoImage(const Vector2i& center, const Vector2i& size) const;

  std::vector<int> copyAndResizeRGBFloatNoHorizon(int sizeXNew, int sizeYNew, const int horizon_y, float* result) const;
  bool shouldBeProcessed() const;

  template <bool rgb = true, bool checkBounds = true, bool overwrite = checkBounds, typename T>
  void copyAndResizeArea(const Vector2i inputPos, const Vector2i inputSize, const Vector2i outputSize, T* result) const
  {
    static_assert(checkBounds || !overwrite, "Cannot overwrite without checkBounds");

    const std::vector<int> xIndices = getIndices(inputSize.x(), outputSize.x(), inputPos.x());
    const std::vector<int> yIndices = getIndices(inputSize.y(), outputSize.y(), inputPos.y());

    for (const int y : yIndices)
    {
      for (const int x : xIndices)
      {
        if constexpr (checkBounds)
        {
          if (x < 0 || x >= width || y < 0 || y >= height)
          {
            if constexpr (overwrite)
            {
              result[0] = 0;
              if constexpr (rgb)
              {
                // Y=0, U=0, V=0 => R=0, G=135, B=0
                if constexpr (std::is_floating_point_v<T>)
                  result[1] = 135.f / 255.f;
                else
                  result[1] = 135;

                result[2] = 0;
              }
            }
            result += rgb ? 3 : 1;
            continue;
          }
        }

        const auto& p = (*this)[y][x];

        if constexpr (rgb)
          ColorModelConversions::fromYCbCrToRGB(p.y, p.cb, p.cr, result[0], result[1], result[2]);
        else
        {
          result[0] = p.y;
          if constexpr (std::is_floating_point_v<T>)
            result[0] /= 255.f;
        }

        result += rgb ? 3 : 1;
      }
    }
  }

  template <bool rgb = true, bool checkBounds = true> void copyAndResizeArea(const Vector2i inputPos, const Vector2i inputSize, const Vector2i outputSize, Out* out) const
  {
    const std::vector<int> xIndices = getIndices(inputSize.x(), outputSize.x(), inputPos.x());
    const std::vector<int> yIndices = getIndices(inputSize.y(), outputSize.y(), inputPos.y());

    for (const int y : yIndices)
    {
      for (const int x : xIndices)
      {
        if constexpr (checkBounds)
        {
          if (x < 0 || x >= width || y < 0 || y >= height)
          {
            *out << 0;
            if constexpr (rgb)
              *out << 135 << 0;
            continue;
          }
        }

        const auto& p = (*this)[y][x];

        if constexpr (rgb)
        {
          unsigned char r, g, b;
          ColorModelConversions::fromYCbCrToRGB(p.y, p.cb, p.cr, r, g, b);
          *out << r << g << b;
        }
        else
          *out << p.y;
      }
    }
  }


protected:
  static std::vector<int> getIndices(const int inputSize, const int outputSize, const int inputPos = 0);
  void serialize(In* in, Out* out);
};

//New class by Naodevils
STREAMABLE_WITH_BASE(ImageUpper, Image,
  using Image::Image;
);
