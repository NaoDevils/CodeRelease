/**
 * @file BallPercept.cpp
 *
 * Very simple representation of a seen ball
 *
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#include "BallPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void BallPercept::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:BallPercept:Image:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BallPercept:Image:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BallPercept:ballPatch", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BallPercept:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:BallPercept", "robot");
  TRANSLATE3D("representation:BallPercept", 0, 0, -230);

  if (status == seen)
  {
    ColorRGBA brushColor;
    ColorRGBA penColor;
    unsigned char alpha = 100;

    switch (detectionSource)
    {
    case BallPatch::DetectionSource::scanlines:
      brushColor = ColorRGBA::red;
      break;
    case BallPatch::DetectionSource::yoloHypothesis:
      brushColor = ColorRGBA::blue;
      break;
    case BallPatch::DetectionSource::ballModel:
      brushColor = ColorRGBA::violet;
      break;
    }
    brushColor.a = alpha;

    switch (detectionVerifier)
    {
    case BallPatch::DetectionVerifier::scanlinesAndCNN:
      penColor = ColorRGBA::red;
      break;
    case BallPatch::DetectionVerifier::ballPositionCNN:
      penColor = ColorRGBA::green;
      break;
    case BallPatch::DetectionVerifier::yolo:
      penColor = ColorRGBA::blue;
      break;
    }

    int textsize = static_cast<int>(radiusInImage / 2.f);
    if (fromUpper)
    {
      CIRCLE("representation:BallPercept:Image:Upper", positionInImage.x(), positionInImage.y(), radiusInImage, 3, Drawings::solidPen, penColor, Drawings::solidBrush, brushColor);
      DRAWTEXT("representation:BallPercept:Image:Upper", positionInImage.x() - textsize / 2, positionInImage.y() + textsize / 2, textsize, ColorRGBA::white, roundf(validity * 100));
    }
    else
    {
      CIRCLE("representation:BallPercept:Image:Lower", positionInImage.x(), positionInImage.y(), radiusInImage, 3, Drawings::solidPen, penColor, Drawings::solidBrush, brushColor);
      DRAWTEXT("representation:BallPercept:Image:Lower", positionInImage.x() - textsize / 2, positionInImage.y() + textsize / 2, textsize, ColorRGBA::white, roundf(validity * 100));
    }

    CIRCLE("representation:BallPercept:Field",
        relativePositionOnField.x(),
        relativePositionOnField.y(),
        radiusOnField / 2,
        0, // pen width
        Drawings::solidPen,
        ColorRGBA::white,
        Drawings::solidBrush,
        ColorRGBA::white);
    CIRCLE("representation:BallPercept:Field",
        relativePositionOnField.x(),
        relativePositionOnField.y(),
        radiusOnField / 4,
        0, // pen width
        Drawings::solidPen,
        ColorRGBA::black,
        Drawings::solidBrush,
        ColorRGBA::black);
    SPHERE3D("representation:BallPercept", relativePositionOnField.x(), relativePositionOnField.y(), radiusOnField, radiusOnField, ColorRGBA::orange);
  }

  COMPLEX_IMAGE(BallPerceptPatch)
  {
    ColorRGBA color;
    INIT_DEBUG_IMAGE_BLACK(BallPerceptPatch, CNN_POSITION_SIZE, CNN_POSITION_SIZE);
    if (!ballPatch.patch.empty() && validity > 0.f)
    {
      ASSERT(ballPatch.patch.size() == (CNN_POSITION_SIZE * CNN_POSITION_SIZE * 3));
      for (int y = 0; y < CNN_POSITION_SIZE; y++)
      {
        for (int x = 0; x < CNN_POSITION_SIZE; x++)
        {
          float r = ballPatch.patch[y * CNN_POSITION_SIZE * 3 + x * 3 + 0] * 255.f;
          float g = ballPatch.patch[y * CNN_POSITION_SIZE * 3 + x * 3 + 1] * 255.f;
          float b = ballPatch.patch[y * CNN_POSITION_SIZE * 3 + x * 3 + 2] * 255.f;
          if (ballPatch.rgb)
          {
            DEBUG_IMAGE_SET_PIXEL_RGB(BallPerceptPatch, x, y, static_cast<unsigned char>(r), static_cast<unsigned char>(g), static_cast<unsigned char>(b));
          }
          else
          {
            DEBUG_IMAGE_SET_PIXEL_YUV(BallPerceptPatch, x, y, static_cast<unsigned char>(r), static_cast<unsigned char>(g), static_cast<unsigned char>(b));
          }
        }
      }

      if (detectionVerifier == BallPatch::DetectionVerifier::ballPositionCNN)
      {
        color = ColorRGBA::green;
      }
      else
      {
        color = ColorRGBA::red;
      }

      CIRCLE("representation:BallPercept:ballPatch",
          static_cast<int>(ballPatch.centerInPatch.x() + 0.5f),
          static_cast<int>(ballPatch.centerInPatch.y() + 0.5f),
          static_cast<int>(ballPatch.radiusInPatch + 0.5f),
          1,
          Drawings::solidPen,
          color,
          Drawings::noBrush,
          color);

      DRAWTEXT("representation:BallPercept:ballPatch", static_cast<int>(ballPatch.centerInPatch.x() - 6 + 0.5f), static_cast<int>(ballPatch.centerInPatch.y() + 1 + 0.5f), 2, color, validity * 100.f);
    }
    SEND_DEBUG_IMAGE(BallPerceptPatch);
  }
}

void MultipleBallPercept::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:MultipleBallPercept:Image:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:MultipleBallPercept:Image:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:MultipleBallPercept:multipleBallPatch", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:MultipleBallPercept:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:MultipleBallPercept", "robot");
  TRANSLATE3D("representation:MultipleBallPercept", 0, 0, -230);

  for (size_t i = 0; i < balls.size(); i++)
  {
    if (balls[i].status == BallPercept::seen)
    {
      ColorRGBA brushColor;
      ColorRGBA penColor;
      unsigned char alpha = 100;

      switch (balls[i].detectionSource)
      {
      case BallPatch::DetectionSource::scanlines:
        brushColor = ColorRGBA::red;
        break;
      case BallPatch::DetectionSource::yoloHypothesis:
        brushColor = ColorRGBA::blue;
        break;
      case BallPatch::DetectionSource::ballModel:
        brushColor = ColorRGBA::violet;
        break;
      }
      brushColor.a = alpha;

      switch (balls[i].detectionVerifier)
      {
      case BallPatch::DetectionVerifier::scanlinesAndCNN:
        penColor = ColorRGBA::red;
        break;
      case BallPatch::DetectionVerifier::ballPositionCNN:
        penColor = ColorRGBA::green;
        break;
      case BallPatch::DetectionVerifier::yolo:
        penColor = ColorRGBA::blue;
        break;
      }

      int textsize = static_cast<int>(balls[i].radiusInImage / 2.f);
      if (balls[i].fromUpper)
      {
        CIRCLE("representation:MultipleBallPercept:Image:Upper", balls[i].positionInImage.x(), balls[i].positionInImage.y(), balls[i].radiusInImage, 3, Drawings::solidPen, penColor, Drawings::solidBrush, brushColor);
        DRAWTEXT("representation:MultipleBallPercept:Image:Upper",
            balls[i].positionInImage.x() - textsize / 2,
            balls[i].positionInImage.y() + textsize / 2,
            textsize,
            ColorRGBA::white,
            roundf(balls[i].validity * 100));
      }
      else
      {
        CIRCLE("representation:MultipleBallPercept:Image:Lower", balls[i].positionInImage.x(), balls[i].positionInImage.y(), balls[i].radiusInImage, 3, Drawings::solidPen, penColor, Drawings::solidBrush, brushColor);
        DRAWTEXT("representation:MultipleBallPercept:Image:Lower",
            balls[i].positionInImage.x() - textsize / 2,
            balls[i].positionInImage.y() + textsize / 2,
            textsize,
            ColorRGBA::white,
            roundf(balls[i].validity * 100));
      }

      CIRCLE("representation:MultipleBallPercept:Field",
          balls[i].relativePositionOnField.x(),
          balls[i].relativePositionOnField.y(),
          balls[i].radiusOnField / 2,
          0, // pen width
          Drawings::solidPen,
          ColorRGBA::white,
          Drawings::solidBrush,
          ColorRGBA::white);
      CIRCLE("representation:MultipleBallPercept:Field",
          balls[i].relativePositionOnField.x(),
          balls[i].relativePositionOnField.y(),
          balls[i].radiusOnField / 4,
          0, // pen width
          Drawings::solidPen,
          ColorRGBA::black,
          Drawings::solidBrush,
          ColorRGBA::black);
      SPHERE3D("representation:MultipleBallPercept", balls[i].relativePositionOnField.x(), balls[i].relativePositionOnField.y(), balls[i].radiusOnField, balls[i].radiusOnField, ColorRGBA::orange);
    }
  }

  COMPLEX_IMAGE(MultipleBallPerceptPatch)
  {
    ColorRGBA color;
    int n = std::max<int>(1, static_cast<int>(std::ceil(std::sqrt(static_cast<float>(balls.size())))));
    INIT_DEBUG_IMAGE_BLACK(MultipleBallPerceptPatch, n * CNN_POSITION_SIZE + n, n * CNN_POSITION_SIZE + n);
    for (int m = 0; m < static_cast<int>(balls.size()); m++)
    {
      int column = m / n;
      int row = m % n;
      if (!balls[m].ballPatch.patch.empty())
      {
        ASSERT(balls[m].ballPatch.patch.size() == (CNN_POSITION_SIZE * CNN_POSITION_SIZE * 3));
        for (int y = 0; y < CNN_POSITION_SIZE; y++)
        {
          for (int x = 0; x < CNN_POSITION_SIZE; x++)
          {
            float r = balls[m].ballPatch.patch[y * CNN_POSITION_SIZE * 3 + x * 3 + 0] * 255.f;
            float g = balls[m].ballPatch.patch[y * CNN_POSITION_SIZE * 3 + x * 3 + 1] * 255.f;
            float b = balls[m].ballPatch.patch[y * CNN_POSITION_SIZE * 3 + x * 3 + 2] * 255.f;
            if (balls[m].ballPatch.rgb)
            {
              DEBUG_IMAGE_SET_PIXEL_RGB(MultipleBallPerceptPatch,
                  x + row * CNN_POSITION_SIZE + row,
                  y + column * CNN_POSITION_SIZE + column,
                  static_cast<unsigned char>(r),
                  static_cast<unsigned char>(g),
                  static_cast<unsigned char>(b));
            }
            else
            {
              DEBUG_IMAGE_SET_PIXEL_YUV(MultipleBallPerceptPatch,
                  x + row * CNN_POSITION_SIZE + row,
                  y + column * CNN_POSITION_SIZE + column,
                  static_cast<unsigned char>(r),
                  static_cast<unsigned char>(g),
                  static_cast<unsigned char>(b));
            }
          }
        }

        if (balls[m].detectionVerifier == BallPatch::DetectionVerifier::ballPositionCNN)
        {
          color = ColorRGBA::green;
        }
        else
        {
          color = ColorRGBA::red;
        }


        CIRCLE("representation:MultipleBallPercept:multipleBallPatch",
            static_cast<int>(balls[m].ballPatch.centerInPatch.x() + 0.5f) + row * CNN_POSITION_SIZE + row,
            static_cast<int>(balls[m].ballPatch.centerInPatch.y() + 0.5f) + column * CNN_POSITION_SIZE + column,
            static_cast<int>(balls[m].ballPatch.radiusInPatch + 0.5f),
            1,
            Drawings::solidPen,
            color,
            Drawings::noBrush,
            color);

        DRAWTEXT("representation:MultipleBallPercept:multipleBallPatch",
            static_cast<int>(balls[m].ballPatch.centerInPatch.x() - 6 + 0.5f) + row * CNN_POSITION_SIZE + row,
            static_cast<int>(balls[m].ballPatch.centerInPatch.y() + 1 + 0.5f) + column * CNN_POSITION_SIZE + column,
            2,
            color,
            balls[m].validity * 100.f);
      }
    }
    SEND_DEBUG_IMAGE(MultipleBallPerceptPatch);
  }
}

void ProcessedBallPatches::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:BallPatches", "drawingOnImage");

  COMPLEX_IMAGE(BallPatches)
  {
    ColorRGBA color;
    int offset = 0;
    int n = std::max<int>(1, static_cast<int>(std::ceil(std::sqrt(static_cast<float>(patches.size())))));
    INIT_DEBUG_IMAGE_BLACK(BallPatches, n * CNN_POSITION_SIZE + n, n * CNN_POSITION_SIZE + n);
    for (int m = 0; m < static_cast<int>(patches.size()); m++)
    {
      int column = m / n;
      int row = m % n;
      if (!patches[m].patch.empty())
      {
        if (patches[m].verifier == BallPatch::DetectionVerifier::ballPositionCNN)
        {
          color = ColorRGBA::green;
          offset = 0;
          ASSERT(patches[m].patch.size() == (CNN_POSITION_SIZE * CNN_POSITION_SIZE * 3));
          for (int y = 0; y < CNN_POSITION_SIZE; y++)
          {
            for (int x = 0; x < CNN_POSITION_SIZE; x++)
            {
              float r = patches[m].patch[y * CNN_POSITION_SIZE * 3 + x * 3 + 0] * 255.f;
              float g = patches[m].patch[y * CNN_POSITION_SIZE * 3 + x * 3 + 1] * 255.f;
              float b = patches[m].patch[y * CNN_POSITION_SIZE * 3 + x * 3 + 2] * 255.f;
              if (patches[m].rgb)
              {
                DEBUG_IMAGE_SET_PIXEL_RGB(
                    BallPatches, x + row * CNN_POSITION_SIZE + row, y + column * CNN_POSITION_SIZE + column, static_cast<unsigned char>(r), static_cast<unsigned char>(g), static_cast<unsigned char>(b));
              }
              else
              {
                DEBUG_IMAGE_SET_PIXEL_YUV(
                    BallPatches, x + row * CNN_POSITION_SIZE + row, y + column * CNN_POSITION_SIZE + column, static_cast<unsigned char>(r), static_cast<unsigned char>(g), static_cast<unsigned char>(b));
              }
            }
          }
        }
        else
        {
          color = ColorRGBA::red;
          offset = static_cast<int>((CNN_POSITION_SIZE - CNN_SCANLINES_SIZE) / 2.f);
          ASSERT(patches[m].patch.size() == (CNN_SCANLINES_SIZE * CNN_SCANLINES_SIZE));
          for (int y = 0; y < CNN_SCANLINES_SIZE; y++)
          {
            for (int x = 0; x < CNN_SCANLINES_SIZE; x++)
            {
              float intensity = patches[m].patch[y * CNN_SCANLINES_SIZE + x] * 255.f;
              DEBUG_IMAGE_SET_PIXEL_YUV(
                  BallPatches, x + offset + row * CNN_POSITION_SIZE + row, y + offset + column * CNN_POSITION_SIZE + column, static_cast<unsigned char>(intensity), 127, 127);
            }
          }
        }
        CIRCLE("representation:BallPatches",
            static_cast<int>(patches[m].centerInPatch.x() + offset + 0.5f) + row * CNN_POSITION_SIZE + row,
            static_cast<int>(patches[m].centerInPatch.y() + offset + 0.5f) + column * CNN_POSITION_SIZE + column,
            static_cast<int>(patches[m].radiusInPatch + 0.5f),
            1,
            Drawings::solidPen,
            color,
            Drawings::noBrush,
            color);

        DRAWTEXT("representation:BallPatches",
            static_cast<int>(patches[m].centerInPatch.x() + offset - 6 + 0.5f) + row * CNN_POSITION_SIZE + row,
            static_cast<int>(patches[m].centerInPatch.y() + offset + 1 + 0.5f) + column * CNN_POSITION_SIZE + column,
            2,
            color,
            patches[m].validity * 100.f);
      }
    }
    SEND_DEBUG_IMAGE(BallPatches);
  }
}
