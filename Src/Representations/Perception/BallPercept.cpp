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
#include "Tools/Debugging/DebugImages.h"

BallPatch::BallPatch(const CheckedBallSpot& ballSpot, const Image& image, const Vector2i& inputPosition, const Vector2i& inputSize, const Vector2i& outputSize, bool rgb)
{
  setPatch(image, inputPosition, inputSize, outputSize, rgb);
  fromBallSpot(ballSpot);
}

void BallPatch::fromBallSpot(const CheckedBallSpot& ballSpot)
{
  ASSERT(!inputSize.isZero());
  ASSERT(!outputSize.isZero());
  this->resizeFactor = static_cast<float>(inputSize.maxCoeff()) / outputSize.maxCoeff();
  this->centerInPatch = (ballSpot.position - inputPosition).cast<float>() / this->resizeFactor;
  this->radiusInPatch = ballSpot.radiusInImage / this->resizeFactor;
  this->validity = ballSpot.validity;
  this->fromUpper = ballSpot.upper;
  this->source = ballSpot.source;
  this->verifier = ballSpot.verifier;
}

BallPercept::BallPercept(const CheckedBallSpot& ballSpot, unsigned timestamp, const Vector2f& posOnField)
    : status(BallPercept::Status::seen), timestamp(timestamp), positionInImage(ballSpot.position.cast<float>()), radiusInImage(ballSpot.radiusInImage),
      relativePositionOnField(posOnField), validity(ballSpot.validity), fromUpper(ballSpot.upper), detectionSource(ballSpot.source), detectionVerifier(ballSpot.verifier)
{
}

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
    case CheckedBallSpot::DetectionSource::scanlines:
      brushColor = ColorRGBA::red;
      break;
    case CheckedBallSpot::DetectionSource::yoloHypothesis:
      brushColor = ColorRGBA::blue;
      break;
    case CheckedBallSpot::DetectionSource::ballModel:
      brushColor = ColorRGBA::violet;
      break;
    }
    brushColor.a = alpha;

    switch (detectionVerifier)
    {
    case CheckedBallSpot::DetectionVerifier::scanlinesAndCNN:
      penColor = ColorRGBA::red;
      break;
    case CheckedBallSpot::DetectionVerifier::ballPositionCNN:
      penColor = ColorRGBA::green;
      break;
    case CheckedBallSpot::DetectionVerifier::yolo:
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
    DECLARE_LOCAL_DEBUG_IMAGE(BallPerceptPatch);

    ColorRGBA color;
    INIT_DEBUG_IMAGE_BLACK(BallPerceptPatch, CNN_POSITION_SIZE, CNN_POSITION_SIZE);
    if (!ballPatch.getPatch().empty() && validity > 0.f)
    {
      ASSERT(ballPatch.getPatch().size() == (CNN_POSITION_SIZE * CNN_POSITION_SIZE * 3));
      for (int y = 0; y < CNN_POSITION_SIZE; y++)
      {
        for (int x = 0; x < CNN_POSITION_SIZE; x++)
        {
          const unsigned char r = ballPatch.getPatch()[y * CNN_POSITION_SIZE * 3 + x * 3 + 0];
          const unsigned char g = ballPatch.getPatch()[y * CNN_POSITION_SIZE * 3 + x * 3 + 1];
          const unsigned char b = ballPatch.getPatch()[y * CNN_POSITION_SIZE * 3 + x * 3 + 2];
          if (ballPatch.rgb)
          {
            DEBUG_IMAGE_SET_PIXEL_RGB(BallPerceptPatch, x, y, r, g, b);
          }
          else
          {
            DEBUG_IMAGE_SET_PIXEL_YUV(BallPerceptPatch, x, y, r, g, b);
          }
        }
      }

      if (detectionVerifier == CheckedBallSpot::DetectionVerifier::ballPositionCNN)
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
      case CheckedBallSpot::DetectionSource::scanlines:
        brushColor = ColorRGBA::red;
        break;
      case CheckedBallSpot::DetectionSource::yoloHypothesis:
        brushColor = ColorRGBA::blue;
        break;
      case CheckedBallSpot::DetectionSource::ballModel:
        brushColor = ColorRGBA::violet;
        break;
      }
      brushColor.a = alpha;

      switch (balls[i].detectionVerifier)
      {
      case CheckedBallSpot::DetectionVerifier::scanlinesAndCNN:
        penColor = ColorRGBA::red;
        break;
      case CheckedBallSpot::DetectionVerifier::ballPositionCNN:
        penColor = ColorRGBA::green;
        break;
      case CheckedBallSpot::DetectionVerifier::yolo:
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
    DECLARE_LOCAL_DEBUG_IMAGE(MultipleBallPerceptPatch);

    ColorRGBA color;
    int n = std::max<int>(1, static_cast<int>(std::ceil(std::sqrt(static_cast<float>(balls.size())))));
    INIT_DEBUG_IMAGE_BLACK(MultipleBallPerceptPatch, n * CNN_POSITION_SIZE + n, n * CNN_POSITION_SIZE + n);
    for (int m = 0; m < static_cast<int>(balls.size()); m++)
    {
      int column = m / n;
      int row = m % n;
      if (!balls[m].ballPatch.getPatch().empty())
      {
        ASSERT(balls[m].ballPatch.getPatch().size() == (CNN_POSITION_SIZE * CNN_POSITION_SIZE * 3));
        for (int y = 0; y < CNN_POSITION_SIZE; y++)
        {
          for (int x = 0; x < CNN_POSITION_SIZE; x++)
          {
            const unsigned char r = balls[m].ballPatch.getPatch()[y * CNN_POSITION_SIZE * 3 + x * 3 + 0];
            const unsigned char g = balls[m].ballPatch.getPatch()[y * CNN_POSITION_SIZE * 3 + x * 3 + 1];
            const unsigned char b = balls[m].ballPatch.getPatch()[y * CNN_POSITION_SIZE * 3 + x * 3 + 2];
            if (balls[m].ballPatch.rgb)
            {
              DEBUG_IMAGE_SET_PIXEL_RGB(MultipleBallPerceptPatch, x + row * CNN_POSITION_SIZE + row, y + column * CNN_POSITION_SIZE + column, r, g, b);
            }
            else
            {
              DEBUG_IMAGE_SET_PIXEL_YUV(MultipleBallPerceptPatch, x + row * CNN_POSITION_SIZE + row, y + column * CNN_POSITION_SIZE + column, r, g, b);
            }
          }
        }

        if (balls[m].detectionVerifier == CheckedBallSpot::DetectionVerifier::ballPositionCNN)
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
    DECLARE_LOCAL_DEBUG_IMAGE(BallPatches);

    int offset = 0;
    int n = std::max<int>(1, static_cast<int>(std::ceil(std::sqrt(static_cast<float>(patches.size())))));
    INIT_DEBUG_IMAGE_BLACK(BallPatches, n * CNN_POSITION_SIZE + n, n * CNN_POSITION_SIZE + n);
    for (int m = 0; m < static_cast<int>(patches.size()); m++)
    {
      int column = m / n;
      int row = m % n;
      if (!patches[m].getPatch().empty())
      {
        const ColorRGBA color = patches[m].verifier == CheckedBallSpot::DetectionVerifier::ballPositionCNN ? ColorRGBA::green : ColorRGBA::red;
        offset = 0;
        ASSERT(patches[m].getPatch().size() == (CNN_POSITION_SIZE * CNN_POSITION_SIZE * 3));
        for (int y = 0; y < CNN_POSITION_SIZE; y++)
        {
          for (int x = 0; x < CNN_POSITION_SIZE; x++)
          {
            const unsigned char r = patches[m].getPatch()[y * CNN_POSITION_SIZE * 3 + x * 3 + 0];
            const unsigned char g = patches[m].getPatch()[y * CNN_POSITION_SIZE * 3 + x * 3 + 1];
            const unsigned char b = patches[m].getPatch()[y * CNN_POSITION_SIZE * 3 + x * 3 + 2];
            if (patches[m].rgb)
            {
              DEBUG_IMAGE_SET_PIXEL_RGB(BallPatches, x + row * CNN_POSITION_SIZE + row, y + column * CNN_POSITION_SIZE + column, r, g, b);
            }
            else
            {
              DEBUG_IMAGE_SET_PIXEL_YUV(BallPatches, x + row * CNN_POSITION_SIZE + row, y + column * CNN_POSITION_SIZE + column, r, g, b);
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
