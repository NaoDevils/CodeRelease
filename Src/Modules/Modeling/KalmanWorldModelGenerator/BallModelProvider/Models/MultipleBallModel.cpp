/**
 * \file MultipleBallModel.cpp
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 *
 * Implementation of classes \c LocalMultipleBallModel and \c RemoteMultipleBallModel.
 */

#include "MultipleBallModel.h"
#include "Tools/Debugging/DebugDrawings.h"


//MARK: BallState

void updateEstimatedBallState(const KalmanPositionHypothesis& hypothesis, BallState& ballState)
{
  // Fill given BallState object with position and velocity of the kalman filter state.
  ballState.position = hypothesis.kalman.position(); // (in mm)
  ballState.velocity = hypothesis.kalman.velocity(); // (in mm/s)
}


//MARK: LocalMultipleBallModel

void LocalMultipleBallModel::draw() const
{
  // Draw local ball hypotheeses.
  ColorRGBA color;
  for (const auto& hypothesis : m_hypotheses)
  {
    if (hypothesis.mergeFactor > 1.f)
      color = ColorRGBA::cyan;
    else
      color = ColorRGBA::blue;
    // Calculate opacity depending on validity.
    unsigned char alpha = static_cast<unsigned char>(55.f + 200.f * hypothesis.validity);
    color.a = alpha;
    DEBUG_DRAWING("module:BallModelProvider:local:hypotheses", "drawingOnField")
    {
      DEBUG_RESPONSE_NOT("debug drawing:module:BallModelProvider:local:covariances")
      {
        CIRCLE("module:BallModelProvider:local:hypotheses", hypothesis.kalman.positionX(), hypothesis.kalman.positionY(), 45, 20, Drawings::solidPen, color, Drawings::solidBrush, color);
      }
      DRAWTEXT("module:BallModelProvider:local:hypotheses", hypothesis.kalman.positionX() + 50, hypothesis.kalman.positionY() + 50, 50, color, static_cast<int>(hypothesis.validity * 100.f + 0.5f) << "%");
    }

    DEBUG_DRAWING("module:BallModelProvider:local:velocities", "drawingOnField")
    {
      ARROW("module:BallModelProvider:local:velocities",
          hypothesis.kalman.positionX(),
          hypothesis.kalman.positionY(),
          hypothesis.kalman.positionX() + hypothesis.kalman.velocityX(),
          hypothesis.kalman.positionY() + hypothesis.kalman.velocityY(),
          5,
          5,
          color);
      DRAWTEXT("module:BallModelProvider:local:velocities",
          hypothesis.kalman.positionX() + hypothesis.kalman.velocityX() + 70,
          hypothesis.kalman.positionY() + hypothesis.kalman.velocityY() - 100,
          50,
          color,
          static_cast<int>(hypothesis.kalman.velocity().norm() + 0.5f) << "mm/s");
    }

    DEBUG_DRAWING("module:BallModelProvider:local:covariances", "drawingOnField")
    {
      COVARIANCE2D("module:BallModelProvider:local:covariances", hypothesis.kalman.covarianceMatrix.cast<float>(), hypothesis.kalman.position());
    }
  }
}


//MARK: RemoteMultipleBallModel

void RemoteMultipleBallModel::draw()
{
  // Draw remote ball hypotheeses.

  DEBUG_DRAWING("module:BallModelProvider:remote:hypotheses", "drawingOnField")
  {
    ColorRGBA color = ColorRGBA::black;

    for (const auto& hypothesis : m_hypotheses)
    {
      if (hypothesis.validity == (bestHypothesis() == nullptr ? 0.f : bestHypothesis()->validity))
        color = ColorRGBA::black; // best hypothesis
      else
        color = ColorRGBA::gray; // all other hypotheses

      CIRCLE("module:BallModelProvider:remote:hypotheses", hypothesis.kalman.positionX(), hypothesis.kalman.positionY(), 45, 20, Drawings::solidPen, color, Drawings::solidBrush, ColorRGBA::gray);
      ARROW("module:BallModelProvider:remote:hypotheses",
          hypothesis.kalman.positionX(),
          hypothesis.kalman.positionY(),
          hypothesis.kalman.positionX() + hypothesis.kalman.velocityX(),
          hypothesis.kalman.positionY() + hypothesis.kalman.velocityY(),
          5,
          5,
          color);
      DRAWTEXT("module:BallModelProvider:remote:hypotheses", hypothesis.kalman.positionX() + 50, hypothesis.kalman.positionY() + 50, 50, color, hypothesis.validity);
      DRAWTEXT("module:BallModelProvider:remote:hypotheses", hypothesis.kalman.positionX() + 70, hypothesis.kalman.positionY() - 100, 50, color, "p " << hypothesis.teammatesString());
    }
  }
}
