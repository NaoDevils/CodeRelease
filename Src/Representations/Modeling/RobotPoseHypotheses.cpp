/**
* @file RobotPoseHypotheses.cpp
*
* contains the implementation of drawing stuff
* for the class RobotPoseHypotheses
*
* @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
*/

#include "RobotPoseHypotheses.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/GaussianDistribution2D.h"
//#include "Platform/GTAssert.h"


void RobotPoseHypotheses::draw()
{
  DECLARE_DEBUG_DRAWING("representation:RobotPoseHypotheses", "drawingOnField"); 
  COMPLEX_DRAWING("representation:RobotPoseHypotheses")
  {
    for(unsigned int i=0; i < hypotheses.size(); ++i)
    {
      // Get eigenvectors:
      GaussianDistribution2D gaussDist;
      gaussDist.covariance = hypotheses[i].getPositionCovariance();
      Vector2d eigenvector1;
      Vector2d eigenvector2;
      double eigenvalue1;
      double eigenvalue2;
      gaussDist.getEigenVectorsAndEigenValues(eigenvector1, eigenvector2, eigenvalue1, eigenvalue2);
      eigenvector1 *= sqrt(eigenvalue1);
      eigenvector2 *= sqrt(eigenvalue2);

      Vector2d pos(hypotheses[i].translation.cast<double>());

      // Check that covariance matrix can be drawn
      if (eigenvector1.x() == eigenvector1.x() && eigenvector1.y() == eigenvector1.y()
        && eigenvector2.x() == eigenvector2.x() && eigenvector2.y() == eigenvector2.y())
      {
        // Draw covariance ellipse:
        ASSERT((eigenvector1.y() != 0.0) || (eigenvector1.x() != 0.0));
        ELLIPSE("representation:RobotPoseHypotheses", pos, sqrt(eigenvalue1), sqrt(eigenvalue2),
          atan2(eigenvector1.y(), eigenvector1.x()), 10, Drawings::solidPen, ColorRGBA(100, 100, 255, 170),
          Drawings::solidBrush, ColorRGBA(100, 100, 255, 170));
        LINE("representation:RobotPoseHypotheses",
          pos.x(), pos.y(), pos.x() + eigenvector1.x(), pos.y() + eigenvector1.y(), 10,
          Drawings::solidPen, ColorRGBA(0, 0, 0));
        LINE("representation:RobotPoseHypotheses",
          pos.x(), pos.y(), pos.x() - eigenvector1.x(), pos.y() - eigenvector1.y(), 10,
          Drawings::solidPen, ColorRGBA(0, 0, 0));
        LINE("representation:RobotPoseHypotheses",
          pos.x(), pos.y(), pos.x() + eigenvector2.x(), pos.y() + eigenvector2.y(), 10,
          Drawings::solidPen, ColorRGBA(0, 0, 0));
        LINE("representation:RobotPoseHypotheses",
          pos.x(), pos.y(), pos.x() - eigenvector2.x(), pos.y() - eigenvector2.y(), 10,
          Drawings::solidPen, ColorRGBA(0, 0, 0));
      }
        
      // Draw hypothesis:
      CIRCLE("representation:RobotPoseHypotheses", 
        pos.x(), pos.y(), 
        40, 
        15, // pen width 
        Drawings::solidPen, 
        ColorRGBA(0,0,0),
        Drawings::solidBrush, 
        ColorRGBA(255,0,0));
      Vector2d dir(100.0,0.0);
      dir.rotate(hypotheses[i].rotation);
      LINE("representation:RobotPoseHypotheses",
        pos.x(), pos.y(), pos.x() + dir.x(), pos.y() + dir.y(), 15, 
        Drawings::solidPen, ColorRGBA(0,0,0));

      // Draw info on hypothesis
      DRAWTEXT("representation:RobotPoseHypotheses", pos.x(), pos.y() - 120, 80, ColorRGBA(255, 255, 255), "pc: " << hypotheses[i].validity);
      DRAWTEXT("representation:RobotPoseHypotheses", pos.x(), pos.y() - 200, 80, ColorRGBA(255, 255, 255), "sc: " << hypotheses[i].symmetry);
    }
  }; // end of complex drawing
}
/*
const RobotPoseHypothesis& RobotPoseHypotheses::getBestHypothesis() const
{
  ASSERT((int)hypotheses.size()>indexOfBestHypothesis);
  return hypotheses[indexOfBestHypothesis];
} */