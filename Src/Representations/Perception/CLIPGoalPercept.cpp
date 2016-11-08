/**
* @file CLIPGoalPercept.cpp
*
* Very simple representation of a seen goal
*
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#include "CLIPGoalPercept.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Debugging/DebugDrawings.h"

void CLIPGoalPercept::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:CLIPGoalPercept:Image:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:CLIPGoalPercept:Image:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:CLIPGoalPercept:Field", "drawingOnField");

  for (int i=0; i<numberOfGoalPosts; i++)
  {
    if (goalPosts[i].fromUpper)
      QUADRANGLE("representation:CLIPGoalPercept:Image:Upper",
        (goalPosts[i].topInImage.x() - goalPosts[i].topWidth / 2),
        (goalPosts[i].topInImage.y()),
        (goalPosts[i].topInImage.x() + goalPosts[i].topWidth / 2),
        (goalPosts[i].topInImage.y()),
        (goalPosts[i].bottomInImage.x() + goalPosts[i].bottomWidth / 2),
        (goalPosts[i].bottomInImage.y()),
        (goalPosts[i].bottomInImage.x() - goalPosts[i].bottomWidth / 2),
        (goalPosts[i].bottomInImage.y()),
        2, Drawings::solidPen, ColorRGBA::yellow);
    else
      QUADRANGLE("representation:CLIPGoalPercept:Image:Lower",
        (goalPosts[i].topInImage.x()-goalPosts[i].topWidth/2),
        (goalPosts[i].topInImage.y()),
        (goalPosts[i].topInImage.x()+goalPosts[i].topWidth/2),
        (goalPosts[i].topInImage.y()),
        (goalPosts[i].bottomInImage.x()+goalPosts[i].bottomWidth/2),
        (goalPosts[i].bottomInImage.y()),
        (goalPosts[i].bottomInImage.x()-goalPosts[i].bottomWidth/2),
        (goalPosts[i].bottomInImage.y()),
        2, Drawings::solidPen, ColorRGBA::yellow);
    Vector2i center = ( goalPosts[i].topInImage
                          + goalPosts[i].bottomInImage ) / 2;

    if (goalPosts[i].fromUpper)
      CIRCLE("representation:CLIPGoalPercept:Image:Upper", center.x(), center.y(),
        5, 2, Drawings::solidPen, ColorRGBA::gray, Drawings::noBrush, ColorRGBA::gray);
    else
      CIRCLE("representation:CLIPGoalPercept:Image:Lower",center.x(),center.y(),
        5, 2, Drawings::solidPen, ColorRGBA::gray, Drawings::noBrush, ColorRGBA::gray);
    int factor = -1;
    if (goalPosts[i].goalPostSide == GoalPost::leftPost)
    {
      factor = 1;
    }
    if (goalPosts[i].goalPostSide != GoalPost::unknownPost)
    {
      if (goalPosts[i].fromUpper)
        ARROW("representation:CLIPGoalPercept:Image:Upper", center.x(), center.y(), center.x() + factor * 30, center.y(),
          2, Drawings::solidPen, ColorRGBA::white);
      else
        ARROW("representation:CLIPGoalPercept:Image:Lower",center.x(),center.y(),center.x()+factor*30,center.y(),
          2,Drawings::solidPen, ColorRGBA::white);
    }


    LINE("representation:CLIPGoalPercept:Field",0,0,goalPosts[i].locationOnField.x(), goalPosts[i].locationOnField.y(),
      10, (goalPosts[i].bottomFound ? Drawings::dottedPen : Drawings::solidPen), ColorRGBA::yellow);
    CIRCLE("representation:CLIPGoalPercept:Field", goalPosts[i].locationOnField.x(), goalPosts[i].locationOnField.y(),
      60, 10, Drawings::solidPen, ColorRGBA::white, Drawings::solidBrush, ColorRGBA::yellow);
    
    if (goalPosts[i].goalPostSide != GoalPost::unknownPost)
    {      
      Vector2f dir(static_cast<float>(goalPosts[i].locationOnField.y()), static_cast<float>(-goalPosts[i].locationOnField.x()));
      ColorRGBA arrowColor = ColorRGBA::gray;
      ARROW("representation:CLIPGoalPercept:Field",
            goalPosts[i].locationOnField.x(),
            goalPosts[i].locationOnField.y(),
            goalPosts[i].locationOnField.x() + dir.x(),
            goalPosts[i].locationOnField.y() + dir.y(),
            20,Drawings::solidPen, arrowColor);
    }
  }

}
