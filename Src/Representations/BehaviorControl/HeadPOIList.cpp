/**
* @file HeadPOIList.h
* 
* Declaration of class HeadPOIList
* @author <a href="mailto:mahdokht.mohammadi@tu-dortmund.de">Mahdokht Mohammadi</a>
* @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
*/

#include "HeadPOIList.h"

void HeadPOIList::addAngle(Vector2a angle)
{
  targets.emplace_back();
  targets.back().type = HeadPOIList::Target::Type::angle;
  targets.back().position = angle.cast<float>();
}

void HeadPOIList::addBall(Vector2a offset)
{
  targets.emplace_back();
  targets.back().type = HeadPOIList::Target::Type::ball;
  targets.back().position = offset.cast<float>();
}

void HeadPOIList::addRemoteBall(Vector2a offset)
{
  targets.emplace_back();
  targets.back().type = HeadPOIList::Target::Type::remoteBall;
  targets.back().position = offset.cast<float>();
}

void HeadPOIList::addFieldPosition(Vector2f position)
{
  targets.emplace_back();
  targets.back().type = HeadPOIList::Target::Type::fieldPosition;
  targets.back().position = position;
}
