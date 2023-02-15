/**
 * @file TeammateData.cpp
 *
 * Representation of information received from my teammates
 *
 * @author Alexis Tsogias
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include <utility>
#include "TeammateData.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"

void TeammateData::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:TeammateData", "drawingOnField");
  for (auto const& teammate : teammates)
  {
    ColorRGBA posCol;
    if (teammate.status == Teammate::FULLY_ACTIVE)
      posCol = ColorRGBA::green;
    else if (teammate.status == Teammate::ACTIVE)
      posCol = ColorRGBA::yellow;
    else
      posCol = ColorRGBA::red;

    const Vector2f& rPos = teammate.pose.translation;
    const float radius = 400.f; // std::max(50.f, teammate.pose.deviation);
    Vector2f dirPos = teammate.pose * Vector2f(radius, 0.f);

    // Circle around Player
    CIRCLE("representation:TeammateData", rPos.x(), rPos.y(), radius, 20, Drawings::solidPen, posCol, Drawings::noBrush, ColorRGBA::white);
    // Direction of the Robot
    LINE("representation:TeammateData", rPos.x(), rPos.y(), dirPos.x(), dirPos.y(), 20, Drawings::solidPen, posCol);
    // Player number
    DRAWTEXT("representation:TeammateData", rPos.x() + 100, rPos.y(), 100, ColorRGBA::black, teammate.number);
    // Role
    //DRAWTEXT("representation:TeammateData", rPos.x() + 100, rPos.y() - 150, 100,
    //         ColorRGBA::black, BehaviorData::RoleAssignment::getName(teammate.behaviorData.role));
    // Intention
    std::string intentionStr;
    switch (teammate.behaviorData.soccerState)
    {
    case BehaviorData::waiting:
      intentionStr = "default";
      break;
    case BehaviorData::positioning:
      if (teammate.behaviorData.role == BehaviorData::keeper)
        intentionStr = "keeper";
      else
        intentionStr = "defensive";
      break;
    case BehaviorData::controlBall:
      intentionStr = "kick";
      break;
    case BehaviorData::penalized:
    case BehaviorData::searchForBall:
      intentionStr = "lost";
      break;
    default:
      intentionStr = "unknown";
    }
    DRAWTEXT("representation:TeammateData", rPos.x() + 100, rPos.y() - 450, 100, ColorRGBA::black, "Intention: " << intentionStr);

    const Vector2f bPos = teammate.pose * teammate.ball.estimate.position;
    //Line from Robot to WalkTarget
    //LINE("representation:TeammateData", rPos.x(), rPos.y(), teammate.walkRequest.request.translation.x(), teammate.walkRequest.request.translation.y(), 10, Drawings::dashedPen, ColorRGBA::magenta);
    // Ball position
    CIRCLE("representation:TeammateData", bPos.x(), bPos.y(), 50, 20, Drawings::solidPen, ColorRGBA::yellow, Drawings::solidBrush, ColorRGBA::yellow);
  }
}

const Teammate* TeammateData::getNewestTeammate() const
{
  const Teammate* newestTeammate = nullptr;

  if (myself.number >= 0)
    newestTeammate = &myself;

  for (const Teammate& teammate : teammates)
  {
    if (!newestTeammate || teammate.timeWhenSent > newestTeammate->timeWhenSent || (teammate.timeWhenSent == newestTeammate->timeWhenSent && teammate.number < newestTeammate->number))
      newestTeammate = &teammate;
  }

  return newestTeammate;
}

Teammate* TeammateData::getNewestEventMessage(TeamCommEvents::SendReason reason)
{
  return const_cast<Teammate*>(std::as_const(*this).getNewestEventMessage(reason));
}

const Teammate* TeammateData::getNewestEventMessage(TeamCommEvents::SendReason reason) const
{
  for (auto& eventMessage : newestEventMessages)
  {
    if (eventMessage.reason == reason)
      return &eventMessage.message;
  }
  return nullptr;
}
