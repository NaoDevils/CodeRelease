#include "SimpleRobotMapProvider.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Math/Geometry.h"

SimpleRobotMapProvider::SimpleRobotMapProvider()
{
  timeOfLastExeccution = 0;
  simpleRobots.clear();
}

void SimpleRobotMapProvider::update(RobotMap& robotMap)
{
  execute();
  // update robot map
  robotMap = internalRobotMap;
}

void SimpleRobotMapProvider::update(LocalRobotMap& localRobotMap)
{
  // TODO: this is wrong, since remote percepts are not excluded!
  execute();
  // update robot map
  (RobotMap&)localRobotMap = internalRobotMap;
}

void SimpleRobotMapProvider::execute()
{
  if (theRobotInfo.penalty != PENALTY_NONE)
  {
    simpleRobots.clear();
    return;
  }

  if (timeOfLastExeccution == theFrameInfo.time)
    return;
  else
    timeOfLastExeccution = theFrameInfo.time;
  internalRobotMap.robots.clear();

  // validity decreases with every frame
  for (unsigned int sr = 0; sr < simpleRobots.size(); sr++)
  {
    simpleRobots[sr].validity = std::max(0.f, simpleRobots[sr].validity - validityUpdate);
  }

  // predict with velocity
  predict();

  // update with local percepts
  updateWithLocalData(false);
  updateWithLocalData(true);

  // global update
  if (global && theRobotPose.validity > 0.4)
  {
    // update with team mate robot map (TODO: send and update with percepts/simple robots)
    updateWithGlobalData();
  }

  // update with team mate robot poses
  updateWithTeamMatePoses();

  mergeSimpleRobots();

  // only the best survive
  prune();

  // fill internal robot map
  fillRobotMap();
}

void SimpleRobotMapProvider::update(SimpleRobotsDistributed& simpleRobotsDistributed)
{
  execute();
  simpleRobotsDistributed.robots.clear();
  std::vector<SimpleRobot>::const_iterator i = simpleRobots.begin();
  std::vector<SimpleRobot>::const_iterator end = simpleRobots.end();
  for (; i != end; i++)
  {
    if (simpleRobotsDistributed.robots.size() >= maxRobotsToSend)
      break;
    if (i->validity > minValidity && !i->fromTeamMatePoses && i->timeOfLastLocalUpdate == theFrameInfo.time)
    {
      SimpleRobotsDistributed::SimpleRobotDistributed srd;
      srd.timeOfLastUpdate = i->timeOfLastUpdate;
      srd.colorCount = (short)std::min(std::max(-10, i->teamMateCounter), 10);
      srd.validity = (short)(i->validity * 1000);
      srd.velocity = i->velocity.cast<short>();
      srd.x = (short)i->pose.translation.x();
      srd.y = (short)i->pose.translation.y();
      srd.r = (short)i->pose.rotation.toDegrees();
      simpleRobotsDistributed.robots.push_back(srd);
    }
  }
}

void SimpleRobotMapProvider::updateWithLocalData(const bool& upper)
{
  const CameraMatrix& cameraMatrix = (upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix);
  std::vector<RobotEstimate>::const_iterator i = upper ? theRobotsPerceptUpper.robots.begin() : theRobotsPercept.robots.begin();
  std::vector<RobotEstimate>::const_iterator end = upper ? theRobotsPerceptUpper.robots.end() : theRobotsPercept.robots.end();
  for (; i != end; i++)
  {
    Vector2f perceptFieldCoordinates = Transformation::robotToField(theRobotPose, i->locationOnField.translation);
    bool merged = false;
    for (unsigned int sr = 0; sr < simpleRobots.size(); sr++)
    {
      SimpleRobot& robot = simpleRobots[sr];
      const Vector2f poseDiff = (perceptFieldCoordinates - robot.pose.translation);
      Vector2f robotRelative = Transformation::fieldToRobot(theRobotPose, robot.pose.translation);
      Vector2f robotAngles;
      robotAngles.x() = std::atan2(robotRelative.y(), robotRelative.x());
      robotAngles.y() = std::atan2(cameraMatrix.translation.z(), robotRelative.norm());

      Vector2f perceptAngles;
      perceptAngles.x() = std::atan2(i->locationOnField.translation.y(), i->locationOnField.translation.x());
      perceptAngles.y() = std::atan2(cameraMatrix.translation.z(), i->locationOnField.translation.norm());

      Angle yDiff = robotAngles.y() - perceptAngles.y();
      Angle xDiff = robotAngles.x() - perceptAngles.x();
      //float poseDiffNorm = poseDiff.norm();
      if (std::abs(yDiff) < mergeAngleYDiff && std::abs(xDiff) < mergeAngleXDiff)
      {
        merged = true;
        Vector2f oldTrans = robot.pose.translation;
        robot.pose.translation += poseDiff * (i->validity * mergeLocationFactor);
        robot.validity = std::min(1.f, robot.validity + i->validity * 0.1f);
        float alpha = 0.2f;
        if (!robot.fromTeamMatePoses)
        {
          robot.velocity = robot.velocity * alpha
              + (robot.pose.translation - oldTrans) * (1 - alpha) * ((1000.f * theFrameInfo.cycleTime) / (1.f + (float)theFrameInfo.getTimeSince(robot.timeOfLastUpdate)));
        }
        float velocityNorm = robot.velocity.norm();
        if (velocityNorm > 250.f)
          robot.velocity *= 250.f / velocityNorm;

        switch (i->robotType)
        {
        case RobotEstimate::teammateRobot:
          robot.teamMateCounter = std::max(-minColorCount + 1, robot.teamMateCounter + 1);
          break;
        case RobotEstimate::opponentRobot:
          robot.teamMateCounter = std::min(minColorCount - 1, robot.teamMateCounter - 1);
          break;
        default:
          break;
        }
        robot.timeOfLastUpdate = theFrameInfo.time;
        robot.timeOfLastLocalUpdate = theFrameInfo.time;
        robot.fromTeamMatePoses = robot.fromTeamMatePoses;
        break;
      }
    }
    if (!merged)
    {
      SimpleRobot newRobot;
      newRobot.fromTeamMatePoses = false;
      newRobot.pose = perceptFieldCoordinates;
      newRobot.velocity = Vector2f::Zero();
      if (i->robotType == RobotEstimate::teammateRobot)
        newRobot.teamMateCounter = 1;
      else if (i->robotType == RobotEstimate::opponentRobot)
        newRobot.teamMateCounter = -1;
      else
        newRobot.teamMateCounter = 0;
      newRobot.validity = std::min((float)i->validity, maxStartValidity);
      newRobot.timeOfLastUpdate = theFrameInfo.time;
      newRobot.timeOfLastLocalUpdate = theFrameInfo.time;
      simpleRobots.push_back(newRobot);
    }
  }
}

void SimpleRobotMapProvider::updateWithTeamMatePoses()
{
  for (auto& mate : theTeammateData.teammates)
  {
    const RobotPose& otherPose = mate.pose;
    if (otherPose.validity > 0.7f && mate.status != Teammate::INACTIVE && mate.timeWhenLastPacketReceived == theFrameInfo.time)
    {
      bool merged = false;
      for (unsigned int sr = 0; sr < simpleRobots.size(); sr++)
      {
        SimpleRobot& robot = simpleRobots[sr];

        const Vector2f poseDiff = (otherPose.translation - robot.pose.translation);
        if (poseDiff.norm() < mergeLocationDiff)
        {
          merged = true;
          robot.pose.rotation = otherPose.rotation;
          robot.pose.translation = otherPose.translation;
          robot.validity = otherPose.validity;
          robot.velocity = mate.walkRequest.request.translation;

          robot.teamMateCounter = minColorCount * 5;
          robot.fromTeamMatePoses = true;
          robot.timeOfLastUpdate = theFrameInfo.time;
          break;
        }
      }
      if (!merged)
      {
        SimpleRobot newRobot;
        newRobot.fromTeamMatePoses = true;
        newRobot.pose = otherPose;
        newRobot.velocity = mate.walkRequest.request.translation;
        newRobot.teamMateCounter = minColorCount * 5;
        newRobot.validity = otherPose.validity;
        newRobot.timeOfLastUpdate = theFrameInfo.time;
        newRobot.timeOfLastLocalUpdate = 0;
        simpleRobots.push_back(newRobot);
      }
    }
  }
}

void SimpleRobotMapProvider::updateWithGlobalData()
{
  for (auto& mate : theTeammateData.teammates)
  {
    if (mate.pose.validity < 0.7f || mate.status != Teammate::FULLY_ACTIVE)
      continue;
    for (auto& teamRobot : mate.simpleRobotsDistributed.robots)
    {
      bool merged = false;
      Vector2f perceptFieldCoordinates = Vector2f((float)teamRobot.x, (float)teamRobot.y);

      // check if percept is robot itself
      if ((perceptFieldCoordinates - theRobotPose.translation).norm() < mergeLocationDiff)
        continue;
      for (unsigned int sr = 0; sr < simpleRobots.size(); sr++)
      {
        SimpleRobot& robot = simpleRobots[sr];
        const Vector2f poseDiff = (perceptFieldCoordinates - robot.pose.translation);
        if (poseDiff.norm() < mergeLocationDiff && teamRobot.timeOfLastUpdate > robot.timeOfLastUpdate)
        {
          merged = true;
          Vector2f oldTrans = robot.pose.translation;
          float otherVal = static_cast<float>(teamRobot.validity) / 1000.f;
          robot.pose.translation += poseDiff * otherVal;
          // TODO: work with validty from simple robots and send them
          robot.validity = std::max(otherVal, robot.validity);
          float alpha = 0.2f;
          if (!robot.fromTeamMatePoses)
          {
            robot.velocity = robot.velocity * alpha
                + (robot.pose.translation - oldTrans) * (1 - alpha) * ((1000.f * theFrameInfo.cycleTime) / ((float)theFrameInfo.getTimeSince(robot.timeOfLastUpdate) + 1.f));
          }
          float velocityNorm = robot.velocity.norm();
          if (velocityNorm > 250.f)
            robot.velocity *= 250.f / velocityNorm;
          robot.teamMateCounter += sgn(teamRobot.colorCount);

          robot.timeOfLastUpdate = theFrameInfo.time;
          robot.fromTeamMatePoses = false;
          break;
        }
      }
      if (!merged && useTeamMatePercepts)
      {
        SimpleRobot newRobot;
        newRobot.fromTeamMatePoses = false;
        newRobot.pose = perceptFieldCoordinates;
        newRobot.velocity = Vector2f((float)teamRobot.velocity.x(), (float)teamRobot.velocity.y());
        float velocityNorm = newRobot.velocity.norm();
        if (velocityNorm > 250.f)
          newRobot.velocity *= 250.f / velocityNorm;
        newRobot.teamMateCounter = teamRobot.colorCount;

        newRobot.validity = (float)teamRobot.validity / 1000.f;
        newRobot.timeOfLastUpdate = teamRobot.timeOfLastUpdate;
        newRobot.timeOfLastLocalUpdate = 0;
        simpleRobots.push_back(newRobot);
      }
    }
  }
}

void SimpleRobotMapProvider::mergeSimpleRobots()
{
  for (unsigned int id = 0; id < simpleRobots.size(); id++)
  {
    SimpleRobot& robot = simpleRobots[id];
    for (unsigned int id2 = id + 1; id2 < simpleRobots.size(); id2++)
    {
      SimpleRobot& otherRobot = simpleRobots[id2];
      const Vector2f poseDiff = (otherRobot.pose.translation - robot.pose.translation);
      if (poseDiff.norm() < mergeLocationDiff && !(robot.teamMateCounter > 0 && otherRobot.teamMateCounter < 0) && !(robot.teamMateCounter < 0 && otherRobot.teamMateCounter > 0))
      {
        robot.pose.rotation = (robot.pose.rotation + otherRobot.pose.rotation) * 0.5f;
        robot.pose.translation = (robot.pose.translation + otherRobot.pose.translation) * 0.5f;
        robot.validity = std::max(robot.validity, otherRobot.validity);
        if (!robot.fromTeamMatePoses && !otherRobot.fromTeamMatePoses)
        {
          robot.velocity = (robot.velocity + otherRobot.velocity) * 0.5f;
        }
        else
        {
          if (otherRobot.fromTeamMatePoses)
          {
            robot.velocity = otherRobot.velocity;
          }
        }
        float velocityNorm = robot.velocity.norm();
        if (velocityNorm > 250.f)
          robot.velocity *= 250.f / velocityNorm;
        robot.teamMateCounter = (robot.teamMateCounter > 0 ? std::max(robot.teamMateCounter, otherRobot.teamMateCounter) : std::min(robot.teamMateCounter, otherRobot.teamMateCounter));
        robot.fromTeamMatePoses = robot.fromTeamMatePoses || otherRobot.fromTeamMatePoses;
        robot.timeOfLastUpdate = std::max(robot.timeOfLastUpdate, otherRobot.timeOfLastUpdate);
        robot.timeOfLastLocalUpdate = std::max(robot.timeOfLastLocalUpdate, otherRobot.timeOfLastLocalUpdate);
        simpleRobots.erase(simpleRobots.begin() + id2);
        break;
      }
    }
  }
}

void SimpleRobotMapProvider::prune()
{
  unsigned int size = static_cast<unsigned int>(simpleRobots.size());

  while (size > maxRobots)
  {
    float minVal = 1.f;
    unsigned int minIndex = 0;
    for (unsigned int sr = 0; sr < size; sr++)
    {
      if (simpleRobots[sr].validity < minVal)
      {
        minVal = simpleRobots[sr].validity;
        minIndex = sr;
      }
    }
    simpleRobots.erase(simpleRobots.begin() + minIndex);
    size = static_cast<unsigned int>(simpleRobots.size());
  }
}

void SimpleRobotMapProvider::predict()
{
  for (unsigned int sr = 0; sr < simpleRobots.size(); sr++)
  {
    SimpleRobot& robot = simpleRobots[sr];
    unsigned timeSinceLastLocalUpdate = theFrameInfo.getTimeSince(simpleRobots[sr].timeOfLastLocalUpdate);
    unsigned timeSinceLastRemoteUpdate = theFrameInfo.getTimeSince(simpleRobots[sr].timeOfLastUpdate);

    if (robot.fromTeamMatePoses)
    {
      if (timeSinceLastRemoteUpdate <= timeSinceLastLocalUpdate && theFrameInfo.getTimeSince(robot.timeOfLastUpdate) > 0 && robot.velocity.norm() > 10.f) // && robot.velocity.norm() / (theFrameInfo.cycleTime) > 20)
      {
        unsigned timeInMsSinceLastUpdate = theFrameInfo.getTimeSince(robot.timeOfLastUpdate);
        float velocityPerTickFactor = (timeInMsSinceLastUpdate / 1000.f);
        robot.pose.translate(robot.velocity * velocityPerTickFactor);
        robot.velocity *= 0.99f;
        robot.timeOfLastUpdate = theFrameInfo.time;
      }
    }
  }
}

void SimpleRobotMapProvider::fillRobotMap()
{
  std::vector<SimpleRobot>::const_iterator i = simpleRobots.begin();
  std::vector<SimpleRobot>::const_iterator end = simpleRobots.end();
  for (; i != end; i++)
  {
    if (i->validity > minValidity)
    {
      RobotMapEntry re;
      if (i->teamMateCounter >= minColorCount)
        re.robotType = RobotEstimate::teammateRobot;
      else if (i->teamMateCounter <= -minColorCount)
        re.robotType = RobotEstimate::opponentRobot;
      else
        re.robotType = RobotEstimate::unknownRobot;
      re.pose = i->pose;
      re.velocity = i->velocity;
      re.validity = i->validity;
      internalRobotMap.robots.push_back(re);
    }
  }
  for (unsigned int sr = 0; sr < simpleRobots.size(); sr++)
  {
    int timeSinceLastLocalUpdate = theFrameInfo.getTimeSince(simpleRobots[sr].timeOfLastLocalUpdate);
    int timeSinceLastRemoteUpdate = theFrameInfo.getTimeSince(simpleRobots[sr].timeOfLastUpdate);

    if (simpleRobots[sr].validity < 0.05f || (timeSinceLastLocalUpdate > maxLocalLifetimeOfUnseenRobot && !simpleRobots[sr].fromTeamMatePoses) || (timeSinceLastRemoteUpdate > maxLifetimeOfUnseenRobot))
    {
      simpleRobots.erase(simpleRobots.begin() + sr);
      sr--;
    }
  }
}

MAKE_MODULE(SimpleRobotMapProvider, modeling)
