/**
 * @file Controller/RoboCupCtrl.cpp
 *
 * This file implements the class RoboCupCtrl.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <A href="mailto:kspiess@tzi.de">Kai Spiess</A>
 * @author Colin Graf
 */

#include "SimulatedRobot.h"
#include "RoboCupCtrl.h"
#include "Tools/ProcessFramework/Process.h"
#include "Platform/SimRobotQt/Robot.h"

#include <QIcon>

#ifdef MACOS
#define TOLERANCE 30.f
#else
#define TOLERANCE 10.f
#endif

RoboCupCtrl* RoboCupCtrl::controller = 0;
SimRobot::Application* RoboCupCtrl::application = 0;

RoboCupCtrl::RoboCupCtrl(SimRobot::Application& application) : robotName(0), simTime(false), delayTime(0), lastTime(0)
{
  Thread<RoboCupCtrl>::setName("Main");

  this->controller = this;
  this->application = &application;
  Q_INIT_RESOURCE(Controller);
}

bool RoboCupCtrl::compile()
{
  // find simulation object
  SimRobotCore2::Scene* scene = (SimRobotCore2::Scene*)application->resolveObject("RoboCup", SimRobotCore2::scene);
  if (!scene)
    return false;

  // initialize simulated time and step length
  time = 10000 - SystemCall::getRealSystemTime();
  simStepLength = int(scene->getStepLength() * 1000.f + 0.5f);
  if (simStepLength > 20)
    simStepLength = 20;
  delayTime = (float)simStepLength;

  // get interfaces to simulated objects
  SimRobot::Object* group = application->resolveObject("RoboCup.robots", SimRobotCore2::compound);
  for (unsigned currentRobot = 0, count = application->getObjectChildCount(*group); currentRobot < count; ++currentRobot)
  {
    SimRobot::Object* robot = (SimRobot::Object*)application->getObjectChild(*group, currentRobot);
    const QString& fullName = robot->getFullName();
    std::string robotName = fullName.toUtf8().constData();
    this->robotName = robotName.c_str();
    Settings settings;
    settings.load();
    GlobalKeeper k;
    robots.push_back(new Robot(fullName.mid(fullName.lastIndexOf('.') + 1).toStdString(), settings));
  }
  this->robotName = 0;
  const SimRobot::Object* balls = (SimRobotCore2::Object*)RoboCupCtrl::application->resolveObject("RoboCup.balls", SimRobotCore2::compound);
  if (balls)
  {
    SimulatedRobot::setBall(RoboCupCtrl::application->getObjectChild(*balls, 0));
    SimRobotCore2::Geometry* ballGeom = (SimRobotCore2::Geometry*)application->resolveObject("RoboCup.balls.ball.SphereGeometry", SimRobotCore2::geometry);
    if (ballGeom)
      ballGeom->registerCollisionCallback(*this);
  }

  updateColors();

  return true;
}

RoboCupCtrl::~RoboCupCtrl()
{
  qDeleteAll(views);
  SimulatedRobot::setBall(0);
  controller = 0;
  application = 0;
}

void RoboCupCtrl::addView(SimRobot::Object* object, const SimRobot::Object* parent, int flags)
{
  views.append(object);
  application->registerObject(*this, *object, parent, flags | SimRobot::Flag::showParent);
}

void RoboCupCtrl::addView(SimRobot::Object* object, const QString& categoryName, int flags)
{
  SimRobot::Object* category = application->resolveObject(categoryName);
  if (!category)
  {
    int lio = categoryName.lastIndexOf('.');
    QString subParentName = categoryName.mid(0, lio);
    QString name = categoryName.mid(lio + 1);
    category = addCategory(name, subParentName);
  }
  addView(object, category, flags);
}

SimRobot::Object* RoboCupCtrl::addCategory(const QString& name, const SimRobot::Object* parent, const char* icon)
{
  class Category : public SimRobot::Object
  {
  public:
    Category(const QString& name, const QString& fullName, const char* icon) : name(name), fullName(fullName), icon(icon) {}

  private:
    QString name;
    QString fullName;
    QIcon icon;

    virtual const QString& getDisplayName() const { return name; }
    virtual const QString& getFullName() const { return fullName; }
    virtual const QIcon* getIcon() const { return &icon; }
  };

  SimRobot::Object* category = new Category(name, parent ? parent->getFullName() + "." + name : name, icon ? icon : ":/Icons/folder.png");
  views.append(category);
  application->registerObject(*this, *category, parent, SimRobot::Flag::windowless | SimRobot::Flag::hidden);
  return category;
}

SimRobot::Object* RoboCupCtrl::addCategory(const QString& name, const QString& parentName)
{
  SimRobot::Object* parent = application->resolveObject(parentName);
  if (!parent)
  {
    int lio = parentName.lastIndexOf('.');
    QString subParentName = parentName.mid(0, lio);
    QString category = parentName.mid(lio + 1);
    parent = addCategory(category, subParentName);
  }
  return addCategory(name, parent);
}

void RoboCupCtrl::start()
{
#ifdef WINDOWS
  VERIFY(timeBeginPeriod(1) == TIMERR_NOERROR);
#endif
  MultiDebugSenderBase::terminating = false;
  for (std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    (*i)->start();
}

void RoboCupCtrl::stop()
{
  MultiDebugSenderBase::terminating = true;
  for (std::list<Robot*>::reverse_iterator i = robots.rbegin(); i != robots.rend(); ++i)
    (*i)->announceStop();
  for (std::list<Robot*>::reverse_iterator i = robots.rbegin(); i != robots.rend(); ++i)
  {
    GlobalKeeper k;
    (*i)->stop();
    delete *i;
  }
  controller = 0;
#ifdef WINDOWS
  VERIFY(timeEndPeriod(1) == TIMERR_NOERROR);
#endif
}

void RoboCupCtrl::update()
{
  if (delayTime != 0.f)
  {
    float t = (float)SystemCall::getRealSystemTime();
    lastTime += delayTime;
    if (lastTime > t) // simulation is running faster then rt
    {
      if (lastTime > t + TOLERANCE)
        SystemCall::sleep(int(lastTime - t - TOLERANCE));
    }
    else if (t > lastTime + TOLERANCE) // slower then rt
      lastTime = t - TOLERANCE;
  }

  gameController.referee();

  statusText = "";
  for (std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    (*i)->update();
  if (simTime)
    time += simStepLength;
}

void RoboCupCtrl::collided(SimRobotCore2::Geometry& geom1, SimRobotCore2::Geometry& geom2)
{
  SimRobotCore2::Body* body = geom2.getParentBody();
  if (!body)
    return;
  body = body->getRootBody();
  GameController::getInstance().setLastBallContactRobot(body);
}

std::string RoboCupCtrl::getRobotName() const
{
  size_t threadId = Thread<ProcessBase>::getCurrentId();
  for (std::list<Robot*>::const_iterator i = robots.begin(); i != robots.end(); ++i)
    for (ProcessList::const_iterator j = (*i)->begin(); j != (*i)->end(); ++j)
      if ((*j)->getId() == threadId)
        return (*i)->getName();
  if (!this->robotName)
    return "Robot1";
  std::string robotName(this->robotName);
  return robotName.substr(robotName.rfind('.') + 1);
}

void RoboCupCtrl::updateColors()
{
  std::map<int, int> ownFieldPlayerCounter, oppFieldPlayerCounter;
  uint8_t ownFieldPlayerColor = TEAM_YELLOW, ownGoalkeeperColor = TEAM_RED, oppFieldPlayerColor = TEAM_BLACK, oppGoalkeeperColor = TEAM_BLUE;

  const std::map<std::string, int> teamColorToEnumMap{
      {"nao-blue", 0},
      {"nao-red", 1},
      {"nao-yellow", 2},
      {"nao-black", 3},
      {"nao-white", 4},
      {"nao-green", 5},
      {"nao-orange", 6},
      {"nao-purple", 7},
      {"nao-brown", 8},
      {"nao-gray", 9},
  };

  for (int i = 0; i < 2; i++)
  {
    SimRobot::Object* group;
    if (i == 0) // Iterate "robots" group:
      group = RoboCupCtrl::application->resolveObject("RoboCup.robots", SimRobotCore2::compound);
    else // Iterate "extras" group:
      group = RoboCupCtrl::application->resolveObject("RoboCup.extras", SimRobotCore2::compound);

    for (unsigned currentRobot = 0, count = RoboCupCtrl::application->getObjectChildCount(*group); currentRobot < count; ++currentRobot)
    {
      SimRobot::Object* robot = (SimRobot::Object*)RoboCupCtrl::application->getObjectChild(*group, currentRobot);

      SimRobotCore2::Appearance* torsoAppearance = dynamic_cast<SimRobotCore2::Appearance*>(RoboCupCtrl::application->resolveObject({"naoTorsoV6"}, robot, SimRobotCore2::appearance));
      if (torsoAppearance)
      {
        const QString& fullNameTeamColor = RoboCupCtrl::application->getObjectChild(*torsoAppearance, 0u)->getFullName();
        const std::string baseNameTeamColor = fullNameTeamColor.mid(fullNameTeamColor.lastIndexOf('.') + 1).toUtf8().constData();
        const int robotColorEnum = teamColorToEnumMap.at(baseNameTeamColor);

        QString robotNumberString(robot->getFullName());
        const int pos = robotNumberString.lastIndexOf('.');
        robotNumberString.remove(0, pos + 6);
        const int robotNumber = robotNumberString.toInt() - 1;

        if (robotNumber == 0) // is own goalie
          ownGoalkeeperColor = robotColorEnum;
        else if (robotNumber == MAX_NUM_PLAYERS) // is opp goalie
          oppGoalkeeperColor = robotColorEnum;
        else
        {
          std::map<int, int>& enumColorMap = robotNumber < MAX_NUM_PLAYERS ? ownFieldPlayerCounter : oppFieldPlayerCounter;

          if (enumColorMap.find(robotColorEnum) == enumColorMap.end())
            enumColorMap[robotColorEnum] = 0;

          enumColorMap[robotColorEnum] += (i == 0) ? 2 : 1; // Active robots count double compared to dummys
        }
      }
    }
  }

  const auto valueLessThan = [](const auto& x, const auto& y)
  {
    return x.second < y.second;
  };
  if (!ownFieldPlayerCounter.empty())
  {
    const auto majorOwnColorIt = std::max_element(ownFieldPlayerCounter.begin(), ownFieldPlayerCounter.end(), valueLessThan);
    ownFieldPlayerColor = majorOwnColorIt->first;
  }
  if (!oppFieldPlayerCounter.empty())
  {
    const auto majorOppColorIt = std::max_element(oppFieldPlayerCounter.begin(), oppFieldPlayerCounter.end(), valueLessThan);
    oppFieldPlayerColor = majorOppColorIt->first;
  }

  gameController.setTeamColors(ownFieldPlayerColor, ownGoalkeeperColor, oppFieldPlayerColor, oppGoalkeeperColor);
}

unsigned RoboCupCtrl::getTime() const
{
  if (simTime)
    return unsigned(time);
  else
    return unsigned(SystemCall::getRealSystemTime() + time);
}
