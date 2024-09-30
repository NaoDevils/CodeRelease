#include <QDragEnterEvent>
#include <QFormLayout>
#include <QLabel>
#include <QMenu>
#include <QProgressBar>
#include <QDrag>
#include <QMimeData>
#include <QApplication>

#include <sstream>
#include <iostream>

#include "models/Robot.h"
#include "models/Team.h"
#include "tools/ShellTools.h"
#include "ui/RobotView.h"
#include "ui/TeamSelector.h"
#include "ui/SensorWindow.h"
#include "Platform/SystemCall.h"
#include <naodevilsbase/naodevilsbase.h>


void RobotView::init()
{
  QFormLayout* layout = new QFormLayout();

  if (playerNumber)
  {
    cPlayerNumber = new QLabel(QString("<font size=5><b>") + QString::number(playerNumber) + QString("</b></font>"));
    cPlayerNumber->setTextInteractionFlags(Qt::NoTextInteraction);
  }

  statusWidget = new QWidget(this);
  statusWidget->setMaximumSize(320, 160);
  this->setMaximumWidth(250);
  this->setMaximumHeight(155);
  QGridLayout* statusLayout = new QGridLayout(statusWidget);
  this->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(ShowContextMenu(const QPoint&)));

  QLabel* bodyNameLabel = new QLabel(QString("<font size=2><b>") + QString("Body") + QString("</b></font>"));
  bodyNameLabel->setTextInteractionFlags(Qt::NoTextInteraction);
  bodyName = new QLabel(this);
  bodyName->setMaximumSize(90, 20);
  bodyName->setMinimumSize(60, 20);
  bodyName->setAlignment(Qt::AlignCenter);
  statusLayout->addWidget(bodyNameLabel, 0, 0, Qt::AlignLeft);
  statusLayout->addWidget(bodyName, 0, 1);

  QLabel* pingLabelWLAN = new QLabel("<font size=2><b>Wlan</b></font>", statusWidget);
  pingLabelWLAN->setTextInteractionFlags(Qt::NoTextInteraction);
  pingBarWLAN = new QLabel(this);
  pingBarWLAN->setMaximumSize(90, 20);
  pingBarWLAN->setMinimumSize(60, 20);
  pingBarWLAN->setAlignment(Qt::AlignCenter);
  //setPings(WLAN, 0);
  statusLayout->addWidget(pingLabelWLAN, 1, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarWLAN, 1, 1);

  QLabel* pingLabelLAN = new QLabel("<font size=2><b>Lan</b></font>", statusWidget);
  pingLabelLAN->setTextInteractionFlags(Qt::NoTextInteraction);
  pingBarLAN = new QLabel(this);
  pingBarLAN->setMaximumSize(90, 20);
  pingBarLAN->setMinimumSize(60, 20);
  pingBarLAN->setAlignment(Qt::AlignCenter);
  //setPings(LAN, 0);
  statusLayout->addWidget(pingLabelLAN, 2, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarLAN, 2, 1);

  QLabel* powerLabel = new QLabel("<font size=2><b>Power</b></font>", statusWidget);
  powerLabel->setTextInteractionFlags(Qt::NoTextInteraction);
  powerBar = new QProgressBar(this);
  powerBar->setMaximumSize(90, 20);
  powerBar->setMinimumSize(60, 20);
  powerBar->setRange(0, 100);
  powerBar->reset();
  powerBar->setAlignment(Qt::AlignCenter);
  statusLayout->addWidget(powerLabel, 3, 0, Qt::AlignLeft);
  statusLayout->addWidget(powerBar, 3, 1);

  QLabel* tempLabel = new QLabel("<font size=2><b>Temp</b></font>", statusWidget);
  tempLabel->setTextInteractionFlags(Qt::NoTextInteraction);
  tempBar = new QProgressBar(this);
  tempBar->setMaximumSize(90, 20);
  tempBar->setMinimumSize(60, 20);
  tempBar->setRange(0, 100);
  tempBar->reset();
  tempBar->setAlignment(Qt::AlignCenter);
  tempBar->setRange(20, 110);
  tempBar->setFormat("%v°C");
  statusLayout->addWidget(tempLabel, 4, 0, Qt::AlignLeft);
  statusLayout->addWidget(tempBar, 4, 1);

  layout->addRow(cPlayerNumber, statusWidget);

  setLayout(layout);
  setStyleSheet("QGroupBox { padding: 15% 0 0 0; }");
  connect(this, SIGNAL(toggled(bool)), this, SLOT(setSelected(bool)));

  if (robot)
  {
    sensorMap = {{"name", robot->name}};
    Session::getInstance().registerDataListener(this, robot);
  }

  update();
  setAcceptDrops(true);

  dataTimer.setSingleShot(true);
  connect(&dataTimer,
      &QTimer::timeout,
      [&]()
      {
        changeData(sensorMap);
      });
}

void RobotView::update()
{
  if (playerNumber)
    cPlayerNumber->setVisible(false);
  statusWidget->setVisible(false);
  setCheckable(false);
  if (robot)
  {
    RobotConfigDorsh* r = robot;
    robot = 0;
    setCheckable(playerNumber);
    robot = r;
    if (playerNumber)
      setChecked(isSelected());
    std::string ipPostfix = robot->wlan.substr(robot->wlan.length() - 2);
    setTitle(QString::fromStdString(robot->name + " (." + ipPostfix + ")"));
    if (playerNumber)
    {
      cPlayerNumber->setEnabled(isSelected());
      cPlayerNumber->setVisible(true);
    }
    statusWidget->setVisible(true);
  }
  else
  {
    setTitle("Empty");
    if (playerNumber)
    {
      cPlayerNumber->setEnabled(false);
      cPlayerNumber->setVisible(true);
    }
  }
  changeData(sensorMap);
}

RobotView::RobotView(TeamSelector* teamSelector, RobotConfigDorsh* robot, unsigned short playerNumber, unsigned short pos)
    : QGroupBox(teamSelector), teamSelector(teamSelector), robot(robot), playerNumber(playerNumber), pos(pos), cPlayerNumber(0)
{
  init();
}

RobotView::RobotView(TeamSelector* teamSelector, RobotConfigDorsh* robot)
    : QGroupBox(teamSelector), teamSelector(teamSelector), robot(robot), playerNumber(0), pos(0), cPlayerNumber(0)
{
  init();
}

QString RobotView::getRobotName() const
{
  if (!robot)
    return "";
  return QString::fromStdString(robot->name);
}

bool RobotView::isSelected() const
{
  if (!robot)
    return false;
  return teamSelector->getSelectedTeam()->isPlayerSelected(robot);
}

void RobotView::setRobot(RobotConfigDorsh* robot)
{
  if (this->robot)
  {
    Session::getInstance().removeDataListener(this, this->robot);
  }
  this->robot = robot;
  if (playerNumber)
  {
    Team* team = teamSelector->getSelectedTeam();
    team->changePlayer(playerNumber, pos, robot);
  }
  if (robot)
  {
    Session::getInstance().registerDataListener(this, robot);
  }
}

unsigned RobotView::lastDataUpdate(const nlohmann::json& json)
{
  const unsigned currentTime = SystemCall::getRealSystemTime();

  const unsigned lan = json.contains("lan") ? currentTime - json["lan"].get<unsigned>() : std::numeric_limits<unsigned>::max();
  const unsigned wlan = json.contains("wlan") ? currentTime - json["wlan"].get<unsigned>() : std::numeric_limits<unsigned>::max();

  return std::min(lan, wlan);
}

unsigned RobotView::lastUpdate(const nlohmann::json& json)
{
  const unsigned currentTime = SystemCall::getRealSystemTime();

  const unsigned tc = json.contains("tc") ? currentTime - json["tc"].get<unsigned>() : std::numeric_limits<unsigned>::max();

  return std::min(tc, lastDataUpdate(json));
}

void RobotView::setPings(const nlohmann::json& json)
{
  if (!robot)
    return;

  if (json.at("name").get<std::string>() != robot->name)
    return;

  const unsigned currentTime = SystemCall::getRealSystemTime();
  if (json.contains("lan") && currentTime - json["lan"].get<unsigned>() < dataTimeout)
    pingBarLAN->setStyleSheet("QLabel { background-color : lime; border: 1px solid silver; }");
  else
    pingBarLAN->setStyleSheet("QLabel { background-color : #e6e6e6; border: 1px solid silver; }");

  if (json.contains("wlan") && currentTime - json["wlan"].get<unsigned>() < dataTimeout)
    pingBarWLAN->setStyleSheet("QLabel { background-color : lime; border: 1px solid silver; font-size: 7pt; }");
  else if (json.contains("tc") && currentTime - json["tc"].get<unsigned>() < dataTimeout)
    pingBarWLAN->setStyleSheet("QLabel { background-color : yellow; border: 1px solid silver; font-size: 7pt; }");
  else
    pingBarWLAN->setStyleSheet("QLabel { background-color : #e6e6e6; border: 1px solid silver; font-size: 7pt; }");

  if (lastDataUpdate(json) < dataTimeout && json.contains("wifi_state"))
    pingBarWLAN->setText(QString::fromStdString(json["wifi_state"].get<std::string>()));
  else
    pingBarWLAN->setText("");
}

void RobotView::setPower(const nlohmann::json& json)
{
  if (!robot)
    return;

  if (json.at("name").get<std::string>() != robot->name)
    return;

  if (lastDataUpdate(json) < dataTimeout && json.contains("sensors"))
  {
    const float battery_charge = json.at("sensors").at("battery").at(0).get<float>();
    const float battery_status = json.at("sensors").at("battery").at(1).get<float>();

    int value = static_cast<int>(battery_charge * 100.f);
    bool powerPlugged = !(static_cast<short>(battery_status) & 0b100000);

    if (powerPlugged)
      powerBar->setStyleSheet("QProgressBar::chunk { background-color: lime; border: 1px solid silver; }");
    else
      powerBar->setStyleSheet("QProgressBar::chunk { background-color: red; border: 1px solid silver; }");

    powerBar->setValue(value);
  }
  else
  {
    powerBar->reset();
    powerBar->setStyleSheet("QProgressBar::chunk { background-color: #e6e6e6; border: 1px solid silver; }");
  }
}

void RobotView::setTemp(const nlohmann::json& json)
{
  if (!robot)
    return;

  if (json.at("name").get<std::string>() != robot->name)
    return;

  if (lastDataUpdate(json) < dataTimeout && json.contains("sensors"))
  {
    const std::vector<float> temperatures = json.at("sensors").at("temperature").get<std::vector<float>>();
    if (temperatures.size() != NDData::Joint::numOfJoints)
      return;

    const auto maxTemp = std::max_element(temperatures.begin() + NDData::Joint::lHipYawPitch, temperatures.begin() + NDData::Joint::rAnkleRoll + 1);
    static constexpr float lowTemp = 40.f;
    static constexpr float highTemp = 90.f;
    const float temp = std::clamp(*maxTemp, lowTemp, highTemp);

    const int hue = static_cast<int>(120.f - (temp - lowTemp) / (highTemp - lowTemp) * 120.f);
    QColor color;
    color.setHsv(hue, 255, 255);
    tempBar->setStyleSheet(QString("QProgressBar::chunk { background-color: %1; border: 1px solid silver; }").arg(color.name()));
    tempBar->setValue(*maxTemp);

    return;
  }

  tempBar->reset();
  tempBar->setStyleSheet("QProgressBar::chunk { background-color: #e6e6e6; border: 1px solid silver; }");
}

void RobotView::setMap(const nlohmann::json& json)
{
  if (!robot)
    return;

  if (json.at("name").get<std::string>() != robot->name)
    return;

  sensorMap = json;

  if (unsigned timediff = lastUpdate(json); timediff < dataTimeout)
    dataTimer.start(dataTimeout - timediff);
  else
    dataTimer.stop();

  QString bodyText = "";
  int state = 0;


  if (lastDataUpdate(json) < dataTimeout)
  {
    const auto sameBodyId = [&](const auto& r)
    {
      return sensorMap.contains("bodyId") && r.second->bodyId == sensorMap.at("bodyId").get<std::string>();
    };
    const auto bodyIt = std::find_if(Session::getInstance().robotsByName.begin(), Session::getInstance().robotsByName.end(), sameBodyId);

    if (bodyIt != Session::getInstance().robotsByName.end())
      bodyText = QString::fromStdString(bodyIt->first);

    state = sensorMap.contains("state") ? sensorMap.at("state").get<int>() : 0;

    float hardware_quality = sensorMap.contains("hardware_quality") ? sensorMap.at("hardware_quality").get<float>() : -1.f;
    if (hardware_quality >= 0.f)
    {
      if (hardware_quality > 0.75)
        bodyName->setStyleSheet("QLabel { background-color : lime; border: 1px solid silver; font-size: 7pt; }");
      else if (hardware_quality > 0.50)
        bodyName->setStyleSheet("QLabel { background-color : yellow; border: 1px solid silver; font-size: 7pt; }");
      else if (hardware_quality > 0.25)
        bodyName->setStyleSheet("QLabel { background-color : orange; border: 1px solid silver; font-size: 7pt; }");
      else
        bodyName->setStyleSheet("QLabel { background-color : red; border: 1px solid silver; font-size: 7pt; }");
    }
    else
    {
      bodyName->setStyleSheet("QLabel { background-color: #e6e6e6; border: 1px solid silver; font-size: 7pt; }");
    }
  }
  else
  {
    bodyName->setStyleSheet("QLabel { background-color: #e6e6e6; border: 1px solid silver; font-size: 7pt; }");
  }

  if (strcmp(bodyText.toStdString().c_str(), robot->name.c_str()) == 0)
    bodyName->setText("<font size = 4>" + bodyText + "</font>");
  else
    bodyName->setText("<font size = 4><b>" + bodyText + "</b></font>");

  if (playerNumber)
  {
    switch (state)
    {
    case 0:
      cPlayerNumber->setText("<font size = 5><b>" + QString::number(playerNumber) + "</b></font>");
      cPlayerNumber->setToolTip("");
      break;
    case 15:
      cPlayerNumber->setText("<font size = 5 color = crimson><b>" + QString::number(playerNumber) + "</b></font>");
      cPlayerNumber->setToolTip("Framework stopped");
      break;
    default:
      cPlayerNumber->setText("<font size = 10 color = crimson><b>&#9760;</b></font>");
      cPlayerNumber->setToolTip("Framework crashed");
    }
  }
}

void RobotView::changeData(const nlohmann::json& json)
{
  setPower(json);
  setTemp(json);
  setMap(json);
  setPings(json);
}

void RobotView::mousePressEvent(QMouseEvent* e)
{
  if (e->button() == Qt::LeftButton)
    dragStartPosition = e->pos();
}

void RobotView::mouseMoveEvent(QMouseEvent* me)
{
  if (!robot)
    return;
  if (!(me->buttons() & Qt::LeftButton))
    return;
  if ((me->pos() - dragStartPosition).manhattanLength() < QApplication::startDragDistance())
    return;

  QDrag* d = new QDrag(this);
  QPixmap pm = grab();
  d->setPixmap(pm);
  d->setHotSpot(me->pos());
  QMimeData* data = new QMimeData();
  data->setText(QString::fromStdString(robot->name));
  d->setMimeData(data);
  d->exec(Qt::MoveAction);
  me->accept();
}

void RobotView::mouseReleaseEvent(QMouseEvent* e)
{
  if (e->button() == Qt::LeftButton)
    setChecked(!isChecked());
}

void RobotView::dragEnterEvent(QDragEnterEvent* e)
{
  if (e->source() && e->source() != this && e->source()->inherits("RobotView"))
  {
    const RobotView* source = dynamic_cast<RobotView*>(e->source());
    if (source->playerNumber || playerNumber)
      e->acceptProposedAction();
  }
}

void RobotView::swap(RobotView& other)
{
  if (!this->playerNumber)
    other.setSelected(false);

  if (!other.playerNumber)
    this->setSelected(false);

  RobotConfigDorsh* thisRobot = this->robot;
  RobotConfigDorsh* otherRobot = other.robot;

  this->setRobot(otherRobot);
  other.setRobot(thisRobot);

  std::swap(this->sensorMap, other.sensorMap);

  this->update();
  other.update();

  emit this->robotChanged();
  emit other.robotChanged();
}

void RobotView::dropEvent(QDropEvent* e)
{
  e->accept();
  RobotView* source = dynamic_cast<RobotView*>(e->source());

  swap(*source);
}

void RobotView::setSelected(bool selected)
{
  if (robot)
  {
    bool isPlayer = false;
    std::vector<RobotConfigDorsh*> players = teamSelector->getSelectedTeam()->getPlayers();
    for (RobotConfigDorsh* player : players)
    {
      if (player != nullptr && robot->name == player->name)
      {
        isPlayer = true;
      }
    }
    if (isPlayer)
    {
      teamSelector->getSelectedTeam()->setSelectPlayer(robot, selected);
      cPlayerNumber->setEnabled(selected);
    }
  }
}

void RobotView::ShowContextMenu(const QPoint& pos) // this is a slot
{
  // for most widgets
  // Rensen: removed due to unused warning
  // QPoint globalPos = statusWidget->mapToGlobal(pos);
  // for QAbstractScrollArea and derived classes you would use:
  // QPoint globalPos = myWidget->viewport()->mapToGlobal(pos);
  if (robot)
  {
    QMenu myMenu;
    QAction* action1 = myMenu.addAction("ssh LAN");
    QAction* action2 = myMenu.addAction("ssh WLAN");
    QAction* action4 = myMenu.addAction("show SensorReader"); //action for opening Simulator: see showSensorReader()
    connect(action1, SIGNAL(triggered()), this, SLOT(Cable()));
    connect(action2, SIGNAL(triggered()), this, SLOT(WIFI()));
    connect(action4, SIGNAL(triggered()), this, SLOT(sSR()));
    myMenu.exec(statusWidget->mapToGlobal(pos));
  }
}

void RobotView::openTerm(bool wlan)
{
  const auto [cmd, params] = connectCommand(wlan ? robot->wlan : robot->lan);
  std::string command = cmd.toStdString() + " \"" + params.join("\" \"").toStdString() + "\"";

#ifdef WINDOWS
  command = "start " + command;
  system(command.c_str());
#else
  command = "x-terminal-emulator -e '" + command + "' &";
  system(command.c_str());
#endif
}

void RobotView::showSensorReader()
{
  printf("SensorMap: %d\n", sensorMap.empty());
  if (!sensorMap.empty())
  {
    sensorWindow = new SensorWindow(sensorMap, robot->name);
    sensorWindow->show();
  }
}
