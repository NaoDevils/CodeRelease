#include <QDragEnterEvent>
#include <QFormLayout>
#include <QLabel>
#include <QMenu>
#include <QProgressBar>

#include <sstream>

#include "Utils/dorsh/models/Robot.h"
#include "Utils/dorsh/models/Team.h"
#include "Utils/dorsh/tools/StringTools.h"
#include "Utils/dorsh/tools/ShellTools.h"
#include "Utils/dorsh/ui/RobotView.h"
#include "Utils/dorsh/ui/TeamSelector.h"
#include "Utils/dorsh/ui/SensorWindow.h"
#include "Utils/dorsh/tools/StringTools.h"



void RobotView::init()
{
  QFormLayout* layout = new QFormLayout();

  if(playerNumber)
    cPlayerNumber = new QLabel(QString("<font size=5><b>") + QString::number(playerNumber) + QString("</b></font>"));

  statusWidget = new QWidget(this);
  statusWidget->setMaximumSize(320, 160);
  this->setMaximumWidth(250);
  this->setMaximumHeight(155);
  QGridLayout* statusLayout = new QGridLayout(statusWidget);
  this->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),
    this, SLOT(ShowContextMenu(const QPoint&)));

  QLabel* bodyNameLabel = new QLabel(QString("<font size=2><b>") + QString("Body") + QString("</b></font>"));
  bodyName = new QLabel(this);
  bodyName->setMaximumSize(90, 20);
  bodyName->setMinimumSize(60, 20);
  bodyName->setAlignment(Qt::AlignCenter);
  statusLayout->addWidget(bodyNameLabel, 0, 0, Qt::AlignLeft);
  statusLayout->addWidget(bodyName, 0, 1);

  QLabel* pingLabelWLAN = new QLabel("<font size=2><b>Wlan</b></font>", statusWidget);
  pingBarWLAN = new QLabel(this);
  pingBarWLAN->setMaximumSize(90, 20);
  pingBarWLAN->setMinimumSize(60, 20);
  pingBarWLAN->setAlignment(Qt::AlignCenter);
  //setPings(WLAN, 0);
  statusLayout->addWidget(pingLabelWLAN, 1, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarWLAN, 1, 1);

  QLabel* pingLabelLAN = new QLabel("<font size=2><b>Lan</b></font>", statusWidget);
  pingBarLAN = new QLabel(this);
  pingBarLAN->setMaximumSize(90, 20);
  pingBarLAN->setMinimumSize(60, 20);
  pingBarLAN->setAlignment(Qt::AlignCenter);
  //setPings(LAN, 0);
  statusLayout->addWidget(pingLabelLAN, 2, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarLAN, 2, 1);

  QLabel* powerLabel = new QLabel("<font size=2><b>Power</b></font>", statusWidget);
  powerBar = new QProgressBar(this);
  powerBar->setMaximumSize(90, 20);
  powerBar->setMinimumSize(60, 20);
  powerBar->setRange(0, 100);
  powerBar->setValue(0);
  powerBar->setAlignment(Qt::AlignCenter);
  if (robot)
  {
    std::map<std::string, std::string> emptyMap;
    emptyMap[robot->name] = 1;
    emptyMap["battery_charge"] = "0.000000";
    emptyMap["battery_status"] = "-32708.000000";
    setPower(emptyMap);
    setMap(emptyMap);
  }
  statusLayout->addWidget(powerLabel, 3, 0, Qt::AlignLeft);
  statusLayout->addWidget(powerBar, 3, 1);

  layout->addRow(cPlayerNumber, statusWidget);

  setLayout(layout);
  connect(this, SIGNAL(toggled(bool)), this, SLOT(setSelected(bool)));

  if(robot)
  {
    Session::getInstance().registerDataListener(this, robot);
    Session::getInstance().registerWifiListener(this, robot);
  }

  wifiOverMap = false;
  wifiOverTeamPack = true;

  update();
  setAcceptDrops(true);
}

void RobotView::update()
{
  if(playerNumber)
    cPlayerNumber->setVisible(false);
  statusWidget->setVisible(false);
  setCheckable(false);
  if(robot)
  {
    RobotConfigDorsh* r = robot;
    robot = 0;
    setCheckable(playerNumber);
    robot = r;
    if(playerNumber)
      setChecked(isSelected());
    std::string ipPostfix = robot->wlan.substr(robot->wlan.length() - 2);
    setTitle(fromString(robot->name + " (." + ipPostfix + ")"));
    if(playerNumber)
    {
      cPlayerNumber->setEnabled(isSelected());
      cPlayerNumber->setVisible(true);
    }
    statusWidget->setVisible(true);
  }
  else
  {
    setTitle(fromString("Empty"));
    cPlayerNumber->setVisible(true);
    cPlayerNumber->setEnabled(false);
  }
}

RobotView::RobotView(TeamSelector* teamSelector,
                     RobotConfigDorsh* robot,
                     unsigned short playerNumber,
                     unsigned short pos)
  : QGroupBox(teamSelector),
    teamSelector(teamSelector),
    robot(robot),
    playerNumber(playerNumber),
    pos(pos),
    cPlayerNumber(0)
{
  init();
}

RobotView::RobotView(TeamSelector* teamSelector,
                     RobotConfigDorsh* robot)
  : QGroupBox(teamSelector),
    teamSelector(teamSelector),
    robot(robot),
    playerNumber(0),
    pos(0),
    cPlayerNumber(0)
{
  init();
}

QString RobotView::getRobotName() const
{
  if(!robot)
    return "";
  return fromString(robot->name);
}

bool RobotView::isSelected() const
{
  if(!robot)
    return false;
  return teamSelector->getSelectedTeam()->isPlayerSelected(robot);
}

void RobotView::setRobot(RobotConfigDorsh* robot)
{
  if(this->robot)
  {
    Session::getInstance().removeDataListener(this, this->robot);
    Session::getInstance().removeWifiListener(this, this->robot);
  }
  this->robot = robot;
  if(playerNumber)
  {
    Team* team = teamSelector->getSelectedTeam();
    team->changePlayer(playerNumber, pos, robot);
  }
  if(robot)
  {
    Session::getInstance().registerDataListener(this, robot);
    Session::getInstance().registerWifiListener(this, robot);
  }
  emit robotChanged();
}

void RobotView::changeWifi(std::string robotName, bool connected)
{
  if (robot)
  {
    if (robot->name == robotName)
    {
      QLabel* bar = pingBarWLAN;
      if (connected)
      {
        bar->setStyleSheet("QLabel { background-color : lime; border: 1px solid silver; }");
        wifiOverTeamPack = true;
        wifiOverMap = false;
      }
      else
      {
        if (!wifiOverMap)
        {
          bar->setStyleSheet("QLabel { background-color : #e6e6e6; border: 1px solid silver; }");
          wifiOverTeamPack = false;
        }

      }
    }
  }
}

void RobotView::setPings(std::map<std::string, std::string> map)
{
  if (robot)
  {
    auto iter = map.find(robot->name);
    if (iter != map.cend())
    {
      auto lanConnection = map.find("lan_connection");
      if (lanConnection != map.cend())
      {
        QLabel* bar = pingBarLAN;

        std::stringstream stream;
        std::string lan_connection = map["lan_connection"];

        bool lanConnected;
        stream << lan_connection;
        stream >> lanConnected;

        if (lanConnected)
        {
          bar->setStyleSheet("QLabel { background-color : lime; border: 1px solid silver; }");
        }
        else
        {
          bar->setStyleSheet("QLabel { background-color : #e6e6e6; border: 1px solid silver; }");
        }
      }
      auto wifiConnection = map.find("wifi_state");
      if (wifiConnection != map.cend())
      {
        QLabel* bar = pingBarWLAN;
        if (map["wifi_state"] == "COMPLETED")
        {
          bar->setStyleSheet("QLabel { background-color : lime; border: 1px solid silver; }");
          wifiOverMap = true;
          wifiOverTeamPack = false;
        }
        else if (map["wifi_state"] == "SCANNING")
        {
          bar->setStyleSheet("QLabel { background-color : yellow; border: 1px solid silver; }");
          wifiOverMap = true;
          wifiOverTeamPack = false;
        }
        else
        {
          if (!wifiOverTeamPack)
          {
            bar->setStyleSheet("QLabel { background-color : #e6e6e6; border: 1px solid silver; }");
            wifiOverMap = false;
          }
        }
        bar->setText(fromString(map["wifi_name"]));
      }
    }
  }
}

void RobotView::setPower(std::map<std::string, std::string> map)
{
  if (robot)
  {
    auto iter = map.find(robot->name);
    if (iter != map.cend())
    {
      std::string battery_charge = map["battery_charge"];
      std::string battery_status = map["battery_status"];

      std::stringstream stream;
      float charge;
      stream << battery_charge;
      stream >> charge;

      int value = static_cast<int>(charge * 100.f);
      //bool powerPlugged = false;
      std::stringstream stream2;
      float plugged;
      stream2 << battery_status;
      stream2 >> plugged;
      bool powerPlugged = !(static_cast<short>(plugged) & 0b100000);
      powerBar->setValue(value);

      if (powerPlugged)
        powerBar->setStyleSheet("QProgressBar::chunk { background-color: lime; border: 1px solid silver; }");
      else
        powerBar->setStyleSheet("QProgressBar::chunk { background-color: red; border: 1px solid silver; }");
    }
  }
}

void RobotView::setMap(std::map<std::string, std::string> map)
{
  if (robot)
  {
    auto iter = map.find(robot->name);
    bool bodyNameFound = false;
    if (iter != map.cend())
    {
      sensorMap = map;
      for (auto it = Session::getInstance().robotsByName.cbegin(), end = Session::getInstance().robotsByName.cend(); it != end; ++it)
      {
        if (it->second->bodyId == sensorMap["bodyId"])
        {
          bodyName->setText(fromString(it->first));
          bodyNameFound = true;
        }
      }

      std::stringstream stream;
      int bhumanState;
      stream << sensorMap["state"];
      stream >> bhumanState;

      if (playerNumber)
      {
        switch (bhumanState)
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

      if (!bodyNameFound)
      {
        bodyName->setText("");
      }
    }
  }
}

void RobotView::mouseMoveEvent(QMouseEvent* me)
{
  if(!robot)
    return;
  QDrag* d = new QDrag(this);
  QPixmap pm = QPixmap::grabWidget(this, rect());
  d->setPixmap(pm);
  d->setHotSpot(me->pos());
  QMimeData* data = new QMimeData();
  data->setText(fromString(robot->name));
  d->setMimeData(data);
  d->exec(Qt::MoveAction);
  me->accept();
}

void RobotView::dragEnterEvent(QDragEnterEvent* e)
{
  if(e->source() && e->source() != this && e->source()->inherits("RobotView"))
    e->acceptProposedAction();
}

void RobotView::dropEvent(QDropEvent* e)
{
  e->accept();
  QString robotName = e->mimeData()->text();
  RobotConfigDorsh* r = Session::getInstance().robotsByName[toString(robotName)];
  RobotView* source = dynamic_cast<RobotView*>(e->source());
  if(source->playerNumber)
  {
    bool selected = source->isSelected();
    if(source->robot)
      source->setSelected(false);
    if(robot)
      source->setRobot(robot);
    else
      source->setRobot(0);
    source->setSelected(selected);
    source->update();
    setRobot(r);
    update();
  }
  else
  {
    // setSelected crashes if you drop a robot from robotpool to robotpool
    //bool selected = isSelected();
    setSelected(false);
    setRobot(r);
    //setSelected(selected);
    update();
    source->setRobot(r);
  }
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
    QAction* action4 = myMenu.addAction("show SensorReader");//action for opening Simulator: see showSensorReader()
    connect(action1, SIGNAL(triggered()), this, SLOT(Cable()));
    connect(action2, SIGNAL(triggered()), this, SLOT(WIFI()));
    connect(action4, SIGNAL(triggered()), this, SLOT(sSR()));
    myMenu.exec(statusWidget->mapToGlobal(pos));
  }
}

void RobotView::openTerm(bool wlan)
{
  char cCurrentPath[FILENAME_MAX];
  GetCurrentDir(cCurrentPath, sizeof(cCurrentPath));
  cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */
  std::string path(cCurrentPath);

  printf("The current working directory is %s", cCurrentPath);

  std::string command = connectCommand(wlan ? robot->wlan : robot->lan);

#ifdef WINDOWS
  command = "start " + command;  // add "start " to be able to focus the dorsh window
  system(command.c_str());

#elif OSX
  // Use AppleScript to open a new terminal window executing the command:
  command = "/usr/bin/osascript -e 'tell application \"Terminal\" to do script \"" + command + "\"'";
  system(command.c_str());
#else
  //linux stuff
  command = "x-terminal-emulator -e '" + command + " &'";   // & to start in background and be able to focus the dorsh window
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