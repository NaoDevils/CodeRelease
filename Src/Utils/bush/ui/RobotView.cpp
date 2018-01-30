#include <QDrag>
#include <QDropEvent>
#include <QDragEnterEvent>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMimeData>
#include <QMouseEvent>
#include <QPalette>
#include <QProgressBar>
#include <QMenu>
#include <QAction>


#include "Utils/bush/models/Robot.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/ui/RobotView.h"
#include "Utils/bush/ui/TeamSelector.h"
#include "Utils/bush/ui/RobotPool.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/Session.h"
#include "Platform/File.h"
#include "Utils/bush/tools/Filesystem.h"


void RobotView::init()
{
  QFormLayout* layout = new QFormLayout();

  if(playerNumber)
    cPlayerNumber = new QLabel(QString("<font size=5><b>") + QString::number(playerNumber) + QString("</b></font>"));

  statusWidget = new QWidget(this);
  statusWidget->setMaximumSize(320, 160);
  this->setMaximumWidth(190);
  this->setMaximumHeight(135);
  QGridLayout* statusLayout = new QGridLayout(statusWidget);
  this->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),
    this, SLOT(ShowContextMenu(const QPoint&)));

  QLabel* pingLabelWLAN = new QLabel("<font size=2><b>Wlan</b></font>", statusWidget);
  pingBarWLAN = new QLabel(this);
  pingBarWLAN->setMaximumSize(50, 20);
  pingBarWLAN->setAlignment(Qt::AlignCenter);
  setPings(WLAN, 0);
  statusLayout->addWidget(pingLabelWLAN, 0, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarWLAN, 0, 1);

  QLabel* pingLabelLAN = new QLabel("<font size=2><b>Lan</b></font>", statusWidget);
  pingBarLAN = new QLabel(this);
  pingBarLAN->setMaximumSize(50, 13);
  pingBarLAN->setAlignment(Qt::AlignCenter);
  setPings(LAN, 0);
  statusLayout->addWidget(pingLabelLAN, 1, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarLAN, 1, 1);

  QLabel* powerLabel = new QLabel("<font size=2><b>Power</b></font>", statusWidget);
  powerBar = new QProgressBar(this);
  powerBar->setMaximumSize(50, 13);
  powerBar->setRange(0, 100);
  powerBar->setValue(0);
  powerBar->setAlignment(Qt::AlignCenter);
  setPower(0);
  statusLayout->addWidget(powerLabel, 2, 0, Qt::AlignLeft);
  statusLayout->addWidget(powerBar, 2, 1);

  layout->addRow(cPlayerNumber, statusWidget);

  setLayout(layout);
  connect(this, SIGNAL(toggled(bool)), this, SLOT(setSelected(bool)));

  if(robot)
  {
    Session::getInstance().registerPingListener(this);
    Session::getInstance().registerPowerListener(this, robot);
  }

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
    Robot* r = robot;
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
                     Robot* robot,
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
                     Robot* robot)
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

void RobotView::setRobot(Robot* robot)
{
  if(this->robot)
  {
    Session::getInstance().removePingListener(this);
    Session::getInstance().removePowerListener(this, this->robot);
  }
  this->robot = robot;
  if(playerNumber)
  {
    Team* team = teamSelector->getSelectedTeam();
    team->changePlayer(playerNumber, pos, robot);
  }
  if(robot)
  {
    Session::getInstance().registerPingListener(this);
    Session::getInstance().registerPowerListener(this, robot);
  }
  emit robotChanged();
}

void RobotView::setPings(ENetwork network, std::map<std::string, double>* pings)
{
  QLabel* bar = network == LAN ? pingBarLAN : pingBarWLAN;
  int value = 2000;
  if(pings)
    value = static_cast<int>((*pings)[robot->name]);

  if(value >= 2000)
    bar->setStyleSheet("QLabel { background-color : #e6e6e6; border: 1px solid silver; }");
  else if(value >= 500)
    bar->setStyleSheet("QLabel { background-color : red; border: 1px solid silver; }");
  else if(value >= 250)
    bar->setStyleSheet("QLabel { background-color : yellow; border: 1px solid silver; }");
  else
    bar->setStyleSheet("QLabel { background-color : lime; border: 1px solid silver; }");

  if(value < 2000)
    bar->setText(QString::number(value) + " ms");
  else
    bar->setText(QString::number(-0, 'f', 1));

}

void RobotView::setPower(std::map<std::string, Power>* power)
{
  int value = 0;
  bool powerPlugged = false;
  bool naoqi = true;
  if (power)
  {
    if ((*power)[robot->name].isValid())
    {
      value = ((*power)[robot->name]).value;
      powerPlugged = ((*power)[robot->name]).powerPlugged;
    }

    naoqi = ((*power)[robot->name]).naoqi;
  }
  powerBar->setValue(value);

  if (naoqi)
    powerBar->setFormat("%p%");
  else
    powerBar->setFormat("NAOqi?");

  if (powerPlugged)
    powerBar->setStyleSheet("QProgressBar::chunk { background-color: lime; }");
  else
    powerBar->setStyleSheet("QProgressBar::chunk { background-color: red; }");
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
  Robot* r = Session::getInstance().robotsByName[toString(robotName)];
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
    bool selected = isSelected();
    setSelected(false);
    setRobot(r);
    setSelected(selected);
    update();
    source->setRobot(r);
  }
}

void RobotView::setSelected(bool selected)
{
  if(robot)
  {
    teamSelector->getSelectedTeam()->setSelectPlayer(robot, selected);
    cPlayerNumber->setEnabled(selected);
  }
}

void RobotView::ShowContextMenu(const QPoint& pos) // this is a slot
{
  // for most widgets
  // Rensen: removed due to unused warning
//  QPoint globalPos = statusWidget->mapToGlobal(pos);
  // for QAbstractScrollArea and derived classes you would use:
  // QPoint globalPos = myWidget->viewport()->mapToGlobal(pos);

  QMenu myMenu;
  QAction *action1 = myMenu.addAction("SSH LAN");
  QAction *action2 = myMenu.addAction("SSH WLAN");
  connect(action1, SIGNAL(triggered()), this, SLOT(Cable()));
  connect(action2, SIGNAL(triggered()), this, SLOT(WIFI()));
  myMenu.exec(statusWidget->mapToGlobal(pos));
}

void RobotView::openTerm(bool wlan)
{
  char cCurrentPath[FILENAME_MAX];
  GetCurrentDir(cCurrentPath, sizeof(cCurrentPath));
  cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */
  std::string path(cCurrentPath);

  printf("The current working directory is %s", cCurrentPath);

  std::string command;
  static std::string keyFile = Filesystem::getNaoKey();
  command = "ssh -i \"" + keyFile + "\"";
  //command = "ssh -i" + path + "\\Keys\\id_rsa_nao  nao@";

  if (wlan)

    command = command + " nao@" + robot->wlan;
  else
    command = command + " nao@" + robot->lan;

#ifdef WINDOWS
  command = "start " + command;  // add "start " to be able to focus the bush window
  system(command.c_str());

#elif OSX
  // Use AppleScript to open a new terminal window executing the command:
  command = "/usr/bin/osascript -e 'tell application \"Terminal\" to do script \"" + command + "\"'";
  system(command.c_str());
#else
  //linux stuff
  command = "x-terminal-emulator -e '" + command + "&'";   // & to start in background and be able to focus the bush window
  system(command.c_str());
#endif
}

