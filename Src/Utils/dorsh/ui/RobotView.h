#pragma once

#include <map>
#include <QGroupBox>

#include "Utils/dorsh/models/Power.h"
#include "Utils/dorsh/Session.h"

#include <stdio.h>  /* defines FILENAME_MAX */
#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

struct RobotConfigDorsh;
class TeamSelector;
class QLabel;
class QProgressBar;

class SensorWindow;

class RobotView : public QGroupBox
{
  Q_OBJECT

  TeamSelector* teamSelector;
  SensorWindow* sensorWindow;
  RobotConfigDorsh* robot;
  unsigned short playerNumber;
  unsigned short pos;
  QLabel* cPlayerNumber;
  QLabel* bodyName;
  QWidget* statusWidget;
  QLabel* pingBarWLAN;
  QLabel* pingBarLAN;
  QLabel* currentBar;
  QProgressBar* powerBar;
  bool wifiOverMap;
  bool wifiOverTeamPack;
  std::map<std::string, std::string> sensorMap;
  void init();
public:
  void update();
  RobotView(TeamSelector* teamSelector,
            RobotConfigDorsh* robot,
            unsigned short playerNumber,
            unsigned short position);
  RobotView(TeamSelector* teamSelector,
            RobotConfigDorsh* robot);
  void setRobot(RobotConfigDorsh* robot);
  unsigned short getPlayerNumber() const { return playerNumber; }
  QString getRobotName() const;
  bool isSelected() const;
protected:
  void mouseMoveEvent(QMouseEvent* me);
  void dragEnterEvent(QDragEnterEvent* e);
  void dropEvent(QDropEvent* e);
public slots:
  void setSelected(bool selected);
  void ShowContextMenu(const QPoint & pos);
  void openTerm(bool wlan);
  void showSensorReader();
private slots:
  void setPings(std::map<std::string, std::string> map);
  void setPower(std::map<std::string, std::string> map);
  void setMap(std::map<std::string, std::string> map);
  void changeData(std::map<std::string, std::string> map) { setPower(map); setMap(map); setPings(map); };
  void changeWifi(std::string robotName, bool connected);
  void Cable() { openTerm(false); }
  void WIFI() { openTerm(true);  }
  void sSR() { showSensorReader(); }
signals:
  void robotChanged();
};
