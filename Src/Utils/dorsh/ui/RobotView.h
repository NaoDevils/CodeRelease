#pragma once

#include <map>
#include <QGroupBox>

#include "Utils/dorsh/models/Power.h"
#include "Utils/dorsh/Session.h"
#include <nlohmann/json.hpp>

#include <QFloat16> // see https://github.com/nlohmann/json/issues/2650
#include <QTimer>

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
  QProgressBar* tempBar;
  QPoint dragStartPosition;
  nlohmann::json sensorMap;
  static constexpr unsigned dataTimeout = 10000;
  QTimer dataTimer;
  void init();
  static unsigned lastDataUpdate(const nlohmann::json&);
  static unsigned lastUpdate(const nlohmann::json&);

public:
  void update();
  RobotView(TeamSelector* teamSelector, RobotConfigDorsh* robot, unsigned short playerNumber, unsigned short position);
  RobotView(TeamSelector* teamSelector, RobotConfigDorsh* robot);
  void setRobot(RobotConfigDorsh* robot);
  unsigned short getPlayerNumber() const { return playerNumber; }
  QString getRobotName() const;
  bool isSelected() const;
  void swap(RobotView& other);

protected:
  void mouseMoveEvent(QMouseEvent* me);
  void dragEnterEvent(QDragEnterEvent* e);
  void dropEvent(QDropEvent* e);
  void mousePressEvent(QMouseEvent* e);
  void mouseReleaseEvent(QMouseEvent* e);
public slots:
  void setSelected(bool selected);
  void ShowContextMenu(const QPoint& pos);
  void openTerm(bool wlan);
  void showSensorReader();
private slots:
  void setPings(const nlohmann::json& json);
  void setPower(const nlohmann::json& json);
  void setTemp(const nlohmann::json& json);
  void setMap(const nlohmann::json& json);
  void changeData(const nlohmann::json& json);
  void Cable() { openTerm(false); }
  void WIFI() { openTerm(true); }
  void sSR() { showSensorReader(); }
signals:
  void robotChanged();
};
