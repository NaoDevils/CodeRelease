#pragma once

#include <map>
#include <string>
#include <QMainWindow>

class QWidget;

class SensorWindow : public QMainWindow
{
  Q_OBJECT
public:
  SensorWindow(std::map<std::string, std::string> map, std::string robotName);
};
