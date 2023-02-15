#pragma once

#include <map>
#include <string>
#include <QMainWindow>
#include <nlohmann/json_fwd.hpp>

class QWidget;

class SensorWindow : public QMainWindow
{
  Q_OBJECT
public:
  SensorWindow(const nlohmann::json& json, std::string robotName);
};
