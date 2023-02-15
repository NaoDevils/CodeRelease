#pragma once

#include <QMainWindow>
#include <map>
#include <QCheckBox>
#include <string>
#include "Utils/dorsh/ui/TeamSelector.h"

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow();
  TeamSelector* teamSelector;
  std::map<std::string, QCheckBox*> checkboxes;
private slots:
  void updateTeam();
};
