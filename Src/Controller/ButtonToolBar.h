/**
* @file Controller/ButtonToolBar.h
*
* Toolbar with awesome buttons.
*
* @author Arne Moos
* @author Florian Maaß
*/

#pragma once

#include <QMenu>
#include <QKeyEvent>
#include "Representations/Infrastructure/SensorData/KeyStates.h"

class ConsoleRoboCupCtrl;

class ButtonToolBar : public QObject
{
  Q_OBJECT

public:
  ButtonToolBar(ConsoleRoboCupCtrl& console) : console(console) {}

  QMenu* createUserMenu() const;

private:
  ConsoleRoboCupCtrl& console;
  KeyStates keyState;

private slots:
  void stand();
  void sitDown();
  void setStand();
  void setSitDown();
  void headAngle(bool active);

  void toggleButton(int pressedKey, bool active);
  void toggleChestButton(bool active);
  void toggleFrontHeadButton(bool active);
  void toggleMiddleHeadButton(bool active);
  void toggleRearHeadButton(bool active);

  void pressButton(int pressedKey);
  void releaseButton(int pressedKey);
  void unchangeButtons();
};
