#include "ButtonToolBar.h"
#include "ConsoleRoboCupCtrl.h"
#include "Platform/SystemCall.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/Infrastructure/JointAngles.h"

#include <QTimer>

QMenu* ButtonToolBar::createUserMenu() const
{
  QMenu* menu = new QMenu(tr("NaoDevils"));
  QAction* standAct = new QAction(QIcon(":/Icons/stand.png"), tr("&MotionRequest stand"), menu);
  QAction* sitDownAct = new QAction(QIcon(":/Icons/sitDown.png"), tr("&MotionRequest sitDown"), menu);
  QAction* headAngleAct = new QAction(QIcon(":/Icons/headAngle.png"), tr("Move Head &Freely"), menu);
  headAngleAct->setCheckable(true);
  headAngleAct->setChecked(false);

  QAction* pressChestButtonAct = new QAction(QIcon(":/Icons/chestButton.png"), tr("&Chest Button - Press and Release"), menu);
  pressChestButtonAct->setCheckable(true);
  pressChestButtonAct->setChecked(false);
  QAction* pressFrontHeadButtonAct = new QAction(QIcon(":/Icons/headFront.png"), tr("&Front Head Button - Press and Release"), menu);
  pressFrontHeadButtonAct->setCheckable(true);
  pressFrontHeadButtonAct->setChecked(false);
  QAction* pressMiddleHeadButtonAct = new QAction(QIcon(":/Icons/headMiddle.png"), tr("&Middle Head Button - Press and Release"), menu);
  pressMiddleHeadButtonAct->setCheckable(true);
  pressMiddleHeadButtonAct->setChecked(false);
  QAction* pressRearHeadButtonAct = new QAction(QIcon(":/Icons/headRear.png"), tr("&Back Head Button - Press and Release"), menu);
  pressRearHeadButtonAct->setCheckable(true);
  pressRearHeadButtonAct->setChecked(false);

  connect(standAct, SIGNAL(triggered()), this, SLOT(stand()));
  connect(sitDownAct, SIGNAL(triggered()), this, SLOT(sitDown()));
  connect(headAngleAct, SIGNAL(toggled(bool)), this, SLOT(headAngle(bool)));

  connect(pressChestButtonAct, SIGNAL(toggled(bool)), this, SLOT(toggleChestButton(bool)));
  connect(pressChestButtonAct,
      &QAction::toggled,
      this,
      [=](bool active)
      {
        if (active)
          QTimer::singleShot(1000,
              pressChestButtonAct,
              [=]
              {
                pressChestButtonAct->setChecked(false);
              });
      });
  connect(pressRearHeadButtonAct, SIGNAL(toggled(bool)), this, SLOT(toggleRearHeadButton(bool)));
  connect(pressRearHeadButtonAct,
      &QAction::toggled,
      this,
      [=](bool active)
      {
        if (active)
          QTimer::singleShot(1000,
              pressRearHeadButtonAct,
              [=]
              {
                pressRearHeadButtonAct->setChecked(false);
              });
      });
  connect(pressMiddleHeadButtonAct, SIGNAL(toggled(bool)), this, SLOT(toggleMiddleHeadButton(bool)));
  connect(pressMiddleHeadButtonAct,
      &QAction::toggled,
      this,
      [=](bool active)
      {
        if (active)
          QTimer::singleShot(1000,
              pressMiddleHeadButtonAct,
              [=]
              {
                pressMiddleHeadButtonAct->setChecked(false);
              });
      });
  connect(pressFrontHeadButtonAct, SIGNAL(toggled(bool)), this, SLOT(toggleFrontHeadButton(bool)));
  connect(pressFrontHeadButtonAct,
      &QAction::toggled,
      this,
      [=](bool active)
      {
        if (active)
          QTimer::singleShot(1000,
              pressFrontHeadButtonAct,
              [=]
              {
                pressFrontHeadButtonAct->setChecked(false);
              });
      });

  menu->addAction(standAct);
  menu->addAction(sitDownAct);
  menu->addAction(headAngleAct);
  menu->addSeparator();
  menu->addAction(pressRearHeadButtonAct);
  menu->addAction(pressMiddleHeadButtonAct);
  menu->addAction(pressFrontHeadButtonAct);
  menu->addAction(pressChestButtonAct);

  return menu;
}

void ButtonToolBar::stand()
{
  setStand();
}

void ButtonToolBar::sitDown()
{
  setSitDown();
}

void ButtonToolBar::setSitDown()
{
  MotionRequest moReq;
  moReq.motion = MotionRequest::Motion::specialAction;
  moReq.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID::sitDown;

  console.setRepresentation("MotionRequest", moReq);
}

void ButtonToolBar::setStand()
{
  MotionRequest moReq;
  moReq.motion = MotionRequest::Motion::specialAction;
  moReq.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID::stand;

  console.setRepresentation("MotionRequest", moReq);
}

void ButtonToolBar::headAngle(bool active)
{
  if (active)
  {
    HeadAngleRequest hareq;
    hareq.pan = JointAngles::off;
    hareq.tilt = JointAngles::off;
    hareq.speed = 150_deg;
    console.setRepresentation("HeadAngleRequest", hareq);
  }
  else
    console.executeConsoleCommand("set representation:HeadAngleRequest unchanged");
}

void ButtonToolBar::toggleButton(int pressedKey, bool active)
{
  if (active)
    pressButton(pressedKey);
  else
    releaseButton(pressedKey);
}

void ButtonToolBar::toggleChestButton(bool active)
{
  toggleButton(KeyStates::chest, active);
}

void ButtonToolBar::toggleFrontHeadButton(bool active)
{
  toggleButton(KeyStates::headFront, active);
}

void ButtonToolBar::toggleMiddleHeadButton(bool active)
{
  toggleButton(KeyStates::headMiddle, active);
}

void ButtonToolBar::toggleRearHeadButton(bool active)
{
  toggleButton(KeyStates::headRear, active);
}

void ButtonToolBar::pressButton(int pressedKey)
{
  keyState.pressed[pressedKey] = true;
  console.setRepresentation("KeyStates", keyState);
  QTimer::singleShot(1000,
      this,
      [=, this]
      {
        releaseButton(pressedKey);
      });
}

void ButtonToolBar::releaseButton(int pressedKey)
{
  keyState.pressed[pressedKey] = false;
  console.setRepresentation("KeyStates", keyState);

  if (std::all_of(keyState.pressed.begin(),
          keyState.pressed.end(),
          [](const bool v)
          {
            return !v;
          }))
  {
    QTimer::singleShot(5, this, SLOT(unchangeButtons()));
  }
}

void ButtonToolBar::unchangeButtons()
{
  console.executeConsoleCommand("set representation:KeyStates unchanged");
}
