#include "ui/RobotPool.h"
#include "ui/TeamSelector.h"
#include "ui/RobotView.h"
#include "models/Robot.h"
#include "models/Team.h"
#include "Session.h"

#include <QGridLayout>
#include <QDrag>
#include <QMimeData>
#include <QMouseEvent>
#include <QPainter>
#include <QApplication>
#include <map>
#include <vector>

RobotPoolDelegate::RobotPoolDelegate(RobotPool* pool) : QStyledItemDelegate(pool), robotPool(pool) {}

void RobotPoolDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
  RobotView* robotView = robotPool->robotViews[index.data().toString()];
  robotView->show();
  robotView->setGeometry(option.rect);
}

QSize RobotPoolDelegate::sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const
{
  RobotView* robotView = robotPool->robotViews[index.data().toString()];
  return robotView->sizeHint();
}

RobotPool::RobotPool(TeamSelector* teamSelector) : QListWidget(), teamSelector(teamSelector), robotViews(), toBeDeletedLater(0)
{
  QPalette palette = teamSelector->palette();
  palette.setColor(QPalette::Base, palette.color(QPalette::Window));
  setPalette(palette);
  setItemDelegate(new RobotPoolDelegate(this));
  setDefaultDropAction(Qt::MoveAction);
  setDragDropMode(QAbstractItemView::DragDrop);
  setAcceptDrops(true);
}

void RobotPool::update()
{
  // delete the old sender widget
  if (toBeDeletedLater)
  {
    toBeDeletedLater->deleteLater();
    toBeDeletedLater = 0;
  }

  /* Use the name of the robot which the robot view represents to discover
   * the sender widget since the address of the sender never matches the
   * addresses of the views.
   */
  QString senderName = "";
  if (sender() && sender()->inherits("RobotView"))
  {
    RobotView* source = dynamic_cast<RobotView*>(sender());
    senderName = source->getRobotName();
  }

  // update robotViews according to changed name
  {
    QStringList keys = robotViews.keys();
    for (const QString& name : keys)
    {
      const auto robotView = robotViews[name];

      const QString robotName = robotView->getRobotName();
      if (!robotName.isEmpty() && name != robotName)
      {
        robotViews.remove(name);
        robotViews[robotView->getRobotName()] = robotView;

        const auto items = findItems(name, Qt::MatchExactly);
        foreach (QListWidgetItem* item, items)
          item->setText(robotView->getRobotName());
      }
    }
  }

  std::vector<RobotConfigDorsh*> robotsInTeam;
  Team* team = teamSelector->getSelectedTeam();
  if (team)
    robotsInTeam = team->getPlayers();

  const auto nameEqualTo = [](const std::string& name)
  {
    return [=](const RobotConfigDorsh* config)
    {
      return config && config->name == name;
    };
  };

  // remove robots that were moved to the team
  for (auto it = robotViews.begin(); it != robotViews.end();)
  {
    const auto& view = it.value();
    const QString robotName = view->getRobotName();
    if (!robotName.isEmpty() && !std::any_of(robotsInTeam.begin(), robotsInTeam.end(), nameEqualTo(robotName.toStdString())))
    {
      ++it;
    }
    else
    {
      disconnect(view, 0, 0, 0);

      view->setParent(parentWidget());
      view->deleteLater();

      // is there a better way to delete an item by name?
      const auto items = findItems(it.key(), Qt::MatchExactly);
      foreach (QListWidgetItem* item, items)
        delete takeItem(row(item));

      it = robotViews.erase(it);
    }
  }

  // add robots that were moved to the pool
  for (const auto& [key, val] : Session::getInstance().robotsByName)
  {
    if (std::any_of(robotsInTeam.begin(), robotsInTeam.end(), nameEqualTo(key)))
      continue;

    if (!robotViews.contains(QString::fromStdString(key)))
    {
      addItem(QString::fromStdString(key));
      RobotView* view = new RobotView(teamSelector, val.get());
      view->setParent(viewport());
      view->hide();
      connect(view, SIGNAL(robotChanged()), this, SLOT(update()), Qt::DirectConnection);
      robotViews[QString::fromStdString(key)] = view;
    }
  }

  sortItems();

  // This fixes a UI bug.
  // Hides all robots after sorting and only visible widgets
  // are shown in RobotPoolDelegate::paint() again.
  for (const auto robotView : robotViews)
    robotView->hide();
}


void RobotPool::dragEnterEvent(QDragEnterEvent* e)
{
  if (e->source() && e->source() != this && e->source()->inherits("RobotView"))
  {
    const RobotView* source = dynamic_cast<RobotView*>(e->source());
    if (source->getPlayerNumber())
      e->acceptProposedAction();
  }
}

void RobotPool::dragMoveEvent(QDragMoveEvent* e)
{
  if (e->source() && e->source() != this && e->source()->inherits("RobotView"))
  {
    const RobotView* source = dynamic_cast<RobotView*>(e->source());
    if (source->getPlayerNumber())
      e->acceptProposedAction();
  }
}

void RobotPool::dropEvent(QDropEvent* e)
{
  if (e->source()->inherits("RobotView"))
  {
    RobotView* source = dynamic_cast<RobotView*>(e->source());
    if (!source->getPlayerNumber())
      return;
    e->accept();

    QString robotName = source->getRobotName();
    addItem(robotName);
    RobotView* view = new RobotView(teamSelector, nullptr);
    view->setParent(viewport());
    view->hide();
    connect(view, SIGNAL(robotChanged()), this, SLOT(update()), Qt::DirectConnection);
    robotViews[robotName] = view;

    view->swap(*source);
  }
}
