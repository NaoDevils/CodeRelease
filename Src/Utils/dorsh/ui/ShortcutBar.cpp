#include "ui/ShortcutBar.h"
#include "ui/Console.h"
#include "cmdlib/Commands.h"
#include "Session.h"

#include <QIcon>
#include <QAction>
#include <QTimer>

ShortcutBar::ShortcutBar(Console* console) : QToolBar("Shortcuts"), console(console) {}

QAction* ShortcutBar::addShortcut(const QString& name, const QString& command, const bool bold)
{
  QAction* a = new QAction(name, this);
  a->setToolTip(name);
  a->setWhatsThis(command);
  a->setData(command);
  if (bold)
  {
    QFont font;
    font.setBold(true);
    font.setPixelSize(20);
    a->setFont(font);
  }

  addAction(a);
  connect(a, SIGNAL(triggered()), this, SLOT(actionTriggered()));

  //if (bold) this->setStyleSheet("font-weight: bold;");
  return a;
}

void ShortcutBar::actionTriggered()
{
  this->setEnabled(false);
  QObject* s = sender();
  if (s && s->inherits("QAction"))
  {
    QAction* a = dynamic_cast<QAction*>(s);
    QVariant data = a->data();
    if (data.canConvert<QString>())
    {
      QString command = data.toString();
      if (command.length() > 0)
      {
        console->fireCommand(command);
      }
    }
  }
  QTimer::singleShot(750, this, SLOT(reenable()));
}

void ShortcutBar::reenable()
{
  this->setEnabled(true);
}
