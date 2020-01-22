#include "Utils/dorsh/ui/ShortcutBar.h"
#include "Utils/dorsh/ui/Console.h"
#include "Utils/dorsh/cmdlib/Commands.h"
#include "Utils/dorsh/tools/StringTools.h"
#include "Utils/dorsh/Session.h"

#include <QIcon>
#include <QAction>
#include <QTimer>

ShortcutBar::ShortcutBar(Console* console)
  : QToolBar("Shortcuts"),
    console(console)
{}

QAction* ShortcutBar::addShortcut(const QString& name,
                                  const QString& command,
                                  const bool bold)
{
  QAction* a = new QAction(name, this);
  a->setToolTip(name);
  a->setWhatsThis(command);
  a->setData(command);
  if (bold) {
    QFont* font = new QFont();
    font->setBold(true);
    font->setPixelSize(20);
    a->setFont(*font);
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
  if(s && s->inherits("QAction"))
  {
    QAction* a = dynamic_cast<QAction*>(s);
    QVariant data = a->data();
    if(data.canConvert(QVariant::String))
    {
      QString command = data.toString();
      if(command.length() > 0)
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
