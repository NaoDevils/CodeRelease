/**
 * @file Controller/StdInConsole.cpp
 *
 * This file implements the class StdInConsole.
 *
 * @author Aaron Larisch
 */

#include "StdInConsole.h"
#ifdef WINDOWS
StdInConsole::StdInConsole(ConsoleRoboCupCtrl* console) {}
#else
#include "ConsoleRoboCupCtrl.h"
#include <string>

StdInConsole::StdInConsole(ConsoleRoboCupCtrl* console) : console(console)
{
  connect(this, SIGNAL(finishedGetLine(QString)), this, SLOT(on_finishedGetLine(QString)), Qt::QueuedConnection);

  notifier = new QSocketNotifier(fileno(stdin), QSocketNotifier::Read);
  notifier->moveToThread(&thread);

  connect(&thread, SIGNAL(finished()), notifier, SLOT(deleteLater()));
  connect(notifier, SIGNAL(activated(int)), this, SLOT(on_readLine()), Qt::ConnectionType::DirectConnection);

  thread.start();
}

void StdInConsole::on_finishedGetLine(const QString& strNewLine)
{
  console->executeConsoleCommand(QString("%1").arg(strNewLine).toUtf8().data());
}

void StdInConsole::on_readLine()
{
  std::string line;
  std::getline(std::cin, line);
  QString strLine{line.c_str()};
  Q_EMIT this->finishedGetLine(strLine);

  if (!std::cin.good())
    disconnect(notifier, SIGNAL(activated(int)), this, SLOT(on_readLine()));
}

StdInConsole::~StdInConsole()
{
  thread.quit();
  thread.wait();
}
#endif
