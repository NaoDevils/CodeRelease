/**
 * @file Controller/StdInConsole.cpp
 *
 * This file implements the class StdInConsole.
 *
 * @author Aaron Larisch
 */

#include "StdInConsole.h"
#include "ConsoleRoboCupCtrl.h"
#include <string>

#ifdef Q_OS_WIN
#include <QWinEventNotifier>
#else
#include <QSocketNotifier>
#endif

StdInConsole::StdInConsole(ConsoleRoboCupCtrl* console) : console(console)
{
  connect(this, &StdInConsole::finishedGetLine, this, &StdInConsole::on_finishedGetLine, Qt::QueuedConnection);

#ifdef Q_OS_WIN
  notifier = new QWinEventNotifier(GetStdHandle(STD_INPUT_HANDLE));
#else
  notifier = new QSocketNotifier(fileno(stdin), QSocketNotifier::Read);
#endif
  notifier->moveToThread(&thread);

  connect(&thread, &QThread::finished, notifier, &QObject::deleteLater);

  connect(notifier,
#ifdef Q_OS_WIN
      &QWinEventNotifier::activated,
#else
      &QSocketNotifier::activated,
#endif
      this,
      &StdInConsole::on_readLine,
      Qt::ConnectionType::DirectConnection);

  thread.start();
}

void StdInConsole::on_finishedGetLine(const QString& strNewLine)
{
  console->executeConsoleCommand(QString("%1").arg(strNewLine).toStdString());
}

void StdInConsole::on_readLine()
{
  std::string line;
  std::getline(std::cin, line);
  Q_EMIT this->finishedGetLine(QString::fromStdString(line));

  if (!std::cin.good())
    disconnect(notifier,
#ifdef Q_OS_WIN
        &QWinEventNotifier::activated,
#else
        &QSocketNotifier::activated,
#endif
        this,
        &StdInConsole::on_readLine);
}

StdInConsole::~StdInConsole()
{
  thread.quit();
  if (!thread.wait(100))
  {
    // On Windows, readline can only be interrupted by using terminate()
    thread.terminate();
    thread.wait();
  }
}
