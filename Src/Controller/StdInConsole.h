/**
 * @file Controller/StdInConsole.cpp
 *
 * This file declares the class StdInConsole.
 *
 * @author Aaron Larisch
 */

#pragma once

#include <QObject>
#include <QThread>
#include <iostream>

class ConsoleRoboCupCtrl;
#ifdef Q_OS_WIN
class QWinEventNotifier;
#else
class QSocketNotifier;
#endif

class StdInConsole : public QObject
{
  Q_OBJECT;

public:
  StdInConsole(ConsoleRoboCupCtrl* console);

  ~StdInConsole();

Q_SIGNALS:
  void finishedGetLine(const QString& strNewLine);

private:
  ConsoleRoboCupCtrl* console;
#ifdef Q_OS_WIN
  QWinEventNotifier* notifier;
#else
  QSocketNotifier* notifier;
#endif

private Q_SLOTS:
  void on_finishedGetLine(const QString& strNewLine);
  void on_readLine();

private:
  QThread thread;
};
