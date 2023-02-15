/**
 * @file Controller/StdInConsole.cpp
 *
 * This file declares the class StdInConsole.
 *
 * @author Aaron Larisch
 */

#pragma once

#include <QObject>
#ifndef WINDOWS
#include <QThread>
#include <QSocketNotifier>
#include <iostream>
#endif

class ConsoleRoboCupCtrl;

class StdInConsole : QObject
{
  Q_OBJECT;

public:
  StdInConsole(ConsoleRoboCupCtrl* console);

#ifndef WINDOWS
  ~StdInConsole();

Q_SIGNALS:
  void finishedGetLine(const QString& strNewLine);

private:
  ConsoleRoboCupCtrl* console;
  QSocketNotifier* notifier;

private Q_SLOTS:
  void on_finishedGetLine(const QString& strNewLine);
  void on_readLine();

private:
  QThread thread;
#endif
};
