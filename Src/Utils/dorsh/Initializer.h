#pragma once

#include "Session.h"
#include <QScopedPointer>

class QApplication;
class MainWindow;

class Initializer
{
  LogLevel logLevel;
  QScopedPointer<QApplication> app;
  QScopedPointer<MainWindow> mainWindow;

public:
  Initializer(int& argc, char** argv);
  ~Initializer();
  int start();
  void log(LogLevel logLevel, const std::string& message);
};
