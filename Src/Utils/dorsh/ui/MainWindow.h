#pragma once

#include <QMainWindow>

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow();
  void cleanUp();
protected:
  void closeEvent(QCloseEvent *event) { cleanUp(); };
};
