#include "Utils/dorsh/Initializer.h"
#include "Utils/dorsh/agents/DataAgent.h"
#include "Utils/dorsh/bhwrapper/Framework.h"
#include "Utils/dorsh/cmdlib/ProcessRunner.h"
#include "Utils/dorsh/models/Robot.h"
#include "Utils/dorsh/models/Team.h"
#include "Utils/dorsh/tools/Platform.h"
#include "Utils/dorsh/tools/StringTools.h"
#include "Utils/dorsh/ui/Console.h"

#include <iostream>
#include <cstdlib>

#include <QApplication>
#include <QStyleFactory>
#include <QSplashScreen>
#include <thread>
#include <chrono>
#include "Utils/dorsh/ui/MainWindow.h"

#include <QThread>

class SplashThread :public QThread
{
protected:
  void run(void)
  {
    QThread::msleep(5000);
  }
};

Initializer::Initializer(int &argc, char** argv) : logLevel(WARN), app(0)
{
  log(TRACE, "Initializer: Initialization started.");

  log(TRACE, "Initializer: changing working directory to...");
  goToConfigDirectory(argv[0]);

  Framework::getInstance("Initializer");
  app = new QApplication(argc, argv);
#ifdef OSX
  //app->setStyle("macintosh");
  app->setStyle("cleanlooks");
#endif
  app->setApplicationName("Dortmund User Shell (dorsh)");
  app->setCursorFlashTime(0);
  Icons::getInstance().init();
  log(TRACE, "Initializer: Created Qt application.");

  Session::getInstance();
  log(TRACE, "Initializer: Session instance created.");

  Session::getInstance().logLevel = WARN;
  log(TRACE, "Initializer: Set log level to " + toString(Session::getInstance().logLevel) + ".");

  Session::getInstance().dataAgent = new DataAgent();
  log(TRACE, "Initializer: Power agent started.");

  RobotConfigDorsh::initRobotsByName(Session::getInstance().robotsByName);
  log(TRACE, "Initializer: Robots loaded.");

  Session::getInstance().dataAgent->initialize(Session::getInstance().robotsByName);
  log(TRACE, "Initializer: Registered robots at power agent.");

  mainWindow = new MainWindow;
  mainWindow->show();

  log(TRACE, "Initializer: Created main window.");

  log(TRACE, "Initializer: Finished initialization.");

  QPixmap pixmap(":/icons/dorsh_terminal_splash.png");
  QSplashScreen splash(pixmap, Qt::WindowStaysOnTopHint);
  splash.show();
  app->processEvents();
  QEventLoop loop;
  SplashThread* thread = new SplashThread();
  QObject::connect(thread, SIGNAL(finished()), &loop, SLOT(quit()));
  QObject::connect(thread, SIGNAL(terminated()), &loop, SLOT(quit()));
  thread->start();
  loop.exec();
  splash.finish(mainWindow);
}

Initializer::~Initializer()
{
  log(TRACE, "Initializer: Shutdown started.");

  mainWindow->deleteLater();
  app->deleteLater();
  log(TRACE, "Initializer: Deleted GUI.");

  Session::getInstance().console = 0;
  log(TRACE, "Initializer: Removed console.");

  Framework::destroy("Initializer");

  log(TRACE, "Initializer: Finished shutdown.");
}

int Initializer::start()
{
  return app->exec();
}

void Initializer::log(LogLevel logLevel, const std::string& message)
{
  if(logLevel >= this->logLevel)
  {
    std::cout << message << std::endl;
  }
}
