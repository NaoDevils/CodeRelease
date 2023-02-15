#include "Utils/dorsh/Initializer.h"
#include "Utils/dorsh/agents/DataAgent.h"
#include "Utils/dorsh/bhwrapper/Framework.h"
#include "Utils/dorsh/cmdlib/ProcessRunner.h"
#include "Utils/dorsh/models/Robot.h"
#include "Utils/dorsh/models/Team.h"
#include "Utils/dorsh/tools/Platform.h"
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

Initializer::Initializer(int& argc, char** argv) : logLevel(WARN), app(0)
{
  log(TRACE, "Initializer: Initialization started.");

  Framework::getInstance("Initializer");
  app.reset(new QApplication(argc, argv));
#ifdef WINDOWS
  app->setStyle("fusion");
#endif
  app->setApplicationName("Dortmund User Shell (dorsh)");
  app->setCursorFlashTime(0);

  QPixmap pixmap(":/icons/dorsh_terminal_splash.png");
  QSplashScreen splash(pixmap, Qt::WindowStaysOnTopHint);
  splash.show();
  app->processEvents();

  log(TRACE, "Initializer: changing working directory to...");
  goToConfigDirectory(argv[0]);

  Icons::getInstance().init();
  log(TRACE, "Initializer: Created Qt application.");

  Session::getInstance();
  log(TRACE, "Initializer: Session instance created.");

  Session::getInstance().logLevel = WARN;
  log(TRACE, "Initializer: Set log level to " + std::to_string(Session::getInstance().logLevel) + ".");

  Session::getInstance().dataAgent.reset(new DataAgent());
  log(TRACE, "Initializer: Power agent started.");

  RobotConfigDorsh::initRobotsByName(Session::getInstance().robotsByName);
  log(TRACE, "Initializer: Robots loaded.");

  mainWindow.reset(new MainWindow);
  mainWindow->show();

  log(TRACE, "Initializer: Created main window.");

  log(TRACE, "Initializer: Finished initialization.");

  splash.close();
}

Initializer::~Initializer()
{
  log(TRACE, "Initializer: Shutdown started.");

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
  if (logLevel >= this->logLevel)
  {
    std::cout << message << std::endl;
  }
}
