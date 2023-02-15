/**
* @file SimRobot/Main.cpp
* Implementation of the main function of SimRobot
* @author Colin Graf
*/

#include <QApplication>
#include <QSurfaceFormat>
#include <QOpenGLWidget>

#ifdef WINDOWS
#include <crtdbg.h>
#endif
#include "MainWindow.h"

#ifdef MACOS
#include <QFileOpenEvent>

MainWindow* mainWindow;

class SimRobotApp : public QApplication
{
public:
  SimRobotApp(int& argc, char** argv) : QApplication(argc, argv) {}

protected:
  bool event(QEvent* ev)
  {
    if (ev->type() == QEvent::FileOpen)
    {
      mainWindow->openFile(static_cast<QFileOpenEvent*>(ev)->file());
      return true;
    }
    else
      return QApplication::event(ev);
  }
};

#define QApplication SimRobotApp

const char* _fromQString(const QString& string)
{
  static char buffer[1000];
  strncpy(buffer, string.toUtf8().constData(), sizeof(buffer));
  buffer[sizeof(buffer) - 1] = 0;
  return buffer;
}

#endif

#ifdef WINDOWS
// prefer dedicated graphic card
// see: https://stackoverflow.com/a/39047129
extern "C"
{
  __declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
  __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

int main(int argc, char* argv[])
{
#ifdef WINDOWS
  _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
  //_CrtSetBreakAlloc(91417); // Use to track down memory leaks
#endif
  QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
  QSurfaceFormat surfaceFormat;
  surfaceFormat.setSwapBehavior(QSurfaceFormat::SingleBuffer);
  surfaceFormat.setStencilBufferSize(0);
  surfaceFormat.setSamples(8);

  /* debugging:
  surfaceFormat.setProfile(QSurfaceFormat::CoreProfile);
  surfaceFormat.setOption(QSurfaceFormat::DebugContext);
  */
  QSurfaceFormat::setDefaultFormat(surfaceFormat);

  QApplication app(argc, argv);

  {
    // Qt6: Force linking with Qt6OpenGLWidget.
    // Instantiate QOpenGLWidget here to force excecution of qt_registerDefaultPlatformBackingStoreOpenGLSupport().
    // Otherwise, the MainWindow is created without OpenGL support and loading the Qt6OpenGLWidgets library
    // afterwards when loading SimRobotCore2 does not enable OpenGL based window composition for the MainWindow,
    // which is required if we want to attach the 3D Robocup widget to it. "no opengl support set"
    QOpenGLWidget widget;
  }

  MainWindow mainWindow(argc, argv);
#ifdef WINDOWS
  app.setStyle("fusion");
#elif MACOS
  ::mainWindow = &mainWindow;
  app.setStyle("macintosh");
#endif
  app.setApplicationName("SimRobot");

  // open file from commandline
  for (int i = 1; i < argc; i++)
    if (*argv[i] != '-')
    {
#ifdef MACOS
      if (strcmp(argv[i], "YES"))
      {
        mainWindow.setWindowOpacity(0);
        mainWindow.show();
        mainWindow.openFile(argv[i]);
        mainWindow.setWindowOpacity(1);
      }
#else
      mainWindow.openFile(argv[i]);
#endif
      break;
    }
  mainWindow.show();

  return app.exec();
}
