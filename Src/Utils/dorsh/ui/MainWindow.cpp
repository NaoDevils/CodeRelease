#include "Platform/File.h"
#include <QBoxLayout>
#include <QDesktopWidget>
#include <QFileDialog>
#include <QGridLayout>
#include <QLabel>
#include <QMenuBar>
#include <QPixmap>
#include <QSplitter>
#include "Utils/dorsh/ui/Console.h"
#include "Utils/dorsh/ui/MainWindow.h"
#include "Utils/dorsh/ui/RobotPool.h"
#include "Utils/dorsh/ui/ShortcutBar.h"
#include "Utils/dorsh/ui/TeamSelector.h"

#include "Utils/dorsh/cmdlib/ProcessRunner.h"
#include "Utils/dorsh/tools/Filesystem.h"
#include "Utils/dorsh/tools/StringTools.h"
#include "Utils/dorsh/tools/Platform.h"

MainWindow::MainWindow()
{
  setWindowTitle("dorsh");

  QSplitter* splitter(new QSplitter(Qt::Vertical));
  TeamSelector* teamSelector(new TeamSelector());
  splitter->addWidget(teamSelector);

  Console* console = new Console(teamSelector);
  splitter->addWidget(console);
  splitter->setStretchFactor(0, 0);

  ShortcutBar* shortcutBar = new ShortcutBar(console);
  addToolBar(Qt::BottomToolBarArea, shortcutBar);
  shortcutBar->addShortcut("help", "help");
  shortcutBar->addShortcut("deploy", "deploy", true);
  //shortcutBar->setItemData( 1, QColor( Qt::red ), Qt::ForegroundRole );
  shortcutBar->addShortcut("push", "pushConfig", true);
  shortcutBar->addShortcut("download logs", "downloadLogs");
  shortcutBar->addShortcut("download camera calibrations", "downloadCameraCalibrations");
  shortcutBar->addShortcut("delete logs", "deleteLogs");
  shortcutBar->addShortcut("simulator", "sim");
  shortcutBar->addShortcut("shutdown", "shutdown -s");
  shortcutBar->addShortcut("reboot", "restart robot");
  shortcutBar->addShortcut("bhuman restart", "restart bhuman");
  shortcutBar->addShortcut("bhuman ssh start", "ssh bhumand start");
  shortcutBar->addShortcut("bhuman stop", "ssh bhumand stop");
  shortcutBar->addShortcut("naoqi start", "ssh sudo /etc/init.d/naoqi start");
  shortcutBar->addShortcut("naoqi stop", "ssh sudo /etc/init.d/naoqi stop");
  shortcutBar->setContextMenuPolicy(Qt::PreventContextMenu);

  QSplitter* hSplitter(new QSplitter(Qt::Horizontal));
  hSplitter->addWidget(splitter);
  RobotPool* robotPool = new RobotPool(teamSelector);
  connect(teamSelector, SIGNAL(currentChanged(int)), robotPool, SLOT(update()));
  QFrame* rightSide = new QFrame(this);
  QGridLayout* rsLayout = new QGridLayout(rightSide);
  rsLayout->addWidget(new QLabel("<b>Robot Pool:</b>"), 0, 0);
  rsLayout->addWidget(robotPool, 1, 0);
  rightSide->setLayout(rsLayout);
  rightSide->setMinimumWidth(250);
  rightSide->setMaximumWidth(325);
  hSplitter->addWidget(rightSide);
  setCentralWidget(hSplitter);

  this->setWindowIcon(QPixmap(":icons/dorsh_terminal.png"));
  console->setFocus(Qt::OtherFocusReason);
  console->resize(60, 500);

  teamSelector->loadTeams();
  int widgetWidth = 1200;
  int widgetHeight = 695;
  splitter->setMinimumWidth(200);
  QWidget::setMinimumHeight(widgetHeight);
  QWidget::setMinimumWidth(widgetWidth);
  QWidget::resize(widgetWidth+200, widgetHeight);
  QDesktopWidget desktop;
  QPoint position((desktop.screenGeometry().width() - frameGeometry().width()) / 2, (desktop.screenGeometry().height() - frameGeometry().height()) / 2);
  QWidget::move(position);
}

void MainWindow::cleanUp()
{
#ifdef WINDOWS
  ProcessRunner scripts(fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/dorshCleanUp.cmd"));
  scripts.run();

  ProcessRunner killSwitch("taskkill /F /IM dorsh.exe");
  killSwitch.run();
#endif
}
