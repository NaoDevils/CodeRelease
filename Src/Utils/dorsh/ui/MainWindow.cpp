#include "Platform/File.h"
#include <QGuiApplication>
#include <QScreen>
#include <QBoxLayout>
#include <QFileDialog>
#include <QGridLayout>
#include <QLabel>
#include <QMenuBar>
#include <QPixmap>
#include <QSplitter>
#include <QCheckBox>
#include <vector>
#include "ui/Console.h"
#include "ui/MainWindow.h"
#include "ui/RobotPool.h"
#include "ui/ShortcutBar.h"

#include "cmdlib/ProcessRunner.h"
#include "tools/Filesystem.h"
#include "tools/Platform.h"
#include "models/Team.h"
#include <algorithm>

MainWindow::MainWindow()
{
  QString windowTitle = "dorsh";

  QString shellDelegate;
#ifdef WINDOWS
  shellDelegate = qEnvironmentVariable("ComSpec");
  if (shellDelegate.isEmpty())
  {
    shellDelegate = qEnvironmentVariable("SystemRoot");
    if (shellDelegate.isEmpty())
    {
      shellDelegate = QDir::toNativeSeparators(QDir::rootPath()) + QStringLiteral("Windows");
    }
    shellDelegate.push_back(QStringLiteral("\\System32\\cmd.exe"));
  }
  ProcessRunner r(shellDelegate,
      QStringList()
          << "/c"
          << "git"
          << "rev-parse"
          << "--abbrev-ref"
          << "HEAD");
#else
  shellDelegate = "/bin/sh";
  ProcessRunner r(shellDelegate,
      QStringList() << "-c"
                    << "git rev-parse --abbrev-ref HEAD");
#endif
  r.run();
  if (!r.error())
  {
    QString gitBranchName = r.getOutput();
    if (gitBranchName != "")
      windowTitle = windowTitle + " -> " + gitBranchName.trimmed();
  }
  setWindowTitle(windowTitle);

  QSplitter* splitter(new QSplitter(Qt::Vertical));
  teamSelector = new TeamSelector();
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
  shortcutBar->addShortcut("shutdown all", "shutdown all");
  shortcutBar->addShortcut("reboot", "restart robot");
  shortcutBar->addShortcut("naodevils restart", "restart naodevils");
  shortcutBar->addShortcut("naodevils start", "ssh systemctl --user start naodevils");
  shortcutBar->addShortcut("naodevils stop", "ssh systemctl --user stop naodevils");
  shortcutBar->addShortcut("base start", "ssh systemctl --user start naodevilsbase");
  shortcutBar->addShortcut("base stop", "ssh systemctl --user stop naodevilsbase");
  shortcutBar->addShortcut("clear speedLimits", "ssh \"systemctl --user stop naodevils && rm -r /home/nao/Persistent/*/speedLimits.* && systemctl --user start naodevils\"");
  shortcutBar->addShortcut("clear persistent", "ssh \"systemctl --user stop naodevils && rm -r /home/nao/Persistent/ && systemctl --user start naodevils\"");
  shortcutBar->setContextMenuPolicy(Qt::PreventContextMenu);

  QSplitter* hSplitter(new QSplitter(Qt::Horizontal));
  hSplitter->addWidget(splitter);
  RobotPool* robotPool = new RobotPool(teamSelector);
  connect(teamSelector, SIGNAL(currentChanged(int)), robotPool, SLOT(update()));
  QFrame* righterSide = new QFrame(this);
  QVBoxLayout* rserLayout = new QVBoxLayout(righterSide);
  rserLayout->addWidget(new QLabel("<b>Config Overlays:</b>"));
  std::vector<std::string> overlays = Filesystem::getOverlays();
  teamSelector->loadTeams();
  std::vector<std::string>& enabledOverlays = teamSelector->getSelectedTeam()->overlays;
  for (size_t i = 0; i < overlays.size(); ++i)
  {
    checkboxes[overlays[i]] = new QCheckBox(QString::fromStdString(overlays[i]));
    checkboxes[overlays[i]]->setChecked(std::find(enabledOverlays.begin(), enabledOverlays.end(), overlays[i]) != enabledOverlays.end());
    connect(checkboxes[overlays[i]], SIGNAL(stateChanged(int)), this, SLOT(updateTeam()));
    rserLayout->addWidget(checkboxes[overlays[i]]);
  }
  rserLayout->setAlignment(Qt::AlignTop);
  QFrame* rightSide = new QFrame(this);
  QVBoxLayout* rsLayout = new QVBoxLayout(rightSide);
  rsLayout->addWidget(new QLabel("<b>Robot Pool:</b>"));
  rsLayout->addWidget(robotPool, 1);
  rightSide->setLayout(rsLayout);
  rightSide->setMinimumWidth(250);
  rightSide->setMaximumWidth(325);
  hSplitter->addWidget(rightSide);
  hSplitter->addWidget(righterSide);
  setCentralWidget(hSplitter);

  this->setWindowIcon(QPixmap(":icons/dorsh_terminal.png"));
  console->setFocus(Qt::OtherFocusReason);
  console->resize(60, 500);

  int widgetWidth = 1200;
  int widgetHeight = 695;
  splitter->setMinimumWidth(200);
  QWidget::setMinimumHeight(widgetHeight);
  QWidget::setMinimumWidth(widgetWidth);
  QWidget::resize(widgetWidth + 400, widgetHeight);
  QRect geometry = QGuiApplication::screens().first()->geometry();
  if (frameGeometry().width() > geometry.width())
    this->resize(geometry.width(), frameGeometry().height());
  QPoint position((geometry.width() - frameGeometry().width()) / 2, (geometry.height() - frameGeometry().height()) / 2);
  QWidget::move(position);
}

void MainWindow::updateTeam()
{
  teamSelector->getSelectedTeam()->overlays.clear();
  for (auto it = checkboxes.cbegin(), end = checkboxes.cend(); it != end; ++it)
  {
    if (it->second->checkState() == Qt::Checked)
    {
      teamSelector->getSelectedTeam()->overlays.push_back(it->first);
    }
  }
}
