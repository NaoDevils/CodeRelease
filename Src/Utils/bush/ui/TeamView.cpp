#include "Utils/bush/ui/TeamView.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/ui/RobotView.h"
#include "Utils/bush/ui/TeamSelector.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/Filesystem.h"

#include <QPushButton>
#include <QComboBox>
//#include <QColordialog> TODO: use this for color picking
#include <QLineEdit>
#include <QSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QToolTip>
#include <QCursor>
#include <QColorDialog>
#include <QString>

// for file modification (robotDetector and modules.cfg)
#include <iostream>
#include "Platform/File.h"
#include <fstream>
#include <sstream>
#include <iomanip>

QString intToStylesheet(int color)
{
    std::stringstream stream;
    stream << "background-color:#" << std::setfill('0') << std::setw(6) << std::hex << (color) << ";";
    return QString(stream.str().c_str());
}

void TeamView::init()
{
  if(team)
  {
    //QFormLayout* layout = new QFormLayout();
    TeamView::layout = new QFormLayout();
    QHBoxLayout * settingsGrid = new QHBoxLayout();
    settingsGrid->setSpacing(6);
    settingsGrid->setAlignment(Qt::AlignmentFlag::AlignLeft);


    /* SAVE */
    pbSave = new QPushButton(QIcon(":icons/disk.png"), "Save");
    pbSave->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_S));
    pbSave->setToolTip("Save Team Configuration");
    settingsGrid->addWidget(pbSave);
    connect(pbSave, SIGNAL(clicked()), teamSelector, SLOT(saveTeams()));

    /* COLOR OWN PICKER */
    QHBoxLayout * colorOwnLayout = new QHBoxLayout();
    colorOwnLayout->setDirection(QBoxLayout::TopToBottom);
    openOwnColorPicker = new QPushButton("");
    openOwnColorPicker->setStyleSheet(intToStylesheet(team->colorOwn));
    openOwnColorPicker->setMaximumHeight(20);
    openOwnColorPicker->setMaximumWidth(20);

    colorOwnLayout->addWidget(new QLabel("Color:"));
    colorOwnLayout->addWidget(openOwnColorPicker);
    settingsGrid->addLayout(colorOwnLayout);

    connect(openOwnColorPicker, SIGNAL(clicked()), this, SLOT(showCPOwn()));
    //
    //settingsGrid->addWidget(cpColorOwn);
    cpColorOwn = new QColorDialog();
    cpColorOwn->setWindowTitle("Team Color");
    cpColorOwn->setCustomColor(0, 0xFFFF00); //yellow
    cpColorOwn->setCustomColor(1, 0x000000); //black
    cpColorOwn->setCustomColor(2, 0xFF00FF); //magenta
    cpColorOwn->setCustomColor(3, 0x00FFFF); //cyan
    cpColorOwn->setCustomColor(4, 0x0000FF); //blue
    cpColorOwn->setCustomColor(5, 0xCECECE); //grey
    cpColorOwn->setCurrentColor(team->colorOwn);

    /* COLOR OPP PICKER */
    QHBoxLayout * colorOppLayout = new QHBoxLayout();
    colorOppLayout->setDirection(QBoxLayout::TopToBottom);
    openOppColorPicker = new QPushButton("");
    openOppColorPicker->setStyleSheet(intToStylesheet(team->colorOpp));
    openOppColorPicker->setMaximumHeight(20);
    openOppColorPicker->setMaximumWidth(20);

    colorOppLayout->addWidget(new QLabel("Opp Color:"));
    colorOppLayout->addWidget(openOppColorPicker);
    settingsGrid->addLayout(colorOppLayout);

    connect(openOppColorPicker, SIGNAL(clicked()), this, SLOT(showCPOpp()));
    //
    //settingsGrid->addWidget(cpColorOwn);
    cpColorOpp = new QColorDialog();
    cpColorOpp->setWindowTitle("Opponent Color");
    cpColorOpp->setCustomColor(0, 0xFFFF00); //yellow
    cpColorOpp->setCustomColor(1, 0x000000); //black
    cpColorOpp->setCustomColor(2, 0xFF00FF); //magenta
    cpColorOpp->setCustomColor(3, 0x00FFFF); //cyan
    cpColorOpp->setCustomColor(4, 0x0000FF); //blue
    cpColorOpp->setCustomColor(5, 0xCECECE); //grey
    cpColorOpp->setCurrentColor(team->colorOpp);


    /* TEAMNUMBER */
    QHBoxLayout * teamnumberLayout = new QHBoxLayout();
    teamnumberLayout->setDirection(QBoxLayout::TopToBottom);
    sbNumber = new QSpinBox(this);
    sbNumber->setRange(1, 99);
    sbNumber->setButtonSymbols(QAbstractSpinBox::NoButtons);
    sbNumber->setMaximumWidth(36);
    sbNumber->setValue(team->number);
    teamnumberLayout->addWidget(new QLabel("Number:", sbNumber));
    teamnumberLayout->addWidget(sbNumber);
    settingsGrid->addLayout(teamnumberLayout);
    connect(sbNumber, SIGNAL(valueChanged(int)), this, SLOT(numberChanged(int)));

    /* LOCATION */
    QHBoxLayout * locationLayout = new QHBoxLayout();
    locationLayout->setDirection(QBoxLayout::TopToBottom);

    cbLocation = new QComboBox(this);
    std::vector<std::string> locations = Filesystem::getLocations();
    for(size_t i = 0; i < locations.size(); ++i)
      cbLocation->addItem(fromString(locations[i]));
    cbLocation->setCurrentIndex(cbLocation->findText(fromString(team->location)));
    locationLayout->addWidget(new QLabel("Location:", lePort));
    locationLayout->addWidget(cbLocation);
    settingsGrid->addLayout(locationLayout);
    connect(cbLocation, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(locationChanged(const QString&)));

    /* GAMEMODE */
    QHBoxLayout * gameModeLayout = new QHBoxLayout();
    gameModeLayout->setDirection(QBoxLayout::TopToBottom);
    cbGameMode = new QComboBox(this);
    cbGameMode->addItem("mixedTeam");
    cbGameMode->addItem("preliminary");
    cbGameMode->addItem("playOff");
    cbGameMode->addItem("penaltyShootout");
    cbGameMode->setCurrentIndex(cbGameMode->findText(fromString(team->gameMode)));
    gameModeLayout->addWidget(new QLabel("Mode:", lePort));
    gameModeLayout->addWidget(cbGameMode);
    settingsGrid->addLayout(gameModeLayout);
    connect(cbGameMode, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(gameModeChanged(const QString&)));

    /* WLANCONFIG */
    QHBoxLayout * wlanConfigLayout = new QHBoxLayout();
    wlanConfigLayout->setDirection(QBoxLayout::TopToBottom);
    cbWlanConfig = new QComboBox(this);
    std::vector<std::string> configs = Filesystem::getWlanConfigs();
    for(size_t i = 0; i < configs.size(); ++i)
      cbWlanConfig->addItem(fromString(configs[i]));
    cbWlanConfig->setCurrentIndex(cbWlanConfig->findText(fromString(team->wlanConfig)));
    wlanConfigLayout->addWidget(new QLabel("Wlan:", cbWlanConfig));
    wlanConfigLayout->addWidget(cbWlanConfig);
    settingsGrid->addLayout(wlanConfigLayout);
    connect(cbWlanConfig, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(wlanConfigChanged(const QString&)));

    /* BUILDCONFIG */
    QHBoxLayout * buildConfigLayout = new QHBoxLayout();
    buildConfigLayout->setDirection(QBoxLayout::TopToBottom);
    cbBuildConfig = new QComboBox(this);
    cbBuildConfig->addItem("Develop");
    cbBuildConfig->addItem("Release");
    cbBuildConfig->addItem("Debug");
    cbBuildConfig->setCurrentIndex(cbBuildConfig->findText(fromString(team->buildConfig)));
    buildConfigLayout->addWidget(new QLabel("Build:", cbBuildConfig));
    buildConfigLayout->addWidget(cbBuildConfig);
    settingsGrid->addLayout(buildConfigLayout);
    connect(cbBuildConfig, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(buildConfigChanged(const QString&)));

    /* WALKCONFIG */
    QHBoxLayout * walkConfigLayout = new QHBoxLayout();
    walkConfigLayout->setDirection(QBoxLayout::TopToBottom);
    cbWalkConfig = new QComboBox(this);
    cbWalkConfig->addItem("LIPM");
    cbWalkConfig->addItem("FLIPM");
    cbWalkConfig->setCurrentIndex(cbWalkConfig->findText(fromString(team->walkConfig)));
    // saving space
    walkConfigLayout->addWidget(new QLabel("Walk:", cbWalkConfig));
    walkConfigLayout->addWidget(cbWalkConfig);
    settingsGrid->addLayout(walkConfigLayout);
    connect(cbWalkConfig, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(walkConfigChanged(const QString&)));
    walkConfigChanged(fromString(team->walkConfig));

    /* MOCAP */
    cbMocapConfig = new QCheckBox(this);
    cbMocapConfig->setChecked(team->mocapConfig);
    settingsGrid->addWidget(new QLabel("Mocap:", cbMocapConfig));
    settingsGrid->addWidget(cbMocapConfig);
    connect(cbMocapConfig, SIGNAL(clicked(bool)), this, SLOT(mocapConfigChanged(bool)));
    mocapConfigChanged(cbMocapConfig->isChecked());

    /* VOLUME */
    sVolume = new QSlider(this);
    sVolume->setMinimum(0);
    sVolume->setMaximum(100);
    sVolume->setTickInterval(1);
    sVolume->setValue(team->volume);
    settingsGrid->addWidget(new QLabel("Vol:", sVolume));
    settingsGrid->addWidget(sVolume);
    connect(sVolume, SIGNAL(valueChanged(int)), this, SLOT(volumeChanged(const int)));

    /* MICVOLUME */
    sMicVolume = new QSlider(this);
    sMicVolume->setMinimum(0);
    sMicVolume->setMaximum(100);
    sMicVolume->setTickInterval(1);
    sMicVolume->setValue(team->micVolume);
    settingsGrid->addWidget(new QLabel("Mic:", sMicVolume));
    settingsGrid->addWidget(sMicVolume);
    connect(sMicVolume, SIGNAL(valueChanged(int)), this, SLOT(micVolumeChanged(const int)));

    settingsGrid->addStretch();

    /* DEVICE */
    QHBoxLayout * deviceLayout = new QHBoxLayout();
    deviceLayout->setDirection(QBoxLayout::TopToBottom);
    cbDeployDevice = new QComboBox(this);
    cbDeployDevice->addItem("auto");
    cbDeployDevice->addItem("lan");
    cbDeployDevice->addItem("wlan");
    cbDeployDevice->setCurrentIndex(cbDeployDevice->findText(fromString(team->deployDevice)));
    deviceLayout->addWidget(new QLabel("Device:", cbDeployDevice));
    deviceLayout->addWidget(cbDeployDevice);
    settingsGrid->addLayout(deviceLayout);
    connect(cbDeployDevice, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(deployDeviceChanged(const QString&)));

    layout->addRow(settingsGrid);
    QFrame* hr = new QFrame(this);
    hr->setFrameStyle(QFrame::Sunken | QFrame::HLine);
    layout->addRow(hr);

    QGridLayout* teamGrid = new QGridLayout();
    generateRobotViews(teamGrid);
    layout->addRow(teamGrid);

    setLayout(layout);
  }
}

TeamView::TeamView(TeamSelector* parent, Team* team)
  : QFrame(parent),
    teamSelector(parent),
    team(team),
    robotViews(),
    //cbColorOwn(0),
    //cbColorOpp(0),
    sbNumber(0),
    lePort(0),
    cbLocation(0),
    cbWlanConfig(0),
    cbWalkConfig(0),
    cbBuildConfig(0)/*,
    handicapSlider(0)*/
{
  init();
}

void TeamView::generateRobotViews(QGridLayout* teamGrid)
{
  std::vector<std::vector<Robot*> > robots = team->getPlayersPerNumber();
  size_t max = robots.size();
  bool backup = true;
  for(size_t j = 0; j < 2; ++j)
    for(size_t i = 0; i < max; ++i)
    {
      RobotView* rv = new RobotView(teamSelector, robots[i][j], (unsigned short) (i + 1), (unsigned short) j);
      robotViews.push_back(rv);
      teamGrid->addWidget(rv, j > 0 ? 2 : 0, (int) i);
    }
  if(backup)
  {
    QFrame* hr = new QFrame(this);
    hr->setFrameStyle(QFrame::Sunken | QFrame::HLine);
    teamGrid->addWidget(hr, 1, 0, 1, (int) max);
  }
}

void TeamView::update(size_t index)
{
  robotViews[index]->update();
}

void TeamView::colorOwnChanged(const QColor color)
{
    if (team)
    {
        team->colorOwn = color.rgb()-4278190080;
        openOwnColorPicker->setStyleSheet(intToStylesheet(team->colorOwn));
    }
}


void TeamView::colorOppChanged(const QColor color)
{
    if (team)
    {
        team->colorOpp = color.rgb()-4278190080;
        openOppColorPicker->setStyleSheet(intToStylesheet(team->colorOpp));
    }
}

void TeamView::numberChanged(int number)
{
  if(team)
  {
    team->number = (unsigned short) number;
    team->port = (unsigned short) (10000 + number);
  }
}

void TeamView::locationChanged(const QString& location)
{
  if(team)
    team->location = toString(location);
}

void TeamView::gameModeChanged(const QString& gameMode)
{
	if (team)
		team->gameMode = toString(gameMode);
}

void TeamView::wlanConfigChanged(const QString& config)
{
  if(team)
    team->wlanConfig = toString(config);
}

void TeamView::walkConfigChanged(const QString& config)
{
  if (team)
    team->walkConfig = toString(config);
}

void TeamView::mocapConfigChanged(bool checked)
{
  if (team)
    team->mocapConfig = checked;
}

void TeamView::buildConfigChanged(const QString& build)
{
  if(team)
    team->buildConfig = toString(build);
}

void TeamView::volumeChanged(const int volume)
{
  if (team)
  {
    team->volume = static_cast<unsigned short>(volume);
    QToolTip::showText(QCursor::pos(), QString::number(volume) + QString("%"));
  }
}

void TeamView::micVolumeChanged(const int volume)
{
  if (team)
  {
    team->micVolume = static_cast<unsigned short>(volume);
    QToolTip::showText(QCursor::pos(), QString::number(volume) + QString("%"));
  }
}

void TeamView::deployDeviceChanged(const QString& device)
{
  if(team)
    team->deployDevice = toString(device);
}

void TeamView::showCPOwn()
{
    cpColorOwn->open();

    connect(cpColorOwn, SIGNAL(colorSelected(QColor)), this, SLOT(colorOwnChanged(QColor)));

}

void TeamView::showCPOpp()
{
    cpColorOpp->open();

    connect(cpColorOpp, SIGNAL(colorSelected(QColor)), this, SLOT(colorOppChanged(QColor)));

}
