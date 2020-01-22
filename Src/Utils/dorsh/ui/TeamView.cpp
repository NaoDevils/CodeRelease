#include "Utils/dorsh/ui/TeamView.h"
#include "Utils/dorsh/models/Team.h"
#include "Utils/dorsh/ui/RobotView.h"
#include "Utils/dorsh/ui/TeamSelector.h"
#include "Utils/dorsh/tools/StringTools.h"
#include "Utils/dorsh/tools/Filesystem.h"

#include <QPushButton>
#include <QComboBox>
//#include <QColordialog> TODO: use this for color picking
#include <QLineEdit>
#include <QSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
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
    //Store Team in Session
    Session::getInstance().setTeamNumber(team);

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
    cbGameMode->addItem("demoIRF");
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
    for (size_t i = 0; i < configs.size(); ++i) 
    {
      QString configItem = fromString(configs[i]);
      if (configItem.endsWith("_5GHz")) 
      {
        configItem.chop(5);
      }
      if (cbWlanConfig->findText(configItem) == -1)
      {
        cbWlanConfig->addItem(configItem);
      }
    }
    cbWlanConfig->setCurrentIndex(cbWlanConfig->findText(fromString(team->wlanConfig)));
    wlanConfigLayout->addWidget(new QLabel("Wlan:", cbWlanConfig));
    wlanConfigLayout->addWidget(cbWlanConfig);
    settingsGrid->addLayout(wlanConfigLayout);
    connect(cbWlanConfig, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(wlanConfigChanged(const QString&)));

	QHBoxLayout * wlanFrequencyLayout = new QHBoxLayout();
	wlanFrequencyLayout->setDirection(QBoxLayout::TopToBottom);
	cbWlanFrequency = new QComboBox(this);
	cbWlanFrequency->addItem("auto");
	cbWlanFrequency->addItem("2.4 GHz");
	cbWlanFrequency->addItem("5 GHz");
	cbWlanFrequency->setCurrentIndex(cbWlanFrequency->findText(fromString(team->wlanFrequency)));
	wlanFrequencyLayout->addWidget(new QLabel("Wlan Frequency:", cbWlanFrequency));
	wlanFrequencyLayout->addWidget(cbWlanFrequency);
	settingsGrid->addLayout(wlanFrequencyLayout);
    connect(cbWlanFrequency, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(wlanFrequencyChanged(const QString&)));

    /* BUILDCONFIG */
    QHBoxLayout * buildConfigLayout = new QHBoxLayout();
    buildConfigLayout->setDirection(QBoxLayout::TopToBottom);
    cbBuildConfig = new QComboBox(this);
    cbBuildConfig->addItem("Develop");
    cbBuildConfig->addItem("DevEnC");
    cbBuildConfig->addItem("Release");
    cbBuildConfig->addItem("Debug");
    cbBuildConfig->setCurrentIndex(cbBuildConfig->findText(fromString(team->buildConfig)));
    buildConfigLayout->addWidget(new QLabel("Build:", cbBuildConfig));
    buildConfigLayout->addWidget(cbBuildConfig);
    settingsGrid->addLayout(buildConfigLayout);
    connect(cbBuildConfig, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(buildConfigChanged(const QString&)));

    /* LOGGINGCONFIG */
    QHBoxLayout * logConfigLayout = new QHBoxLayout();
    logConfigLayout->setDirection(QBoxLayout::TopToBottom);
    cbLogConfig = new QComboBox(this);
    cbLogConfig->addItem("none");
    cbLogConfig->addItem("LowFramerate");
    cbLogConfig->addItem("Sequence");
    cbLogConfig->setCurrentIndex(cbLogConfig->findText(fromString(team->logConfig)));
    // saving space
    logConfigLayout->addWidget(new QLabel("Logging:", cbLogConfig));
    logConfigLayout->addWidget(cbLogConfig);
    settingsGrid->addLayout(logConfigLayout);
    connect(cbLogConfig, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(logConfigChanged(const QString&)));
    logConfigChanged(fromString(team->logConfig));

    /* MOCAP */
    cbMocapConfig = new QCheckBox(this);
    cbMocapConfig->setChecked(team->mocapConfig);
    settingsGrid->addWidget(new QLabel("Mocap:", cbMocapConfig));
    settingsGrid->addWidget(cbMocapConfig);
    connect(cbMocapConfig, SIGNAL(clicked(bool)), this, SLOT(mocapConfigChanged(bool)));
    mocapConfigChanged(cbMocapConfig->isChecked());

    /* VOLUME */
    QVBoxLayout* volumeGrid = new QVBoxLayout();
    QHBoxLayout* speakerGrid = new QHBoxLayout();
    speakerGrid->setAlignment(Qt::AlignmentFlag::AlignLeft);
    sVolume = new QSlider(this);
    sVolume->setOrientation(Qt::Horizontal);
    sVolume->setMinimum(0);
    sVolume->setMaximum(100);
    sVolume->setTickInterval(1);
    sVolume->setValue(team->volume);
    sVolume->setMinimumWidth(25);
    sVolume->setMaximumWidth(100);
    speakerGrid->addWidget(new QLabel("Vol:", sVolume));
    speakerGrid->addWidget(sVolume, 0, Qt::AlignmentFlag::AlignLeft);
    volumeGrid->addLayout(speakerGrid);
    connect(sVolume, SIGNAL(valueChanged(int)), this, SLOT(volumeChanged(const int)));

    /* MICVOLUME */
    QHBoxLayout* micGrid = new QHBoxLayout();
    micGrid->setAlignment(Qt::AlignmentFlag::AlignLeft);
    /*sMicVolume = new QSlider(this);
    sMicVolume->setMinimum(0);
    sMicVolume->setMaximum(100);
    sMicVolume->setTickInterval(1);
    sMicVolume->setValue(team->micVolume); */
    // disabled slider for mic volume
    // use team.cfg to change it
    //sMicVolume->setEnabled(false);
    QString strMic = QString("Mic:");
    QString strMicVolume = QString("%1%").arg(team->micVolume);
    micGrid->addWidget(new QLabel(strMic));
    micGrid->addWidget(new QLabel(strMicVolume));
    //micGrid->addWidget(new QLabel(strMicVolume, sMicVolume)); // for slider
    //micGrid->addWidget(sMicVolume, 0, Qt::AlignmentFlag::AlignLeft);
    //connect(sMicVolume, SIGNAL(valueChanged(int)), this, SLOT(micVolumeChanged(const int)));
    volumeGrid->addLayout(micGrid);
    settingsGrid->addLayout(volumeGrid);

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
    cbLogConfig(0),
    cbBuildConfig(0)/*,
    handicapSlider(0)*/
{
  init();
}

void TeamView::generateRobotViews(QGridLayout* teamGrid)
{
  std::vector<std::vector<RobotConfigDorsh*> > robots = team->getPlayersPerNumber();
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

void TeamView::wlanFrequencyChanged(const QString& frequency)
{
  if (team)
    team->wlanFrequency = toString(frequency);
}

void TeamView::logConfigChanged(const QString& config)
{
  if (team)
    team->logConfig = toString(config);
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
