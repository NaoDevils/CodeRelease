#pragma once

#include <QFrame>
#include <QCheckBox>
#include <vector>

class Team;
class QFormLayout;
class QPushButton;
class QComboBox;
class QColorDialog;
//class QColorDialog; TODO: use this for color picking
class QLineEdit;
class QSpinBox;
class QSlider;
class QLabel;
class QGridLayout;
class TeamSelector;
class RobotView;

class TeamView : public QFrame
{
  Q_OBJECT

  TeamSelector* teamSelector;
  Team* team;

  std::vector<RobotView*> robotViews;
  QFormLayout* layout;
  QPushButton* pbSave;
  QPushButton* openOwnColorPicker;
  QPushButton* openOppColorPicker;
  QColorDialog* cpColorOwn;
  QColorDialog* cpColorOpp;
  //QComboBox* cbColorOwn;
  //QComboBox* cbColorOpp;
  QSpinBox* sbNumber;
  QLineEdit* lePort;
  QComboBox* cbLocation;
  QComboBox* cbGameMode;
  QComboBox* cbWlanConfig;
  QComboBox* cbWlanFrequency;
  QComboBox* cbLogConfig;
  QCheckBox* cbMocapConfig;
  QComboBox* cbBuildConfig;
  QComboBox* cbDeployDevice;
  QSlider* sVolume;
  QSlider* sMicVolume;

  void init();
public:
  TeamView(TeamSelector* parent, Team* team);
  void generateRobotViews(QGridLayout* teamGrid);
  void update(size_t index);

private slots:
  void showCPOwn();
  void showCPOpp();
  //void colorOwnChanged(const QString& color);
  //void colorOppChanged(const QString& color);
  void colorOwnChanged(const QColor color);
  void colorOppChanged(const QColor color);
  void numberChanged(int number);
  void locationChanged(const QString& location);
  void gameModeChanged(const QString& gameMode);
  void wlanConfigChanged(const QString& config);
  void wlanFrequencyChanged(const QString& frequency);
  void logConfigChanged(const QString& config);
  void mocapConfigChanged(bool checked);
  void buildConfigChanged(const QString& build);
  void volumeChanged(const int volume);
  void micVolumeChanged(const int volume);
  void deployDeviceChanged(const QString& device);
};
