#pragma once

#include <string>
#include <map>
#include <vector>
#include <QColor>
#include "Tools/Streams/Streamable.h"

struct RobotConfigDorsh;

class Team : public Streamable
{
  std::vector<std::vector<RobotConfigDorsh*> > players;
  std::map<RobotConfigDorsh*, bool> selectedPlayers;

  void serialize(In*, Out*);

  void init();
public:
  std::string name;
  unsigned short number;
  unsigned short port;
  //std::string colorOwn;
  //std::string colorOpp;
  int colorOwn;
  int colorOpp;
  std::string location;
  std::string gameMode;
  std::string wlanConfig;
  std::string wlanFrequency;
  std::string logConfig;
  bool mocapConfig;
  std::string buildConfig;
  unsigned short volume;
  unsigned short micVolume;
  std::string deployDevice;

  Team();
  Team(const std::string& name, unsigned short number);
  void addPlayer(unsigned int playerNumber, bool  substitutePlayer, RobotConfigDorsh& robot);
  std::vector<std::vector<RobotConfigDorsh*> > getPlayersPerNumber() const;
  std::vector<RobotConfigDorsh*> getPlayers() const;
  std::vector<RobotConfigDorsh*> getPlayersWrapped() const;
  unsigned short getPlayerNumber(const RobotConfigDorsh& robot) const;
  static std::vector<Team> getTeams(const std::string& filename = "");

  void changePlayer(unsigned short number, unsigned short pos, RobotConfigDorsh* robot);
  void setSelectPlayer(RobotConfigDorsh* robot, bool select);
  void setSelectPlayer(size_t index, bool select);
  bool isPlayerSelected(RobotConfigDorsh* robot);
  bool isPlayerSelected(size_t index);
  std::vector<RobotConfigDorsh*> getSelectedPlayers() const;

  static void writeTeams(Out& stream, const std::vector<Team>& teams);
  static void readTeams(In& stream, std::vector<Team>& teams);
};
