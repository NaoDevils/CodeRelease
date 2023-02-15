#pragma once

#include <QTabWidget>
#include <QString>
#include <map>
#include <vector>
#include <memory>

class Team;
struct RobotConfigDorsh;
class QFrame;
class QAction;
class RobotView;
class TeamView;

class TeamSelector : public QTabWidget
{
  Q_OBJECT

  std::vector<std::unique_ptr<Team>> teams;
  std::map<unsigned short, Team*> teamsMap;
  std::map<unsigned short, int> teamPages;
  std::map<unsigned short, TeamView*> teamViews;
  std::vector<QAction*> selectActions;
  void generateRobotViews(Team& team, QFrame* teamPage);

public:
  TeamSelector();
  void addTeam(const Team& team);
  void removeTeam(unsigned short number);
  Team* getSelectedTeam() const;
  std::vector<RobotConfigDorsh*> getSelectedRobots() const;
  void loadTeams(const QString& filename = "", bool overwrite = true);
  void saveTeams(const QString& filename);

private:
  bool upperRowSelected;
  bool lowerRowSelected;
private slots:
  void selectPlayer();
  void multiSelectPlayer(bool upperRow);
  void selectUpperRow() { multiSelectPlayer(true); };
  void selectLowerRow() { multiSelectPlayer(false); };
  void selectNext();
  void selectPrev();
  void saveTeams();
  void reloadTeam() { loadTeams("", true); };
};
