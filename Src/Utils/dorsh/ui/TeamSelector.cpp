#include <QFrame>
#include <QGridLayout>
#include <QAction>

#include "Tools/Streams/OutStreams.h"
#include "models/Team.h"
#include "models/Robot.h"
#include "ui/TeamSelector.h"
#include "ui/TeamView.h"
#include "ui/RobotView.h"

TeamSelector::TeamSelector() : teams(), teamsMap(), teamPages(), teamViews()
{
  static const size_t NUM_PLAYERS = 12; //due to F1..F12 keys and two rows with 1-6 players
  selectActions.reserve(NUM_PLAYERS);
  for (int i = 0; i < (int)NUM_PLAYERS; ++i)
  {
    QAction* a = new QAction(this);
    a->setShortcut(QKeySequence(Qt::Key_F1 + i));
    addAction(a);
    connect(a, SIGNAL(triggered()), this, SLOT(selectPlayer()));
    selectActions.push_back(a);
  }

  QAction* aNext = new QAction(this);
  aNext->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_PageDown));
  addAction(aNext);
  connect(aNext, SIGNAL(triggered()), this, SLOT(selectNext()));

  QAction* aPrev = new QAction(this);
  aPrev->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_PageUp));
  addAction(aPrev);
  connect(aPrev, SIGNAL(triggered()), this, SLOT(selectPrev()));

  QAction* aMulti = new QAction(this);
  aMulti->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_U));
  addAction(aMulti);
  connect(aMulti, SIGNAL(triggered()), this, SLOT(selectUpperRow()));

  QAction* bMulti = new QAction(this);
  bMulti->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_L));
  addAction(bMulti);
  connect(bMulti, SIGNAL(triggered()), this, SLOT(selectLowerRow()));

  QAction* reload = new QAction(this);
  reload->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
  addAction(reload);
  connect(reload, SIGNAL(triggered()), this, SLOT(reloadTeam()));

  upperRowSelected = false;
  lowerRowSelected = false;
}


void TeamSelector::addTeam(const Team& team)
{
  teams.push_back(std::make_unique<Team>(team));
  teamsMap[team.number] = teams.back().get();
  std::map<unsigned short, int>::iterator i = teamPages.find(team.number);
  TeamView* teamPage = new TeamView(this, teams.back().get());
  int index = -1;
  if (i != teamPages.end())
  {
    index = i->second;
    QWidget* oldPage = widget(index);
    removeTab(index);
    oldPage->deleteLater();
    insertTab(index, teamPage, QString::fromStdString("Team: " + team.name));
  }
  else
  {
    index = addTab(teamPage, QString::fromStdString("Team: " + team.name));
  }
  teamViews[team.number] = teamPage;
  teamPages[team.number] = index;
}

void TeamSelector::removeTeam(unsigned short number)
{
  std::map<unsigned short, int>::iterator pageIndexIter = teamPages.find(number);
  if (pageIndexIter != teamPages.end())
  {
    removeTab(pageIndexIter->second);
    teamPages.erase(pageIndexIter);
  }
  std::map<unsigned short, TeamView*>::iterator viewIndexIter = teamViews.find(number);
  if (viewIndexIter != teamViews.end())
  {
    viewIndexIter->second->deleteLater();
    teamViews.erase(viewIndexIter);
  }
  teamsMap.erase(number);
  for (auto it = teams.begin(); it != teams.end(); ++it)
  {
    if ((*it)->number == number)
      it = teams.erase(it);
  }
}

Team* TeamSelector::getSelectedTeam() const
{
  if (!teams.size())
    return 0;
  int i = currentIndex();
  return teams[i >= 0 ? i : 0].get();
}

std::vector<RobotConfigDorsh*> TeamSelector::getSelectedRobots() const
{
  Team* selectedTeam = getSelectedTeam();
  if (selectedTeam)
    return selectedTeam->getSelectedPlayers();
  else
  {
    Session::getInstance().log(CRITICAL, "TeamSelector: No team selected.");
    return std::vector<RobotConfigDorsh*>();
  }
}

void TeamSelector::loadTeams(const QString& filename, bool overwrite)
{
  if (overwrite)
  {
    size_t teamCount = teams.size();
    for (size_t i = 0; i < teamCount; ++i)
      removeTeam(teams[i]->number);
  }

  std::vector<Team> loadedTeams = Team::getTeams(filename.toStdString());
  for (size_t i = 0; i < loadedTeams.size(); ++i)
    addTeam(loadedTeams[i]);
}

void TeamSelector::saveTeams(const QString& filename)
{
  std::vector<Team> _teams;
  _teams.reserve(teams.size());
  for (size_t i = 0; i < teams.size(); ++i)
    _teams.push_back(*teams[i]);
  OutMapFile stream(filename.toStdString());
  Team::writeTeams(stream, _teams);
}

void TeamSelector::saveTeams()
{
  saveTeams("teams.cfg");
}

void TeamSelector::selectPlayer()
{
  QObject* s = sender();
  if (!s)
    return;
  int number = -1;
  for (int i = 0; i < (int)selectActions.size(); ++i)
    if (selectActions[i] == s)
      number = i;

  if (number >= 0)
  {
    Team* t = getSelectedTeam();
    t->setSelectPlayer(number, !t->isPlayerSelected(number));
    teamViews[t->number]->update(number);
  }
}

void TeamSelector::multiSelectPlayer(bool upperRow)
{
  int min;
  int max;
  if (upperRow)
  {
    min = 0;
    max = 6;
  }
  else
  {
    min = 6;
    max = 12;
  }

  for (int i = min; i < max; ++i)
  {
    Team* t = getSelectedTeam();
    if (upperRow)
    {
      if (!upperRowSelected)
        t->setSelectPlayer(i, true);
      else
        t->setSelectPlayer(i, false);
    }
    else
    {
      if (!lowerRowSelected)
        t->setSelectPlayer(i, true);
      else
        t->setSelectPlayer(i, false);
    }
    teamViews[t->number]->update(i);
  }

  if (upperRow)
    upperRowSelected = !upperRowSelected;
  else
    lowerRowSelected = !lowerRowSelected;
}

void TeamSelector::selectNext()
{
  if (currentIndex() + 1 < count())
    setCurrentIndex(currentIndex() + 1);
}

void TeamSelector::selectPrev()
{
  if (currentIndex() > 0)
    setCurrentIndex(currentIndex() - 1);
}
