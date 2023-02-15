#pragma once

#include <string>

class BehaviorLogger
{
public:
  BehaviorLogger();

  void start(const std::string& name);

  void stop();

  void startInner(const std::string& name);

  void stopInner();

  void addSuccessReason(const std::string& reason);

  void addFailedReason(const std::string& reason);

  void addNote(const std::string& note);

  void clear();

  std::string get(size_t index);

  void addReason(bool suc, const std::string& reason);

private:
  std::vector<std::string> textVector;
  int currentIndex = -1;
};
