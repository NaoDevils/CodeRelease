#include "BehaviorLogger.h"

BehaviorLogger::BehaviorLogger()
{
  clear();
}

void BehaviorLogger::start(const std::string& name)
{
  std::string text = name + ": ";
  textVector.push_back(text);
  currentIndex += 1;
}

void BehaviorLogger::stop() {}

void BehaviorLogger::startInner(const std::string& name)
{
  textVector.at(currentIndex) += "[" + name + ":";
}

void BehaviorLogger::stopInner()
{
  textVector.at(currentIndex) += "] ";
}

void BehaviorLogger::addReason(const bool suc, const std::string& reason)
{
  if (suc)
    addSuccessReason(reason);
  else
    addFailedReason(reason);
}

void BehaviorLogger::addSuccessReason(const std::string& reason)
{
  textVector.at(currentIndex) += " (Suc: " + reason + ")";
}

void BehaviorLogger::addFailedReason(const std::string& reason)
{
  textVector.at(currentIndex) += " (Fail: " + reason + ")";
}

void BehaviorLogger::addNote(const std::string& note)
{
  textVector.at(currentIndex) += " (Note: " + note + ")";
}

void BehaviorLogger::clear()
{
  textVector.clear();
  currentIndex = -1;
}

std::string BehaviorLogger::get(const size_t index)
{
  if (textVector.size() <= index)
  {
    return "None";
  }
  return textVector.at(index);
}