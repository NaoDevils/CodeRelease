/**
 * @file DebugRequest.cpp
 * Implementation of class DebugRequest
 */

#include <cstring>
#include <cstdio>

#include "DebugRequest.h"
#include "Platform/BHAssert.h"

#ifdef WINDOWS
#include <Windows.h>
#endif

DebugRequest::DebugRequest(const std::string& description, bool enable) : description(description), enable(enable) {}

void DebugRequestTable::addRequest(const DebugRequest& debugRequest, bool force)
{
  lastName = 0;
  nameToIndex.clear();
  if (debugRequest.description == "poll")
  {
    poll = true;
    pollCounter = 0;
    alreadyPolledDebugRequestCounter = 0;
  }
  else if (debugRequest.description == "disableAll")
    removeAllRequests();
  else
  {
    for (int i = 0; i < currentNumberOfDebugRequests; i++)
    {
      if (debugRequest.description == debugRequests[i].description)
      {
        if (!debugRequest.enable && !force)
          debugRequests[i] = debugRequests[--currentNumberOfDebugRequests];
        else
          debugRequests[i] = debugRequest;
        return;
      }
    }
    if (debugRequest.enable || force)
    {
      ASSERT(currentNumberOfDebugRequests < maxNumberOfDebugRequests);
      debugRequests[currentNumberOfDebugRequests++] = debugRequest;
    }
  }
}

void DebugRequestTable::disable(const char* name)
{
  lastName = 0;
  nameToIndex.clear();
  for (int i = 0; i < currentNumberOfDebugRequests; i++)
    if (debugRequests[i].description == name)
    {
      disabledDebugRequests.insert(name);
      debugRequests[i] = debugRequests[--currentNumberOfDebugRequests];
      return;
    }
}

bool DebugRequestTable::notYetPolled(const char* name)
{
  for (int i = 0; i < alreadyPolledDebugRequestCounter; ++i)
    if (strcmp(name, alreadyPolledDebugRequests[i]) == 0)
      return false;
  alreadyPolledDebugRequests[alreadyPolledDebugRequestCounter++] = name;
  return true;
}

void DebugRequestTable::propagateDisabledRequests(DebugRequestTable& drt)
{
  for (const char* dr : disabledDebugRequests)
    drt.disable(dr);
}

void DebugRequestTable::clearDisabledRequests()
{
  disabledDebugRequests.clear();
}

In& operator>>(In& stream, DebugRequest& debugRequest)
{
  return stream >> debugRequest.enable >> debugRequest.description;
}

Out& operator<<(Out& stream, const DebugRequest& debugRequest)
{
  return stream << debugRequest.enable << debugRequest.description;
}

void DebugRequestTable::print(const char* message)
{
#ifdef WINDOWS
  char s[256];
  snprintf(s, sizeof(s), "%.254s\n", message);
  OutputDebugString(s);
#else
  fprintf(stderr, "%s\n", message);
#endif
}
