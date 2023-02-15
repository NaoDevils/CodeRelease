/**
 * The file implements a class that represents the blackboard containing all
 * representations used in a process.
 * @author Thomas Röfer
 */

#include "Blackboard.h"
#include "Tools/Streams/Streamable.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include <unordered_map>

#ifndef WINDOWS
#include <cxxabi.h>
#endif

/** The instance of the blackboard of the current process. */
static thread_local Blackboard* theInstance = nullptr;

/** The actual type of the map for all entries. */
class Blackboard::Entries : public std::unordered_map<std::string, Blackboard::Entry>
{
};
class Blackboard::CopyEntries : public std::unordered_map<std::string, Blackboard::CopyEntry>
{
};

Blackboard::Blackboard() : entries(*new Entries), copyEntries(*new CopyEntries)
{
  theInstance = this;
}

Blackboard::~Blackboard()
{
  ASSERT(theInstance == this);
  theInstance = 0;
  ASSERT(entries.size() == 0);
  delete &entries;
  delete &copyEntries;
}

Blackboard::Entry& Blackboard::get(const char* representation)
{
  return entries[representation];
}

const Blackboard::Entry& Blackboard::get(const char* representation) const
{
  return entries.find(representation)->second;
}

bool Blackboard::exists(const char* representation) const
{
  return entries.find(representation) != entries.end();
}

Streamable& Blackboard::operator[](const char* representation)
{
  Entry& entry = get(representation);
  ASSERT(entry.data);
  return *entry.data;
}

const Streamable& Blackboard::operator[](const char* representation) const
{
  const Entry& entry = get(representation);
  ASSERT(entry.data);
  return *entry.data;
}

void Blackboard::free(const char* representation)
{
  Entry& entry = get(representation);
  ASSERT(entry.counter > 0);
  if (--entry.counter == 0)
  {
    entries.erase(representation);
    ++version;

    std::string rep(representation);
    const size_t found = rep.rfind("-Copy");
    if (found != std::string::npos)
      rep = rep.replace(found, 5, "");
    copyEntries.erase(rep);
  }
}

Blackboard& Blackboard::getInstance()
{
  return *theInstance;
}

void Blackboard::setInstance(Blackboard& blackboard)
{
  theInstance = &blackboard;
}

void Blackboard::addCopyEntry(const char* representation)
{
  std::string rep(representation);
  Entry& entry = get(representation);

  const size_t found = rep.rfind("-Copy");
  if (found == std::string::npos)
  {
    auto copyEntry = copyEntries.find(rep);
    if (copyEntry != copyEntries.end())
      copyEntry->second.original = &*entry.data;
  }
  else
  {
    rep = rep.replace(found, 5, "");
    copyEntries[rep].copy = &*entry.data;

    auto nonCopy = entries.find(rep);
    if (nonCopy != entries.end())
      copyEntries[rep].original = &*nonCopy->second.data;
  }
}

void Blackboard::copyUsedRepresentations()
{
  for (auto& [rep, copyEntry] : copyEntries)
  {
    // if a representation is provided by default, then there is no original
    if (copyEntry.copy && copyEntry.original)
      *copyEntry.copy = *copyEntry.original;
  }
}

std::string Blackboard::demangle(const char* name)
{
#ifdef WINDOWS
  if (!strncmp(name, "struct ", 7))
    return name + 7;
  else if (!strncmp(name, "class ", 6))
    return name + 6;
  else
    return name;
#else
  char realName[1000]; // This should be big enough, so realloc is never called.
  int status;
  size_t length = sizeof(realName);
  abi::__cxa_demangle(name, realName, &length, &status);
  ASSERT(status == 0);
  return realName;
#endif
}
