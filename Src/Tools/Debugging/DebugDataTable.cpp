/**
 * @file Tools/Debugging/DebugDataTable.cpp
 * This file implements a table for generic handling of streamable debug data.
 * Representations mentioned in the table will be overwritten with the table
 * entry.
 *
 * @author Michael Spranger
 * @author Tobias Oberlies
 * @author Thomas Röfer
 */

#include "Tools/Debugging/DebugDataTable.h"
#include "Tools/MessageQueue/InMessage.h"

DebugDataTable::~DebugDataTable()
{
  for (std::unordered_map<std::string, std::atomic<char*>>::iterator iter = table.begin(); iter != table.end(); ++iter)
  {
    char* data = iter->second.load(std::memory_order_acquire);
    if (data)
      delete[] data;
  }
}

void DebugDataTable::processChangeRequest(InMessage& in)
{
  std::string name;
  char change;
  in.bin >> name >> change;
  std::unordered_map<std::string, std::atomic<char*>>::iterator iter = table.find(name);
  if (change)
  {
    int size = in.getBytesLeft();
    char* buffer = new char[size];
    in.bin.read(buffer, size);
    if (iter == table.end())
      table[name].store(buffer, std::memory_order_release);
    else
    {
      char* data = iter->second.exchange(buffer, std::memory_order_acq_rel);
      if (data)
        delete[] data;
    }
  }
  else if (iter != table.end())
  {
    char* data = iter->second.load(std::memory_order_acquire);
    if (data)
      delete[] data;
    table.erase(iter);
  }
}
