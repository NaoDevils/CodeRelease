/**
 * @file TeamCommSocket.h
 *
 * This representation contains two std::function properties that allow to
 * send and receive team communication data.
 */

#pragma once

#include <functional>
#include <vector>
#include "Representations/Infrastructure/TeamCommData.h"
#include "Tools/Streams/Streamable.h"


struct TeamCommSocket : public Streamable
{
  std::function<bool(const TeamCommData&)> send = nullptr;
  std::function<std::vector<TeamCommData>()> receive = nullptr;

  TeamCommSocket() = default;
  TeamCommSocket(const TeamCommSocket&) = delete;
  TeamCommSocket& operator=(const TeamCommSocket&) = delete;

  virtual Streamable& operator=(const Streamable&) noexcept
  {
    // this representation is not copyable
    ASSERT(false);
    return *this;
  }

  virtual void serialize(In* in, Out* out)
  {
    // this representation is not streamable
    ASSERT(false);
  };
};
