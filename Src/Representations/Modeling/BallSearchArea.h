#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Enum.h"

struct BallSearchArea : public Streamable
{
  private:
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(lowerX);
      STREAM(upperX);
      STREAM(lowerY);
      STREAM(upperY);
      STREAM_REGISTER_FINISH;
    }

  public:

    ENUM(GameSituation,
    { ,
      Defensive,
      Moderat,
      Offensive,
    });

    float lowerX; // the lower x bound of the search area
    float upperX; // the upper x bound of the search area
    float lowerY; // the lower y bound of the search area
    float upperY; // the upper y bound of the search area
    GameSituation gameSituation; 

    BallSearchArea& operator=(const BallSearchArea &other)
    {
      if (this == &other)
        return *this;
      lowerX = other.lowerX;
      upperX = other.upperX;
      lowerY = other.lowerY;
      upperY = other.upperY;
      return *this;
    }

    inline bool test() const
    {
      return true;
    }
};