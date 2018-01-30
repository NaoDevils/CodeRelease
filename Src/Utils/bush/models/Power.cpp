#include "Utils/bush/models/Power.h"

Power::Power() : value(101)
{}

Power::Power(int value, bool powerPlugged, bool naoqi) : value(value), powerPlugged(powerPlugged), naoqi(naoqi)
{}

bool Power::isValid()
{
  return value >= 0 && value <= 100;
}
