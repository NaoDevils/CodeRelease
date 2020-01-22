#include "Utils/dorsh/models/Power.h"

Power::Power() : value(101)
{}

Power::Power(int value, bool powerPlugged) : value(value), powerPlugged(powerPlugged)
{}

bool Power::isValid()
{
  return value >= 0 && value <= 100;
}
