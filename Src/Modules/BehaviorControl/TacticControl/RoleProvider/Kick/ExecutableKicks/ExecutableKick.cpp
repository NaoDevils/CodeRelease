#include "ExecutableKick.h"

std::ostream& operator<<(std::ostream& os, const ExecutableKick& e)
{
  return (os << "BestKick: "
             << ", widthSet=" << e.widthSet << ", width=" << e.width);
}