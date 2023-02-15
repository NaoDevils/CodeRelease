/**
 * The file implements a class that instantiates all libraries.
 * @author Thomas Röfer
 */

#include <cstring>
#include "Libraries.h"

namespace NDBehavior
{
  CycleLocal<Libraries*> Libraries::theInstance(nullptr);

  Libraries::Libraries(const BehaviorControlBase& base, BehaviorOutput& behaviorOutput) : BehaviorBase((theInstance = this, base), behaviorOutput) {}

  Libraries::~Libraries()
  {
    theInstance.reset();
  }

  void Libraries::operator=(const Libraries& other)
  {
    memcpy((void*)this, (void*)&other, sizeof(*this));
  }

  void Libraries::preProcessLibraries()
  {
    for (LibraryBase* library : libraries)
      library->preProcess();
  }

  void Libraries::postProcessLibraries()
  {
    for (LibraryBase* library : libraries)
      library->postProcess();
  }
} // namespace NDBehavior
