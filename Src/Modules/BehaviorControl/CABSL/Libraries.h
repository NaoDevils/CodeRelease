/**
 * The file declares a class that instantiates all libraries.
 * @author Thomas Röfer
 */

#pragma once

#include "LibraryBase.h"
#include "Tools/ProcessFramework/CycleLocal.h"

namespace NDBehavior
{
#include "Libraries/LibTactic.h"

  class Libraries : public BehaviorBase
  {
  private:
    static CycleLocal<Libraries*> theInstance;
    std::vector<LibraryBase*> libraries; /**< All the member libraries of this class. */

  public:
    LibTactic libTactic; /**< Contains calculations for tactics like KickoffDefense */

    Libraries(const BehaviorControlBase& base, BehaviorOutput& behaviorOutput);
    ~Libraries();

    /**
     * Assignment operator, because the standard operator is not accepted by the compiler.
     * @param other The instance that is cloned.
     */
    void operator=(const Libraries& other);

    /** Calls the preProcess() method of each member library */
    void preProcessLibraries();

    /** Calls the postProcess() method of each member library */
    void postProcessLibraries();

    friend class LibraryBase;
  };
} // namespace NDBehavior
