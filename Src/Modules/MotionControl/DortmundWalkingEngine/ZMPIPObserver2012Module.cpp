/** 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de"> Oliver Urbann</a>
*/

#include "ZMPIPObserver2012Module.h"
#include "Tools/Debugging/DebugDrawings.h"
ZMPIPObserver2012Module::ZMPIPObserver2012Module():
observer(
           theControllerParams,
           theTargetCoM,
           thePatternGenRequest,
           theZMPModel,
           theWalkingInfo,
           theWalkingEngineParams,
           theActualCoM
           )
{
}

void ZMPIPObserver2012Module::update(ObservedError &theObservedError)
{
	observer.update(theObservedError);
};


MAKE_MODULE(ZMPIPObserver2012Module, dortmundWalkingEngine)