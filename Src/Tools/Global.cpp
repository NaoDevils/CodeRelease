/**
* @file Global.cpp
* Implementation of a class that contains pointers to global data.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
*/

#include "Global.h"

thread_local Global Global::theInstance;
