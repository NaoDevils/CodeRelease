/**
* @file Global.cpp
* Implementation of a class that contains pointers to global data.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "Global.h"

thread_local AnnotationManager* Global::theAnnotationManager = 0;
thread_local OutMessage* Global::theDebugOut = 0;
thread_local Settings* Global::theSettings = 0;
thread_local DebugRequestTable* Global::theDebugRequestTable = 0;
thread_local DebugDataTable* Global::theDebugDataTable = 0;
thread_local StreamHandler* Global::theStreamHandler = 0;
thread_local DrawingManager* Global::theDrawingManager = 0;
thread_local DrawingManager3D* Global::theDrawingManager3D = 0;
thread_local TimingManager* Global::theTimingManager = 0;
