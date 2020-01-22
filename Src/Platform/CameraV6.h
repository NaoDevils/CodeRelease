/**
* @file Platform/CameraV6.h
*
* Inclusion of a platform dependent camera interface.
*
* @author Colin Graf
*/

#pragma once

#ifdef TARGET_ROBOT
#include "Linux/NaoCameraV6.h"
#define CAMERA_INCLUDED
#endif
