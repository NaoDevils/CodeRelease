/**
* @file Platform/Semaphore.h
*
* Inclusion of platform dependent definitions for semaphore usage.
*
* @author Colin Graf
*/

#pragma once

#if defined(LINUX)
#include "Linux/Semaphore.h"
#define SEMAPHORE_INCLUDED
#endif

#ifdef MACOS
#include "macOS/Semaphore.h"
#define SEMAPHORE_INCLUDED
#endif

#ifdef WINDOWS
#include "Windows/Semaphore.h"
#define SEMAPHORE_INCLUDED
#endif

#ifndef SEMAPHORE_INCLUDED
#error Unknown platform
#endif
