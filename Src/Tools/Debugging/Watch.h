#pragma once

#include "Platform/BHAssert.h"

#define WATCH(x) ASSERT(x==x);
#define WATCH_ARRAY(a) for (unsigned int i=0; i<sizeof(a)/sizeof(a[0]); i++) ASSERT(a[i]==a[i]);

class Watch
{
public:
	virtual void watch(){};
};
