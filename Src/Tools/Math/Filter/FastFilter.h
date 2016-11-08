#pragma once

#include "../../DynamicRingBuffer.h"
#include "Filter.h"
#include <string>
#include <vector>

template <typename T> class FastFilter
{
public:
	FastFilter(void) { buffer=NULL; }
	FastFilter(const unsigned int size) { buffer=new DynamicRingBuffer<T>(size); }
	~FastFilter(void) { if (buffer!=NULL) delete buffer; };

	T nextValue(T v)
	{
		if (buffer==NULL)
			return 0;

		buffer->add(v);
		return buffer->getAverage();
	}

	bool createBuffer(const unsigned int size)
	{
		if (buffer!=NULL)
			delete buffer;

		buffer=new DynamicRingBuffer<T>(size);

		return true;
	}

private:
	DynamicRingBuffer<T> *buffer;
};
