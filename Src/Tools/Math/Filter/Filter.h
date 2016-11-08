#pragma once

class Filter
{
public:
	Filter(void){};
	~Filter(void){};

	virtual double nextValue(double v) = 0;
};
