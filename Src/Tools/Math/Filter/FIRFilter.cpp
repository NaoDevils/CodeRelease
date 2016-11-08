#include "FIRFilter.h"
#include <fstream>
#include <stdlib.h>

using namespace std;

FIRFilter::FIRFilter(void)
{
	static double d=1;
	buffer=0;
	setCoefficients(&d, 1);
}

double FIRFilter::nextValue(double v)
{
	buffer->add(v);
	double output=0;
	for (int i=0; i<buffer->getCapacity(); i++)
		output+=(*buffer)[i]*coefficients[i];
	return output;
}

void FIRFilter::setCoefficients(double *coefficients, int n)
{
	this->n=n;

	if (buffer!=0)
		delete buffer;
	
	buffer=new DynamicRingBuffer<double>(n);
	this->coefficients.clear();

	for (int i=0; i<n; i++)
		this->coefficients.push_back(coefficients[i]);
}

bool FIRFilter::readCoefficients(string path)
{
	bool running=false;
	string line;
	char *stopstring;

	if (buffer!=0)
		delete buffer;
	coefficients.clear();

	ifstream fcf(path.c_str());
	if (fcf.is_open())
	{
		while (!fcf.eof())
		{
			getline(fcf, line);
			if (line[0]=='%')
				continue;
			if (line.find("Numerator:", 0)!=string::npos)
			{
				running=true;
				continue;
			}

			double d=strtod(line.c_str(), &stopstring);

			if (d==0)
				running=false;
			
			if (running)
				coefficients.push_back(d);
			
		}
		fcf.close();
	}

	n=static_cast<int>(coefficients.size());

	buffer=new DynamicRingBuffer<double>(n);

	return true;
}
