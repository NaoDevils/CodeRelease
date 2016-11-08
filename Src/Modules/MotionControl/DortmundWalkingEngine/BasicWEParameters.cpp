/** 
* @file Modules/MotionControl/DortmundWalkingEngine/Parameters.cpp
* Loader for parameters.dat
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/

#include "Parameters.h"
#include <stdlib.h>
#include <stdio.h>

BasicWEParameters::BasicWEParameters(void)
{
	Gd = NULL;
	z_h = stepDuration = footYDistance = dt = 0;
	N = engineFreq = 0;
}

BasicWEParameters::~BasicWEParameters(void)
{
	CleanUp();
}

void BasicWEParameters::CleanUp()
{
	if (Gd!=NULL)
		delete Gd;
	Gd=NULL;
}

int BasicWEParameters::load(char *path)
{
	CleanUp();
	FILE *stream;
//	int count=0;
	char tempstr[200], *stopstring;

	stream=fopen(path, "r");
	
	fscanf(stream, "%s", tempstr);
	stepDuration = strtod(tempstr, &stopstring);

	fscanf(stream, "%s", tempstr);
	footYDistance = strtod(tempstr, &stopstring);

	fscanf(stream, "%s", tempstr);
	maxLegLength = strtod(tempstr, &stopstring);

	fscanf(stream, "%s", tempstr);
	z_h = strtod(tempstr, &stopstring);


	fscanf(stream, "%s", tempstr);
	dt = strtod(tempstr, &stopstring);
	engineFreq=(unsigned int)(1/dt);

	fscanf(stream, "%s", tempstr);
	N = (unsigned int)strtod(tempstr, &stopstring);


	fscanf(stream, "%s", tempstr);
	L[0] = strtod(tempstr, &stopstring);
	fscanf(stream, "%s", tempstr);
	L[1] = strtod(tempstr, &stopstring);
	fscanf(stream, "%s", tempstr);
	L[2] = strtod(tempstr, &stopstring);


	for (int row=0; row<3; row++)
	{
		for (int col=0; col<3; col++)
		{
			fscanf(stream, "%s", tempstr);
			A0[row][col]= strtod(tempstr, &stopstring);
		}
	}

	fscanf(stream, "%s", tempstr);
	Gi = strtod(tempstr, &stopstring);

	for (int i=0; i<3; i++)
	{
		fscanf(stream, "%s", tempstr);
		Gx[i] = strtod(tempstr, &stopstring);	
	}

	for (int i=0; i<3; i++)
	{
		fscanf(stream, "%s", tempstr);
		b0[i] = strtod(tempstr, &stopstring);	
	}

	for (int i=0; i<3; i++)
	{
		fscanf(stream, "%s", tempstr);
		c0[i] = strtod(tempstr, &stopstring);	
	}


	Gd = new double[N];

	for (unsigned int i=0; i<N; i++)
	{   
		fscanf(stream, "%s", tempstr);
		Gd[i] = strtod(tempstr, &stopstring);
	}

	
	fclose(stream);

	return 1;
}
