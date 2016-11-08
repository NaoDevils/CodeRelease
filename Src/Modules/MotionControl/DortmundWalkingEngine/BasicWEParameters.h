/** 
* @file Modules/MotionControl/DortmundWalkingEngine/Parameters.h
* Loader for parameters.dat
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/

#pragma once

class BasicWEParameters
{
public:
	/** 
	* Constructor.
	*/
  BasicWEParameters(void);
public:
	/** destructor */
	~BasicWEParameters(void);

	/** 
	* Loads the specified parameters.dat
	* @param path Path to the file
	* @return Errorcode
	*/
	int load(char *path);

	double dt; /**< Duration of one frame */
	double z_h;	/**< desired CoM position (height) */
	unsigned int N; /**< Length of the preview phase */
	double *Gd; /**< Data for preview controller */
	double A0[3][3]; /**< Data for preview controller */
	double Gi; /**< Data for preview controller */
	double Gx[3]; /**< Data for preview controller */
	double b0[3]; /**< Data for preview controller */
	double c0[3]; /**< Data for preview controller */
	double L[3]; /**< Data for preview controller */

	double stepDuration; /**< Duration in s for one walk phase */
	double footYDistance; /**< Distance between feet */
	double maxLegLength; /**< The maximum leg length */
	unsigned int engineFreq; /* 1/dt */

protected:
	void CleanUp();
};
