#pragma once

class CFourier
{
public:
	unsigned long int fundamental_frequency;
	float *vector;
	CFourier(void);
	~CFourier(void);
	// FFT 1D
	void ComplexFFT(float data[], unsigned long number_of_samples, unsigned int sample_rate, int sign);
};
