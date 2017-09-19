#ifndef FILTER_H
#define FILTER_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
#endif

class Filter
{

public:
	float LastOutput = 0;
	float a;

	Filter(float palameter);

	float Input(int value);
};

#endif
