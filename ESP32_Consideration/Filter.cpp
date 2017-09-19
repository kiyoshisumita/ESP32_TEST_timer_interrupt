#include "Filter.h"

Filter::Filter(float palameter)
{
	a = palameter;
}

float Filter::Input(int value)
{
	LastOutput = a * LastOutput + (1 - a) * value;
	return LastOutput;
}

