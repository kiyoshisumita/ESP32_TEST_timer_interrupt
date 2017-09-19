#ifndef PID_H
#define PID_H

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

class PID
{
public:
	PID(float Kp, float Ki, float Kd);
	
	~PID();

	float target = 0;
	float output = 0;

	float Kp = 0;
	float Ki = 0;
	float Kd = 0;

	float dt;
	float P, I, D, preP;

	float Input(int value, float microsec);
};

#endif

