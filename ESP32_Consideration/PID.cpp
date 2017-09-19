#include "PID.h"


PID::PID(float Kp, float Ki, float Kd)
{
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;
}

PID::~PID()
{

}

float PID::Input(int value, float microsec)
{
	dt = microsec / 1000000.0f;

	P = target - value;
	I += P * dt;
	D = (P - preP) / dt;

	preP = P;

	output -= Kp * P + Ki * I + Kd * D;

	constrain(output, -255, 255);
}

