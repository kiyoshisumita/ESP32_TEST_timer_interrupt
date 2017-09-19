#ifndef INITIAL_H
#define INITIAL_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
#endif

#define MODE_OSC 100
#define MODE_SERIAL 200

const int ModeComn = MODE_OSC;

const int FirmwareVer = 10;
const int DeviceID = 0;
const int BaudrateNum = 9; //230400
const int DelayForReturn = 0;

const int AnalogNumber = 6;
const int MotorNumber = 3;

const int MemorySize = 256;
const int BufferSize = 256;

const byte AssignedAnalogPin[AnalogNumber] = { 36, 39, 34, 35, 32, 33 };
const byte AssignedModePIn = 10;
const uint8_t AssignedPWMForwardPin[MotorNumber] = { 2, 18, 21 };
const uint8_t AssignedPWMBackPin[MotorNumber]    = { 4, 19, 22 };

//Timer
const int TimerSpan = 1000;
const int TimerSpan33 = 3300;
const int retry = 10;

//OSC;
//temp, will remove
//const String SSID= "techgarage";          // your network SSID (name)
//const String PASS = "killtheproject";                    // your network password
const String SSID= "exiii_Develop_g";          // your network SSID (name)
const String PASS = "8e4dcb22914af";                    // your network password

const uint32_t BaudRateList[11] =
{
  9600,
  14400,
  19200,
  28800,
  38400,
  57600,
  76800,
  115200,
  153600,
  230400,
  460800,
};

#endif
