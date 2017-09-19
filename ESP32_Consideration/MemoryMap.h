// SerialBoard.h

#ifndef SERIALBOARD_h
#define SERIALBOARD_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
#endif

#include "InitialValue.h"
#include "Filter.h"

struct Pointer
{
	Pointer(byte A, byte L);

	byte Address;
	byte Length;
};

const int dataRecieveHeaderSize = 7;
const int checkSumSize = 1;

struct PacketMap
{
	const Pointer Header   = Pointer(0x00, 2);
	const Pointer deviceID = Pointer(0x02, 1);
	const Pointer Flags    = Pointer(0x03, 1);
	const Pointer Address  = Pointer(0x04, 1);
	const Pointer Length   = Pointer(0x05, 1);
	const Pointer Count    = Pointer(0x06, 1);
};

class MemoryMap
{
private:
	byte memory[MemorySize];

	byte calcCheckSum(byte *dataArray, int start, int end);

public:
	const struct PacketMap PMap;
	const byte RomStert = 4;
	const byte RomEnd   = 127;

	const int model = 1;
	const int minInt = 0;
	const int maxInt = 4096;

	const int valueMin[AnalogNumber] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	const int valueMax[AnalogNumber] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	
	const Pointer ModelNo     = Pointer(0, 2);
	const Pointer Ver         = Pointer(2, 1);
	const Pointer ID    = Pointer(4, 1);
	const Pointer BaudRate    = Pointer(6, 1);
	const Pointer ReturnDelay = Pointer(7, 1);

	const Pointer AnalogReadPin[AnalogNumber] =
	{
		Pointer(0x08, 1),
		Pointer(0x09, 1),
		Pointer(0x0A, 1),
		Pointer(0x0B, 1),
		Pointer(0x0C, 1),
		Pointer(0x0D, 1),
	};

	const Pointer AnalogReadMin[AnalogNumber] =
	{
		Pointer(0x0E, 2),
		Pointer(0x10, 2),
		Pointer(0x12, 2),
		Pointer(0x14, 2),
		Pointer(0x16, 2),
		Pointer(0x18, 2),
	};

	const Pointer AnalogReadMax[AnalogNumber] =
	{
		Pointer(0x1A, 2),
		Pointer(0x1C, 2),
		Pointer(0x1E, 2),
		Pointer(0x20, 2),
		Pointer(0x22, 2),
		Pointer(0x24, 2),
	};

	const Pointer AnalogReadInvert[AnalogNumber] =
	{
		Pointer(0x26, 1),
		Pointer(0x27, 1),
		Pointer(0x28, 1),
		Pointer(0x29, 1),
		Pointer(0x2A, 1),
		Pointer(0x2B, 1),
	};

	const Pointer GoalPosition[AnalogNumber] =
	{
		Pointer(0x2C, 2),
		Pointer(0x2E, 2),
		Pointer(0x30, 2),
		Pointer(0x32, 2),
		Pointer(0x34, 2),
		Pointer(0x36, 2),
	};

	const Pointer GoalTorque[AnalogNumber] =
	{
		Pointer(0x38, 2),
		Pointer(0x3A, 2),
		Pointer(0x3C, 2),
		Pointer(0x3E, 2),
		Pointer(0x40, 2),
		Pointer(0x42, 2),
	};

	const Pointer AnalogReadValue[AnalogNumber] =
	{
		Pointer(0x80, 1),
		Pointer(0x81, 1),
		Pointer(0x82, 1),
		Pointer(0x83, 1),
		Pointer(0x84, 1),
		Pointer(0x85, 1),
	};

	const Pointer AnalogReadRawValue[AnalogNumber] =
	{
		Pointer(0x86, 2),
		Pointer(0x88, 2),
		Pointer(0x8A, 2),
		Pointer(0x8C, 2),
		Pointer(0x8E, 2),
		Pointer(0x90, 2),
	};

	const Pointer AnalogReadFilteredValue[AnalogNumber] =
	{
		Pointer(0x92, 2),
		Pointer(0x94, 2),
		Pointer(0x96, 2),
		Pointer(0x98, 2),
		Pointer(0x9A, 2),
		Pointer(0x9C, 2),
	};

	const Pointer CurrentPosition[AnalogNumber] =
	{
		Pointer(0x9E, 2),
		Pointer(0xA0, 2),
		Pointer(0xA2, 2),
		Pointer(0xA4, 2),
		Pointer(0xA6, 2),
		Pointer(0xA8, 2),
	};

	const Pointer CurrentTorque[AnalogNumber] =
	{
		Pointer(0xAA, 2),
		Pointer(0xAC, 2),
		Pointer(0xAE, 2),
		Pointer(0xB0, 2),
		Pointer(0xB2, 2),
		Pointer(0xB4, 2),
	};

	MemoryMap();
	void timerUpdate();
	int16_t getValue(Pointer p);
	void setValue(Pointer p, int16_t value);
	void setPacketData(byte *PacketBuffer);
	int returnPacketData(byte *PacketBuffer);
	int16_t getReturnValue(byte *PacketBuffer, const Pointer p);
	void setReturnValue(byte *PacketBuffer, const Pointer p, int16_t value);
};


#endif
