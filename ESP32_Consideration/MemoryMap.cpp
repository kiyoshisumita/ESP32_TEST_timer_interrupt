// 
// 
// 

#include "MemoryMap.h"

Pointer::Pointer(byte A, byte L)
{
	Address = A;
	Length = L;
};

Filter AnalogFilter[AnalogNumber] =
{
  Filter(0.8),
  Filter(0.8),
  Filter(0.8),
  Filter(0.8),
  Filter(0.8),
  Filter(0.8),
};

MemoryMap::MemoryMap() {
	for (int i = 0; i < MemorySize; i++) {
		memory[i] = 0x00;
	}
	
	setValue(ModelNo, model);
	setValue(Ver, FirmwareVer);
	setValue(ID, DeviceID);
	setValue(BaudRate, BaudrateNum);
	setValue(ReturnDelay, DelayForReturn);

	for (int i = 0; i < AnalogNumber; i++)
	{
		setValue(AnalogReadPin[i], AssignedAnalogPin[i]);
		setValue(AnalogReadMin[i], minInt);
		setValue(AnalogReadMax[i], maxInt);
		setValue(AnalogReadInvert[i], 0);
		setValue(GoalPosition[i], minInt);
		setValue(GoalTorque[i], minInt);
		setValue(AnalogReadValue[i], 0);
		setValue(AnalogReadRawValue[i], minInt);
		setValue(AnalogReadFilteredValue[i], minInt);
		setValue(CurrentPosition[i], minInt);
		setValue(CurrentTorque[i], minInt);
	}
}

void MemoryMap::timerUpdate()
{
    for (int i = 0; i < AnalogNumber; i++)
    {
      int rawPos = analogRead(getValue(AnalogReadPin[i]));
      int fixPos = constrain(rawPos, getValue(AnalogReadMin[i]), getValue(AnalogReadMax[i]));
      int mapPos;

      if (getValue(AnalogReadInvert[i]) == 0)
      {
        mapPos = map(fixPos, getValue(AnalogReadMin[i]), getValue(AnalogReadMax[i]), valueMin[i], valueMax[i]);
      }
      else
      {
        mapPos = map(fixPos, getValue(AnalogReadMin[i]), getValue(AnalogReadMax[i]), valueMax[i], valueMin[i]);
      }

      AnalogFilter[i].Input(rawPos);

      setValue(AnalogReadRawValue[i], rawPos);
      setValue(AnalogReadValue[i], mapPos);
      setValue(AnalogReadFilteredValue[i], AnalogFilter[i].LastOutput);
    }
}

int16_t MemoryMap::getValue(const Pointer p)
{
	if (p.Length == 1)
	{
		return memory[p.Address];
	}
	else if (p.Length == 2)
	{
		return makeWord(memory[p.Address + 1], memory[p.Address]);
	}
}

int16_t MemoryMap::getReturnValue(byte *PacketBuffer, const Pointer p)
{
	if (p.Length == 1)
	{
		return PacketBuffer[p.Address];
	}
	else if (p.Length == 2)
	{
		return makeWord(PacketBuffer[p.Address + 1], PacketBuffer[p.Address]);
	}
}

void MemoryMap::setValue(const Pointer p, int16_t value)
{
	if (p.Length == 1)
	{
		memory[p.Address] = lowByte(value);
	}
	else if (p.Length == 2)
	{
		memory[p.Address] = lowByte(value);
		memory[p.Address + 1] = highByte(value);
	}
}

void MemoryMap::setReturnValue(byte *PacketBuffer, const Pointer p, int16_t value)
{
	if (p.Length == 1)
	{
		PacketBuffer[p.Address] = lowByte(value);
	}
	else if (p.Length == 2)
	{
		PacketBuffer[p.Address] = lowByte(value);
		PacketBuffer[p.Address + 1] = highByte(value);
	}
}


void MemoryMap::setPacketData(byte *PacketBuffer)
{
  int address = getReturnValue(PacketBuffer, PMap.Address);
  int length = getReturnValue(PacketBuffer, PMap.Length);
  int count = getReturnValue(PacketBuffer, PMap.Count);

  if (count == 1) //ShortPacket
  {
    if (RomStert <= address && address <= RomEnd)
    {
      for (int i = 0; i < length; i++)
      {
        memory[address + i] = PacketBuffer[dataRecieveHeaderSize + i];
      }
    }
  }
  else if (count > 1) //LongPacket
  {

  }
}

int MemoryMap::returnPacketData(byte *PacketBuffer)
{
  int address = getReturnValue(PacketBuffer, PMap.Address);
  int length = getReturnValue(PacketBuffer, PMap.Length);

  int packetSize = dataRecieveHeaderSize + length + checkSumSize;

  setReturnValue(PacketBuffer, PMap.Header, 0xDFFD);
  setReturnValue(PacketBuffer, PMap.Count, 0x01);

  for (int i = 0; i < length; i++)
  {
    PacketBuffer[dataRecieveHeaderSize + i] = memory[address + i];
  }

  PacketBuffer[packetSize - checkSumSize] = calcCheckSum(PacketBuffer, 2, packetSize - checkSumSize);

  return packetSize;
}

byte MemoryMap::calcCheckSum(byte * dataArray, int start, int end)
{
	byte eachsum = 0;

	for (int i = start; i < end; i++)
	{
		eachsum = (byte)(eachsum ^ dataArray[i]);
	}

	return eachsum;
}
