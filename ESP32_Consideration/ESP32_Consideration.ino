/*---------------------------------------------------------------------------------------------
 exiii inc.
 ESP32 Consideratione
 Use ESP32 dev board as SerialDevice via OSC or Serial connection and Other Consideratione.
	 Description.（ESP32_CommandBoard.inoをBaseとし、ESP32のファームウェア性能向上の検討用）
--------------------------------------------------------------------------------------------- */
#include <vector>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <driver/periph_ctrl.h>
#include <driver/timer.h>
#include <driver/adc.h>
#include <driver/mcpwm.h>
#include <driver/gpio.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <Preferences.h>
#include "MemoryMap.h"
#include "InitialValue.h"
#include "PID.h"


Preferences preferences;

byte PacketBuffer[BufferSize];

int dataRecieveCount = 0;
int dataRecieveSize = 0;
int retryCount = 0;

WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP
OSCErrorCode error;

String oscName;
//IPAddress outIp(192, 168, 10, 101);        // remote IP of your computer
IPAddress outIp(192, 168, 70, 64);        // remote IP of your computer
unsigned int outPort = 50194;          // remote port to receive OSC
unsigned int localPort = 50194;        // local port to listen for OSC packets (actually not used for sending)

String ssid;
String pass;

volatile SemaphoreHandle_t timerSemaphoreA;
hw_timer_t * timerA = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

MemoryMap mmap;
bool oscInitialized = false;


////
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

// Debug LED
static bool toggle04 = 0x01;
static bool toggle21 = 0x01;
static bool toggle22 = 0x01;
static bool toggle27 = 0x01;

// CPU CORE SELECT
enum {
	CORE_1 = 0,
	CORE_2,
	CORE_3,
	CORE_4,
};

#define TIMER_INTR_SEL		TIMER_INTR_LEVEL  	// Timer level interrupt */
#define TIMER_GROUP    		TIMER_GROUP_0     	// Test on timer group 0 */
#define TIMER_DIVIDER  		2               	// Hardware timer clock divider */
#define TIMER_INTERVAL0_SEC	0.0001f  			// test interval for timer 0 [10msec(0.01)/1msec(0.001)/100usec(0.0001),10usec(0.00001)]
#define	TIMER_CPU_CLOCK		80 * 100000L		// CPU Clock 80Mhz	160000000L	160Mhz
#define TIMER_SCALE    		(TIMER_BASE_CLK / TIMER_DIVIDER)  // used to calculate counter value

volatile xSemaphoreHandle semaphore;
static intr_handle_t s_timer_handle;
ETSTimer Timer;

///

#pragma region  ARDUINO_STD_SETUP_AND_LOOP
void setup()
{
	//Serial.begin(BaudRateList[mmap.getValue(mmap.BaudRate)]);
	Serial.begin(BaudRateList[mmap.getValue(mmap.BaudRate)]);
	Serial.println("Start Serup");

	pinMode( 4, OUTPUT);
	pinMode(21, OUTPUT);
	pinMode(22, OUTPUT);
	pinMode(27, OUTPUT);

	ConnectionSetup();

	TimerSetup();		// 33msec
	TimerSetup2();		// 100usec

	const int MotorControlFrequency = 100000;
	const int MotorControlResolution = 8;

	for (int i = 0; i < MotorNumber; i++)
	{
		pinMode(mmap.getValue(mmap.AnalogReadPin[i]), INPUT);

		if (AssignedPWMForwardPin != 0)
		{
			ledcSetup(i, MotorControlFrequency, MotorControlResolution);
			ledcAttachPin(AssignedPWMForwardPin[i], i);

			ledcSetup(i + MotorNumber, MotorControlFrequency, MotorControlResolution);
			ledcAttachPin(AssignedPWMBackPin[i], i + MotorNumber);
		}
	}
}

void loop()
{
	//Data Update
	if (ModeComn == MODE_OSC)
	{
		oscUpdate();
	}
	else if (ModeComn == MODE_SERIAL)
	{
		serialUpdate();
	}

	// If Timer has fired
	if(xSemaphoreTake(timerSemaphoreA, 0) == pdTRUE)
	{
		//Serial.println("TimerCalled");

		toggle27 ^= 0x01;
		digitalWrite(27, toggle27);

		//Data Update

		//Motor control part

		//Motor Control
		mmap.timerUpdate();

		if (oscInitialized)
		{
			// freeRTOS Task Create
			xTaskCreatePinnedToCore(task_Connection,   "Connection",   4096, NULL, 1, NULL, CORE_1);
			xTaskCreatePinnedToCore(task_ControlMotor, "MoterControl", 4096, NULL, 1, NULL, CORE_2);
			oscInitialized = false;
		}
	}
	delay(10);
}

#pragma endregion

#pragma region TIMER_FUNCTIONS

static void IRAM_ATTR onTimerA()
{

	toggle04 ^= 0x01;
	digitalWrite(4, toggle04);

	portENTER_CRITICAL_ISR(&timerMux);
	isrCounter++;
	lastIsrAt = millis();
	portEXIT_CRITICAL_ISR(&timerMux);

	xSemaphoreGiveFromISR(timerSemaphoreA, NULL);
}

void TimerSetup()
{

	// Create semaphore to inform us when the timer has fired
	timerSemaphoreA = xSemaphoreCreateBinary();

	// Use 1st timer of 4 (counted from zero).
	// Set 80 divider for prescaler (see ESP32 Technical Reference Manual for moreinfo).
	timerA = timerBegin(TIMER_0, TIMER_BASE_CLK/1000000, true);	// 100mse

	// Attach onTimer function to our timer.
	timerAttachInterrupt(timerA, &onTimerA, true);

	// Set alarm to call onTimer function every second (value in microseconds).
	// Repeat the alarm (third parameter)
	timerAlarmWrite(timerA, TimerSpan33, true);

	// Start an alarm
	timerAlarmEnable(timerA);

}

static void IRAM_ATTR onTimerB(void *para)
{
	toggle21 ^= 0x01;
	digitalWrite(21, toggle21);

	TIMERG0.hw_timer[TIMER_1].update = 1;
	TIMERG0.int_clr_timers.t1 = 1;

	TIMERG0.hw_timer[TIMER_1].config.alarm_en = 1;

	xSemaphoreGiveFromISR(timerSemaphoreA, NULL);
}

void TimerSetup2()
 {
	// Configure timer
	timer_config_t config;
	config.alarm_en = true;
	config.counter_en = false;
	config.counter_dir = TIMER_COUNT_UP;
	config.auto_reload = true;
	config.divider = TIMER_BASE_CLK / 1000000;   /* 1 us per tick */
	config.divider = TIMER_DIVIDER;
	config.intr_type = TIMER_INTR_SEL;
	config.counter_en = TIMER_PAUSE;
	timer_init(TIMER_GROUP_0, TIMER_1, &config);

	timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0x00000000ULL);
	timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, TIMER_INTERVAL0_SEC * TIMER_SCALE);
	timer_enable_intr(TIMER_GROUP_0, TIMER_1);
	timer_isr_register(TIMER_GROUP_0, TIMER_1, &onTimerB, NULL, 0, &s_timer_handle);

	timer_start(TIMER_GROUP_0, TIMER_1);
}

#pragma endregion

#pragma region COMMUNICATION
void ConnectionSetup()
{
	if (ModeComn == MODE_OSC)
	{
		Serial.println("OSC Mode");

		ssid = SSID;
		pass = PASS;

		//OSC setup
		WiFi.disconnect();
		WiFi.mode(WIFI_STA);
		WiFi.begin(ssid.c_str(), pass.c_str());
		Serial.print("try to connect ");
		Serial.println(ssid);

		int count = 0;

		while (WiFi.status() != WL_CONNECTED)
		{
			Serial.println("Error cannot connect to WiFi!");
			delay(1000);
			count++;

			if (count >= 5)
			{
				WiFi.disconnect();
				WiFi.mode(WIFI_STA);
				WiFi.begin(ssid.c_str(), pass.c_str());
				Serial.print("try to connect ");
				count = 0;
			}

			/*
			//get ip address and passphrase
			while (Serial.available() > 0)
			{
			delay(500);
			char getchr = Serial.read();
			static int getCharMode = 0;
			if (getchar == 0x00) {
			//start read id
			getCharMode = 1;
			}				else if (getchr == 0x0F) {
			//end read id
			getCharMode = 0;
			}
			else if (getchr == 0xF0) {
			//start read pass
			getCharMode = 2;
			}
			else if (getchr == 0xFF) {
			//end read pass
			getCharMode = 0;
			}
			else {
			if (getCharMode == 1) {
			}
			else if (getCharMode == 2) {
			}
			}
			}*/
		}

		Serial.print(WiFi.localIP());

		//pass own ip to pc
		Serial.print("IP address: ");
		Serial.println(WiFi.localIP());

		Udp.begin(localPort);
	}
	else if (ModeComn == MODE_SERIAL)
	{
		//Serial setup
		while (!Serial)
		{
			; // wait for serial port to connect. Needed for native USB port only
		}
	}
}


//Serial communication functions
//under construction
void serialUpdate()
{
	if (Serial.available() >= dataRecieveHeaderSize)
	{
		dataRecieveCount += Serial.readBytes(&PacketBuffer[0], dataRecieveHeaderSize);

		while (PacketBuffer[0] != 0xFA || PacketBuffer[1] != 0xAF)
		{
			for (int i = 0; i < dataRecieveHeaderSize - 1; i++)
			{
				PacketBuffer[i] = PacketBuffer[i + 1];
				Serial.readBytes(&PacketBuffer[dataRecieveHeaderSize - 1], 1);
			}

			retryCount++;

			if (retryCount > retry)
			{
				return;
			}
		}

		dataRecieveSize = PacketBuffer[5] * PacketBuffer[6];
		dataRecieveCount += Serial.readBytes(&PacketBuffer[7], dataRecieveSize + checkSumSize);

		if (dataRecieveCount == (dataRecieveHeaderSize + dataRecieveSize + checkSumSize))
		{
			switch (mmap.getValue(mmap.PMap.Flags))
			{

			case 0b00000000:
				recieveData(PacketBuffer);
				break;

			case 0b00001111:
				returnData(PacketBuffer);
				break;
			}
		}
		else
		{
			//sendError();
		}
	}
}

void recieveData(byte * PacketBuffer)
{
	mmap.setPacketData(PacketBuffer);
}

void returnData(byte * PacketBuffer)
{
	int packetSize = mmap.returnPacketData(PacketBuffer);
	Serial.write(PacketBuffer, packetSize);
}

//OSC communication functions
void oscUpdate()
{
	OSCMessage receiveMsg;

	int size = Udp.parsePacket();

	if (size > 0)
	{
		while (size--)
		{
			receiveMsg.fill(Udp.read());
		}

		if (!receiveMsg.hasError())
		{
			receiveMsg.dispatch("/Osc/Init", receiveOscInit);
			receiveMsg.dispatch("/Osc/AddTarget", receiveOscAddTarget);
			receiveMsg.dispatch("/Command/Send", receiveOscData);
			receiveMsg.dispatch("/Command/Demand", returnOscData);
		}
		else
		{
			error = receiveMsg.getError();
		}
	}
}

void receiveOscInit(OSCMessage &msg)
{
	Serial.println("get osc init");

	OSCMessage rmsg("/Osc/Init/Return");
	rmsg.add("osc");
	rmsg.add(WiFi.localIP().toString());
	Udp.beginPacket(outIp, outPort);
	rmsg.send(Udp);
	Udp.endPacket();
	rmsg.empty();
}

void receiveOscAddTarget(OSCMessage &msg)
{
	Serial.println("get osc add target");
	oscInitialized = true;
	int len = msg.getDataLength(0);
	char tmpchr[15];
	String intIP[4];

	msg.getString(0, tmpchr, len);
	len = msg.getDataLength(1);
	oscName = String(tmpchr);

	msg.getString(1, tmpchr, len);
	split(String(tmpchr), '.', intIP);

	for (int i = 0; i < 4; i++)
	{
		Serial.println(intIP[i]);
	}

	outIp = IPAddress(intIP[0].toInt(), intIP[1].toInt(), intIP[2].toInt(), intIP[3].toInt());
	outPort = (unsigned int)msg.getInt(2);
}

void receiveOscData(OSCMessage &message)
{
	//Serial.println("get osc data");
	uint8_t Buffer[BufferSize];
	message.getBlob(1, Buffer, BufferSize);
	mmap.setPacketData(Buffer);
}

void returnOscData(OSCMessage &message)
{
	//Serial.println("get osc demand");
	uint8_t Buffer[BufferSize];
	message.getBlob(1, Buffer, BufferSize);
	int length = mmap.returnPacketData(Buffer);

	OSCMessage msg("/Serial/Demand/Return");
	msg.add(Buffer, length);
	Udp.beginPacket(outIp, outPort);
	msg.send(Udp);
	Udp.endPacket();
	msg.empty();
}

#pragma region TASK_FUNCTIONS

PID pid[MotorNumber] =
{
	PID(0.03, 0.015, 0.01),
	PID(0.03, 0.015, 0.01),
	PID(0.03, 0.015, 0.01),
};
#pragma endregion

#pragma region	TASK
void task_ControlMotor(void* arg) {

	const int torqueMax = 255;

	while( true ) {
	  	//Serial.println("motor control called");
		for (int i = 0; i < MotorNumber; i++)
		{
			//pid[i].target = mmap.getValue(mmap.GoalPosition[i]);

			//int position = mmap.getValue(mmap.AnalogReadFilteredValue[i]);

			//pid[i].Input(position, TimerSpan);

			/*
			if (i == 0)
			{
				Serial.print("motor : ");
				Serial.print(i);
				Serial.print(" / target :");
				Serial.print(pid[i].target);
				Serial.print(" / position :");
				Serial.print(position);
				Serial.print(" / output :");
				Serial.print(pid[i].output);
				Serial.print(" / P :");
				Serial.print(pid[i].P);
				Serial.print(" / I :");
				Serial.print(pid[i].I);
				Serial.print(" / D :");
				Serial.print(pid[i].D);
				Serial.println("");
			}
			*/
		}

		bool enable =  false;
		const int th = 20;
		int torque = 0;
		int position = 0;
		int target = 0;
		int i = 0;

		for (i = 0; i < MotorNumber; i++)
		{
			torque = 0;

			//int torque = mmap.getValue(mmap.GoalTorque[i]);

			//int torque = pid[i].output;

			position = mmap.getValue(mmap.AnalogReadFilteredValue[i]);
			target = mmap.getValue(mmap.GoalPosition[i]);

			if (enable)
			{
				if (position > (target + th))
				{
					enable = false;
					torque = (-64);
				}
			}
			else
			{
				if (position < target)
				{
					enable = true;
					torque = (-255);
				}
			}
		}

		if (i  == 0)
		{
			Serial.print("position :");
			Serial.print(position);
			Serial.print(" / target :");
			Serial.print(target);
			Serial.print(" / torque :");
			Serial.print(torque);
			Serial.println("");
		}


		//Serial.print(torque);
		//Serial.print(" ");

		if (AssignedPWMForwardPin[i] != 0)
		{
			constrain(torque, -1 * torqueMax, torqueMax);

			if (torque > 0)
			{
				ledcWrite(i, abs(torque));
				ledcWrite(i + MotorNumber, 0);
			}
			else if (torque < 0)
			{
				ledcWrite(i, 0);
				ledcWrite(i + MotorNumber, abs(torque));
			}
			else
			{
				ledcWrite(i, 0);
				ledcWrite(i + MotorNumber, 0);
			}
		}
		//vTaskDelay(1);		// 1msec
		toggle22 ^= 0x01;
		digitalWrite(22, toggle22);
	}
}

void task_Connection(void* arg) {

	while ( true ) {
		vTaskDelay(1);		// 1msec
	}
}

#pragma endregion


int split(String data, char delimiter, String *dst)
{
	int index = 0;
	int arraySize = (sizeof(data) / sizeof((data)[0]));
	int datalength = data.length();

	for (int i = 0; i < datalength; i++)
	{
		char tmp = data.charAt(i);

		if (tmp == delimiter)
		{
			index++;
			if (index >(arraySize - 1)) return -1;
		}
		else
		{
			dst[index] += tmp;
		}
	}

	return (index + 1);
}
