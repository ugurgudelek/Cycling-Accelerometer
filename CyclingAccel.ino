// Name:		CyclingAccel_Esp.ino
// Created:	6/7/2016 11:13:02 AM
// Author:	ugurgudelek
//


//#define BLUETOOTH_CONFIG

//MASTER		: atmega32u4 ,	has RF24, no battery,	need RF24 Lib, Joystick Lib
//CYCLER		: atmega328  ,	has RF24, need battery, need RF24 Lib, MPU6050 Lib,Filtering Lib
//HANDLE_BAR	: atmega328  ,	has RF24, need battery,	need RF24 Lib

//#define MASTER
//#define HANDLE_BAR
#define CYCLER

#if defined(MASTER)
	#define RF
	#define BLUETOOTH
#endif
#if defined(CYCLER)
	#define RF
#endif
#if defined(HANDLE_BAR)
	#define BLUETOOTH
#endif

//#define DEBUG
//#define DEBUG_CALIBRATED
//#define DEBUG_RAW
//#define DEBUG_SENT
//#define DEBUG_RECV
//#define TEST

#define TIMEOUT_PERIOD 100  //ms
#define REREQUEST_PERIOD 40 //ms
#define DATA_SIZE 10



#if defined(BLUETOOTH) || defined(BLUETOOTH_CONFIG)
#include <SoftwareSerial.h>
	#if defined(MASTER)
		SoftwareSerial bluetoothSerial(8,9);
	#else
		SoftwareSerial bluetoothSerial(9,10);
	#endif
#endif

#if defined(RF)
#include <RF24_config.h>
#include <RF24.h>
#include <printf.h>
#include <nRF24L01.h>
RF24 radio = RF24(6, 5);
// Single radio pipe address for the 2 nodes to communicate.
const uint64_t pipe = 0xE8E8F0F0E1LL;
#endif // BLUETOOTH || BLUETOOTH_CONFIG || !RF

#if defined(MASTER)
#include <Joystick.h>
#include <HID.h>
float velocityForMaster = 0.0;

#elif defined(CYCLER)
#include <I2Cdev.h>
#include <MPU6050.h>
#include <RunningStatistics.h>
#include <FilterOnePole.h> 
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
//////////////////////////////CYCLING DEFINITIONS//////////////////////////////

#define CYCLING_CIRCUMFERENCE 4 //2*PI*26inch = 4.14941558
//neglect btw rollOffset + rollThreshold , rollOffset - rollThreshold
float valueThreshold = 100.0f;
float valueOffset = 0.0f;
bool valueState = false;
float velocity = 0.0f;
long velocityTic = -1;
float oldVelocity = 0.0f;
long velocityTimeout = 2000;
long velocityTimeoutTimer = -1;

//////////////////////////////CYCLING DEFINITIONS//////////////////////////////


//////////////////////////////FILTER DEFINITIONS//////////////////////////////

// filters out changes faster that 2 Hz.
float _filterFrequency = 2.0;

FilterOnePole gz_filter;

RunningStatistics gz_filter_statistics;


//////////////////////////////FILTER DEFINITIONS//////////////////////////////


//////////////////////////////MPU DEFINITIONS//////////////////////////////

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;


//////////////////////////////MPU DEFINITIONS//////////////////////////////
float filterHandler();
float calcVelocity(float value);

#elif defined(HANDLE_BAR)
#define ANALOG_A_X_AXIS_PIN A1
#define ANALOG_A_Y_AXIS_PIN	A0
#define ANALOG_B_X_AXIS_PIN	A4
#define ANALOG_B_Y_AXIS_PIN	A3
#define BUTTON_1 6
#define BUTTON_2 3
#define BUTTON_3 7
#define BUTTON_4 5

#endif

int calibrated(int actualValue, int min, int origin, int max);

typedef enum
{
	master = 1,
	cycler,
	handle_bar
} Role;

Role role;

struct Device
{
	Role id;
	float sendPacket[DATA_SIZE + 2];
	float recvPacket[DATA_SIZE + 2];
	uint8_t packetSize = sizeof(sendPacket);

	void setSendPacket(Role target, float dataToSent[DATA_SIZE])
	{
		sendPacket[0] = id;
		sendPacket[1] = target;
		for (int i = 0; i < DATA_SIZE; i++)
			sendPacket[i + 2] = dataToSent[i];
	}
} Device_type;

struct Packet
{
private:
	float _analogAX;
	float _analogAY;
	float _analogBX;
	float _analogBY;
	float _button1;
	float _button2;
	float _button3;
	float _button4;
	float _velocity = 5;
public:
	void setAnalogAX(float ax)
	{
		_analogAX = ax;
	};

	void setAnalogAY(float ay)
	{
		_analogAY = ay;
	};

	void setAnalogBX(float bx)
	{
		_analogBX = bx;
	};

	void setAnalogBY(float by)
	{
		_analogBY = by;
	};

	void setButton1(float b)
	{
		_button1 = b;
	};

	void setButton2(float b)
	{
		_button2 = b;
	};

	void setButton3(float b)
	{
		_button3 = b;
	};
	void setButton4(float b)
	{
		_button4 = b;
	};
	void setVelocity(float v)
	{
		_velocity = constrain(v, 0.0, 10.0);
	};

	float getAnalogAX()
	{
		return constrain(calibrated(_analogAX, 10, 502, 1023), -127, 127);
	};

	float getAnalogAY()
	{
		return constrain(calibrated(_analogAY, 0, 506, 1023), -127, 127);
	};

	float getAnalogBX()
	{
		return constrain(calibrated(_analogBX, 0, 516, 930), -127, 127);
	};

	float getAnalogBY()
	{
		return constrain(calibrated(_analogBY, 0, 508, 1023), -127, 127);
	};

	float getButton1()
	{
		return _button1;
	};

	float getButton2()
	{
		return _button2;
	};

	float getButton3()
	{
		return _button3;
	};
	float getButton4()
	{
		return _button4;
	};
	float getVelocity()
	{
		return _velocity;
	};

	float getRawAnalogAX()
	{
		return _analogAX;
	};

	float getRawAnalogAY()
	{
		return _analogAY;
	};

	float getRawAnalogBX()
	{
		return _analogBX;
	};

	float getRawAnalogBY()
	{
		return _analogBY;
	};


	//do not let joystick go outside of defined circle
	void optimizeAXYAxis(float* arr, float coefX, float coefY)
	{
		double ax = getAnalogAX() * _velocity;
		double ay = getAnalogAY() * _velocity;
		double r = sqrt(ax * ax + ay * ay);
		double coef = r / 127.0;

		double xCalibrated = 0.0;
		double yCalibrated = 0.0;

		if (coef > 0.0)
		{
			xCalibrated = ax / coef;
			yCalibrated = ay / coef;
		}
		double xMin = (ax < 0) ? (xCalibrated) : -(xCalibrated);
		double xMax = (ax < 0) ? -(xCalibrated) : (xCalibrated);

		double yMin = (ay < 0) ? (yCalibrated) : -(yCalibrated);
		double yMax = (ay < 0) ? -(yCalibrated) : (yCalibrated);

		//float* returned = new float[sizeof(float)*2];

		arr[0] = constrain(ax * coefX, xMin, xMax);
		arr[1] = constrain(ay * coefY, yMin, yMax);
	}
} Packet_in;


Device device;
Packet packet;


bool* multiRequest(Role* roleArray, uint8_t sizeOfArray);
void multiRequestBluetooth();
void respond(float* data);
bool detectBleBaudRate();


void setup()
{
	
	#if  defined(BLUETOOTH_CONFIG)
		#if defined(MASTER)
			String	deviceName	= "Master Bluetooth";
			int		password	= 1234;
			String	uart		= "38400,0,0";
		#elif defined(CYCLER)
			String deviceName = "Cycler Bluetooth";
			int		password = 1234;
			String	uart = "38400,0,0";
		#elif defined(HANDLE_BAR)
			String deviceName = "Handle_Bar Bluetooth";
			int		password = 1234;
			String	uart = "38400,0,0";
		#endif

		Serial.begin(38400);
		Serial.println("HC-05 Modul Ayarlaniyor...");
		Serial.println("Lutfen 5 sn icinde HC-05 modulun uzerindeki butona basili tutarak baglanti yapiniz.");
		bluetoothSerial.begin(38400);
		delay(5000);
		bluetoothSerial.print("AT+NAME=");
		bluetoothSerial.println(deviceName);
		Serial.print("Isim ayarlandi: ");
		Serial.println(deviceName);
		delay(1000);
		bluetoothSerial.print("AT+PSWD=");
		bluetoothSerial.println(password);
		Serial.print("Sifre ayarlandi: ");
		Serial.println(password);
		delay(1000);
		bluetoothSerial.print("AT+UART=");
		bluetoothSerial.println(uart);
		Serial.print("Baud rate ayarlandi: ");
		Serial.println(uart);
		delay(2000);
		Serial.println("Islem tamamlandi.");
		
	#elif defined(RF) || defined(BLUETOOTH)
		Serial.begin(9600);
		

		#if defined(RF)
			//start radio
			radio.begin();
	
			radio.setDataRate(RF24_2MBPS);
			radio.setPALevel(RF24_PA_MAX);
			radio.openWritingPipe(pipe);
			radio.openReadingPipe(1, pipe);
		#endif
		#if defined(BLUETOOTH)
			//start bluetooth comm
			bluetoothSerial.begin(9600);
		#endif // if(RF) , elif(BLUETOOTH)



		#if defined(MASTER)
	
			device.id = master;
			#if defined(RF)
				Serial.println("master Radio");
			#endif // RF
			#if defined(BLUETOOTH)
				Serial.println("master Bluetooth");
			#endif // BLUETOOTH
			Joystick.begin();
			delay(5000);

		#elif defined(CYCLER)
			device.id = cycler;

			Wire.begin();
			#if defined(RF)
				Serial.println("cycler Radio");
			#endif // RF

			// initialize device
			Serial.println("Initializing I2C devices...");
			accelgyro.initialize();

			// verify connection
			Serial.println("Testing device connections...");
			Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
	

		#elif defined(HANDLE_BAR)
			device.id = handle_bar;
			pinMode(ANALOG_A_X_AXIS_PIN, INPUT);
			pinMode(ANALOG_A_Y_AXIS_PIN, INPUT);
			pinMode(ANALOG_B_X_AXIS_PIN, INPUT);
			pinMode(ANALOG_B_Y_AXIS_PIN, INPUT);
			pinMode(BUTTON_1, INPUT_PULLUP);
			pinMode(BUTTON_2, INPUT_PULLUP);
			pinMode(BUTTON_3, INPUT_PULLUP);
			pinMode(BUTTON_4, INPUT_PULLUP);
			#if defined(RF)
				Serial.println("handleBar Radio");
			#endif // RF
			#if defined(BLUETOOTH)
							Serial.println("Handle Bar Bluetooth");
			#endif // BLUETOOTH

		#endif


		delay(2000);
		#if defined(RF)
			printf_begin();
			radio.printDetails();
			Serial.println("Details printed.");
			radio.startListening();
		#endif // RF

	#endif // if(BLUETOOTH_CONFIG) , elif(RF||BLUETOOTH)
}


//void sendBluetoothRequest()
//{
//	bluetoothSerial.write(100);
//#ifdef DEBUG
//	Serial.println("Request sent.");
//#endif
//	
//}

void loop()
{
	
#ifdef TEST
	while (bluetoothSerial.available())
	{
		Serial.write(bluetoothSerial.read());
	}
	while (Serial.available())
	{
		bluetoothSerial.write(Serial.read());
	}
	delay(100);
#endif

#ifndef TEST
	#if defined(MASTER)
		unsigned long timeSpent = millis();
		Role rArr[1] = { cycler };
		#if defined(RF)
				multiRequest(rArr, 1);
		#endif
		#if defined(BLUETOOTH)
				//multiRequestBluetooth();
				sendBluetoothRequest();
				while (bluetoothSerial.available())
				{
					uint8_t incoming[9];
					bluetoothSerial.readBytesUntil('\n',incoming, 9);
					//crc check
					if(incoming[8] == (incoming[0] + incoming[1] + incoming[2] + incoming[3] + incoming[4] + incoming[5] + incoming[6] + incoming[7]) / 8)
					{
#ifdef DEBUG
						Serial.print(incoming[0]); Serial.print("\t");
						Serial.print(incoming[1]); Serial.print("\t");
						Serial.print(incoming[2]); Serial.print("\t");
						Serial.print(incoming[3]); Serial.print("\t");
						Serial.print(incoming[4]); Serial.print("\t");
						Serial.print(incoming[5]); Serial.print("\t");
						Serial.print(incoming[6]); Serial.print("\t");
						Serial.print(incoming[7]); Serial.println("\t");
#endif
						packet.setAnalogAX(incoming[0] * 4);
						packet.setAnalogAY(incoming[1] * 4);
						packet.setAnalogBX(incoming[2] * 4);
						packet.setAnalogBY(incoming[3] * 4);
						
						packet.setButton1(incoming[7]);
						packet.setButton2(incoming[5]);
						packet.setButton3(incoming[6]);
						packet.setButton4(incoming[4]);
					}

					
				}
				
				
		#endif //if(RF), elif(BLUETOOTH)
		
		float optimized[2];
		packet.optimizeAXYAxis(optimized, 0.2, 0.2);

		Joystick.setXAxis(-optimized[0]);
		Joystick.setYAxis(-optimized[1]);
		Joystick.setXAxisRotation(map(packet.getAnalogBX() + 127, 0, 254, 1, 359));
		Joystick.setYAxisRotation(map(packet.getAnalogBY() + 127, 0, 254, 1, 359));
		Joystick.setButton(0, packet.getButton1());
		Joystick.setButton(1, packet.getButton2());
		Joystick.setButton(2, packet.getButton3());
		Joystick.setButton(3, packet.getButton4());

		timeSpent = millis() - timeSpent;

		#ifdef DEBUG_CALIBRATED
			Serial.print(optimized[0]); Serial.print("\t");
			Serial.print(optimized[1]);	Serial.print("\t");
			Serial.print(packet.getAnalogBX());	Serial.print("\t");
			Serial.print(packet.getAnalogBY());	Serial.print("\t");
			Serial.println(packet.getVelocity());
		#endif // DEBUG_CALIBRATED
		#ifdef DEBUG_RAW
			Serial.print(packet.getRawAnalogAX()); Serial.print("\t");
			Serial.print(packet.getRawAnalogAY());	Serial.print("\t");
			Serial.print(packet.getRawAnalogBX());	Serial.print("\t");
			Serial.print(packet.getRawAnalogBY());	Serial.println("\t");
		#endif // DEBUG_CALIBRATED


	

	#elif defined(CYCLER)
		//get raw data from MPU
		accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		//filter them to make smoothie
		//calculate current velocity
		float filtered = filterHandler();
		float velocity = calcVelocity(filtered);

		float data[10] = { velocity };
		#if defined(RF)
				respond(data);
				Serial.println(filtered);
		#endif
		

	#elif defined(HANDLE_BAR)
		uint8_t data[9] = { analogRead(ANALOG_A_X_AXIS_PIN)/4,
			analogRead(ANALOG_A_Y_AXIS_PIN)/4,
			analogRead(ANALOG_B_X_AXIS_PIN)/4,
			analogRead(ANALOG_B_Y_AXIS_PIN)/4,
			!digitalRead(BUTTON_1),
			!digitalRead(BUTTON_2),
			!digitalRead(BUTTON_3),
			!digitalRead(BUTTON_4),
			0
			};
		data[8] = (data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7]) / 8;
#ifdef DEBUG
		Serial.print(data[0]); Serial.print("\t");
		Serial.print(data[1]); Serial.print("\t");
		Serial.print(data[2]); Serial.print("\t");
		Serial.print(data[3]); Serial.print("\t");
		Serial.print(data[4]); Serial.print("\t");
		Serial.print(data[5]); Serial.print("\t");
		Serial.print(data[6]); Serial.print("\t");
		Serial.print(data[7]); Serial.print("\t");
		Serial.print(data[8]); Serial.println("\t");
#endif
		

	/*uint64_t data = 0;
	data |= (uint16_t(analogRead(ANALOG_A_X_AXIS_PIN)) & 0b0000001111111111) << 54;
	data |= (uint16_t(analogRead(ANALOG_A_Y_AXIS_PIN)) & 0b0000001111111111) << 44;
	data |= (uint16_t(analogRead(ANALOG_B_X_AXIS_PIN)) & 0b0000001111111111) << 34;
	data |= (uint16_t(analogRead(ANALOG_B_Y_AXIS_PIN)) & 0b0000001111111111) << 24;
	data |= (uint16_t(!digitalRead(BUTTON_1)) & 0b0000000000000001) << 23;
	data |= (uint16_t(!digitalRead(BUTTON_2)) & 0b0000000000000001) << 22;
	data |= (uint16_t(!digitalRead(BUTTON_3)) & 0b0000000000000001) << 21;*/
			#if defined(RF)
					respond(data);
			#endif
			#if defined(BLUETOOTH)
					if(bluetoothSerial.read() == 100)
					{
						bluetoothSerial.write(data, 9);
						bluetoothSerial.write('\n');
					}
						
					
					
			#endif
					
	#endif // if(MASTER), elif(CYCLER), elif(HANDLE_BAR)
#endif

	//double angle = (analogX == 0 && analogY == 0)? 0.0 : (atan2(analogY, analogX) * 180 / PI);
	//angle += (angle < 0) ? 360.0 : 0.0;
}


#if defined(CYCLER)

float filterHandler()
{
	gz_filter.input(gz);
	gz_filter_statistics.input(gz_filter.output());
	return gz_filter_statistics.mean();
}

bool isToggleGz(float value)
{
	if (valueState == false && value > valueOffset + valueThreshold)
	{
		valueState = true;
		return true;
	}
	if (valueState == true && value < valueOffset - valueThreshold)
	{
		valueState = false;
		return true;
	}

	return false;
}

float calcVelocity(float value)
{
	if (isToggleGz(value))
	{
		velocityTimeoutTimer = -1;
		if (velocityTic == -1)
		{
			velocityTic = millis();
		}

		else
		{
			long halvedPeriod = (millis() - velocityTic);
			float freq = 1000.0f / float(halvedPeriod * 2.0f);
			velocity = freq * (float)CYCLING_CIRCUMFERENCE;

			velocityTic = -1;

		}
	}
	else
	{
		if (velocityTimeoutTimer == -1)
		{
			velocityTimeoutTimer = millis();
		}
		else if (millis() - velocityTimeoutTimer > velocityTimeout)
		{
			velocity = 0.0;

		}
	}

	return velocity;

}

#endif

#if defined(RF)

	bool* multiRequest(Role* roleArray, uint8_t sizeOfArray)
	{
		bool timeoutOccurred[2];
		bool checked[2];

		unsigned long readTimeoutTimer;
		long reRequestTimer;
		for (int i = 0; i < sizeOfArray; i++)
		{
			timeoutOccurred[i] = false;
			checked[i] = false;
			readTimeoutTimer = millis();
			reRequestTimer = -1;

			while (true)
			{
				if (millis() - readTimeoutTimer > TIMEOUT_PERIOD)
				{
					#ifdef DEBUG
						Serial.print("TIMEOUT\t");
						Serial.println(roleArray[i]);
					#endif // DEBUG

					timeoutOccurred[i] = true;
					break;
				}

				while ((millis() - reRequestTimer > REREQUEST_PERIOD) || reRequestTimer == -1)
				{
					radio.stopListening();

					float data[DATA_SIZE] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
					device.setSendPacket(roleArray[i], data);
					#ifdef DEBUG_SENT
						Serial.print("Request from : \t");
						Serial.println(device.sendPacket[1]);
					#endif // DEBUG_SENT
					reRequestTimer = millis();
					radio.write(device.sendPacket, device.packetSize);

					radio.startListening();
				}
				if (radio.available())
				{
					while (radio.available())
					{
						radio.read(&device.recvPacket, device.packetSize);
					}


					if (device.recvPacket[0] == cycler)
					{
						if (checked[i] == false)
						{
							#ifdef DEBUG_RECV
								Serial.print("Respond\t"); Serial.print(cycler);Serial.print(" :: \t");
								Serial.print(millis() - readTimeoutTimer); Serial.println("ms\t");
							#endif // DEBUG_RECV
							checked[i] = true;
							packet.setVelocity(device.recvPacket[2]);
							break;
						}
					}

					else if (device.recvPacket[0] == handle_bar)
					{
						if (checked[i] == false)
						{
							#ifdef DEBUG_RECV
								Serial.print("Respond\t"); Serial.print(handle_bar);Serial.print(" :: \t");
								Serial.print(millis() - readTimeoutTimer); Serial.println("ms\t");
							#endif // DEBUG_RECV

							checked[i] = true;
							packet.setAnalogAX(device.recvPacket[2]);
							packet.setAnalogAY(device.recvPacket[3]);
							packet.setAnalogBX(device.recvPacket[4]);
							packet.setAnalogBY(device.recvPacket[5]);
							packet.setButton1(device.recvPacket[6]);
							packet.setButton2(device.recvPacket[7]);
							packet.setButton3(device.recvPacket[8]);
							break;
						}
					}
				}
			}
		}

		return timeoutOccurred;
	}

	void respond(float* data)
	{
		if (radio.available())
		{
			while (radio.available())
			{
				radio.read(&device.recvPacket, device.packetSize);
			}

			// only respond if master requires an answer
			if (device.recvPacket[0] == master)
			{
				// only respond if id is mine
				if (device.recvPacket[1] == device.id)
				{
					//wait a lil bit for reply
					delay(5);
					radio.stopListening();

					device.setSendPacket(master, data);
					radio.write(device.sendPacket, device.packetSize);

					radio.startListening();

					#ifdef DEBUG
						Serial.println("OK");
					#endif // DEBUG
				}
			}
		}
	}
#endif
#if defined(BLUETOOTH)
	void multiRequestBluetooth() {};
#endif // if(RF), elif(BLUETOOTH)


int calibrated(int actualValue, int min, int origin, int max)
{
	if (actualValue == origin)
		return 0;

	return (actualValue < origin) ? map(actualValue, min, origin, -127, 0) : map(actualValue, origin, max, 0, 127);
}

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
	return (float)(x - in_min) * (float)(out_max - out_min) / (float)(in_max - in_min) + out_min;
}

#if defined(BLUETOOTH_CONFIG) || defined(BLUETOOTH)
bool detectBleBaudRate() {
	long bauds[] = { 9600,115200,
		1200,2400,4800,19200,38400,57600,230400 };
	Serial.println("Detecting BLE baud rate:");
	for (int i = 0; i<(sizeof(bauds) / sizeof(long)); i++) {
		Serial.write("Checking ");
		long cur_baud = bauds[i];
		Serial.println(cur_baud, DEC);

		bluetoothSerial.begin(cur_baud);
		bluetoothSerial.println("AT");
		bluetoothSerial.flush();
		delay(50);

		String response = bluetoothSerial.readString();
		Serial.println(response);
		if (response == "OK\r\n") {
			Serial.println("Detected");
			return true;
		}
		else {
			bluetoothSerial.end();
		}
	}
	return false;
}
struct BLE
{
	void request(String s)
	{
		//println used because it has carriage return and new line feed inside
		bluetoothSerial.println(s);
	}
	String respond()
	{
		if (bluetoothSerial.available())
		{
			return bluetoothSerial.readStringUntil('\n');
		}
		return "NAN";
	}

}ble_device;

BLE ble;
#endif

