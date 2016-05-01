/* Jank Wheels Drive Software
 * Created By: Greg Balke
 * Date: April 2016
 */ 

#include <cmath>
#include "Arduino.h"
#include "HardwareSerial.h"

#include "WProgram.h"
//#include "Servo.h"

//Working Values:
//	kp = 0.58
//	kd = -0.58, -0.53
//	ki = 0.01
const int driveSpeed = 400;		// Drive speed for motors.
const float kp = 0.58;			// Proportional Correction Constant.
const float kd = -0.6;// -0.6;			// Derivative Multiplier.
const float ki = 0;//0.05;				// Integral Multiplier.
const int centerPoint = 60;

// BLUETOOTH START SETTING //
#define BTSTART 1

// IO defines //
#define ao 	20
#define si	21
#define clk	22

#define srv 17

#define drv 16
#define brk 12

// Constants //
#define DRK 0
#define BRT 1
#define FWD 0
#define BCK 1

// Serial //
#define SERIALBT 1
#define PRINTLINE 0
#define BAUD 115200

#define HWSerial Serial1

// Variables //
const int size = 128;			// Pixel Width of Line Cam.
const int maxVal = 1024;		// Largest Light Val Output.

const int clear = 15;			// Delay for Cam Exposure.
const int endDelay = 0;		// Large Delay for Image to Buffer over Serial (set to 0 when running).
const int time = 2;				// Time for duty cycle calc.
const float duty = 0.5;			// Duty cycle multiplier.
const int d1 = time * duty;		// Calculating Time for On Period.
const int d2 = time - d1;		// Calculating Time for Off Period.

const int mapScale = 50;		// Scaling of Map for Serial.

// SERVO CONSTS DO NOT TOUCH UNLESS TESTING ON OSCILLISCOPE //
const int freq = 50;			// 50Hz frequency (20ms) period for signal.
const int precision = 11;		// Number of bits of precision on servo.
const int maxServo = pow ( 2,precision ) / 16;
const int servoMid = 80;
const int servoShift = 30;

int error = 0;					// Accummulated Error (integral).
int lastErr = 0;

int intImage[size];				// Stores Raw Cam Data.
int heightMap[size];			// Height Map for Printing.
int derivative[size-1];			// Stores the Derivative of the Cam Data.
int highBright = 0;

// Functions //
void lineUpdate ();				// Pulls new data from the Cam.
void linePrint  ();				// Prints the Height Map to Serial.
void drive      ();				// PID & Drive control.

// Timer //
//elapsedMillis dt;
//PWMServo servo;					// The Servo Object.

// pwm clk 128,1
int main () 
{
	bool MANUAL = false;

	// Initializing pins.
	pinMode (clk, OUTPUT);
	pinMode (si, OUTPUT);
	
	// Attaching Servo.
	pinMode (srv, OUTPUT);

	// Initializing Serial.
	HWSerial.begin(BAUD);	

	while (1)
	{

		if (BTSTART)
		{
			char incomingByte='\0';
			char COM[2] = "";
			int count = 0;
	
			while (1)
			{
				if (HWSerial.available() > 0) {
					incomingByte = HWSerial.read();	
		            HWSerial.print("UART received:");
		            HWSerial.println(incomingByte, DEC);
	
					HWSerial.print(count + "\0");
	
					COM[count] = incomingByte;
					count++;
	
					if (count >= 2)
						count = 0;
				}
	
				if (COM[0] == 'S' && COM[1] == 'T')
				{
					HWSerial.println("Starting");
					break;
				}
				else if (COM[0] == 'M' && COM[1] == 'N')
				{
					HWSerial.println("Starting Manual");
					MANUAL = true;
					break;
				}
			
				delayMicroseconds(100);
	
			}
		}
			
		// Set Servo to 0 (testing).
		analogWriteFrequency(srv, freq);
		analogWriteResolution ( precision );
	
		//analogWrite (srv, servoMid + servoShift); // Max 64.
	
		// Testing motors.
		pinMode (drv, OUTPUT);
		pinMode (brk, OUTPUT);
	
		digitalWrite (brk, LOW);

		bool ON = true;

		while (1 && ON)
		{
			if (MANUAL)
			{
				analogWrite (drv, 0);
				analogWrite (srv, servoMid);

				if (HWSerial.available() > 0) {
					switch (HWSerial.read())
					{
						case 'w':
							analogWrite (drv, driveSpeed/1.5);
							break;
						case 'a':
							analogWrite (srv, servoMid - 20);
							break;
						case 'd':
							analogWrite (srv, servoMid + 20);
							break;
						case 'k':
							ON = false;
							break;
					}
				}

				delayMicroseconds(100000);
			}
			else
			{
				if (BTSTART)
				{
					if (HWSerial.available() > 0) {
						if (HWSerial.read() == 'k')
							break;
					}
				}		
	
				// Takes a new scan from cam.
				lineUpdate ();
	
				// PID.
				drive      ();
	
				analogWrite (drv, driveSpeed - abs(driveSpeed * (lastErr/40)));
			
			}
			if (SERIALBT && PRINTLINE)
			{
				// Print height map to Serial.
				linePrint  ();
			}
		}
	
		analogWrite (drv, 0);
		HWSerial.println ("END");

	}

}

void lineUpdate ()
{
	highBright = 0;

	// Set Start High.
	digitalWrite (si, HIGH);
	delayMicroseconds (d1);
	digitalWrite (clk, HIGH);
	delayMicroseconds (d1);
	digitalWrite (si, LOW);
	delayMicroseconds (d1);
	digitalWrite (clk, LOW);


	// Clearing frame buffer.
	for (int i = 1; i < size; i++) 
	{
		digitalWrite (clk, HIGH);
		delayMicroseconds (d1);
		digitalWrite (clk, LOW);	
		delayMicroseconds (d2);
	}

	// Begin Start, delay while Frame is captured, begin new read start.
	digitalWrite (si, HIGH);
	delay (clear);
	digitalWrite (si, LOW);
	delayMicroseconds (d1);
	digitalWrite (si, HIGH);
	delayMicroseconds (d1);
	digitalWrite (clk, HIGH);
	delayMicroseconds (d1);
	digitalWrite (si, LOW);
	delayMicroseconds (d1);
	digitalWrite (clk, LOW);	
	intImage[0] = analogRead(ao);

	// Read new image.
	for (int i = 1; i < size; i++) 
	{
		digitalWrite (clk, HIGH);
		delayMicroseconds (d1);
		digitalWrite (clk, LOW);

		intImage[i] = analogRead (ao);
		if (intImage[i] > highBright)
		{
			highBright = i;
		}
	}

}

void linePrint ()
{
	for (int i = 0; i < size; i++)
	{
		heightMap[i] = intImage[i] / mapScale;
	}

	for (int i = maxVal / mapScale - 1; i > 0; i--)
	{
		for (int j = 0; j < size; j++)
		{
			HWSerial.print ((heightMap[j] >= i)?"*":" ");
		}

		HWSerial.print('\n');
		//Serial.print( intImage[i] );
		//Serial.print( " " );
	}

	delay (endDelay);

	HWSerial.print( "\033[2J" );
	HWSerial.print( "\033[20;0H" );
}

// Using PID
// PID = k_p(err) + k_d(de/dt) + (sum of err)
void drive () 
{
	int high = 0, low = 0, err = 0;

	double PID;

	derivative[0] = intImage[0] - intImage[1];

	for (int i = 0; i < size - 4; i++)
	{
		derivative[i] = intImage[i] - intImage[i+1];	
		
		if ( derivative[i] > derivative[high] )
		{
			high = i;	
		}
		if ( derivative[i] < derivative[low] )
		{
			low = i;
		}
	}

	err = ((high + low)/2) - centerPoint;

	error += err;

	PID = (err * kp) + (((lastErr - err)) * kd) + (error * ki);

	if ( PID > -servoShift || PID < servoShift )
	{
		analogWrite (srv, servoMid + (int)PID);
	}

	lastErr = err;
	
	//dt = 0;

	//HWSerial.println (high);
	//HWSerial.println (low);
	HWSerial.print (err + '\n');
}
