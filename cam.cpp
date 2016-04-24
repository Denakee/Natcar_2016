#include <bitset>
#include <cmath>
#include <string>
#include "Arduino.h"

#include "WProgram.h"
//#include "Servo.h"

// IO defines //
#define ao 	20
#define si	21
#define clk	22

#define srv 17

#define drv 11
#define brk 12

// Constants //
#define DRK 0
#define BRT 1
#define FWD 0
#define BCK 1

// Variables //
const int size = 128;			// Pixel Width of Line Cam.
const int maxVal = 1024;		// Largest Light Val Output.

const int clear = 15;			// Delay for Cam Exposure.
const int endDelay = 100;		// Large Delay for Image to Buffer over Serial (set to 0 when running).
const int time = 2;				// Time for duty cycle calc.
const float duty = 0.5;			// Duty cycle multiplier.
const int d1 = time * duty;		// Calculating Time for On Period.
const int d2 = time - d1;		// Calculating Time for Off Period.

const int mapScale = 50;		// Scaling of Map for Serial.

const float kp = 1;				// Proportional Correction Constant.
const float kd = 0;				// Derivative Multiplier.
const float ki = 0;				// Integral Multiplier.

// SERVO CONSTS DO NOT TOUCH UNLESS TESTING ON OSCILLISCOPE //
const int freq = 50;			// 50Hz frequency (20ms) period for signal.
const int precision = 11;		// Number of bits of precision on servo.
const int maxServo = pow ( 2,precision ) / 16;
const int servoMid = 85;
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

//PWMServo servo;					// The Servo Object.

// pwm clk 128,1
int main () 
{
	// Initializing pins.
	pinMode (clk, OUTPUT);
	pinMode (si, OUTPUT);
	
	// Attaching Servo.
	pinMode (srv, OUTPUT);

	// Initializing Serial.
	Serial.begin(9600);

	// Set Servo to 0 (testing).
	analogWriteFrequency(srv, freq);
	analogWriteResolution ( precision );

	//analogWrite (srv, servoMid + servoShift); // Max 64.

	// Testing motors.
	pinMode (drv, OUTPUT);
	pinMode (brk, OUTPUT);

	digitalWrite (brk, LOW);
	digitalWrite (drv, HIGH);

	while (1)
	{
		

		// Takes a new scan from cam.
		lineUpdate ();

		// PID.
		drive      ();

		// Print height map to Serial.
		linePrint  ();
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
			Serial.print ((heightMap[j] >= i)?"*":" ");
		}

		Serial.println();
		//Serial.print( intImage[i] );
		//Serial.print( " " );
	}

	delay (endDelay);

	Serial.print( "\033[2J" );
	Serial.print( "\033[20;0H" );
}

// Using PID
// PID = k_p(err) + k_d(de/dt) + (sum of err)
void drive () 
{
	int high = 0, low = 0, err = 0;

	double PID;

	derivative[0] = intImage[0] - intImage[1];

	for (int i = 0; i < size - 1; i++)
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

	err = ((high + low)/2) - 64;

	error += err;

	PID = (err * kp) + (((lastErr - err)/20) * kd) + (error * ki);

	if ( PID > -servoShift || PID < servoShift )
	{
		analogWrite (srv, servoMid + (int)PID);
	}

	lastErr = err;
	
	Serial.println (err);
}
