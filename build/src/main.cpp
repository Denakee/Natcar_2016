/* Jank Wheels Drive Software
 * Created By: Greg Balke
 * Date: April 2016
 */ 

#include <cmath>
#include "Arduino.h"
#include "HardwareSerial.h"

#include "WProgram.h"

#include "CamControl.h"
#include "BlueCom.h"
#include "DataStruct.h"

//#include "Servo.h"

//Working Values:
//	kp = 0.58			/	0.7
//	kd = -0.58, -0.53	/	1.4
//	ki = 0.01			/	0.01

//250, 0, 0.7, 1.4, 0.0, 80, OFF

carVars CVS = { 600,		// Drive Speed for motors.
				0,			// Servo Angle.
				0.71,		// Proportional Correction Constant.
				3.85,		// Derivative Multiplier.
				0.0,		// Integral Multiplier.
				50,			// Speed Divider.
				OFF};		// Drive State.

const int centerPoint = 60;

// BLUETOOTH START SETTING //
#define BTSTART 1
#define SERIALBT 1

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
#define PRINTLINE 0
#define BAUD 115200

#if SERIALBT
#define HWSerial Serial1
#else
#define HWSerial Serial
#endif

// Variables //
const int size = 128;			// Pixel Width of Line Cam.
const int maxVal = 1024;		// Largest Light Val Output.

const int clear = 2000;			// Delay for Cam Exposure.
const int endDelay = 100;		// Large Delay for Image to Buffer over Serial (set to 0 when running).

const int mapScale = 5;		// Scaling of Map for Serial.

// SERVO CONSTS DO NOT TOUCH UNLESS TESTING ON OSCILLISCOPE //
const int freq = 50;			// 50Hz frequency (20ms) period for signal.
const int precision = 11;		// Number of bits of precision on servo.
const int maxServo = pow ( 2,precision ) / 16;
const int servoMid = 80;
const int servoShift = 30;

int error = 0;					// Accummulated Error (integral).
int lastErr = 0;

int pixMap[size];				// Stores Raw Cam Data.
int heightMap[size];			// Height Map for Printing.
int derivative[size-1];			// Stores the Derivative of the Cam Data.
int highBright = 0;

// Functions //
void linePrint  ();				// Prints the Height Map to Serial.
void drive      ( BlueCom *, CamControl *);				// Drive control.
int  PID		();				// PID Drive.
void manualDrive();				// Drives Manual.

// Timer //
//elapsedMillis dt;
//PWMServo servo;					// The Servo Object.

// pwm clk 128,1
int main () 
{
	
	// Initializing pins.
	pinMode (clk, OUTPUT);
	pinMode (si, OUTPUT);
	
	// Attaching Servo.
	pinMode (srv, OUTPUT);

	CamControl *cam;
	BlueCom *bt;

	cam = new CamControl( clk, ao, si, size );

	if (SERIALBT)
		bt = new BlueCom (BAUD);
	else
	{
		// Initializing Serial.
		HWSerial.begin(BAUD);	

		//delay ( 2000 );

		HWSerial.println("Booting");
	}
		
	// Set Servo to 0 (testing).
	analogWriteFrequency(srv, freq);
	analogWriteResolution ( precision );
	
	//analogWrite (srv, servoMid + servoShift); // Max 64.
	
	// Testing motors.
	pinMode (drv, OUTPUT);
	pinMode (brk, OUTPUT);
	
	digitalWrite (brk, LOW);


	// Primary Loop.	
	while (1)
	{
		
		CVS.mode = OFF;

		if (BTSTART)
		{
			while (!(CVS.mode = bt->BTCheck()))
			{
				//lineUpdate();
				delayMicroseconds(100);
			}

		}
		else
		{
			CVS.mode = 1;
		}

		digitalWrite (brk, LOW);

		while (CVS.mode)
		{	
			if (CVS.mode == ON)
				drive( bt, cam );
			else if (CVS.mode == MANUAL)
				manualDrive();
			else if (CVS.mode == SETTINGS)
				bt->settings( CVS );
		}
		
		//HWSerial.clear();

		analogWrite (drv, 0);
		analogWrite (srv, servoMid);
		
		digitalWrite (brk, HIGH);

		HWSerial.print ("END. Sum Error: ");
		HWSerial.println (error);
		HWSerial.flush();
		error = 0;
	}

}

void linePrint ()
{
	for (int i = 0; i < size; i++)
	{
		heightMap[i] = pixMap[i] / mapScale;
	}

	for (int i = maxVal / mapScale - 1; i > 0; i--)
	{
		for (int j = 0; j < size; j++)
		{
			HWSerial.print ((heightMap[j] >= i)?"*":" ");
		}

		HWSerial.println("\n");
		//Serial.print( intImage[i] );
		//Serial.print( " " );
	}

	delay (endDelay);

	HWSerial.print( "\033[2J" );
	HWSerial.print( "\033[20;0H" );
}

void drive( BlueCom * bt, CamControl * cam )
{
	static int lastSpeed = CVS.driveSpeed;
	int newSpeed = 0, calcSpeed = 0;

	if (BTSTART)
	{
		if (!bt->checkKill())
		{
			lastSpeed = CVS.driveSpeed;
			CVS.mode = OFF;
		}
	}		

	// Takes a new scan from cam.
	cam->clearBuffer ();
	cam->lineUpdate ( pixMap, clear );

	// PID.
	int turn = PID();

	if ( turn > -servoShift || turn < servoShift )
	{
		analogWrite (srv, servoMid + turn);
	}
	
	calcSpeed = CVS.driveSpeed - abs(CVS.driveSpeed * (lastErr/CVS.speedDiv));

	//if ( calcSpeed < lastSpeed - CVS.driveSpeed/5 )
	//{
	//	newSpeed = 0;
	//}
	if (calcSpeed > lastSpeed)
	{
		newSpeed = lastSpeed + 3;
	}
	else
	{
		newSpeed = calcSpeed;
	}

	
	if ( lastErr >= 28 )
	{
		analogWrite (drv, 0);
		analogWrite (brk, 2023);
	}
	else
	{
		analogWrite (brk, 0);
		analogWrite (drv, newSpeed);
		lastSpeed = newSpeed;
	}
	
	if (PRINTLINE)
	{
		// Print height map to Serial.
		linePrint  ();
	}
}

// Using PID
// PID = k_p(err) + k_d(de/dt) + (sum of err)
int PID () 
{
	int high = 0, low = 0, err = 0;

	float PID;

	derivative[0] = pixMap[0] - pixMap[1];

	for (int i = 0; i < size - 4; i++)
	{
		derivative[i] = pixMap[i] - pixMap[i+1];	
		
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

	if (abs(high-low) > 20) //|| 20 - abs(err) < lastErr) 
	{
		err = lastErr;
	}

	if (!( abs(error + err) > 250 ))
	{
		error += err;
	}

	PID = (err * CVS.kp) + (((lastErr - err)) * (-CVS.kd)) + (error * CVS.ki);

	lastErr = err;

	return (int) PID;		
}

void manualDrive ()
{
	analogWrite (drv, 0);
	//analogWrite (srv, servoMid);

	if (HWSerial.available() > 0) {
		switch (HWSerial.read())
		{
			case 'w':
				analogWrite (drv, CVS.driveSpeed/1.5);
				break;
			case 'a':
				analogWrite (srv, servoMid - 20);
				break;
			case 'd':
				analogWrite (srv, servoMid + 20);
				break;
			case 's':
				analogWrite (srv, servoMid);
				break;
			case 'k':
				CVS.mode = OFF;
				break;
		}
	}

	delayMicroseconds(100);
}


