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
//	kp = 0.58			/	0.7
//	kd = -0.58, -0.53	/	1.4
//	ki = 0.01			/	0.01
volatile int driveSpeed = 250;		// Drive speed for motors.
volatile float kp = 0.7;			// Proportional Correction Constant.
volatile float kd = -1.4;// -0.6;			// Derivative Multiplier.
volatile float ki = 0.0;//0.05;				// Integral Multiplier.
volatile float speedDiv = 80;
const int centerPoint = 60;

// BLUETOOTH START SETTING //
#define BTSTART 0
#define SERIALBT 0

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

const int clear = 5000;			// Delay for Cam Exposure.
const int endDelay = 100;			// Large Delay for Image to Buffer over Serial (set to 0 when running).
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

int MODE = 0;					// 0 = Automated, 1 = Manual, 2 = Settings
bool ON = false;

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
void manualDrive();				// Drives Manual.
bool settings	();
bool BTCheck	();


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

	// Initializing Serial.
	HWSerial.begin(BAUD);	

	//delay ( 2000 );

	HWSerial.println("Booting");

	//HWSerial.print("AT+PIN4678");
		
	// Set Servo to 0 (testing).
	analogWriteFrequency(srv, freq);
	analogWriteResolution ( precision );
	
	//analogWrite (srv, servoMid + servoShift); // Max 64.
	
	// Testing motors.
	pinMode (drv, OUTPUT);
	pinMode (brk, OUTPUT);
	
	digitalWrite (brk, LOW);
		
	while (1)
	{
		
		MODE = 0;

		if (BTSTART)
		{
			while (BTCheck())
			{
				//lineUpdate();
				delayMicroseconds(10000);

				//drive ();
			}
		}
		
		ON = true;

		digitalWrite (brk, LOW);

		while (ON)
		{	
			if (MODE == 0)
			{
				if (BTSTART)
				{
					if (!settings())
						break;
				}		
	
				// Takes a new scan from cam.
				lineUpdate ();
	
				// PID.
				drive      ();
	
				analogWrite (drv, driveSpeed - abs(driveSpeed * (lastErr/speedDiv)));
				//analogWrite (drv, driveSpeed);
				
				if (PRINTLINE)
				{
					// Print height map to Serial.
					linePrint  ();
				}	
			}
			else if (MODE == 1)
				manualDrive();
			else if (MODE == 2)
				settings();
		}
		
		//HWSerial.clear();

		analogWrite (drv, 0);
		
		digitalWrite (brk, HIGH);

		HWSerial.print ("END. Sum Error: ");
		HWSerial.println (error);
		MODE = 0;
		error = 0;
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
	delayMicroseconds (clear);
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

		HWSerial.println("\n");
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

	if (abs(high-low) > 20 && 0) //|| 20 - abs(err) < lastErr) 
	{
		err = lastErr;
	}
	error += err;

	PID = (err * kp) + (((lastErr - err)) * kd) + (error * ki);

	if ( PID > -servoShift || PID < servoShift )
	{
		analogWrite (srv, servoMid + (int)PID);
	}

	lastErr = err;
	
	//dt = 0;
	if (ON)
	{
		//HWSerial.println (high);
		//HWSerial.println (low);
		//HWSerial.print (err);
		//HWSerial.print (" ");
	}
}

void manualDrive ()
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

bool settings ()
{
	static int setting = 0;
	static int count = 0;
	static char digit = ' ';
	static float num = 0;
	
	if (HWSerial.available() > 0) {
		if (setting == 0)
		{
			switch (HWSerial.read())
			{
				case 'w':
					HWSerial.print ("kp: ");
					HWSerial.println (kp);
					HWSerial.print ("kd: ");
					HWSerial.println (kd);
					HWSerial.print ("ki: ");
					HWSerial.println (ki);
					HWSerial.print ("speed: ");
					HWSerial.println (driveSpeed);
					break;
				case 'p':
					setting = 1;
					break;
				case 'd':
					setting = 2;				
					break;
				case 'i':
					setting = 3;
					break;
				case 's':
					setting = 4;
					break;
				case 'v':
					setting = 5;
					break;
				case '0':
					analogWrite ( srv, servoMid );
					break;
				case 'k':
					ON = false;
					return false;
			}
		}
		else
		{
			digit = HWSerial.read();
			HWSerial.println(digit);
			if (isdigit(digit) && digit != 'k')
			{
				num += ((digit-'0') / pow(10, count - 1));
				count ++;
			}
			else if (digit == 'e')
			{
				switch (setting)
				{
					case 1:
						HWSerial.print ("kp: ");
						HWSerial.println (num);
						kp = num;
						break;
					case 2:
						HWSerial.print ("kd: ");
						HWSerial.println (num);
						kd = -num;
						break;
					case 3:
						ki = num;
						HWSerial.print ("ki: ");
						HWSerial.println (num);
						break;
					case 4:
						driveSpeed = (int) (num * 10);
						HWSerial.print ("speed: ");
						HWSerial.println (driveSpeed);
						break;
					case 5:
						speedDiv = num * 10;
						HWSerial.print ("speedDiv: ");
						HWSerial.println (speedDiv);
						break;
				}
				num = 0;
				setting = 0;
				count = 0;
			}	
		}
	}	

	delayMicroseconds(100000);

	return true;
}

bool BTCheck ()
{
	static char incomingByte='\0';
	static char COM[2] = "";
	static int count = 0;
	
	if (HWSerial.available() > 0) {
		incomingByte = HWSerial.read();	
        HWSerial.print("UART received: ");
        HWSerial.println(incomingByte);

		HWSerial.print(count + "\0");

		COM[count] = incomingByte;
		count++;

		if (count >= 2)
			count = 0;
	}

	if (COM[0] == 'S' && COM[1] == 'T')
	{
		HWSerial.println("Starting Drive");
		MODE = 0;
		COM[0] = '\0';
		COM[1] = '\0';
		return false;
	}
	else if (COM[0] == 'M' && COM[1] == 'N')
	{
		HWSerial.println("Starting Manual");
		MODE = 1;
		COM[0] = '\0';
		COM[1] = '\0';
		return false;
	}
	else if (COM[0] == 'P' && COM[1] == 'D')
	{
		HWSerial.println("PID Settings");
		MODE = 2;
		COM[0] = '\0';
		COM[1] = '\0';
		return false;
	}

	return true;
}



