/* Jank Wheels Bluetooth Communication
 * Created by: Greg Balke
 * Date: May 2016
 * File: BlueCom.cpp
 */

#include <cmath>
#include <stdlib.h>
#include "BlueCom.h"
#include "HardwareSerial.h"
#include "core_pins.h"

#define HWSerial Serial1

BlueCom::BlueCom ( int baud ) 
{
	HWSerial.begin (baud);
	HWSerial.println ("Booting");
}

int BlueCom::BTCheck ()
{
	static char incomingByte='\0';
	static char COM[2] = "";
	
	if (HWSerial.available() > 0) {
		incomingByte = HWSerial.read();	
        HWSerial.print(incomingByte);
	
		COM[0] = COM[1];
		COM[1] = incomingByte;
	}

	if (COM[0] == 'S' && COM[1] == 'T')
	{
		HWSerial.println();
		HWSerial.println("Starting Drive");
		COM[0] = '\0';
		COM[1] = '\0';
		return ON;
	}
	else if (COM[0] == 'M' && COM[1] == 'N')
	{
		HWSerial.println();
		HWSerial.println("Starting Manual");
		COM[0] = '\0';
		COM[1] = '\0';
		return MANUAL;
	}
	else if (COM[0] == 'P' && COM[1] == 'D')
	{
		HWSerial.println();
		HWSerial.println("PID Settings");
		COM[0] = '\0';
		COM[1] = '\0';
		return SETTINGS;
	}
	
	return OFF;
}

bool BlueCom::checkKill ()
{
	while (HWSerial.available() > 0) {
		if (HWSerial.read() == 'k')
			return false;
	}
	
	return true;
}

bool BlueCom::settings ( carVars & CVS )
{
	static int setting = 0;
	static int count = 0;
	static char digit = ' ';
	static char * digits = new char[32];
	static float num = 0;
	
	if (HWSerial.available() > 0) {
		if (setting == 0)
		{
			switch (HWSerial.read())
			{
				case 'w':
					HWSerial.print ("kp: ");
					HWSerial.println (CVS.kp);
					HWSerial.print ("kd: ");
					HWSerial.println (CVS.kd);
					HWSerial.print ("ki: ");
					HWSerial.println (CVS.ki);
					HWSerial.print ("speed: ");
					HWSerial.println (CVS.driveSpeed);
					break;
				case 'p':
					HWSerial.print ("kp: ");
					setting = 1;
					break;
				case 'd':
					HWSerial.print ("kd ");
					setting = 2;			
					break;
				case 'i':
					HWSerial.print ("ki: ");
					setting = 3;
					break;
				case 's':
					HWSerial.print ("speed: ");
					setting = 4;
					break;
				case 'v':
					HWSerial.print ("speedDiv: ");
					setting = 5;
					break;
				case '0':
					CVS.servoPos;
					break;
				case 'k':
					CVS.mode = OFF;
					return false;
			}
		}
		else
		{
			
			digit = HWSerial.read();
			
			if (isdigit(digit) || digit == '.')
			{
				HWSerial.print(digit);

				digits[count] = digit;
				count ++;
			}
			else if (digit == 'e')
			{
				HWSerial.println();
				
				digits[count] = '\0';
				num = atof(digits);

				switch (setting)
				{					
					case 1:
						CVS.kp = num;
						break;
					case 2:
						CVS.kd = num;
						break;
					case 3:
						CVS.ki = num;
						break;
					case 4:
						CVS.driveSpeed = (int) num;
						break;
					case 5:
						CVS.speedDiv = num;
						break;
				}

				HWSerial.println("OK");

				num = 0;
				setting = 0;
				count = 0;
				digits[0] = 0;
			}	
		}
	}		

	return true;
}
