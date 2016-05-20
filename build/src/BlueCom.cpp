/* Jank Wheels Bluetooth Communication
 * Created by: Greg Balke
 * Date: May 2016
 * File: BlueCom.cpp
 */

#include <cmath>
#include "BlueCom.h"
#include "HardwareSerial.h"

#define HWSerial Serial1

BlueCom::BlueCom () {}

bool BlueCom::settings ( carVars * CVS )
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
					HWSerial.println (CVS->kp);
					HWSerial.print ("kd: ");
					HWSerial.println (CVS->kd);
					HWSerial.print ("ki: ");
					HWSerial.println (CVS->ki);
					HWSerial.print ("speed: ");
					HWSerial.println (CVS->driveSpeed);
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
					CVS->servoPos;
					break;
				case 'k':
					CVS->mode = OFF;
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
						CVS->kp = num;
						break;
					case 2:
						HWSerial.print ("kd: ");
						HWSerial.println (num);
						CVS->kd = -num;
						break;
					case 3:
						CVS->ki = num;
						HWSerial.print ("ki: ");
						HWSerial.println (num);
						break;
					case 4:
						CVS->driveSpeed = (int) (num * 10);
						HWSerial.print ("speed: ");
						HWSerial.println (CVS->driveSpeed);
						break;
					case 5:
						CVS->speedDiv = num * 10;
						HWSerial.print ("speedDiv: ");
						HWSerial.println (CVS->speedDiv);
						break;
				}
				num = 0;
				setting = 0;
				count = 0;
			}	
		}
	}	

	return true;
}
