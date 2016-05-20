/* Jank Wheels Bluetooth Communication
 * Created by: Greg Balke
 * Date: May 2016
 * File: BlueCom.h
 */

#ifndef __BLUECOM_H__
#define __BLUECOM_H__

#define HWSerial Serial1 // Serial 1 is the hardware serial on pin 0 and 1.

#include "DataStruct.h"

class BlueCom
{
	public:
		BlueCom( int );
		
		int BTCheck();
		bool settings(carVars &);
		bool checkKill();
};

#endif
