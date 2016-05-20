#ifndef __CAMCONTROL_H__
#define __CAMCONTROL_H__

#include "Arduino.h"

class CamControl
{
	int clk, ao, si, d, pix;

	public:
		CamControl();
		CamControl( int, int, int, int );

		void lineUpdate( int*, int );
		void clearBuffer();
};

#endif
