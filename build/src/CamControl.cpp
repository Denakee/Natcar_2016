#include "CamControl.h"

CamControl::CamControl ( int clock, int analog, int signal, int numPix )
{
	clk = clock;
	ao = analog;
	si = signal;

	pix = numPix;

	d = 1;
}

void CamControl::lineUpdate ( int * pixels, int delay )
{
	// Begin Start, delay while Frame is captured, begin new read start.
	digitalWrite (si, HIGH);

	// Exposure Delay
	delayMicroseconds (delay);

	digitalWrite (si, LOW);
	delayMicroseconds (d);
	digitalWrite (si, HIGH);
	delayMicroseconds (d);
	digitalWrite (clk, HIGH);
	delayMicroseconds (d);
	digitalWrite (si, LOW);
	delayMicroseconds (d);
	digitalWrite (clk, LOW);	
	pixels[0] = analogRead(ao);

	// Read new image.
	for (int i = 1; i < pix; i++) 
	{
		digitalWrite (clk, HIGH);
		delayMicroseconds (d);
		digitalWrite (clk, LOW);

		pixels[i] = analogRead (ao);
	}
}

void CamControl::clearBuffer ()
{
	// Set Start High.
	digitalWrite (si, HIGH);
	delayMicroseconds (d);
	digitalWrite (clk, HIGH);
	delayMicroseconds (d);
	digitalWrite (si, LOW);
	delayMicroseconds (d);
	digitalWrite (clk, LOW);

	// Clearing frame buffer.
	for (int i = 1; i < pix; i++) 
	{
		digitalWrite (clk, HIGH);
		delayMicroseconds (d);
		digitalWrite (clk, LOW);	
		delayMicroseconds (d);
	}
}


