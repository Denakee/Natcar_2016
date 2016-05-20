#ifndef __DATASTRUCT_H__
#define __DATASTRUCT_H__

#define SETTINGS 3
#define MANUAL 2
#define ON 1
#define OFF 0

struct carVars
{
	volatile int driveSpeed, servoPos;
	volatile float kp, kd, ki, speedDiv;
	volatile int mode;
};

#endif
