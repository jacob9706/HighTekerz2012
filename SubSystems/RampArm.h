#ifndef __RAMPARM_H
#define __RAMPARM_H

class Servo;
class Solenoid;

#include "WPILib.h"

class RampArm
{
public:
	RampArm(Servo* rampArmServo, 
					Solenoid* rampArmSolenoid);
	~RampArm();
	void Reset();
	void PeriodicSystem(bool ChangeRampState);
	bool IsRampUp;
	bool IsRampMoving;

private:
	bool rampGoingUp;
	bool rampGoingDown;
	Servo* _rampArmServo; 
	Solenoid* _rampArmSolenoid;
	UINT startTime;
	UINT elapsedTime;
};

#endif
