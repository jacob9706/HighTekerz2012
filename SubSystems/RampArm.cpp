#include "RampArm.h"
#include "Servo.h"
#include "Solenoid.h"

const static float SERVO_LOCK_POSITION = 102.0;

RampArm::RampArm(Servo* rampArmServo, Solenoid* rampArmSolenoid)
{
	_rampArmServo = rampArmServo;
	_rampArmSolenoid = rampArmSolenoid;
	
	IsRampUp = true;
	IsRampMoving = false;
	rampGoingUp = false;
	rampGoingDown = false;
	startTime = 0;
}

RampArm::~RampArm()
{
}

void RampArm::Reset()
{
}

void RampArm::PeriodicSystem(bool ChangeRampState)
{
	if (!IsRampMoving && ChangeRampState)
	{
		startTime = GetFPGATime();
		IsRampMoving = true;
		if (IsRampUp)
		{
			rampGoingDown = true;
		}
		else
		{
			rampGoingUp = true;
		}		
	}
	
	if (IsRampMoving)
	{
		elapsedTime = GetFPGATime() - startTime;
		if (elapsedTime > 2000000)
		{
			IsRampMoving = false;
			rampGoingUp = false;
			rampGoingDown = false;					
		}
	}
	
	if (rampGoingUp)
	{
		
	}
	
	if (rampGoingDown)
	{
		
	}
}

void testFunction()
{
	
}
