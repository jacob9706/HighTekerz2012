#include "RampArm.h"
#include "Servo.h"
#include "Solenoid.h"

const static float SERVO_LOCK_POSITION = 102.0;

RampArm::RampArm(Servo* rampArmServo, Solenoid* rampArmSolenoid)
{
	_rampArmServo = rampArmServo;
	_rampArmSolenoid = rampArmSolenoid;
	
	_rampArmServo->SetAngle(0.0);
	_rampArmSolenoid->Set(false);
	
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

		if (elapsedTime < 800000)
		{
			if (rampGoingDown)
			{
				_rampArmSolenoid->Set(true);
			}
			if (rampGoingUp)
			{
				_rampArmServo->SetAngle(0.0);
			}
		}
		else if (elapsedTime < 1600000 && elapsedTime >= 800000)
		{
			if (rampGoingDown)
			{
				_rampArmServo->SetAngle(SERVO_LOCK_POSITION);
			}
			if (rampGoingUp)
			{
				_rampArmSolenoid->Set(false);				
			}
		}
		else 
		{
			IsRampMoving = false;
			rampGoingUp = false;
			rampGoingDown = false;
			IsRampUp = !IsRampUp;
		}	
	}	
}

void testFunction()
{
	
}
