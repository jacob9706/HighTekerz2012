#include "Shooter.h"
#include "Victor.h"
#include "..\LowLevel\Encoder3574.h"
#include "Solenoid.h"

Shooter::Shooter(Victor *topWheel,
		Victor *bottomWheel,
		Victor *rotationMotor,
		Victor *pitchMotor,
		Encoder3574 *topWheelEncoder,
		Encoder3574 *bottomWheelEncoder,
		Encoder3574 *rotationEncoder,
		Encoder3574 *pitchEncoder,
		Solenoid *kicker)
{
	_topWheel = topWheel;
	_rotationMotor = rotationMotor;
	_pitchMotor = pitchMotor;
	_topWheelEncoder = topWheelEncoder;
	_bottomWheelEncoder = bottomWheelEncoder;
	_rotationEncoder = rotationEncoder;
	_pitchEncoder = pitchEncoder;
	_kicker = kicker;
	
	IsShooting = false;
}

Shooter::~Shooter()
{

}


void Shooter::Periodic(bool shoot)
{
}

void Shooter::ManualRotate(float axis)
{
	_rotationMotor->Set(-axis/2);
}

void Shooter::ManualPitch(float axis)
{
	_pitchMotor->Set(-axis/2.1);
}


