#include "WPILib.h"
#include "Drivetrain.h"
#include "math.h"

const static float maxDriveChange = .04;

Drivetrain::Drivetrain(UINT32 left1, UINT32 left2, UINT32 right1, UINT32 right2, Encoder* leftEncoder, Encoder* rightEncoder) 
: RobotDrive(left1, left2, right1, right2), DeadReckoner(leftEncoder, rightEncoder)
{
	rightMotorSetting = 0.0;
	leftMotorSetting = 0.0;
}

Drivetrain::~Drivetrain()
{

}

// ignore positioning
float NewSpeed(float oldSpeed,float newRequestedSpeed)
{
	float diff = newRequestedSpeed - oldSpeed;
	if(diff > maxDriveChange || diff < -maxDriveChange)
	{
		if(diff > 0)
		{
			return oldSpeed + maxDriveChange;
		}
		else
		{
			return oldSpeed - maxDriveChange;
		}
	}
	else
	{
		return newRequestedSpeed;
	}
}
//-----------

void Drivetrain::Periodic(float moveLeft, float moveRight)
{
	RobotDrive::TankDrive(NewSpeed(leftMotorSetting, moveLeft), NewSpeed(rightMotorSetting, moveRight));

	DeadReckoner::Update();
}
