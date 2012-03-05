#include "WPILib.h"
#include "Drivetrain.h"
#include "math.h"

const static float allowableInputDifference = .1;
const static float maxDriveChange = .025;

Drivetrain::Drivetrain(UINT32 left1, UINT32 left2, UINT32 right1, UINT32 right2, Encoder* leftEncoder, Encoder* rightEncoder) 
: RobotDrive(left1, left2, right1, right2), DeadReckoner(leftEncoder, rightEncoder)
{
	rightMotorSetting = 0.0;
	leftMotorSetting = 0.0;
	SetExpiration(.1);
}

Drivetrain::~Drivetrain()
{

}

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

void Drivetrain::Periodic(float moveLeftInput, float moveRightInput)
{
	if (fabs(moveLeftInput - moveRightInput) < allowableInputDifference)
	{
		if (fabs(moveLeftInput) > fabs(moveRightInput))
		{ 
			moveRightInput = moveLeftInput;
		}
		else
		{
			moveLeftInput = moveRightInput;
		}
	}

	float newLeftSpeed = NewSpeed(leftMotorSetting, moveLeftInput);
	float newRightSpeed = NewSpeed(rightMotorSetting, moveRightInput);

	
	RobotDrive::TankDrive(newLeftSpeed, newRightSpeed);
	
	leftMotorSetting = newLeftSpeed;
	rightMotorSetting = newRightSpeed;

	DeadReckoner::Update();
}
