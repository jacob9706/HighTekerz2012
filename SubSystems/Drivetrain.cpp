#include "WPILib.h"
#include "Drivetrain.h"
#include "math.h"

const static float allowableInputDifference = .25;
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
	//if(diff > maxDriveChange || diff < -maxDriveChange)
	if(true)
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
	//	else
	//	{
	//		return newRequestedSpeed;
	//	}
}

//Returns 1 if positive, -1 if negative, 0 if zero
int Sign(float number)
{
	if(number >0)
	{
		return 1;
	}
	else if(number<0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

void Drivetrain::Periodic(float moveLeftInput, float moveRightInput)
{
	int leftSign = Sign(moveLeftInput);
	int rightSign = Sign(moveRightInput);

	//abs of inputs are within allowable tolerence
	if (fabs(moveLeftInput - moveRightInput) <= allowableInputDifference)
	{
		//inputs have same sign
		if(leftSign == rightSign)
		{
			// left is les than right
			if(moveLeftInput<moveRightInput)
			{
				//set to highest value
				moveLeftInput = moveRightInput;
			}
			//falls into else if right is less than left or are same
			else
			{
				// set to highest value
				moveRightInput = moveLeftInput;
			}
		}
	}


	float newLeftSpeed = NewSpeed(leftMotorSetting, moveLeftInput);
	float newRightSpeed = NewSpeed(rightMotorSetting, moveRightInput);


	RobotDrive::TankDrive(newLeftSpeed, newRightSpeed);

	leftMotorSetting = newLeftSpeed;
	rightMotorSetting = newRightSpeed;

	DeadReckoner::Update();
}
