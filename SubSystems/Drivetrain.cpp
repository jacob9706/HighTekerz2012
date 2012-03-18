#include "WPILib.h"
#include "Drivetrain.h"
#include "math.h"

const static float allowableInputDifference = .25;
const static float maxDriveChange = .015;
const static float ENCODER_FOLLOW_SCALE = .02;

Drivetrain::Drivetrain(UINT32 left1, UINT32 left2, UINT32 right1, UINT32 right2, Encoder* leftEncoder, Encoder* rightEncoder) 
: RobotDrive(left1, left2, right1, right2), DeadReckoner(leftEncoder, rightEncoder)
{
	rightMotorSetting = 0.0;
	leftMotorSetting = 0.0;
	SetExpiration(.1);

	speedMatchLeftCounterStart = 0;
	speedMatchRightCounterStart = 0;
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
}

//Returns 1 if positive, -1 if negative, 0 if zero
float Sign(float number)
{
	if(number >0)
	{
		return 1.0;
	}
	else if(number<0)
	{
		return -1.0;
	}
	else
	{
		return 0.0;
	}
}

float ScaleDriving(float inSpeed)
{
	float calculatedSpeed;
	if (fabs(inSpeed)<0.1)
	{
		calculatedSpeed = inSpeed*4;
	}
	else
	{
		(Sign(inSpeed)*(inSpeed*inSpeed*60/99))+(inSpeed*39/99);
	}

	return calculatedSpeed;
}

void Drivetrain::Periodic(float moveLeftInput, float moveRightInput, bool enableMatchEncoders)
{
	float leftInputSign = Sign(moveLeftInput);
	float rightInputSign = Sign(moveRightInput);

	//abs of inputs are within allowable tolerence and same sign
	if (fabs(moveLeftInput - moveRightInput) <= allowableInputDifference && leftInputSign == rightInputSign)
	{
		// if we were not in speed match, then start the counters
		if (!speedMatch)
		{
			speedMatchLeftCounterStart = leftCount;
			speedMatchRightCounterStart = rightCount;
		}

		// set speed match for encounter scaling later
		speedMatch = true;

		// get difference in favor of left
		float leftDiff = moveLeftInput - moveRightInput;
		float leftDiffSign = Sign(leftDiff);

		// when signs match.
		if (leftInputSign == leftDiffSign)
		{
			moveRightInput += leftDiff;
		}
		// when they dont - trust us, the math works....
		else
		{
			moveLeftInput -= leftDiff;
		}
	}
	else
	{
		speedMatch = false;
	}

	float newLeftSpeed = NewSpeed(leftMotorSetting, moveLeftInput);
	float newRightSpeed = NewSpeed(rightMotorSetting, moveRightInput);

	//	scaledLeft = ScaleDriving(newLeftSpeed);
	//	scaledRight = ScaleDriving(newRightSpeed);

	scaledLeft = newLeftSpeed;
	scaledRight = newRightSpeed;

	if (speedMatch && (leftInputSign == rightInputSign) && enableMatchEncoders)
	{ 
		int deltaLeft = leftCount - speedMatchLeftCounterStart;
		int deltaRight =  rightCount - speedMatchRightCounterStart;

		//moving forward
		if (leftInputSign > 0)
		{
			if (deltaLeft > deltaRight)
			{
				scaledLeft -= (deltaLeft - deltaRight) * ENCODER_FOLLOW_SCALE; 
			}
			else
			{
				scaledRight -= (deltaRight - deltaLeft) * ENCODER_FOLLOW_SCALE;
			}
		}
		//backward or stopped
		else
		{
			//left is ahead of right!!!
			if (deltaLeft < deltaRight)
			{
				scaledLeft -= (deltaLeft - deltaRight) * ENCODER_FOLLOW_SCALE; 
			}
			else
			{
				scaledRight -= (deltaRight - deltaLeft) * ENCODER_FOLLOW_SCALE;
			}
		}
	}

	RobotDrive::TankDrive(scaledLeft, scaledRight);

	leftMotorSetting = newLeftSpeed;
	rightMotorSetting = newRightSpeed;

	DeadReckoner::Update();
}
