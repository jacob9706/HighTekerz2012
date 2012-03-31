<<<<<<< HEAD
#include "WPILib.h"
#include "Drivetrain.h"
#include "math.h"

const static float allowableInputDifference = .25;
const static float maxDriveChange = .02;
const static float ENCODER_FOLLOW_SCALE = .02;

Drivetrain::Drivetrain(UINT32 left1, UINT32 left2, UINT32 right1, UINT32 right2, Encoder* leftEncoder, Encoder* rightEncoder) 
: RobotDrive(left1, left2, right1, right2), DeadReckoner(leftEncoder, rightEncoder)
{
	rightMotorSetting = 0.0;
	leftMotorSetting = 0.0;
	SetExpiration(0.5);//changed from .1 to .5 for debuging

	speedMatchLeftCounterStart = 0;
	speedMatchRightCounterStart = 0;
}

Drivetrain::~Drivetrain()
{

}

float NewSpeed(float oldSpeed,float newRequestedSpeed)
{
	// if new speed is toward positive
	if(newRequestedSpeed - oldSpeed > 0)
	{
		return oldSpeed + maxDriveChange;
	}
	else
	{
		return oldSpeed - maxDriveChange;
	}
}

//Returns 1 if positive, -1 if negative, 0 if zero
float Sign(float number)
{
	if(number > 0)
	{
		return 1.0;
	}
	else if(number < 0)
	{
		return -1.0;
	}
	else
	{
		return 0.0;
	}
}

float in1 = .05;
float out1 = .35;
float in2 = .96;
float out2 = .70;

float ratio0 = out1/in1;
float ratio1 = (out2 - out1)/(in2 - in1);
float ratio1Add = out1-(ratio1*in1);
float ratio2 = (1-out2)/(1-in2);
float ratio2Add = out2-(ratio2*in2);

float ScaleDriving(float inSpeed)
{			
	float calculatedSpeed;
	if (fabs(inSpeed) < in1)
	{
		calculatedSpeed = inSpeed*ratio0;
	}
	else if (fabs(inSpeed) < in2)
	{
		calculatedSpeed = inSpeed * ratio1 + Sign(inSpeed) * ratio1Add;
	}
	else
	{
		calculatedSpeed = inSpeed * ratio2 + Sign(inSpeed) * ratio2Add;
	}
	return calculatedSpeed;
}

void Drivetrain::Periodic(float moveLeftInput, float moveRightInput, bool enableMatchEncoders, bool STOP)
{
	moveLeftInput = -moveLeftInput;
	moveRightInput = -moveRightInput;
	
	float leftInputSign = Sign(moveLeftInput);
	float rightInputSign = Sign(moveRightInput);

	//abs of inputs are within allowable tolerence and same sign
	if (fabs(moveLeftInput - moveRightInput) <= allowableInputDifference && leftInputSign == rightInputSign)
	{
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

	float newLeftSpeed = NewSpeed(leftMotorSetting, moveLeftInput);
	float newRightSpeed = NewSpeed(rightMotorSetting, moveRightInput);

	scaledLeft = ScaleDriving(newLeftSpeed);
	scaledRight = ScaleDriving(newRightSpeed);


	if ((leftInputSign == rightInputSign) && enableMatchEncoders)
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
	
	if (STOP)
	{
		newLeftSpeed = 0.0;
		newRightSpeed = 0.0;
		scaledLeft = 0.0;
		scaledRight = 0.0;
	}

	RobotDrive::TankDrive(scaledLeft, scaledRight);

	leftMotorSetting = newLeftSpeed;
	rightMotorSetting = newRightSpeed;

	DeadReckoner::Update();
}
=======
#include "WPILib.h"
#include "Drivetrain.h"
#include "math.h"

const static float allowableInputDifference = .25;
const static float maxDriveChange = .02;
const static float ENCODER_FOLLOW_SCALE = .02;

Drivetrain::Drivetrain(UINT32 left1, UINT32 left2, UINT32 right1, UINT32 right2, Encoder* leftEncoder, Encoder* rightEncoder) 
: RobotDrive(left1, left2, right1, right2), DeadReckoner(leftEncoder, rightEncoder)
{
	rightMotorSetting = 0.0;
	leftMotorSetting = 0.0;
	SetExpiration(0.5);//changed from .1 to .5 for debuging

	speedMatchLeftCounterStart = 0;
	speedMatchRightCounterStart = 0;
}

Drivetrain::~Drivetrain()
{

}

float NewSpeed(float oldSpeed,float newRequestedSpeed)
{
	// if new speed is toward positive
	if(newRequestedSpeed - oldSpeed > 0)
	{
		return oldSpeed + maxDriveChange;
	}
	else
	{
		return oldSpeed - maxDriveChange;
	}
}

//Returns 1 if positive, -1 if negative, 0 if zero
float Sign(float number)
{
	if(number > 0)
	{
		return 1.0;
	}
	else if(number < 0)
	{
		return -1.0;
	}
	else
	{
		return 0.0;
	}
}

float in1 = .05;
float out1 = .35;
float in2 = .96;
float out2 = .70;

float ratio0 = out1/in1;
float ratio1 = (out2 - out1)/(in2 - in1);
float ratio1Add = out1-(ratio1*in1);
float ratio2 = (1-out2)/(1-in2);
float ratio2Add = out2-(ratio2*in2);

float ScaleDriving(float inSpeed)
{			
	float calculatedSpeed;
	if (fabs(inSpeed) < in1)
	{
		calculatedSpeed = inSpeed*ratio0;
	}
	else if (fabs(inSpeed) < in2)
	{
		calculatedSpeed = inSpeed * ratio1 + Sign(inSpeed) * ratio1Add;
	}
	else
	{
		calculatedSpeed = inSpeed * ratio2 + Sign(inSpeed) * ratio2Add;
	}
	return calculatedSpeed;
}

void Drivetrain::Periodic(float moveLeftInput, float moveRightInput, bool enableMatchEncoders, bool STOP)
{
	moveLeftInput = -moveLeftInput;
	moveRightInput = -moveRightInput;
	
	float leftInputSign = Sign(moveLeftInput);
	float rightInputSign = Sign(moveRightInput);

	//abs of inputs are within allowable tolerence and same sign
	if (fabs(moveLeftInput - moveRightInput) <= allowableInputDifference && leftInputSign == rightInputSign)
	{
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

	float newLeftSpeed = NewSpeed(leftMotorSetting, moveLeftInput);
	float newRightSpeed = NewSpeed(rightMotorSetting, moveRightInput);

	scaledLeft = ScaleDriving(newLeftSpeed);
	scaledRight = ScaleDriving(newRightSpeed);


	if ((leftInputSign == rightInputSign) && enableMatchEncoders)
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
	
	if (STOP)
	{
		newLeftSpeed = 0.0;
		newRightSpeed = 0.0;
		scaledLeft = 0.0;
		scaledRight = 0.0;
	}

	RobotDrive::TankDrive(scaledLeft, scaledRight);

	leftMotorSetting = newLeftSpeed;
	rightMotorSetting = newRightSpeed;

	DeadReckoner::Update();
}
>>>>>>> 5cf0ff622f058e49bd836e49cbd450d29583cd9d
