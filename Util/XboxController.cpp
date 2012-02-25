#include "XboxController.h"
#include "Joystick.h"
#include "math.h"

XboxController::XboxController(int port)
{
	xbox = new Joystick(port);
	joystickCurve = 1.0;
}

XboxController::~XboxController()
{

}


/*----------------------------------------*/
/*
 * This is where the sticks
 * and triggers are gotten
 */

float XboxController::GetLeftX()
{
	return pow(xbox->GetRawAxis(1), joystickCurve);
}

float XboxController::GetLeftY()
{
	return pow(xbox->GetRawAxis(2), joystickCurve);
}

float XboxController::GetLeftTrigger()
{
	float trigger = pow(xbox->GetRawAxis(3), joystickCurve);
	
	//adds a little bit of tolerance of .1 for trigger
	if(trigger >= .1)
	{
		return trigger;
	}
	
	else
	{
		return 0;
	}
}

float XboxController::GetRightTrigger()
{
	float trigger = pow(xbox->GetRawAxis(3), joystickCurve);
	
	//adds a little bit of tolerance of .1 for trigger
	if(trigger <= -.1)
	{
		return trigger;
	}

	else
	{
		return 0;
	}
}

float XboxController::GetTriggerAxis()
{
	float trigger = pow(xbox->GetRawAxis(3), joystickCurve);
		
		//adds a little bit of tolerance of .1 for trigger
		if( 1 >= abs(trigger) <= -.1)
		{
			return trigger;
		}

		else
		{
			return 0;
		}
}

float XboxController::GetRightX()
{
	return pow(xbox->GetRawAxis(4), joystickCurve);
}

float XboxController::GetRightY()
{
	return pow(xbox->GetRawAxis(5), joystickCurve);
}


/*----------------------------------------*/
/*
 * This Gets Raw sate of button
 * It is good for things that
 * need to have the button held
 * down to work.
 */

bool XboxController::GetA()
{
	return xbox->GetRawButton(1);
}

bool XboxController::GetB()
{
	return xbox->GetRawButton(2);
}

bool XboxController::GetX()
{
	return xbox->GetRawButton(3);
}

bool XboxController::GetY()
{
	return xbox->GetRawButton(4);
}

bool XboxController::GetLB()
{
	return xbox->GetRawButton(5);
}

bool XboxController::GetRB()
{
	return xbox->GetRawButton(6);
}

bool XboxController::GetSelect()
{
	return xbox->GetRawButton(7);
}

bool XboxController::GetStart()
{
	return xbox->GetRawButton(8);
}

bool XboxController::GetLeftStickClick()
{
	return xbox->GetRawButton(9);
}

bool XboxController::GetRightStickClick()
{
	return xbox->GetRawButton(10);
}
