#ifndef __XBOX_CONTROLLER_H
#define __XBOX_CONTROLLER_H

class Joystick;
class ToggleButton;

class XboxController
{
public:
	XboxController(int port);
	~XboxController();

	float GetLeftX();
	float GetLeftY();

	float GetLeftTrigger();
	float GetRightTrigger();
	float GetTriggerAxis();

	float GetRightX();
	float GetRightY();

	bool GetA();
	bool GetB();
	bool GetX();
	bool GetY();
	bool GetLB();
	bool GetRB();
	bool GetSelect();
	bool GetStart();
	bool GetLeftStickClick();
	bool GetRightStickClick();

	float joystickCurve;
private:
	Joystick *xbox;
};


#endif
