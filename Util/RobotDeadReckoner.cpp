#include <cmath>
#include "WPILib.h"
#include "RobotDeadReckoner.h"

RobotDeadReckoner::RobotDeadReckoner(Encoder* Left, Encoder* Right)
{
	encoderRight = Right;
	encoderLeft = Left;
	wheelRadius = 4;//Wheel Radius (Center Wheel)
	axleWidthCenterToCenter = 30+(7/8);
	encoderTicksPerRotation = 360;
	transmitionSprocketTeeth = 12;
	wheelSprocketTeeth = 26;
	ticksPerRotation = (wheelSprocketTeeth/transmitionSprocketTeeth)*encoderTicksPerRotation; //ticks per rotation of wheel

	encoderTicksRight = encoderRight->Get();
	encoderTicksLeft = encoderLeft->Get();

	pi = 3.14159;
}

float RobotDeadReckoner::getX()
{
	float x = wheelRadius*cos(getHeading())*(encoderTicksRight+encoderTicksLeft)*(pi/ticksPerRotation);
	return x;
}

float RobotDeadReckoner::getY()
{
	float y = wheelRadius*sin(getHeading())*(encoderTicksRight+encoderTicksLeft)*(pi/ticksPerRotation);
	return y;
}

float RobotDeadReckoner::getHeading()
{
	float heading = (2*pi)*(wheelRadius/axleWidthCenterToCenter)*(encoderTicksRight-encoderTicksLeft);
	return heading;
}
//(2*Pi)*(Wr/D)*(T1-T2)*(Pi/Tr);//Heading of robot in radian
//(10*cos(position[0]))*(T1+T2)*(Pi/Tr);//X position of robot in inches
//(10*sin(position[0]))*(T1+T2)*(Pi/Tr);//Y position of robot in inches
