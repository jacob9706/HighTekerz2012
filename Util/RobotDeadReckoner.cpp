#include <cmath>
#include "WPILib.h"


class RobotDeadReckoner
{
public:
	RobotDeadReckoner(Encoder *e1, Encoder *e2);
	float getX();
	float getY();
	float getHeading();
private:
	Encoder *encoder1;//Encoder1 (Left Transmision while looking from the back)
	Encoder *encoder2;//Encoder2 (Right Transmision while looking from the back)
	int wheelRadius;//Wheel Radius (Center Wheel)
	float axleWidthCenterToCenter;
	int encoderTicksPerRotation;
	int transmitionSprocketTeeth;
	int wheelSprocketTeeth;
	int ticksPerRotation; //ticks per rotation of wheel
	float encoderTicks1;
	float encoderTicks2;
	float pi;
};

RobotDeadReckoner::RobotDeadReckoner(Encoder *e1, Encoder *e2)
{
	encoder1 = e1;
	encoder2 = e2;
	wheelRadius = 4;//Wheel Radius (Center Wheel)
	axleWidthCenterToCenter = 30+(7/8);
	encoderTicksPerRotation = 360;
	transmitionSprocketTeeth = 12;
	wheelSprocketTeeth = 26;
	ticksPerRotation = (wheelSprocketTeeth/transmitionSprocketTeeth)*encoderTicksPerRotation; //ticks per rotation of wheel

	encoderTicks1 = encoder1->Get();
	encoderTicks2 = encoder2->Get();

	pi = 3.14159;
}

float RobotDeadReckoner::getX()
{
	float x = wheelRadius*cos(getHeading())*(encoderTicks1+encoderTicks2)*(pi/ticksPerRotation);
	return x;
}

float RobotDeadReckoner::getY()
{
	float y = wheelRadius*sin(getHeading())*(encoderTicks1+encoderTicks2)*(pi/ticksPerRotation);
	return y;
}

float RobotDeadReckoner::getHeading()
{
	float heading = (2*pi)*(wheelRadius/axleWidthCenterToCenter)*(encoderTicks1-encoderTicks2);
	return heading;
}
//(2*Pi)*(Wr/D)*(T1-T2)*(Pi/Tr);//Heading of robot in radian
//(10*cos(position[0]))*(T1+T2)*(Pi/Tr);//X position of robot in inches
//(10*sin(position[0]))*(T1+T2)*(Pi/Tr);//Y position of robot in inches
