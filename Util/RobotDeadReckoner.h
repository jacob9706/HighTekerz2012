#ifndef __ROBOT_DEAD_RECKONER_H
#define __ROBOT_DEAD_RECKONER_H

class Encoder;

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


#endif
