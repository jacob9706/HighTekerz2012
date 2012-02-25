#ifndef __ROBOT_DEAD_RECKONER_H
#define __ROBOT_DEAD_RECKONER_H

class Encoder;

class RobotDeadReckoner
{
public:
	RobotDeadReckoner(Encoder* Left, Encoder* Right);
	float getX();
	float getY();
	float getHeading();
private:
	Encoder* encoderRight;//Encoder1 (Left Transmision while looking from the back)
	Encoder* encoderLeft;//Encoder2 (Right Transmision while looking from the back)
	int wheelRadius;//Wheel Radius (Center Wheel)
	float axleWidthCenterToCenter;
	int encoderTicksPerRotation;
	int transmitionSprocketTeeth;
	int wheelSprocketTeeth;
	int ticksPerRotation; //ticks per rotation of wheel
	float encoderTicksRight;
	float encoderTicksLeft;
	float pi;
};


#endif
