#include "DeadReckoner.h"
#include "math.h"
#include "Encoder.h"

const static double ticksPerRevolution = 540.0;
const static double pi = 3.14159;
const static double wheelWidth = 31.5;
const static double wheelRadius = 3.892;

DeadReckoner::DeadReckoner(Encoder *leftEncoder2, Encoder *rightEncoder2)
{
	leftEncoder = leftEncoder2;
	rightEncoder = rightEncoder2;
	leftCount = 0;
	rightCount = 0;
	heading = 0.0;
	x = 0.0;
	y = 0.0;
}
DeadReckoner::~DeadReckoner()
{
	
}

void DeadReckoner::ResetPosition()
{
	leftEncoder->Reset();
	rightEncoder->Reset();
	leftCount = 0;
	rightCount = 0;
	x = 0.0;
	y = 0.0;
	heading = 0.0;
}
void DeadReckoner::Update()
{
	int newLeftEncoder = leftEncoder->Get();
	int newRightEncoder = rightEncoder->Get();
	
	leftCountDelta = newLeftEncoder - leftCount;
	rightCountDelta = newRightEncoder - rightCount;

	heading += (2.0*pi)*(wheelRadius/(wheelWidth))*((leftCountDelta-rightCountDelta)/ticksPerRevolution);
	x += wheelRadius*sin(heading)*(leftCountDelta+rightCountDelta)*(pi/ticksPerRevolution); //IN
	y += wheelRadius*cos(heading)*(leftCountDelta+rightCountDelta)*(pi/ticksPerRevolution); //IN

	leftCount = newLeftEncoder;
	rightCount = newRightEncoder;
}

//encoder reading : -121.85
//14'3"
//14'7"

float DeadReckoner::PositionX()
{
	return x;
}
float DeadReckoner::PositionY()
{
	return y;
}
float DeadReckoner::Heading()
{
	return heading;
}
