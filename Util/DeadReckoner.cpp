#include "DeadReckoner.h"
#include "math.h"
#include "Encoder.h"

const static double ticksPerRevolution = 780.0;
const static double pi = 3.14159;
const static double wheelWidth = 30.875;
const static double wheelRadius = 4.0;

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
	leftCountDelta = leftEncoder->Get() - leftCount;
	rightCountDelta = rightEncoder->Get() - rightCount;

	leftCount = leftEncoder->Get();
	rightCount = rightEncoder->Get();
		
	heading = (2.0*pi)*(wheelRadius/(wheelWidth))*((leftCount-rightCount)/ticksPerRevolution);
	x += wheelRadius*sin(heading)*(leftCountDelta+rightCountDelta)*(pi/ticksPerRevolution);
	y += wheelRadius*cos(heading)*(leftCountDelta+rightCountDelta)*(pi/ticksPerRevolution);
}

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
