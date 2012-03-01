#include "DeadReckoner.h"
#include "math.h"
#include "Encoder.h"

DeadReckoner::DeadReckoner(Encoder *leftEncoder2, Encoder *rightEncoder2)
{
	leftEncoder = leftEncoder2;
	rightEncoder = rightEncoder2;
	leftCount = 0;
	rightCount = 0;
	heading = 0.0;
	x = 0.0;
	y = 0.0;
	pi = 3.14159;
}
DeadReckoner::~DeadReckoner()
{
	
}
void DeadReckoner::Update()
{
	leftCount = leftEncoder->Get();
	rightCount = rightEncoder->Get();
	heading = (2.0*pi)*(4.0/(30.0 +(7.0/8.0)))*(leftCount-rightCount);
	x = 4.0*sin(heading)*(leftCount+rightCount)*(pi/780.0);
	y = 4.0*cos(heading)*(leftCount+rightCount)*(pi/780.0);
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
