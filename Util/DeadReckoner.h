#ifndef __DEAD_RECKONER_H
#define __DEAD_RECKONER_H

class Encoder3574;

class DeadReckoner
{
public:
	DeadReckoner(Encoder3574 *leftEncoder, Encoder3574 *rightEncoder);
	~DeadReckoner();
	void ResetPosition();
	void Update();
	float PositionX();
	float PositionY();
	float Heading();
	float AngularVelocity();
	float Velocity();
	int leftCount;
	int rightCount;
	Encoder3574 *leftEncoder;
	Encoder3574 *rightEncoder;
private:
	float heading;
	float x;
	float y;
	double lastTime;
	int leftCountDelta;
	int rightCountDelta;
	int lastLeftCount;
	int lastRightCount;
	double lastHeading;
};

#endif 
