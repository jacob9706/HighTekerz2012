#ifndef __DEAD_RECKONER_H
#define __DEAD_RECKONER_H

class Encoder;

class DeadReckoner
{
public:
	DeadReckoner(Encoder *leftEncoder, Encoder *rightEncoder);
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
	Encoder *leftEncoder;
	Encoder *rightEncoder;
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
