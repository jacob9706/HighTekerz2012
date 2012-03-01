#ifndef __DEAD_RECKONER_H
#define __DEAD_RECKONER_H

class Encoder;

class DeadReckoner
{
public:
	DeadReckoner(Encoder *leftEncoder, Encoder *rightEncoder);
	~DeadReckoner();
	void Update();
	float PositionX();
	float PositionY();
	float Heading();
private:
	Encoder *leftEncoder;
	Encoder *rightEncoder;
	int leftCount;
	int rightCount;
	float heading;
	float x;
	float y;
	float pi;
};

#endif 
