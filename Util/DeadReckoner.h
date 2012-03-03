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
	int leftCount;
	int rightCount;
private:
	Encoder *leftEncoder;
	Encoder *rightEncoder;
	float heading;
	float x;
	float y;
	float pi;
};

#endif 
