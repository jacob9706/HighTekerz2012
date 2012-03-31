#ifndef __DRIVETRAIN_H
#define __DRIVETRAIN_H

class RobotDrive;
#include "../Util/DeadReckoner.h"

class Drivetrain : public RobotDrive, public DeadReckoner
{
public:
	Drivetrain(UINT32 left1, UINT32 left2, UINT32 right1, UINT32 right2, 
			Encoder* leftEncoder, 
			Encoder* rightEncoder
			);
	~Drivetrain();
	void Periodic(float moveLeft, float moveRight, bool enableMatchEncoders = false, bool STOP = false);
	float AngularVelocity();
	float Velocity();
	float rightMotorSetting;
	float leftMotorSetting;
	float scaledRight;
	float scaledLeft;
	bool speedMatch;
	int speedMatchLeftCounterStart;
	int speedMatchRightCounterStart;
private:

};

#endif

#ifndef __DRIVETRAIN_H
#define __DRIVETRAIN_H

class RobotDrive;
#include "../Util/DeadReckoner.h"

class Drivetrain : public RobotDrive, public DeadReckoner
{
public:
	Drivetrain(UINT32 left1, UINT32 left2, UINT32 right1, UINT32 right2, 
			Encoder* leftEncoder, 
			Encoder* rightEncoder
			);
	~Drivetrain();
	void Periodic(float moveLeft, float moveRight, bool enableMatchEncoders = false, bool STOP = false);
	float AngularVelocity();
	float Velocity();
	float rightMotorSetting;
	float leftMotorSetting;
	float scaledRight;
	float scaledLeft;
	bool speedMatch;
	int speedMatchLeftCounterStart;
	int speedMatchRightCounterStart;
private:

};

#endif
