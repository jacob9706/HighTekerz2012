#ifndef __SHOOTER_H
#define __SHOOTER_H

class Victor;
class Encoder3574;
class Solenoid;

class Shooter
{
public:
	Shooter(Victor *topWheel,
			Victor *bottomWheel,
			Victor *rotationMotor,
			Victor *pitchMotor,
			Encoder3574 *topWheelEncoder,
			Encoder3574 *bottomWheelEncoder,
			Encoder3574 *rotationEncoder,
			Encoder3574 *pitchEncoder,
			Solenoid *kicker);
	~Shooter();
	void Periodic(bool shoot);
	void ManualRotate(float axis);
	void ManualPitch(float axis);
	bool IsShooting;
	int leftCount;
	int rightCount;
private:
	Victor *_topWheel;
	Victor *_bottomWheel;
	Victor *_rotationMotor;
	Victor *_pitchMotor;
	Encoder3574 *_topWheelEncoder;
	Encoder3574 *_bottomWheelEncoder;
	Encoder3574 *_rotationEncoder;
	Encoder3574 *_pitchEncoder;
	Solenoid *_kicker;
};

#endif
