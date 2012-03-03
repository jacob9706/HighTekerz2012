#ifndef __SHOOTER_H
#define __SHOOTER_H

class Victor;
class Encoder;
class Solenoid;

class Shooter
{
public:
	Shooter(Victor *topWheel,
			Victor *bottomWheel,
			Victor *rotationMotor,
			Victor *pitchMotor,
			Encoder *topWheelEncoder,
			Encoder *bottomWheelEncoder,
			Encoder *rotationEncoder,
			Encoder *pitchEncoder,
			Solenoid *kicker);
	~Shooter();
	void Periodic(bool shoot);
	void ManualRotate(float axis);
	void ManualPitch(float axis);
private:
	Victor *_topWheel;
	Victor *_bottomWheel;
	Victor *_rotationMotor;
	Victor *_pitchMotor;
	Encoder *_topWheelEncoder;
	Encoder *_bottomWheelEncoder;
	Encoder *_rotationEncoder;
	Encoder *_pitchEncoder;
	Solenoid *_kicker;
};

#endif
