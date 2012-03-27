#ifndef __ELEVATORSYSTEM_H
#define __ELEVATORSYSTEM_H

class Relay;
class DigitalInput;

enum MotorState 
{
	UP,
	STOP,
	DOWN
};


class ElevatorSystem
{
public:
	ElevatorSystem(Relay* lowerElevatorMotor, 
					Relay* upperElevatorMotor, 
					DigitalInput* lowerLimitSwitch, 
					DigitalInput* upperLimitSwitch);
	~ElevatorSystem();
	void ManualLower(MotorState direction);
	void ManualUpper(MotorState direction);
	void ManualFreezeAll();
	void PeriodicSystem(bool startElevator);
	bool IsRunning;

private:
	bool elevatorUp;
	bool elevatorDown;
	bool limitReachedLowerTop;
	bool limitReachedLowerBottom;
	MotorState lowerEv;
	MotorState upperEv;
	Relay* _lowerElevatorMotor;
	Relay* _upperElevatorMotor;
	DigitalInput* _lowerLimitSwitchOpen;
	DigitalInput* _upperLimitSwitchOpen;
	double timeStart;
};



#endif
