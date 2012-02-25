#ifndef __ELEVATORSYSTEM_H
#define __ELEVATORSYSTEM_H

class Relay;
class DigitalInput;

class ElevatorSystem
{
public:
	ElevatorSystem(Relay* lowerElevatorMotor, 
					Relay* upperElevatorMotor, 
					DigitalInput* lowerLimitSwitch, 
					DigitalInput* upperLimitSwitch);
	~ElevatorSystem();
	void PeriodicSystem(bool startElevator);
	bool IsRunning;
private:
	bool elevatorUp;
	bool elevatorDown;
	Relay* _lowerElevatorMotor;
	Relay* _upperElevatorMotor;
	DigitalInput* _lowerLimitSwitch;
	DigitalInput* _upperLimitSwitch;
	double timeStart;
};


#endif
