#include "ElevatorSystem.h"
#include "DigitalInput.h"
#include "Relay.h"

ElevatorSystem::ElevatorSystem(Relay* lowerElevatorMotor, 
		Relay* upperElevatorMotor, 
		DigitalInput* lowerLimitSwitch, 
		DigitalInput* upperLimitSwitch)
{
	_lowerElevatorMotor = lowerElevatorMotor;
	_lowerLimitSwitch = lowerLimitSwitch;
	_upperElevatorMotor = upperElevatorMotor;
	_upperLimitSwitch = upperLimitSwitch;
	
	IsRunning = false;	
	elevatorUp = false;
	elevatorDown = false;
	timeStart = 601.0;
}

ElevatorSystem::~ElevatorSystem()
{
	
}

void ElevatorSystem::PeriodicSystem(bool startElevator)
{
	// test to run if not running
	if (!IsRunning && startElevator)
	{
		IsRunning = true;
		elevatorUp = true;
		timeStart = 0.0;
	}

	// timer for the upper motor (terrible dependancy on loop length of teleop
	if (timeStart < 500)
	{
		// start top el
		_upperElevatorMotor->Set(Relay::kOn);
		_upperElevatorMotor->Set(Relay::kReverse);
		timeStart++;
	}
//	else
//	{
//		_upperElevatorMotor->Set(Relay::kOff);
//	}

	// back and forth elevator
	if (elevatorDown)
	{
		// run motor down
		_lowerElevatorMotor->Set(Relay::kOn);
		_lowerElevatorMotor->Set(Relay::kForward);

		if (!_lowerLimitSwitch->Get())  // bottom switch clicked
		{
			elevatorDown = false;

			// stop the elevators
			_lowerElevatorMotor->Set(Relay::kOff);
			IsRunning = false;
		}
	}
	if (elevatorUp)
	{
		// start bottom el
		_lowerElevatorMotor->Set(Relay::kOn);
		_lowerElevatorMotor->Set(Relay::kReverse);

		// if top limit clicked
		if (!_upperLimitSwitch->Get())
		{
			//stop bottom el
			
			_lowerElevatorMotor->Set(Relay::kOff);

			elevatorUp = false;
			elevatorDown = true;
		}
	}
}
