<<<<<<< HEAD
#include "ElevatorSystem.h"
#include "DigitalInput.h"
#include "Relay.h"

ElevatorSystem::ElevatorSystem(Relay* lowerElevatorMotor, 
		Relay* upperElevatorMotor, 
		DigitalInput* lowerLimitSwitch, 
		DigitalInput* upperLimitSwitch)
{
	_lowerElevatorMotor = lowerElevatorMotor;
	_lowerLimitSwitchOpen = lowerLimitSwitch;
	_upperElevatorMotor = upperElevatorMotor;
	_upperLimitSwitchOpen = upperLimitSwitch;
	
	IsRunning = false;	
	elevatorUp = false;
	elevatorDown = false;
	timeStart = 1001.0;
	
	limitReachedLowerBottom = !_lowerLimitSwitchOpen->Get();
	limitReachedLowerTop = !_upperLimitSwitchOpen->Get();
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
	if (timeStart < 1000)
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

		if (!_lowerLimitSwitchOpen->Get())  // bottom switch clicked
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
		if (!_upperLimitSwitchOpen->Get())
		{
			//stop bottom el

			_lowerElevatorMotor->Set(Relay::kOff);

			elevatorUp = false;
			elevatorDown = true;
		}
	}
	
	if (limitReachedLowerBottom)
	{
		
	}
	
	
}

void ElevatorSystem::ManualFreezeAll()
{
	_lowerElevatorMotor->Set(Relay::kOff);
	_upperElevatorMotor->Set(Relay::kOff);

	IsRunning = false;	
	elevatorUp = false;
	elevatorDown = false;
	timeStart = 1001.0;
}
=======
#include "ElevatorSystem.h"
#include "DigitalInput.h"
#include "Relay.h"

ElevatorSystem::ElevatorSystem(Relay* lowerElevatorMotor, 
		Relay* upperElevatorMotor, 
		DigitalInput* lowerLimitSwitch, 
		DigitalInput* upperLimitSwitch)
{
	_lowerElevatorMotor = lowerElevatorMotor;
	_lowerLimitSwitchOpen = lowerLimitSwitch;
	_upperElevatorMotor = upperElevatorMotor;
	_upperLimitSwitchOpen = upperLimitSwitch;
	
	IsRunning = false;	
	elevatorUp = false;
	elevatorDown = false;
	timeStart = 1001.0;
	
	limitReachedLowerBottom = !_lowerLimitSwitchOpen->Get();
	limitReachedLowerTop = !_upperLimitSwitchOpen->Get();
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
	if (timeStart < 1000)
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

		if (!_lowerLimitSwitchOpen->Get())  // bottom switch clicked
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
		if (!_upperLimitSwitchOpen->Get())
		{
			//stop bottom el

			_lowerElevatorMotor->Set(Relay::kOff);

			elevatorUp = false;
			elevatorDown = true;
		}
	}
	
	if (limitReachedLowerBottom)
	{
		
	}
	
	
}

void ElevatorSystem::ManualFreezeAll()
{
	_lowerElevatorMotor->Set(Relay::kOff);
	_upperElevatorMotor->Set(Relay::kOff);

	IsRunning = false;	
	elevatorUp = false;
	elevatorDown = false;
	timeStart = 1001.0;
}
>>>>>>> 5cf0ff622f058e49bd836e49cbd450d29583cd9d
