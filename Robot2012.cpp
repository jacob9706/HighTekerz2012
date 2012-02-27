#include "WPILib.h"
#include "math.h"
#include "SubSystems/ElevatorSystem.h"
#include "Util/Switch.h"
#include "Util/XboxController.h"

//Robot Server///////////
STATUS tcpServer (void) ;
MSG_Q_ID robotQueue;
MSG_Q_ID getRobotMsgQueue()
{
	return robotQueue;
}

//Robot Class///////////
class Robot2012 : public SimpleRobot
{	
	// Drive System //////////////////////
	RobotDrive *myRobot;

	// Outputs ///////////////////////////
	
	// Motors
	PWM* greenLightControl;
	Victor* bBallRotator;
	Victor* bBallPitchMotor;
	Victor* bBallShooterTop;
	Victor* bBallShooterBottom;
	Relay* bBallElevatorBottom;
	Relay* bBallElevatorTop;
	Relay* bBallCollector;

	// Air
	Compressor* airCompressor;
	
	Solenoid* shooterArm;
	Solenoid* rampArm;

	// Inputs //////////////////////////
	// On robot //
	// Encoders

	Encoder* encoderWheelsLeft;
	Encoder* encoderWheelsRight;
	Encoder* encoderTurretRotation;
	Encoder* bBallShooterTopEncoder;
	Encoder* bBallShooterBottomEncoder;
	Encoder* bBallAngle;
	
	// Switches
	
//	DigitalInput *bBallCollectorSensor;
//	DigitalInput *bBallElevatorSensor;
	DigitalInput *bBallElevatorTopLimit;
	DigitalInput *bBallElevatorBottomLimit;
	
	// Other
	ADXL345_I2C *accel;
	Gyro *gyro;
	
	// Driver Station //

	DriverStation *driverStationControl;				// driver station object
	DriverStationLCD* dsLCD;

	XboxController *xboxDrive;
	XboxController *xboxShoot;
	
	// Systems and Support ///////////////////

	ElevatorSystem* robotElevator;
	
	Switch shooterSwitch;
	Switch switchAimTrigger;
	
	bool isShootStick;
	bool shooterState;
	
	double speed1, speed2;
	UINT8 lightValue;
	
public:
	/**
	 * Create an instance of a RobotDrive with left and right motors plugged into PWM
	 */

	Robot2012(void)
	{
		// Drive System /////////////////////
		myRobot = new RobotDrive(3, 4, 1, 2);	// create robot drive base
		myRobot->SetExpiration(0.1);

		speed1 = 0;
		speed2 = 0;
		lightValue = 0;
		
		// Outputs //////////////////////////		
		// Motors
		greenLightControl = new  PWM(1,6);
		greenLightControl->SetPeriodMultiplier(PWM::kPeriodMultiplier_1X);
		greenLightControl->EnableDeadbandElimination(false);
		
		bBallPitchMotor = new Victor(2,3);
		bBallRotator = new Victor(2,4);
		bBallShooterTop = new Victor(2,1);
		bBallShooterBottom = new Victor(2,2);
		
		bBallElevatorBottom = new Relay(2,1,Relay::kBothDirections);
		bBallElevatorTop = new Relay(2,2,Relay::kBothDirections);
		bBallCollector = new Relay(2,4,Relay::kBothDirections);

		// Air
		airCompressor = new Compressor(1,10,2,3);
		airCompressor->Start();
		
		shooterArm = new Solenoid(1);
		
		// Inputs //////////////////////////
		// On robot //
		// Encoders

		encoderWheelsLeft = new Encoder(1,1,1,2,false);
		encoderWheelsRight = new Encoder(1,3,1,4,true);
		encoderTurretRotation = new Encoder(2,3,2,4);
		bBallAngle = new Encoder(2,11,2,12);
		bBallShooterTopEncoder = new Encoder(2,5,2,6);
		bBallShooterBottomEncoder = new Encoder(2,7,2,8);
		
		encoderWheelsLeft->Start();
		encoderWheelsRight->Start();
		encoderTurretRotation->Start();
		bBallAngle->Start();
		bBallShooterTopEncoder->Start();
		bBallShooterBottomEncoder->Start();
		
		// Switches
//		bBallCollectorSensor = new DigitalInput(2,3);
//		bBallElevatorSensor = new DigitalInput(2,4);
//		bBallBrushSensor = new DigitalInput(2,5);
		bBallElevatorTopLimit = new DigitalInput(2,2);
		bBallElevatorBottomLimit = new DigitalInput(2,1);
		
		// Other
//		accel = new ADXL345_I2C(1);
//		gyro = new Gyro(1);
		
		// Driver Station //

		driverStationControl = DriverStation::GetInstance();
		dsLCD = DriverStationLCD::GetInstance();

		xboxDrive = new XboxController(1);
		xboxShoot = new XboxController(2);
		
		// Systems and Support ///////////////////
		robotElevator = new ElevatorSystem(bBallElevatorBottom, bBallElevatorTop, bBallElevatorBottomLimit, bBallElevatorTopLimit);

		shooterSwitch = Switch();
		switchAimTrigger = Switch();
		
		shooterState = false;
		isShootStick = driverStationControl->GetDigitalIn(1);
		
		//Robot Server///////////
			robotQueue = msgQCreate(100, 1024, MSG_Q_PRIORITY);		
			if (taskSpawn("tcpServer", 100, 0, 10000, 
					(FUNCPTR) tcpServer, 0,0,0,0,0,0,0,0,0,0) == ERROR) 
			{ 
				/* if taskSpawn fails, close fd and return to top of loop */ 

				perror ("taskSpawn"); 

			}
		
		// init
		
		//random

		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Hello World");
		dsLCD->UpdateLCD();

	}

	void processVisionBridge(char * msg) {
		char * workingMessage = strtok(msg,",");
		
		do {
			double value = atof(&workingMessage[1]);
			switch(workingMessage[0]) {
			case 'a': speed1 = value; break;
			case 'b': speed2 = value; break;
			case 'c': lightValue = atoi(&workingMessage[1]); break;
			}
		} while (workingMessage = strtok(0, ","));
	}
	
	/**
	 * Drive left & right motors for 2 seconds, enabled by a jumper (jumper
	 * must be in for autonomous to operate).
	 */
	void Autonomous(void)
	{
	}

	
	
	/**
	 * Runs the motors under driver control with either tank or arcade steering selected
	 * by a jumper in DS Digin 0. Also an arm will operate based on a joystick Y-axis. 
	 */
	
	void OperatorControl(void)
	{
		printf("hi");
		while (IsOperatorControl())
		{
			//greenLightControl->SetRaw(10);
			if (isShootStick != driverStationControl->GetDigitalIn(1))
			{
			}
			
			// Drive //////////////////////////////
			myRobot->TankDrive(-xboxDrive->GetLeftY(), -xboxDrive->GetRightY());	 // drive with tank style
			
			// Aim ////////////////////////////////
			//bBallRotator->Set(-xboxShoot->GetRightX()/2);
			bBallPitchMotor->Set((xboxShoot->GetLeftY()/2.1));
//			bBallShooterTop->Set(driverStationControl->GetAnalogIn(1)*-1);
//			bBallShooterBottom->Set(driverStationControl->GetAnalogIn(1)*-1);
			
			//  initial plan ///////////////////////////////////////////////
//			Driver
//			-rb & rt load balls
//			-lb & lt unload balls
//
//			Shooter
//			-lt rt rotation
//			-y activate elevator
//			-a shoot
			

			// Collect and Shoot bBalls///////////
/*
			if(xboxShoot->GetRB() || xboxShoot->GetRightTrigger() < -.1)
			{
				shooterState = true;
			}
			
			if(xboxShoot->GetLB() || xboxShoot->GetLeftTrigger() > .1)
			{
				shooterState = false;
			}
			
			if (shooterState)
			{
				bBallShooterTop->Set(driverStationControl->GetAnalogIn(1)*-1);
				bBallShooterBottom->Set(driverStationControl->GetAnalogIn(2)*-1);
			}
			else
			{
				bBallShooterTop->Set(0.0);
				bBallShooterBottom->Set(0.0);
			}
*/			
			shooterArm->Set(xboxShoot->GetA());
			
			

			// collector ///////////////////////////////
			if (xboxDrive->GetLB() || xboxDrive->GetLeftTrigger() > .1)
			{
				bBallCollector->Set(Relay::kOn);
				bBallCollector->Set(Relay::kReverse);						
			}
			else if (xboxDrive->GetRB() || xboxDrive->GetRightTrigger() < -.1)
			{
				bBallCollector->Set(Relay::kOn);
				bBallCollector->Set(Relay::kForward);						
			}
			else
			{
				bBallCollector->Set(Relay::kOff);
			}

			
			// ELEVATORS //////////////////////////////
			// REVERSE IS UP ///////////
			// bottom down
			if (xboxShoot->GetSelect())
			{
				bBallElevatorBottom->Set(Relay::kOn);
				bBallElevatorBottom->Set(Relay::kForward);						
			}
			// bottom up
			else if (xboxShoot->GetStart())
			{
				bBallElevatorBottom->Set(Relay::kOn);
				bBallElevatorBottom->Set(Relay::kReverse);						
			}
			else
			{
				bBallElevatorBottom->Set(Relay::kOff);
			}
			
			// top down			
			if (xboxShoot->GetX())
			{
				bBallElevatorTop->Set(Relay::kOn);
				bBallElevatorTop->Set(Relay::kForward);
			}
			// top up
			else if (xboxShoot->GetB())
			{
				bBallElevatorTop->Set(Relay::kOn);
				bBallElevatorTop->Set(Relay::kReverse);						
			}
			else
			{
				bBallElevatorTop->Set(Relay::kOff);
			}
			
			// Air compressor
			
//			robotElevator->PeriodicSystem((switchAimTrigger.State(aimStick->GetTrigger())));
			robotElevator->PeriodicSystem(xboxShoot->GetY());
			
			//Robot Server///////////
			char msgBuf[1024];


			//printf("%d\n", val);
			memset(msgBuf, 0, sizeof(char) * 1024);
			if (msgQReceive(robotQueue, msgBuf, 1024, NO_WAIT) != ERROR) {
				printf("Got a message: %s\n", msgBuf);
				
				processVisionBridge(msgBuf);
				//speed = atoi(msgBuf);
				//speed /= 1000;
				//printf("speed: %f", speed);
			}
			
			bBallRotator->Set(speed1);
			bBallShooterBottom->Set(speed2);
			greenLightControl->SetRaw(lightValue);
			
			// random output stuff!! ////////////
/*			printf("lft: %d  ", encoderWheelsLeft->Get());
			printf("rt: %d  ", encoderWheelsRight->Get());			
			printf("turret: %d  ", encoderTurretRotation->Get());
			printf("top enc: %d  ", bBallShooterTopEncoder->Get());			
*/
			printf("Y position: %f  ", GetFieldy(encoderWheelsLeft->Get(), encoderWheelsRight->Get(), GetFieldHeading(encoderWheelsLeft->Get(), encoderWheelsRight->Get())));
			printf("X position: %f  ", GetFieldX(encoderWheelsLeft->Get(), encoderWheelsRight->Get(), GetFieldHeading(encoderWheelsLeft->Get(), encoderWheelsRight->Get())));
			printf("Heading: %f \r", GetFieldHeading(encoderWheelsLeft->Get(), encoderWheelsRight->Get()));

			Wait(0.01);
		}
	}

	float degrees(float rads)
	{
		//180/pi = 57.296
		return rads*57.296;
	}
	
	void CalculateIdealAngleAndSpeed(double distance, double height, double* computedAngle, double* computedSpeed)
	{
		*computedAngle = 0.0;
		*computedSpeed = 0.0;
		
		// =45+((DEGREES(ATAN(height/distance)))/2)
		*computedAngle = 45.0+((degrees(atan(height/distance)))/2.0);

		printf("a: %f/n",*computedAngle);
		//32.2 ft/s/s
		// =SQRT(32.2*(height+(SQRT(height^2+distance^2))))
		*computedSpeed = sqrt(32.2*(height+(sqrt(pow(height,2.0)+pow(distance,2.0)))));
		printf("S: %f/n",*computedSpeed);
	}
	
	float GetFieldHeading(int encoderTicks1, int encoderTicks2)
	{
		int wheelRadius = 4;//Wheel Radius (Center Wheel)
		float axleWidthCenterToCenter = 30+(7/8);		

		double pi = 3.14159;
		float heading = (2*pi)*(wheelRadius/axleWidthCenterToCenter)*(encoderTicks1-encoderTicks2);
		return heading;
	}
	
	float GetFieldy(int encoderTicks1, int encoderTicks2, float heading)
	{
		int wheelRadius = 4;//Wheel Radius (Center Wheel)
		int encoderTicksPerRotation = 360;
		int transmitionSprocketTeeth = 12;
		int wheelSprocketTeeth = 26;
		int ticksPerRotation = (wheelSprocketTeeth/transmitionSprocketTeeth)*encoderTicksPerRotation; //ticks per rotation of wheel

		double pi = 3.14159;

		float y = wheelRadius*sin(heading)*(encoderTicks1+encoderTicks2)*(pi/ticksPerRotation);
		return y;
	}
	
	float GetFieldX(int encoderTicks1, int encoderTicks2, float heading)
	{
		int wheelRadius = 4;//Wheel Radius (Center Wheel)
		int encoderTicksPerRotation = 360;
		int transmitionSprocketTeeth = 12;
		int wheelSprocketTeeth = 26;
		int ticksPerRotation = (wheelSprocketTeeth/transmitionSprocketTeeth)*encoderTicksPerRotation; //ticks per rotation of wheel

		double pi = 3.14159;

		float x = wheelRadius*cos(heading)*(encoderTicks1+encoderTicks2)*(pi/ticksPerRotation);
		return x;
	}
	
};


START_ROBOT_CLASS(Robot2012);
