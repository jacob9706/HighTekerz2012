#include "WPILib.h"
#include "math.h"
#include "SubSystems/ElevatorSystem.h"
#include "SubSystems/Drivetrain.h"
#include "SubSystems/RampArm.h"
#include "Util/Switch.h"
#include "Util/XboxController.h"
#include "Util/DeadReckoner.h"
#include "Settings.h"

const static float TURRET_ROTATION_TICKS = 720;

//#include "SubSystems/Shooter.h"

//Robot Server///////////
STATUS tcpServer (void) ;
MSG_Q_ID robotQueue;
MSG_Q_ID getRobotMsgQueue()
{
	return robotQueue;
}

class SamplePIDOutput : public PIDOutput 
{
public:
	SamplePIDOutput(Victor *motor) 
	{
		_motor = motor;
	}

	void PIDWrite(float output) 
	{
		_motor->Set(output);
	}
private:
	Victor* _motor;
};

class SamplePIDSource : public PIDSource 
{
public:
	SamplePIDSource(Encoder* enc) 
	{
		//"K" is a tuning constant, which you use to adjust the "strength" of the filter. K must be in the range zero to +1. When K=0, there is no filtering. When K=1, the filtering is so "strong" that the filtered value never changes.
		TUNING_CONSTANT = .07;
		SCALE = 100000;

		_enc = enc;
		encoderCurrent = 0;
		encoderLast = 0;
		timeLast = GetFPGATime();
		timeCurrent = 0;
		rate = 0;
		newRate = 0;
	}

	double PIDGet()
	{
		return SCALE * rate;
	}
	
	void Update()
	{
		encoderCurrent = _enc->Get();
		timeCurrent = GetFPGATime();
		newRate = (encoderCurrent-encoderLast) / (timeCurrent - timeLast);

		//new_filtered_value = K*previous_filtered_value + (1-K)* new_sample
		rate = (TUNING_CONSTANT * rate) + ((1-TUNING_CONSTANT) * newRate);

		encoderLast = encoderCurrent;
		timeLast = timeCurrent;
	}

	double TUNING_CONSTANT;	
	double SCALE;
private:
	Encoder* _enc;

	int encoderCurrent ;
	int encoderLast;
	UINT timeLast;
	UINT timeCurrent;
	double rate;
	double newRate;
};


//Robot Class///////////
class Robot2012 : public SimpleRobot
{	
	// Drive System //////////////////////
	Drivetrain *myRobot;

	// Outputs ///////////////////////////

	// Vision
	PWM* greenLightControl;	

	// Motors
	Victor* bBallRotator;
	Victor* bBallPitchMotor;
	Victor* bBallShooterTop;
	Victor* bBallShooterBottom;
	Victor* bBallCollector;
	Relay* bBallElevatorBottom;
	Relay* bBallElevatorTop;
	Servo* rampServo;

//	Relay* mockRelay;
	
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
	Encoder* encoderShooterTop;
	Encoder* encoderShooterBottom;
	DigitalInput* bBallAngleSensor;
	
	PIDController *testPID;
	
	// Switches
	
	DigitalInput *bBallElevatorTopLimit;
	DigitalInput *bBallElevatorBottomLimit;
	Switch* shooterSwitch;
	Switch* switchAimTrigger;

	// Other
	ADXL345_I2C *accel;
	Gyro *gyro;

	// Driver Station //

	DriverStation *driverStationControl;
	DriverStationLCD* dsLCD;

	XboxController *xboxDrive;
	XboxController *xboxShoot;
	
	// Systems and Support ///////////////////

	ElevatorSystem* robotElevator;
	RampArm *robotRampArm;

	//Shooter *robotShooter;

	bool isShootStick;
	bool shooterState;
	
	double speed1, speed2;
	UINT8 lightValue;
	
	int bBallTopWheelSpeed,
		bBallBottomWheelSpeed,
		bBallAngle;
	
public:
	/**
	 * Create an instance of a RobotDrive with left and right motors plugged into PWM
	 */

	Robot2012(void)
	{
		bBallTopWheelSpeed = 0;
		bBallBottomWheelSpeed = 0;
		bBallAngle = 0;
		speed1 = 0;
		speed2 = 0;
		lightValue = 0;

		AxisCamera::GetInstance();
		// Outputs //////////////////////////
		// Vision
		greenLightControl = new  PWM(1,6);
		greenLightControl->SetPeriodMultiplier(PWM::kPeriodMultiplier_1X);
		greenLightControl->EnableDeadbandElimination(false);

		// Motors
		bBallPitchMotor = new Victor(2,3);
		bBallRotator = new Victor(2,4);
		bBallShooterTop = new Victor(2,1);
		bBallShooterBottom = new Victor(2,2);
		
		bBallElevatorBottom = new Relay(2,1,Relay::kBothDirections);
		bBallElevatorTop = new Relay(2,2,Relay::kBothDirections);
		bBallCollector = new Victor(2,5);
		rampServo = new Servo(2,6);

//		mockRelay = new Relay(2,5,Relay::kBothDirections);

		// Air
		airCompressor = new Compressor(1,10,2,3);
		airCompressor->Start();
		
		shooterArm = new Solenoid(1);
		rampArm = new Solenoid(2);
		
		// Inputs //////////////////////////
		// On robot //
		// Encoders

		encoderWheelsLeft = new Encoder(1,1,1,2,false,Encoder::k1X);
		encoderWheelsRight = new Encoder(1,3,1,4,true,Encoder::k1X);
		encoderTurretRotation = new Encoder(2,3,2,4);
		bBallAngleSensor = new DigitalInput(2,11);
		encoderShooterTop = new Encoder(2,5,2,6,true,Encoder::k1X);
		encoderShooterBottom = new Encoder(2,7,2,8,true,Encoder::k1X);
		
		testPID = new PIDController(0.1, 0.1, 0.1, encoderShooterTop, bBallShooterTop);
		encoderShooterTop->SetDistancePerPulse(100);
		encoderShooterBottom->SetDistancePerPulse(1000);
		encoderShooterTop->SetPIDSourceParameter(Encoder::kRate);
		encoderShooterBottom->SetPIDSourceParameter(Encoder::kRate);
		
		
		encoderWheelsLeft->Start();
		encoderWheelsRight->Start();
		encoderTurretRotation->Start();
		encoderShooterTop->Start();
		encoderShooterBottom->Start();
		
		testPID->Enable();
		
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
		robotElevator = new ElevatorSystem(bBallElevatorBottom, 
				bBallElevatorTop, 
				bBallElevatorBottomLimit, 
				bBallElevatorTopLimit);

//		robotElevator = new ElevatorSystem(bBallElevatorTop, 
//				mockRelay, 
//				bBallElevatorBottomLimit, 
//				bBallElevatorTopLimit);

		robotRampArm = new RampArm(rampServo,rampArm);

//		robotShooter = new Shooter(bBallShooterTop,
//									bBallShooterBottom,
//									bBallRotator,
//									bBallPitchMotor,
//									bBallShooterTopEncoder,
//									bBallShooterBottomEncoder,
//									encoderTurretRotation,
//									bBallAngle,
//									shooterArm);

//		shooterSwitch = Switch();
//		switchAimTrigger = Switch();

		shooterState = false;
		isShootStick = driverStationControl->GetDigitalIn(1);

		// Drive System /////////////////////
		myRobot = new Drivetrain(3, 4, 1, 2, encoderWheelsLeft, encoderWheelsRight);	// create robot drive base

		//Robot Server///////////
		robotQueue = msgQCreate(100, 1024, MSG_Q_PRIORITY);		
		if (taskSpawn("tcpServer", 100, 0, 10000, 
				(FUNCPTR) tcpServer, 0,0,0,0,0,0,0,0,0,0) == ERROR) 
		{
			/* if taskSpawn fails, close fd and return to top of loop */ 

			perror ("taskSpawn"); 
		}
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

	void OperatorControl(void)
	{
		// any setup?
		robotElevator->ManualFreezeAll();
		
		char msgBuf[1024];
		int encoderTopLast = 0;
		int encoderBottomLast = 0;
		int encoderTopCurrent = 0;
		int encoderBottomCurrent = 0;
		UINT encoderTimeLast = GetFPGATime();
		UINT encoderTimeCurrent = 0;
		int loopCount = 0;
		int startTimeAngle = 0;
		
		Switch rampArmSwitch;
		Switch rampServoSwitch;

		rampArm->Set(false);
		
		
		PIDController turnController( 0.1, // P
				0.00, // I
				0.5, // D
				encoderShooterTop, // source
				bBallShooterTop, // output
				0.005); // period
		turnController.SetInputRange(-360.0, 360.0);
		turnController.SetOutputRange(-0.6, 0.6);
		turnController.SetTolerance(1.0 / 90.0 * 100);
		turnController.Disable();


		while (IsOperatorControl())
		{
			// get sensor feedback /////////////////////

			//set tilt
			if (bBallAngleSensor->Get() == 1)
			{
				// these for loops are to prevent a while lock.  if it loops 5000 times it ejects
				for (int i = 0; i < 500; i++)
				{
					if (bBallAngleSensor->Get() == 0)
					{
						break;
					}
				}
								startTimeAngle = GetFPGATime();
				for (int i = 0; i < 500; i++)
				{
					if (bBallAngleSensor->Get() == 1)
					{
						break;
					}
				}
				bBallAngle = GetFPGATime() - startTimeAngle;					
			}
			
			// set speed of shooters
			encoderTimeCurrent = GetFPGATime();
			encoderTopCurrent = encoderShooterTop->Get();
			encoderBottomCurrent = encoderShooterBottom->Get();

			//new_filtered_value = K*previous_filtered_value + (1-K)* new_sample
			//"K" is a tuning constant, which you use to adjust the "strength" of the filter. K must be in the range zero to +1. When K=0, there is no filtering. When K=1, the filtering is so "strong" that the filtered value never changes.

			float tuningConstant = .05;
			bBallTopWheelSpeed = tuningConstant * bBallTopWheelSpeed + 
					(1-tuningConstant) * (encoderTopCurrent-encoderTopLast)*100000 / ((encoderTimeCurrent - encoderTimeLast));
			bBallBottomWheelSpeed = tuningConstant * bBallBottomWheelSpeed + 
					(1-tuningConstant) * (encoderBottomCurrent-encoderBottomLast)*100000 / ((encoderTimeCurrent - encoderTimeLast));


			encoderTopLast = encoderTopCurrent;
			encoderBottomLast = encoderBottomCurrent;
			encoderTimeLast = encoderTimeCurrent;
			
			//greenLightControl->SetRaw(10);

			if (isShootStick != driverStationControl->GetDigitalIn(1))
			{
			}
			
			// Drive //////////////////////////////
			float speedAdjust = (driverStationControl->GetAnalogIn(4)/5);
			myRobot->Periodic(-xboxDrive->GetLeftY() * speedAdjust,
					-xboxDrive->GetRightY() * speedAdjust);	 // drive with tank style

			// Aim ////////////////////////////////
			bBallRotator->Set(-xboxShoot->GetRightX()/2);
			bBallPitchMotor->Set((xboxShoot->GetLeftY()/2.1));
			
			// Collect and Shoot bBalls///////////

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

			
			shooterArm->Set(xboxShoot->GetA());

			// collector ///////////////////////////////
			if (xboxDrive->GetLB() || xboxDrive->GetLeftTrigger() > .1)
			{
				bBallCollector->Set(-1.0);
			}
			else if ((xboxDrive->GetRB() || xboxDrive->GetRightTrigger() < -.1) && !robotElevator->IsRunning)
			{
				bBallCollector->Set(1.0);						
			}
			else
			{
				bBallCollector->Set(0.0);
			}
			
			// ELEVATORS //////////////////////////////
			
			// elevator system
			robotRampArm->PeriodicSystem(xboxDrive->GetLeftStickClick());


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
			
			robotElevator->PeriodicSystem(xboxShoot->GetY());
			

			
			//Reset DeadReckoner
			if(xboxDrive->GetSelect())
			{
				myRobot->ResetPosition();
			}
			
			
			
			// ramp arm kick off
//			rampArm->Set(rampArmSwitch.State(xboxDrive->GetLeftStickClick()));
			
			// ramp arm servo test
			
//			if (xboxDrive->GetRightStickClick())
//			{
//				rampServo->SetAngle(102.0);
//			}
//			else
//			{
//				rampServo->SetAngle(0.0);
//			}
			
			
			//Robot Server///////////
			//printf("%d\n", val);
			memset(msgBuf, 0, sizeof(char) * 1024);
			if (msgQReceive(robotQueue, msgBuf, 1024, NO_WAIT) != ERROR) {
//				printf("Got a message: %s\n", msgBuf);
				
				processVisionBridge(msgBuf);
				//speed = atoi(msgBuf);
				//speed /= 1000;
				//printf("speed: %f", speed);
			}
			
//			bBallRotator->Set(speed1);
			//bBallShooterBottom->Set(speed2);
			greenLightControl->SetRaw(lightValue);
			
			
			// random output stuff!! ////////////
			
			
			
			if(loopCount % 50 == 0)
			{
				
				Debug();
			}
			
			loopCount++;
			Wait(0.005);
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

//		printf("a: %f/n",*computedAngle);
		//32.2 ft/s/s
		// =SQRT(32.2*(height+(SQRT(height^2+distance^2))))
		*computedSpeed = sqrt(32.2*(height+(sqrt(pow(height,2.0)+pow(distance,2.0)))));
//		printf("S: %f/n",*computedSpeed);
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
	
	void Debug()
	{
//		printf("l:%f", myRobot->leftMotorSetting);
//		printf("r:%f", myRobot->rightMotorSetting);

//		printf("x:%f", myRobot->PositionX());
//		printf(" y:%f", myRobot->PositionY());		
//		printf(" h:%f", myRobot->Heading());
		
		//printf("X position: %f  ", myRobot->PositionX() );
		//printf("Y position: %f  ", myRobot->PositionY() );
		//printf("Heading: %f", myRobot->Heading() );
		dsLCD->Clear();
//		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, " lWheel: %i", myRobot->leftCount);
//		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, " rWheel: %i", myRobot->rightCount);

		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, " tWheel: %d", bBallTopWheelSpeed);
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, " bWheel: %d", bBallBottomWheelSpeed);
		dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, " tRot:%i", encoderTurretRotation->Get());

		dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, " Angle:%d", bBallAngle);

//		dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, " Angle:%f", myRobot->Heading());
//		dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, " X:%f", myRobot->PositionX());
//		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, " Y:%f", myRobot->PositionY());
		
		

		dsLCD->UpdateLCD();
	}

	//TODO: check the angle of shooter.
	//TODO: Play with Rampup code to get everybody's needs

};

START_ROBOT_CLASS(Robot2012);
