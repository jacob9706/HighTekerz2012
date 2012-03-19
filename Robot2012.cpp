#include "LowLevel/Encoder3574.h"
//#include "LowLevel/PIDController3574.h"
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
STATUS tcpServer (void);
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
	double PIDSCALE;
	SamplePIDSource(Encoder3574* enc) 
	{
		//"K" is a tuning constant, which you use to adjust the "strength" of the filter. K must be in the range zero to +1. When K=0, there is no filtering. When K=1, the filtering is so "strong" that the filtered value never changes.
		TUNING_CONSTANT = .8;
		SCALE = 100000.0;
		PIDSCALE = 1;
		_enc = enc;
		encoderCurrent = 0;
		encoderLast = 0;
		timeLast = GetFPGATime();
		timeCurrent = 0;
		rate = 0;
		newRate = 0;
	}

	int hi()
	{
		return encoderLast;
	}

	double PIDGet()
	{
		return rate / PIDSCALE;
	}

	void Update()
	{
		encoderCurrent = _enc->Get();
		timeCurrent = GetFPGATime();
		newRate = (encoderCurrent-encoderLast)*SCALE / (timeCurrent - timeLast);

		//new_filtered_value = K*previous_filtered_value + (1-K)* new_sample
		rate = (TUNING_CONSTANT * rate) + ((1-TUNING_CONSTANT) * newRate);

		encoderLast = encoderCurrent;
		timeLast = timeCurrent;
	}

	double TUNING_CONSTANT;	
	double SCALE;
private:
	Encoder3574* _enc;

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

	Encoder3574* encoderWheelsLeft;
	Encoder3574* encoderWheelsRight;
	Encoder3574* encoderTurretRotation;
	Encoder3574* encoderShooterTop;
	Encoder3574* encoderShooterBottom;
	DigitalInput* bBallAngleSensor;

	// Switches

	DigitalInput *bBallElevatorTopLimit;
	DigitalInput *bBallElevatorBottomLimit;
	Switch* shooterSwitch;
	Switch* switchAimTrigger;

	AnalogChannel* tilt;

	// Other
//	ADXL345_I2C *accel;
//	Gyro *gyro;

	// Driver Station //

	DriverStation *driverStationControl;
	DriverStationLCD* dsLCD;

	XboxController *xboxDrive;
	XboxController *xboxShoot;

	// Systems and Support ///////////////////

	ElevatorSystem* robotElevator;
	RampArm *robotRampArm;

//	SamplePIDSource* PIDTopShooterSource;
//	SamplePIDOutput* PIDTopShooterOut;

//	SamplePIDSource* PIDBottomShooterSource;
//	SamplePIDOutput* PIDBottomShooterOut;

	//Shooter *robotShooter;

	bool shooterState;

	double speed1, speed2;
//	double PIDReading;
	UINT8 lightValue;

	int bBallTopWheelSpeed,
	bBallBottomWheelSpeed,
	bBallAngle;
	
//	float WheelSpeedAtKickTop;
//	float WheelSpeedAtKickBottom;
//	float TiltAtKick;
public:
	void SetupTurret()
	{
		bBallPitchMotor = new Victor(2, 3);
		bBallRotator = new Victor(2, 4);
		bBallShooterTop = new Victor(2, 2);
		bBallShooterBottom = new Victor(2, 1);
		shooterArm = new Solenoid(1);
		encoderTurretRotation = new Encoder3574(2, 3, 2, 4);
		bBallAngleSensor = new DigitalInput(2, 11);
		encoderShooterTop = new Encoder3574(2, 5, 2, 6, false, Encoder3574::k1X);
		encoderShooterBottom = new Encoder3574(2, 7, 2, 8, false, Encoder3574::k1X);

		tilt = new AnalogChannel(1);

		//        encoderShooterTop->SetDistancePerPulse(100);
		//        encoderShooterBottom->SetDistancePerPulse(1000);
		//        encoderShooterTop->SetPIDSourceParameter(Encoder::kRate);
		//        encoderShooterBottom->SetPIDSourceParameter(Encoder::kRate);
		encoderTurretRotation->Start();
		encoderShooterTop->Start();
		encoderShooterBottom->Start();

//		PIDTopShooterSource = new SamplePIDSource(encoderShooterTop);
//		PIDTopShooterOut = new SamplePIDOutput(bBallShooterTop);
//		PIDTopShooterSource->PIDSCALE = 1100.0;

//		PIDBottomShooterSource = new SamplePIDSource(encoderShooterBottom);
//		PIDBottomShooterOut = new SamplePIDOutput(bBallShooterBottom);

		bBallTopWheelSpeed = 0;
		bBallBottomWheelSpeed = 0;
		bBallAngle = 0;

		//		bBallBrushSensor = new DigitalInput(2,5);

		// Systems and Support ///////////////////
		//		robotShooter = new Shooter(bBallShooterTop,
		//									bBallShooterBottom,
		//									bBallRotator,
		//									bBallPitchMotor,
		//									bBallShooterTopEncoder,
		//									bBallShooterBottomEncoder,
		//									encoderTurretRotation,
		//									bBallAngle,
		//									shooterArm);

		shooterState = false;
	}

	void SetupCameras()
	{
		//setup cameras
		AxisCamera::GetInstance();
		// Outputs //////////////////////////
		// Vision
		lightValue = 0;
		greenLightControl = new PWM(1, 6);
		greenLightControl->SetPeriodMultiplier(PWM::kPeriodMultiplier_1X);
		greenLightControl->EnableDeadbandElimination(false);

		speed1 = 0;
		speed2 = 0;

		//Robot Server///////////
		robotQueue = msgQCreate(100, 1024, MSG_Q_PRIORITY);
		if (taskSpawn("tcpServer", 100, 0, 10000, 
				(FUNCPTR) tcpServer, 0,0,0,0,0,0,0,0,0,0) == ERROR) 
		{
			/* if taskSpawn fails, close fd and return to top of loop */ 

			perror ("taskSpawn"); 
		}
	}

	void SetupCollectorElevator()
	{
		// collector/elevator
		bBallElevatorBottom = new Relay(2, 1, Relay::kBothDirections);
		bBallElevatorTop = new Relay(2, 2, Relay::kBothDirections);
		bBallCollector = new Victor(2, 5);
		
		bBallElevatorTopLimit = new DigitalInput(2, 2);
		bBallElevatorBottomLimit = new DigitalInput(2, 1);

//		 mockRelay = new Relay(2,5,Relay::kBothDirections);
		
		//		bBallCollectorSensor = new DigitalInput(2,3);
		//		bBallElevatorSensor = new DigitalInput(2,4);

//		robotElevator = new ElevatorSystem(bBallElevatorTop, mockRelay, bBallElevatorBottomLimit, bBallElevatorTopLimit);
		robotElevator = new ElevatorSystem(bBallElevatorBottom,
										   bBallElevatorTop,
										   bBallElevatorBottomLimit,
										   bBallElevatorTopLimit
										  );

	}

	void SetupArm()
	{
		// ramp
		rampServo = new Servo(2, 6);
		rampArm = new Solenoid(2);
		robotRampArm = new RampArm(rampServo, rampArm);
	}

	void SetupRobot()
	{
		// On robot //
		encoderWheelsLeft = new Encoder3574(1, 1, 1, 2, true);
		encoderWheelsRight = new Encoder3574(1, 3, 1, 4, false);
		encoderWheelsLeft->Start();
		encoderWheelsRight->Start();
		// Drive System /////////////////////
		myRobot = new Drivetrain(3, 4, 1, 2, encoderWheelsLeft, encoderWheelsRight); // create robot drive base
		// other
		// Air
		airCompressor = new Compressor(1, 10, 2, 3);
		airCompressor->Start();

		// Other
		//		accel = new ADXL345_I2C(1);
		//		gyro = new Gyro(1);
	}

	void SetupDriverStation()
	{
		// Driver Station //
		driverStationControl = DriverStation::GetInstance();
		dsLCD = DriverStationLCD::GetInstance();
		xboxDrive = new XboxController(1);
		xboxShoot = new XboxController(2);
	}

	/**
	 * Create an instance of a RobotDrive with left and right motors plugged into PWM
	 */
	Robot2012(void)
	{
		SetupTurret();
		SetupCameras();
		SetupCollectorElevator();
		SetupArm();
		SetupRobot();
		SetupDriverStation();		
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
		myRobot->speedMatchLeftCounterStart = 0;
		myRobot->speedMatchRightCounterStart = 0;
		while(IsAutonomous() && IsEnabled())
		{
			if(((myRobot->rightCount + myRobot->leftCount) / 2) < 4000)
			{
				myRobot->Periodic((driverStationControl->GetAnalogIn(4)/5), (driverStationControl->GetAnalogIn(4)/5), true);
			}
			else 
			{
				myRobot->Periodic(0.0, 0.0, true);
			}
			Debug();
			Wait(.005);
		}
	}

	void OperatorControl(void)
	{
		// any setup?
		char msgBuf[1024];
		int loopCount = 0;

		Switch rampArmSwitch;
		Switch rampServoSwitch;

		rampArm->Set(false);
		robotElevator->ManualFreezeAll();

// TODO: use pid controller
//		PIDController speedcontroller( 0.1, // P
//				0.00, // I
//				0.5, // D
//				PIDTopShooterSource, // source
//				PIDTopShooterOut, // output
//				0.005); // period
//		speedcontroller.Enable();

		while (IsOperatorControl())
		{
//			PIDTopShooterSource->Update();
//			PIDBottomShooterSource->Update();
			//			PIDReading = speedcontroller.GetError();
			// get sensor feedback /////////////////////

//			//turret			

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
//				speedcontroller.SetSetpoint(0.8);
				bBallShooterTop->Set(driverStationControl->GetAnalogIn(1));
				bBallShooterBottom->Set(driverStationControl->GetAnalogIn(2));
			}
			else
			{
//				speedcontroller.SetSetpoint(0.0);
				bBallShooterTop->Set(0.0);
				bBallShooterBottom->Set(0.0);
			}

			shooterArm->Set(xboxShoot->GetA());

			//greenLightControl->SetRaw(10);

			// Drive //////////////////////////////
			float speedAdjust = (driverStationControl->GetAnalogIn(4)/5);
			myRobot->Periodic(-xboxDrive->GetLeftY() * speedAdjust,
					-xboxDrive->GetRightY() * speedAdjust);	 // drive with tank style

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

			// ramp arm
			robotRampArm->PeriodicSystem(xboxDrive->GetLeftStickClick());


			// REVERSE IS UP ///////////
			// bottom down
			if (xboxShoot->GetSelect())
			{
				robotElevator->ManualFreezeAll();
				bBallElevatorBottom->Set(Relay::kOn);
				bBallElevatorBottom->Set(Relay::kForward);						
			}
			// bottom up
			else if (xboxShoot->GetStart())
			{
				robotElevator->ManualFreezeAll();
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
			
			// this is the override to stop the motors.
			if(xboxDrive->GetA())
			{
				myRobot->TankDrive(0.0,0.0);
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

		//		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, " tWheel: %d", bBallTopWheelSpeed);
		//		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, " bWheel: %d", bBallBottomWheelSpeed);

		//		dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, " Angle:%f", bBallAngle);

		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, " tWheel: %f", encoderShooterTop->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, " bWheel: %f", encoderShooterBottom->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, " tWheel: %f", encoderShooterTop->GetFPGAPeriod()*1000);
		dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, " bWheel: %f", encoderShooterBottom->GetFPGAPeriod()*1000);
		//		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, " left: %f", myRobot->scaledLeft);
		//		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, " right: %f", myRobot->scaledRight);

//		dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, " tRot:%i", encoderTurretRotation->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, " tilt: %f", tilt->GetVoltage());

//		dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, " isRunnng: %d", robotElevator->IsRunning);
//		dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, " bottom: %d", bBallElevatorBottomLimit->Get());
//		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, " top: %d", bBallElevatorTopLimit->Get());
		
//		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, " topSwitch: %s", bBallElevatorTopLimit->Get());

//		dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, " lWheel: %i", myRobot->leftCount);
//		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, " rWheel: %i", myRobot->rightCount);

//		dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, " Angle:%f", (myRobot->Heading() / (2.0 * 3.1415927)));
//		dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, " X:%f", myRobot->PositionX());
//		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, " Y:%f", myRobot->PositionY());

//		dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, " Tilt:%f", TiltAtKick);
//		dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, " top:%f", WheelSpeedAtKickTop);
//		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, " bottom:%f", WheelSpeedAtKickBottom);


		dsLCD->UpdateLCD();
	}

	//TODO: check the angle of shooter.
	//TODO: Play with Rampup code to get everybody's needs

};

START_ROBOT_CLASS(Robot2012);
