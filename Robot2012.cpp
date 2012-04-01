#include "WPILib.h"
#include "math.h"
#include "SubSystems/ElevatorSystem.h"
#include "SubSystems/Drivetrain.h"
#include "SubSystems/RampArm.h"
#include "Util/Switch.h"
#include "Util/XboxController.h"
#include "Util/DeadReckoner.h"
#include "Settings.h"
#include "RobotSupport.cpp"
#include "Vision/VisionProcessorBridge.h"

const static float TURRET_ROTATION_TICKS = 1488;

//#include "SubSystems/Shooter.h"

vspMessage myVSPMessage;

uint32_t lastMessageSeen = 0;

vspMessage * getVSPMessage() {
	return &myVSPMessage;
}

//Robot Server///////////
STATUS tcpServer (void);

class Smoother
{
public:
	float TUNING_CONSTANT;
	Smoother (float StartReading)
	{
		lastReading = StartReading;
		TUNING_CONSTANT = .5;
	}

	float NewValue(float CurrentReading)
	{
		//new_filtered_value = K*previous_filtered_value + (1-K)* new_sample
		CurrentReading = (TUNING_CONSTANT * lastReading) + ((1-TUNING_CONSTANT) * CurrentReading);
		lastReading = CurrentReading;
		return CurrentReading;
	}
private:
	float lastReading;
};

class EncoderSmoother 
{
public:
	double PIDSCALE;
	EncoderSmoother(Encoder* enc) 
	{
		/*"K" is a tuning constant, which you use to adjust the "strength" of the filter.
		 * K must be in the range zero to +1. When K=0, there is no filtering. When K=1, the
		 * filtering is so "strong" that the filtered value never changes.
		 */
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

	double Get()
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

	EncoderSmoother* TopShooterSmoothed;
	EncoderSmoother* BottomShooterSmoothed;
	Smoother* TiltReadingSmoothed;

	// messages from the computer
	float msgAngleUp,
	msgDistance,
	msgRightBasketAngle,
	msgCenterBasketAngle,
	msgLeftBasketAngle;

	bool shooterWheelState;
	bool shooting;
	
	// global current wheel speed
	float bBallTopWheelSpeed,
	bBallBottomWheelSpeed,
	bBallAngle,
	bBallRotationLocation;


	//	double PIDReading;
	UINT8 lightValue;
	
	UINT32 opShooterTimer;
	

	PIDScale* PIDTopWheel;
	PIDScale* PIDBottomWheel;
	PIDScale* PIDTiltReading;
	PIDScale* PIDTurretRotation;
	
	
	DigitalInput *testCompressor;
	

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
		encoderTurretRotation = new Encoder(2, 3, 2, 4);
		encoderShooterTop = new Encoder(2, 5, 2, 6, false, Encoder::k1X);
		encoderShooterBottom = new Encoder(2, 7, 2, 8, false, Encoder::k1X);

		tilt = new AnalogChannel(1);

		encoderTurretRotation->Start();
		encoderShooterTop->Start();
		encoderShooterBottom->Start();

		TopShooterSmoothed = new EncoderSmoother(encoderShooterTop);
		BottomShooterSmoothed = new EncoderSmoother(encoderShooterBottom);
		TiltReadingSmoothed = new Smoother(tilt->GetVoltage());

		bBallTopWheelSpeed = 0.0;
		bBallBottomWheelSpeed = 0.0;
		bBallAngle = 2.0;
		bBallRotationLocation = 0.0;

		PIDTopWheel = new PIDScale(11.45, 0.0, 231.85, 1200.0);
		PIDBottomWheel = new PIDScale(64.2, 0.0, 2500.0, 1200.0);
		PIDTiltReading = new PIDScale(.25, 0.0, 0.0, 0.287);
		PIDTurretRotation = new PIDScale(1.0, 0.0, 1.0, 1440.0);

		shooterWheelState = false;
		shooting = false;
		
		opShooterTimer = 0;
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

		msgAngleUp = 0.0;
		msgDistance = 0.0;
		msgRightBasketAngle = 0.0;
		msgCenterBasketAngle = 0.0;
		msgLeftBasketAngle = 0.0;

		myVSPMessage.semVSPMessage = semMCreate(SEM_Q_FIFO);

		//Robot Server///////////

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
		encoderWheelsLeft = new Encoder(1, 1, 1, 2, true);
		encoderWheelsRight = new Encoder(1, 3, 1, 4, false);
		encoderWheelsLeft->Start();
		encoderWheelsRight->Start();
		// Drive System /////////////////////
		myRobot = new Drivetrain(3, 4, 1, 2, encoderWheelsLeft, encoderWheelsRight); // create robot drive base
		// other
		// Air
		airCompressor = new Compressor(1, 10, 2, 3);
		airCompressor->Start();
		
		testCompressor = new DigitalInput(1,10);

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

	void processVisionBridge(char * msg) 
	{
		char * workingMessage = strtok(msg,",");


		do 
		{
			double value = atof(&workingMessage[1]);

			switch(workingMessage[0]) 
			{
			case 'a': msgAngleUp = value; break;
			case 'd': msgDistance = value; break;
			case 'r': msgRightBasketAngle = value; break;
			case 'm': msgCenterBasketAngle = value; break;
			case 'l': msgLeftBasketAngle = value; break;
			}

		} while (workingMessage = strtok(0, ","));
	}
	
	void processVSPMessage()
	{
		semTake(myVSPMessage.semVSPMessage, WAIT_FOREVER);

		if (myVSPMessage.count > lastMessageSeen) {
			lastMessageSeen = myVSPMessage.count;
			processVisionBridge(myVSPMessage.buf);
		}

		semGive(myVSPMessage.semVSPMessage);
	}

	void SetAngle(float degrees)
	{

	}

	/**
	 * Drive left & right motors for 2 seconds, enabled by a jumper (jumper
	 * must be in for autonomous to operate).
	 */
	void Autonomous(void)
	{
		
		greenLightControl->SetRaw(255);

		myRobot->ResetPosition();
		shooterArm->Set(false);

		UINT32 waitForArmFirstShot = 0;
		//UINT32 waitForArmSecondShot = 0;
		UINT32 wheelSpinupStart = 0;
		UINT32 loadSecondBallWait = 0;
		UINT32 timer = 0;
		UINT32 elapsedTime = 0;
		UINT32 loadBallWait = 0;
		
		int state = 0;
		
		//Auto one steps
		bool startCycle = true;
		bool shotFirstBall = false;
		bool shotSecondBall = false;
		bool drivenToBridge = false;
		bool shooting = false;
		bool autoDone = false;
		bool backedUp = false;
		bool ballLoaded = true;
		
		float
		calculatedBottom = 0,
		calculatedTop = 0,
		// To keep in range just in case
		calculatedAngle = 2.0;

		bBallAngle = 2.0;
		bBallBottomWheelSpeed = 0.0;
		bBallTopWheelSpeed = 0.0;
		bBallRotationLocation = 0.0;

		while(IsAutonomous() && IsEnabled())
		{
//			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "top wheel %f", TopShooterSmoothed->Get());
//			dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "bottom wheel %f", BottomShooterSmoothed->Get());
			

			/* this is the section for 1
			 * turn OFF 
			 * 1
			 */
			
			if(!driverStationControl->GetDigitalIn(1))
			{
				bBallAngle = 2.0816; 

				//Set Wheel Speeds to full   
				bBallTopWheelSpeed=800;
				bBallBottomWheelSpeed=1300;

				//ramp arm down
				if(robotRampArm->IsRampUp)
				{
					robotRampArm->PeriodicSystem(true);
				}

				if(startCycle)
				{
					//TODO:Wait 10 cycles
					if( BottomShooterSmoothed->Get() > 1050)
					{
						//Shoot
						shooterArm->Set(true);

						if(waitForArmFirstShot == 0)
						{
							waitForArmFirstShot = GetFPGATime();
						}
					}

					if(GetFPGATime() - waitForArmFirstShot > 5000000)
					{
						shooterArm->Set(false);
						shotFirstBall = true;
						startCycle = false;
					}
				}
				//drive to bridge
				if(shotFirstBall && myRobot->PositionY() < 71.0)
				{
					myRobot->Periodic((driverStationControl->GetAnalogIn(4)/5), (driverStationControl->GetAnalogIn(4)/5), true);
					bBallAngle = 2.0;
					bBallBottomWheelSpeed = 1100.0;
					bBallTopWheelSpeed = 270.0;
					bBallElevatorTop->Set(Relay::kOn);
					bBallElevatorTop->Set(Relay::kReverse);
				}

				//reached bridge
				else
				{
					bBallElevatorTop->Set(Relay::kOff);
					myRobot->Periodic(0.0, 0.0, true);
				}
				if(myRobot->PositionY() >= 71.0)
				{
					drivenToBridge = true;
				}
				if(drivenToBridge)
				{
					shooterArm->Set(true);
				}
			}

			
			/* this is the section for 2
			 * turn OFF 
			 * 2
			 */

			//Freethrow Line
			if(!driverStationControl->GetDigitalIn(2))
			{
				bBallAngle = 2.04; 
				bBallTopWheelSpeed = 170.0;
				bBallBottomWheelSpeed = 1080.0;
				shooterWheelState = true;

				if(wheelSpinupStart == 0)
				{
					wheelSpinupStart = GetFPGATime();
				}
				//wait for time
				if(GetFPGATime() - wheelSpinupStart < 6000000)
				{
					// wait for go time
				}
				else
				{
					if(waitForArmFirstShot == 0)
					{
						shooting = true;
						shooterArm->Set(true);
						waitForArmFirstShot = GetFPGATime();
					}
					if(GetFPGATime() - waitForArmFirstShot > 500000)
					{
						shooterArm->Set(false);
						shooting = false;
						shotFirstBall = true;
						startCycle = false;

					}
					if(shotFirstBall)
					{
						//set elevator up
						bBallElevatorTop->Set(Relay::kOn);
						bBallElevatorTop->Set(Relay::kReverse);

						if(loadSecondBallWait == 0)
						{
							loadSecondBallWait = GetFPGATime();
						}
						if(GetFPGATime() - loadSecondBallWait < 5000000)
						{
							
						}
						else if(GetFPGATime() - loadSecondBallWait < 5500000)
						{
							bBallElevatorTop->Set(Relay::kOff);
							shooting = true;
							shooterArm->Set(true);
						}
						else
						{
							shooting = false;
							shooterArm->Set(false);
						}
					}
				}
			}
			
			
			
			
			//Freethrow Line 2 (switch 3)
			if(!driverStationControl->GetDigitalIn(3))
			{
				myRobot->Periodic(0.0, 0.0, true);
				bBallAngle = 2.06;
				bBallTopWheelSpeed = 170.0;
				bBallBottomWheelSpeed = 1080.0;
				
				shooterWheelState = true;

				processVSPMessage();
				
				if(wheelSpinupStart == 0)
				{
					wheelSpinupStart = GetFPGATime();
				}
				
				elapsedTime = GetFPGATime() - wheelSpinupStart;
				
				//wait for time
				if(elapsedTime < 6000000)
				{
					// wait for go time
				}
				//shoot first
				else if(elapsedTime < 6500000)
				{
					shooting = true;
					shooterArm->Set(true);
				}
				//pull arm back && run elevator
				else if(elapsedTime < 13000000)
				{
					//set elevator up
					bBallElevatorTop->Set(Relay::kOn);
					bBallElevatorTop->Set(Relay::kReverse);
					
					shooting = false;
					shooterArm->Set(false);
				}
				//shoot again
				else if(elapsedTime < 13500000)
				{
					bBallElevatorTop->Set(Relay::kOff);
					shooting = true;
					shooterArm->Set(true);
				}
				//stop shooting
				else if(elapsedTime < 14000000)
				{
					shooting = false;
					shooterArm->Set(false);
				}
			}

			/*
			 * ========================= Select 4 ==============================
			 * 
			 * Drive to bridge
			 * Lower Bridge
			 * Drive back
			 * Shoot twice
			 */
			
			if(!driverStationControl->GetDigitalIn(4))
			{
				
				//set angle to make it from top of key
				bBallAngle = 2.04;

				//Set Wheel Speeds to make it from the bridge    
				bBallTopWheelSpeed = 400.0;
				bBallBottomWheelSpeed = 1100.0;
				
				shooterWheelState = true;
				
				processVSPMessage();
				
				if(state == 0)
				{
					robotRampArm->PeriodicSystem(true);
					state++;
				}
				//drive to bridge
				if(state == 1)
				{
					if(myRobot->PositionY() < 60.0)
					{
						myRobot->Periodic(-(driverStationControl->GetAnalogIn(4)/5.), -(driverStationControl->GetAnalogIn(4)/5.), true);
					}
					else
					{
						state++;
					}
				}
				//wait
				if(state == 2)
				{
					if(timer == 0)
					{
						timer = GetFPGATime();
					}
					if(GetFPGATime() - timer < 2000000)
					{
						myRobot->Periodic(0.0, 0.0, true);
					}
					else
					{
						state++;
						timer = 0;
					}
				}
				//drive back
				if(state == 3)
				{
					if(myRobot->PositionY() >= 0.0)
					{
						myRobot->Periodic((driverStationControl->GetAnalogIn(4)/5.), (driverStationControl->GetAnalogIn(4)/5.), true);
					}
					else
					{
						state++;
					}
				}
				//shoot
				if(state == 4)
				{
					myRobot->Periodic(0.0, 0.0, true);
					if(timer == 0)
					{
						timer = GetFPGATime();
					}
					
					elapsedTime = GetFPGATime() - timer;
					
					if(elapsedTime < 500000)
					{
						
					}
					else if(elapsedTime < 600000)
					{
						OpShooter(true);
						robotElevator->PeriodicSystem(true);
						robotRampArm->PeriodicSystem(true);
					}
					else if(elapsedTime < 5000000)
					{
						//wait
					}
					else if(elapsedTime < 5100000)
					{
						OpShooter(true);
					}
				}
				robotRampArm->PeriodicSystem(false);
				robotElevator->PeriodicSystem(false);
				OpShooter(false);
			}
			
			
			
			
			/*
			 * ========================= Select 5 ==============================
			 * 
			 * Drive to bridge and shoot on the way
			 * Lower Bridge
			 * Drive back
			 * Shoot again
			 */
			
			if(!driverStationControl->GetDigitalIn(5)){
				/*
				 * VERY Rough calculation of wheel speeds 
				 * could not find direct correlation between
				 * distance and wheel speed
				 * 
				 * 194.5 = initial position from basket)
				 */
				calculatedAngle = (myRobot->PositionY() + 194.5) / 102;
				calculatedTop = (myRobot->PositionY() + 194.5) * 1.48;
				calculatedBottom = (myRobot->PositionY() + 194.5) * 6.19;
				
				bBallAngle = calculatedAngle;
				bBallTopWheelSpeed = calculatedTop;
				bBallBottomWheelSpeed = calculatedBottom;
				
				//Turn on shooter wheels
				shooterWheelState = true;

				//Follow Turret
				processVSPMessage();				

				//Ramp arm down
				if(robotRampArm->IsRampUp && !autoDone)
				{
					robotRampArm->PeriodicSystem(true);
				}
				
				
				//Shoot first ball on the way
				if(!shotFirstBall){
					//Check if wheels are at right speed for position
					if(BottomShooterSmoothed->Get() >= calculatedBottom - 20 &&
							TopShooterSmoothed->Get() >= calculatedTop -20 && 
							(calculatedAngle > tilt->GetVoltage() - 0.08 && 
									calculatedAngle < tilt->GetVoltage() + 0.08
							)
					)
					{
						OpShooter(true);
						shotFirstBall = true;
						ballLoaded = false;
					}
					else{
						OpShooter(false);
					}
				}
				
				if(!ballLoaded){
					bBallElevatorTop->Set(Relay::kOn);
					bBallElevatorTop->Set(Relay::kReverse);

					if(loadBallWait == 0)
					{
						loadBallWait = GetFPGATime();
					}
					if(GetFPGATime() - loadBallWait > 4000000)
					{
						loadBallWait = 0;
						ballLoaded = true;
					}
				}
				
				//Drive to bridge
				if(!drivenToBridge){
					if(myRobot->PositionY() < 60){
						myRobot->Periodic(-(driverStationControl->GetAnalogIn(4)/5), -(driverStationControl->GetAnalogIn(4)/5), true);
					}
					else{
						myRobot->Periodic(0.0, 0.0, true);
						drivenToBridge = true;
					}
				}
				
				//Got to bridge, now backup
				if(drivenToBridge){
					if(myRobot->PositionY() > 50){
						myRobot->Periodic((driverStationControl->GetAnalogIn(4)/5), (driverStationControl->GetAnalogIn(4)/5), true);
					}
					else{
						myRobot->Periodic(0.0, 0.0, true);
						backedUp = true;
					}
				}
				
				//Backed up, now shoot again
				if(backedUp && shotFirstBall && !shotSecondBall && ballLoaded){
					OpShooter(true);
					shotSecondBall = true;
					autoDone = true;
				}
				
				if(autoDone){
					if(!robotRampArm->IsRampUp && autoDone)
					{
						robotRampArm->PeriodicSystem(true);
					}
				}
				
			}



			/*
			 * ========================= Select 6 ==============================
			 * 
			 * Drive to bridge
			 * Lower bridge
			 * Shoot twice
			 * Back up
			 * Raise Arm
			 */
			
			if(!driverStationControl->GetDigitalIn(6))
			{
				//set angle to make it from the bridge
				bBallAngle = 2.01;


				//Set Wheel Speeds to make it from the bridge    
				bBallTopWheelSpeed = 400.0;
				bBallBottomWheelSpeed = 1320.0;
//				bBallBottomWheelSpeed = 1800.0;
//				bBallTopWheelSpeed = driverStationControl->GetAnalogIn(1)*1300/5;
//				bBallBottomWheelSpeed = driverStationControl->GetAnalogIn(2)*1300/5;
				
				//Turn on shooter wheels
				shooterWheelState = true;

				//Follow Turret
				processVSPMessage();				
				
				//Ramp arm down
				if(robotRampArm->IsRampUp && !autoDone)
				{
					robotRampArm->PeriodicSystem(true);
				}

				//drive to bridge
				if(myRobot->PositionY() < 60.0)
				{
					myRobot->Periodic(-(driverStationControl->GetAnalogIn(4)/5.), -(driverStationControl->GetAnalogIn(4)/5.), true);
				}

				//reached bridge
				else
				{
					if(!shotSecondBall)
					{
						myRobot->Periodic(0.0, 0.0, true);
					}
				}
				if(myRobot->PositionY() >= 60.0)
				{
					if(wheelSpinupStart == 0)
					{
						wheelSpinupStart = GetFPGATime();
					}
					if(GetFPGATime() - wheelSpinupStart > 5000000)
					{
						drivenToBridge = true;
					}
				}
				if(drivenToBridge && !shotFirstBall)
				{
					//take first shot
					shooting = true;
					shooterArm->Set(true);
					if(waitForArmFirstShot == 0){
						waitForArmFirstShot = GetFPGATime();
					}
					if(GetFPGATime() - waitForArmFirstShot > 400000)
					{
						shooting = false;
						shooterArm->Set(false);
						shotFirstBall = true;
					}
				}
				
				if(shotFirstBall)
				{
					//set elevator up
					bBallElevatorTop->Set(Relay::kOn);
					bBallElevatorTop->Set(Relay::kReverse);

					if(loadSecondBallWait == 0)
					{
						loadSecondBallWait = GetFPGATime();
					}
					if(GetFPGATime() - loadSecondBallWait > 4000000)
					{
						shooting = true;
						shooterArm->Set(true);
						
						shotSecondBall = true;
						
					}
				}
				if(shotSecondBall)
				{
					if(myRobot->PositionY() >= 30.0)
					{
						myRobot->Periodic((driverStationControl->GetAnalogIn(4)/5.), (driverStationControl->GetAnalogIn(4)/5.), true);
					}
					else
					{
						autoDone = true;
						if(!robotRampArm->IsRampUp && autoDone)
						{
							robotRampArm->PeriodicSystem(true);
						}
					}
				}
			}

			driverStationControl->SetDigitalOut(3, shotSecondBall);
			OpTurret(false);
			Debug();
			Wait(.005);
		}
	}

	void OpShooter(bool wantToShoot)
	{
		if(wantToShoot && !shooting)
		{
			shooting = true;
		}
		
		if(shooting)
		{
			
			if(opShooterTimer == 0)
			{
				opShooterTimer = GetFPGATime();
			}
		
			if(GetFPGATime() - opShooterTimer < 500000)
			{
				shooterArm->Set(true);
			}
			else if(GetFPGATime() - opShooterTimer < 1000000)
			{
				shooterArm->Set(false);
			}
			else
			{
				shooting = false;
				opShooterTimer = 0;
			}
		}
	}
	
	
	
	
	
	
	
	
	
	void OpTurret(bool gotMessage)
	{
		// Aim ////////////////////////////////
		//----------

//		float rotationChange = PIDTurretRotation->CalculateChange((float)(encoderTurretRotation->Get()), driverStationControl->In(4)*1488.0/2.5 - 1488);
		float rotationChange = PIDTurretRotation->CalculateChange((float)(encoderTurretRotation->Get()), bBallRotationLocation);

		//rotator here.....--------------------------------------------------------------------------

		
		///Track Cener Basket
		// -10000 == don't know
		if(msgCenterBasketAngle > -6000 && driverStationControl->GetDigitalIn(8))
		{
			rotationChange = msgCenterBasketAngle / 10.0;
			
			if( rotationChange < 100.0 && rotationChange > .015)
			{
				rotationChange = 0.125;
			}
			else if( rotationChange > -100.0 && rotationChange < -.015)
			{
				rotationChange = -0.125;
			}
			bBallRotationLocation = encoderTurretRotation->Get();
		}


		if(fabs(xboxShoot->GetRightX()) > .2)
		{
			rotationChange = xboxShoot->GetRightX();
			bBallRotationLocation = encoderTurretRotation->Get();			
		}

		if(encoderTurretRotation->Get() > TURRET_ROTATION_TICKS && bBallRotationLocation > 0.0){
			rotationChange = 0.0;
		}
		// If rotation too low, will stop the motor
		if(encoderTurretRotation->Get() < -TURRET_ROTATION_TICKS && bBallRotationLocation < 0.0){
			rotationChange = 0.0;
		}

		bBallRotator->Set(rotationChange);


		//---------------------------------------------Tilt--------------------------------

		float tiltChange = PIDTiltReading->CalculateChange(TiltReadingSmoothed->NewValue(tilt->GetVoltage()),bBallAngle);

		if(tiltChange > -.22 && tiltChange < -.01)
		{
			tiltChange = -.22;
		}
		else if(tiltChange < .33 && tiltChange > .01)
		{
			tiltChange = .33;
		}
		
		if(fabs(xboxShoot->GetLeftY()) > 0.2)
		{
			tiltChange = xboxShoot->GetLeftY();
			bBallAngle = TiltReadingSmoothed->NewValue(tilt->GetVoltage());
			
			//down
			if(tiltChange < 0 )
			{
				tiltChange /= 4.0;
			}
			//up
			else
			{
				tiltChange /= 2.1;
			}
		}
//      this code is for vpc
//		else
//		{
//			if(tiltChange < 0 )
//			{
//				bBallAngle = tiltChange * 8.0;
//			}
//			else
//			{
//				bBallAngle = tiltChange * 10.0;
//			}
//		}
		// If tilt is too high, will SAY NO!
		if(tilt->GetVoltage() > 2.28 && tiltChange > 0.0){
			tiltChange = 0.0;
		}
		// If tilt too low, will stop the motor
		if(tilt->GetVoltage() < 2.0 && tiltChange < 0.0){
			tiltChange = 0.0;
		}
		bBallPitchMotor->Set(tiltChange);
		
		
		//------------------------------- spin wheels -----------------------
		TopShooterSmoothed->Update();
		BottomShooterSmoothed->Update();
		
		//Spin up wheels
		float topChange = PIDTopWheel->CalculateChange(TopShooterSmoothed->Get(),bBallTopWheelSpeed);
		float botChange = PIDBottomWheel->CalculateChange(BottomShooterSmoothed->Get(),bBallBottomWheelSpeed);

		// Collect and Shoot bBalls///////////
		// right button or right trigger (ignoring accidents)
		if(xboxShoot->GetRB() || xboxShoot->GetRightTrigger() < -.1){
			shooterWheelState = true;
		}

		// left button or left trigger (ignoring accidents)
		if(xboxShoot->GetLB() || xboxShoot->GetLeftTrigger() > .1){
			shooterWheelState = false;
		}

		driverStationControl->SetDigitalOut(1, shooterWheelState);
		driverStationControl->SetDigitalOut(2, shooterArm->Get());
		if(shooterWheelState)
		{
			
			
			if (!shooterArm->Get() || PIDTopWheel->CurrentMotorValue < 0.1)
			{
				PIDTopWheel->CurrentMotorValue += topChange;
				if (PIDTopWheel->CurrentMotorValue > 1) { PIDTopWheel->CurrentMotorValue = 1; }
				if (PIDTopWheel->CurrentMotorValue < 0) { PIDTopWheel->CurrentMotorValue = 0; }
				PIDBottomWheel->CurrentMotorValue += botChange;
				if (PIDBottomWheel->CurrentMotorValue > 1) { PIDBottomWheel->CurrentMotorValue = 1; }
				if (PIDBottomWheel->CurrentMotorValue < 0) { PIDBottomWheel->CurrentMotorValue = 0; }
			}

			bBallShooterTop->Set(PIDTopWheel->CurrentMotorValue);
			bBallShooterBottom->Set(PIDBottomWheel->CurrentMotorValue);
			
		}
		else
		{
			PIDTopWheel->zeroOut();
			PIDBottomWheel->zeroOut();        	
			bBallShooterTop->Set(0.0);
			bBallShooterBottom->Set(0.0);
		}
	}

	void OpElevator()
	{
		// ELEVATORS //////////////////////////////
		// REVERSE IS UP ///////////
		// bottom down
		if(xboxShoot->GetSelect()){
			robotElevator->ManualFreezeAll();
			bBallElevatorBottom->Set(Relay::kOn);
			bBallElevatorBottom->Set(Relay::kForward);
		}else
			// bottom up
			if(xboxShoot->GetStart()){
				robotElevator->ManualFreezeAll();
				bBallElevatorBottom->Set(Relay::kOn);
				bBallElevatorBottom->Set(Relay::kReverse);
			}else{
				bBallElevatorBottom->Set(Relay::kOff);
			}

		// top down			
		if(xboxShoot->GetX()){
			bBallElevatorTop->Set(Relay::kOn);
			bBallElevatorTop->Set(Relay::kForward);
		}else
			// top up
			if(xboxShoot->GetB()){
				bBallElevatorTop->Set(Relay::kOn);
				bBallElevatorTop->Set(Relay::kReverse);
			}else{
				bBallElevatorTop->Set(Relay::kOff);
			}

		robotElevator->PeriodicSystem(xboxShoot->GetY());
	}

	void OpDriveCollectRampReset()
	{
		// Drive //////////////////////////////
		//        float speedAdjust = (driverStationControl->In(4) / 5);
		float speedAdjust = 1.0;
		myRobot->Periodic(xboxDrive->GetLeftY() * speedAdjust, xboxDrive->GetRightY() * speedAdjust, false, // don't use the encoder adjustment now
				xboxDrive->GetA()); // check for stop system!!
		// collector ///////////////////////////////
		if(xboxDrive->GetLB() || xboxDrive->GetLeftTrigger() > .1){
			bBallCollector->Set(-1.0);
		}else
			if((xboxDrive->GetRB() || xboxDrive->GetRightTrigger() < -.1) && !robotElevator->IsRunning){
				bBallCollector->Set(1.0);
			}else{
				bBallCollector->Set(0.0);
			}

		//Reset DeadReckoner
		if(xboxDrive->GetSelect()){
			myRobot->ResetPosition();
		}
		// ramp arm
		robotRampArm->PeriodicSystem(xboxDrive->GetLeftStickClick());
	}

	void OperatorControl(void)
	{
		// any setup?
		int loopCount = 0;
		int lastMessageLoopCount = 0;

		Switch rampArmSwitch;
		Switch rampServoSwitch;

		rampArm->Set(false);
		robotElevator->ManualFreezeAll();
		shooterWheelState = false;
		
		//bBallBottomWheelSpeed var
		bBallAngle = 2.0;
		bBallRotationLocation = 0.0;

		while (IsOperatorControl())
		{
			//If no button is pushed manual angle and wheel speed
			if(!xboxDrive->GetB() && !xboxDrive->GetX())
			{
				bBallTopWheelSpeed = driverStationControl->GetAnalogIn(1)*1000.0;
				bBallBottomWheelSpeed = driverStationControl->GetAnalogIn(2)*1000.0;
			}
			//if B on drive stick shoot from freethrow line
			else if(xboxDrive->GetB())
			{
				bBallAngle = 2.04; 
				bBallTopWheelSpeed = 170.0;
				bBallBottomWheelSpeed = 1080.0;
			}
			//if X on drive stick shoot from side of our bridge
			else if(xboxDrive->GetX())
			{
				bBallAngle = 2.04;
				bBallTopWheelSpeed = 915.0;
				bBallBottomWheelSpeed = 1300.0;
			}
			
			greenLightControl->SetRaw(255);

			//Robot Server///////////
			//printf("%d\n", val);

			processVSPMessage();

			/* 
			 * The commented out lines below are now enclosed within
			 * the function processVSPMessage()
			 */
			
//			semTake(myVSPMessage.semVSPMessage, WAIT_FOREVER);
//
//			if (myVSPMessage.count > lastMessageSeen) {
//				lastMessageSeen = myVSPMessage.count;
//				processVisionBridge(myVSPMessage.buf);
//			}
//
//			semGive(myVSPMessage.semVSPMessage);

			OpTurret(lastMessageLoopCount+20 < loopCount);
			OpDriveCollectRampReset();
			OpElevator();

			//------------ shoot the ball with the arm ---------------------//
			OpShooter(xboxShoot->GetA());

			
			// log once in a while, not every time
			Debug();

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
		static UINT32 loopCount = 0;
		loopCount++;
		if(loopCount % 20)
		{
			return;
		}
		dsLCD->Clear();
		//msgCenterBasketAngle
		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, " l,rWheel: %i, %i", myRobot->leftCount, myRobot->rightCount);
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, " t,bWheel: %i, %i", encoderShooterTop->Get(), encoderShooterBottom->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, " tlt,rot: %f, %i", tilt->GetVoltage(),encoderTurretRotation->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "top wheel %f", TopShooterSmoothed->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "bottom wheel %f", BottomShooterSmoothed->Get());

		//		dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, " top: %f", PIDTopWheel->CurrentMotorValue );
		
//		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, " bot0 :%f", PIDBottomWheel->CurrentMotorValue);
//		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, " message :%f", msgCenterBasketAngle);
		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "SWco,to,bo: %d, %d, %d", testCompressor->Get(), bBallElevatorTopLimit->Get(), bBallElevatorBottomLimit->Get());
		
		printf("top %f", TopShooterSmoothed->Get());
		printf(" bottom %f ", BottomShooterSmoothed->Get());
		printf(" basket %f \n", msgCenterBasketAngle);
		dsLCD->UpdateLCD();
	}

	//TODO: check the angle of shooter.
	//TODO: Play with Rampup code to get everybody's needs

};

START_ROBOT_CLASS(Robot2012);
