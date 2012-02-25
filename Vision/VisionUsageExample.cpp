#include "WPILib.h"
#include "math.h"
#include "JSON.h"  //JACOB TAKES HUGE COCK IN HIS ASS


STATUS tcpServer (void) ;



MSG_Q_ID robotQueue;

MSG_Q_ID getRobotMsgQueue() {
	return robotQueue;
}

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
class Robot2012 : public SimpleRobot
{
	RobotDrive *myRobot; // robot drive system
	Victor* bridgeBar;

	Encoder* encoderWheels12;
	Encoder* encoderWheels34;
	Encoder* encoderTurretRotation;
	Encoder* bBallAngle;
	Victor* bBallRotator;
	Victor* bBallPitchMotor;
	Victor* bBallCollector1;
	Victor* bBallCollector2;
	Victor* bBallShooterTop;
	Victor* bBallShooterBottom;
	Relay* bBallElevator;
	Relay* bBallElevator2;
	Compressor* airCompressor;
	Solenoid* shooterArm;
	Solenoid* rampArm;

	DigitalInput *bBallCollectorSensor;
	DigitalInput *bBallElevatorSensor;
	DigitalInput *bBallBrushSensor;
	DigitalInput *bBallElevatorTopLimit;
	DigitalInput *bBallElevatorBottomLimit;
	DigitalInput *beamTest;
	ADXL345_I2C *accel;
	Gyro *gyro;

	I2C *mI2c;

	DriverStation *driverStationControl; // driver station object
	Joystick *rightStick; // joystick 1 (arcade stick or right tank stick)
	Joystick *leftStick; // joystick 2 (tank left stick)
	Joystick *aimStick; // joystick 3 to control arm
	Joystick* buttons;

	DriverStationLCD* dsLCD;
	bool elevatorButtonState;
	bool elevatorButtonLastState;
	bool elevatorUp;
	bool elevatorDown;

public:
	/**
	 * Create an instance of a RobotDrive with left and right motors plugged into PWM
	 */

	Robot2012(void)
	{
		myRobot = new RobotDrive(2, 1, 3, 4); // create robot drive base
		bridgeBar = new Victor(5);
		encoderWheels12 = new Encoder(1,2,true);
		encoderWheels34 = new Encoder(3,4,false);
		encoderTurretRotation = new Encoder(2,1,2,2);
		bBallAngle = new Encoder(2,6,2,7);
		bBallCollector1 = new Victor(2,1);
		bBallCollector2 = new Victor(2,2);
		bBallShooterTop = new Victor(2,4);
		bBallShooterBottom = new Victor(2,5);
		bBallPitchMotor = new Victor(2,6);
		bBallRotator = new Victor(2,7);
		bBallElevator = new Relay(2);
		bBallElevator2 = new Relay(1);
		airCompressor = new Compressor(10,3);
		rightStick = new Joystick(1); // create the joysticks
		leftStick = new Joystick(2);
		aimStick = new Joystick(3);
		buttons = new Joystick(4);
		

		//bBallCollectorSensor = new DigitalInput(2,3);
		bBallElevatorSensor = new DigitalInput(2,4);
		bBallBrushSensor = new DigitalInput(2,5);
		bBallElevatorTopLimit = new DigitalInput(1);
		bBallElevatorBottomLimit = new DigitalInput(2,7);
		beamTest = new DigitalInput(1,5);
		robotQueue = msgQCreate(100, 1024, MSG_Q_PRIORITY);		
		if (taskSpawn("tcpServer", 100, 0, 10000, 
				(FUNCPTR) tcpServer, 0,0,0,0,0,0,0,0,0,0) == ERROR) 
		{ 
			/* if taskSpawn fails, close fd and return to top of loop */ 

			perror ("taskSpawn"); 

		}

		//accel = new ADXL345_I2C(1);
		mI2c = (DigitalModule::GetInstance(1))->GetI2C(0x3A);
		mI2c->SetCompatibilityMode(true);
		mI2c->Write(0x2D, 0x08);
		//gyro = new Gyro(1);
		driverStationControl = DriverStation::GetInstance();

		dsLCD = DriverStationLCD::GetInstance();
		// init
		//encoderWheels12->Start();
		//encoderWheels34->Start();
		//encoderTurretRotation->Start();
		//random crap

		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Hello World");
		dsLCD->UpdateLCD();

		//Update the motors at least every 100ms.
		//armUpperLimit = new DigitalInput(1); // create the limit switch inputs
		//armLowerLimit = new DigitalInput(2);
		myRobot->SetExpiration(0.1);
		elevatorButtonState = false;
		elevatorButtonLastState = false;
		elevatorUp = false;
		elevatorDown = false;


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
		char msgBuf[1024];
		UINT8 val;
		printf("hi");
		//Victor armMotor(5); // create arm motor instance
		while (IsOperatorControl())
		{
			double x = RobotPosition(encoderWheels12,encoderWheels34,"x");
			myRobot->TankDrive(leftStick, rightStick); // drive with tank style
			bBallRotator->SetSpeed(aimStick->GetX());
			bBallPitchMotor->Set((aimStick->GetY())/10);
			bBallShooterTop->SetSpeed(driverStationControl->GetAnalogIn(1));
			bBallShooterBottom->SetSpeed(driverStationControl->GetAnalogIn(1));
			MotorSwitch(6, bBallShooterTop,.5);
			MotorSwitch(6, bBallShooterBottom,.5);
			MotorSwitch(7, bBallShooterTop,.8);
			MotorSwitch(7, bBallShooterBottom,.8);
			MotorSwitch(8, bBallShooterTop,.9);
			MotorSwitch(8, bBallShooterBottom,.9);
			MotorSwitch(9, bBallShooterTop,.99);
			MotorSwitch(9, bBallShooterBottom,.99);
			//printf("%d", bBallElevatorTopLimit->Get());
			//printf("%d \n", beamTest->Get());
			//printf("%04f \n", accel->GetAcceleration(ADXL345_I2C::kAxis_X));

			mI2c->Read(0x32, 1, &val);

			//printf("%d ", val);
			mI2c->Read(0x33, 1, &val);

			//printf("%d\n", val);
			memset(msgBuf, 0, sizeof(char) * 1024);
			if (msgQReceive(robotQueue, msgBuf, 1024, NO_WAIT) != ERROR) {
				printf("Got a message: %s", msgBuf);

				JSONValue *value = JSON::Parse(msgBuf);

				JSONObject root = value->AsObject();

				printf("Num = %d\n", (int)(root[L"num"]->AsNumber()));

			}
			Wait(0.01);
		}
	}
	void MotorSwitch(int button, Victor* motor, float speed)
	{
		if (buttons->GetRawButton(button))
		{
			motor->Set(speed);
		}
	}

	void ElevatorSystem()
	{
		if (buttons->GetRawButton(7) == true && elevatorButtonLastState == false)
		{
			elevatorUp = true;
		}
		elevatorButtonLastState = elevatorButtonState;
		if (elevatorDown)
		{
			// in case people are pushing the button too much
			elevatorUp = false;
			// run motor down
			if (false)   // bottom button pushed
			{
				elevatorDown = false;
			}
		}
		if (elevatorUp)
		{
			// start bottom el
			//bBallElevator->Set(Relay::Value::kForward);
			//bBallElevator->Set(Relay::Value::kOn);
			// start top el
			//bBallElevator2->Set(Relay::Value::kForward);
			//bBallElevator2->Set(Relay::Value::kOn);
			// if top limit clicked
			/*if (!bBallElevatorTopLimit->Get())
			{
				//stop bottom el
				elevatorUp = false;
				elevatorDown = true;
			}*/
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

	double RobotPosition(Encoder *encoder1, Encoder *encoder2, string what)
	{
		double Pi = atan(1)*4; //Pi Constant
		int Wr = 4; //Wheel Radius
		int D = 31; //Set equal to separation of wheels
		int Tr; //Set equal to ticks per rev
		encoder1->Start();
		encoder2->Start();

		int T1 = encoder1->Get();
		int T2 = encoder2->Get();

		double position[3];

		position[0] = (2*Pi)*(Wr/D)*(T1-T2)*(Pi/Tr);//Heading of robot in radian
		position[1] = (10*cos(position[0]))*(T1+T2)*(Pi/Tr);//X position of robot in inches
		position[2] = (10*sin(position[0]))*(T1+T2)*(Pi/Tr);//Y position of robot in inches

		if(what == "x" || what == "X")
		{
			return position[1];//return x position
		}
		if(what == "y" || what == "Y")
		{
			return position[2];//return y position
		}
		else
		{
			return position[0];//return heading in radian
		}

	}
};
