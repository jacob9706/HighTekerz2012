//		GREEN_LIGHT new  PWM(1,6);
//
//		bBallPitchMotor = new Victor(2,3);
//		bBallRotator = new Victor(2,4);
//		bBallShooterTop = new Victor(2,1);
//		bBallShooterBottom = new Victor(2,2);
//		
//		bBallElevatorBottom = new Relay(2,1,Relay::kBothDirections);
//		bBallElevatorTop = new Relay(2,2,Relay::kBothDirections);
//		bBallCollector = new Victor(2,5);
//		rampServo = new Servo(2,6);
//
////		mockRelay = new Relay(2,5,Relay::kBothDirections);
//
//		// Air
//		airCompressor = new Compressor(1,10,2,3);
//		shooterArm = new Solenoid(1);
//		rampArm = new Solenoid(2);
//		
//
//		encoderWheelsLeft = new Encoder(1,1,1,2,false,Encoder::k1X);
//		encoderWheelsRight = new Encoder(1,3,1,4,true,Encoder::k1X);
//		encoderTurretRotation = new Encoder(2,3,2,4);
//		bBallAngleSensor = new DigitalInput(2,11);
//		encoderShooterTop = new Encoder(2,5,2,6,true);
//		encoderShooterBottom = new Encoder(2,7,2,8,true);
//		
//		bBallElevatorTopLimit = new DigitalInput(2,2);
//		bBallElevatorBottomLimit = new DigitalInput(2,1);
//
////		accel = new ADXL345_I2C(1);
////		gyro = new Gyro(1);
//		
//		// Driver Station //
//		
//		xboxDrive = new XboxController(1);
//		xboxShoot = new XboxController(2);
//		
//		robotElevator = new ElevatorSystem(bBallElevatorBottom, 
//				bBallElevatorTop, 
//				bBallElevatorBottomLimit, 
//				bBallElevatorTopLimit);
//
//		myRobot = new Drivetrain(3, 4, 1, 2, encoderWheelsLeft, encoderWheelsRight);	// create robot drive base
//		
