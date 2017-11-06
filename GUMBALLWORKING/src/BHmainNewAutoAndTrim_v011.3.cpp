#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "WPILib.h"

#include "rrPixy.hpp"
#include "rrLidar.hpp"
#include "neScaler.h"
#include "rrPID.h"

#include <ctrlib/CANTalon.h>
#include "AHRS.h"
// somethign
#include <PowerDistributionPanel.h>

class Robot: public frc::IterativeRobot {
	CANTalon frontLeftChannel;
	CANTalon rearLeftChannel;
	CANTalon frontRightChannel;
	CANTalon rearRightChannel;
	RobotDrive robotDrive;	// robot drive systemf
	CANTalon shooterMotor;
	CANTalon gumballMotor;
	CANTalon winchMotor;
	CANTalon intakeMotor;
	CANTalon angleMotor;
	Servo *lidarServo = new Servo(1);

	rrPixy myPixy2;
	rrPixy myPixy;
	rrLidar myLidar;
	neScaler myScaler;

	rrPID gyroPID;
	rrPID pixyDistPID;
	rrPID pixyAnglePID;
	rrPID lidarPID;
	rrPID pixy2AnglePID;


	Joystick stick;			// only joystick
	Joystick stick2;
	AHRS *ahrs;
public:

	Robot():

		frontLeftChannel(26), //Cam
		rearLeftChannel(19),
		frontRightChannel(24),
		rearRightChannel(25),
//			frontLeftChannel(29), //Mittens
//			rearLeftChannel(27),
//			frontRightChannel(30),
//			rearRightChannel(28),


		robotDrive(frontLeftChannel, rearLeftChannel,
				frontRightChannel, rearRightChannel),
		shooterMotor(17),
		gumballMotor(15), //15 on Cam, 28 on Mittens
		winchMotor(16),
		intakeMotor(18),
		angleMotor(20),
		stick(0),
		stick2(1)
{
	robotDrive.SetExpiration(0.1);
	robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);	// invert the left side motors
	robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);	// you may need to change or remove this to match your robot
	ahrs = new AHRS(SerialPort::kMXP); /* Alternatives:  SPI::kMXP, I2C::kMXP or SerialPort::kUSB */
	myPixy.init(0x54); // Shooter camera
	myPixy2.init(0x55); //Gear camera
	myLidar.init();
	myScaler.init(2.0, 1.0, 0.05);
	frontLeftChannel.SetVoltageRampRate(240);
	frontRightChannel.SetVoltageRampRate(240);
	rearLeftChannel.SetVoltageRampRate(240);
	rearRightChannel.SetVoltageRampRate(240);
	shooterMotor.SetVoltageRampRate(24);
	intakeMotor.SetVoltageRampRate(24);
	intakeMotor.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
	shooterMotor.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
}

void RobotInit() {
	timer = new Timer();
	rightVsLeft.AddDefault(leftSide, leftSide);
	rightVsLeft.AddObject(rightSide, rightSide);
	rightVsLeft.AddObject(centerSide, centerSide);
	frc::SmartDashboard::PutData("Right Vs Left", &rightVsLeft);

	autoFunction.AddDefault(gearDropOnly, gearDropOnly);
	autoFunction.AddObject(shootOnly, shootOnly);
	autoFunction.AddObject(dropAndShoot, dropAndShoot);
	frc::SmartDashboard::PutData("Auto Modes", &autoFunction);

	redVsBlue.AddDefault(redSide, redSide);
	redVsBlue.AddObject(blueSide, blueSide);
	frc::SmartDashboard::PutData("Red Vs Blue", &redVsBlue);

	SmartDashboard::PutNumber("Target kp", 0.03);
	SmartDashboard::PutNumber("Target ki", 0.001);
	SmartDashboard::PutNumber("Target kid", .003);
	SmartDashboard::PutNumber("Target kpd", .03);
	SmartDashboard::PutNumber("Target kp Distance Pixy", 0.03);
	SmartDashboard::PutNumber("Target ki Distance Pixy", 0.001);
	SmartDashboard::PutNumber("TARGET PIXEL Bottom Y", 112);
	SmartDashboard::PutNumber("TARGET offset Shooter", -11.5);
	SmartDashboard::PutNumber("TARGET offset Gear", -25);
	SmartDashboard::PutNumber("Lidar Servo Angle", 90);

	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	Timer *timer;
	Timer *newAutoTimer;
	double MyHeading;
	double shooterAimingTick = 0;

	void AutonomousInit() override {
		autoSelectedFunction = autoFunction.GetSelected();
		autoSelectedSide = rightVsLeft.GetSelected();
		std::cout << "Auto selected: " << autoSelectedFunction << std::endl;
		std::cout << "Side selected: " << autoSelectedSide << std::endl;
		autoStep = 0;
		newAutoStep = 0;
		newAutoTimer = new Timer();
		newAutoTimer->Start();
		timer = new Timer();
		timer->Start();
		ahrs->ZeroYaw();

		lidarPID.init(1, 0.023, 0.0001);
		pixyDistPID.init(2, 0.007, 0.0001);
		pixyDistPID.outputLim = .3;
		pixyDistPID.intLim = 1000;
		pixyAnglePID.init(3, 0.02, 0.001);
		pixyAnglePID.intLim = 100;
		gyroPID.init(4, 0.03, 0.0007);
		pixy2AnglePID.init(5, 0.002, 0.0);

		pixy2AnglePID.intLim = 100;

		robotDrive.MecanumDrive_Cartesian(0,0,0);
	}
	int newAutoStep = 0;
	int autoStep=0;
	double servoTime=0;
	double driveTime = 0;
	double conditionTime = 0;
	float targetDistance = 0;
	int xOrY;	 //x = 0, y = 1
	float targetBottomY = 112;
	float targetOffset = -11.5;
	float targetGearOffset = -15;
	float lidarServoAngle = 90;

	float kP_angleShooter = .02;
	float kI_angleShooter = .001;
	float kP_distShooter = .007;
	float kI_distShooter = .0001;
	float kP_Gear = .02; // 4/24 testing = .01
	float kI_Gear = 0.001; // 4/24 testing = .001
	int autoCoef_X = 1;
	int autoCoef_Y = 1;
	int autoCoef_Z = 1;

//	float gumballSpeed(float stickSpeed){
//		float outputSpeed = stickSpeed * 1;
//		return outputSpeed;
//		SmartDashboard::PutNumber("Gumball Variable Speed", outputSpeed);
//	}


	void newShootOnlyLeft(){
		switch(newAutoStep){
		case 0:{
			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			}
			else if(true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			ahrs->ZeroYaw();
			ahrs->SetAngleAdjustment(90);
			increment_since_last_find = 11;
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);
			break;
		}
		case 1:{
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if (newAutoTimer->Get() > 1){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian((autoCoef_X)*(-.3), 0, 0); // + on gumball, slide right
			break;
		}
		case 2:{
			if (newAutoTimer->Get() > 5){ //stop everything
				newAutoStep=99;
			}
			else if (true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian(0,0,0);
			break;
		}
		case 3:{
			autoGoToAngle(-30); // turn left 30*
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if (pixyAnglePID.errorSignal < 2){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			break;
		}
		case 4:{
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if (true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian(0,0,0);
			break;
		}
		case 5:{
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if(fabs(increment_since_last_find)<2){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian((autoCoef_X)*(-.3), 0, 0); // + on gumball, slide right
			break;
		}
		case 6:{
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if (true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian(0,0,0);
			break;
		}
		case 7:{
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if (shooterAimingTick > 500){
				newAutoStep++;
				newAutoTimer->Reset();
				shooterAimingTick = 0;
			}

			if (abs(pixyAnglePID.errorSignal) >8){
				robotDrive.MecanumDrive_Cartesian(0.0, 0.0, (autoCoef_Z)*(-pixyAnglePID.motorOutput()));
			}
			else {
				robotDrive.MecanumDrive_Cartesian(0.0, (autoCoef_Y)*(pixyDistPID.motorOutput()), (autoCoef_Z)*(-pixyAnglePID.motorOutput())); // y - on gumball
			}

			if (fabs(pixyAnglePID.errorSignal) <2){
				shooterAimingTick++;
			}
			else {
				shooterAimingTick = 0;
			}
			pixyAnglePID.debug();
			pixyDistPID.debug();
			break;
		}
		case 8:{
			if(newAutoTimer->Get() > 10){
				newAutoStep = 99;
			}
			else if (newAutoTimer->Get()> 9){
				newAutoStep++;
			}
			shooterMotor.Set(-1);
			gumballBool = true;
			break;
		}
		default:{
			robotDrive.MecanumDrive_Cartesian(0,0,0);
			shooterMotor.Set(0);
			gumballBool = false;
			break;
		}
		}
	}

	void newShootOnlyRight(){
		switch(newAutoStep){
		case 0:{
			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			}
			else if(true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			ahrs->ZeroYaw();
			ahrs->SetAngleAdjustment(90);
			increment_since_last_find = 11;
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);
			break;
		}
		case 1:{
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if (newAutoTimer->Get() > 1){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian((autoCoef_X)*.3, 0, 0); // - on gumball, slide left
			break;
		}
		case 2:{//stop everything
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if (true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian(0,0,0);
			break;
		}
		case 3:{
			autoGoToAngle(30); // turn right 30*
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if (pixyAnglePID.errorSignal < 2){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			break;
		}
		case 4:{
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if (true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian(0,0,0);
			break;
		}
		case 5:{
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if(fabs(increment_since_last_find)<2){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian((autoCoef_X)*-.3, 0, 0); // - on gumball?
			break;
		}
		case 6:{
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if (true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian(0,0,0);
			break;
		}
		case 7:{//track to target
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if (shooterAimingTick > 500){
				newAutoStep++;
				newAutoTimer->Reset();
				shooterAimingTick = 0;
			}

			if (abs(pixyAnglePID.errorSignal) >8){
				robotDrive.MecanumDrive_Cartesian(0.0, 0.0, (autoCoef_Z)*-pixyAnglePID.motorOutput());
			}
			else {
				robotDrive.MecanumDrive_Cartesian(0.0, (autoCoef_Y)* pixyDistPID.motorOutput(), (autoCoef_Z)* -pixyAnglePID.motorOutput()); // y - on gumball
			}

			if (fabs(pixyAnglePID.errorSignal) <2){
				shooterAimingTick++;
			}
			else {
				shooterAimingTick = 0;
			}
			pixyAnglePID.debug();
			pixyDistPID.debug();
			break;
		}
		case 8:{
			if(newAutoTimer->Get() > 10){
				newAutoStep = 99;
			}
			else if (newAutoTimer->Get()> 9){
				newAutoStep++;
			}
			shooterMotor.Set(-1);
			gumballBool = true;
			break;
		}
		default:{
			robotDrive.MecanumDrive_Cartesian(0,0,0);
			shooterMotor.Set(0);
			gumballBool = false;
			break;
		}
		}
	}

	void newGearDropOnlyLeft(){
		switch(newAutoStep){
		case 0:{  // Initialization
			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			}
			else if(true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			ahrs->ZeroYaw();
			ahrs->SetAngleAdjustment(179.999);
			p2increment_since_last_find = 11;
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);
			break;
		}
		case 1:{
			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			}
			else if(p2increment_since_last_find < 10){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian(0, (autoCoef_Y)* -0.3, 0);

			break;
		}
		case 2:{
			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			}
			else if(true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);

			break;
		}
		case 3:{
			autoGoToAngle(-30);

			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			}
			else if(fabs(pixyAnglePID.errorSignal)<2){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			break;
		}
		case 4:{
			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			}
			else if(true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);

			break;
				}
		case 5:{
			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			} else if(m_pdp.GetCurrent(3) > 15){
				newAutoStep++;
				newAutoTimer->Reset();
			}

			if (fabs(pixy2AnglePID.errorSignal) >5){
				robotDrive.MecanumDrive_Cartesian(0.0, 0, (autoCoef_Z)* -pixy2AnglePID.motorOutput());
			}
			else {
				robotDrive.MecanumDrive_Cartesian((autoCoef_X)*.5, 0.0, 0.0);
			}
			break;
		}


		default:{
			robotDrive.MecanumDrive_Cartesian(0,0,0);
			shooterMotor.Set(0);
			gumballBool = false;
			break;
		}
		}
	}

	void newGearDropOnlyRight(){

		switch(newAutoStep){
		case 0:{
			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			}
			else if(true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			ahrs->ZeroYaw();
			p2increment_since_last_find = 11;
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);
			break;
		}
		case 1:{
			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			}
			else if(p2increment_since_last_find < 10){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian(0, (autoCoef_Y)*0.2, 0);

			break;
		}
		case 2:{
			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			}
			else if(true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);

			break;
		}
		case 3:{
			autoGoToAngle(30);

			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			}
			else if(fabs(pixyAnglePID.errorSignal)<2){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			break;
		}
		case 4:{
			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			}
			else if(true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);

			break;
		}
		case 5:{
			if (newAutoTimer->Get() > 10){
				newAutoStep=99;
			}
//			else if(m_pdp.GetCurrent(3) > 15){
//				newAutoStep++;
//				newAutoTimer->Reset();
//			}

			if (fabs(pixy2AnglePID.errorSignal) >5){
				robotDrive.MecanumDrive_Cartesian(0.0, 0.0, (autoCoef_Z)*-pixy2AnglePID.motorOutput());
			}
			else {
				robotDrive.MecanumDrive_Cartesian((autoCoef_X)*.5, 0.0, 0.0);
			}
			break;
		}
		default:{
			robotDrive.MecanumDrive_Cartesian(0,0,0);
			shooterMotor.Set(0);
			gumballBool = false;
			break;
			}
		}
	}

	void newGearDropOnlyCenter(){
		switch(newAutoStep){
		case 0:{
			if (newAutoTimer->Get() > 10) {
				newAutoStep=99;
			}
			else if(true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			ahrs->ZeroYaw();
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);
			break;
		}

		case 1:{
			if (newAutoTimer->Get() > 10) {
				newAutoStep=99;
			}
			else if(true){
				newAutoStep++;
				newAutoTimer->Reset();
			}

			robotDrive.MecanumDrive_Cartesian(0, 0, 0);
			break;
		}
		case 2:{
			if (newAutoTimer->Get() > 15) {
				newAutoStep=99;
			}
			else if (false){
				newAutoStep++;
				newAutoTimer->Reset();
			}

			if (fabs(pixy2AnglePID.errorSignal) >5){
				robotDrive.MecanumDrive_Cartesian(0.0,0.0, (autoCoef_Z)*pixy2AnglePID.motorOutput());
			}
			else {
				robotDrive.MecanumDrive_Cartesian((autoCoef_X)*.5, 0.0, 0.0);
			}
			break;
		}
		default:{
			shooterMotor.Set(0);
			gumballBool = false;
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);
			break;
			}

		}
	}


	void newShootAndDropCenter(){
		switch(newAutoStep){
		case 0:{
			if (newAutoTimer->Get() > 10) {
				newAutoStep=99;
			}
			else if(true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			ahrs->ZeroYaw();
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);
			break;
		}

		case 1:{
			if (newAutoTimer->Get() > 10) {
				newAutoStep=99;
			}
			else if(true){
				newAutoStep++;
				newAutoTimer->Reset();
			}

			robotDrive.MecanumDrive_Cartesian(0, 0, 0);
			break;
		}
		case 2:{
			if (newAutoTimer->Get() > 15) { //change time
				newAutoStep=99;
			}
			else if (false){
				newAutoStep++;
				newAutoTimer->Reset();
			}

			if (fabs(pixy2AnglePID.errorSignal) >5){
				robotDrive.MecanumDrive_Cartesian(0.0,0.0, (autoCoef_Z)*pixy2AnglePID.motorOutput());
			}
			else {
				robotDrive.MecanumDrive_Cartesian((autoCoef_X)*.5, 0.0, 0.0);
			}
			break;
		}
		case 3:{
					if (newAutoTimer->Get() > 5){
						newAutoStep=99;
					}
					else if (newAutoTimer->Get() > 1){
						newAutoStep++;
						newAutoTimer->Reset();
					}
					robotDrive.MecanumDrive_Cartesian((autoCoef_X)*(-.3), 0, 0); // + on gumball, slide right
					break;
				}

		case 4:{
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if (true){
				newAutoStep++;
				newAutoTimer->Reset();
			}
			robotDrive.MecanumDrive_Cartesian(0,0,0);
			break;
		}
		case 5: {

			autoGoToAngle(15); // turn right 15*
			if (newAutoTimer->Get() > 5){
				newAutoStep=99;
			}
			else if (pixyAnglePID.errorSignal < 2){
				newAutoStep++;
				newAutoTimer->Reset();
			}
					break;
		}
		default:{
			shooterMotor.Set(0);
			gumballBool = false;
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);
			break;
			}

		}
	}



	void AutonomousPeriodic() {
		double AutoTime = timer->Get();
//		double changeTime = AutoTime - lastTime;
		numBlocks2 = myPixy2.rrPixyGetBlocks(100); // was getBlocks(100);
		numBlocks = myPixy.rrPixyGetBlocks(100); // was getBlocks(100);
		turnAngle();
		turnAngle2();
		pixyTeleDebugs();
		SmartDashboard::PutNumber("Condition Time", conditionTime);
		SmartDashboard::PutNumber("AutoTime", AutoTime);
		SmartDashboard::PutNumber("AutoStep", autoStep);
//		SmartDashboard::PutNumber("Auto Lidar Measurement", myLidar.GetLidarDistance());
		SmartDashboard::PutNumber("New autoStep", newAutoStep);
		SmartDashboard::PutNumber("Auto p2Increment", p2increment_since_last_find);
			if (autoSelectedSide == leftSide) {
				if (autoSelectedFunction == gearDropOnly){
					newGearDropOnlyLeft();
				}else if(autoSelectedFunction == shootOnly){
					newShootOnlyLeft();
				}else if(autoSelectedFunction == dropAndShoot){
//					gearDropShootLeft(changeTime);
				}
			}else if (autoSelectedSide == rightSide){
				if (autoSelectedFunction == gearDropOnly){
					newGearDropOnlyRight();
				}else if(autoSelectedFunction == shootOnly){
					newShootOnlyRight();
				}else if(autoSelectedFunction == dropAndShoot){
//					gearDropShootRight(changeTime);
				}
			}else if (autoSelectedSide == centerSide){
				newGearDropOnlyCenter();
			}



//		lastTime = AutoTime;
	} //end of automous_periodic

	uint16_t numBlocks = 0;
	uint16_t numBlocks2 = 0;

	double newHeading =0;
	double newHeading2 = 0;
	int increment_since_last_good_verticle = 0;
	int increment_since_last_find = 0;
	int increment_since_last_find2 = 0;
	int p2increment_since_last_find = 0;

	uint16_t lastX;
	uint16_t lastY;
	uint16_t lastX2;
	uint16_t lastY2;
	uint16_t pixy2lastX;
	uint16_t pixy2lastY;
	uint16_t pixy2lastX2;
	uint16_t pixy2lastY2;
	uint16_t pixy2LeftestX;
	const float shootingLenseFOV = 50.0;
	const float gearLenseFOV = 75.0;

	void autoGoToDistance(double changeInTime, double targetDistance, int xOrY){ //x = 0, y = 1
		gyroAngle(0);
		closeDistance(targetDistance);
		float currentDistance = myLidar.GetLidarDistance();
		if (xOrY == 1)robotDrive.MecanumDrive_Cartesian(0, -lidarPID.motorOutput(), -gyroPID.controlOutput(0, MyHeading));
		else if (xOrY == 0)robotDrive.MecanumDrive_Cartesian(-lidarPID.motorOutput(), 0, -gyroPID.controlOutput(0, MyHeading));
		if (currentDistance < targetDistance + 4 && currentDistance > targetDistance - 4){
			conditionTime += changeInTime;
		}
		else{
			conditionTime = 0;
		}
		if (conditionTime > .5){
			autoStep++;
		}
	}
	void autoGoToAngle(double angleToGo){
		gyroAngle(angleToGo);
		robotDrive.MecanumDrive_Cartesian(0,0,-pixyAnglePID.motorOutput());
		SmartDashboard::PutNumber("PID:3 : rrError Signal", pixyAnglePID.errorSignal);
	}
	void autoGoToAngleForward(double changeInTime, double angleToGo, double Speed){
		gyroAngle(angleToGo);
		robotDrive.MecanumDrive_Cartesian(Speed,0,-gyroPID.motorOutput());
//		SmartDashboard::PutNumber("PID:3 : rrError Signal", gyroPID.errorSignal);
//		if (fabs(gyroPID.errorSignal) < 8){
//			autoStep++;
//		}
	}
	void autoGoToDrop(){
		numBlocks = myPixy.rrPixyGetBlocks(2); // was getBlocks(100);
		numBlocks2 = myPixy2.rrPixyGetBlocks(100); // was getBlocks(100);
		turnAngle2();
		pixy2AnglePID.debug();
		SmartDashboard::PutNumber("auto pixy age", increment_since_last_find);
		SmartDashboard::PutNumber("Pixy auto motor output", pixy2AnglePID.motorOutput());
		if (abs(pixy2AnglePID.errorSignal) >=5){
			robotDrive.MecanumDrive_Cartesian(0.0, 0.0, -pixy2AnglePID.motorOutput());
		}
		else {
			robotDrive.MecanumDrive_Cartesian(.5, 0.0, 0.0);
		}

;
	}
	void autoGoToShoot(double changeInTime, double distance){
		increment_since_last_find++;
		pixyCloseDistance(distance);
		turnAngle();
		SmartDashboard::PutNumber("auto pixy age", increment_since_last_find);

		if (abs(pixyAnglePID.errorSignal) >50){
			robotDrive.MecanumDrive_Cartesian(0.0, 0.0, pixyAnglePID.motorOutput());
		}
		else {
			robotDrive.MecanumDrive_Cartesian(0.0, -pixyDistPID.motorOutput(), pixyAnglePID.motorOutput());
		}
		if((abs(pixyAnglePID.errorSignal) <3) && (abs(pixyDistPID.errorSignal) < 7)){
			conditionTime += changeInTime;
		}
		else{
			conditionTime = 0;
		}
		if (conditionTime > .5){
			autoStep++;
		}
	}
	void autoShootingRoutine(){
		gumballBool = true;
		shooterMotor.Set(-1);
		unjamInput();
	}
	bool trackTwo = false;
	void gyroAngle(double headingToGo){
		pixyAnglePID.debug();
		pixyAnglePID.controlOutput(headingToGo, ahrs->GetYaw());
	}
	void turnAngle()
	{
		pixyAnglePID.debug();
		float turnAngle1Heading = ahrs->GetYaw();
		MyHeading = ahrs->GetYaw();
		if((numBlocks ==1) && (myPixy.rrReturnBlock0().signature==1)){ // was block[0]
			lastX = myPixy.rrReturnBlock0().x;// was block[0]
			lastY = myPixy.rrReturnBlock0().y;// was block[0]
			SmartDashboard::PutNumber("Block @ X = ",lastX);
			SmartDashboard::PutNumber("Block @ Y = ",lastY);
			double changeinHeading = (160.0 - lastX)/320.0*shootingLenseFOV;
			newHeading = turnAngle1Heading - changeinHeading;
			SmartDashboard::PutNumber("change in heading pixy1, 1 block", changeinHeading);
			increment_since_last_find = 0;
		}
		else if((numBlocks ==2 ) && (myPixy.rrReturnBlock0().signature == 1 and myPixy.rrReturnBlock1().signature==1))
		{
			//Block 1
			lastX = myPixy.rrReturnBlock0().x;// was block[0]
			lastY = myPixy.rrReturnBlock0().y;// was block[0]
			//Show on Smart Dashboard
			SmartDashboard::PutNumber("Block @ X = ",lastX);
			SmartDashboard::PutNumber("Block @ Y = ",lastY);
			increment_since_last_find = 0;
			//Block 2
			lastX2 = myPixy.rrReturnBlock1().x;// was block[1]
			lastY2 = myPixy.rrReturnBlock1().y;// was block[1]
			//Show on Smart Dashboard
			SmartDashboard::PutNumber("Block2 @ X = ",lastX2);
			SmartDashboard::PutNumber("Block2 @ Y = ",lastY2);
			increment_since_last_find2 = 0;
			double avLastX = (lastX + lastX2)/2;
			double changeinHeading = (160.0 - avLastX)/320.0*shootingLenseFOV;
			SmartDashboard::PutNumber("change in heading pixy1, 2 blocks", changeinHeading);
			newHeading = turnAngle1Heading - changeinHeading;
		}
		pixyAnglePID.controlOutput(newHeading + targetOffset, turnAngle1Heading);


	}
	void turnAngle2()
	{
		float turnAngle2Heading = ahrs->GetYaw();
		pixy2AnglePID.debug();
		if((numBlocks2 ==2 ) && (myPixy2.rrReturnBlock0().signature == 1 and myPixy2.rrReturnBlock1().signature==1))
		{
			//Block 1
			pixy2lastX = myPixy2.rrReturnBlock0().x;// was block[0]
			pixy2lastY = myPixy2.rrReturnBlock0().y;// was block[0]
			//Show on Smart Dashboard
			SmartDashboard::PutNumber("Pixy2 Block @ X = ",pixy2lastX);
			SmartDashboard::PutNumber("Pixy2 Block @ Y = ",pixy2lastY);
			p2increment_since_last_find = 0;
			//Block 2
			pixy2lastX2 = myPixy2.rrReturnBlock1().x;// was block[1]
			pixy2lastY2 = myPixy2.rrReturnBlock1().y;// was block[1]
			//Show on Smart Dashboard
			SmartDashboard::PutNumber("Pixy2 Block2 @ X = ",pixy2lastX2);
			SmartDashboard::PutNumber("Pixy2 Block2 @ Y = ",pixy2lastY2);
			if (pixy2lastX < pixy2lastX2) pixy2LeftestX = pixy2lastX;
			else pixy2LeftestX = pixy2lastX2;
			p2increment_since_last_find=0;
			double changeinHeading = (160.0 - pixy2LeftestX)/320.0*gearLenseFOV;
			newHeading2 = turnAngle2Heading - changeinHeading;
		}else if(numBlocks2 == 1 && myPixy2.rrReturnBlock0().signature == 1){
			pixy2LeftestX = myPixy2.rrReturnBlock0().x;// was block[0]
			p2increment_since_last_find=0;
			double changeinHeading = (160.0 - pixy2LeftestX)/320.0*gearLenseFOV;
			newHeading2 = turnAngle2Heading - changeinHeading;
			SmartDashboard::PutNumber("Pixy2 Block @ X = ",pixy2LeftestX);
		}
		SmartDashboard::PutNumber("Pixy2 Leftest block", pixy2LeftestX);


		pixy2AnglePID.controlOutput(newHeading2+targetGearOffset, turnAngle2Heading);
		SmartDashboard::PutNumber("newHeading2 + targetGearOffset",newHeading2+targetGearOffset);
	}
	int lastLidarDistance = 0;
	float lastBottomY;
	float bottomY;
	float kpDistPixy = 0;
	float kiDistPixy = 0;
	float intergrationTotalPixyDistance = 0;
	float errorSignalPixyDistance = 0;
	float lastGoodbY = 0;
	float BottomY_Threshold = 20;
	uint16_t pixyLastY1;
	uint16_t pixyLastY2;
	void pixyCloseDistance(double targetDistance){
		float blockHeight;
		float bestGuessThisTime_BottomY;
		pixyLastY1 = myPixy.rrReturnBlock0().y;// was block[0]
		pixyLastY2 = myPixy.rrReturnBlock1().y;

		if((numBlocks == 1) && (myPixy.rrReturnBlock0().signature==1)){ // was block[0]

			blockHeight = myPixy.rrReturnBlock0().height;
			bestGuessThisTime_BottomY = (blockHeight/2) + pixyLastY1;

			if (abs(bestGuessThisTime_BottomY - lastGoodbY) < BottomY_Threshold ) {
				lastGoodbY = bestGuessThisTime_BottomY;
				increment_since_last_good_verticle = 0;
			} else if (increment_since_last_good_verticle > 30) {
				lastGoodbY = bestGuessThisTime_BottomY;
//				increment_since_last_good_verticle = 0;
			}
		}
		else if((numBlocks > 1) && (myPixy.rrReturnBlock0().signature==1) && (myPixy.rrReturnBlock1().signature ==1)){  // Added 2nd .signature
			if (pixyLastY1 >= pixyLastY2){
				blockHeight = myPixy.rrReturnBlock0().height;
				bestGuessThisTime_BottomY = (blockHeight/2) + pixyLastY1;
			}
			else if (pixyLastY1 < pixyLastY2) {
				blockHeight = myPixy.rrReturnBlock1().y;
				bestGuessThisTime_BottomY = (blockHeight/2) + pixyLastY2;
			} else {
				bestGuessThisTime_BottomY  = 9999;
			}
			if (abs(bestGuessThisTime_BottomY - lastGoodbY) < BottomY_Threshold ) {
				lastGoodbY = bestGuessThisTime_BottomY;
				increment_since_last_good_verticle = 0;
			} else if (increment_since_last_good_verticle > 30) {
				lastGoodbY = bestGuessThisTime_BottomY;
//				increment_since_last_good_verticle = 0;
			}
		}

		SmartDashboard::PutNumber("Bottom Y = ",lastGoodbY);
		pixyDistPID.debug();
		pixyDistPID.controlOutput(targetDistance, lastGoodbY);
	}

	void closeDistance(float distance)
	{
		lidarPID.debug();
		lidarPID.controlOutput(distance, lastLidarDistance);
	}
	void TeleopInit() {
		ahrs->ZeroYaw();
		myLidar.initTeleop();
		timer->Start();
		lidarPID.init(1, 0.023, 0.0001);
		pixyDistPID.init(2, 0.007, 0.0001);
		pixyDistPID.outputLim = .3;
		pixyDistPID.intLim = 1000;
		pixyAnglePID.init(3, 0.02, 0.001);
		pixyAnglePID.intLim = 100;
		gyroPID.init(4, 0.03, 0.0007);
		pixy2AnglePID.init(5, 0.002, 0.0);
		pixy2AnglePID.intLim = 100;


	}
	int trimStep = 1;
	bool firstTrimPress = false;
//	void pixyDistanceTrim(){
//		switch (trimStep){
//		case 1:
//			if (stick.GetPOVCount() == 1 && firstTrimPress == false){
//				firstTrimPress = true;
//				targetBottomY -= 1;		//trim closer, press "up" on hat
//				trimStep = 2;
//			}
//			else if (stick.GetPOVCount() == 2 && firstTrimPress == false){
//				firstTrimPress = true;
//				targetBottomY += 1;		//trim further, press "down" on hat
//				trimStep = 2;
//			}
//			break;
//		case 2:
//			if (stick.GetPOVCount() == 0 && firstTrimPress == true){ //wait for hat to be unpressed, reset state
//				firstTrimPress = false;
//				trimStep = 1;
//			}
//			break;
//
//		default: //in case bad things happen
////			firstTrimPress = false;
//			break;
//
//		}
//		SmartDashboard::PutNumber("Trim State", trimStep);
//	}
	void twitch(){
		if (stick.GetPOV() == 0){//up
			robotDrive.MecanumDrive_Cartesian(0, .5, 0);
		}
		else if (stick.GetPOV() == 180){ //down
			robotDrive.MecanumDrive_Cartesian(0, -.5, 0);
		}
		else if (stick.GetPOV() == 90){ //right
			robotDrive.MecanumDrive_Cartesian(.5, 0, 0);
		}
		else if (stick.GetPOV() == 270){ //left
			robotDrive.MecanumDrive_Cartesian(-.5, 0, 0);
		}
		else {robotDrive.MecanumDrive_Cartesian(0,0,0);}
	}
	bool ModeAutoAim = false;
	double jamTimeStart = -1000;
	void unjamInput() {
		double jamTime = timer->Get() - jamTimeStart;
		double backwardsTime = .4;
//		double forwardsTime = backwardsTime + .3;
		if (jamTime < backwardsTime){
			gumballMotor.Set(-1);
		}
//		else if (jamTime < forwardsTime) {
//			gumballMotor.Set(-0.5);
//		}
		else if (m_pdp.GetCurrent(7) >= 8) {
			jamTimeStart = timer->Get();
		}else if (gumballBool)gumballMotor.Set(1);
		 else if (gumballBool == false)gumballMotor.Set(0);
	}

	void teleopStick1() {
		pixyCloseDistance(targetBottomY);
		if (stick.GetRawButton(1) == true){
			ahrs->ZeroYaw();
		}
		else if (stick.GetRawButton(2) == true){
			robotDrive.MecanumDrive_Cartesian(-myScaler.scaleOutput(stick.GetX()),
					-myScaler.scaleOutput(stick.GetY()),
					-myScaler.scaleOutput(stick.GetZ()));
		}
//		else if (stick.GetRawButton(3) == true){
//			robotDrive.MecanumDrive_Cartesian(0.0, 0.0, -pixyAnglePID.motorOutput());
//		}
//		else if (stick.GetRawButton(4) == true){
//			robotDrive.MecanumDrive_Cartesian(0.0, -pixyDistPID.motorOutput(), 0.0);

	//			gyroAngle(0);
	//			robotDrive.MecanumDrive_Cartesian(0,0,-gyroPID.motorOutput());
//		}
		else if (stick.GetRawButton(3) == true){ //align to shoot
			if (abs(pixyAnglePID.errorSignal) >8){
				robotDrive.MecanumDrive_Cartesian(0.0, 0.0, -pixyAnglePID.motorOutput());
			}
			else {
				robotDrive.MecanumDrive_Cartesian(0.0, -pixyDistPID.motorOutput(), -pixyAnglePID.motorOutput()); //y positive for mittens
			}
		}
		else if (stick.GetRawButton(4) == true){ //align to gear drop
			if (abs(pixy2AnglePID.errorSignal) >5){
				robotDrive.MecanumDrive_Cartesian(0.0, 0.0, -pixy2AnglePID.motorOutput());
			}
			else {
				robotDrive.MecanumDrive_Cartesian(0.5, 0.0, 0.0);
			}
		}
		else if(stick.GetRawButton(11) == true){
//			myLidar.i2cReset();
			myPixy.i2cReset();
		}
		else if(stick.GetPOV() != -1){
			twitch();
		}
		else {
			ModeAutoAim = false;
			robotDrive.MecanumDrive_Cartesian(-myScaler.scaleOutput(stick.GetX()),
					-myScaler.scaleOutput(stick.GetY()),
					-myScaler.scaleOutput(stick.GetZ()),
					ahrs->GetYaw());
			}
	}
	void teleopStick2(){
		if (stick2.GetRawButton(1)== true){ // Xbox button A
			intakeMotor.Set(-1);

		}
		else if (stick2.GetRawButton(3)== true){ //Xbox button X
			shooterMotor.Set(-1);
			gumballBool = true;
		}
		else if (stick2.GetRawButton(2)== true){ // Xbox button B
			intakeMotor.Set(1);
		}
//		if(stick2.GetRawButton(6)== true){
//			gumballMotor.Set(1);
//		}
		else {
			winchMotor.Set(fabs(myScaler.scaleOutput(stick2.GetRawAxis(5))));
			gumballBool = false;
			intakeMotor.Set(0);
			shooterMotor.Set(0);
		}
	}
	void pixyTeleDebugs(){
		increment_since_last_find++;
		increment_since_last_good_verticle++;
		increment_since_last_find2++;
		p2increment_since_last_find++;
		targetBottomY = SmartDashboard::GetNumber("TARGET PIXEL Bottom Y", 112);
		targetOffset = SmartDashboard::GetNumber("TARGET offset Shooter", -11.5);
		targetGearOffset = SmartDashboard::GetNumber("TARGET offset Gear", -15);
		lidarServoAngle = SmartDashboard::GetNumber("Lidar Servo Angle", 90);
		kP_angleShooter = SmartDashboard::GetNumber("kP_angleShooter", kP_angleShooter);
		kI_angleShooter = SmartDashboard::GetNumber("kI_angleShooter", kI_angleShooter);
		kP_distShooter = SmartDashboard::GetNumber("kP_distShooter", kP_distShooter);
		kI_distShooter = SmartDashboard::GetNumber("kI_distShooter", kI_distShooter);
		kP_Gear = SmartDashboard::GetNumber("kP_Gear", kP_Gear);
		kI_Gear = SmartDashboard::GetNumber("kI_Gear", kI_Gear);

		autoCoef_X = SmartDashboard::GetNumber("autoCoef_X", autoCoef_X);
		autoCoef_Y = SmartDashboard::GetNumber("autoCoef_Y", autoCoef_Y);
		autoCoef_Z = SmartDashboard::GetNumber("autoCoef_Z", autoCoef_Z);

		pixyAnglePID.setKpKi(kP_angleShooter, kI_angleShooter);
		pixy2AnglePID.setKpKi(kP_Gear, kI_Gear);
		pixyDistPID.setKpKi(kP_distShooter, kI_distShooter);

		SmartDashboard::PutNumber("age of pixy data", increment_since_last_find);
		SmartDashboard::PutNumber("age of pixy data2", increment_since_last_find2);
		SmartDashboard::PutNumber("age of pixy2 data2", p2increment_since_last_find);
		SmartDashboard::PutNumber("age of pixy vert data", increment_since_last_good_verticle);

		SmartDashboard::PutNumber("kP_angleShooter", kP_angleShooter);
		SmartDashboard::PutNumber("kI_angleShooter", kI_angleShooter);
		SmartDashboard::PutNumber("kP_distShooter", kP_distShooter);
		SmartDashboard::PutNumber("kI_distShooter", kI_distShooter);
		SmartDashboard::PutNumber("kP_Gear", kP_Gear);
		SmartDashboard::PutNumber("kI_Gear", kI_Gear);

		SmartDashboard::PutNumber("autoCoef_X", autoCoef_X);
		SmartDashboard::PutNumber("autoCoef_Y", autoCoef_Y);
		SmartDashboard::PutNumber("autoCoef_Z", autoCoef_Z);
	}
	bool gumballBool;
	void TeleopPeriodic() {
//		pixyDistanceTrim();
		SmartDashboard::PutNumber("POV numbers", stick.GetPOV());
		unjamInput();
		numBlocks = myPixy.rrPixyGetBlocks(2); // was getBlocks(100);
		numBlocks2 = myPixy2.rrPixyGetBlocks(100); // was getBlocks(100);
		SmartDashboard::PutNumber("number of blocks P2", numBlocks2);
		turnAngle();
		turnAngle2();
//		closeDistance(20); //Lidar distance in cm
		trackTwo = false;
//		lidarServo->SetAngle(lidarServoAngle); // Turn Pixy to the left to drop gears
//		lastLidarDistance = myLidar.GetLidarDistance();
		teleopStick1();
		teleopStick2();
		pixyTeleDebugs(); // Alive increments
		frc::SmartDashboard::PutNumber("Winch Motor Current", m_pdp.GetCurrent(3));
		SmartDashboard::PutNumber("Current Lidar Measurement", lastLidarDistance);
		SmartDashboard::PutNumber("Navx yaw", ahrs->GetYaw());
		frc::SmartDashboard::PutNumber("Gumball Current", m_pdp.GetCurrent(7));
		frc::SmartDashboard::PutNumber("Current Channel 7", m_pdp.GetCurrent(7));
		frc::SmartDashboard::PutNumber("Current Wheel Motor Auto Trigger", m_pdp.GetCurrent(3));
	}

	void TestPeriodic() {
		lw->Run();
	}
private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> autoFunction;
	frc::SendableChooser<std::string> redVsBlue;
	frc::SendableChooser<std::string> rightVsLeft;
	const std::string leftSide = "Left side";
	const std::string rightSide = "Right Side";
	const std::string centerSide = "Center Side";

	const std::string gearDropOnly = "Drop Gear";
	const std::string shootOnly = "Shoot Only";
	const std::string dropAndShoot = "Drop Gear and Shoot";

	const std::string redSide = "Red Side";
	const std::string blueSide = "Blue Side";

	std::string autoSelectedFunction;
	std::string autoSelectedSide;
	std::string autoSelectedColor;

	// Object for dealing with the Power Distribution Panel (PDP).
	frc::PowerDistributionPanel m_pdp;

};

START_ROBOT_CLASS(Robot)
