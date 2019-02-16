/*
 * DriveTrain.cpp
 *
 *  Created on: Nov 10, 2018
 *      Author: super-tails
 */

#include "subsystems/DriveTrain.h"
#include "Robot.h"
#include <iostream>
#include <algorithm>
#include <SmartDashboard/SmartDashboard.h>

//Determines if the left drivetrain should phase or not (this keeps the drivetrains in sync so they go the same speed)
void DriveTrain::setLeftSidePhase(bool phase) {
	frc::SmartDashboard::SetPersistent("Left Sensor Phase");
	frc::SmartDashboard::PutBoolean("Left Sensor Phase", phase);

	m_frontLeft.SetSensorPhase(phase);
}

//Determines if the right drivetrain should phase or not (this keeps the drivetrains in sync so they go the same speed)
void DriveTrain::setRightSidePhase(bool phase) {
	frc::SmartDashboard::SetPersistent("Right Sensor Phase");
	frc::SmartDashboard::PutBoolean("Right Sensor Phase", phase);

	m_frontRight.SetSensorPhase(phase);
}

//Gets the left sensor phase set with setLeftSidePhase()
bool DriveTrain::leftSidePhase() const {
	return frc::SmartDashboard::GetBoolean("Left Sensor Phase", false);
}

//Gets the right sensor phase set with setRightSidePhase()
bool DriveTrain::rightSidePhase() const {
	return frc::SmartDashboard::GetBoolean("Right Sensor Phase", false);
}

//Returns the current position of the left and right encoders
std::pair< int, int > DriveTrain::getEncoderPositions() {
	return { m_frontLeft.GetSelectedSensorPosition(), m_frontRight.GetSelectedSensorPosition() };
}

//Function that allows for individual control of all the motors on the test robot.
DriveTrain::DriveTrain(int frontLeft, int midLeft, int backLeft, int frontRight, int midRight, int backRight) :
	m_frontLeft(frontLeft),
	m_midLeft(midLeft),
	m_backLeft(backLeft),
	m_frontRight(frontRight),
	m_midRight(midRight),
	m_backRight(backRight),
	m_arm(7),
	m_shiftFast(0),
	m_shiftSlow(1),
	Subsystem("DriveTrain")
{
	//If phase is enabled on the left side this will disable it.
	if (!frc::SmartDashboard::SetDefaultBoolean("Left Sensor Phase", false)) {
		std::cout << "Setting default left side phase\n";
		setLeftSidePhase(false);
	}
	else {
		setLeftSidePhase(leftSidePhase());
	}

	//If phase is enabled on the right side this will disable it.
	if (!frc::SmartDashboard::SetDefaultBoolean("Right Sensor Phase", false)) {
		std::cout << "Setting default right side phase\n";
		setRightSidePhase(false);
	}
	else {
		setRightSidePhase(rightSidePhase());
		std::cout << "Setting default right side phase\n";
		setRightSidePhase(false);
	}

	//Left motors will follow the front left motor.
	m_midLeft.Follow(m_frontLeft);
	m_backLeft.Follow(m_frontLeft);

	//Right motors will follow the front right motor.
	m_midRight.Follow(m_frontRight);
	m_backRight.Follow(m_frontRight);

	//Inverts the left motors so all motors drive in the same direction.
	m_frontLeft.SetInverted(true);
	m_midLeft.SetInverted(true);
	m_backLeft.SetInverted(true);

	// Configure both front left and front right talons identically
	std::array< WPI_TalonSRX*, 2 > talons = { &m_frontLeft, &m_frontRight };
	for (int i = 0; i < 2; ++i) {
		talons[i]->ConfigNominalOutputForward(0, 10);
		talons[i]->ConfigNominalOutputReverse(0, 10);
		talons[i]->ConfigPeakOutputForward(1, 10);
		talons[i]->ConfigPeakOutputReverse(-1, 10);

		talons[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
		talons[i]->ConfigClosedloopRamp(0.0, 10);
	}

	//High speed config for the front left motor.
	m_frontLeft.Config_kF(0, 0.0779131, 10);
	m_frontLeft.Config_kP(0, 0.07, 10);
	m_frontLeft.Config_kI(0, 0.0, 10);
	m_frontLeft.Config_kD(0, 0, 10);

	//Low speed config for the front right motor.
	m_frontRight.Config_kF(0, 0.0905309, 10);
	m_frontRight.Config_kP(0, 0.09, 10);
	m_frontRight.Config_kI(0, 0 , 10);
	m_frontRight.Config_kD(0, 0, 10);

	//High speed config for the front left motor.
	m_frontLeft.Config_kF(1, 0.0300882, 10);
	m_frontLeft.Config_kP(1, 0.03, 10);
	m_frontLeft.Config_kI(1, 0.0, 10);
	m_frontLeft.Config_kD(1, 0, 10);

	//High speed config for the front right motor.
	m_frontRight.Config_kF(1, 0.0343288, 10);
	m_frontRight.Config_kP(1, 0.03, 10);
	m_frontRight.Config_kI(1, 0.0, 10);
	m_frontRight.Config_kD(1, 0, 10);

	//Disables phasing for the two front motors
	m_frontLeft.SetSensorPhase(false);
	m_frontRight.SetSensorPhase(false);

	frc::Scheduler::GetInstance()->RegisterSubsystem(this);
}

//Sets manual control as the default for controlling.
void DriveTrain::InitDefaultCommand() {
	SetDefaultCommand(Robot::m_manualControl.get());
}

//Reset the motor sensors (likely to get rid of inconsistencies).
void DriveTrain::resetSensors() {
	m_frontLeft.SetSelectedSensorPosition(0, 0, 10);
	m_frontRight.SetSelectedSensorPosition(0, 0, 10);
}

//Makes low speed the default
bool fast = false;

//Gets input from the joystick for manual control of the robot.
void DriveTrain::drive(InputState state) {
	double lSpeed = -state.y + state.r;
	double rSpeed = -state.y - state.r;
	lSpeed = std::max(std::min(lSpeed, 1.0), -1.0);
	rSpeed = std::max(std::min(rSpeed, 1.0), -1.0);

	m_shiftFast.Set(buttonValue(state, "SHIFT_FAST"));
	m_shiftSlow.Set(buttonValue(state, "SHIFT_SLOW"));

	fast |= buttonValue(state, "SHIFT_FAST");
	fast &= !buttonValue(state, "SHIFT_SLOW");

	drive(lSpeed, rSpeed, buttonValue(state, "TRIGGER"));
}

//RIP Elevator code.
/*void DriveTrain::driveElevator() {
	drive(0.1,0.1, false);
}*/


//Gets the sign of a given number.
template < typename T >
int signum(const T& value) {
	return (value > 0) - (value < 0);
}

//Calibrates the sensors for both motors so they will operate in phase with each other. Individual speeds for each motor may also be set.
void DriveTrain::calibratePhase(double leftSpeed, double rightSpeed) {
	// Allow time for measured velocity to change after inverting a phase
	static int debounce = 0;

	//Ends function if velocity is not properly reset by setting debounce to 0.
	if (debounce > 0) {
		--debounce;
		return;
	}

	//Obtains the velocity of the front left motor.
	double leftVelocity = m_frontLeft.GetSelectedSensorVelocity();

	//Makes sure that speed, signum, and velocity are in the correct range to keep the left motors in phase.
	if (std::abs(leftSpeed) > 0.5 && std::abs(leftVelocity) > 10 && signum(leftVelocity) != signum(leftSpeed)) {
		bool currentPhase = leftSidePhase();
		std::cout << "Left side sensor phase '" << std::boolalpha << currentPhase << "' incorrect, inverting...\n";
		setLeftSidePhase(!currentPhase);
		debounce = 1000;
	}

	//Obtains the velocity of the front right motor.
	double rightVelocity = m_frontRight.GetSelectedSensorVelocity();

	//Makes sure that speed, signum, and velocity are in the correct range to keep the right motors in phase.
	if (std::abs(rightSpeed) > 0.5 && std::abs(rightVelocity) > 10 && signum(rightVelocity) != signum(rightSpeed)) {
		bool currentPhase = rightSidePhase();
		std::cout << "Right side sensor phase '" << std::boolalpha << currentPhase << "' incorrect, inverting...\n";
		setRightSidePhase(!currentPhase);
		debounce = 1000;
	}
}

//Function for setting the speed or operating through percent output for the motors while the robot is being driven.
void DriveTrain::drive(double leftSpeed, double rightSpeed, bool percent) {
	//calibratePhase(leftSpeed, rightSpeed);

	//If operating through percent output sets contorl mode to percent output and speed is controlled through that.
	if (percent) {
		m_frontLeft.Set(ControlMode::PercentOutput, leftSpeed);
		m_frontRight.Set(ControlMode::PercentOutput, rightSpeed);
	}
	else {
		if (fast) {
			//Sets the profile for high speed motor operation.
			m_frontLeft.SelectProfileSlot(1, 0);
			m_frontRight.SelectProfileSlot(1, 0);

			//Sets and adjusts the velocity of the front motors
			m_frontLeft.Set(ControlMode::Velocity, leftSpeed * 28000.0);
			m_frontRight.Set(ControlMode::Velocity, rightSpeed * 28000.0);
		}
		else {
			//Sets the profile for low speed motor operation.
			m_frontLeft.SelectProfileSlot(0, 0);
			m_frontRight.SelectProfileSlot(0, 0);

			//Sets and adjusts the velocity of the front motors
			m_frontLeft.Set(ControlMode::Velocity, leftSpeed * 10000.0);
			m_frontRight.Set(ControlMode::Velocity, rightSpeed * 10000.0);
		}
	}

	//Sets and adjusts default average velocity of 30 for the front motors.
	static MovingAverage leftVelocityAverage(30);
	static MovingAverage rightVelocityAverage(30);

	//Defines velocity as the value from the front motor on each side of the drivetrain.
	const double leftVelocity = m_frontLeft.GetSelectedSensorVelocity(0);
	const double rightVelocity = m_frontRight.GetSelectedSensorVelocity(0);

	//Sets the Left Side Speed in smart dashboard to the variable leftSpeed.
	frc::SmartDashboard::PutNumber("Left Side Speed", leftSpeed);
	//Sets the Right Side Speed in smart dashboard to the variable rightSpeed.
	frc::SmartDashboard::PutNumber("Right Side Speed", rightSpeed);

	//Sets the Left Side Velocity in smart dashboard to the current calculated velocity.
	frc::SmartDashboard::PutNumber("Left Side Velocity", leftVelocityAverage.Process(leftVelocity));
	//Sets the Right Side Velocity in smart dashboard to the current calculated velocity.
	frc::SmartDashboard::PutNumber("Right Side Velocity", rightVelocityAverage.Process(rightVelocity));

	//Sets the Left Side Error in smart dashboard to the current error in the closed loop (only for high speed operation).
	frc::SmartDashboard::PutNumber("Left Side Error", m_frontLeft.GetClosedLoopError(fast));
	//Sets the Right Side Error in smart dashboard to the current error in the closed loop (only for high speed operation).
	frc::SmartDashboard::PutNumber("Right Side Error", m_frontRight.GetClosedLoopError(fast));
}
