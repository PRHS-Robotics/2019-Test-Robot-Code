/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include "subsystems/DriveTrain.h"
#include "subsystems/Input.h"
#include "subsystems/Autonomous.h"
#include "subsystems/ArduinoInterface.h"

#include <iostream>

#include <SmartDashboard/SmartDashboard.h>

#include <frc/Timer.h>

#include <thread>

int main(int argc, char *argv[]) {
	frc::StartRobot< Robot >();
}

std::pair< std::vector< Segment >, std::vector< Segment > > pathresult;

auto f = [](std::vector< Waypoint > waypoints) {
	pathresult = generateTrajectory(waypoints);
};


std::unique_ptr< DriveTrain > Robot::m_driveTrain{};
std::unique_ptr< Input > Robot::m_input{};
//std::unique_ptr< Autonomous > Robot::m_autonomous{};
std::unique_ptr< Arduino > Robot::m_arduino{};
std::unique_ptr< frc::SerialPort > Robot::m_serialPort{};
std::unique_ptr< frc::AnalogInput > Robot::m_analogInput{};
std::unique_ptr< frc::Compressor > Robot::m_compressor{};
std::unique_ptr< std::thread > Robot::m_calculation{};

std::unique_ptr< ManualControl > Robot::m_manualControl{};
std::unique_ptr< ApproachCargo > Robot::m_approachCargo{};
std::unique_ptr< SpeedTest > Robot::m_speedTest{};
std::unique_ptr< FollowPath > Robot::m_followPath{};

void Robot::RobotInit() {
	m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
	m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	m_driveTrain = std::make_unique< DriveTrain >(1, 2, 3, 4, 5, 6);

	m_input = std::make_unique< Input >(1, 2);

	m_serialPort = std::make_unique< frc::SerialPort >(9600, frc::SerialPort::Port::kUSB);
	if (m_serialPort && m_serialPort->StatusIsFatal()) {
		m_serialPort = nullptr;
	}

	m_arduino = std::make_unique< Arduino >();

	m_analogInput = std::make_unique< frc::AnalogInput >(0);
	if (m_analogInput) {
		if (m_analogInput->StatusIsFatal()) {
			m_analogInput = nullptr;
		}
		else {
			m_analogInput->SetSampleRate(62500);
			m_analogInput->SetAverageBits(12);
		}
	}

	m_compressor = std::make_unique< frc::Compressor >();
	if (m_compressor) {
		if (m_compressor->StatusIsFatal()) {
			m_compressor = nullptr;
		}
		else {
			m_compressor->Start();
		}
	}

	std::vector< Waypoint > waypoints = {
		{ 0.0, 1.0, 0.0 },
		{ 0.0, 3.0, 0.0 }
	};

	m_manualControl = std::make_unique< ManualControl >(Robot::m_input.get()),
	m_approachCargo = std::make_unique< ApproachCargo >(10),
	m_speedTest = std::make_unique< SpeedTest >(Robot::m_input.get()),

	//m_calculation = std::make_unique< std::thread >(f, waypoints);

	/*frc::CameraServer *camser = frc::CameraServer::GetInstance();
	camser->StartAutomaticCapture();*/

	frc::SmartDashboard::init();

	frc::SmartDashboard::PutNumber("Forward Limit", 3.000);
	frc::SmartDashboard::PutNumber("Reverse Limit", 2.883);

	m_input->getButton("MANUAL_OVERRIDE")->WhenPressed(m_manualControl.get());
	m_input->getButton("SEARCH_AND_DESTROY")->WhenPressed(m_approachCargo.get());
	m_input->getButton("DEBUG_BUTTON_2")->WhenPressed(m_speedTest.get());
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */

void Robot::AutonomousInit() {
	m_autoSelected = m_chooser.GetSelected();
	// m_autoSelected = SmartDashboard::GetString(
	// 		"Auto Selector", kAutoNameDefault);
	std::cout << "Auto selected: " << m_autoSelected << std::endl;


	if (m_autoSelected == kAutoNameCustom) {
		// Custom Auto goes here
	} else {
		// Default Auto goes here
	}
}

void Robot::AutonomousPeriodic() {
	m_driveTrain->drive(0.0, 0.0);

	if (m_autoSelected == kAutoNameCustom) {
		// Custom Auto goes here
	} else {
		// Default Auto goes here
	}
}

void Robot::TeleopInit() {
	m_driveTrain->resetSensors();

	if(m_arduino->handshake()) {
		std::cout << "Successfully communicated with Arduino\n";
	}
	else {
		std::cout << "Failed to establish communication with Arduino\n";
	}

	//m_calculation->join();

	for (auto& point : pathresult.first) {
		std::cout << "Time: " << point.dt << "\n";
	}	
}

void Robot::TeleopPeriodic() {
	frc::SmartDashboard::PutNumber("Analog Input Raw", m_analogInput->GetVoltage());
	frc::SmartDashboard::PutNumber("Analog Input Averaged", m_analogInput->GetAverageVoltage());

	frc::Scheduler::GetInstance()->Run();

	if (buttonValue(m_input->getInput(), "DEBUG_BUTTON")) {
		auto result = m_arduino->readData();
		if (result.second) {
			SensorFrame data = result.first;
			std::cout << "Degrees: " << data.degrees << "\n";
			std::cout << "Distance: " << data.distance << "\n";
			std::cout << sizeof(float) << "\n";

			float value = 2.53;
			for (int i = 0; i < 4; ++i) {
				std::cout << std::hex << int(reinterpret_cast< unsigned char* >(&value)[i]) << ", ";
			}
			std::cout << "\n";
		}
	}
}

void Robot::TestPeriodic() {}

//START_ROBOT_CLASS(Robot)
