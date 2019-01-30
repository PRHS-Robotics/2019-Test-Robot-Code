/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>

#include <string>
#include <thread>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/SerialPort.h>
#include <frc/AnalogInput.h>
#include <frc/Compressor.h>
#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include <networktables/NetworkTable.h>

class DriveTrain;
class Input;
class Autonomous;
class Arduino;

class Robot : public frc::TimedRobot {
public:
	void RobotInit() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;

	static std::unique_ptr< DriveTrain > m_driveTrain;
	static std::unique_ptr< Input > m_input;
	static std::unique_ptr< Autonomous > m_autonomous;
	static std::unique_ptr< Arduino > m_arduino;
	static std::unique_ptr< frc::SerialPort > m_serialPort;
	static std::unique_ptr< frc::AnalogInput > m_analogInput;
	static std::unique_ptr< frc::Compressor > m_compressor;
	static std::unique_ptr< std::thread > m_calculation;
};
