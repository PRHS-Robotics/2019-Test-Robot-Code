#include "commands/ApproachTape.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/ArduinoInterface.h"
#include "Robot.h"

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <iostream>

ApproachTape::ApproachTape(int yawSamples) :
    m_yawSamples(yawSamples),
    m_yawAverager(m_yawSamples),
    frc::Command("ApproachTape", *static_cast< frc::Subsystem* >(Robot::m_driveTrain.get()))
{

}

void ApproachTape::Initialize() {
    m_lastDetected = false;
	Robot::m_arduino->readData(true);

	auto ntinstance = nt::NetworkTableInstance::GetDefault();
	auto table = ntinstance.GetTable("ChickenVision");

	nt::NetworkTableEntry useTape = table->GetEntry("Tape");
	useTape.SetBoolean(true);
	
}

void ApproachTape::Execute() {
	auto ntinstance = nt::NetworkTableInstance::GetDefault();
	auto table = ntinstance.GetTable("ChickenVision");

	nt::NetworkTableEntry detected = table->GetEntry("tapeDetected");
	nt::NetworkTableEntry yaw = table->GetEntry("tapeYaw");
	nt::NetworkTableEntry distance = table->GetEntry("tapeDistance");
	frc::SmartDashboard::PutNumber("Tape Contours", table->GetEntry("tapeContours").GetDouble(0.0));

	SonarMax sensorboi(3);

	if(sensorboi.getDistance()>8){

		static MovingAverage yawAverager(m_yawSamples);
    	if (detected.GetBoolean(false)) {
			std::cout << "Yaw: " << yaw.GetDouble(0.0) << "\n";

			double yawValue = yaw.GetDouble(0.0);

			const int SAMPLES = 10;
			static MovingAverage yawAverager(SAMPLES);

			if (detected.GetBoolean(false) && !m_lastDetected) {
				yawAverager.Clear();
			}

			m_lastDetected = detected.GetBoolean(false);

			double speed = 0.0;

			double yawAverage = yawAverager.Process(yawValue);

			if (yawAverage > -10 && yawAverage < 10) {
				speed = (1.0 - std::abs(yawAverage / 10.0)) * 0.2;
			}

			frc::SmartDashboard::PutNumber("Average Yaw", yawAverage);

			// TODO: Add gradual ramp-up
			Robot::m_driveTrain->drive(yawAverage / 60.0 + speed, -yawAverage / 60.0 + speed);
		}
	}
}

bool ApproachTape::IsFinished() {
    // TODO: Determine when close enough to tape
    return false;
}

void ApproachTape::End() {
    Robot::m_driveTrain->drive(0.0, 0.0);
	Robot::m_arduino->readData(false);
}

void ApproachTape::Interrupted() {
    End();
}