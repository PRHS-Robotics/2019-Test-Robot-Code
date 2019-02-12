#include "commands/ApproachCargo.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/ArduinoInterface.h"
#include "Robot.h"

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <iostream>

ApproachCargo::ApproachCargo(int yawSamples) :
    m_yawSamples(yawSamples),
    m_yawAverager(m_yawSamples),
    frc::Command("ApproachCargo", *static_cast< frc::Subsystem* >(Robot::m_driveTrain.get()))
{

}

void ApproachCargo::Initialize() {
    m_lastDetected = false;
	Robot::m_arduino->readData(false);

	auto ntinstance = nt::NetworkTableInstance::GetDefault();
	auto table = ntinstance.GetTable("ChickenVision");

	nt::NetworkTableEntry useTape = table->GetEntry("Tape");
	useTape.SetBoolean(false);
}

void ApproachCargo::Execute() {
	auto ntinstance = nt::NetworkTableInstance::GetDefault();
	auto table = ntinstance.GetTable("ChickenVision");

	nt::NetworkTableEntry detected = table->GetEntry("cargoDetected");
	nt::NetworkTableEntry yaw = table->GetEntry("cargoYaw");

	frc::SmartDashboard::PutNumber("Cargo Contours", table->GetEntry("cargoContours").GetDouble(0.0));
	static double yawCurrent = 0;

    if (detected.GetBoolean(false)) {
		std::cout << "Yaw: " << yaw.GetDouble(0.0) << "\n";

		double yawValue = yaw.GetDouble(0.0);

		const int SAMPLES = 2;
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

		double yawDiff;

		yawDiff = yawAverage - yawCurrent;
		yawCurrent += yawDiff / 20.0;
		Robot::m_driveTrain->drive(yawCurrent / 50.0 + speed, -yawCurrent / 50.0 + speed);
	}
	else {
		Robot::m_driveTrain->drive(0.0, 0.0);
	}


bool ApproachCargo::IsFinished() {
    // TODO: Determine when close enough to cargo
    return false;
}

void ApproachCargo::End() {
    Robot::m_driveTrain->drive(0.0, 0.0);
	Robot::m_arduino->readData(false);
}

void ApproachCargo::Interrupted() {
    End();
}