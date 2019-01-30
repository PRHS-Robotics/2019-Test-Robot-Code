#include "commands/ApproachCargo.h"
#include "subsystems/DriveTrain.h"
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
}

void ApproachCargo::Execute() {
	auto ntinstance = nt::NetworkTableInstance::GetDefault();
	auto table = ntinstance.GetTable("ChickenVision");

	nt::NetworkTableEntry detected = table->GetEntry("cargoDetected");
	nt::NetworkTableEntry yaw = table->GetEntry("cargoYaw");

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
		Robot::m_driveTrain->drive(yawAverage / 30.0 + speed, -yawAverage / 30.0 + speed);
	}
}

bool ApproachCargo::IsFinished() {
    // TODO: Determine when close enough to cargo
    return false;
}

void ApproachCargo::End() {
    Robot::m_driveTrain->drive(0.0, 0.0);
}

void ApproachCargo::Interrupted() {
    End();
}