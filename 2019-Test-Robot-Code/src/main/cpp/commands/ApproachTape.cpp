#include "commands/ApproachTape.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/ArduinoInterface.h"
#include "Robot.h"

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <iostream>
#include <algorithm>

ApproachTape::ApproachTape() :
    frc::Command("ApproachTape", *Robot::m_driveTrain.get())
{

}

void ApproachTape::Initialize() {
	Robot::m_lights->Set(true);

	auto ntinstance = nt::NetworkTableInstance::GetDefault();
	auto table = ntinstance.GetTable("ChickenVision");

	nt::NetworkTableEntry useTape = table->GetEntry("Tape");
	useTape.SetBoolean(true);
}

void ApproachTape::Execute() {
	auto result = getTargetYaw();

	double turningSpeed = result.first / 50.0;
	double forwardSpeed = result.second * std::max((1.0 - std::abs(result.first / 10.0)) * 0.2, 0.0);

	/* --- TODO: Determine distance --- */
	if (Robot::m_sonarMax->getDistance() < 25.0) {
		turningSpeed = 0.0;
		forwardSpeed = 0.0;
	}

	Robot::m_driveTrain->drive(forwardSpeed + turningSpeed, forwardSpeed - turningSpeed);
}

bool ApproachTape::IsFinished() {
    // TODO: Determine when close enough to tape
    return false;
}

void ApproachTape::End() {
    Robot::m_driveTrain->drive(0.0, 0.0);
	Robot::m_lights->Set(false);
}

void ApproachTape::Interrupted() {
    End();
}

std::pair< double, bool > ApproachTape::getTargetYaw() {
	auto ntinstance = nt::NetworkTableInstance::GetDefault();
	auto table = ntinstance.GetTable("ChickenVision");

	nt::NetworkTableEntry detected = table->GetEntry("tapeDetected");
	nt::NetworkTableEntry yaw = table->GetEntry("tapeYaw");

	frc::SmartDashboard::PutNumber("Tape Contours", table->GetEntry("tapeContours").GetDouble(0.0));

	if (!detected.GetBoolean(false)) {
		return { 0.0, false };
	}

	return { yaw.GetDouble(0.0), true };
}