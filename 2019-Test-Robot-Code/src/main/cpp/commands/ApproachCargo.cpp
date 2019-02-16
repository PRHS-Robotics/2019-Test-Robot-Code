#include "commands/ApproachCargo.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/ArduinoInterface.h"
#include "Robot.h"

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <iostream>
#include <algorithm>

ApproachCargo::ApproachCargo() :
    frc::Command("ApproachCargo", *Robot::m_driveTrain.get())
{

}

void ApproachCargo::Initialize() {
	auto ntinstance = nt::NetworkTableInstance::GetDefault(); // recieves data from the rasberry pi networks
	auto table = ntinstance.GetTable("ChickenVision"); // sets the chicken vision network data to the table variable

	//ensures that the tape function is not selected
	nt::NetworkTableEntry useTape = table->GetEntry("Tape"); 
	useTape.SetBoolean(false);
}

void ApproachCargo::Execute() {
	auto result = getTargetYaw(); // sets result as the yaw value of the detected cargo

	//sets the speed that the robot turns to follow the robot and the forward speed to chase the ball
	double turningSpeed = result.first / 50.0;
	double forwardSpeed = std::max((1.0 - std::abs(result.first / 10.0), 0.0) * 0.2, 0.0);

	Robot::m_driveTrain->drive(forwardSpeed + turningSpeed, forwardSpeed - turningSpeed);
}

bool ApproachCargo::IsFinished() {
    // TODO: Determine when close enough to cargo
    return false;
}

void ApproachCargo::End() {
    Robot::m_driveTrain->drive(0.0, 0.0); // sets the robot speed to zero
}

void ApproachCargo::Interrupted() {
    End();
}

std::pair< double, bool > ApproachCargo::getTargetYaw() {
	auto ntinstance = nt::NetworkTableInstance::GetDefault();
	auto table = ntinstance.GetTable("ChickenVision");

	nt::NetworkTableEntry detected = table->GetEntry("cargoDetected"); //sets detected as 1 if cargo is detected: 0 if cargo is not detected
	nt::NetworkTableEntry yaw = table->GetEntry("cargoYaw"); // sets the yaw value for the cargo

	frc::SmartDashboard::PutNumber("Cargo Contours", table->GetEntry("cargoContours").GetDouble(0.0)); // detects the number of objects seen as cargo in the view

	if (!detected.GetBoolean(false)) {
		return { 0.0, false }; // checks to see of anything is detected
	}

	return { yaw.GetDouble(0.0), true }; // returns the yaw of the detected cargo
}