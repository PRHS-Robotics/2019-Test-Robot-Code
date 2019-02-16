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
	Robot::m_lights->UpdateDutyCycle(0.2); //sets power of the led lights to 20 percent

	auto ntinstance = nt::NetworkTableInstance::GetDefault(); //receives data from rasberry pi networks
	auto table = ntinstance.GetTable("ChickenVision"); // gets the Chicken Vision network from pi networks

	//checks to make sure the tape function in chicken vision is being used
	nt::NetworkTableEntry useTape = table->GetEntry("Tape"); 
	useTape.SetBoolean(true);
}

void ApproachTape::Execute() {
	auto result = getTargetYaw(); // sets result as the yaw value of the cargo

	//sets the speed that the robot turns to follow the cargo
	//sets the speed that the robot chases the cargo forward
	double turningSpeed = result.first / 50.0;
	double forwardSpeed = result.second * std::max((1.0 - std::abs(result.first / 10.0)) * 0.2, 0.0);

	/* --- TODO: Determine distance --- */
	//if the ultrasonic detects something the robot will stop itself
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
    Robot::m_driveTrain->drive(0.0, 0.0); //sets the speed of the robot to zero
	Robot::m_lights->UpdateDutyCycle(0.0); // sets the power of the lights to zero
}

void ApproachTape::Interrupted() {
    End();
}

std::pair< double, bool > ApproachTape::getTargetYaw() {
	auto ntinstance = nt::NetworkTableInstance::GetDefault(); // retrieves network data from the rasberry pi
	auto table = ntinstance.GetTable("ChickenVision"); //retrieves chicken vision table from rasberry pi 

	nt::NetworkTableEntry detected = table->GetEntry("tapeDetected"); // value of 1 if tape is detected: value of 0 if tape is not detected
	nt::NetworkTableEntry yaw = table->GetEntry("tapeYaw"); //value for the yaw of the tape

	frc::SmartDashboard::PutNumber("Tape Contours", table->GetEntry("tapeContours").GetDouble(0.0)); //detects the number of pieces of tape in the frame of vision

	if (!detected.GetBoolean(false)) {
		return { 0.0, false }; // if no tape is detected then the returned value is false
	}

	return { yaw.GetDouble(0.0) - 5.0, true }; // returns the postition of tape if tape is detected
}