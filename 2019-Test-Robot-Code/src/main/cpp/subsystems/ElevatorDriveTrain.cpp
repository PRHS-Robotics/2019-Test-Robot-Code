/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//#include "Robot.h"
#include "subsystems/ElevatorDriveTrain.h"
#include <SmartDashboard/SmartDashboard.h>
//ElevatorDriveTrain::ElevatorDriveTrain() {}
ElevatorDriveTrain::ElevatorDriveTrain(int upmotor, int drivemotor):
Subsystem("ElevatorDriveTrain")
{}




void ElevatorDriveTrain::Drive() {
  frc::SmartDashboard::PutString("elevating?", "elevating")

}
