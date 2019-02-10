/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#ifndef SRC_SUBSYSTEMS_ELEVATORDRIVETRAIN_H_
#define SRC_SUBSYSTEMS_ELEVATORDRIVETRAIN_H_
#pragma once

#include <frc/commands/Command.h>
#include "subsystems/Input.h"
class ElevatorDriveTrain : public frc::Subsystem {
 public:
  ElevatorDriveTrain(int upmotor, int drivemotor);
  void Drive();
};
#endif /*SRC_SUBSYSTEMS_ELEVATORDRIVETRAIN_H_*/
