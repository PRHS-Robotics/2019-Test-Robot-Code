#include "commands/ManualControl.h"
#include "Robot.h"
#include "subsystems/DriveTrain.h"

ManualControl::ManualControl(Input *input) :
    m_input(input),
    Command("ManualControl", *static_cast< frc::Subsystem* >(Robot::m_driveTrain.get()))
{

}

void ManualControl::Execute() {
    Robot::m_driveTrain->drive(m_input->getInput()); //recieves driver input to control robot 
}

bool ManualControl::IsFinished() {
    return false;
}

void ManualControl::End() {
    Robot::m_driveTrain->drive(0.0, 0.0); //sets speed of robot to zero
}

void ManualControl::Interrupted() {
    End();
}