#include "commands/SpeedTest.h"
#include "Robot.h"
#include "subsystems/DriveTrain.h"

SpeedTest::SpeedTest(double speed) :
    m_speed(speed),
    Command("SpeedTest", *static_cast< frc::Subsystem* >(Robot::m_driveTrain.get()))
{

}

void SpeedTest::Initialize() {

}

void SpeedTest::Execute() {
    Robot::m_driveTrain->drive(m_speed, m_speed);
}

bool SpeedTest::IsFinished() {
    return false;
}

void SpeedTest::End() {
    Robot::m_driveTrain->drive(0.0, 0.0);
}

void SpeedTest::Interrupted() {
    End();
}