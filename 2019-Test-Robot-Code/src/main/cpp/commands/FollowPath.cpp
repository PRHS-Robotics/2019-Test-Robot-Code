#include "commands/FollowPath.h"
#include "subsystems/DriverTrain.h"

FollowPath::FollowPath(std::vector< Segment > leftData, std::vector< Segment > rightData) :
    m_lFollower{ 0, 0, 0, 0, 0 },
    m_rFollower{ 0, 0, 0, 0, 0 },
    m_config{ 0, 96, 0.6383528 /* ish */, 1.0, 0.0, 0.0, 1.0 / 15.0, 0.0 }
    m_leftData(leftData),
    m_rightData(rightData),
    Command("FollowPath", Robot::m_driveTrain.get())
{

}

void FollowPath::Initialize() {

    Robot::m_driveTrain->resetSensors();

    // TODO: Measure wheel circumference
    // TODO: Adjust gains
    // TODO: Adjust max speed
    // m_config

}

void FollowPath::Execute() {
    double l = pathfinder_follow_encoder(
        m_config,
        &m_lFollower,
        m_leftData.get(),
        m_leftData.size(),
        Robot::m_driveTrain->getEncoderPositions().first
    );

    double r = pathfinder_follow_encoder(
        m_config,
        &m_rFollower,
        m_rightData.get(),
        m_rightData.size(),
        Robot::m_driveTrain->getEncoderPositions().second
    );

    // TODO: Apparently you need a gyro????

    Robot::m_driveTrain->drive(l, r);
}