#include "commands/FollowPath.h"
#include "subsystems/DriveTrain.h"
#include "Robot.h"

#include <iostream>

FollowPath::FollowPath(std::vector< Segment > leftData, std::vector< Segment > rightData) :
    m_lFollower(EncoderFollower{ 0, 0, 0, 0, 0 }),
    m_rFollower(EncoderFollower{ 0, 0, 0, 0, 0 }),
    m_config(EncoderConfig{ 0, 4096, 0.314159265/* ish */,  0.01, 0 , 0.0, 1.0 / 1.5, 0.0 }),
    m_leftData(leftData),
    m_rightData(rightData),
    Command("FollowPath", *Robot::m_driveTrain.get())
{

}

void FollowPath::Initialize() {

    Robot::m_driveTrain->resetSensors();

    m_lFollower = EncoderFollower{ 0, 0, 0, 0, 0 };
    m_rFollower = EncoderFollower{ 0, 0, 0, 0, 0 };

    // TODO: Measure wheel circumference
    // TODO: Adjust gains
    // TODO: Adjust max speed
    // m_config

}

void FollowPath::Execute() {
    double l = pathfinder_follow_encoder(
        m_config,
        &m_lFollower,
        m_leftData.data(),
        m_leftData.size(),
        Robot::m_driveTrain->getEncoderPositions().first
    );

    double r = pathfinder_follow_encoder(
        m_config,
        &m_rFollower,
        m_rightData.data(),
        m_rightData.size(),
        Robot::m_driveTrain->getEncoderPositions().second
    );
    
    double ypr[3];
    Robot::m_gyro->GetYawPitchRoll(ypr);
    double gyro_heading = std::fmod((std::fmod(ypr[0], 360) + 360), 360);  // Assuming gyro angle is given in degrees
    double desired_heading = std::fmod((std::fmod(r2d(m_lFollower.heading), 360) + 360), 360);

    std::cout << "Follower: " << desired_heading << ", Gyro: " << gyro_heading << "\n";

    double temp = desired_heading - gyro_heading;
    double angle_difference = temp;
    if (temp > 180) {
        angle_difference -= 360;
    }
    if (temp < -180) {
        angle_difference += 360;
    }
    
    double turn = /*0.8*/1.0 * (-1.0/80.0) * angle_difference;

    Robot::m_driveTrain->drive(l + turn, r - turn);
}

bool FollowPath::IsFinished() {
    return m_lFollower.finished && m_rFollower.finished;
}

void FollowPath::End() {
    Robot::m_driveTrain->drive(0.0, 0.0);
}

void FollowPath::Interrupted() {
    End();
}