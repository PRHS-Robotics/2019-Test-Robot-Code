#include "subsystems/Vision.h"
#include "Robot.h"
#include "subsystems/DriveTrain.h"
#include "networktables/NetworkTableInstance.h"
#include "cmath"

void VisionSystem::setMode(bool tape) {
	auto ntinstance = nt::NetworkTableInstance::GetDefault();
	auto table = ntinstance.GetTable("ChickenVision");

    table->GetEntry("Tape").SetBoolean(tape);
}

bool VisionSystem::targetFound() const {
    /* TODO */
    return false;
}

Disp VisionSystem::getDisplacement() const {
    return m_disp;
}

void VisionSystem::update() {
    auto newPositions = Robot::m_driveTrain->getEncoderPositions();
    double ypr[3];
    Robot::m_gyro->GetYawPitchRoll(ypr);
    double newAngle = ypr[0];

    if (/* TODO: Check if camera has been updated*/false) {
        m_disp = Disp{ 0.0, 0.0, 0.0 };
    }
    else {
        int changeL = newPositions.first  - m_lastPositionL;
        int changeR = newPositions.second - m_lastPositionR;

        double changeAngle = newAngle - m_lastAngle;

        double xChange = (changeL * METERS_PER_TICK) * std::cos(ypr[0]);
        double yChange = (changeR * METERS_PER_TICK) * std::sin(ypr[0]);

        m_disp.x += xChange;
        m_disp.y += yChange;
        m_disp.angle += changeAngle;
    }

    m_lastPositionL = newPositions.first;
    m_lastPositionR = newPositions.second;
    m_lastAngle = newAngle;
}