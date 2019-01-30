#pragma once

#include <frc/commands/Command.h>

class SpeedTest : public frc::Command {
public:
    SpeedTest(double speed);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    void End() override;

    void Interrupted() override;

private:
    double m_speed;
};