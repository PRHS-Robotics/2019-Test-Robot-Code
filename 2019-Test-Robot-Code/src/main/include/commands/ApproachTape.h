#pragma once

#include <frc/commands/Command.h>

#include <ctre/Phoenix.h>

class ApproachTape : public frc::Command {
public:
    ApproachTape(int yawSamples);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    void End() override;

    void Interrupted() override;

private:
	bool m_lastDetected = false;
    const int m_yawSamples;
    MovingAverage m_yawAverager;
};