#pragma once

struct Disp {
    double x = 0.0, y = 0.0, angle = 0.0;
};

class VisionSystem {
public:
    VisionSystem() = default;

    void setMode(bool tape);

    bool targetFound() const;

    void update();

    Disp getDisplacement() const;

private:
    // Tracks movement since last camera update
    Disp m_disp = { 0, 0, 0 };

    int m_lastPositionL = 0;
    int m_lastPositionR = 0;
    
    double m_lastAngle = 0;

    double METERS_PER_TICK = 0.0 /* TODO */;
};