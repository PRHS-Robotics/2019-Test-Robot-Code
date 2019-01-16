/*
 * Autonomous.h
 *
 *  Created on: Jan 3, 2019
 *      Author: super-tails
 */

#ifndef SRC_SUBSYSTEMS_AUTONOMOUS_H_
#define SRC_SUBSYSTEMS_AUTONOMOUS_H_

#include <cstddef>
#include <vector>
#include <tuple>
#include <pathfinder.h>
#include <ctre/Phoenix.h>

constexpr const std::size_t EDGES_PER_INCH_FAST = 10;
constexpr const std::size_t EDGES_PER_INCH_SLOW = 5;

/*
 * measured in native sensor units
 * +y
 *  ^
 *  |
 *  0 --> +x
 *
 */

class DriveTrain;

struct Action {
	double leftSpeed;
	double rightSpeed;
	double duration;
};

std::pair< std::vector< Segment >, std::vector< Segment > > generateTrajectory(std::vector< Waypoint >& waypoints);

void loadMotionProfilePoints(WPI_TalonSRX& motor, std::vector< Segment >& points);

void loadPath(WPI_TalonSRX& leftMotor, WPI_TalonSRX& rightMotor, std::vector< Waypoint >& path);

class Autonomous {
public:

private:

};



#endif /* SRC_SUBSYSTEMS_AUTONOMOUS_H_ */
