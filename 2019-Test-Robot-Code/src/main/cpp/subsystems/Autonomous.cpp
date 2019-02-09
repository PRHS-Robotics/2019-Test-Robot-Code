/*
 * Autonomous.cpp
 *
 *  Created on: Jan 3, 2019
 *      Author: super-tails
 */

#include "subsystems/Autonomous.h"
#include "subsystems/DriveTrain.h"
#include "Timer.h"
#include <ctre/Phoenix.h>
#include <iostream>

std::pair< std::vector< Segment >, std::vector< Segment > > generateTrajectory(std::vector< Waypoint >& waypoints) {
	TrajectoryCandidate candidate{ 0 };
	std::cout << "I AM RUNNING THE CODE";
	// TODO: Determine actual maximum velocity and acceleration values
	candidate.length = pathfinder_prepare(waypoints.data(), waypoints.size(), FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST, 0.02, 1.5, 0.25, 60.0, &candidate);

	//candidate.length = candidate.path_length;

	if (candidate.length == INT_MAX) {
		std::cout << "Invalid candidate length\n";
		return { {}, {} };
	}

	std::cout << "Candidate length: " << candidate.length << "\n";

	std::vector< Segment > trajectory(candidate.length);

	// Create the central trajectory
	pathfinder_generate(&candidate, trajectory.data());

	std::vector< Segment > leftTrajectory(candidate.length);
	std::vector< Segment > rightTrajectory(candidate.length);

	const double wheelbase_width = 0.6;

	pathfinder_modify_tank(trajectory.data(), candidate.length, leftTrajectory.data(), rightTrajectory.data(), wheelbase_width);

	return { leftTrajectory, rightTrajectory };
}

void loadMotionProfilePoints(WPI_TalonSRX& motor, std::vector< Segment >& points) {
	for (std::size_t i = 0; i < points.size(); ++i) {
		auto& segment = points[i];

		TrajectoryPoint point;

		// TODO: Double check scaling factors
		point.position = segment.position * 96.0 * 0.319; // 96.0 edges per rotation, 0.319 meters per rotation
		point.velocity = segment.velocity / 0.319 * 96.0 / 10.0; // 1 rotation per 0.319 meters, 96.0 edges per rotations, 1 second / 10 100ms
		point.headingDeg = 0.0;
		point.profileSlotSelect0 = 0;
		point.profileSlotSelect1 = 0;
		point.timeDur = segment.dt; // TODO: Determine if we need to subtract the default time interval from this value
		point.zeroPos = (i == 0);
		point.isLastPoint = (i + 1 == points.size());

		motor.PushMotionProfileTrajectory(point);
	}
}

void loadPath(WPI_TalonSRX& leftMotor, WPI_TalonSRX& rightMotor, std::vector< Waypoint >& path) {
	auto result = generateTrajectory(path);

	loadMotionProfilePoints(leftMotor, result.first);
	loadMotionProfilePoints(rightMotor, result.second);
}
