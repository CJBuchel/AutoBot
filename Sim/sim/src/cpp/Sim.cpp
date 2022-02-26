#include "Sim.h"

#include "robot/Motor.h"
#include "robot/Encoder.h"
#include "robot/Drivetrain.h"

#include "Control.h"

#define LAPTOP

#ifdef LAPTOP
double rotsPerMeter = 7.5; // laptop
#else
double rotsPerMeter = 7.5; // 7.5
#endif

// Robot
Motor left1{0}, left2{1}, right1{2}, right2{3};
Encoder leftEnc{0}, rightEnc{1};
Gearbox leftGearbox{{&left1, &left2}, &leftEnc};
Gearbox rightGearbox{{&right1, &right2}, &rightEnc};

DrivetrainConfig config{leftGearbox, rightGearbox};
Drivetrain drivetrain{config};

Trajectories trajectories;

std::vector<Splines::Waypoint> draw_points;

/**
 * Initializer (Updates once)
 */
void Sim::Init() {
  trajectories.build();

  double step = 0.01;
  for (double i = 0.0f; i < trajectories.trajectory.getRawTrajectory().totalLength; i+=step) {
    Splines::Waypoint point = trajectories.trajectory.getCoords(i);
    draw_points.push_back(point);
  }
}

void Sim::drawTrajectory() {
  for (auto point : draw_points) {
    double x = (point.x*100);
    double y = (point.y*100);
    cv::circle(_window.getWindow(), {(int)x, (int)y}, 1, cv::Scalar(255,255,255), -1);
  }

  for (auto controlPoint : trajectories.trajectory.getRawTrajectory().points) {
    double x = controlPoint.x*100;
    double y = controlPoint.y*100;

    cv::circle(_window.getWindow(), {(int)x,(int)y}, 4, cv::Scalar(0,0,255), -1);
  }
}

/**
 * Periodic Update
 */
void Sim::Periodic() {
  drawTrajectory();
  double leftPower = 0, rightPower = 0;
  double averageDistance = ((leftEnc.getRotations()/rotsPerMeter) + (rightEnc.getRotations()/rotsPerMeter))/2;

  std::pair<double, double> output = trajectories.follow(averageDistance, World::getGyro(0), Config::Sim::getGlobalDT(), true);
  leftPower = output.first;
  rightPower = output.second;

  drivetrain.set(leftPower, rightPower);
}