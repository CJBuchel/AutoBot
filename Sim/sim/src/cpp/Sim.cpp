#include "Sim.h"

#include "robot/Motor.h"
#include "robot/Encoder.h"
#include "robot/Drivetrain.h"

#include "Trajectories.h"

// #define LAPTOP

#ifdef LAPTOP
double rotsPerMeter = 12;
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

/**
 * Initializer (Updates once)
 */
void Sim::Init() {
  trajectories.build();
}

/**
 * Periodic Update
 */
void Sim::Periodic() {
  double leftPower = 0, rightPower = 0;
  double averageDistance = ((leftEnc.getRotations()/rotsPerMeter) + (rightEnc.getRotations()/rotsPerMeter))/2;

  std::pair<double, double> output = trajectories.follow(averageDistance, World::getGyro(0), Config::Sim::getGlobalDT(), true);
  leftPower = output.first;
  rightPower = output.second;

  drivetrain.set(leftPower, rightPower);
}