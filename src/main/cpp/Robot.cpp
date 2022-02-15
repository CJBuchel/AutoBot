// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>


#include "TrajectoryUtilities.h"

Trajectory<Splines::CatmullRom> trajectory{{
  {0,4}, {1,4}, {2,4}, {3,4}}
};
frc::Joystick controller{0};

double rawRots = 1.15;
static double covnert2Meters(double rotations) {
  double rotsperMeter = (rawRots/1);
  return rotations/rotsperMeter;
}


void Robot::RobotInit() {
  trajectory.build();
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {
  // Initial values to reconfigure
  double leftPower = 0, rightPower = 0;
  double leftEnc = 0, rightEnc = 0;
  double robotGyro = 0;

  double distance = covnert2Meters(((leftEnc + rightEnc)/2));

  if (distance < trajectory.getRawTrajectory().totalLength) {
    leftPower = 0.15;
    rightPower = 0.15;

    double goalAngle = trajectory.getAngle(distance);

    double angleError = trajectory.wrap(goalAngle-robotGyro, 180);
    angleError /= 180;

    leftPower += angleError;
    rightPower -= angleError;
  }

  m1.Set(leftPower);
  m2.Set(leftPower);
  m3.Set(-rightPower);
  m4.Set(-rightPower);
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  double leftJoy = controller.GetRawAxis(1);
  double rightJoy = controller.GetRawAxis(5);

  double leftPower = 0, rightPower = 0;
  if (fabs(leftJoy) >= 0.15) {
    leftPower = std::pow(leftJoy, 3);
  }

  if (fabs(rightJoy) >= 0.15) {
    rightPower = std::pow(rightJoy, 3);
  }

  m1.Set(leftPower);
  m2.Set(leftPower);

  m3.Set(-rightPower);
  m4.Set(-rightPower);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
