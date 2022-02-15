// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/Encoder.h>
#include <frc/interfaces/Gyro.h>
#include <AHRS.h>


#include "Trajectories.h"

double dt, lastTime;


Trajectories trajectories;

frc::Joystick controller{0};

// Encoders
frc::Encoder leftEncoder{2,3};
frc::Encoder rightEncoder{5,4};

// Gyro
AHRS *gyro;


double wheelCirc = (M_PI*0.15);

PID::PIDGains gains{"Angle Gains", 0.002, 0.5, 0.0001};
PID::PIDController pidController{gains};

void Robot::RobotInit() {

  leftEncoder.SetDistancePerPulse(1.0/2048.0);
  rightEncoder.SetDistancePerPulse(1.0/2048.0);

  gyro = new AHRS(frc::SPI::Port::kMXP);
  trajectories.build();
}

void Robot::RobotPeriodic() {
  // std::cout << "LeftEncoder: " << (double)leftEncoder.GetDistance() << ", " << "RightEncoder: " << (double)rightEncoder.GetDistance() << ", Gyro: " << gyro->GetAngle() << std::endl;
}

void Robot::AutonomousInit() {
  gyro->Reset();
  leftEncoder.Reset();
  rightEncoder.Reset();
  pidController.setWrap(180);
}

void Robot::AutonomousPeriodic() {
  double currentTime = (double)frc::Timer::GetFPGATimestamp();
  dt = currentTime - lastTime;

  // Initial values to reconfigure
  double leftPower = 0, rightPower = 0;
  double leftEnc = leftEncoder.GetDistance(), rightEnc = leftEncoder.GetDistance();
  double robotGyro = gyro->GetAngle();

  double distance = (((leftEnc + rightEnc)/2.0)*wheelCirc);

  std::cout << "Distance: " << distance << ", Gyro: " << robotGyro << "\n";

  if (distance < trajectories.trajectory.getRawTrajectory().totalLength) {
    leftPower = 0.25;
    rightPower = 0.25; 


    double goalAngle = trajectories.trajectory.getAngle(distance);
    
    // PID
    pidController.setSetpoint(goalAngle);
    double anglePower = pidController.calculate(robotGyro, dt);

    leftPower += anglePower;
    rightPower -= anglePower;
  }

  // pidController.setSetpoint(90);
  // double anglePower = pidController.calculate(robotGyro, dt);

  // leftPower += anglePower;
  // rightPower -= anglePower;

  m1.Set(-leftPower);
  m2.Set(-leftPower);
  m3.Set(rightPower);
  m4.Set(rightPower);
  lastTime = currentTime;
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  double leftJoy = controller.GetRawAxis(1);
  double rightJoy = controller.GetRawAxis(5);

  double leftPower = 0, rightPower = 0;
  if (fabs(leftJoy) >= 0.15) {
    leftPower = (std::pow(leftJoy, 3)*0.35);
  }

  if (fabs(rightJoy) >= 0.15) {
    rightPower = (std::pow(rightJoy, 3)*0.35);
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
