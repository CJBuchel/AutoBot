// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>


#include <sensors/Encoder.h>
#include <sensors/NavX.h>
#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>

#include "Trajectories.h"

// Motors
rev::CANSparkMax m1{4, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
rev::CANSparkMax m2{1, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
rev::CANSparkMax m3{2, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
rev::CANSparkMax m4{3, rev::CANSparkMaxLowLevel::MotorType::kBrushed};

void set(double left, double right) {
  m1.Set(left);
  m2.Set(left);
  m3.Set(right);
  m4.Set(right);
}

// Joystick
frc::Joystick controller{0};


// Gyro
wml::sensors::NavX navx;
wml::sensors::NavXGyro *gyro;

// Encoders
wml::sensors::DigitalEncoder leftEncoder{2,3, 2048};
wml::sensors::DigitalEncoder rightEncoder{5,4, 2048};

double wheelCirc = (M_PI*0.1524);
Trajectories trajectories;

double dt, lastTime;

void Robot::RobotInit() {
  gyro = new wml::sensors::NavXGyro(navx, wml::sensors::AngularAxis::YAW);
  trajectories.build();
  gyro->Reset();
  leftEncoder.ZeroEncoder();
  rightEncoder.ZeroEncoder();
}


void Robot::RobotPeriodic() {

}


void Robot::AutonomousInit() {
  gyro->Reset();
  leftEncoder.ZeroEncoder();
  rightEncoder.ZeroEncoder();
}

void Robot::AutonomousPeriodic() {
  double currentTime = (double)frc::Timer::GetFPGATimestamp();
  dt = currentTime - lastTime;
  double leftPower = 0, rightPower = 0;

  double distance = (((leftEncoder.GetEncoderRotations() + rightEncoder.GetEncoderRotations())/2.0)*wheelCirc);
  std::pair<double, double> output = trajectories.follow(distance, gyro->GetAngle(), dt, false);

  leftPower = output.first;
  rightPower = output.second;
  
  set(-leftPower, rightPower);
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

  set(leftPower, -rightPower);
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
