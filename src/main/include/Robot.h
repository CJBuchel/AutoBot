// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <rev/CANSparkMax.h>

rev::CANSparkMax m1{4, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
rev::CANSparkMax m2{1, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
rev::CANSparkMax m3{2, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
rev::CANSparkMax m4{3, rev::CANSparkMaxLowLevel::MotorType::kBrushed};


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
};
