// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  void init();
  void moveFirstJoint(double motor_power);
  void moveSecondJoint(double motor_power);

 private:
    rev::CANSparkMax first_joint_motor{1, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax second_joint_motor{2, rev::CANSparkMax::MotorType::kBrushless};
};
