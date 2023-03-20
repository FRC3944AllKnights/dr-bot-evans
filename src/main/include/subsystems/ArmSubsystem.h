// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <frc/SmartDashboard/SmartDashboard.h>

#include "Constants.h"

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  void init();

  frc2::CommandPtr moveArmCommand(double shoulder_angle, double elbow_angle);
  frc2::CommandPtr moveShoulderCommand(double desired_angle);
  frc2::CommandPtr moveElbowCommand(double desired_angle);

  frc2::CommandPtr waitForElbowMove(double desired_angle);
  frc2::CommandPtr waitForShoulderMove(double desired_angle);

  //sets cube vs cone mode
  void setCone(); //LB
  void setCube(); //RB
  
  //position presets
  frc2::CommandPtr homePosition(); //→
  frc2::CommandPtr floorPickupPosition(); //↓
  frc2::CommandPtr chutePickupPosition(); //←
  frc2::CommandPtr trayPickupPosition(); //↑ 
  frc2::CommandPtr bottomDropPosition(); //A
  frc2::CommandPtr midDropPosition(); //X
  frc2::CommandPtr highDropPosition(); //Y

  frc2::CommandPtr testArm();

 private:
    bool isConeMode = true;
    double shoulderGearRatio = -64.0/360.0;
    double elbowGearRatio = -25.0*60.0/16.0/360.0;

    rev::CANSparkMax shoulder_motor{1, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxPIDController shoulder_pidController = shoulder_motor.GetPIDController();
    rev::SparkMaxRelativeEncoder shoulder_encoder = shoulder_motor.GetEncoder();

    rev::CANSparkMax elbow_motor{2, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxPIDController elbow_pidController = elbow_motor.GetPIDController();
    rev::SparkMaxRelativeEncoder elbow_encoder = elbow_motor.GetEncoder();
};
