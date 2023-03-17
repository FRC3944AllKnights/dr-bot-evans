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

  //sets cube vs cone mode
  void setCone(); //LB
  void setCube(); //RB
  
  //position presets
  void homePosition();
  void floorPickupPosition(); //↓
  void chutePickupPosition(); //←
  void trayPickupPosition(); //↑
  void bottomDropPosition(); //A
  void midDropPosition(); //X
  void highDropPosition(); //Y

 private:
    bool isConeMode = true;
    double shoulderGearRatio = 36;
    double elbowGearRatio = 25*3;

    rev::CANSparkMax shoulder_motor{1, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxPIDController shoulder_pidController = shoulder_motor.GetPIDController();
    rev::SparkMaxRelativeEncoder shoulder_encoder = shoulder_motor.GetEncoder();

    rev::CANSparkMax elbow_motor{2, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxPIDController elbow_pidController = elbow_motor.GetPIDController();
    rev::SparkMaxRelativeEncoder elbow_encoder = elbow_motor.GetEncoder();

};
