// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc/SmartDashboard/SmartDashboard.h>

#include "MotionControlArmSubsystem.h"
#include "Constants.h"

class ArmSubsystem : public frc2::SubsystemBase {


 public: 
  ArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  void init();
  bool getMode();

  void setElbowFast();
  void setElbowSlow();
  void moveArm(double s, double e);

  frc2::CommandPtr moveArmCommand(double s, double e);
  frc2::CommandPtr moveShoulderFirst(double s, double e);
  frc2::CommandPtr moveElbowFirst(double s, double e);
  frc2::CommandPtr moveShoulderCommand(double s);
  frc2::CommandPtr moveElbowCommand(double e);

  frc2::CommandPtr waitForElbowMove(double e);
  frc2::CommandPtr waitForShoulderMove(double s);

  //sets cube vs cone mode
  void setCone(); //LB
  void setCube(); //RB

  double getElbowAngle();
  double getShoulderAngle();
  void getSetStates();
  
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
    /*enum gamePieceSelector { CONE, CUBE };
    gamePieceSelector selectGamePiece() { if(isConeMode){return CONE;}else{return CUBE;}; }
    frc2::CommandPtr m_exampleSelectCommand = frc2::cmd::Select<gamePieceSelector>(
      [this] { return selectGamePiece(); },
      // Maps selector values to commands
      std::pair{CONE, testArm},
      std::pair{CUBE, frc2::cmd::Print("Command two was selected!")});*/

    bool isConeMode = true;

    double desired_elbow_angle = 0;
    double desired_shoulder_angle = 0;

    MotionControlArmSubsystem m_elbow{2, 0, 0.0, 4, 0.2, 0, ArmConstants::elbowGearRatio};
    MotionControlArmSubsystem m_shoulder{1, 1, 0.0, 6, 0.2, 0, ArmConstants::shoulderGearRatio};
};
