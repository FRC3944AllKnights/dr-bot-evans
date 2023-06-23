// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/MotionControlArmSubsystem.h"

#include "commands/autonomous.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  void initAllSubsystems();

 private:
  // The driver's controller
  frc2::CommandXboxController m_driverController{OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  // ArmSubsystem m_arm;
  IntakeSubsystem m_intake;
  MotionControlArmSubsystem m_elbow{2, 4, 0.2, 0, ArmConstants::elbowGearRatio};
  MotionControlArmSubsystem m_shoulder{1, 6, 0.2, 0, ArmConstants::shoulderGearRatio};
  bool Cube = false;


  //frc2::CommandPtr m_placeCone = autos::PlaceCone(&m_drive, &m_arm, &m_intake);
  //frc2::CommandPtr m_placeConeAndDock = autos::PlaceConeAndDock(&m_drive, &m_arm, &m_intake);
  //frc2::CommandPtr m_placeConeAndBalance = autos::PlaceConeAndBalance(&m_drive, &m_arm, &m_intake);

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};
