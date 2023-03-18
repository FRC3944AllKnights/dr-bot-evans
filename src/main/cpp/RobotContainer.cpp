// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc2/command/button/POVButton.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ArmSubsystem.h"


using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true, true);
      },
      {&m_drive}));

      m_intake.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_intake.grabPlace(
            {frc::ApplyDeadband(
                m_driverController.GetLeftTriggerAxis(), OIConstants::kDriveDeadband)},
            {frc::ApplyDeadband(
                m_driverController.GetRightTriggerAxis(), OIConstants::kDriveDeadband)});
      },
      {&m_intake}));
}

void RobotContainer::ConfigureButtonBindings() {
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart).OnTrue(
        new frc2::InstantCommand([this] { m_drive.SetX(); }, {&m_drive})
    );

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart).OnTrue(
        new frc2::InstantCommand([this] { m_arm.init(); }, {&m_arm})
    );

    //arm drop positions
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA).OnTrue(
        new frc2::InstantCommand([this] {m_arm.bottomDropPosition(); }, {&m_arm})
    );

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX).OnTrue(
        new frc2::InstantCommand([this] {m_arm.midDropPosition(); }, {&m_arm})
    );

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY).OnTrue(
        new frc2::InstantCommand([this] {m_arm.highDropPosition(); }, {&m_arm})
    );

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper).OnTrue(
        new frc2::InstantCommand([this] {m_arm.setCone(); }, {&m_arm})
    );

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper).OnTrue(
        new frc2::InstantCommand([this] {m_arm.setCube(); }, {&m_arm})
    );

    //arm pickup positions
    frc2::POVButton(&m_driverController, 0).OnTrue(
        new frc2::InstantCommand([this] {m_arm.trayPickupPosition(); }, {&m_arm})
    );

    frc2::POVButton(&m_driverController, 270).OnTrue(
        new frc2::InstantCommand([this] {m_arm.chutePickupPosition(); }, {&m_arm})
    );

    frc2::POVButton(&m_driverController, 180).OnTrue(
        new frc2::RunCommand([this] {m_arm.floorPickupPosition(); }, {&m_arm})
    );

    frc2::POVButton(&m_driverController, 90).OnTrue(
        new frc2::InstantCommand([this] {m_arm.homePosition(); }, {&m_arm})
    );
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{3_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController{AutoConstants::kPXController, 0, 0},
      frc2::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, true, false); },
          {}));
}
