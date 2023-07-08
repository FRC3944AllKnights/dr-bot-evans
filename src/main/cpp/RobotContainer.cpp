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
#include <units/angular_velocity.h>
#include <frc2/command/button/POVButton.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ArmSubsystem.h"


using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
 /* m_chooser.SetDefaultOption("place cone", m_placeCone.get());
  m_chooser.AddOption("place cone and dock", m_placeConeAndDock.get());
  m_chooser.AddOption("place cone and balance", m_placeConeAndBalance.get());
*/

  // Put the chooser on the dashboard
  frc::SmartDashboard::PutData(&m_chooser);

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY()*m_drive.slowFactor, OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX()*m_drive.slowFactor, OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX()*m_drive.slowFactor, OIConstants::kDriveDeadband)},
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
      {&m_intake}
    ));

    m_shoulder.SetDefaultCommand(frc2::RunCommand(
        [this] {
            m_shoulder.GetArmPosition();
        },
        {&m_shoulder}
    ));

    m_elbow.SetDefaultCommand(frc2::RunCommand(
        [this] {
            m_elbow.GetArmPosition();
        },
        {&m_elbow}
    ));    

    
    /*m_arm.SetDefaultCommand(frc2::RunCommand(
        [this] {
            m_arm.getSetStates();
        },
        {&m_arm}
    ));*/
}

void RobotContainer::initAllSubsystems() {
    //m_arm.init();
    m_intake.init();
    m_elbow.SetLimits(3_rad_per_s, 10_rad / (1_s * 1_s));
    m_shoulder.SetLimits(3_rad_per_s, 10_rad / (1_s * 1_s));
}

void RobotContainer::ConfigureButtonBindings() {
    m_driverController.Start().OnTrue(new frc2::InstantCommand([this] { m_drive.SetX(); }, {&m_drive}));
    // m_driverController.Start().OnTrue(new frc2::InstantCommand([this] { m_arm.init(); }, {&m_arm}));

    m_driverController.Back().OnTrue(new frc2::InstantCommand([this] { m_drive.ZeroHeading(); }, {&m_drive}));

    //gamepiece placing positions
   // m_driverController.B().OnTrue(m_arm.testArm());
    m_driverController.B().OnTrue(m_drive.setSlowFactor(0.1));

    //  m_driverController.A().OnTrue(m_arm.bottomDropPosition());
    m_driverController.A().OnTrue(m_drive.setSlowFactor(0.1));

    // m_driverController.X().OnTrue(m_arm.midDropPosition());
    m_driverController.X().OnTrue(m_drive.setSlowFactor(0.1));

    //  m_driverController.Y().OnTrue(m_arm.highDropPosition());
    m_driverController.Y().OnTrue(m_drive.setSlowFactor(0.1));

    //gamepiece pickup positions
   // frc2::POVButton(&m_driverController, 0).OnTrue(m_arm.trayPickupPosition());
    frc2::POVButton(&m_driverController, 0).OnTrue(m_drive.setSlowFactor(0.1));

   // frc2::POVButton(&m_driverController, 270).OnTrue(m_arm.chutePickupPosition());
    frc2::POVButton(&m_driverController, 270).OnTrue(m_drive.setSlowFactor(0.1));

   // frc2::POVButton(&m_driverController, 180).OnTrue(m_arm.floorPickupPosition());
    frc2::POVButton(&m_driverController, 180).OnTrue(m_drive.setSlowFactor(0.25));

   // frc2::POVButton(&m_driverController, 90).OnTrue(m_arm.homePosition());
    frc2::POVButton(&m_driverController, 90).OnTrue(m_drive.setSlowFactor(0.5));

    //choose gamepiece
    m_driverController.LeftBumper().OnTrue(new frc2::InstantCommand([this] {Cube = false; }));
    m_driverController.LeftBumper().OnTrue(new frc2::InstantCommand([this] {m_intake.setCone(); }, {&m_intake}));

    m_driverController.RightBumper().OnTrue(new frc2::InstantCommand([this] {Cube = true; }));
    m_driverController.RightBumper().OnTrue(new frc2::InstantCommand([this] {m_intake.setCube(); }, {&m_intake}));

     // Move the arm to 2 radians above horizontal when the 'A' button is pressed.

    
  m_driverController.A().OnTrue(frc2::cmd::RunOnce(
      [this] {
        if(Cube == true){ /*elbow gear ratio = 93.75
                            shoulder gear ratio = 64 */
          m_elbow.SetGoal(units::radian_t{-21 / m_elbow.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_elbow.Enable();
          m_shoulder.SetGoal(units::radian_t{-0.7 / m_shoulder.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_shoulder.Enable();
        }
        else{
          m_elbow.SetGoal(units::radian_t{-1.4 });
          m_elbow.Enable();
          m_shoulder.SetGoal(units::radian_t{0 });
          m_shoulder.Enable();
        }
        
        
      },
      {&m_elbow, &m_shoulder}));

  // Move the arm to neutral position when the 'B' button is pressed.
 
  m_driverController.B().OnTrue(frc2::cmd::RunOnce(
      [this] {
        if(Cube == true){ 
          m_elbow.SetGoal(units::radian_t{-15 / m_elbow.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_elbow.Enable();
          m_shoulder.SetGoal(units::radian_t{-2.2 / m_shoulder.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_shoulder.Enable();
        }
        else{
          m_elbow.SetGoal(units::radian_t{-18 / m_elbow.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_elbow.Enable();
          m_shoulder.SetGoal(units::radian_t{-6 / m_shoulder.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_shoulder.Enable();
        }
        
      },
      {&m_elbow, &m_shoulder}));

      m_driverController.Y().OnTrue(frc2::cmd::RunOnce(
      [this] {
        if(Cube == true){
          m_elbow.SetGoal(units::radian_t{-26.5 / m_elbow.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_elbow.Enable();
          m_shoulder.SetGoal(units::radian_t{-9 / m_shoulder.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_shoulder.Enable();
        }
        else{
          m_elbow.SetGoal(units::radian_t{-41.6 / m_elbow.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_elbow.Enable();
          m_shoulder.SetGoal(units::radian_t{-14.3 / m_shoulder.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_shoulder.Enable();
        }
        
        
      },
      {&m_elbow, &m_shoulder}));


      frc2::POVButton(&m_driverController, 0).OnTrue(frc2::cmd::RunOnce(
      [this] {
        if(Cube == true){
          m_elbow.SetGoal(units::radian_t{-29 / m_elbow.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_elbow.Enable();
          m_shoulder.SetGoal(units::radian_t{0 / m_shoulder.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_shoulder.Enable();
        }
        else{
          m_elbow.SetGoal(units::radian_t{-20 / m_elbow.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_elbow.Enable();
          m_shoulder.SetGoal(units::radian_t{0 / m_shoulder.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_shoulder.Enable();
        }
        
        
      },
      {&m_elbow, &m_shoulder}));

      frc2::POVButton(&m_driverController, 270).OnTrue(frc2::cmd::RunOnce(
      [this] {
        if(Cube == true){
          m_elbow.SetGoal(units::radian_t{-7.7 / m_elbow.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_elbow.Enable();
          m_shoulder.SetGoal(units::radian_t{-0.9 / m_shoulder.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_shoulder.Enable();
        }
        else{
          m_elbow.SetGoal(units::radian_t{-9.4 / m_elbow.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_elbow.Enable();
          m_shoulder.SetGoal(units::radian_t{-4.8 / m_shoulder.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_shoulder.Enable();
        }
        
        
      },
      {&m_elbow, &m_shoulder}));

      frc2::POVButton(&m_driverController, 180).OnTrue(frc2::cmd::RunOnce(
      [this] {
        if(Cube == true){
          m_elbow.SetGoal(units::radian_t{-32.8 / m_elbow.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_elbow.Enable();
          m_shoulder.SetGoal(units::radian_t{12.6 / m_shoulder.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_shoulder.Enable();
          }
          
      },
      {&m_elbow, &m_shoulder}));

      frc2::POVButton(&m_driverController, 90).OnTrue(frc2::cmd::RunOnce(
      [this] {
        if(Cube == true){
          m_elbow.SetGoal(units::radian_t{0 / m_elbow.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_elbow.Enable();
          m_shoulder.SetGoal(units::radian_t{0 / m_shoulder.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_shoulder.Enable();
        }
        else{
          m_elbow.SetGoal(units::radian_t{0 / m_elbow.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_elbow.Enable();
          m_shoulder.SetGoal(units::radian_t{0 / m_shoulder.motorGearRatio * ( 2 * 3.1415926535 ) });
          m_shoulder.Enable(); 
        }

      },
      {&m_elbow, &m_shoulder}));

  
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_chooser.GetSelected();
}
