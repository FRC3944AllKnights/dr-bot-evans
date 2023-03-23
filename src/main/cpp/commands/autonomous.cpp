#include "commands/autonomous.h"

#include <frc/SmartDashboard/SmartDashboard.h>

using namespace AutoConstants;

frc2::CommandPtr autos::SimpleAuto(DriveSubsystem* drive) {
    return frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive forward while the command is executing
             [drive] {drive->Drive(-0.1_mps, 0_mps, 0_rad_per_s, false, true);},
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
             //distance to drive
             [drive] {
               return drive->GetPose().X() <= 1_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr();
}

frc2::CommandPtr autos::PlaceConeAndDriveBack(DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake) {
    return frc2::cmd::Sequence(
        arm->highDropPosition(),
        frc2::cmd::Wait(3.0_s).AsProxy().AndThen([intake] {intake->grabPlace(0.0, 0.3);}),
        frc2::cmd::Wait(1.0_s).AsProxy().AndThen(arm->homePosition()),
        //drive backward
        frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive while the command is executing
             [drive] {drive->Drive(-0.1_mps, 0_mps, 0_rad_per_s, false, true);},
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
             //distance to drive
             [drive] {
              frc::SmartDashboard::PutNumber("gyro heading", drive->GetPose().X().value());
              return drive->GetPose().X() <= -1.0_m;
               
             },
             // Requires the drive subsystem
             {drive}).ToPtr(),
        //turn 180 degrees
        frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive  while the command is executing
             [drive] {drive->Drive(0_mps, 0_mps, 0.3_rad_per_s, false, true);},
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, false);},
             //distance to drive
             [drive] {
              frc::SmartDashboard::PutNumber("gyro heading", drive->GetPose().Rotation().Degrees().value());
               return drive->GetPose().Rotation().Degrees() >= 148.0_deg;
             },
             // Requires the drive subsystem
             {drive}).ToPtr(),
        //drive forward
        frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive while the command is executing
             [drive] {drive->Drive(0.1_mps, 0_mps, 0_rad_per_s, false, true);},
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
             //distance to drive
             [drive] {
               return drive->GetPose().X() <= 1_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr()
    );
}