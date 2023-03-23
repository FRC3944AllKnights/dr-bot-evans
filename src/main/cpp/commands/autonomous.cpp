#include "commands/autonomous.h"

#include <frc/SmartDashboard/SmartDashboard.h>

using namespace AutoConstants;

frc2::CommandPtr autos::PlaceCone(DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake) {
    return frc2::cmd::Sequence(
        arm->highDropPosition(),
        frc2::cmd::Wait(2.0_s).AsProxy().AndThen(intake->autoGrabPlace(-0.3, 1.0_s)),
        //drive backward to avoid arm collision
        frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive while the command is executing
             [drive] {drive->Drive(-0.1_mps, 0_mps, 0_rad_per_s, false, true);},
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
             //distance to drive
             [drive] {
              frc::SmartDashboard::PutNumber("distance", drive->GetPose().X().value());
              return drive->GetPose().X() <= -0.5_m;
               
             },
             // Requires the drive subsystem
             {drive}).ToPtr(),
        arm->homePosition(),
        //turn 180 degrees
        frc2::cmd::Wait(0.5_s).AsProxy().AndThen(frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // turn in place
             [drive] {drive->Drive(0_mps, 0_mps, 0.3_rad_per_s, false, true);},
             // stop turning
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, false);},
             // turn 180 degrees (cancel early due to extra movement)
             [drive] {
              frc::SmartDashboard::PutNumber("gyro heading", drive->GetPose().Rotation().Degrees().value());
               return drive->GetPose().Rotation().Degrees() >= 148.0_deg;
             },
             // Requires the drive subsystem
             {drive}).ToPtr())
    );
}

frc2::CommandPtr autos::PlaceConeAndDock(DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake) {
    return frc2::cmd::Sequence(
        arm->highDropPosition(),
        frc2::cmd::Wait(2.0_s).AsProxy().AndThen(intake->autoGrabPlace(-0.3, 1.0_s)),
        //drive backward to avoid arm collision
        frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive while the command is executing
             [drive] {drive->Drive(-0.1_mps, 0_mps, 0_rad_per_s, false, true);},
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
             //distance to drive
             [drive] {
              frc::SmartDashboard::PutNumber("distance", drive->GetPose().X().value());
              return drive->GetPose().X() <= -0.5_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr(),
        arm->homePosition(),
        //drive backward to charge station
        frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive while the command is executing
             [drive] {drive->Drive(-0.1_mps, 0_mps, 0_rad_per_s, false, true);},
             // stop driving and lock in place
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, false);
                                        drive->SetX();},
             //distance to drive
             [drive] {
              frc::SmartDashboard::PutNumber("distance", drive->GetPose().X().value());
              return drive->GetPose().X() <= -1.5_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr()
    );
}

frc2::CommandPtr autos::PlaceConeAndBalance(DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake) {
    return frc2::cmd::Sequence(
        arm->highDropPosition(),
        frc2::cmd::Wait(2.0_s).AsProxy().AndThen(intake->autoGrabPlace(-0.3, 1.0_s)),
        //drive backward to avoid arm collision
        frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive while the command is executing
             [drive] {drive->Drive(-0.1_mps, 0_mps, 0_rad_per_s, false, true);},
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
             //distance to drive
             [drive] {
              frc::SmartDashboard::PutNumber("distance", drive->GetPose().X().value());
              return drive->GetPose().X() <= -0.5_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr(),
        arm->homePosition(),
        //drive backward to charge station
        frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive while the command is executing
             [drive] {drive->Drive(-0.4_mps, 0_mps, 0_rad_per_s, false, true);},
             // stop driving and lock in place
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, false);
                                        drive->SetX();},
             //distance to drive
             [drive] {
              frc::SmartDashboard::PutNumber("distance", drive->GetPose().X().value());
              return (drive->GetPose().X() <= -2.0_m || abs(drive->GetRoll()) < -8);
             },
             // Requires the drive subsystem
             {drive}).ToPtr(),
        frc2::cmd::Run([drive] {drive->autoBalance();}).WithTimeout(30.0_s)
    );
}