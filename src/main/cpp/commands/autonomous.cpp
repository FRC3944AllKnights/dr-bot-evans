#include "commands/autonomous.h"

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
        //turn 180 degrees
        frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive  while the command is executing
             [drive] {drive->Drive(0_mps, 0_mps, 0.1_rad_per_s, false, true);},
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                                        drive->ZeroHeading();},
             //distance to drive
             [drive] {
               return drive->GetPose().Rotation().Degrees() >= 180_deg;
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