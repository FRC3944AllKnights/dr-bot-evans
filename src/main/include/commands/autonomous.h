#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "Constants.h"

/** Container for auto command factories. */
namespace autos {

/**
 * Places a cone, then turns.
 */
frc2::CommandPtr PlaceCone(DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake);

/**
 * places a cone, then drives backward onto the ramp (doesn't do active balance)
 */
frc2::CommandPtr PlaceConeAndDock(DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake);

/**
 * Place a cone
*/
frc2::CommandPtr PlaceConeAndBalance(DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake);

}  // namespace autos