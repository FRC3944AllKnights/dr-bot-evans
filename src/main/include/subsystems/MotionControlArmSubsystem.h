#pragma once

#include <frc/Encoder.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/ProfiledPIDSubsystem.h>

#include <numbers>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>
#include <frc/SmartDashboard/SmartDashboard.h>



/**
 * A robot arm subsystem that moves with a motion profile.
 */
class MotionControlArmSubsystem : public frc2::ProfiledPIDSubsystem<units::radians> {
  using State = frc::TrapezoidProfile<units::radians>::State;

 public:
  MotionControlArmSubsystem(int canID, double kP, double kI, double kD, double gearRatio);

  void UseOutput(double output, State setpoint) override;

  units::radian_t GetMeasurement() override;

 private:
  rev::CANSparkMax motor;
  rev::SparkMaxRelativeEncoder encoder = motor.GetEncoder();
  frc::ArmFeedforward m_feedforward;
  double motorGearRatio;
  
};

namespace MotionArmConstants {
constexpr int kMotorPort = 4;

constexpr double kAp = 4;
constexpr double kAi = 0.2;
constexpr double kAd = 0;

// These are fake gains; in actuality these must be determined individually for
// each robot
constexpr auto kS = 1_V;
constexpr auto kG = 1_V;
constexpr auto kV = 0.5_V * 1_s / 1_rad;
constexpr auto kA = 0.1_V * 1_s * 1_s / 1_rad;

constexpr auto kMaxVelocity = 3_rad_per_s;
constexpr auto kMaxAcceleration = 10_rad / (1_s * 1_s);

// The offset of the arm from the horizontal in its neutral position,
// measured from the horizontal
constexpr auto kArmOffset = 0.5_rad;
}  // namespace ArmConstants

/*namespace AutoConstants {
constexpr auto kAutoTimeoutSeconds = 12_s;
constexpr auto kAutoShootTimeSeconds = 7_s;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants
*/
