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


/**
 * A robot arm subsystem that moves with a motion profile.
 */
class MotionControlArmSubsystem : public frc2::ProfiledPIDSubsystem<units::radians> {
  using State = frc::TrapezoidProfile<units::radians>::State;

 public:
  MotionControlArmSubsystem();

  void UseOutput(double output, State setpoint) override;

  units::radian_t GetMeasurement() override;

 private:
  frc::PWMSparkMax m_motor;
  frc::Encoder m_encoder;
  frc::ArmFeedforward m_feedforward;
};

namespace DriveConstants {
constexpr int kLeftMotor1Port = 0;
constexpr int kLeftMotor2Port = 1;
constexpr int kRightMotor1Port = 2;
constexpr int kRightMotor2Port = 3;

constexpr int kLeftEncoderPorts[]{0, 1};
constexpr int kRightEncoderPorts[]{2, 3};
constexpr bool kLeftEncoderReversed = false;
constexpr bool kRightEncoderReversed = true;

constexpr int kEncoderCPR = 1024;
constexpr double kWheelDiameterInches = 6;
constexpr double kEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterInches * std::numbers::pi) /
    static_cast<double>(kEncoderCPR);
}  // namespace DriveConstants

namespace ArmConstants {
constexpr int kMotorPort = 4;

constexpr double kP = 1;

// These are fake gains; in actuality these must be determined individually for
// each robot
constexpr auto kS = 1_V;
constexpr auto kG = 1_V;
constexpr auto kV = 0.5_V * 1_s / 1_rad;
constexpr auto kA = 0.1_V * 1_s * 1_s / 1_rad;

constexpr auto kMaxVelocity = 3_rad_per_s;
constexpr auto kMaxAcceleration = 10_rad / (1_s * 1_s);

constexpr int kEncoderPorts[]{4, 5};
constexpr int kEncoderPPR = 256;
constexpr auto kEncoderDistancePerPulse =
    2.0_rad * std::numbers::pi / kEncoderPPR;

// The offset of the arm from the horizontal in its neutral position,
// measured from the horizontal
constexpr auto kArmOffset = 0.5_rad;
}  // namespace ArmConstants

namespace AutoConstants {
constexpr auto kAutoTimeoutSeconds = 12_s;
constexpr auto kAutoShootTimeSeconds = 7_s;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants

