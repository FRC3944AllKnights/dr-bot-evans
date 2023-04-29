#include "subsystems/ArmSubsystem.h"
#include "subsystems/MotionControlArmSubsystem.h"

#include "Constants.h"

using namespace ArmConstants;
using State = frc::TrapezoidProfile<units::radians>::State;

MotionControlArmSubsystem::MotionControlArmSubsystem()
    : frc2::ProfiledPIDSubsystem<units::radians>(
          frc::ProfiledPIDController<units::radians>(
              kP, 0, 0, {kMaxVelocity, kMaxAcceleration})),
      m_motor(kMotorPort),
      m_encoder(kEncoderPorts[0], kEncoderPorts[1]),
      m_feedforward(kS, kG, kV, kA) {
  m_encoder.SetDistancePerPulse(kEncoderDistancePerPulse.value());
  // Start arm in neutral position
  SetGoal(State{kArmOffset, 0_rad_per_s});
}

void MotionControlArmSubsystem::UseOutput(double output, State setpoint) {
  // Calculate the feedforward from the sepoint
  units::volt_t feedforward =
      m_feedforward.Calculate(setpoint.position, setpoint.velocity);
  // Add the feedforward to the PID output to get the motor output
  m_motor.SetVoltage(units::volt_t{output} + feedforward);
}

units::radian_t MotionControlArmSubsystem::GetMeasurement() {
  return units::radian_t{m_encoder.GetDistance()} + kArmOffset;
}