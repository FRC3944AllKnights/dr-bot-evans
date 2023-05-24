#include "subsystems/ArmSubsystem.h"
#include "subsystems/MotionControlArmSubsystem.h"

#include "Constants.h"

using namespace MotionArmConstants;
using State = frc::TrapezoidProfile<units::radians>::State;

MotionControlArmSubsystem::MotionControlArmSubsystem(int canID, double gearRatio)
    : frc2::ProfiledPIDSubsystem<units::radians>(frc::ProfiledPIDController<units::radians>(
              kP, kI, kD, {kMaxVelocity, kMaxAcceleration})),
      m_feedforward(kS, kG, kV, kA), 
      motor(canID, rev::CANSparkMax::MotorType::kBrushless) {
  motorGearRatio = gearRatio;
  // Start arm in neutral position
  SetGoal(State{kArmOffset, 0_rad_per_s});
}

void MotionControlArmSubsystem::UseOutput(double output, State setpoint) {
  // Calculate the feedforward from the sepoint
  units::volt_t feedforward =
      m_feedforward.Calculate(setpoint.position, setpoint.velocity);
  // Add the feedforward to the PID output to get the motor output
  motor.SetVoltage(units::volt_t{output} + feedforward);
}

units::radian_t MotionControlArmSubsystem::GetMeasurement() {
  return units::radian_t{encoder.GetPosition() / motorGearRatio * ( 2 * 3.1415926535 ) } + kArmOffset;
}