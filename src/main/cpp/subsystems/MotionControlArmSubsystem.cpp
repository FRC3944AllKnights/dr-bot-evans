#include "subsystems/ArmSubsystem.h"
#include "subsystems/MotionControlArmSubsystem.h"

#include "Constants.h"

using namespace MotionArmConstants;
using State = frc::TrapezoidProfile<units::radians>::State;

MotionControlArmSubsystem::MotionControlArmSubsystem(int canID, int potID, double potStart, double kP, double kI, double kD, double gearRatio)
    : frc2::ProfiledPIDSubsystem<units::radians>(frc::ProfiledPIDController<units::radians>(
              kP, kI, kD, {kMaxVelocity, kMaxAcceleration})),
      m_feedforward(kS, kG, kV, kA), 
      motor(canID, rev::CANSparkMax::MotorType::kBrushless),
      pot(potID, 4.712, 0.0) {
  motorGearRatio = gearRatio;
  // Start arm in neutral position
  SetGoal(State{0_rad, 0_rad_per_s});
  offset = potStart;
}

void MotionControlArmSubsystem::GetArmPosition(){
  std::string id = std::to_string(motorGearRatio);
  frc::SmartDashboard::PutNumber("motor " + id + " current position", (-offset + pot.Get()));
}

void MotionControlArmSubsystem::UseOutput(double output, State setpoint) {
  // Calculate the feedforward from the sepoint
  units::volt_t feedforward =
      m_feedforward.Calculate(setpoint.position, setpoint.velocity);
      std::string id = std::to_string(motorGearRatio);
      frc::SmartDashboard::PutNumber("motor" + id + " desired position", setpoint.position.value());
  // Add the feedforward to the PID output to get the motor output
  motor.SetVoltage(units::volt_t{output} + feedforward);

} 

void MotionControlArmSubsystem::SetLimits(units::angular_velocity::radians_per_second_t maxVelocity
  , units::angular_acceleration::radians_per_second_squared_t maxAcceleration){
      frc::ProfiledPIDController<units::angle::radians>::Constraints constraints = {maxVelocity, maxAcceleration};
      m_controller.SetConstraints(constraints);
  }

units::radian_t MotionControlArmSubsystem::GetMeasurement() {
  return units::radian_t{ (-offset + pot.Get())};
}