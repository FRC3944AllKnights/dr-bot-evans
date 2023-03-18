#pragma once
#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  void grabPlace(double LT, double RT); //LT, RT

 private:

 rev::CANSparkMax intake_motor{3, rev::CANSparkMax::MotorType::kBrushless};







};