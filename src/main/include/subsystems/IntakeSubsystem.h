#pragma once
#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <frc/SmartDashboard/SmartDashboard.h>

#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
    public:
        IntakeSubsystem();
        void grabPlace(double LT, double RT); //LT, RT
        void setCube();
        void setCone();
        void stopSuck(double intakeSpeed);
        void init();

        /**
         * runs at a set speed repeatedly until timeout condition
        */
        frc2::CommandPtr autoGrabPlace(double speed, units::second_t timeout);

    private:
        double gamePieceMultiplier = 0.45;
        double previousPosition = 10;
        double currentPosition = 0;
        double previousInput = 0;
        double currentInput = 0;
        double setPosition = 0;
        int loops = 0;
        bool positionLatch = true;
        rev::CANSparkMax intake_motor{3, rev::CANSparkMax::MotorType::kBrushless};
        rev::SparkMaxRelativeEncoder intake_encoder = intake_motor.GetEncoder();
        rev::SparkMaxPIDController intake_pidController = intake_motor.GetPIDController();
};

