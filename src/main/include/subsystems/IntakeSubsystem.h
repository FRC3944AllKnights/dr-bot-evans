#pragma once
#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
    public:
        IntakeSubsystem();
        void grabPlace(double LT, double RT); //LT, RT
        void setCube();
        void setCone();

        /**
         * runs at a set speed repeatedly until timeout condition
        */
        frc2::CommandPtr autoGrabPlace(double speed, units::second_t timeout);

    private:
        double gamePieceMultiplier = 0.5;
        rev::CANSparkMax intake_motor{3, rev::CANSparkMax::MotorType::kBrushless};
};