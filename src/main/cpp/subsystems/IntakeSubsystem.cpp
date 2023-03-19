#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem(){}

void IntakeSubsystem::grabPlace(double LT, double RT){
    if(LT > 0) {
        intake_motor.Set(LT);
    }
    else if(RT > 0){
        intake_motor.Set(-RT);
    }
    else{
        intake_motor.Set(0.05);
    }
}