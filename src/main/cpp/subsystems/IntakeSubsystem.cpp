#include "subsystems/IntakeSubsystem.h""

IntakeSubsystem::IntakeSubsystem(){}

void IntakeSubsystem::grabPlace(double LT, double RT){
    if(LT > 0) {
        intake_motor.Set(LT);
    }
    else{
        intake_motor.Set(-RT);
    }
}