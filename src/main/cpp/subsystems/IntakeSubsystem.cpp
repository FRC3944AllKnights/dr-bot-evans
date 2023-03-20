#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem(){}

void IntakeSubsystem::grabPlace(double LT, double RT){
    if(LT > 0) {
        intake_motor.Set(gamePieceMultiplier*LT);
    }
    else if(RT > 0){
        intake_motor.Set(-gamePieceMultiplier*RT);
    }
    else{
        intake_motor.Set(0.0);
    }
}

void IntakeSubsystem::setCube(){
    gamePieceMultiplier = -0.5;
}
void IntakeSubsystem::setCone(){
    gamePieceMultiplier = 0.5;
}