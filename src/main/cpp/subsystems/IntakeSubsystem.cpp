#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem(){}

void IntakeSubsystem::grabPlace(double LT, double RT){
    if(LT > 0) {
        stopSuck(gamePieceMultiplier*LT);
    }
    else if(RT > 0){
        stopSuck(-gamePieceMultiplier*RT);
    }
    else{
        intake_motor.Set(0.0);
    }
}

frc2::CommandPtr IntakeSubsystem::autoGrabPlace(double speed, units::second_t timeout){
    return frc2::cmd::Run(
        [this, speed, timeout] { this->intake_motor.Set(speed); }, {this}).WithTimeout(timeout);
}

void IntakeSubsystem::setCube(){
    gamePieceMultiplier = -0.5;
}
void IntakeSubsystem::setCone(){
    gamePieceMultiplier = 0.5;
}
    
void IntakeSubsystem::stopSuck(double intakeSpeed){ 
    if(intakeSpeed > 0){

        currentInput = 1;
    }
    else if(intakeSpeed < 0){

        currentInput = -1;
    }
    else {
        currentInput = 0;
    }

    
    currentPosition = intake_encoder.GetPosition();
    double positionDifference = currentPosition - previousPosition;

    frc::SmartDashboard::PutNumber("Position Difference", positionDifference);

    if((positionDifference <= 0.1 and positionDifference >= -0.1 ) and currentInput == previousInput){

        intake_motor.Set(0.0);

    }
    else{

        intake_motor.Set(intakeSpeed);
        
    }
    previousPosition = currentPosition;
    previousInput = currentInput;
}