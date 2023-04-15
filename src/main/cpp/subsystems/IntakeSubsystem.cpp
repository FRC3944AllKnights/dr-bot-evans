#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem(){
        // set PID coefficients and smartmotion values of shoulder
        intake_pidController.SetP(1e-4);
        intake_pidController.SetI(0.0);
        intake_pidController.SetD(0.0);
        intake_pidController.SetFF(0.000156);
}

void IntakeSubsystem::grabPlace(double LT, double RT){
    if(LT > 0.1) {
        stopSuck(gamePieceMultiplier*LT);
    }
    else if(RT > 0.1){
        stopSuck(-gamePieceMultiplier*RT);
    }
    else{
        stopSuck(0.0);
    }
}

frc2::CommandPtr IntakeSubsystem::autoGrabPlace(double speed, units::second_t timeout){
    return frc2::cmd::Run(
        [this, speed, timeout] { this->stopSuck(speed); }, {this}).WithTimeout(timeout);
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

    frc::SmartDashboard::PutNumber("Current Position", currentPosition);
    frc::SmartDashboard::PutNumber("Set Position", setPosition);


    if(currentInput != previousInput){
        loops = 0;
        positionLatch = true;
    }
    if((positionDifference <= 0.2 and positionDifference >= -0.2 ) and loops >= 10){
      if(positionLatch == true){
        setPosition = currentPosition;
        positionLatch = false;
      }
      intake_pidController.SetReference(setPosition, rev::CANSparkMax::ControlType::kPosition);
      frc::SmartDashboard::PutNumber("If negative, position control", -1);
     
    }
    else{
        intake_motor.Set(intakeSpeed);
        frc::SmartDashboard::PutNumber("If negative, position control", 1);
    }

    previousPosition = currentPosition;
    previousInput = currentInput;
    loops += 1;
}