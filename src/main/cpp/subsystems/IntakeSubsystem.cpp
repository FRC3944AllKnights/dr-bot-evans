#include "subsystems/IntakeSubsystem.h"
using namespace IntakeConstants;
IntakeSubsystem::IntakeSubsystem(){
}
void IntakeSubsystem::init(){
        // set PID coefficients and smartmotion values of shoulder
        intake_pidController.SetP(intakeP);
        intake_pidController.SetI(intakeI);
        intake_pidController.SetD(intakeD);
        intake_pidController.SetFF(intakeFF);
        intake_pidController.SetIZone(intakeIZone);
        intake_pidController.SetOutputRange(intakeMinOutput, intakeMaxOutput);
}
void IntakeSubsystem::grabPlace(double LT, double RT){
    if(LT > 0.1) {
        stopSuck(gamePieceSpeed*LT);
    }
    else if(RT > 0.1){
        stopSuck(-gamePieceSpeed*RT);
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
    gamePieceSpeed = gamePieceMultiplier * -1;
}
void IntakeSubsystem::setCone(){
    gamePieceSpeed = gamePieceMultiplier * 1;
}
    
void IntakeSubsystem::stopSuck(double intakeSpeed){
    intake_pidController.SetReference(setPosition, rev::CANSparkMax::ControlType::kPosition);
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
        frc::SmartDashboard::PutNumber("If negative, position control", 1);
        intake_motor.Set(intakeSpeed);
        
    }

    frc::SmartDashboard::PutNumber("Motor speed", intake_motor.Get());

    previousPosition = currentPosition;
    previousInput = currentInput;
    loops += 1;
}