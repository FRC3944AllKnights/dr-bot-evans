#include "subsystems/ArmSubsystem.h"

using namespace ArmConstants;

ArmSubsystem::ArmSubsystem() {}

void ArmSubsystem::init(){
    // set PID coefficients and smartmotion values of shoulder
    shoulder_pidController.SetP(shoulderP);
    shoulder_pidController.SetI(shoulderI);
    shoulder_pidController.SetD(shoulderD);
    shoulder_pidController.SetFF(shoulderFF);
    shoulder_pidController.SetOutputRange(shoulderMinOutput, shoulderMaxOutput);
    shoulder_pidController.SetSmartMotionMaxVelocity(shoulderMaxVel);
    shoulder_pidController.SetSmartMotionMinOutputVelocity(shoulderMinVel);
    shoulder_pidController.SetSmartMotionMaxAccel(shoulderMaxAcc);
    shoulder_pidController.SetSmartMotionAllowedClosedLoopError(shoulderAllErr);

    // set PID coefficients and smartmotion values of elbow
    elbow_pidController.SetP(elbowP);
    elbow_pidController.SetI(elbowI);
    elbow_pidController.SetD(elbowD);
    elbow_pidController.SetFF(elbowFF);
    elbow_pidController.SetOutputRange(elbowMinOutput, elbowMaxOutput);
    elbow_pidController.SetSmartMotionMaxVelocity(elbowMaxVel);
    elbow_pidController.SetSmartMotionMinOutputVelocity(elbowMinVel);
    elbow_pidController.SetSmartMotionMaxAccel(elbowMaxAcc);
    elbow_pidController.SetSmartMotionAllowedClosedLoopError(elbowAllErr);
}

void ArmSubsystem::setCone(){
    isConeMode = true;
}

void ArmSubsystem::setCube(){
    isConeMode = false;
}
    

void ArmSubsystem::homePosition(){
    shoulder_pidController.SetReference(0, rev::CANSparkMax::ControlType::kSmartMotion);
    elbow_pidController.SetReference(-1, rev::CANSparkMax::ControlType::kSmartMotion);
};

void ArmSubsystem::floorPickupPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference((-36/360*5), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((-25*60/16/360*90), rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference((-36/360*5), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((-25*60/16/360*95), rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void ArmSubsystem::chutePickupPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference((-36/360*15), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((-25*60/16/360*5), rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference((-36/360*0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((-25*60/16/360*30), rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void ArmSubsystem::trayPickupPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference(-1, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(-2, rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference(-1, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(-2, rev::CANSparkMax::ControlType::kSmartMotion);
    }
};
void ArmSubsystem::bottomDropPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference(-1, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(-2, rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference(-1, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(-2, rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void ArmSubsystem::midDropPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference((-36/360*50), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((-25*60/16/360*130), rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference((-36/360*45), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((-25*60/16/360*130), rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void ArmSubsystem::highDropPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference((-36/360*70), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((-25*60/16/360*150), rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference((-36/360*60), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((-25*60/16/360*150), rev::CANSparkMax::ControlType::kSmartMotion);
    }
};