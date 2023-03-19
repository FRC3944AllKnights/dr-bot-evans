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
        shoulder_pidController.SetReference((shoulderGearRatio*0.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*65.0), rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference((shoulderGearRatio*10.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*102.5), rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void ArmSubsystem::chutePickupPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference((shoulderGearRatio*35.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*30.0), rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference((shoulderGearRatio*0.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*27.0), rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void ArmSubsystem::trayPickupPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference(shoulderGearRatio*75.0, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*110.0, rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference(shoulderGearRatio*75.0, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*125.0, rev::CANSparkMax::ControlType::kSmartMotion);
    }
};
void ArmSubsystem::bottomDropPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference(shoulderGearRatio*0.0, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*80.0, rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference(shoulderGearRatio*0.0, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*80.0, rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void ArmSubsystem::midDropPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference((shoulderGearRatio*50.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*130), rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference((shoulderGearRatio*45.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*130.0), rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void ArmSubsystem::highDropPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference((shoulderGearRatio*70.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((shoulderGearRatio*150.0), rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference((shoulderGearRatio*60.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((shoulderGearRatio*150.0), rev::CANSparkMax::ControlType::kSmartMotion);
    }
};