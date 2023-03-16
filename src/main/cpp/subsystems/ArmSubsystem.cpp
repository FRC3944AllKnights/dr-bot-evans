#include "subsystems/ArmSubsystem.h"

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

void ArmSubsystem::moveFirstJoint(double motor_power){
    shoulder_motor.Set(motor_power);
}

void ArmSubsystem::moveSecondJoint(double motor_power){
    elbow_motor.Set(motor_power);
}

void ArmSubsystem::homePosition(){
    shoulder_pidController.SetReference(0, rev::CANSparkMax::ControlType::kSmartMotion);
    elbow_pidController.SetReference(0, rev::CANSparkMax::ControlType::kSmartMotion);
};

void floorPickupPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference(shoulderGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference(shoulderGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*0.5, rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void chutePickupPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference(shoulderGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference(shoulderGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*0.5, rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void trayPickupPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference(shoulderGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference(shoulderGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*0.5, rev::CANSparkMax::ControlType::kSmartMotion);
    }
};
void bottomDropPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference(shoulderGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference(shoulderGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*0.5, rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void midDropPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference(shoulderGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference(shoulderGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*0.5, rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void highDropPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference(shoulderGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference(shoulderGearRatio*0.2, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*0.5, rev::CANSparkMax::ControlType::kSmartMotion);
    }
};