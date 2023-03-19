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

    frc::SmartDashboard::PutNumber("Set Elbow Degrees", 0);
    frc::SmartDashboard::PutNumber("Set Shoulder Degrees", 0);
}

void ArmSubsystem::setCone(){
    isConeMode = true;
}

void ArmSubsystem::setCube(){
    isConeMode = false;
}
    

void ArmSubsystem::homePosition(){
    shoulder_pidController.SetReference(0.0, rev::CANSparkMax::ControlType::kSmartMotion);
    elbow_pidController.SetReference(0.0, rev::CANSparkMax::ControlType::kSmartMotion);
};

void ArmSubsystem::floorPickupPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference((shoulderGearRatio*0.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*73.0), rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference((shoulderGearRatio*10.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*111.5), rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void ArmSubsystem::chutePickupPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference((shoulderGearRatio*29.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*36.0), rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference((shoulderGearRatio*0.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*38.0), rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void ArmSubsystem::trayPickupPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference(shoulderGearRatio*82.0, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*130, rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference(shoulderGearRatio*83.0, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*165.0, rev::CANSparkMax::ControlType::kSmartMotion);
    }
};
void ArmSubsystem::bottomDropPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference(shoulderGearRatio*0.0, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*21.0, rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference(shoulderGearRatio*0.0, rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference(elbowGearRatio*75.0, rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void ArmSubsystem::midDropPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference((shoulderGearRatio*50.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*61), rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference((shoulderGearRatio*10.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*41.0), rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void ArmSubsystem::highDropPosition(){
    if(isConeMode){
        shoulder_pidController.SetReference((shoulderGearRatio*95.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*196.0), rev::CANSparkMax::ControlType::kSmartMotion);
    } else{
        shoulder_pidController.SetReference((shoulderGearRatio*65.0), rev::CANSparkMax::ControlType::kSmartMotion);
        elbow_pidController.SetReference((elbowGearRatio*151.0), rev::CANSparkMax::ControlType::kSmartMotion);
    }
};

void ArmSubsystem::testArm(){
    double elbowSetPoint = frc::SmartDashboard::GetNumber("Set Elbow Degrees", 0);
    double shoulderSetPoint = frc::SmartDashboard::GetNumber("Set Shoulder Degrees", 0);

    shoulder_pidController.SetReference((shoulderGearRatio*shoulderSetPoint), rev::CANSparkMax::ControlType::kSmartMotion);
    elbow_pidController.SetReference((elbowGearRatio*elbowSetPoint), rev::CANSparkMax::ControlType::kSmartMotion);
}