#include "subsystems/ArmSubsystem.h"

using namespace ArmConstants;
using namespace std;

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

frc2::CommandPtr ArmSubsystem::moveShoulderCommand(double desired_angle){
    return frc2::cmd::RunOnce([this, desired_angle] { this->shoulder_pidController.SetReference(desired_angle*shoulderGearRatio, rev::CANSparkMax::ControlType::kSmartMotion); }, {this});
}

frc2::CommandPtr ArmSubsystem::moveElbowCommand(double desired_angle){
    return frc2::cmd::RunOnce([this, desired_angle] { this->elbow_pidController.SetReference(desired_angle*elbowGearRatio, rev::CANSparkMax::ControlType::kSmartMotion); }, {this});
}

frc2::CommandPtr ArmSubsystem::waitForElbowMove(double desired_angle){
    return frc2::cmd::WaitUntil([this, desired_angle] { return (this->elbow_encoder.GetPosition()/this->elbowGearRatio < (desired_angle+3) 
                        && this->elbow_encoder.GetPosition()/this->elbowGearRatio > (desired_angle-3)); });
}

frc2::CommandPtr ArmSubsystem::waitForShoulderMove(double desired_angle){
    return frc2::cmd::WaitUntil([this, desired_angle] { return (this->shoulder_encoder.GetPosition()/this->shoulderGearRatio < (desired_angle+3) 
                        && this->shoulder_encoder.GetPosition()/this->shoulderGearRatio > (desired_angle-3)); });
}

frc2::CommandPtr ArmSubsystem::moveArmCommand(double shoulder_angle, double elbow_angle){
    if (elbow_angle*elbowGearRatio > elbow_encoder.GetPosition()){
        return std::move(moveShoulderCommand(shoulder_angle)).AndThen(std::move(waitForShoulderMove(shoulder_angle))).AndThen(std::move(moveElbowCommand(elbow_angle)));
    } else{
        return std::move(moveElbowCommand(elbow_angle)).AndThen(std::move(waitForElbowMove(elbow_angle))).AndThen(std::move(moveShoulderCommand(shoulder_angle)));
    }
}
    

frc2::CommandPtr ArmSubsystem::homePosition(){
    return moveArmCommand(0.0, 1.0);
};

frc2::CommandPtr ArmSubsystem::floorPickupPosition(){
    if(isConeMode){
        return moveArmCommand(0.0, 75.0);
    } else{
        return moveArmCommand(10.0, 111.5);
    }
};

frc2::CommandPtr ArmSubsystem::chutePickupPosition(){
    if(isConeMode){
        return moveArmCommand(29.0, 36.0);
    } else{
        return moveArmCommand(0.0, 38.0);
    }
};

frc2::CommandPtr ArmSubsystem::trayPickupPosition(){
    if(isConeMode){
        return moveArmCommand(82.0, 130.0);
    } else{
        return moveArmCommand(83.0, 165.0);
    }
};
frc2::CommandPtr ArmSubsystem::bottomDropPosition(){
    if(isConeMode){
        return moveArmCommand(0.0, 21.0);
    } else{
        return moveArmCommand(0.0, 75.0);
    }
};

frc2::CommandPtr ArmSubsystem::midDropPosition(){
    if(isConeMode){
        return moveArmCommand(45.0, 61.0);
    } else{
        return moveArmCommand(10.0, 41.0);
    }
};

frc2::CommandPtr ArmSubsystem::highDropPosition(){
    if(isConeMode){
        return moveArmCommand(90.0, 130.0);
    } else{
        return moveArmCommand(60.0, 93.0);
    }
};

frc2::CommandPtr ArmSubsystem::testArm(){
    double elbowSetPoint = frc::SmartDashboard::GetNumber("Set Elbow Degrees", 0);
    double shoulderSetPoint = frc::SmartDashboard::GetNumber("Set Shoulder Degrees", 0);

    return moveArmCommand(shoulderSetPoint, elbowSetPoint);
}