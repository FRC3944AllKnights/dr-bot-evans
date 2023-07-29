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

    elbow_motor_offset = elbow_pot.Get() - elbow_pot_offset;
    shoulder_motor_offset = shoulder_pot.Get() - shoulder_pot_offset;

    frc::SmartDashboard::PutNumber("Elbow Offset", elbow_motor_offset);
    frc::SmartDashboard::PutNumber("Soulder Offset", shoulder_motor_offset);

    shoulder_pidController.SetIAccum(0.0);
    elbow_pidController.SetIAccum(0.0);

    shoulder_encoder.SetPosition(0.0);
    elbow_encoder.SetPosition(0.0);

    moveArm(0.0, 0.0);
}

void ArmSubsystem::setCone(){
    isConeMode = true;
}

void ArmSubsystem::setCube(){
    isConeMode = false;
}

void ArmSubsystem::setElbowFast(){
    elbow_pidController.SetSmartMotionMaxVelocity(elbowMaxVel);
    shoulder_pidController.SetSmartMotionMaxVelocity(1000);
    elbow_pidController.SetSmartMotionMaxAccel(elbowMaxAcc);  
}

void ArmSubsystem::setElbowSlow(){
    elbow_pidController.SetSmartMotionMaxAccel(1500);
    elbow_pidController.SetSmartMotionMaxVelocity(1000);
    shoulder_pidController.SetSmartMotionMaxVelocity(shoulderMaxVel);
}

void ArmSubsystem::moveArm(double s, double e){
    shoulder_pidController.SetReference((s + shoulder_motor_offset)*shoulderGearRatio, rev::CANSparkMax::ControlType::kSmartMotion);
    elbow_pidController.SetReference((e + elbow_motor_offset)*elbowGearRatio, rev::CANSparkMax::ControlType::kSmartMotion);
}

frc2::CommandPtr ArmSubsystem::moveShoulderCommand(double s){
    return frc2::cmd::RunOnce([this, s] { this->shoulder_pidController.SetReference(s*shoulderGearRatio, rev::CANSparkMax::ControlType::kSmartMotion); }, {this});
}

frc2::CommandPtr ArmSubsystem::moveElbowCommand(double e){
    return frc2::cmd::RunOnce([this, e] { this->elbow_pidController.SetReference(e*elbowGearRatio, rev::CANSparkMax::ControlType::kSmartMotion); }, {this});
}

frc2::CommandPtr ArmSubsystem::waitForElbowMove(double e){
    return frc2::cmd::WaitUntil([this, e] { return (this->elbow_encoder.GetPosition()/this->elbowGearRatio < (e+10) 
                        && this->elbow_encoder.GetPosition()/this->elbowGearRatio > (e-10)); });
}

frc2::CommandPtr ArmSubsystem::waitForShoulderMove(double s){
    return frc2::cmd::WaitUntil([this, s] { return (this->shoulder_encoder.GetPosition()/this->shoulderGearRatio < (s+10) 
                        && this->shoulder_encoder.GetPosition()/this->shoulderGearRatio > (s-10)); });
}

frc2::CommandPtr ArmSubsystem::moveShoulderFirst(double s, double e){
    frc2::CommandPtr moveArm = frc2::cmd::RunOnce(
        [this, e, s] { this->moveArm(s, e); }, {this});

    frc2::CommandPtr setShoulderFirst = frc2::cmd::RunOnce(
        [this] {this->setElbowSlow(); }, {this});

    return std::move(setShoulderFirst).AndThen(std::move(moveArm));
    //return std::move(moveShoulderCommand(s)).AndThen(std::move(waitForShoulderMove(s))).AndThen(std::move(moveElbowCommand(e)));
}

frc2::CommandPtr ArmSubsystem::moveElbowFirst(double s, double e){
    frc2::CommandPtr moveArm = frc2::cmd::RunOnce(
        [this, e, s] { this->moveArm(s, e); }, {this});

    frc2::CommandPtr setElbowFirst = frc2::cmd::RunOnce(
        [this] {this->setElbowFast(); }, {this});

    return std::move(setElbowFirst).AndThen(std::move(moveArm));
    //return std::move(moveElbowCommand(e)).AndThen(std::move(waitForElbowMove(e))).AndThen(std::move(moveShoulderCommand(s)));
}

frc2::CommandPtr ArmSubsystem::moveArmCommand(double s, double e){
    return frc2::ConditionalCommand(moveElbowFirst(s, e).Unwrap(), moveShoulderFirst(s, e).Unwrap(),
        [this, e] {return (e < this->getElbowAngle());} ).ToPtr();
}

double ArmSubsystem::getElbowAngle(){
    double encoder_position = elbow_encoder.GetPosition()/elbowGearRatio;
    return encoder_position;
} 

double ArmSubsystem::getShoulderAngle(){
    double encoder_position = shoulder_encoder.GetPosition()/shoulderGearRatio;
    return encoder_position;
} 

void ArmSubsystem::getSetStates(){
    frc::SmartDashboard::PutNumber("Elbow Angle", getElbowAngle());
    frc::SmartDashboard::PutNumber("Shoulder Angle", getShoulderAngle());
}

frc2::CommandPtr ArmSubsystem::homePosition(){
    return moveArmCommand(0.0, 3.0);
};


frc2::CommandPtr ArmSubsystem::floorPickupPosition(){
    return frc2::ConditionalCommand(moveArmCommand(0.0, 73.0).Unwrap(), moveArmCommand(10.0, 111.5).Unwrap(),
            [this] {return this->isConeMode;} ).ToPtr();
};

frc2::CommandPtr ArmSubsystem::chutePickupPosition(){
    return frc2::ConditionalCommand(moveArmCommand(29.0, 36.0).Unwrap(), moveArmCommand(0.0, 38.0).Unwrap(),
            [this] {return this->isConeMode;} ).ToPtr();
};

frc2::CommandPtr ArmSubsystem::trayPickupPosition(){
    return frc2::ConditionalCommand(moveArmCommand(82.0, 128.0).Unwrap(), moveArmCommand(83.0, 165.0).Unwrap(),
            [this] {return this->isConeMode;} ).ToPtr();
};
frc2::CommandPtr ArmSubsystem::bottomDropPosition(){
    return frc2::ConditionalCommand(moveArmCommand(0.0, 21.0).Unwrap(), moveArmCommand(0.0, 75.0).Unwrap(),
            [this] {return this->isConeMode;} ).ToPtr();
};

frc2::CommandPtr ArmSubsystem::midDropPosition(){
    return frc2::ConditionalCommand(moveArmCommand(45.0, 61.0).Unwrap(), moveArmCommand(10.0, 41.0).Unwrap(),
            [this] {return this->isConeMode;} ).ToPtr();
};

frc2::CommandPtr ArmSubsystem::highDropPosition(){
    return frc2::ConditionalCommand(moveArmCommand(96.0, 129.0).Unwrap(), moveArmCommand(60.0, 93.0).Unwrap(),
            [this] {return this->isConeMode;} ).ToPtr();
};

frc2::CommandPtr ArmSubsystem::testArm(){

    return frc2::cmd::RunOnce(
        [this] { this->moveArm(this->desired_shoulder_angle, this->desired_elbow_angle); }, {this}
    );
}