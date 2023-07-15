#include "subsystems/ArmSubsystem.h"

using namespace ArmConstants;
using namespace std;

ArmSubsystem::ArmSubsystem() {}

void ArmSubsystem::init(){
    

    // set PID coefficients and smartmotion values of elbow
    
    frc::SmartDashboard::PutNumber("Set Elbow Degrees", 0);
    frc::SmartDashboard::PutNumber("Set Shoulder Degrees", 0);

    

    m_elbow.SetLimits(3_rad_per_s, 10_rad / (1_s * 1_s));
    m_shoulder.SetLimits(3_rad_per_s, 10_rad / (1_s * 1_s));

    moveArm(0.0, 1.0);
}

void ArmSubsystem::setCone(){
    isConeMode = true;
}

void ArmSubsystem::setCube(){
    isConeMode = false;
}

void ArmSubsystem::setElbowFast(){
    m_elbow.SetLimits(3_rad_per_s, 10_rad / (1_s * 1_s));
    m_shoulder.SetLimits(1_rad_per_s, 10_rad / (1_s * 1_s)); 
}

void ArmSubsystem::setElbowSlow(){
    m_elbow.SetLimits(1_rad_per_s, 10_rad / (1_s * 1_s));
    m_shoulder.SetLimits(3_rad_per_s, 10_rad / (1_s * 1_s));
}

void ArmSubsystem::moveArm(double s, double e){
    m_shoulder.SetGoal(units::radian_t{s});
    m_elbow.SetGoal(units::radian_t{e});
    
}

frc2::CommandPtr ArmSubsystem::waitForElbowMove(double e){
    return frc2::cmd::WaitUntil([this, e] { return (this->m_elbow.GetMeasurement().value() < (e+0.174533) 
                        && this->m_elbow.GetMeasurement().value() > (e-0.174533)); });
}

frc2::CommandPtr ArmSubsystem::waitForShoulderMove(double s){
    return frc2::cmd::WaitUntil([this, s] { return (this->m_shoulder.GetMeasurement().value() < (s+0.174533) 
                        && this->m_shoulder.GetMeasurement().value() > (s-0.174533)); });
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
    double encoder_position = this->m_elbow.GetMeasurement().value()/elbowGearRatio;
    return encoder_position;
} 

double ArmSubsystem::getShoulderAngle(){
    double encoder_position = this->m_shoulder.GetMeasurement().value()/shoulderGearRatio;
    return encoder_position;
} 

void ArmSubsystem::getSetStates(){
    frc::SmartDashboard::PutNumber("Elbow Angle", getElbowAngle());
    frc::SmartDashboard::PutNumber("Shoulder Angle", getShoulderAngle());

    desired_elbow_angle = frc::SmartDashboard::GetNumber("Set Elbow Degrees", 70.0);
    desired_shoulder_angle = frc::SmartDashboard::GetNumber("Set Shoulder Degrees", 0.0);
}

frc2::CommandPtr ArmSubsystem::homePosition(){
    return moveArmCommand(0.0, 3.0);
};


frc2::CommandPtr ArmSubsystem::floorPickupPosition(){
    return frc2::ConditionalCommand(moveArmCommand(0, -1.4).Unwrap(), moveArmCommand(-0.0687, -1.407).Unwrap(),
            [this] {return this->isConeMode;} ).ToPtr();
};

frc2::CommandPtr ArmSubsystem::chutePickupPosition(){
    return frc2::ConditionalCommand(moveArmCommand(-0.589, -1.206).Unwrap(), moveArmCommand(-0.216, -1.005).Unwrap(),
            [this] {return this->isConeMode;} ).ToPtr();
};

frc2::CommandPtr ArmSubsystem::trayPickupPosition(){
    return frc2::ConditionalCommand(moveArmCommand(-1.403, -2.788).Unwrap(), moveArmCommand(-0.883, -1.776).Unwrap(),
            [this] {return this->isConeMode;} ).ToPtr();
};
frc2::CommandPtr ArmSubsystem::bottomDropPosition(){
    return frc2::ConditionalCommand(moveArmCommand(0.0, -1.340).Unwrap(), moveArmCommand(0.0, -1.943).Unwrap(),
            [this] {return this->isConeMode;} ).ToPtr();
};

frc2::CommandPtr ArmSubsystem::midDropPosition(){
    return frc2::ConditionalCommand(moveArmCommand(-0.471, -0.630).Unwrap(), moveArmCommand(-0.088, -0.516).Unwrap(),
            [this] {return this->isConeMode;} ).ToPtr();
};

frc2::CommandPtr ArmSubsystem::highDropPosition(){
    return frc2::ConditionalCommand(moveArmCommand(1.237, -2.198).Unwrap(), moveArmCommand(1.237, -2.198).Unwrap(),
            [this] {return this->isConeMode;} ).ToPtr();
};

frc2::CommandPtr ArmSubsystem::testArm(){

    return frc2::cmd::RunOnce(
        [this] { this->moveArm(this->desired_shoulder_angle, this->desired_elbow_angle); }, {this}
    );
}