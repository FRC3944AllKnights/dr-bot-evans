#include "subsystems/ArmSubsystem.h"

ArmSubsystem::ArmSubsystem() {}

void ArmSubsystem::init(){

}

void ArmSubsystem::moveFirstJoint(double motor_power){
    first_joint_motor.Set(motor_power);
}

void ArmSubsystem::moveSecondJoint(double motor_power){
    second_joint_motor.Set(motor_power);
}