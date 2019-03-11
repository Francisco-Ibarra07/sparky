#include "projx_robot_control/simulation_motor_controller.hpp"

// Constructor initializes publishers
MasterMotorController::MasterMotorController(){
    initROSComm();
}

// Empty desctructor
MasterMotorController::~MasterMotorController() {}

// Define publishers for each motor joint
void MasterMotorController::initROSComm(){    
   
    // Front wheels
    FL_motor_pub = nh.advertise<std_msgs::Float64>("/sparky/FL_effort_motor_controller/command", 1);
    FR_motor_pub = nh.advertise<std_msgs::Float64>("/sparky/FR_effort_motor_controller/command", 1);

    // Rear wheels
    RL_motor_pub = nh.advertise<std_msgs::Float64>("/sparky/RL_effort_motor_controller/command", 1);
    RR_motor_pub = nh.advertise<std_msgs::Float64>("/sparky/RR_effort_motor_controller/command", 1);

    ROS_INFO("Master MOTOR Controller ROS Comm Initialized !");
}

// Public move forward command that moves all wheels at the same speed
void MasterMotorController::moveForward(std_msgs::Float64 target){
    
    // Send all motors the same forward speed command
    FL_motor_pub.publish(target);
    FR_motor_pub.publish(target);
    RL_motor_pub.publish(target);
    RR_motor_pub.publish(target);

}

void MasterMotorController::moveReverse(std_msgs::Float64 target){
    
    // Send all motors the same reverse speed command
    FL_motor_pub.publish(target);
    FR_motor_pub.publish(target);
    RL_motor_pub.publish(target);
    RR_motor_pub.publish(target);
}

void MasterMotorController::stopAllWheels(){
    motor_speed.data = 0.0;

    FL_motor_pub.publish(motor_speed);
    FR_motor_pub.publish(motor_speed);
    RL_motor_pub.publish(motor_speed);
    RR_motor_pub.publish(motor_speed);
}

void MasterMotorController::moveFrontWheels(std_msgs::Float64 target){
    FL_motor_pub.publish(target);
    FR_motor_pub.publish(target);
}

void MasterMotorController::moveRearWheels(std_msgs::Float64 target){
    RL_motor_pub.publish(target);
    RR_motor_pub.publish(target);
}
