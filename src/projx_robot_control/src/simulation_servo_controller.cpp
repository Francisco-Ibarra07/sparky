#include "projx_robot_control/simulation_servo_controller.hpp"

// Constructor initializes publishers
ServoController::ServoController(){
    initROSComm();
}

// Empty desctructor
ServoController::~ServoController() {}

// Define publishers for each motor joint
void ServoController::initROSComm(){    
    // Front servos
    FL_servo_pub = nh.advertise<std_msgs::Float64>("/sparky/FL_effort_servo_controller/command", 1);
    FR_servo_pub = nh.advertise<std_msgs::Float64>("/sparky/FR_effort_servo_controller/command", 1);

    // Rear servos
    RL_servo_pub = nh.advertise<std_msgs::Float64>("/sparky/RL_effort_servo_controller/command", 1);
    RR_servo_pub = nh.advertise<std_msgs::Float64>("/sparky/RR_effort_servo_controller/command", 1);

    ROS_INFO("Master SERVO Controller ROS Comm Initialized !");
}

void ServoController::setDirection(std_msgs::Float64 direction){
    
    FL_servo_pub.publish(direction);
    FR_servo_pub.publish(direction);
    RL_servo_pub.publish(direction);
    RR_servo_pub.publish(direction);
}

void ServoController::setToForwardPosition(){
    servo_position.data = FORWARD_POSITION;

    FL_servo_pub.publish(servo_position);
    FR_servo_pub.publish(servo_position);
    RL_servo_pub.publish(servo_position);
    RR_servo_pub.publish(servo_position);
}

void ServoController::setToSidePosition(){
    servo_position.data = SIDE_POSITION;

    FL_servo_pub.publish(servo_position);
    FR_servo_pub.publish(servo_position);
    RL_servo_pub.publish(servo_position);
    RR_servo_pub.publish(servo_position);
}