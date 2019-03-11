#include "projx_robot_control/simulation_control_driver.hpp"

BaseDriver::BaseDriver(){
    initROSComm();
}

BaseDriver::~BaseDriver() { 
    ROS_INFO("Base Driver finished !"); 
}

void BaseDriver::initROSComm(){

    // Initialize cmd_vel subscriber
    // The publisher is the 'turtlebot_teleop_keyboard' which we remap the topic to "/cmd_vel"
    cmd_vel_sub = nh.subscribe("/cmd_vel", 0, &BaseDriver::velocityCallback, this);
    
    ROS_INFO("Base Driver Intialized");
}

// Dissect Twist message and pass into interpreter
void BaseDriver::velocityCallback(const geometry_msgs::Twist::ConstPtr& msg){

    // Extract Twist message
    x_velocity.data = msg->linear.x;
    z_velocity.data = msg->angular.z;

    // Move robot based on message (only interpreting x and z direction for now)
    interpretCommand(x_velocity, z_velocity);
}

// Interprets x and z velocity commands for now. 
void BaseDriver::interpretCommand(std_msgs::Float64& x_velocity, std_msgs::Float64& z_velocity){

    // If x_vel is not zero, that means we are either moving forward or reverse (ASSUMING z_vel is also 0)
    if(x_velocity.data != 0.0){
        // Set wheels forward
        servo_controller.setToForwardPosition();

        // If x_vel is positive, we are moving forward
        if(x_velocity.data > 0.0)
            motor_controller.moveForward(x_velocity);
        else
            motor_controller.moveReverse(x_velocity);
    }
    // Else if there is action in the Z direction, do a spin
    else if(z_velocity.data != 0.0){

        // Set wheels facing outwards
        servo_controller.setToSidePosition();

        // If z_vel is positive, we are moving forward
        if(z_velocity.data > 0.0)
            motor_controller.moveForward(z_velocity);
        else
            motor_controller.moveReverse(z_velocity);

    }
    // If Z and X are both zero, set robot to neutral position
    else
        robotNeutralPosition();

}

void BaseDriver::robotSpinCounterClockwise(std_msgs::Float64& z_velocity){
    
    // In order to spin counter clockwise, we need to set all wheels to SERVO_SIDE_POSITION first
    servo_controller.setToSidePosition();
    
    // Then we move the FRONT wheels forward
    motor_controller.moveFrontWheels(z_velocity);

    // And the REAR wheels backwards by reversing the value of 'z_velocity' first
    std_msgs::Float64 z_vel_reversed;
    z_vel_reversed.data = z_velocity.data * -1;
    motor_controller.moveRearWheels(z_vel_reversed);
}

void BaseDriver::robotSpinClockwise(std_msgs::Float64& z_velocity){
    //Set wheels to side position
    servo_controller.setToSidePosition();

    // Move REAR wheels forward
    std_msgs::Float64 z_vel_reversed;
    z_vel_reversed.data = z_velocity.data * -1;
    motor_controller.moveRearWheels(z_vel_reversed);

    // Move FRONT wheels reverse
    motor_controller.moveFrontWheels(z_velocity);
}

void BaseDriver::robotNeutralPosition(){
    // Neutral position is having the wheels in forward direction
    servo_controller.setToForwardPosition();          

    // AND the robot wheels completely stopped
    motor_controller.stopAllWheels();
}

int main(int argc, char** argv){

    ros::init(argc, argv, "base_driver_node");
    BaseDriver base_driver_node;

    ros::spin();

    return 0;

}