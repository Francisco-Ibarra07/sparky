#ifndef BASE_DRIVER
#define BASE_DRIVER

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
//NOTE: Need to include the CPP file not the header files
#include "/home/ubuntu/project-x/src/projx_robot_control/src/simulation_servo_controller.cpp" 
#include "/home/ubuntu/project-x/src/projx_robot_control/src/simulation_motor_controller.cpp"

class BaseDriver{

    private:
        ros::NodeHandle nh;
        ros::Subscriber cmd_vel_sub;
        std_msgs::Float64 x_velocity;
        std_msgs::Float64 y_velocity;
        std_msgs::Float64 z_velocity;
        ServoController servo_controller;
        MasterMotorController motor_controller;

        void initROSComm();

        // Dissect Twist message and pass into interpreter
        void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);

        // Interprets x and z velocity commands for now. 
        void interpretCommand(std_msgs::Float64& x_velocity, std_msgs::Float64& z_velocity);
        void robotSpinCounterClockwise(std_msgs::Float64& z_velocity);
        void robotSpinClockwise(std_msgs::Float64& z_velocity);
        void robotNeutralPosition();

    public:
        BaseDriver();
        ~BaseDriver();
};

#endif //BASE_DRIVER
