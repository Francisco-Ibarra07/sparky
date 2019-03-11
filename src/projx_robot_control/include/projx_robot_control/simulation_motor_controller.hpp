#ifndef MASTER_MOTOR_CONTROLLER
#define MASTER_MOTOR_CONTROLLER

// Master motor controller
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>

class MasterMotorController{

    private:
        ros::NodeHandle nh;
        ros::Publisher FL_motor_pub;
        ros::Publisher FR_motor_pub;
        ros::Publisher RL_motor_pub;
        ros::Publisher RR_motor_pub;
        std_msgs::Float64 motor_speed;
        
        void initROSComm();

    public:
        MasterMotorController();
        ~MasterMotorController();
        void moveForward(std_msgs::Float64 target);
        void moveReverse(std_msgs::Float64 target);
        void stopAllWheels();
        void moveRearWheels(std_msgs::Float64 target);
        void moveFrontWheels(std_msgs::Float64 target);
};

#endif // MASTER_MOTOR_CONTROLLER