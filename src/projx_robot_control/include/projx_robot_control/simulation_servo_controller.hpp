#ifndef SERVO_CONTROLLER
#define SERVO_CONTROLLER

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>

class ServoController{

    private:
        ros::NodeHandle nh;
        ros::Publisher FL_servo_pub;
        ros::Publisher FR_servo_pub;
        ros::Publisher RL_servo_pub;
        ros::Publisher RR_servo_pub;

        std_msgs::Float64 servo_position;
        float SIDE_POSITION = 1.0;
        float FORWARD_POSITION = -1.0;

        void initROSComm();

    public:
        ServoController();
        ~ServoController();
        void setDirection(std_msgs::Float64 direction);
        void setToForwardPosition();
        void setToSidePosition();
};

#endif // SERVO_CONTROLLER