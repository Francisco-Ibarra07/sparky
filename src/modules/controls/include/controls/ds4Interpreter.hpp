/*
    Subscribes to joy stick input from the PS4 controller and publishes directly to /nav/drive_messages.
    
    Publishes:
        /nav/drive_messages
    Subscribes:
        /joy
*/
#ifndef DS4_H
#define DS4_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32MultiArray.h"

class DualShock4Interpreter{
    private:
        ros::Subscriber ds4_sub;
        ros::Publisher  ds4_pub;
        ros::NodeHandle ds4_nh;
    
        void DualShock4Callback(const sensor_msgs::Joy::ConstPtr& joyMsgs);
    
    public:
        DualShock4Interpreter();
        ~DualShock4Interpreter();
        void initializeROSComm();
        
};

#endif //DS4_H