/*
    Subscribes to joy stick input from the PS4 controller and publishes directly to /nav/drive_messages.
    
    Publishes:
        /nav/drive_messages
    Subscribes:
        /joy

    Right Analog Stick Indexes:
        Up = [5]
        Down = (negative) [5]

        Right = (negative) [2]
        Left = [2]
    
    Developer notes for PS4 Node:
        -Fix issue of drastic changes in value when we move joystick
        -Add code to send rotation movement messages
*/

#include "ros/ros.h"
#include "controls/ds4Interpreter.hpp" 

DualShock4Interpreter::DualShock4Interpreter(){}
DualShock4Interpreter::~DualShock4Interpreter(){}

void DualShock4Interpreter::DualShock4Callback(const sensor_msgs::Joy::ConstPtr& joyMsgs){
    std_msgs::Int32MultiArray driveMessagePackage;
    int analogPercent;
    int oldXValue;
    int oldZValue;

    //Clear array to start with a fresh array
    driveMessagePackage.data.clear();

    //Forward or Reverse
    if(joyMsgs->axes[5] > 0 || joyMsgs->axes[5] < 0){
        analogPercent = joyMsgs->axes[5] * 100;

        driveMessagePackage.data.push_back(analogPercent);
        driveMessagePackage.data.push_back(0);

        
    }
    //Left or Right
    else if(joyMsgs->axes[2] > 0 || joyMsgs->axes[2] < 0){
        analogPercent= joyMsgs->axes[2] * 100;
        
        driveMessagePackage.data.push_back(0);
        driveMessagePackage.data.push_back(-analogPercent);
    
    }
    // else if(joyMsgs->buttons[]) turn clockwise
    // else if (joyMsgs->buttons[]) turn counter clockwise
    //No movement
    else{
        driveMessagePackage.data.push_back(0);
        driveMessagePackage.data.push_back(0);
    }

    ds4_pub.publish(driveMessagePackage);
}

//Initializes publisher and subscriber
void DualShock4Interpreter::initializeROSComm(){
    ds4_pub = ds4_nh.advertise<std_msgs::Int32MultiArray>("/nav/drive_messages", 10);
    ROS_INFO_ONCE("Drive Messages Pub Set !");
    
    ds4_sub = ds4_nh.subscribe("joy", 100, &DualShock4Interpreter::DualShock4Callback, this);
    ROS_INFO_ONCE("Dual Shock Sub Set !");

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ds4Interpreter");
    ROS_INFO("Main Navigation Node Initialized !");

    DualShock4Interpreter ds4i;
    ds4i.initializeROSComm();

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
}
