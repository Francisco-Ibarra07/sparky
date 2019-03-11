/*
    This node will be ran on the Rasbperry Pi and is responsible for controlling all the motors and servos. Based on the subscribed navigation messages,
    this node uses the 'moveManager' to command all motors and servos to move. 'moveManager' will also attempt to recover a motor or servo if any of them go down.
    If it can't, 'moveManager' will clean up and shut the system down.
    
    Publishes:
        N/A
    Subscribes:
        /nav/drive_messages

    Developer notes for RPI Node:
        -Servo commands still need to be validated (setTarget(), getPosition())
        -Connect RPI to master roscore
        -It seems that ros::spin() does not make main loop back. Therefore moveManager's "isOk()" function only works on startup. (10/8/18 - Added a check at the top of the callback function. Need to test if it works)
        -Uncomment moveManager->exitSafeStart() when ready for motor testing
*/

#include <ros/ros.h>
#include <signal.h>
#include "MSFusion.cpp"
#include "std_msgs/Int32MultiArray.h"

Movement* moveManager;
void driveMessagesCallback(const std_msgs::Int32MultiArray::ConstPtr& driveMessagePackage);
void mySigintHandler(int sig);

int main(int argc, char **argv)
{
    //Initialize node_RaspberryPi with the option to create my own shutdown steps
    ros::init(argc, argv, "node_RaspberryPi", ros::init_options::NoSigintHandler);
    ROS_INFO("Rasbperry Pi Node Initialized !");
    ros::NodeHandle nh;
    ros::Subscriber driveMessages;

    //Creation of 'Movement' object and a quick check to see if all the motors, servos, and serial ports were initialized correctly. If not, shutdown.
    moveManager = new Movement();
    if(!moveManager->isOk()){ delete moveManager; return -1; }

    //Initialize motors
    //moveManager->exitSafeStart();
    if(!moveManager->isOk()){ delete moveManager; return -1; }
    ROS_INFO("Motors Initialized !");
    
    //Initialize subscription to /nav/drive_messages
    driveMessages = nh.subscribe("/nav/drive_messages", 100, driveMessagesCallback);
    ROS_INFO_ONCE("Raspberry Pi Subscribed to /nav/drive_messages Set !");
    
    signal(SIGINT, mySigintHandler);
    ros::spin();

    return 0;
}

/* 
    When ros gets the message to shutdown (Ctrl-C), this function allows this node to do additional steps before fully shutting down.
    In this case, mySigintHandle() is useful for the raspberry pi because it allows us to call 'delete moveManager', which cleans up the pi
    before shutting down. Examples of cleaning up: closing serial port (ttyS0), freeing motor and servo objects, logging exit messages,
    and stopping all motor movement.
        @param sig - Takes in the signal for shutdown. More info here: http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown#Custom_SIGINT_Handler
 */
void mySigintHandler(int sig){
    ROS_INFO("Exiting");
    moveManager->stopAndDisable();
    delete moveManager;
    ros::shutdown();
}

/* 
    This is the callback function for incoming /nav/drive_messages. 'node_RasbperryPi' is subscribed to these drive messages and uses the
    moveManager object to move the robot accordingly.
        @param driveMessagePackage - Is the array that contains the specified drive messages for the robot
 */
void driveMessagesCallback(const std_msgs::Int32MultiArray::ConstPtr& driveMessagePackage){

    if(!moveManager->isOk()){
        moveManager->stopAndDisable();
        delete moveManager;
        ros::shutdown();
    }

    short x_drive = driveMessagePackage->data.at(0);
    short z_drive = driveMessagePackage->data.at(1);

    if(x_drive > 0 || x_drive < 0){
        moveManager->moveStraight(x_drive);
    }
    else if(z_drive > 0 || z_drive < 0){
        moveManager->moveSideways(z_drive);
    }
}
