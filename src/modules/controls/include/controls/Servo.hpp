//Data sheet for servo controller: https://www.pololu.com/docs/pdf/0J40/maestro.pdf

#ifndef SERVO_H
#define SERVO_H

#include <ros/ros.h>
#include <wiringSerial.h>
#include <string>
#include <unistd.h>
#include <stdexcept>
#include <stdbool.h>

typedef enum {
    FRONT_L_SERVO=  1,
    FRONT_R_SERVO=  2,
    REAR_L_SERVO=   3,
    REAR_R_SERVO=   4,
} SERVO;


class Servo{

    private:
        int *fd;

        std::string servoName;
        unsigned char channelNumber;
        unsigned char controllerDeviceNumber;

        int servoPosition;
        const unsigned short HOME_POSITION = 0;
        const unsigned short SIDE_MOVEMENT_POSITION = 0;
        
    public:
        Servo(SERVO whichServo, int* _fd);
        ~Servo();

        int goHome();
        int goSideMovementPosition();
        int setTarget(unsigned short target);
        int getPosition();
        int getMovingState();
        void setPWM();
        void setAcceleration(int target);
        int setSpeed(int targetSpeed);
    
};

Servo::Servo(SERVO whichServo, int* _fd){
    switch(whichServo)
    {
        case FRONT_L_SERVO:
				controllerDeviceNumber= 0x01;
				servoName= "Front Left Servo";
                channelNumber= 0x00;
			break;

			case FRONT_R_SERVO:
				controllerDeviceNumber= 0x02;
				servoName= "Front Right Servo";
                channelNumber= 0x01;
			break;

			case REAR_L_SERVO:
				controllerDeviceNumber= 0x03;
				servoName= "Rear Left Servo";
                channelNumber= 0x02;
			break;

			case REAR_R_SERVO:
				controllerDeviceNumber= 0x04;
				servoName= "Rear Right Servo";
                channelNumber= 0x03;
			break;

			default: //This case shouldn't happen because 'whichServo' should always be a SERVO enum value but, just in case, throw an error
				ROS_FATAL("Illegal Argument: 'whichServo' needs to be a valid SERVO");
				serialClose(*_fd);
                channelNumber= -1; 
			break;
    }

    fd= _fd;
}

Servo::~Servo(){
    ROS_INFO("Servo channel #%d deleted !", channelNumber);
}

int Servo::goHome(){
    if(servoPosition != HOME_POSITION){
        return setTarget(HOME_POSITION);
    }
    else{
        return 0;
    }
}

int Servo::goSideMovementPosition(){
    if(servoPosition != SIDE_MOVEMENT_POSITION){
        return setTarget(SIDE_MOVEMENT_POSITION);
    }
    else {
        return 0;
    }
}

//TODO: Test servo signals
int Servo::setTarget(unsigned short target){
    int setTarget_command[4] = {0x84, channelNumber, target & 0x7F, 0}; //target >> 7 & 0x7F};
    int check = write(*fd, setTarget_command, 4);
    if (check == -1){
        ROS_FATAL("Servo number: %d failed to setTarget()", channelNumber);
        return -1;
    } 
    else{
        servoPosition = target;
        return 0;
    }
}

int Servo::getPosition(){
    unsigned char getPosition_command[2] = {0x90, channelNumber};
    int check = write(*fd, getPosition_command, 2);
    if (check == -1){
        ROS_FATAL("Servo number: %d failed to getPosition()", channelNumber);
        return -9999;
    } 
    else{
        unsigned char pololuResponse[2];
        pololuResponse[0] = serialGetchar(*fd);
        pololuResponse[1] = serialGetchar(*fd);
        servoPosition = pololuResponse[0] + 256 * pololuResponse[1];

        return servoPosition;
    }
}

int Servo::getMovingState(){}
void Servo::setPWM(){}
void Servo::setAcceleration(int target){}
int Servo::setSpeed(int targetSpeed){}

#endif //SERVO_H