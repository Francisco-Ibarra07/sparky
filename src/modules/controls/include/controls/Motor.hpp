#ifndef MOTOR_H
#define MOTOR_H

#include <ros/ros.h>
#include "wiringSerial.h"
#include <string>
#include <unistd.h>
#include <stdexcept>
#include <stdbool.h>

typedef enum{
	FRONT_L_MOTOR= 	 1,
	FRONT_R_MOTOR=   2,
	REAR_L_MOTOR=    3,
	REAR_R_MOTOR=    4,
} MOTOR;

typedef enum {
    directionNotInitialized, 				 
    FORWARD,							 
    REVERSE			 						
} DIRECTION;				 

class Motor{

	   int* fd;								 //Stores the address of 'fd' so we can use it in Motor.cpp/hpp

	   unsigned char controllerDeviceNumber; //Stores the device number of the motor needed for the pololu
	   unsigned char forwardDirectionID;	 //Stores the forward ID direction of the motor
	   unsigned char reverseDirectionID;	 //Stores the reverse ID direction of the motor

	   bool motorIsSafe;				 //Stores whether an error has occured with this motor		
	   int motorSpeed;					 	 //Stores the speed of the motor. This variable is constantly adjusted.
	   DIRECTION direction;					 //Stores the direction of the motor. This variable is constantly adjusted.
	   std::string motorName;				 //Store the given name when the motor object is created
public:

		//Constructor for a Motor object
		Motor(MOTOR whichMotor, int* _fd);	 //Constructor that takes in the desired motor to initialize and the address of the file descriptor pointer (&fd)
		~Motor();							 //Destructor which prints out the name of the motor object which is being deleted

	   //---MOTOR MOVEMENT FUNCTIONS---//
	   int Motor_exitSafeStart();			 //Function which writes to the microcontroller to get this motor to start. If anything goes wrong it will update the safety variable accordingly
	   int Motor_moveForward(int speed);	 //Takes in a speed and moves motor forward according to that speed
	   int Motor_moveReverse(int speed);	 //Takes in a speed and moves motor in reverse according to that speed
	   int Motor_stop();					 //Stops the motor. 
	   int Motor_disable();				 	 //Disables the motor 

	   //---MOTOR GETTERS---//
	   int getcontrollerDeviceNumber();		 //Returns either 1, 2, 3, or 4 depending on the device number of the motor
	   int getMotorSpeed();					 //Returns the speed of the motor
	   bool isOk();	 						 //Returns whether or not the motor is error free
	   DIRECTION getDirection();			 //Returns the direction in which the motor is moving (Either forward or reverse)
	   std::string getName();				 //Returns the name of the motor
	   float getTemperature();				 //Returns the temperature of the motor
		
	   //---MOTOR SETTERS---//
	   void setDirectionVar(DIRECTION newDirection); //Sets the new direction of the motor
	   void setSpeedVar(int newSpeed); 				 //Sets the new speed and the direction of the motor

};


Motor::Motor(MOTOR whichMotor, int* _fd) 
{ 
		//Assigns device number, name, forward and reverse ID depending on motor chosen
		switch(whichMotor)
		{
			case FRONT_L_MOTOR:
				controllerDeviceNumber= 0x01;
				motorName= "Front Left Motor";
				forwardDirectionID= 0x06;
				reverseDirectionID= 0x05;
			break;

			case FRONT_R_MOTOR:
				controllerDeviceNumber= 0x02;
				motorName= "Front Right Motor";
				forwardDirectionID = 0x05;
				reverseDirectionID = 0x06;
			break;

			case REAR_L_MOTOR:
				controllerDeviceNumber= 0x03;
				motorName= "Rear Left Motor";
				forwardDirectionID = 0x06;
				reverseDirectionID = 0x05;
			break;

			case REAR_R_MOTOR:
				controllerDeviceNumber= 0x04;
				motorName= "Rear Right Motor";
				forwardDirectionID = 0x05;
				reverseDirectionID = 0x06;
			break;

			default: //This case shouldn't happen because 'whichMotor' should always be a MOTOR enum value but just in case throw an error
				ROS_FATAL("Illegal Argument: 'whichMotor' needs to be a valid MOTOR");
				serialClose(*_fd); 
			break;
		}

	//Initialize the rest of the variables
	fd= _fd;
	motorSpeed= 0;	
	motorIsSafe = true;
	direction = directionNotInitialized;

}

Motor::~Motor()
{
	ROS_INFO("Controller #%d deleted !", controllerDeviceNumber);
}

bool Motor::isOk(){
	return motorIsSafe;
}

int Motor::Motor_exitSafeStart()
{
	int exitSafeStart_command[3] = {0xAA, this->controllerDeviceNumber, 0x83};
	int check;
	check = write(*fd, exitSafeStart_command, 3);
	if(check == -1){
		ROS_FATAL("Controller #%d failed to exitSafeStart()", controllerDeviceNumber);
		motorIsSafe = false;
		return -1;
	}
	else{
		ROS_INFO("Controller #%d set !", controllerDeviceNumber);
		return 0;
	}
}

//---------------MOTOR MOVEMENT FUNCTIONS------------//

int Motor::Motor_moveForward(int newSpeed)
{
	if(newSpeed <= 0){
		newSpeed = 0;
	} 
	else if (newSpeed >= 100){
		newSpeed = 100;
	}

    int moveForward_command[5] = {0xAA, this->controllerDeviceNumber, forwardDirectionID, 0x00, newSpeed};
	int check = write(*fd, moveForward_command, 5);

	if(check == -1){
		ROS_FATAL("Controller #%d failed to moveForward()", controllerDeviceNumber);
		motorIsSafe = false;
		return -1;
	}
	else{
		setDirectionVar(FORWARD);
		setSpeedVar(newSpeed);
		return 0;
	}
}

int Motor::Motor_moveReverse(int newSpeed)
{
	if(newSpeed >= 0){
		newSpeed = 0;
	} 
	else if (newSpeed < -100){
		newSpeed = -100;
	}
   
    int moveReverse_command[5] = {0xAA, this->controllerDeviceNumber, reverseDirectionID, 0x00, newSpeed};
	int check = write(*fd, moveReverse_command, 5);

	if(check == -1){
		ROS_FATAL("Controller #%d failed to moveReverse()", controllerDeviceNumber);
		motorIsSafe = false;
		return -1;
	} 
	else{
		setDirectionVar(REVERSE);
		setSpeedVar(newSpeed);
		return 0;
	}
}

int Motor::Motor_disable()
{
	int disable_command[3] = {0xAA, this->controllerDeviceNumber, 0x60};
	int check=0;
	check = write(*fd, disable_command, 3);
	if(check == -1){
		ROS_FATAL("Controller #%d failed to stop",controllerDeviceNumber);
		motorIsSafe = false;
		return -1;
	}
	else{
		ROS_INFO("Controller #%d disabled", controllerDeviceNumber);
		return 0;
	}
}

int Motor::Motor_stop()
{
	int stop_command[5] = {0xAA, controllerDeviceNumber, reverseDirectionID, 0x00, 0x00};
	int check = write(*fd, stop_command, 5);

	if(check == -1){
		ROS_FATAL("Controller #%d failed to stop",controllerDeviceNumber);
		motorIsSafe = false;
		return -1;
	}
	else{
		ROS_INFO("Disable safestart on motor %d first", controllerDeviceNumber);
		return 0;
	}
}

//MOTOR GETTERS
int Motor::getcontrollerDeviceNumber(){
	return this->controllerDeviceNumber;
}

int Motor::getMotorSpeed(){
	return this->motorSpeed;
}

std::string Motor::getName(){
	return this->motorName;
}

DIRECTION Motor::getDirection(){
	return this->direction;
}

//Temperature returned is in units of 0.1 degrees Celsius
//For example if 295 is returned then the temp is 29.5 *Celsius
float Motor::getTemperature(){
	int getTemp[4]= {0xAA, 0x02, 0x21, 24};
	char lowByte, highByte;
	write(*fd, getTemp, 4);
	lowByte= serialGetchar(*fd);
	highByte= serialGetchar(*fd);

	return (lowByte + 256 * highByte) * 0.1;
}

//-------------MOTOR SETTERS--------------//

void Motor::setSpeedVar(int newSpeed){
	this->motorSpeed = newSpeed;
}

void Motor::setDirectionVar(DIRECTION newDirection){
	direction= newDirection;
}


#endif //MOTOR_H

