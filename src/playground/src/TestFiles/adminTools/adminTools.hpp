/* The purpose of this file is to provide admin tools given access by user. 
For now this will be unprotected and always available.*/

#include <iostream>
#include "Motor.hpp"
using namespace std;



class Protection
{

public:
	extern int current_code; // the code needed to access admin tools.
	int code(); //one time use to generate save code that must be transmitted every time a admin command is used.
	bool checker(int code); //checks to see if code matches current code. If so, allows the program to continue.



};

class Tools
{
public:
	string listofCommands[] = {"moveForward", "moveReverse", "setAngle", "setSpeed", "stopMotor"}; //list of all commands, eventually in a file that we can change. Protected file of course.
public:	
	void listOfCommands(int code); //prints all commands available. Checks to see if access is allowed first.
	void moveForwardMotor(Motor* target, int speed, int fd);
	void moveReverseMotor(Motor* target, int speed, int fd);
	void setSpeedMotor(Motor* target, int speed, int fd);


};