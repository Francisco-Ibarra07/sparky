#include <iostream>
#include "adminTools.hpp"
#include "Motor.hpp"
using namespace std;


int Protection::code()
{
	current_code = rand();
	return current_code;
}

bool Protection::checker(int code)
{
	if(code == current_code)
	{
		cout << "Access Granted" << endl;
		return true;
	}
	else
	{
		cout << "Access Denied" <<  endl;
		return false;
	}
}

void Tools::listOfCommands(int code)
{
	if(Protection::checker(code) == true)
	{
		for (int i =0; i<listOfCommands.length();i++)
		{
		cout << listOfCommands[i] << ", " << endl;
		}
	}
	
}

void moveForwardMotor(Motor* target, int speed, int fd)
{
	target->moveForward(speed, fd);
}

void moveReverseMotor(Motor* target, int speed, int fd)
{
	target->moveReverse(speed, fd);
}
void setSpeedMotor(Motor* target, int speed, int fd)
{
	target->moveForward(speed, fd);
}