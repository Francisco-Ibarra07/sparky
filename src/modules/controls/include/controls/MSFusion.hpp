#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <ros/ros.h>
#include "controls/Servo.hpp"
#include "controls/Motor.hpp"

class Movement{

    private:
        Motor* frontL;
        Motor* frontR;
        Motor* rearL;
        Motor* rearR;

        Servo* frontL_S;
        Servo* frontR_S;
        Servo* rearL_S;
        Servo* rearR_S;

        int fd;
        bool objectsCreated;
        bool moveState;

    public:
        Movement();
        ~Movement();
        bool isOk();
        void exitSafeStart();
        void stopAndDisable();
        void moveSideways(short speed);
        void moveStraight(short speed);
        void moveForward(unsigned short speed);
        void moveReverse(short speed);
        void moveLeft(short speed);
        void moveRight(unsigned short speed);
        void rotateClockwise(short speed);
        void rotateCounterClockwise(short speed);
};

#endif //MOVEMENT_H