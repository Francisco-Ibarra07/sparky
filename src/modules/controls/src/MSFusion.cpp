#include "controls/MSFusion.hpp"

Movement::Movement(){
    const char* port = "/dev/ttyS0";
    int baudRate = 9600;

    fd = serialOpen(port, baudRate);
    if(fd == -1){
        ROS_FATAL("Serial port failed to open !");
        moveState = false;
        objectsCreated = false;
    }
    else{
        frontL = new Motor(FRONT_L_MOTOR, &fd);
        frontR = new Motor(FRONT_R_MOTOR, &fd);
        rearL = new Motor(REAR_L_MOTOR, &fd);
        rearR = new Motor(REAR_R_MOTOR, &fd);

        frontL_S = new Servo(FRONT_L_SERVO, &fd);
        frontR_S = new Servo(FRONT_R_SERVO, &fd);
        rearL_S = new Servo(REAR_L_SERVO, &fd);
        rearR_S = new Servo(REAR_R_SERVO, &fd);
        
        objectsCreated = true;
        moveState = true;
    }
}

Movement::~Movement(){
    if(objectsCreated){
        delete frontL;
        delete frontR;
        delete rearL;
        delete rearR;
        delete frontL_S;
        delete frontR_S;
        delete rearR_S;
        delete rearL_S;
    }
    serialClose(fd);
    ROS_INFO("Serial closed");
}

bool Movement::isOk(){
    return moveState;
}

void Movement::exitSafeStart(){
    if(frontL->Motor_exitSafeStart() == -1) { moveState = false; return; }
    if(frontR->Motor_exitSafeStart() == -1) { moveState = false; return; }
    if(rearL->Motor_exitSafeStart() == -1) { moveState = false; return; }
    if(rearR->Motor_exitSafeStart() == -1) { moveState = false; return; }
}

void Movement::moveSideways(short speed){
    if(speed > 0)
        moveRight(speed);
    else
        moveLeft(speed);
}

void Movement::moveStraight(short speed){
    if(speed > 0)
        moveForward(speed);
    else
        moveReverse(speed);
}

//No need for a return statement here. This function just stops and disables any motor that is still online
void Movement::stopAndDisable(){
    if(frontL->isOk()) {frontL->Motor_disable();}
    if(frontR->isOk()) {frontR->Motor_disable();}
    if(rearL->isOk()) {rearL->Motor_disable();}
    if(rearR->isOk()) {rearR->Motor_disable();} 
 
    if(frontL->isOk()) {frontL->Motor_stop();}
    if(frontR->isOk()) {frontR->Motor_stop();}
    if(rearL->isOk()) {rearL->Motor_stop();}
    if(rearR->isOk()) {rearR->Motor_stop();}
}

void Movement::moveForward(unsigned short speed){
    if(frontL_S->goHome() == -1) { moveState = false; return; }
    if(frontR_S->goHome() == -1) { moveState = false; return; }
    if(rearL_S->goHome() == -1) { moveState = false; return; }
    if(rearR_S->goHome() == -1) { moveState = false; return; }

    if(frontL->Motor_moveForward(speed) == -1) { moveState = false; return; }
    if(frontR->Motor_moveForward(speed) == -1) { moveState = false; return; }
    if(rearL->Motor_moveForward(speed) == -1) { moveState = false; return; }
    if(rearR->Motor_moveForward(speed) == -1) { moveState = false; return; }
}

void Movement::moveReverse(short speed){
    if(frontL_S->goHome() == -1) { moveState = false; return; }
    if(frontR_S->goHome() == -1) { moveState = false; return; }
    if(rearL_S->goHome() == -1) { moveState = false; return; }
    if(rearR_S->goHome() == -1) { moveState = false; return; }

    if(frontL->Motor_moveReverse(speed) == -1) { moveState = false; return; }
    if(frontR->Motor_moveReverse(speed) == -1) { moveState = false; return; }
    if(rearL->Motor_moveReverse(speed) == -1) { moveState = false; return; }
    if(rearR->Motor_moveReverse(speed) == -1) { moveState = false; return; }
}

void Movement::moveLeft(short speed){
    if(frontL_S->goSideMovementPosition() == -1) { moveState = false; return; }
    if(frontR_S->goSideMovementPosition() == -1) { moveState = false; return; }
    if(rearL_S->goSideMovementPosition() == -1) { moveState = false; return; }
    if(rearR_S->goSideMovementPosition() == -1) { moveState = false; return; }

    if(frontL->Motor_moveReverse(speed) == -1) { moveState = false; return; }
    if(frontR->Motor_moveReverse(speed) == -1) { moveState = false; return; }
    if(rearL->Motor_moveReverse(speed) == -1) { moveState = false; return; }
    if(rearR->Motor_moveReverse(speed) == -1) { moveState = false; return; }
}
void Movement::moveRight(unsigned short speed){
    if(frontL_S->goSideMovementPosition() == -1) { moveState = false; return; }
    if(frontR_S->goSideMovementPosition() == -1) { moveState = false; return; }
    if(rearL_S->goSideMovementPosition() == -1) { moveState = false; return; }
    if(rearR_S->goSideMovementPosition() == -1) { moveState = false; return; }

    if(frontL->Motor_moveForward(speed) == -1) { moveState = false; return; }
    if(frontR->Motor_moveForward(speed) == -1) { moveState = false; return; }
    if(rearL->Motor_moveForward(speed) == -1) { moveState = false; return; }
    if(rearR->Motor_moveForward(speed) == -1) { moveState = false; return; }
}

void Movement::rotateCounterClockwise(short speed){
    if(frontL_S->goSideMovementPosition() == -1) { moveState = false; return; }
    if(frontR_S->goSideMovementPosition() == -1) { moveState = false; return; }
    if(rearL_S->goSideMovementPosition() == -1) { moveState = false; return; }
    if(rearR_S->goSideMovementPosition() == -1) { moveState = false; return; }

    if(frontL->Motor_moveReverse(speed) == -1) { moveState = false; return; }
    if(frontR->Motor_moveReverse(speed) == -1) { moveState = false; return; }

    if(rearL->Motor_moveForward(speed) == -1) { moveState = false; return; }
    if(rearR->Motor_moveForward(speed) == -1) { moveState = false; return; }
}

void Movement::rotateClockwise(short speed){
    if(frontL_S->goSideMovementPosition() == -1) { moveState = false; return; }
    if(frontR_S->goSideMovementPosition() == -1) { moveState = false; return; }
    if(rearL_S->goSideMovementPosition() == -1) { moveState = false; return; }
    if(rearR_S->goSideMovementPosition() == -1) { moveState = false; return; }

    if(frontL->Motor_moveForward(speed) == -1) { moveState = false; return; }
    if(frontR->Motor_moveForward(speed) == -1) { moveState = false; return; }

    if(rearL->Motor_moveReverse(speed) == -1) { moveState = false; return; }
    if(rearR->Motor_moveReverse(speed) == -1) { moveState = false; return; }
}

