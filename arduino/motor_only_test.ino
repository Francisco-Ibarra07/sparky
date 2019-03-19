#include <SoftwareSerial.h>

SoftwareSerial pololu = SoftwareSerial(10,11);

void exitSafeStart(){
  pololu.write(0x83);
}

void setSpeed(int speed){
  if (speed < 0)
  {
    pololu.write(0x86);  // motor reverse command
    speed = -speed;  // make speed positive
  }
  else
  {
    pololu.write(0x85);  // motor forward command
  }
  pololu.write(speed & 0x1F);
  pololu.write(speed >> 5);
}


void setup() {
 pololu.begin(9600);
 delay(5);
 pololu.write(0xAA);
 exitSafeStart();
}

void loop() {
  setSpeed(800);
  delay(3000);

  setSpeed(-800);
  delay(3000);
}
