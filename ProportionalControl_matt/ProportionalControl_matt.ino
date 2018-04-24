//This is an example of how you would control 1 stepper

#include <AccelStepper.h>

int motorSpeed = 1000; //maximum steps per second (about 3rps / at 16 microsteps)
int motorAccel = 8000; //steps/second/second to accelerate

int motorDirPin = 22; //digital pin 2
int motorStepPin = 23; //digital pin 3

int motor_loc = 0;

//set up the accelStepper intance
//the "1" tells it we are using a driver
AccelStepper stepper(1, motorStepPin, motorDirPin); 



void setup(){
  stepper.setMaxSpeed(motorSpeed);
  stepper.setSpeed(motorSpeed);
  stepper.setAcceleration(motorAccel);
  
//  stepper.moveTo(32000); //move 32000 steps (should be 10 rev)
                           //actually looks like about 1 rev
}

void loop(){
  //if stepper is at desired location
  if (Serial.available()){
    motor_loc = Serial.parseInt()*8.888888888888;
    Serial.print(motor_loc / 8.888888888888);
    Serial.print('\n');
  }
  
//  if (stepper.distanceToGo() == 0){
    //go the other way the same amount of steps
    //so if current position is 400 steps out, go position -400
    stepper.moveTo(motor_loc); 
//  }
  

  
  //these must be called as often as possible to ensure smooth operation
  //any delay will cause jerky motion
  stepper.run();
}

