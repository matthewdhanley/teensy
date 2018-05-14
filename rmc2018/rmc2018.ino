#include "AltEncoder.h"
#include "ros.h"
#include <AccelStepper.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
<<<<<<< HEAD
#include <geometry_msgs/Point.h>
=======
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>

using namespace AltEncoder;

//================================INIT ENCODER OBJECT===========================

// list of encoders
// need one for each motor
// the numbers are pin numbers
Encoder *encoderList[] =
        { //            A  B
                new Encoder(8, 9),               // left wheels
                new Encoder(10, 11),             // right wheels
                new Encoder(24, 25),             // auger
                new Encoder(26, 27),             // linear motion
<<<<<<< HEAD
                new Encoder(28, 29),              // rotation
=======
                new Encoder(28, 29),              // dump linear 1
                new Encoder(30, 31),              // dump linear 2
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
                nullptr
        };


//================================STEPPER STUFF=================================
//This is an example of how you would control 1 stepper
int motorSpeed = 5000; //maximum steps per second (about 3rps / at 16 microsteps)
<<<<<<< HEAD
int motorAccel = 1000; //steps/second/second to accelerate

int motorDirPin = 21; //digital pin 2
int motorStepPin = 20; //digital pin 3
=======
int motorAccel = 3000; //steps/second/second to accelerate

int motorDirPin = 22; //digital pin 2
int motorStepPin = 23; //digital pin 3

>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
int motor_loc = 0;

//set up the accelStepper intance
//the "1" tells it we are using a driver
AccelStepper stepper(1, motorStepPin, motorDirPin); 

//================================GLOBALS=======================================
//MIGHT WANT TO CHANGE:
<<<<<<< HEAD
float motorMax = 100;  // change to adjust maximum drivetrain output!!!!!!!!!!!
int timeout = 1000; // time w/o command before robot goes to safe mode
float angle_throttle = 0.30;
float linear_throttle = 0.20;
float dump_throttle = 0.70;
float auger_throttle = 1.00;
=======
float motorMax = 80;  // change to adjust maximum drivetrain output!!!!!!!!!!!
int timeout = 10000; // time w/o command before robot goes to safe mode
float angle_throttle = 30;
float linear_throttle = 20;
float dump_throttle = 70;

>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db


//Probably don't need to change:
ros::NodeHandle nh;
std_msgs::Int32 measured_angle;
<<<<<<< HEAD

// position messages
std_msgs::Int32 linearPos;
std_msgs::Int32 rotPos;
std_msgs::Int32 augerPos;

=======
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
const int pwm_zero_throttle = 1535;
const float pwm_throttle_scaler = 5.12;
unsigned long lastCommand = 0; //used as a check for failsafe--CLT
volatile bool failsafe = 1; //start with failsafe activated...


//=====ENDSTOP PINS===========
// should probably be using nominally open endstops
int linearBottom = 33;
int linearTop = 34;
<<<<<<< HEAD
int dumpTop = 35;
int backLeft = 36;
int backRight = 37;
=======
int dumpBottom = 35;
int dumpTop = 36;
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db

struct Motor{
    float throttle;
    int pin;
<<<<<<< HEAD
    long pos;
=======
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
};

struct MotorControl{
    Motor left;
    Motor right;
    Motor auger;
    Motor linear;
    Motor dump;
    Motor angular;
};

// global struct for motor information
// THIS IS WHERE THROTTLES AND PINS NUMBERS ARE STORED
MotorControl motors = 
  {
<<<<<<< HEAD
    { 0.0, 2, 0 },
    { 0.0, 3, 0 },
    { 0.0, 4, 0 },
    { 0.0, 5, 0 },
    { 0.0, 6, 0 },
    { 0.0, 7, 0 }
=======
    { 0.0, 2 },
    { 0.0, 3 },
    { 0.0, 4 },
    { 0.0, 5 },
    { 0.0, 6 },
    { 0.0, 7 }
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
  };


//================================DEFINE INTERVAL TIMERS========================
IntervalTimer interruptTimer; //used w/ controlloop

//================================ROS===========================================
void callbackVel(const geometry_msgs::Twist &data)
{
    // if we're going straigt, just make the motors go straight
    if (data.angular.z == 0 && data.linear.x != 0) {
        motors.left.throttle = data.linear.x;
        motors.right.throttle = data.linear.x;
    }
    else if(data.angular.z != 0 && data.linear.x == 0)
    {
        // do a skid turn
        motors.left.throttle = data.angular.z;
        motors.right.throttle = -data.angular.z;
    }
    else if(data.angular.z != 0 && data.linear.x != 0)
    {
        // mix forward and turning
<<<<<<< HEAD
        motors.left.throttle  = (abs(data.linear.z) + data.angular.z)/2;
        motors.right.throttle = (abs(data.linear.z) - data.angular.z)/2;
=======
        motors.left.throttle = data.linear.x + data.angular.z;
        motors.right.throttle = data.linear.x - data.angular.z;
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
    }
    else
    {
        // stop the damn motors.
        motors.left.throttle = 0;
        motors.right.throttle = 0;
    }

    lastCommand = millis(); //remember millis returns an unsigned long.
    //millis checks how long the program has been running.  Pretty dope, eh?
}

void callbackToolRotate(const geometry_msgs::Vector3 &data)
{
  if (data.x != 0){
<<<<<<< HEAD
    motors.angular.throttle = data.x * angle_throttle;
=======
    motors.angular.throttle = angle_throttle*data.x;
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
  }
  else
  {
    motors.angular.throttle = 0;
  }
}

void callbackToolAuger(const geometry_msgs::Vector3 &data)
{
<<<<<<< HEAD
  if (data.x != 0)
  {
    motors.auger.throttle = data.x * auger_throttle;
=======
  if (data.y != 0)
  {
    motors.auger.throttle = data.y;
  }
  
  else if (data.x != 0)
  {
    motors.auger.throttle = -data.x;
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
  }
  else
  {
    motors.auger.throttle = 0;
  }
}

void callbackToolLinear(const geometry_msgs::Vector3 &data)
{
<<<<<<< HEAD
  if (data.x != 0)
  {
    motors.linear.throttle = data.x * linear_throttle;
=======
  if (data.y != 0)
  {
    motors.linear.throttle = linear_throttle;
  }
  
  else if (data.x != 0)
  {
    motors.linear.throttle = -linear_throttle;
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
  }
  else
  {
    motors.linear.throttle = 0;
  }
}

void callbackDump(const geometry_msgs::Vector3 &data)
{
  if (data.x != 0){
<<<<<<< HEAD
    motors.dump.throttle = data.x * dump_throttle;
=======
    motors.dump.throttle = -dump_throttle;
  }
  else if (data.y != 0){
    motors.dump.throttle = dump_throttle;
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
  }
  else
  {
    motors.dump.throttle = 0;
  }
}

void callbackStepper(const std_msgs::Int32 &data)
{
<<<<<<< HEAD
    motor_loc = data.data;
    lastCommand = millis(); //remember millis returns an unsigned long.

=======
    motor_loc = (data.data  * 8.8);
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
}

ros::Subscriber <geometry_msgs::Twist> cmd_vel("robot/cmd_vel", &callbackVel);
ros::Subscriber <geometry_msgs::Vector3> tool_rotate("robot/tool/rotate", &callbackToolRotate);
ros::Subscriber <geometry_msgs::Vector3> tool_auger("robot/tool/auger", &callbackToolAuger);
ros::Subscriber <geometry_msgs::Vector3> tool_linear("robot/tool/linear", &callbackToolLinear);
ros::Subscriber <geometry_msgs::Vector3> dump("robot/dump", &callbackDump);
ros::Subscriber <std_msgs::Int32> stepper_sub("stepper_angle/commanded", &callbackStepper);

ros::Publisher stepper_measured("stepper_angle/measured", &measured_angle);
<<<<<<< HEAD
ros::Publisher LinPos("encoders/linear", &linearPos);
ros::Publisher AugPos("encoders/auger", &augerPos);
ros::Publisher RotPos("encoders/rotation", &rotPos);

=======
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db

//this makes the teensy subscribe to the topic robot/motor_control_serial and sends
//the twist message into the function callbackVel.  piMotorControlSerial.py is
//the program that sends the message.


void setup() {
    //================================ROS=========================================
    nh.initNode(); //initialize the node
    nh.subscribe(cmd_vel); //declare cmd_sub as a subscriber
    nh.subscribe(tool_rotate); // for rotating the tool
    nh.subscribe(tool_auger);
    nh.subscribe(tool_linear);
    nh.subscribe(dump);
    nh.subscribe(stepper_sub);

    nh.advertise(stepper_measured);
<<<<<<< HEAD
    nh.advertise(LinPos);
    nh.advertise(RotPos);
    nh.advertise(AugPos);


=======
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
    
    //================================ENDSTOP PINS================================
    //declare that the endstop pins are inputting data to the teensy.
    pinMode(linearBottom, INPUT); //rotation bottom
    pinMode(linearTop, INPUT); //rotation top
<<<<<<< HEAD
    pinMode(backLeft, INPUT);
    pinMode(backRight, INPUT);
=======
    pinMode(dumpBottom, INPUT); //linear bottom
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
    pinMode(dumpTop, INPUT); //linear top


    //=================================STEPPER MOTOR==============================
    stepper.setMaxSpeed(motorSpeed);
    stepper.setSpeed(motorSpeed);
    stepper.setAcceleration(motorAccel);

    //================================STARTING INTERVAL TIMERS====================
    //this runs the control loop a specified number of times per second.  Be aware
    //that it interupts the running processes and can change variables.  This could
    //cause some weird memory issues, so it could be a location of bugs.
<<<<<<< HEAD
    interruptTimer.begin(controlloop, 50000); // every x microseconds 50000=0.05
=======
    interruptTimer.begin(controlloop, 2000); //go through control loop 100 times second
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db


    //================================ENCODER SAMPLING============================
    Controller::begin(encoderList, 30/*µs*/);  //33khz
    // choose a sampling period which is at least a factor of two smaller than the
    //shortest time between two encoder signal edges.

    //================================SETTING UP MOTORS===========================
    int pwmfreq = 250;   //250hz = 4ms each
    analogWriteFrequency(motors.left.pin, pwmfreq);//drive1
    analogWriteFrequency(motors.right.pin, pwmfreq);//drive2
    analogWriteFrequency(motors.linear.pin, pwmfreq);//drive3
    analogWriteFrequency(motors.auger.pin, pwmfreq); //drive4
    analogWriteFrequency(motors.dump.pin, pwmfreq); //linear
    analogWriteFrequency(motors.angular.pin, pwmfreq); //rotate
    analogWriteResolution(12); // sets resolution to 12 bits instead of default 8
}

//==================================MAIN========================================
void loop() {

    //================================ROS=========================================
    nh.spinOnce();

<<<<<<< HEAD
=======


//    delay(10);
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
}//endmain


void controlloop(void) {
    long checktime = millis() - lastCommand;
    //millis() returns how long the prgram has been running
    //lastCommand is the last time that a command was recieved, relative to the start
    //of the prgram.
    //millis returns unsigned long
    //in the very off chance that we have a strange timing error, we will get overflow
    //and this will result in some strange shit.
    //check if message has been recieved in last xx seconds to activate failsafe
    if (checktime < timeout) {
        failsafe = 0;
    } else {
        failsafe = 1;
    }

    motorWrite(motors);
    if (stepper.distanceToGo() == 0){
        //go the other way the same amount of steps
        //so if current position is 400 steps out, go position -400
        
        stepper.moveTo(motor_loc);
<<<<<<< HEAD
        measured_angle.data = stepper.currentPosition();
=======
        measured_angle.data = stepper.currentPosition() / (3200.0 / 400.0);
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
        stepper_measured.publish( &measured_angle );
    }

    stepper.run();
<<<<<<< HEAD

    linearPos.data = encoderList[3]->counter;
    rotPos.data = encoderList[2]->counter;
    augerPos.data = encoderList[4]->counter;

    LinPos.publish( &linearPos );
    RotPos.publish( &rotPos );
    AugPos.publish( &augerPos );

=======
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
}

void motorWrite(MotorControl motor_struct) {
    //used to convert throttle in percentage
    if (failsafe != 0)
    {
        // STOP ALL THE MOTORS
        analogWrite( motor_struct.left.pin,   pwm_zero_throttle );
        analogWrite( motor_struct.right.pin,  pwm_zero_throttle );
        analogWrite( motor_struct.auger.pin,  pwm_zero_throttle );
        analogWrite( motor_struct.linear.pin, pwm_zero_throttle );
        analogWrite( motor_struct.dump.pin,   pwm_zero_throttle );
        analogWrite( motor_struct.angular.pin,   pwm_zero_throttle );
    }
    else
    {
        // copy struct and convert throttles
        MotorControl motorsConverted = motor_struct;
<<<<<<< HEAD
        // check if the endstops are activated for dumping
        if (!digitalRead(backLeft)){
          motorsConverted.left.throttle   = convertThrottle(-motor_struct.left.throttle);
        }
        else{
          motorsConverted.left.throttle  =    pwm_zero_throttle ;
        }
        if (!digitalRead(backRight)){
          motorsConverted.right.throttle  = convertThrottle(motor_struct.right.throttle);
        }
        else{
          motorsConverted.right.throttle  =    pwm_zero_throttle ;
        }
        
        motorsConverted.auger.throttle  = convertThrottle(motor_struct.auger.throttle);

        // also send this throttle through function to constrain if endstops are hit
        motorsConverted.linear.throttle = convertThrottle(
                                              safeLinearThrottle(motor_struct.linear.throttle)
                                          );
                                          
=======
        motorsConverted.left.throttle   = convertThrottle(-motor_struct.left.throttle);
        motorsConverted.right.throttle  = convertThrottle(motor_struct.right.throttle);
        motorsConverted.auger.throttle  = convertThrottle(motor_struct.auger.throttle);
        motorsConverted.linear.throttle = convertThrottle(motor_struct.linear.throttle);
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
        motorsConverted.dump.throttle   = convertThrottle(motor_struct.dump.throttle);
        motorsConverted.angular.throttle   = convertThrottle(motor_struct.angular.throttle);


        // send them to the motors
        analogWrite(motorsConverted.left.pin,   motorsConverted.left.throttle);
        analogWrite(motorsConverted.right.pin,  motorsConverted.right.throttle);
        analogWrite(motorsConverted.auger.pin,  motorsConverted.auger.throttle);
        analogWrite(motorsConverted.linear.pin, motorsConverted.linear.throttle);
        analogWrite(motorsConverted.dump.pin,   motorsConverted.dump.throttle);
        analogWrite(motorsConverted.angular.pin,   motorsConverted.angular.throttle);

    }
}

// function to convert the throttles to values the ESCs like
float convertThrottle(float throttle){
    float new_throttle = pwm_zero_throttle + // bias
                         constrain( throttle, -motorMax, motorMax) * pwm_throttle_scaler;
    return new_throttle;
}
<<<<<<< HEAD

// function to check for linear endstops being activated. If they are, restrict throttle to zero
float safeLinearThrottle(float throttle){
  if (!digitalRead(linearBottom)){
    constrain(throttle, 0, motorMax)
  }
  if (!digitalRead(linearTop)){
    constrain(throttle, -motorMax, 0)
  }
  return throttle;
}

=======
>>>>>>> 333f0c36f84365e08428e0b7756abc8d844fa9db
