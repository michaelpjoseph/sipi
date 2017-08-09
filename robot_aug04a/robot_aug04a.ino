// Hurley / SIPI IT Robot
//    Mike Joseph - mjoseph1@hurleymc.com - 2017.Aug.04

// Parts of this code utilize portions ofthe "Simple Motor Shield sketch" by arduino.cc user "Krodal" dated June 2012 which is Public Domain.  
// That work was derivative of the AdaFruit MotorShield library, which is also Public Domain.

//  Basic command examples:
//    valid speed ranges:  0 - 255
//
//    WHOLE ROBOT CONTROL
//    +  m_moveForward(speed)   -   moves the robot forward
//    +  m_moveBackward(speed)  -   moves the robot backward
//    +  m_spinLeft(speed)      -   spins the robot left by moving right wheel forward and left wheel reversed
//    +  m_spinRight(speed)     -   spins the robot right by moving the left wheen forward and right wheel reversed
//    +  m_stop()               -   stops the robot.
//
//    SINGLE WHEEL CONTROL
//    motor definitions:
//      1 - left motor
//      2 - right motor
//    motor commands:
//      FORWARD
//      BACKWARD
//      RELEASE
//    +  motor(1, BACKWARD, speed)  -   moves the left motor in reverse at a specified speed
//    +  motor(2, FORWARD, speed)   -   moves the right motor forward at speed
//    +  motor(2, RELEASE)          -   stops the right motor


//***  DEFINITIONS AND CONFIGURATION - Ignore this and skip down to setup()
//  *  This portion of the code defines the communication pins and codes used between the Arduino and the SainSmart 293d shield used for motor and Servo control.  
//  *  It also defines the Servo object used to control the 'head' of the robot. It should never need to be changed unless new sensors or motors are added to the
//  *  design.
#include <Servo.h>

// Arduino pins for the shift register
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

// 8-bit bus after the 74HC595 shift register
// (not Arduino pins)
// These are used to set the direction of the bridge driver.
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6

// Arduino pins for the PWM signals.
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5
#define SERVO1_PWM 10
#define SERVO2_PWM 9

// Codes for the motor function.
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

// Arduino Pins for HC-SR04 distance sensor (using analog 4 and 5 as digital)
#define PINGTRIGGER A4
#define PINGRETURN  A5

// Arduiono pins for the IR proximity sensor
#define PROXSENSE   A0
// Declare classes for Servo connectors of the MotorShield.
Servo servo_1;



//***  APPLICATION SETUP
//  *  Nothing here should need to be touched.  This section of the code sets up pins and output options.   
void setup() {
  Serial.begin(9600);
  Serial.println("Hurley / SIPI Robotics Example Sketch");

  servo_1.attach(SERVO1_PWM);
  pinMode(PINGTRIGGER, OUTPUT);
  pinMode(PINGRETURN, INPUT);
  pinMode(PROXSENSE, INPUT);
}

//*** MAIN PROGRAM LOOP
void loop() {
  //Serial.println(sensor_ping_distance());
  //delay(250);



  //lookAround();

  //  --  move forward at moderate speed and loops until the IR sensor reads an obsticle
  m_moveForward(255);
  while (digitalRead(PROXSENSE) == HIGH) {
    delay(50);
  }

  // -- once an obsticle is detected, do this:
  m_stop();
  delay(100);
  m_moveBackward(255);
  delay(500);
  m_stop();
  m_spinLeft(200);
  delay(500);
  m_spinRight(255);
  delay(500);
  m_stop();

}


//*** SUBROUTINES START HERE:
long sensor_ping_distance() {
  long duration, distance;
  digitalWrite(PINGTRIGGER, LOW); // TRIGGER LOW - should not be needed but ensures pin state is off
  delayMicroseconds(2); 
  digitalWrite(PINGTRIGGER, HIGH); // TRIGGER HIGH - creates ultrasonic ping
  delayMicroseconds(10);
  digitalWrite(PINGTRIGGER, LOW);  // TRIGGER LOW
  duration = pulseIn(PINGRETURN, HIGH);  // LISTEN FOR RESPONSE 
  distance = (duration/2) / 29.1;  // math stolen from another sketch.  probably has something to do with the speed of sound.
  return distance;
}


void m_stop() {
  motor(1, RELEASE, 0);
  motor(2, RELEASE, 0);
}

void m_moveForward(int speed) {
  m_stop();
  motor(1, FORWARD, speed);
  motor(2, FORWARD, speed);
}

void m_moveBackward(int speed) {
  m_stop();
  motor(1, BACKWARD, speed);
  motor(2, BACKWARD, speed);
}

void m_backupLeft(int speed) {
  m_stop();
  motor(1, BACKWARD, speed);
}

void m_spinLeft(int speed) {
  m_stop();
  motor(2, BACKWARD, speed);
  motor(1, FORWARD, speed);
}

void m_spinRight(int speed) {
  m_stop();
  motor(1, BACKWARD, speed);
  motor(2, FORWARD, speed);
}


void lookAround() {
  for (int i = 0; i <= 180; i += 45) {
    servo_1.write(i);
    Serial.println("= 0 =");
    delay(1000);
    Serial.println(sensor_ping_distance());
    delay(2000);
  }
}





//***  EVERYTHING FROM HERE DOWN IS THE SIMPLE MOTOR CONTROL LIBRARY
//  *  Nothing here should ever need to be touched

// Initializing
// ------------
// There is no initialization function.
//
// The shiftWrite() has an automatic initializing.
// The PWM outputs are floating during startup,
// that's okay for the Motor Shield, it stays off.
// Using analogWrite() without pinMode() is valid.
//


// ---------------------------------
// motor
//
// Select the motor (1-4), the command,
// and the speed (0-255).
// The commands are: FORWARD, BACKWARD, BRAKE, RELEASE.
//
void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {  
  switch (nMotor)
  {
  case 1:
    motorA   = MOTOR1_A;
    motorB   = MOTOR1_B;
    break;
  case 2:
    motorA   = MOTOR2_A;
    motorB   = MOTOR2_B;
    break;
  case 3:
    motorA   = MOTOR3_A;
    motorB   = MOTOR3_B;
    break;
  case 4:
    motorA   = MOTOR4_A;
    motorB   = MOTOR4_B;
    break;
  default:
    break;
  }

  switch (command)
  {
  case FORWARD:
    motor_output (motorA, HIGH, speed);
    motor_output (motorB, LOW, -1);   // -1: no PWM set
    break;
  case BACKWARD:
    motor_output (motorA, LOW, speed);
    motor_output (motorB, HIGH, -1);  // -1: no PWM set
    break;
  case BRAKE:
    // The AdaFruit library didn't implement a brake.
    // The L293D motor driver ic doesn't have a good
    // brake anyway.
    // It uses transistors inside, and not mosfets.
    // Some use a software break, by using a short
    // reverse voltage.
    // This brake will try to brake, by enabling
    // the output and by pulling both outputs to ground.
    // But it isn't a good break.
    motor_output (motorA, LOW, 255); // 255: fully on.
    motor_output (motorB, LOW, -1);  // -1: no PWM set
    break;
  case RELEASE:
    motor_output (motorA, LOW, 0);  // 0: output floating.
    motor_output (motorB, LOW, -1); // -1: no PWM set
    break;
  default:
    break;
  }
  }
}


// ---------------------------------
// motor_output
//
// The function motor_ouput uses the motor driver to
// drive normal outputs like lights, relays, solenoids,
// DC motors (but not in reverse).
//
// It is also used as an internal helper function
// for the motor() function.
//
// The high_low variable should be set 'HIGH'
// to drive lights, etc.
// It can be set 'LOW', to switch it off,
// but also a 'speed' of 0 will switch it off.
//
// The 'speed' sets the PWM for 0...255, and is for
// both pins of the motor output.
//   For example, if motor 3 side 'A' is used to for a
//   dimmed light at 50% (speed is 128), also the
//   motor 3 side 'B' output will be dimmed for 50%.
// Set to 0 for completelty off (high impedance).
// Set to 255 for fully on.
// Special settings for the PWM speed:
//  Set to -1 for not setting the PWM at all.
//
void motor_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
  case MOTOR1_A:
  case MOTOR1_B:
  motorPWM = MOTOR1_PWM;
  break;
  case MOTOR2_A:
  case MOTOR2_B:
  motorPWM = MOTOR2_PWM;
  break;
  case MOTOR3_A:
  case MOTOR3_B:
  motorPWM = MOTOR3_PWM;
  break;
  case MOTOR4_A:
  case MOTOR4_B:
  motorPWM = MOTOR4_PWM;
  break;
  default:
  // Use speed as error flag, -3333 = invalid output.
  speed = -3333;
  break;
  }

  if (speed != -3333)
  {
  // Set the direction with the shift register
  // on the MotorShield, even if the speed = -1.
  // In that case the direction will be set, but
  // not the PWM.
  shiftWrite(output, high_low);

  // set PWM only if it is valid
  if (speed >= 0 && speed <= 255)    
  {
    analogWrite(motorPWM, speed);
  }
  }
}


// ---------------------------------
// shiftWrite
//
// The parameters are just like digitalWrite().
//
// The output is the pin 0...7 (the pin behind
// the shift register).
// The second parameter is HIGH or LOW.
//
// There is no initialization function.
// Initialization is automatically done at the first
// time it is used.
//
void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  // Do the initialization on the fly,
  // at the first time it is used.
  if (!shift_register_initialized)
  {
  // Set pins for shift register to output
  pinMode(MOTORLATCH, OUTPUT);
  pinMode(MOTORENABLE, OUTPUT);
  pinMode(MOTORDATA, OUTPUT);
  pinMode(MOTORCLK, OUTPUT);

  // Set pins for shift register to default value (low);
  digitalWrite(MOTORDATA, LOW);
  digitalWrite(MOTORLATCH, LOW);
  digitalWrite(MOTORCLK, LOW);
  // Enable the shift register, set Enable pin Low.
  digitalWrite(MOTORENABLE, LOW);

  // start with all outputs (of the shift register) low
  latch_copy = 0;

  shift_register_initialized = true;
  }

  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);

  // Use the default Arduino 'shiftOut()' function to
  // shift the bits with the MOTORCLK as clock pulse.
  // The 74HC595 shiftregister wants the MSB first.
  // After that, generate a latch pulse with MOTORLATCH.
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5); // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5); // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}


