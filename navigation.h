#ifndef NAVIGATION_H
#define NAVIGATION_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

int add_two_integers(int a, int b);

/* Define Motor Ports (M1-M4) on Adafruit Motor Shield */
#define PORT_MOTOR_R 1
#define PORT_MOTOR_L 3
/*Define pin locations on Arduino Uno Wifi Rev2*/
#define PIN_IR_LINE_R 1
#define PIN_IR_LINE_L 3
/*Defin pin locations on Arduino Uno for Ultrasonic Sensors */
#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04

/*  Variables  */

// needs to be volatile as we are accessing in a interrupt
extern volatile long encoder_ticks_r;
extern volatile long encoder_ticks_l;

extern int motor_r_vel;
extern int motor_l_vel;

extern long duration; // variable for the duration of sound wave travel
extern int distance; // variable for the distance measurement
// Create Adafruit_DCMotor Object for RHS Motor
extern Adafruit_MotorShield AFMS;
extern Adafruit_DCMotor *r_motor;
extern Adafruit_DCMotor *l_motor;

/*  High level behaviour  */

void setup_sensors();
void line_follower(int sensor_left, int sensor_right, int motor_left, int motor_right);
void align_with_intersection();


/*  Mid level behaviour  */

void drive_forward(int mm);
void turn_robot_clock(float degrees);
void turn_robot_anticlock(float degrees);


/*  Low level behaviour  */

long get_r_encoder_ticks();
long get_l_encoder_ticks();
void set_vel_r_motor(int vel, bool forward);
void set_vel_l_motor(int vel, bool forward);
void sweep();
int getDetectorDist();
#endif
