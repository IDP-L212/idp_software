#ifndef NAVIGATION_H
#define NAVIGATION_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include <PID_v1.h>

int add_two_integers(int a, int b);

/* Define Motor Ports (M1-M4) on Adafruit Motor Shield */
#define PORT_MOTOR_R 1
#define PORT_MOTOR_L 2

#define ENCODER_R_PIN 2
#define ENCODER_L_PIN 3

/*Define pin locations on Arduino Uno Wifi Rev2*/
#define PIN_IR_LINE_BR A2
#define PIN_IR_LINE_BL A0
/*Defin pin locations on Arduino Uno for Ultrasonic Sensors */
#define echoPin_1 11 // attach pin D11 Arduino to pin Echo of HC-SR04
#define trigPin_1 12 // attach pin D12 Arduino to pin Trig of HC-SR04
#define echoPin_2 9 // attach pin D9 Arduino to pin Echo of HC-SR04
#define trigPin_2 8 // attach pin D8 Arduino to pin Trig of HC-SR04

#define servoPin 10

#define amber_led 4
#define photoResistor 0
#define red 7
#define green_led 6
#define red_led 5

#define Lswitch 2 
#define flag 0

#define buttonPin 7

/*  Variables  */

// needs to be volatile as we are accessing in a interrupt
extern volatile long r_ticks;
extern volatile long l_ticks;

extern int motor_r_vel;
extern int motor_l_vel;

extern long duration; // variable for the duration of sound wave travel
extern int distance;  // variable for the distance measurement
extern int distance_2;  // variable for the distance measurement

extern int ledState;
extern unsigned long previousMillis;
extern const long interval;

// Create Adafruit_DCMotor Object for RHS Motor
extern Adafruit_MotorShield AFMS;
extern Adafruit_DCMotor *r_motor;
extern Adafruit_DCMotor *l_motor;
extern Servo myservo;


/* encoder variables and functions */

void updateOdometry();
void run_navigation();
// extern long diag_timer = 0;
// extern int last_r = 0;
// extern int last_l = 0;

/* experimental encoder navigation */

void set_target(int tar_x, int tar_y);

/*  High level behaviour  */

void setup_sensors();
void setup_motor();
void line_follower();
void align_with_intersection();

/*  Mid level behaviour  */

void zero_position();
void drive_forward(int mm);
void drive_forward_encoder(int mm);
void turn_robot_encoder(double degrees);
void turn_robot_encoder_rads(double rads);
void turn_robot_clock(float degrees);
void turn_robot_anticlock(float degrees);

/*  Low level behaviour  */

long get_r_encoder_ticks();
long get_l_encoder_ticks();
void set_vel_r_motor(int vel, bool forward);
void set_vel_l_motor(int vel, bool forward);
bool IR_line_sensor(int IR_PIN, int threshold);

void wall_follower(int distance_2);

void move_forward(int final_count);
void move_backward(int final_count);
void stop_moving();

int getDetectorDist();
int getDetectorDist2();
void amber_light();
void sweep();

bool is_block_red();
void red_on();
void green_on();

void open_servo();
void close_servo();

void print_debug();

#endif
