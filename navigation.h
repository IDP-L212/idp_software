
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
/*Define pin locations on Arduino Uno*/
#define PIN_IR_LINE_BR A2
#define PIN_IR_LINE_BL A0
/*Defin pin locations on Arduino Uno for Ultrasonic Sensors */
#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04
#define servoPin 10

#define amber_led 4
#define photoResistor 0
#define red 7
#define green_led 6
#define red_led 5

#define Lswitch 13
#define Lswitch_2 2 
#define flag 0

#define buttonPin 3

/*  Variables  */

// needs to be volatile as we are accessing in a interrupt
extern volatile long encoder_ticks_r;
extern volatile long encoder_ticks_l;

extern int motor_r_vel;
extern int motor_l_vel;

extern long duration; // variable for the duration of sound wave travel
extern int distance; // variable for the distance measurement
extern int ledState;
extern unsigned long previousMillis;
extern const long interval;

// Create Adafruit_DCMotor Object for RHS Motor
extern Adafruit_MotorShield AFMS;
extern Adafruit_DCMotor *r_motor;
extern Adafruit_DCMotor *l_motor;
extern Servo myservo;

/*  High level behaviour  */

void setup_sensors();
void line_follower();
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
bool IR_line_sensor(int IR_PIN, int threshold);

void wall_follower(int distance_2);

void move_forward(int final_count);
void move_backward(int final_count);
void stop_moving();

int getDetectorDist();
int getDetectorDist2();
void amber_light();
void sweep();
void arc();

bool is_block_red();
void red_on();
void green_on();

void open_servo();
void close_servo();
bool switch_closed();
bool switch_closed_2();

bool button_on();
void print_debug();

#endif
