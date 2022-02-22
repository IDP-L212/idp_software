
#ifndef NAVIGATION_H
#define NAVIGATION_H
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor_right = AFMS.getMotor(1);
Adafruit_DCMotor *motor_left = AFMS.getMotor(2);

int add_two_integers(int a, int b);
#endif
void line_follower(int sensor_left, int sensor_right, int motor_left, int motor_right);

void rotate_cw(bool has_rotated_cw);
void rotate_ccw(bool has_rotated_ccw);