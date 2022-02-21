#include <Wire.h>
#include "Adafruit_MotorShield.h"

#include "navigation.h"
// variable declaration
// Change to new pin locations when using on new hardware
int sensor_left = 6;
int sensor_right = 7;
int motor_right = 12;
int motor_left = 13;

bool has_rotated_cw;
bool has_rotated_ccw;

void setup()
{
    // put your setup code here, to run once:
    pinMode(sensor_left, INPUT);
    pinMode(sensor_right, INPUT);
    pinMode(motor_left, OUTPUT);
    pinMode(motor_right, OUTPUT);
    Serial.begin(9600);
}

void loop()
{
    // put your main code here, to run repeatedly:
    // line_follower(sensor_left, sensor_right, motor_left, motor_right);

    //rotate_ccw(motor_left, motor_right, has_rotated_ccw);
    //rotate_cw(motor_left, motor_right, has_rotated_cw);

    has_rotated_ccw = false;
    while (has_rotated_ccw == false) {
        rotate_ccw(motor_left, motor_right, has_rotated_ccw);
    }
    
}
