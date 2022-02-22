#include <Wire.h>
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"


#include "navigation.h"
// variable declaration
// Change to new pin locations when using on new hardware

//int sensor_left = 6;
//int sensor_right = 7;
//int motor_right = 12;
//int motor_left = 13;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor_right = AFMS.getMotor(1);
Adafruit_DCMotor *motor_left = AFMS.getMotor(2);


bool has_rotated_cw;
bool has_rotated_ccw;

void setup()
{
    // put your setup code here, to run once:
    //pinMode(sensor_left, INPUT);
    //pinMode(sensor_right, INPUT);
    //pinMode(motor_left, OUTPUT);
    //pinMode(motor_right, OUTPUT);
    
    AFMS.begin();
    Serial.begin(9600);
}

void loop()
{
    // put your main code here, to run repeatedly:
    // line_follower(sensor_left, sensor_right, motor_left, motor_right);

    //rotate_ccw(motor_left, motor_right, has_rotated_ccw);
    //rotate_cw(motor_left, motor_right, has_rotated_cw);

    has_rotated_cw = false;
    while (has_rotated_cw == false) {
        motor_left->setSpeed(150);
        motor_left->run(FORWARD);
        motor_right->setSpeed(150);
        motor_right->run(BACKWARD);
        delay(2000);
        has_rotated_cw = true;
    }
    
}
