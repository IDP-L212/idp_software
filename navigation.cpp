#include "navigation.h"

volatile long encoder_ticks_r = 0;
volatile long encoder_ticks_l = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *r_motor = AFMS.getMotor(PORT_MOTOR_R);
Adafruit_DCMotor *l_motor = AFMS.getMotor(PORT_MOTOR_L);

void setup_sensors() {
    // put your setup_sensors code here
    pinMode(PIN_IR_LINE_L, INPUT);
    pinMode(PIN_IR_LINE_R, INPUT);
    AFMS.begin();
}

int add_two_integers(int a, int b)
{
    return a + b;
}


/*  High level behaviour  */

void line_follower(int sensor_left, int sensor_right, int motor_left, int motor_right)
{
    if (digitalRead(sensor_left) == HIGH & digitalRead(sensor_right) == HIGH)
    {
        Serial.println("FORWARD");
        digitalWrite(motor_left, HIGH);
        digitalWrite(motor_right, HIGH);
    }

    else if (digitalRead(sensor_right) == HIGH & digitalRead(sensor_left) == LOW)
    {
        Serial.println("RIGHT");
        digitalWrite(motor_right, LOW);
        digitalWrite(motor_left, HIGH);
    }
    else if (digitalRead(sensor_right) == LOW & digitalRead(sensor_left) == HIGH)
    {
        Serial.println("LEFT");
        digitalWrite(motor_right, HIGH);
        digitalWrite(motor_left, LOW);
    }
    else
    {
        Serial.println("STOP");
        digitalWrite(motor_right, LOW);
        digitalWrite(motor_left, LOW);
    }
}

void align_with_intersection() {
    return;  // TODO
}


/*  Mid level behaviour  */

// drive forward X mm forward using encoders
void drive_forward(int mm) {
    return; // TODO
}

// in place, rotate X degrees clockwise
void turn_robot(float degrees, bool has_turned) {
    l_motor->setSpeed(150);
    l_motor->run(FORWARD);
    r_motor->setSpeed(150);
    l_motor->run(BACKWARD);
    delay(1000);
    has_turned = true;
    return; // TODO
}


/*  Low level behaviour  */

long get_r_encoder_ticks() {
    return encoder_ticks_r; // TODO implement interrupts
}

long get_l_encoder_ticks() {
    return encoder_ticks_l; // TODO implement interrupts
}

void set_vel_r_motor(int vel, bool forward) {
    r_motor->setSpeed(vel);
    if (forward == true) {
        r_motor->run(FORWARD);
    }
    else{
        r_motor->run(BACKWARD);
    }
}

void set_vel_l_motor(int vel, bool forward) {
    l_motor->setSpeed(vel);
    if (forward == true)
    {
        l_motor->run(FORWARD);
    }
    else
    {
        l_motor->run(BACKWARD);
    }
}


// TODO Sensors

// Line sensors -> TBD By Misha on 22nd Feb
//   is_line_detected (for right/left and front/back)
// Distance sensor (for sweep) -> done by Misha
//   get_ultrasound_distance (used for both sweep + detecting if block is in gripper)
// Colour sensor + gripping in general
//   is_block_red, is_block_blue, is_block_present_in_gripper, ...

