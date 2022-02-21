
#include "navigation.h"

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

// in place, rotate X degrees
void turn_robot(float degrees) {
    return; // TODO
}


/*  Low level behaviour  */

long get_r_encoder_ticks() {
    return 0L; // TODO
}

long get_l_encoder_ticks() {
    return 0L; // TODO
}

void set_vel_r_motor(int vel, bool forward) {
    return; // TODO
}

void set_vel_l_motor(int vel, bool forward) {
    return; // TODO
}


