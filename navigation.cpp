
#include "navigation.h"

volatile long encoder_ticks_r = 0;
volatile long encoder_ticks_l = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *r_motor = AFMS.getMotor(PORT_MOTOR_R);
Adafruit_DCMotor *l_motor = AFMS.getMotor(PORT_MOTOR_L);

void setup_sensors()
{
    // put your setup_sensors code here
    pinMode(PIN_IR_LINE_BR, INPUT);
    pinMode(PIN_IR_LINE_BL, INPUT);
    AFMS.begin();
}

int add_two_integers(int a, int b)
{
    return a + b;
}

/*  High level behaviour  */

void line_follower()
{
    if (IR_line_sensor(PIN_IR_LINE_BL, 20) == false & IR_line_sensor(PIN_IR_LINE_BR, 150) == false)
    {
        Serial.println("FORWARD");
        set_vel_r_motor(250, true);
        set_vel_l_motor(250, true);
    }

    else if (IR_line_sensor(PIN_IR_LINE_BL, 20) == true & IR_line_sensor(PIN_IR_LINE_BR, 150) == false)
    {
        Serial.println("RIGHT");
        set_vel_r_motor(0, true);
        set_vel_l_motor(250, true);
    }
    else if (IR_line_sensor(PIN_IR_LINE_BL, 20) == false & IR_line_sensor(PIN_IR_LINE_BR, 150) == true)
    {
        Serial.println("LEFT");
        set_vel_r_motor(250, true);
        set_vel_l_motor(0, true);
    }
    else
    {
        Serial.println("STOP");
        set_vel_r_motor(0, true);
        set_vel_l_motor(0, true);
    }
}

void align_with_intersection()
{
    return; // TODO
}

/*  Mid level behaviour  */

// drive forward X mm forward using encoders
void drive_forward(int mm)
{
    return; // TODO
}

// in place, rotate X degrees
void turn_robot(float degrees)
{
    return; // TODO
}

/*  Low level behaviour  */

long get_r_encoder_ticks()
{
    return encoder_ticks_r; // TODO implement interrupts
}

long get_l_encoder_ticks()
{
    return encoder_ticks_l; // TODO implement interrupts
}

void set_vel_r_motor(int vel, bool forward)
{
    r_motor->setSpeed(vel);
    if (forward == true)
    {
        r_motor->run(FORWARD);
    }
    else
    {
        r_motor->run(BACKWARD);
    }
}

void set_vel_l_motor(int vel, bool forward)
{
    l_motor->setSpeed(vel);
    if (forward == true)
    {
        l_motor->run(BACKWARD);
    }
    else
    {
        l_motor->run(FORWARD);
    }
}

bool IR_line_sensor(int IR_PIN, int threshold)
{
    // true correponds to white line, false corresponds to a black line
    int output_voltage = analogRead(IR_PIN);
    if (output_voltage < threshold)
    {
        return false;
    }
    else
    {
        return true;
    }
}

// TODO Sensors

// Line sensors -> TBD By Misha on 22nd Feb
//   is_line_detected (for right/left and front/back)
// Distance sensor (for sweep) -> done by Misha
//   get_ultrasound_distance (used for both sweep + detecting if block is in gripper)
// Colour sensor + gripping in general
//   is_block_red, is_block_blue, is_block_present_in_gripper, ...
