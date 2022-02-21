
#ifndef NAVIGATION_H
#define NAVIGATION_H
#include <Arduino.h>

int add_two_integers(int a, int b);

/*  Variables  */

// needs to be volatile as we are accessing in a interrupt
// volatile long encoder_r;
// volatile long encoder_l;

// int motor_r_vel;
// int motor_l_vel;


/*  High level behaviour  */

void line_follower(int sensor_left, int sensor_right, int motor_left, int motor_right);
void align_with_intersection();


/*  Mid level behaviour  */

void drive_forward(int mm);
void turn_robot(float degrees);


/*  Low level behaviour  */

long get_r_encoder_ticks();
long get_l_encoder_ticks();
void set_vel_r_motor(int vel, bool forward);
void set_vel_l_motor(int vel, bool forward);
#endif