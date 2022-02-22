
#ifndef NAVIGATION_H
#define NAVIGATION_H
#include <Arduino.h>
int add_two_integers(int a, int b);
#endif
void line_follower(int sensor_left, int sensor_right, int motor_left, int motor_right);

void rotate_cw(bool has_rotated_cw);
void rotate_ccw(bool has_rotated_ccw);