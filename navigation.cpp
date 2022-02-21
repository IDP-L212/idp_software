
#include "navigation.h""
int add_two_integers(int a, int b)
{
    return a + b;
}

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