#include "navigation.h"

void setup()
{
    setup_sensors();
    Serial.begin(9600);
}

void loop()
{
    has_turned = false;
    while (has_turned == false) {
        turn_robot(90);
    }
    // put your main code here, to run repeatedly
}
