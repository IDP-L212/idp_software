#include "navigation.h"

bool has_turned = false;
bool finish = false;

void setup()
{
    setup_sensors();
    Serial.begin(9600);
}

void loop()
{
    while (finish == false) {
        
        set_vel_l_motor(200, true);
        set_vel_r_motor(200, true);
        delay(3000);
        
        sweep();
        finish = true;
    }
    // put your main code here, to run repeatedly
}
