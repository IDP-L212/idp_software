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
    /*
    while (finish == false) {
        
        set_vel_l_motor(200, true);
        set_vel_r_motor(200, true);
        delay(3000);
        
        sweep();
    
        finish = true;
    
    }
    */
    
   
    /* distance = getDetectorDist();
    if (distance > 10) {
        set_vel_l_motor(250, true);
        set_vel_r_motor(250, true);
    }
    
    else {
        set_vel_l_motor(0, true);
        set_vel_r_motor(0, true);
    }
    delay(100);
    */
   while (finish == false) {
        turn_robot_anticlock(40);
        sweep();
        delay(3000);
        finish = true;
   }
    
    

    
    // put your main code here, to run repeatedly
    
}
