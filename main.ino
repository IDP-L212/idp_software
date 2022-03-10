#include "navigation.h"
bool finish = false;

void setup()
{
    setup_sensors();
    Serial.begin(9600);
}

#define encoder_r A5
#define encoder_l A4

void loop()
{ 

    line_follower();

   /* while(finish == false){
       delay(1000);
       drive_forward(400);
       turn_robot_clock(90);
       drive_forward(200);
       finish = true;
   }
   */
    
    


}