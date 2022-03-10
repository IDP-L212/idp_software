#include "navigation.h"

bool has_turned = false;
bool finish = false;
int count = 0;
int distance_2 = 999;

void setup()
{
    setup_sensors();
    Serial.begin(9600);
}

void loop()
{
    /*
    drive_forward(1000);
    turn_robot_anticlock(90);
    drive_forward(1000);
    turn_robot_anticlock(90);
    drive_forward(-100);
    sweep();
    turn_robot_clock(90);
    drive_forward(100);
    if (is_block_red() == true) {
        red_on();
        // Set up return algorithm around red corner
    }
    else {
        green_on();
        // Set up return algorithm around blue corner
    }
    */
    digitalWrite(amber_led, HIGH);
    delay(1000); //starting delay to make reset obvious 
    while (finish == false) {
        
        //inital alignment with start area wall to ensure robot start location is consistent for each run
        while (count < 100) {
            set_vel_l_motor(220, false); //DIFFERENT SPEEDS TO FIX ROTOR DIFFERENCE
            set_vel_r_motor(200, false);
            count = count + 1;
            delay(25);
        }
        count = 0;
        //drive from start area to first corner 
        while (distance_2 > 8) {
            wall_follower();
            distance_2 = getDetectorDist2();
        }

        //rotate robot by 90 degrees
        count = 0;
        distance_2 = 999;
        turn_robot_anticlock(90);

        while (distance_2 > 28) {
            wall_follower();
            distance_2 = getDetectorDist2();
        }

        //turn robot 90 degrees anticlockwise
        set_vel_l_motor(0, true);
        set_vel_r_motor(0, true);
        turn_robot_anticlock(80);

        delay(1000);
        digitalWrite(red, HIGH);
        delay(5000);
        digitalWrite(red, LOW);

        while (count < 100) {
            set_vel_l_motor(220, true);
            set_vel_r_motor(200, true);
            count = count + 1;
            delay(25);
        }
        count = 0;

        set_vel_l_motor(0, true);
        set_vel_r_motor(0, true);
        delay(1000);
        close_servo();
        delay(1000);
        
        turn_robot_clock(90);
        delay(1000);

        while (count < 45) {
            set_vel_l_motor(220, true);
            set_vel_r_motor(200, true);
            count = count + 1;
            delay(25);
        }
        set_vel_l_motor(0, true);
        set_vel_r_motor(0, true);
        turn_robot_anticlock(90);
        delay(1000);

        //(ACTUAL)
        while (distance_2 > 8) {
            wall_follower();
            distance_2 = getDetectorDist2();
        }
        
        distance_2 = 999;
        
        turn_robot_anticlock(90);
        
        while (distance_2 > 100) {
            wall_follower();
            distance_2 = getDetectorDist2();
        }
        distance_2 = 999;
        count = 0;

        set_vel_l_motor(0, true);
        set_vel_r_motor(0, true);

        turn_robot_anticlock(90);
        delay(1000);

        while (count < 50) {
            set_vel_l_motor(200, false);
            set_vel_r_motor(200, false);
            count = count + 1;
            delay(25);
        }

        count = 0;

        while (count < 68) {
            set_vel_l_motor(220, true);
            set_vel_r_motor(200, true);
            count = count + 1;
            delay(25);
        }

        count = 0;
        set_vel_l_motor(0, true);
        set_vel_r_motor(0, true);
        delay(1000);
        open_servo();
        delay(1000);
        while (count < 100) {
            set_vel_l_motor(220, false);
            set_vel_r_motor(200, false);
            count = count + 1;
            delay(25);
        }

        set_vel_l_motor(0, true);
        set_vel_r_motor(0, true);

        turn_robot_clock(90);

        delay(1000);

        while (distance_2 > 8) {
            wall_follower();
            distance_2 = getDetectorDist2();
        }

        set_vel_l_motor(0, true);
        set_vel_r_motor(0, true);
        
        digitalWrite(amber_led, LOW);

        /*
        //turn to be in-line with the block
        turn_robot_clock(90);
        //drive towards block for arbitary distance 
        while (count < 10) {
            set_vel_l_motor(150, true);
            set_vel_r_motor(150, true);
            count = count + 1;
        }
        count = 0;
        set_vel_l_motor(0, true);
        set_vel_r_motor(0,true);
        //detect block colour once block is positioned in funnel 
        if (is_block_red() == true){
            red_on();
        }
        else{
            green_on();
        }

        close_servo();
        turn_robot_anticlock(180);*/
        
        /*
        set_vel_l_motor(240, true);
        set_vel_r_motor(200, true);
        delay(5000);
        set_vel_l_motor(0, true);
        set_vel_r_motor(0, true);
        */

       /* while (count < 100) {
            set_vel_l_motor(240, false); 
            set_vel_r_motor(200, false);
            count = count + 1;
            delay(25);
        }
        set_vel_l_motor(0, true);
        set_vel_r_motor(0, true); */

        //drive from first corner to collection area (SWEEP)
        
        /* while (distance_2 > 50) {
            wall_follower();
            distance_2 = getDetectorDist2();
        }
        
        //stop robot once it has reached collection area
        count = 0;
        set_vel_l_motor(0, true);
        set_vel_r_motor(0, true);
        
        //turn robot 90 degrees anticlockwise
        turn_robot_anticlock(90);
         // go backwards and hit wall
        while (count < 50) {
            set_vel_l_motor(240, false);
            set_vel_r_motor(200, false);
            count = count + 1;
            delay(25);
        }
        delay(1000);
        count = 0;
        
        //run sweep algorithm to locate block
        sweep();

        turn_robot_clock(65);
        delay(1000);
        while (count < 100) {
            set_vel_l_motor(240, true);
            set_vel_r_motor(200, true);
            count = count + 1;
            delay(25);
        }

        count = 0;

        set_vel_l_motor(0, true);
        set_vel_r_motor(0, true);
        delay(1000);

        open_servo();

        delay(1000);
        turn_robot_anticlock(85);

        set_vel_l_motor(0, true);
        set_vel_r_motor(0, true);

        delay(1000); 
        
        */


        finish = true;
   }


    /*
    while (finish == false) {
        
        set_vel_l_motor(200, true);
        set_vel_r_motor(200, true);
        delay(3000);
        
        sweep();
    
        finish = true;
    
    }
    */
    
    /*
    distance = getDetectorDist();
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
    
   /*while (finish == false) {
        turn_robot_anticlock(40);
        sweep();
        delay(3000);
        finish = true;
   }
    */

   /*
   if (is_block_red() == true) {
       digitalWrite(green_led, LOW);
       digitalWrite(red_led, HIGH);
   }
   else {
       digitalWrite(green_led, HIGH);
       digitalWrite(red_led, LOW);
   }

   delay(50);
   */
   

   /*while (finish == false){
        //set_vel_l_motor(120, true);
        //set_vel_r_motor(210, true);
        //delay(5000);
        //set_vel_l_motor(0, true);
        //set_vel_r_motor(0, true);

        sweep();

        //turn_robot_anticlock(90);
        //open_servo();
        //delay(3000);
        //close_servo();
        //delay(3000);
        finish = true;
   }
   */
    
    // put your main code here, to run repeatedly
    
}
