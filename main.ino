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
    delay(1000); //starting delay to make reset obvious 
    while (finish == false) {
        
        open_servo();
        //inital alignment with start area wall to ensure robot start location is consistent for each run
        move_backward(100);
        //drive from start area to first corner (up to distance of 8 cm from wall)
        wall_follower(8);
        //rotate robot by 90 degrees anticlockwise
        turn_robot_anticlock(90);
        //drive to the position of first block (28 cm from wall)
        wall_follower(26);
        //stop moving, turn robot 90 degrees anticlockwise, and shine red LED (first block is red)
        stop_moving();
        turn_robot_anticlock(90);
        delay(1000);
        red_on(); 
        //move towards the block, stop moving, then close the grabber
        move_forward(100);
        stop_moving();
        delay(1000); 
        /* bool red_block = is_block_red();
        if (red_block == true) {
            red_on();
        }
        else {
            green_on();
        }*/
        
        close_servo();
        delay(1000);
        //align robot with wall
        turn_robot_clock(90);
        delay(1000);
        move_forward(40);
        stop_moving();
        turn_robot_anticlock(90);
        delay(1000);
        //follow wall until robot reaches corner again and turn anticlockwise
        wall_follower(8);
        turn_robot_anticlock(90);

        /* //BLUE BLOCK
        wall_follower(8);
        turn_robot_anticlock(90);
        stop_moving();
        close_servo();
        wall_follower(83);
        stop_moving();
        turn_robot_anticlock(90);
        delay(1000);
        move_backward(105);
        move_forward(65);
        stop_moving();
        delay(1000);
        open_servo();
        delay(1000);
        move_backward(100);
        stop_moving();
        turn_robot_clock(90);
        delay(1000);
        */

        //RED BLOCK
        //follow wall until robot reaches first drop-off area and turn anticlockwise
        wall_follower(102);
        stop_moving();
        turn_robot_anticlock(90);
        delay(1000);
        //move back to align with wall then move forward to drop-off area.
        move_backward(105);
        move_forward(65);
        stop_moving();
        delay(1000);
        //open grabber, reverse and realign with wall
        open_servo();
        delay(1000);
        move_backward(100);
        stop_moving();
        //return to start position
        turn_robot_clock(90);
        delay(1000);
        wall_follower(8);
        stop_moving();
        turn_robot_anticlock(90);
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

