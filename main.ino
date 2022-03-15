#include "navigation.h"

bool has_turned = false;
bool finish = false;
int buttonState = 0;
int red_count = 0;
int blue_count = 0;
bool red_block;

void setup()
{
    setup_sensors();
    Serial.begin(9600);
}

void loop()
{
    delay(1000); //starting delay to make reset obvious 
    buttonState = digitalRead(buttonPin);
    // if (buttonState == HIGH) {
        while (finish == false) {

            /*
            if (red_count == 0 and blue_count == 0) {
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
                //RED BLOCK
                //follow wall until robot reaches first drop-off area and turn anticlockwise
                wall_follower(102);
                stop_moving();
                turn_robot_anticlock(90);
                delay(1000);
                //move back to align with wall then move forward to drop-off area.
                move_backward(105);
                move_forward(57);
                stop_moving();
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
                red_count += 1;
            }

            open_servo();
            // inital alignment with start area wall to ensure robot start location is consistent for each run
            //move_backward(100);
            // drive from start area to first corner (up to distance of 8 cm from wall)
            wall_follower(8);
            // rotate robot by 90 degrees anticlockwise
            turn_robot_anticlock(90);
            // drive to the position of first block (28 cm from wall)
            wall_follower(50);
            // stop moving, turn robot 90 degrees anticlockwise, and sweep.
            stop_moving();
            turn_robot_anticlock(90);
            stop_moving();
            move_backward(100);
            sweep();
            // turn towards block, move, and collect it
            turn_robot_clock(70);
            stop_moving();
            green_on();
            move_forward(60);
            stop_moving();
            close_servo();
            // turn back to wall and wall follow until robot reaches cornrer
            turn_robot_anticlock(130);
            wall_follower(8);
            turn_robot_anticlock(90);
            stop_moving();
            // determine colour of block to determine route to take
            red_block = is_block_red();

            if (red_block == false and blue_count == 0 and red_count == 1) {
                //BLUE BLOCK
                // move to end of board
                green_on();
                wall_follower(8);
                turn_robot_anticlock(90);
                stop_moving();
                // move adjacent to blue drop-off area
                wall_follower(83);
                stop_moving();
                turn_robot_anticlock(90);
                delay(1000);
                // move towards blue drop-off area and open servo
                move_backward(105);
                move_forward(65);
                stop_moving();
                open_servo();
                delay(1000);
                // reverse into wall, turn, and loop again by wall following
                move_backward(100);
                stop_moving();
                turn_robot_clock(90);
                delay(1000);
            }

            else if (red_block == true and blue_count == 0 and red_count == 1) {
                // RED BLOCK 2
                // move to end of board
                red_on();
                wall_follower(102);
                stop_moving();
                turn_robot_anticlock(90);
                delay(1000);
                //move back to align with wall then move forward to drop-off area.
                move_backward(105);
                move_forward(35);
                turn_robot_clock(90);
                move_forward(25);
                stop_moving();
                //open grabber, reverse and realign with wall
                open_servo();
                delay(1000);
                move_backward(100);
            }
            
            
            
            /*
            bool red_block = is_block_red();
            if (red_block == true) {
                red_on();
            }
            else {
                green_on();
            }
            */
            
            //drive from first corner to collection area (SWEEP)
            //wall_follower(50);
            //stop robot once it has reached collection area
            //stop_moving();
            //turn robot 90 degrees anticlockwise
            //turn_robot_anticlock(90);
            // go backwards and hit wall
            //move_backward(100);
            //delay(1000);
            // run sweep algorithm to locate block
            
            open_servo();
            sweep();
            turn_robot_clock(70);
            delay(1000);
            // move forward to capture block
            move_forward(60);
            stop_moving();
            // close servo, turn, and follow wall to corner
            close_servo();
            delay(1000);
            move_forward(100);
            //turn_robot_anticlock(130);
            stop_moving();
            move_backward(20);
            turn_robot_anticlock(90);
            wall_follower(8);
            stop_moving();
            
            
            //zero_position();
            /*drive_forward_encoder(200);
            delay(200);
            zero_position();
            turn_robot_encoder(90);
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
    //}
}

