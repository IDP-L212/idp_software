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
    //if (button_on() == true) {
        while (finish == false) {

            //**********-----------------------------FIRST BLOCK-------------------------------------************//
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
                //stop moving, turn robot 90 degrees anticlockwise, and shine red LED (assume first block is red)
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
                move_forward(30);
                stop_moving();
                turn_robot_anticlock(90);
                delay(1000);
                //follow wall until robot reaches corner again and turn anticlockwise
                wall_follower(8);
                stop_moving();
                red_block = is_block_red();
                delay(1000);
                turn_robot_anticlock(90);
                if (red_block == false and blue_count == 0 and red_count == 0)
                {
                    // BLOCK 1 IS BLUE  - BLUE LOCATION 1
                    green_on();
                    wall_follower(8);
                    turn_robot_anticlock(90);
                    stop_moving();
                    // move adjacent to blue drop-off area
                    wall_follower(83);
                    stop_moving();
                    turn_robot_anticlock(90);
                    delay(1000);
                    // move towards blue drop-off area and open servo (BLUE LOCATION 1)
                    move_backward(105);
                    move_forward(51); // distance from wall to BLUE LOCATION 1
                    stop_moving();
                    open_servo();
                    delay(1000);
                    // reverse into wall, turn, and loop again by wall following
                    move_backward(100);
                    stop_moving();
                    blue_count += 1; //BLUE BLOCK PLACED
                    turn_robot_clock(90);
                    delay(1000);
                }

                else if (red_block == true and blue_count == 0 and red_count == 0)
                {
                    // BLOCK 1 IS RED - RED LOCATION 1
                    red_on();
                    wall_follower(102);
                    stop_moving();
                    turn_robot_anticlock(90);
                    delay(1000);
                    // move back to align with wall then move forward to drop-off area.
                    move_backward(105);
                    move_forward(52); //distance from wall to RED LOCATION 1
                    //stop robot
                    stop_moving();
                    // open grabber, reverse and realign with wall
                    open_servo();
                    delay(1000);
                    move_backward(100);
                    stop_moving();
                    red_count += 1; //RED BLOCK PLACED
                    turn_robot_clock(90);
                    delay(1000);
                    wall_follower(8);
                    stop_moving();
                    turn_robot_anticlock(90);
                    // reverse into back-wall for alignment
                    move_backward(100);
                }
            }
            
             //**********-----------------------------SECOND BLOCK-------------------------------------************//
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
            // turn towards block, move, and collect it
            sweep();
            turn_robot_clock(70);
            stop_moving();
            //GUESS COLOUR (ASSUMED BLUE)
            green_on();
            move_forward(70);
            stop_moving();
            close_servo();
            delay(1000);
            // turn back to wall and wall follow until robot reaches cornrer
            wall_follower(8);
            stop_moving();
            red_block = is_block_red();
            delay(1000);
            turn_robot_anticlock(90);
            if (red_block == false and blue_count == 0) { //SCENARIO 1
                //BLOCK 2 IS BLUE  -> BLUE LOCATION 1
                green_on();
                wall_follower(8);
                turn_robot_anticlock(90);
                stop_moving();
                // move adjacent to blue drop-off area
                wall_follower(83);
                stop_moving();
                turn_robot_anticlock(90);
                delay(1000);
                // move towards blue drop-off area and open servo (BLUE LOCATION 1)
                move_backward(105);
                move_forward(51);
                stop_moving();
                open_servo();
                delay(1000);
                // reverse into wall, turn, and loop again by wall following
                move_backward(100);
                stop_moving();
                blue_count += 1;
                turn_robot_clock(90);
                delay(1000);
            }

            else if (red_block == true and red_count == 1) { //SCENARIO 2
                // BLOCK 2 IS RED -> RED LOCATION 2
                // move to end of board
                red_on();
                wall_follower(102);
                stop_moving();
                turn_robot_anticlock(90);
                delay(1000);
                //move back to align with wall then move forward to drop-off area.
                move_backward(105);
                move_forward(28);
                turn_robot_clock(90);
                move_forward(30);
                stop_moving();
                //open grabber, reverse and realign with wall
                open_servo();
                delay(1000);
                move_backward(30);
                stop_moving();
                red_count += 1;
                turn_robot_anticlock(90);
                delay(1000);
                // arc around red block
                // hit table side-wall for alignment
                move_backward(100);
                arc();
                move_backward(30);
                turn_robot_anticlock(90);
                wall_follower(8);
                stop_moving();
                turn_robot_anticlock(90);
                //reverse into back-wall for alignment
                move_backward(100);
            }
            else if (red_block == true and red_count == 0) //SCENARIO 3
            {                                                                       
                // BLOCK 2 IS RED -> RED LOCATION 1
                // move to end of board
                red_on();
                wall_follower(102);
                stop_moving();
                turn_robot_anticlock(90);
                delay(1000);
                // move back to align with wall then move forward to drop-off area.
                move_backward(105);
                move_forward(50);
                stop_moving();
                // open grabber, reverse and realign with wall
                open_servo();
                delay(1000);
                move_backward(100);
                stop_moving();
                red_count += 1;
                turn_robot_clock(90);
                delay(1000);
                wall_follower(8);
                stop_moving();
                turn_robot_anticlock(90);
                // reverse into back-wall for alignment
                move_backward(100);
            }
            else  //SCENARIO 4
            {
                // BLOCK 2 IS BLUE -> BLUE LOCATION 2
                // move to end of board
                green_on();
                wall_follower(8);
                stop_moving();
                turn_robot_anticlock(90);
                delay(1000);
                // move back to align with wall then move forward to drop-off area.
                move_backward(105);
                wall_follower(83);
                turn_robot_anticlock(90);
                move_backward(100);
                move_forward(22); //perpendicular to drop-off location 
                turn_robot_anticlock(90);
                move_forward(28); //head on from drop-off location
                stop_moving();
                // open grabber, reverse and realign with wall
                open_servo();
                delay(1000);
                move_backward(30);
                turn_robot_clock(90);
                move_backward(105);               
                stop_moving();
                blue_count += 1;
                move_forward(20);
                turn_robot_clock(90);
                delay(1000);
            }
            
            
            



            // RED POSITION 1
            /*close_servo();
            delay(1000);
            wall_follower(102);
            turn_robot_anticlock(90);
            move_backward(100);
            move_forward(51); //distance to RED LOCATION 1 
            stop_moving();
            //open grabber, reverse and realign with wall
            open_servo();
            delay(1000);
            move_backward(100);
            stop_moving();
            red_count += 1; //RED BLOCK PLACED
            //turn_robot_clock(90);
            delay(1000);
            //wall_follower(8);
            stop_moving();*/



            // RED POSITION 2 + ARC
            /*close_servo();
            move_backward(105);
            move_forward(28);
            turn_robot_clock(90);
            move_forward(30);
            stop_moving();
            //open grabber, reverse and realign with wall
            open_servo();
            delay(1000);
            move_backward(30);
            stop_moving();
            red_count += 1;
            turn_robot_anticlock(90);
            move_backward(100);
            stop_moving();
            arc();
            move_backward(20);
            turn_robot_anticlock(90);
            /*wall_follower(8);
            stop_moving();
            turn_robot_anticlock(90);
            //reverse into back-wall for alignment
            move_backward(100);
            stop_moving();*/



            //BLOCK 2 IS BLUE  -> BLUE LOCATION 1
            // move towards blue drop-off area and open servo (BLUE LOCATION 1)
            /*close_servo();
            move_backward(105);
            move_forward(51);
            stop_moving();
            open_servo();
            delay(1000);
            // reverse into wall, turn, and loop again by wall following
            move_backward(100);
            stop_moving();
            blue_count += 1;
            //turn_robot_clock(90);
            delay(1000);*/



            // BLUE LOCATION 2
            /*wall_follower(100);
            stop_moving();
            turn_robot_anticlock(90);
            delay(1000);*/
            // move towards blue drop-off area and open servo (BLUE LOCATION 2)
            
            
            /*close_servo();
            move_backward(100);
            move_forward(22);
            turn_robot_anticlock(90);
            move_forward(28);
            stop_moving();
            open_servo();
            delay(1000);
            move_backward(30);
            turn_robot_clock(90);
            move_backward(105);               
            stop_moving();
            blue_count += 1;
            move_forward(15);
            turn_robot_clock(90);
            delay(1000);
            wall_follower(8);
            turn_robot_anticlock(90);*/

            /*open_servo(); 
            sweep();
            turn_robot_clock(70);
            stop_moving();
            // GUESS COLOUR (RED)
            red_on();
            move_forward(70);
            stop_moving();
            close_servo();
            delay(1000);
            // turn back to wall and wall follow until robot reaches cornrer
            turn_robot_anticlock(130);
            wall_follower(8);
            turn_robot_anticlock(90);
            stop_moving();*/
            
            

            /*wall_follower(100);
            turn_robot_anticlock(90);
            stop_moving();*/

            /*if (switch_closed_2() == true) {
                red_on();
            }*/

            finish = true;
       //}
    }
}
