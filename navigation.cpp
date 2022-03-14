#include "navigation.h"

volatile long encoder_ticks_r = 0;
volatile long encoder_ticks_l = 0;
long duration;
int distance;
int distance_2;
int ledState = LOW;
unsigned long previousMillis = 0;
const long interval = 4000;
long diag_timer = 0;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *r_motor = AFMS.getMotor(PORT_MOTOR_R);
Adafruit_DCMotor *l_motor = AFMS.getMotor(PORT_MOTOR_L);
Servo myservo;

// ENCODER CODE

// int last_r = 0;
// int last_l = 0;

bool done = false;

int ticks_per_rev = 40;
double wheel_distance = 176; //mm

// RIGHT WHEEL

double r_wheel_r = 35; //mm

volatile long r_ticks = 0;
double r_mm_per_tick = 2*3.1415926*r_wheel_r/ticks_per_rev;

bool r_low = false;
bool r_low_prev = false;
double r_target = 0;
double r_speed = 0;
int r_dir = FORWARD;


// LEFT WHEEL

double l_wheel_r = 35; //mm

double l_mm_per_tick = 2*3.1415926*l_wheel_r/ticks_per_rev;
volatile long l_ticks = 0;

bool l_low = true;
bool l_low_prev = false;
double l_target = 0;
double l_speed = 0;
int l_dir = BACKWARD;

//ODOM

int odom_delta_t = 0;
double x = 0;
double target_x = 0;
double start_x = 0;
double last_x = 0;
double y = 0;
double target_y = 0;
double start_y = 0;
double last_y = 0;
double theta = 0;
double meanDistance, SR, SL;

double r_ticks_prev = 0;
double l_ticks_prev = 0;

// PIDs

byte DRIVE_STRAIGHT = 0;
byte TURN_IN_PLACE = 1;

byte mode = DRIVE_STRAIGHT;

double zero_error_straight = 0;
double deviation_from_line = 0;
double target_theta = 0;
double line_correction_speed = 0;
double turn_speed = 0;

double speed = 0;
double cur_dist = 0;
double tar_dist = 0;
double start_dist = 0;
//Specify the links and initial tuning parameters
double Kp=50, Ki=0, Kd=0;
//PID PIDr(&r_ticks, &r_speed, &r_target, 60, Ki, Kd, DIRECT);
//PID PIDl(&l_ticks, &l_speed, &l_target, 35, Ki, Kd, DIRECT);
PID PID_drive_straight(&cur_dist, &speed, &start_dist, 4.5, Ki, Kd, DIRECT);

double theta_delta = 0;
int drift_speed_range = 50;
// TODO TUNE THIS

//int l_motor_offset = 5;
int l_motor_offset = 20;
bool backwards = false;
bool r_backwards = false;
bool l_backwards = false;


PID PID_sideways_drift(&deviation_from_line, &line_correction_speed, &zero_error_straight, 0.45, 0.025, 0.19, DIRECT);
//PID PID_sideways_drift(&deviation_from_line, &line_correction_speed, &zero_error_straight, 0,0,0, DIRECT);


PID PID_turn_in_place(&theta, &turn_speed, &target_theta, 120, 0, 0.0, DIRECT);

int max_speed = 200;


void tick_right() {
  if (r_dir != BACKWARD)
    r_ticks += 1;
   else
    r_ticks -= 1;

   updateOdometry();
}

void tick_left() {
  if (l_dir != BACKWARD)
    l_ticks += 1;
   else
    l_ticks -= 1;
//
   updateOdometry();
}
void updateOdometry()
{

  SR = r_mm_per_tick * (r_ticks - r_ticks_prev);
  SL = l_mm_per_tick * (l_ticks - l_ticks_prev);
    
  l_ticks_prev = l_ticks;
  r_ticks_prev = r_ticks;

    theta += (SR - SL) / wheel_distance;
    
  if(theta > 6.29)
    theta -= 6.28;
  else if(theta < 0)
    theta += 6.28;

    meanDistance = (SL + SR)/2;
    last_x = x;
    last_y = y;
    x += meanDistance*cos(theta);
    y += meanDistance*sin(theta);
}


void drive() {
  mode = DRIVE_STRAIGHT;
  
//  tick_drive();
  
}

bool crossed_line = false;

double distance_to_line(double line_start_x, double line_start_y, double line_end_x, double line_end_y, double x, double y)
{
  double normalLength = sqrt(pow(line_end_x - line_start_x, 2) + pow(line_end_y - line_start_y, 2));
  double distance = (double)((x - line_start_x) * (line_end_y - line_start_y) - (y - line_start_y) * (line_end_x - line_start_x)) / normalLength;
  return distance;
}

double shortest_distance(double x1, double y1,
                       double a, double b,
                       double c) {
    double d = (a * x1 + b * y1 + c) /
             (sqrt(a * a + b * b));
    
//    Serial.print("Perpendicular distance is ");
//    Serial.println(d);
    return d;
}
 

void tick_drive() {
//  PID PIDstraight(&cur_dist, &speed, &start_dist, 10, Ki, Kd, DIRECT);


    // TODO backwards backing up
    cur_dist = start_dist - sqrt(pow((target_x - x), 2) + pow((target_y - y), 2));
    crossed_line = 0 > shortest_distance(x, y, target_x - start_x, target_y - start_y, target_x*(start_x - target_x) - target_y*(target_y - start_y));
//    deviation_from_line = shortest_distance(x, y, target_y - start_y, start_x - target_x, start_y*(target_x - start_x) - target_y + start_y) ;
    deviation_from_line = distance_to_line(start_x, start_y, target_x, target_y, x, y);
//    Serial.print(distance_to_line(start_x, start_y, target_x, target_y, x, y));
//    Serial.print(", ");
//    Serial.println(deviation_from_line);
    if (cur_dist >= start_dist-1|| !crossed_line) {
//      cur_dist = start_dist;
//      deviation_from_line = 0;
      done = true;
      run_navigation();

      #ifdef DEBUG
//      print_debug();
      #endif
    }
//    deviation_from_line = shortest_distance(x, y, target_x - start_x, start_y - target_y, start_x*(target_y - start_y) - start_y*(target_x - start_x)) ;

}



void setup_motor() {
  // setup encoders
  pinMode(ENCODER_R_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), tick_right, CHANGE);

  pinMode(ENCODER_L_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), tick_left, CHANGE);

  PID_drive_straight.SetMode(AUTOMATIC);
  PID_drive_straight.SetOutputLimits(100, 200);

  PID_sideways_drift.SetMode(AUTOMATIC);
  PID_sideways_drift.SetOutputLimits(-drift_speed_range, drift_speed_range);
  
  PID_turn_in_place.SetMode(AUTOMATIC);
  PID_turn_in_place.SetOutputLimits(-200, 200);
  AFMS.begin(); 
  delay(100);
  x = 0;
  y = 0;
  theta = 0;
}

int dest_dist = 1000;

// in radians
int CCW = 1;
int CW = 0;
int turn_dir = CCW;
double start_theta = 0;


void run_navigation() {
  updateOdometry();
    //  PID PIDstraight(&cur_dist, &speed, &start_dist, 10, Ki, Kd, DIRECT);
//    if (r_ticks != r_ticks_prev || l_ticks != l_ticks_prev) {
//        readTicks();
        if (done && (mode == DRIVE_STRAIGHT || mode == TURN_IN_PLACE)) {
          r_speed = 0;
          l_speed = 0;
        } else if (mode == DRIVE_STRAIGHT) {
          Serial.println("drive straight");
          tick_drive();
          turn_speed = 0;
          PID_drive_straight.Compute();
          PID_sideways_drift.Compute();

          r_speed = speed;
          l_speed = speed;

          if (r_speed > max_speed) {
            r_speed = max_speed;
          }
          
          if (l_speed > max_speed) {
            l_speed = max_speed;
          }
          if (true) {
            l_speed += l_motor_offset;  // offset due to motor difference
            r_speed -= line_correction_speed;
            l_speed += line_correction_speed;
          } else {
            l_speed -= l_motor_offset;  // offset due to motor difference
            r_speed += line_correction_speed;
            l_speed -= line_correction_speed;
          }
          if(!backwards) {
            r_dir = FORWARD;
          } else {
            r_dir = BACKWARD;
          }
          
          if(!backwards) {
            l_dir = FORWARD;
          } else {
            l_dir = BACKWARD;
          }
          
        } else if (mode == TURN_IN_PLACE) {
          line_correction_speed = 0;

          theta_delta = theta - target_theta;
          if (theta_delta < 0)
          
          if (turn_dir == CCW) { // turn CCW
            r_backwards = true;
            l_backwards = false;
            r_dir = BACKWARD;
            l_dir = FORWARD;
          } else {
            l_backwards = true;
            r_backwards = false;
            r_dir = FORWARD;
            l_dir = BACKWARD;
          }
//          delay(10000000);
//          theta_delta = fabs(theta - target_theta);
          PID_turn_in_place.Compute();
//          if (fabs(turn_speed) < 10)
          turn_speed = 130;
          r_speed = +fabs(turn_speed);
//          r_speed += 5;  // offset due to motor difference
          l_speed = +fabs(turn_speed);
//          Serial.println(fabs(theta - target_theta));
          if (start_theta != theta && fabs(theta_delta) < 0.03) {
            Serial.println(theta_delta);
            Serial.println("TURN target reached");
            done = true;
            turn_speed = 0;
            r_speed = 0;
            l_speed = 0;
            print_debug();
          }
        }
//    }
    

    
//    r_speed = 180;
    r_motor->setSpeed(r_speed);
    r_motor->run(r_dir);
    l_motor->setSpeed(l_speed);
    l_motor->run(l_dir);
  
}

void zero_position() {
    x = 0;
    y = 0;
    theta = 0;
    r_ticks = 0;
    r_ticks_prev = 0;
    target_x = 0;
    cur_dist = 0;
    start_dist = 0;
    target_y = 0;
    target_theta = 0;
    start_theta = 0;
    l_ticks = 0;
    l_ticks_prev = 0;
    start_y = 0;
    target_y = 0;
    start_x = 0;
    target_x = 0;
    
}
void drive_forward_encoder(int mm) {
  Serial.println("DRIVING FORWARD");
  done = false;
  double dx = mm*cos(theta);
  double dy = mm*sin(theta);
  
  target_x = x + dx;
  target_y = y + dy;
  start_x = x;
  start_y = y;
  Serial.print(target_x);
  Serial.print(", ");
  Serial.print(target_y);
  Serial.print(", ");
  Serial.print(start_x);
  Serial.print(", ");
  Serial.print(start_y);
  Serial.println(", ");
  mode = DRIVE_STRAIGHT;
  cur_dist = 0;
  start_dist = sqrt(pow((dx), 2) + pow((dy), 2));
//  set_target(x + dx, y + dy);
  while (!done) {
    run_navigation();
    if (millis() - diag_timer > 250 && !done) {
      print_debug();
      
      diag_timer = millis();
    }
  }
  
  deviation_from_line=0;
  run_navigation();
        print_debug();
  Serial.println("DONE FORWARD");
}


void drive_forward_smart(int mm) {
  Serial.println("DRIVING FORWARD");
  done = false;
  
  mode = DRIVE_STRAIGHT;
  
  while (!done) {
    run_navigation();
    if (millis() - diag_timer > 250 && !done) {
      print_debug();
      
      diag_timer = millis();
    }
  }
  run_navigation();
        print_debug();
  Serial.println("DONE FORWARD");
}


void turn(double tar_theta) {
  // TODO
//  return;
  mode = TURN_IN_PLACE;
  target_theta = tar_theta;
  Serial.println(tar_theta);
  start_theta = theta;
  PID_turn_in_place.Compute();
  if (target_theta - theta < 0) {
    turn_dir = CW;
    Serial.println("CW");
  } else {
    turn_dir = CCW;
    Serial.println("CCW");
  }

  while (target_theta < 0)
    target_theta += 6.28;
  while (target_theta > 6.28)
    target_theta -= 6.28;
  Serial.println(turn_dir);
    Serial.println(target_theta);

  done = false;
}


void turn_robot_encoder(double degrees) {
  double rads = degrees*3.1415926/180.;
  Serial.println("START TURN");
  done = false;
  turn(theta + rads);
  mode = TURN_IN_PLACE;
  while (!done) {
    run_navigation();
    if (millis() - diag_timer > 250 && !done) {
      print_debug();
      diag_timer = millis();
    }
  }
  
  run_navigation();
  print_debug();
  Serial.println("DONE TURN");
}



void turn_robot_encoder_rads(double rads) {
  
  turn_robot_encoder(rads*180/3.1415926/180.);
}

double d_theta;

void set_target(int tar_x, int tar_y) {
  // rotate towards end point
  target_x = tar_x;
  target_y = tar_y;
  target_theta = atan2(tar_y-y, tar_x-x);
  start_dist = sqrt(pow((tar_x - x), 2) + pow((tar_y - y), 2));
  cur_dist = 0;
  start_x = x;
  start_y = y;
  
  Serial.print(target_x);
  Serial.print(", ");
  Serial.print(target_y);
  Serial.print(", ");
  Serial.print(start_x);
  Serial.print(", ");
  Serial.print(start_y);
  Serial.println(", ");
  
  Serial.print("need to turn ");
  Serial.println(target_theta - theta);
  Serial.print("need to drive ");
    Serial.println(start_dist);

  if (fabs(target_theta - theta) > 0.1) {
    turn_robot_encoder_rads(target_theta -theta);
      
  } else {
    
    Serial.print("skipping turning");
  }
    
  delay(2000);
//  Serial.println("now driving");
//  drive_forward(start_dist);
  
  drive_forward_smart(start_dist);
}



// ENCODER CODE


void setup_sensors()
{
    // put your setup_sensors code here
    pinMode(PIN_IR_LINE_BR, INPUT);
    pinMode(PIN_IR_LINE_BL, INPUT);

    pinMode(trigPin_1, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echoPin_1, INPUT); // Sets the echoPin as an INPUT

    pinMode(trigPin_2, OUTPUT); // Sets the trigPin as an OUTPUT for front US sensor
    pinMode(echoPin_2, INPUT); // Sets the echoPin as an INPUT for front US sensor

    pinMode(amber_led, OUTPUT);
    pinMode(red, OUTPUT);
    pinMode(photoResistor, OUTPUT);
    pinMode(green_led, OUTPUT);
    pinMode(red_led, OUTPUT);
    
    pinMode(buttonPin, INPUT_PULLUP);

    AFMS.begin();
    myservo.attach(servoPin);
}

/*  High level behaviour  */

void line_follower()
{
    if (IR_line_sensor(PIN_IR_LINE_BL, 20) == false & IR_line_sensor(PIN_IR_LINE_BR, 150) == false)
    {
        Serial.println("FORWARD");
        set_vel_r_motor(250, true);
        set_vel_l_motor(250, true);
    }

    else if (IR_line_sensor(PIN_IR_LINE_BL, 20) == true & IR_line_sensor(PIN_IR_LINE_BR, 150) == false)
    {
        Serial.println("RIGHT");
        set_vel_r_motor(0, true);
        set_vel_l_motor(250, true);
    }
    else if (IR_line_sensor(PIN_IR_LINE_BL, 20) == false & IR_line_sensor(PIN_IR_LINE_BR, 150) == true)
    {
        Serial.println("LEFT");
        set_vel_r_motor(250, true);
        set_vel_l_motor(0, true);
    }
    else
    {
        Serial.println("STOP");
        set_vel_r_motor(0, true);
        set_vel_l_motor(0, true);
    }
}

void wall_follower(int wall_distance) {
    distance_2 = 999;
    while (distance_2 > wall_distance) {
        distance = getDetectorDist();
        if (distance > 5) {
            set_vel_l_motor(230, true);
            set_vel_r_motor(170, true);
        }
        else if (distance < 5) {
            set_vel_l_motor(170, true);
            set_vel_r_motor(230, true);
        }
        else {
            set_vel_r_motor(230, true);
            set_vel_l_motor(230, true);
        }
        delay(25);
        distance_2 = getDetectorDist2();
        amber_light();
    }
}

void move_forward(int final_count) {
    int count = 0;
    while (count < final_count) {
        set_vel_l_motor(220, true);
        set_vel_r_motor(200, true);
        count = count + 1;
        delay(25);
        amber_light();
    }
}

void move_backward(int final_count) {
    int count = 0;
    while (count < final_count) {
        set_vel_l_motor(200, false);
        set_vel_r_motor(200, false);
        count = count + 1;
        delay(25);
        amber_light();
    }
}

void stop_moving() {
    set_vel_l_motor(0, true);
    set_vel_r_motor(0, true);
    delay(1000);
}

void amber_light() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
    }
    if (ledState == LOW) {
        ledState = HIGH;
    } 
    else {
        ledState = LOW;
    }
    digitalWrite(amber_led, ledState);
}

void align_with_intersection()
{
    return; // TODO
}

/*  Mid level behaviour  */

// drive forward X mm forward using encoders
void drive_forward(int mm)
{
    drive_forward_encoder(mm); // THIS FOR NOW CALLS ENCODER !!!!
    // TODO
}

// in place, rotate X degrees
void turn_robot(float degrees)
{
    return; // TODO
}

/*  Low level behaviour  */

long get_r_encoder_ticks()
{
    return encoder_ticks_r; // TODO implement interrupts
}

long get_l_encoder_ticks()
{
    return encoder_ticks_l; // TODO implement interrupts
}

void set_vel_l_motor(int vel, bool forward)
{
    l_motor->setSpeed(vel);
    if (forward ==true)
    {
        l_motor->run(BACKWARD);
    }
    else
    {
        l_motor->run(FORWARD);
    }
}

void set_vel_r_motor(int vel, bool forward)
{
    r_motor->setSpeed(vel);
    if (forward == false)
    {
        r_motor->run(FORWARD);
    }
    else
    {
        r_motor->run(BACKWARD);
    }
}

bool IR_line_sensor(int IR_PIN, int threshold)
{
    // true correponds to white line, false corresponds to a black line
    int output_voltage = analogRead(IR_PIN);
    if (output_voltage < threshold)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void turn_robot_anticlock(float degrees)
{
    set_vel_l_motor(180, false);
    set_vel_r_motor(180, true);
    // delay(degrees/360 * 5000); //small wheels
    delay(degrees / 360 * 4600); // big wheels
    set_vel_l_motor(0, false);
    set_vel_r_motor(0, false);
    return;
}
// rotate X degrees clockwise
void turn_robot_clock(float degrees)
{
    set_vel_l_motor(180, true);
    set_vel_r_motor(180, false);
    // delay(degrees/360 * 5000); //small wheels
    delay(degrees / 360 * 4600); // big wheels
    set_vel_l_motor(0, false);
    set_vel_r_motor(0, false);
    return;
}

void sweep()
{
    distance = 999;
    while (distance > 20)
    {
        set_vel_r_motor(170, true);
        set_vel_l_motor(230, true);
        distance = getDetectorDist();
        delay(50);
    }
    set_vel_l_motor(0, true);
    set_vel_r_motor(0, true);
    delay(1000);
    // turn_robot_anticlock(90);
}

int getDetectorDist()
{
    digitalWrite(trigPin_1, LOW); // Clears the trigPin condition
    delayMicroseconds(2);       // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin_1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_1, LOW);
    duration = pulseIn(echoPin_1, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
    distance = duration * 0.034 / 2; // Calculating the distance. Speed of sound wave divided by 2 (go and back)
    return distance;                // Displays the distance on the Serial Monitor
}

int getDetectorDist2()
{
    digitalWrite(trigPin_2, LOW); // Clears the trigPin condition
    delayMicroseconds(2);       // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin_2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_2, LOW);
    duration = pulseIn(echoPin_2, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
    distance = duration * 0.034 / 2; // Calculating the distance. Speed of sound wave divided by 2 (go and back)
    delay(5);
    return distance;                // Displays the distance on the Serial Monitor
}


bool is_block_red() {
    digitalWrite(red, HIGH);
    delay(1000);
    double ambient = analogRead(photoResistor);
    delay(50);
    if (ambient < 500) {
      return true;
    }
    else {
      return false;
    }
}

void red_on() {
    digitalWrite(green_led, LOW);
    digitalWrite(red_led, HIGH);
    delay(5000);
    digitalWrite(green_led, LOW);
    digitalWrite(red_led, LOW);
}

void green_on() {
    digitalWrite(green_led, HIGH);
    digitalWrite(red_led, LOW);
    delay(5000);
    digitalWrite(green_led, LOW);
    digitalWrite(red_led, LOW);
}

void open_servo() {
    myservo.write(165);
}

void close_servo() {
    myservo.write(30);
}

// TODO Sensors
// Colour sensor + gripping in general
//   is_block_red, is_block_blue, is_block_present_in_gripper, ...


void print_debug() {
  
      Serial.print(r_ticks);
    //   Serial.print("(");
    //   Serial.print(r_ticks-last_r);
    //   last_r = r_ticks;
    //   Serial.print(") ");
      
      Serial.print(l_ticks);
    //   Serial.print("(");
    //   Serial.print(l_ticks-last_l);
    //   last_l = l_ticks;
    //   Serial.print(") ");
      
      Serial.print(r_ticks - l_ticks);
      Serial.print(" | ");
//      Serial.println();
      Serial.print(x);
      Serial.print(" ");
  
      Serial.print(y);
      Serial.print(" ");
  
      Serial.print(theta);
      
      Serial.print("(");
      Serial.print(target_theta);
//      last_l = l_ticks;
      Serial.print(") ");
      Serial.print("(");
      Serial.print(theta_delta);
//      last_l = l_ticks;
      Serial.print(") ");
      Serial.print(" -- cur_dist=");
////      
////      Serial.print(l_ticks);
////      Serial.print(", ");
////      
////      Serial.print(l_ticks_prev);
////      Serial.print(" | ");
      Serial.print(cur_dist);
      Serial.print(", start_dist=");
      
//      Serial.print(tar_dist);
//      Serial.print(", start_dist=");
      Serial.print(start_dist);
      Serial.print(", deviation=");
      Serial.print(deviation_from_line);
      Serial.print(" || ");
      
      Serial.print(r_speed);
      Serial.print(", ");
      Serial.print(l_speed);
      Serial.print(", ");
//      
      Serial.print(line_correction_speed);
      Serial.print(", ");
      Serial.print(turn_speed);
      Serial.print(", ");
      
      Serial.print(done);
      Serial.print(", ");
//            Serial.print(" || ");
//      Serial.print(target_theta);

      Serial.println();
}
