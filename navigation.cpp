
#include "navigation.h"

volatile long encoder_ticks_r = 0;
volatile long encoder_ticks_l = 0;
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *r_motor = AFMS.getMotor(PORT_MOTOR_R);
Adafruit_DCMotor *l_motor = AFMS.getMotor(PORT_MOTOR_L);
Servo myservo;

int distance;
int distance_2;
int ledState = LOW;
unsigned long previousMillis = 0;
const long interval = 4000;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *r_motor = AFMS.getMotor(PORT_MOTOR_R);
Adafruit_DCMotor *l_motor = AFMS.getMotor(PORT_MOTOR_L);
Servo myservo;


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
    pinMode(Lswitch, INPUT);
    pinMode(Lswitch_2, INPUT);

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

int num_triggers = 0;
int last_dis_2 = 0;
void wall_follower(int wall_distance) {
    num_triggers = 0;
    distance_2 = 999;
    last_dis_2 = 0;
    int start_time = millis();
    // istance_2 > wall_distance
    while (distance_2 > wall_distance || millis() - start_time < 5000) {
        if (distance_2 < wall_distance && distance_2 != last_dis_2) {
            num_triggers += 1;
            last_dis_2 = distance_2;
        } else {
            num_triggers = 0;
        }
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
        if (switch_closed() == true) {
            move_backward(20);
            turn_robot_anticlock(25);
            delay(25);
        }
        else if (switch_closed_2() == true) {
            move_backward(20);
            turn_robot_anticlock(75);
            delay(25);
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
        delay(30);
        amber_light();
    }
}

void move_backward(int final_count) {
    int count = 0;
    while (count < final_count) {
        set_vel_l_motor(200, false);
        set_vel_r_motor(200, false);
        count = count + 1;
        delay(30);
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
    if (forward ==false)
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
    if (forward == true)
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
    delay(degrees / 360 * 4200); // big wheels
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
    delay(degrees / 360 * 4200); // big wheels
    set_vel_l_motor(0, false);
    set_vel_r_motor(0, false);
    return;
}

void sweep()
{
    distance = 999;
    while (distance > 25)
    {
        set_vel_r_motor(170, true);
        set_vel_l_motor(250, true);
        distance = getDetectorDist();
        delay(50);
    }
    //set_vel_r_motor(170, true);
    //set_vel_l_motor(250, true);
    //delay(1000);
    set_vel_l_motor(0, true);
    set_vel_r_motor(0, true);
    delay(1000);
    /*turn_robot_clock(90);
    // move forward to capture block
    move_forward(70);
    stop_moving();
    // close servo, turn, and follow wall to corner
    close_servo();
    delay(1000);
    move_forward(100);
    //turn_robot_anticlock(130);
    stop_moving();
    move_backward(20);
    turn_robot_anticlock(90);*/
}

void reverse_sweep()
{
    distance = 999;
    while (distance > 20)
    {
        set_vel_r_motor(170, false);
        set_vel_l_motor(230, false);
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
    if (ambient > 440) {
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

bool switch_closed() {
    if (digitalRead(Lswitch) == HIGH) {
        return true;
    }
    else if (digitalRead(Lswitch) == LOW) {
        return false;
    }
}

bool switch_closed_2() {
    if (digitalRead(Lswitch_2) == LOW) {
        return true;
    }
    else if (digitalRead(Lswitch_2) == HIGH) {
        return false;
    }
}

bool button_on() {
    if (digitalRead(buttonPin) == LOW) {
        return true;
    }
    else if (digitalRead(buttonPin) == HIGH) {
        return false;
    }
}

void arc()
{
    set_vel_r_motor(150, true);
    set_vel_l_motor(250, true);
    delay(8000);
}

// encoder code

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

double theta_delta = 0;
int drift_speed_range = 50;
// TODO TUNE THIS

//int l_motor_offset = 5;
int l_motor_offset = 20;
bool r_backwards = false;
bool l_backwards = false;


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


bool crossed_line = false;

double shortest_distance(double x1, double y1,
                       double a, double b,
                       double c) {
    double d = (a * x1 + b * y1 + c) /
             (sqrt(a * a + b * b));

//    Serial.print("Perpendicular distance is ");
//    Serial.println(d);
    return d;
}


double distance_to_line(double line_start_x, double line_start_y, double line_end_x, double line_end_y, double x, double y)
{
  double normalLength = sqrt(pow(line_end_x - line_start_x, 2) + pow(line_end_y - line_start_y, 2));
  double distance = (double)((x - line_start_x) * (line_end_y - line_start_y) - (y - line_start_y) * (line_end_x - line_start_x)) / normalLength;
  return distance;
}

bool initial_crossed_line = false;

void tick_drive() {
  
    cur_dist = start_dist - sqrt(pow((target_x - x), 2) + pow((target_y - y), 2));
    crossed_line = 0 > shortest_distance(x, y, target_x - start_x, target_y - start_y, target_x*(start_x - target_x) - target_y*(target_y - start_y));
//    deviation_from_line = shortest_distance(x, y, target_y - start_y, start_x - target_x, start_y*(target_x - start_x) - target_y + start_y) ;
    deviation_from_line = distance_to_line(start_x, start_y, target_x, target_y, x, y);
//    Serial.print(distance_to_line(start_x, start_y, target_x, target_y, x, y));
//    Serial.print(", ");
//    Serial.println(deviation_from_line);
    if (crossed_line != initial_crossed_line) {
//      cur_dist = start_dist;
//      deviation_from_line = 0;
      done = true;
      run_navigation();

      #ifdef DEBUG
      print_debug();
      #endif
    }

}



void setup_motor() {
  // setup encoders
  pinMode(ENCODER_R_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), tick_right, CHANGE);

  pinMode(ENCODER_L_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), tick_left, CHANGE);

  AFMS.begin(); 
  delay(100);
  zero_position();
}

int dest_dist = 1000;

// in radians
int CCW = 1;
int CW = 0;
int turn_dir = CCW;
double start_theta = 0;


void run_navigation() {
  updateOdometry();
  if (done) {
    r_speed = 0;
    l_speed = 0;
  } else {
    r_speed = max_speed - (r_ticks - l_ticks);
    l_speed = max_speed + (r_ticks - l_ticks) + l_motor_offset;
  }
    
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
  initial_crossed_line = 0 > shortest_distance(x, y, target_x - start_x, target_y - start_y, target_x*(start_x - target_x) - target_y*(target_y - start_y));
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

// encoder code end
void setup_sensors()
{
    // put your setup_sensors code here
    pinMode(PIN_IR_LINE_BR, INPUT);
    pinMode(PIN_IR_LINE_BL, INPUT);

    pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

    pinMode(red, OUTPUT);
    pinMode(photoResistor, OUTPUT);
    pinMode(green_led, OUTPUT);
    pinMode(red_led, OUTPUT);
    
    AFMS.begin();
    myservo.attach(servoPin);
}

int add_two_integers(int a, int b)
{
    return a + b;
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

void align_with_intersection() {
    return;  // TODO
}


/*  Mid level behaviour  */

// drive forward X mm forward using encoders
void drive_forward(int mm) {
    return; // TODO
}

// in place, rotate X degrees anticlockwise
void turn_robot_anticlock(float degrees) {
    set_vel_l_motor(180, false);
    set_vel_r_motor(180, true);
    //delay(degrees/360 * 5000); //small wheels
    delay(degrees/360 * 10125); //big wheels
    set_vel_l_motor(0, false);
    set_vel_r_motor(0, false);
    return;
}

// rotate X degrees clockwise
void turn_robot_clock(float degrees) {
    set_vel_l_motor(180, true);
    set_vel_r_motor(180, false);
    //delay(degrees/360 * 5000); //small wheels
    delay(degrees/360 * 10125); //big wheels
    set_vel_l_motor(0, false);
    set_vel_r_motor(0, false);
    return;
}


/*  Low level behaviour  */

long get_r_encoder_ticks() {
    return encoder_ticks_r; // TODO implement interrupts
}

long get_l_encoder_ticks() {
    return encoder_ticks_l; // TODO implement interrupts
}

void set_vel_r_motor(int vel, bool forward) {
    r_motor->setSpeed(vel);
    if (forward == true) {
        r_motor->run(FORWARD);
    }
    else{
        r_motor->run(BACKWARD);
    }
}

void set_vel_l_motor(int vel, bool forward) {
    l_motor->setSpeed(vel);
    if (forward == true)
    {
        l_motor->run(FORWARD);
    }
    else
    {
        l_motor->run(BACKWARD);
    }
}

void sweep() {
    distance = 999;
    while (distance > 40) {
        set_vel_l_motor(180, false);
        set_vel_r_motor(180, true);
        distance = getDetectorDist();
        delay(100);
    }
    set_vel_l_motor(0, true);
    set_vel_r_motor(0, true);
}

int getDetectorDist() {
  digitalWrite(trigPin, LOW); // Clears the trigPin condition
  delayMicroseconds(2); // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
  distance = duration * 0.034 / 2; // Calculating the distance. Speed of sound wave divided by 2 (go and back)
  return distance; // Displays the distance on the Serial Monitor
}

bool is_block_red() {
    digitalWrite(red, HIGH);
    delay(1000);
    double ambient = analogRead(photoResistor);
    delay(50);
    if (ambient > 870) {
      return true;
    }
    else {
      return false;
    }
}

void open_servo() {
    myservo.write(30);
}

void close_servo() {
    myservo.write(165);
}

// TODO Sensors

// Line sensors -> TBD By Misha on 22nd Feb
//   is_line_detected (for right/left and front/back)
// Distance sensor (for sweep) -> done by Misha
//   get_ultrasound_distance (used for both sweep + detecting if block is in gripper)
// Colour sensor + gripping in general
//   is_block_red, is_block_blue, is_block_present_in_gripper, ...



