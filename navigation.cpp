
#include "navigation.h"

volatile long encoder_ticks_r = 0;
volatile long encoder_ticks_l = 0;
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *r_motor = AFMS.getMotor(PORT_MOTOR_R);
Adafruit_DCMotor *l_motor = AFMS.getMotor(PORT_MOTOR_L);
Servo myservo;

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



