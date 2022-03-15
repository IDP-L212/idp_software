#include "navigation.h"

volatile long encoder_ticks_r = 0;
volatile long encoder_ticks_l = 0;
long duration;
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
        if (switch_closed() == true) {
            move_backward(40);
            turn_robot_anticlock(45);
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
    while (distance > 20)
    {
        set_vel_r_motor(170, true);
        set_vel_l_motor(250, true);
        distance = getDetectorDist();
        delay(50);
    }
    set_vel_r_motor(170, true);
    set_vel_l_motor(250, true);
    delay(1000);
    set_vel_l_motor(0, true);
    set_vel_r_motor(0, true);
    delay(1000);
    turn_robot_clock(90);
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
    turn_robot_anticlock(90);
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
    if (ambient > 250) {
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

bool button_on() {
    if (digitalRead(buttonPin) == HIGH) {
        return true;
    }
    else if (digitalRead(buttonPin) == LOW) {
        return false;
    }
}

// TODO Sensors
// Colour sensor + gripping in general
//   is_block_red, is_block_blue, is_block_present_in_gripper, ...