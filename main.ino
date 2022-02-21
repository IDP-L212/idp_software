// variable declaration
// Change to new pin locations when using on new hardware
int sensor_left = 6;
int sensor_right = 7;
int motor_right = 12;
int motor_left = 13;
void setup()
{
    // put your setup code here, to run once:
    pinMode(sensor_left, INPUT);
    pinMode(sensor_right, INPUT);
    pinMode(motor_left, OUTPUT);
    pinMode(motor_right, OUTPUT);
    Serial.begin(9600);
}

void loop()
{
    // put your main code here, to run repeatedly:

    if (digitalRead(sensor_left) == LOW & digitalRead(sensor_right) == LOW)
    {
        Serial.println("FORWARD");
        digitalWrite(motor_left, HIGH);
        digitalWrite(motor_right, HIGH);
    }

    else if (digitalRead(sensor_right) == LOW & digitalRead(sensor_left) == HIGH)
    {
        Serial.println("RIGHT");
        digitalWrite(motor_right, LOW);
        digitalWrite(motor_left, HIGH);
    }
    else if (digitalRead(sensor_right) == HIGH & digitalRead(sensor_left) == LOW)
    {
        Serial.println("LEFT");
        digitalWrite(motor_right, HIGH);
        digitalWrite(motor_left, LOW);
    }
    else
    {
        Serial.println("STOP");
        digitalWrite(motor_right, LOW);
        digitalWrite(motor_left, LOW);
    }
}
