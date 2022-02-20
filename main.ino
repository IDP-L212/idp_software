//Main executable file to be uploaded to Arduino Uno Board
#include "lent_212_library.h"

void setup()
{
    Serial.begin(9600);
    int sum = add_two_integers(1,2);
    Serial.println(sum);
}

void loop(){}