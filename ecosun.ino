#include <Servo.h>

Servo myservo;
int pos = 90;

static const int SENSITIVITY = 100;
int leftSensor = A0;
int rightSensor = A1;
int sensorValue;

void setup() 
{
  Serial.begin(9600);

  myservo.attach(9);
  myservo.write(pos);
}

void loop()
{
  delay(200);
  int leftRead = analogRead(leftSensor);
  int rightRead = analogRead(rightSensor);

  int diff = leftRead - rightRead;
  if (diff <= SENSITIVITY && diff >= -SENSITIVITY) {
    return;
  } 
  
  if (diff < 0 && pos < 170) {
    pos += 5;
  } else if (diff > 0 && pos > 10) {
    pos -= 5;
  }
  myservo.write(pos);
}

