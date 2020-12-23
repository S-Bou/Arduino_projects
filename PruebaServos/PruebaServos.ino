#include <Servo.h>
#define time 2000
Servo servo1;

int PINSERVO = 2;
int PULSOMIN = 1000;
int PULSOMAX = 2000;

#define LED_USER 13

void setup() {
  // put your setup code here, to run once:
  servo1.attach(PINSERVO, PULSOMIN, PULSOMAX);
pinMode(LED_USER, OUTPUT);
}
void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(LED_USER, HIGH);
  
  servo1.write(0);
  delay(time);
  digitalWrite(LED_USER, LOW);
  servo1.write(180);
  delay(time);
}
