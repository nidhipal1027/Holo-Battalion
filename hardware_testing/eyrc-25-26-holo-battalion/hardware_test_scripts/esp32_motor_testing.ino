#include <ESP32Servo.h>

// Assigning servo names
Servo servo1; 
Servo servo2; 
Servo servo3; 
Servo micro_servo;

// Defining pins
#define servo1_pin  27
#define servo2_pin  26
#define servo3_pin  25
#define servo_pin   33
#define led_pin     13 

void servo_init(){
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);
  micro_servo.attach(servo_pin);
}

void test_servo(int dir){
  servo1.write(dir);
  servo2.write(dir);
  servo3.write(dir);
  micro_servo.write(dir);
  if(dir != 90){
    digitalWrite(led_pin, HIGH);
  }
  else{
    digitalWrite(led_pin, LOW);
  }
}

void setup() {
  servo_init();
  pinMode(led_pin, OUTPUT);
}

void loop() {
  test_servo(0);
  delay(1000);
  test_servo(90);
  delay(1000);
  test_servo(180);
  delay(1000);
  test_servo(90);
  delay(1000);
}