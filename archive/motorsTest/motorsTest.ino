#include <arduino.h>
// #include "direction.h"

// Motors
int motor1Pin1 = 4; 
int motor1Pin2 = 2; 
int pwmPin1 = 3;

int motor2Pin1 = 7; 
int motor2Pin2 = 6; 
int pwmPin2 = 5;

int speed = 170;

void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(pwmPin1, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
}

void loop() {
  if (serial.available()){

  }
  
  delay(2500);
  // back();
  // delay(5000);
  // left();
  // delay(5000);
  // right();
  // delay(5000);
  stop();
  delay(4000);
}


void drive(int behavior, int left, int right) 
{
  switch (behavior) {
    case 0: // Forward

    case 1: // Backward

    case 2: // Left

    case 3: // Right

    case 4: // STOP
  }
  // MotorSpeeds 
  analogWrite(pwmPin1,left);
  analogWrite(pwmPin2,right);

  // Drive all 4 motors Forward == Forwards
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 

  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
}