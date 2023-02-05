#include "direction.h"


// Motors
int motor1Pin1 = 4; 
int motor1Pin2 = 2; 
int enable1Pin = 15;

int motor2Pin1 = 19; 
int motor2Pin2 = 18; 
int enable2Pin = 5;

int motor3Pin1 = 13; 
int motor3Pin2 = 12; 
int enable3Pin = 14;

int motor4Pin1 = 27; 
int motor4Pin2 = 26; 
int enable4Pin = 25;



// Setting PWM properties
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int pwmChannel3 = 2;
const int pwmChannel4 = 3;

const int freq = 30000;
const int resolution = 8;
int dutyCycle = 200;

void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);
  pinMode(enable3Pin, OUTPUT);

  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Pin2, OUTPUT);
  pinMode(enable4Pin, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);
  ledcSetup(pwmChannel3, freq, resolution);
  ledcSetup(pwmChannel4, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin1, pwmChannel1);
  ledcAttachPin(enable1Pin2, pwmChannel2);
  ledcAttachPin(enable1Pin3, pwmChannel3);
  ledcAttachPin(enable1Pin4, pwmChannel4);

}

void loop() {
  // put your main code here, to run repeatedly:
  forwards();
  delay(5000);
  backwards();
  delay(5000);
  left();
  delay(5000);
  right();
  delay(5000);
}

