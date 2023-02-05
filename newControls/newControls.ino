#include <Servo.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
//150, 120 straight
//0, 130 left
//120, 0 right

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = true;
boolean printDebugging = true;
float serialInput[2]; //0: linear, 1: angular

int motor1Pin1 = 4; 
int motor1Pin2 = 2; 
int pwmPin1 = 3;

int motor2Pin1 = 7; 
int motor2Pin2 = 6; 
int pwmPin2 = 5;


void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(pwmPin1, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
}

void loop() {
  recvWithEndMarker();
  showNewData();
  if (printDebugging){
    Serial.println(serialInput[0]);
    Serial.println(serialInput[1]);
  }
  
  drive(serialInput[0], serialInput[1]);
  delay(50);
}


// -------------------------------------------------------------------

void drive(int leftSpeed, int rightSpeed) 
{
  if (rightSpeed >= 0 && leftSpeed >= 0) // Forward
  {
    // MotorSpeeds 
      analogWrite(pwmPin1,leftSpeed);
      analogWrite(pwmPin2,rightSpeed);

      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH); 

      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      if (printDebugging){
        Serial.println("Forward");
      }
  }
  else if (rightSpeed < 0 && leftSpeed < 0) // Backwards
  {
    // MotorSpeeds 
      analogWrite(pwmPin1,leftSpeed);
      analogWrite(pwmPin2,rightSpeed);

      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW); 

      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      if (printDebugging){
        Serial.println("Backward");
      }
  }
  else if (rightSpeed >= 0 && leftSpeed < 0) // Pivot Left
  {
    // MotorSpeeds 
      analogWrite(pwmPin1,leftSpeed);
      analogWrite(pwmPin2,rightSpeed);

      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH); 

      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      if (printDebugging){
        Serial.println("Left");
      }
  }
  else if (rightSpeed < 0 && leftSpeed >= 0) // Pivot Right
  {
    // MotorSpeeds 
      analogWrite(pwmPin1,leftSpeed);
      analogWrite(pwmPin2,rightSpeed);

      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW); 

      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
      if (printDebugging){
        Serial.println("Right");
      }
  }
  else if (rightSpeed == 0 && leftSpeed == 0) // STOP
  {
    // MotorSpeeds 
      analogWrite(pwmPin1,leftSpeed);
      analogWrite(pwmPin2,rightSpeed);

      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, LOW); 

      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      if (printDebugging){
        Serial.println("Stop");
      }
  }
  else {
    // MotorSpeeds 
      analogWrite(pwmPin1,leftSpeed);
      analogWrite(pwmPin2,rightSpeed);

      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, LOW); 

      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      if (printDebugging){
        Serial.println("Stop2");
      }
  }
      
}

// -------------------------------------------------------------------



void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char startMarker = '<';
    char rc;
    bool gottenStart = false;
    
    while (Serial.available() > 0 && newData == false) {
        //Serial.println("while loop start");
        rc = Serial.read();
        //Serial.println(rc);

        if (rc == startMarker){
          gottenStart = true;
          //Serial.println("gotten start");
        }

        if (!gottenStart){
          //Serial.println("hasn't gotten start yet");
          continue;
        }
        //Serial.println("has gotten start");

        

        if (rc != endMarker) {
            if (rc != startMarker){
              receivedChars[ndx] = rc;
              ndx++;
              if (ndx >= numChars) {
                  ndx = numChars - 1;
              }
            }
            
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        //Serial.println(receivedChars);
        char* velVal = strtok(receivedChars, ",");
        char* firstValChar = velVal;
        velVal = strtok(NULL, ",");
        char* secondValChar = velVal;
        float firstFloat = atof(firstValChar);
        float secondFloat = atof(secondValChar);
        //Serial.print("first float: ");
        //Serial.print(firstFloat);
        //Serial.print(" | second float: ");
        //Serial.println(secondFloat);

        serialInput[0] = firstFloat;
        serialInput[1] = secondFloat;
        newData = false;
    }
}