#ifndef _DIRECTION_H
#define _DIRECTION_H


void forward() 
{
  // MotorSpeeds 
  analogWrite(pwmPin1,255);
  analogWrite(pwmPin2,255);
  analogWrite(pwmPin3,255);
  analogWrite(pwmPin4,255);

  // Drive all 4 motors Forward == Forwards
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 

  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH); 

  digitalWrite(motor3Pin1, LOW);
  digitalWrite(motor3Pin2, HIGH); 

  digitalWrite(motor4Pin1, LOW);
  digitalWrite(motor4Pin2, HIGH); 
}

void back() 
{
  // MotorSpeeds 
  analogWrite(pwmPin1,255);
  analogWrite(pwmPin2,255);
  analogWrite(pwmPin3,255);
  analogWrite(pwmPin4,255);

  // Drive all 4 motors Backward == Backwards
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 

  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW); 

  digitalWrite(motor3Pin1, HIGH);
  digitalWrite(motor3Pin2, LOW); 

  digitalWrite(motor4Pin1, HIGH);
  digitalWrite(motor4Pin2, LOW); 
}

void left() 
{
  // MotorSpeeds 
  analogWrite(pwmPin1,255);
  analogWrite(pwmPin2,255);
  analogWrite(pwmPin3,255);
  analogWrite(pwmPin4,255);
  
  // Drive motors 2 & 4 Forwards and 1 & 3 Backwards == Strafe Left
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 

  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH); 

  digitalWrite(motor3Pin1, HIGH);
  digitalWrite(motor3Pin2, LOW); 

  digitalWrite(motor4Pin1, LOW);
  digitalWrite(motor4Pin2, HIGH); 

}

void right() 
{
  // MotorSpeeds 
  analogWrite(pwmPin1,255);
  analogWrite(pwmPin2,255);
  analogWrite(pwmPin3,255);
  analogWrite(pwmPin4,255);

  // Drive motors 1 & 3 Forwards and 2 & 4 Backwards == Strafe Right
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 

  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW); 

  digitalWrite(motor3Pin1, LOW);
  digitalWrite(motor3Pin2, HIGH); 

  digitalWrite(motor4Pin1, HIGH);
  digitalWrite(motor4Pin2, LOW); 
}

void stop()
{
  // MotorSpeeds 
  analogWrite(pwmPin1,255);
  analogWrite(pwmPin2,255);
  analogWrite(pwmPin3,255);
  analogWrite(pwmPin4,255);
  // STOP ALL MOTORS == Full Stop
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW); 

  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW); 

  digitalWrite(motor3Pin1, LOW);
  digitalWrite(motor3Pin2, LOW); 

  digitalWrite(motor4Pin1, LOW);
  digitalWrite(motor4Pin2, LOW); 
}

void forwardRight() 
{
  // Drive motors 1 & 3 Forwards == forward right(NE)
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 

  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW); 

  digitalWrite(motor3Pin1, LOW);
  digitalWrite(motor3Pin2, HIGH); 

  digitalWrite(motor4Pin1, LOW);
  digitalWrite(motor4Pin2, LOW); 
}

void forwardLeft() 
{
  // Drive motors 2 & 4 Forwards == forward left(NW)
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW); 

  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH); 

  digitalWrite(motor3Pin1, LOW);
  digitalWrite(motor3Pin2, LOW); 

  digitalWrite(motor4Pin1, LOW);
  digitalWrite(motor4Pin2, HIGH); 
}

void backwardRight() 
{
  // Drive motors 2 & 4 backwards == backward right(SE)
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW); 

  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW); 

  digitalWrite(motor3Pin1, LOW);
  digitalWrite(motor3Pin2, LOW); 

  digitalWrite(motor4Pin1, HIGH);
  digitalWrite(motor4Pin2, LOW); 
}

void backwardLeft() 
{
  // Drive motors 1 & 3 backwards == backward left(SW)
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 

  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW); 

  digitalWrite(motor3Pin1, HIGH;
  digitalWrite(motor3Pin2, LOW); 

  digitalWrite(motor4Pin1, LOW);
  digitalWrite(motor4Pin2, LOW); 
}



#endif