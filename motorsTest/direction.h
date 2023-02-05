#ifndef __DIRECTION_H
#define __DIRECTION_H


void forward(int speed) 
{
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

void back(int speed) 
{
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

void left(int speed) 
{
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

void right(int speed) 
{
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

void forwardRight(int speed) 
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

void forwardLeft(int speed) 
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

void backwardRight(int speed) 
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

void backwardLeft(int speed) 
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