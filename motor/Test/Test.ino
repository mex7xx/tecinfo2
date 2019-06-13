#include <ESP.h>

#define MotorA D1
#define MotorDirA D3 //HIGH = Forward ; LOW = Backward
#define MotorB D2
#define MotorDirB D4
#define EnableMotorAB D0
#define StepPin D5    //StepWidth

//Set left and right motors with the motor power obtained from PID.
void motorspeed(int MotorSpeed) {
// Motor A control
  if (MotorSpeed >= 0) {
    digitalWrite(MotorDirA, HIGH);
    digitalWrite(MotorDirB, LOW);
  }
  else{
    digitalWrite(MotorDirB, HIGH);
    digitalWrite(MotorDirA, LOW);
  }
  
  for(int x = 0; x < abs(MotorSpeed); x++) {
    digitalWrite(MotorA,HIGH); 
    digitalWrite(MotorB,HIGH); 
    delayMicroseconds(100); 
    digitalWrite(MotorA,LOW); 
    digitalWrite(MotorB,LOW);
    delayMicroseconds(100); 
    ESP.wdtFeed();
  }
 
}


void setup() {
  // set the motor control and PWM pins to output mode
  pinMode(MotorA, OUTPUT);
  pinMode(MotorDirA, OUTPUT);
  pinMode(MotorB, OUTPUT);
  pinMode(MotorDirB, OUTPUT);

  pinMode(StepPin,OUTPUT);
  digitalWrite(StepPin, HIGH);
  pinMode(EnableMotorAB, OUTPUT);
  digitalWrite(EnableMotorAB, HIGH);
}
void loop() {
  motorspeed(10);
  //delay(1000);
}
