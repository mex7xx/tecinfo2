/*  Simple Stepper Motor Control
 *  Mit dem A4988 Treiber Baustein
 *  PHOF Basiert auf dem Code von
 *  Dejan Nedelkovski, www.HowToMechatronics.com
 */

// defines pins numbers
const int stepPin = 3; 
const int dirPin = 4; 

const int MS1 = 8; 
const int MS2 = 9; 
const int MS3 = 10; 
 
void setup() {
   Serial.begin(9600);
   pinMode(stepPin,OUTPUT); 
   pinMode(dirPin,OUTPUT);
   pinMode(MS1,OUTPUT);
   pinMode(MS2,OUTPUT);
   pinMode(MS3,OUTPUT);
}
void loop() {
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 200; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(500); 
  }
  delay(1000); // One second delay
  
  digitalWrite(dirPin,LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 400; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(500);
  }
  delay(1000);
  
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 200; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(500); 
  }
  delay(1000); // One second delay
}
