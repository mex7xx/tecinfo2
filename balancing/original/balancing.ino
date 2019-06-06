#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

#define EnableA 5 //PWM of Motor A
#define EnableB 6 //PWM of Motor B
#define MotorA1 4 //Direction Control of Motors A and B
#define MotorA2 3
#define MotorB1 8
#define MotorB2 9
#define Motor_Slack 45 //Compensate for the Dead zone in the PWM range
#define Kp 40 //proportional constant
#define Kd -2 //Derivative constant
#define Ki 10 //Integral constant
#define sampleTime 0.005 //sampling time 5 micro seconds
#define target_Angle -2 //The angle to keep the robot.
#define P_inc 1 // inc/dec value for PID tuning
#define D_inc 1 // inc/dec value for PID tuning
#define I_inc 1 // inc/dec value for PID tuning
#define Angle_inc .5 // inc/dec value for PID tuning
#define MS_inc 1
#define offbalance 30
#define PID_min -255
#define PID_max 255
#define Motor_min -100
#define Motor_max 100
#define runEvery(t) for (static long _lasttime;\
(uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
_lasttime += (t))
uint32_t timer;
MPU6050 mpu;

int16_t accX, accZ, gyroY; //16bit integer
volatile int motorPower, gyroRate, pidout, kp, ki, kd;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
float targetAngle, MotorSlack;



//Set left and right motors with the motor power obtained from PID.
void motorspeed(int MotorAspeed, int MotorBspeed) {
// Motor A control
  if (MotorAspeed >= 0) {
    digitalWrite(MotorA1, HIGH);
    digitalWrite(MotorA2, LOW);
  }
  else{
    digitalWrite(MotorA2, HIGH);
    digitalWrite(MotorA1, LOW);
  }
  analogWrite(EnableA, abs(MotorAspeed));
  
  // Motor B control
  if (MotorBspeed >= 0) {
    digitalWrite(MotorB2, HIGH);
    digitalWrite(MotorB1, LOW);
  }
  else {
    digitalWrite(MotorB1, HIGH);
    digitalWrite(MotorB2, LOW);
  }
  analogWrite(EnableB, abs(MotorBspeed));
}

void angle() {

  accX = mpu.getAccelerationX();
  accZ = mpu.getAccelerationZ();
  gyroY = mpu.getRotationY();
  accAngle = atan2(accX, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle); //Complimentary Filter
  prevAngle = currentAngle;

}

void init_PID() {
  // initialize Timer1
  cli(); // disable global interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  // set compare match register to set sample time 5ms
  OCR1A = 9999;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // enable global interrupts
}

void PID() {
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300); // Prevent Integral windup
  //calculate output from P, I and D values
  // pidout = Kp*(error) + Ki*(errorSum)*sampleTime + Kd *gyroRate;
  pidout = kp*(error) + ki*(errorSum)*sampleTime + kd *gyroRate;
  
  pidout = constrain(pidout, PID_min, PID_max); //constrain the PWM output
  motorPower = map(pidout, PID_min, PID_max, Motor_min, Motor_max);
  
  //Check for off balance conditon
  /* if(currentAngle > offbalance)
  motorPower = 0; //turn off the motors
  */
  // Compensate for DC motor "dead" zone
  
  if (motorPower > 0) motorPower = motorPower + MotorSlack;
  if (motorPower < 0) motorPower = motorPower - MotorSlack;

}


void BalancePID() {
  Serial.print ("kp");
  Serial.print (kp);
  Serial.print ("ki");
  Serial.print (ki);
  Serial.print ("kd");
  Serial.println (kd);
}

void Aerror() {
  Serial.print ("Target Angle");
  Serial.println (targetAngle);
}

void Mslack() {
  Serial.print ("MotorSlack");
  Serial.print (MotorSlack);
}


void ReceiveData (){
  if (Serial.available()) {
    char cmd = Serial.read();
    // Implement BT Commands
    switch (cmd) {
    case 'B' : // Return PID control values
      BalancePID();
      break;
    case 'P' : // Increment P multiplier
      kp = kp + P_inc;
      BalancePID();
      break;
    case 'p' : // Decriment P multiplier
      kp = kp - P_inc;
      BalancePID();
      break;
    case 'I' : // Increment I multiplier
      ki = ki + I_inc;
      BalancePID();
      break;
    case 'i' : // Decriment I multiplier
      ki = ki - I_inc;
      BalancePID();
      break;
    case 'D' : // Increment D multiplier
      kd = kd + D_inc;
      BalancePID();
      break;
    case 'd' : // Decriment D multiplier
      kd = kd - D_inc;
      BalancePID();
      break;
    case 'A' : // Return Roll Error value
      Aerror();
      break;
      case 'S' : // Return Motor Slack value
      Mslack();
      break;
    case 'a' : // Increment roll error
      targetAngle = targetAngle + Angle_inc;
      Aerror();
      break;
    case 'b' : // Decrement roll error
      targetAngle = targetAngle - Angle_inc;
      Aerror();
      break;
    case 's' : // Increment motor slack
      MotorSlack = MotorSlack + MS_inc;
      Mslack();
      break;
    case 't' : // Increment motor slack
      if (MotorSlack >= 1) MotorSlack = MotorSlack - MS_inc;
      Mslack();
      break;
    
    default :
      Serial.println (cmd);
    }
    cmd = ' ' ; // clear BT receive string
  }
}

void setup() {
  Serial.begin(9600);
  
  // set the motor control and PWM pins to output mode
  pinMode(EnableA, OUTPUT);
  pinMode(EnableB, OUTPUT);
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);
  
  digitalWrite(EnableA, HIGH);
  digitalWrite(EnableB, HIGH);
  digitalWrite(MotorA1, HIGH);
  digitalWrite(MotorA2, HIGH);
  digitalWrite(MotorB1, HIGH);
  digitalWrite(MotorB2, HIGH);
  
  
  // set the status LED to output mode
  pinMode(13, OUTPUT);
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setXAccelOffset(-3638);
  mpu.setYAccelOffset(-667);
  mpu.setZAccelOffset(591);
  mpu.setXGyroOffset(217);
  mpu.setYGyroOffset(18);
  mpu.setZGyroOffset(-2);
  init_PID(); // initialize PID sampling loop
  kp=Kp;
  ki=Ki;
  kd=Kd;
  targetAngle = target_Angle ;
  MotorSlack = Motor_Slack;
}


void loop() {

  runEvery(5){
    angle();
  }
  ReceiveData();
  motorspeed(motorPower, motorPower);

}

ISR (TIMER1_COMPA_vect) {
  PID();
  count++;
  if(count == 200) {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}
