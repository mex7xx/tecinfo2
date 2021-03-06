#include <Wire.h>
#include <math.h>

// -------------------PINs Motor --------------
#define MotorA D1
#define MotorDirA D3 //HIGH = Forward ; LOW = Backward
#define MotorB D2
#define MotorDirB D4
#define EnableMotorAB D0
#define StepPin D5    //StepWidth

// Select SDA and SCL pins for I2C communication 
#define scl D6
#define sda D7

//#define Motor_Slack 45 //Compensate for the Dead zone in the PWM range
#define Kp 1 //proportional constant
#define Kd 0 //Derivative constant
#define Ki 0 //Integral constant
#define sampleTime 0.005 //sampling time 5 micro seconds
#define target_Angle 0 //The angle to keep the robot.
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





// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;


// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_GYRO_XOUT_H  =  0x43;
const uint8_t MPU6050_REGISTER_GYRO_YOUT_H = 0x45;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;


int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;
volatile int motorPower, gyroRate =0, pidout, kp, ki, kd;
volatile float accAngle = 0.0, gyroAngle=0.0, currentAngle = 0.0, error, prevError=0, errorSum=0;
float targetAngle, MotorSlack;

int16_t gyroCalli = 0;


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
    delayMicroseconds(500); 
    digitalWrite(MotorA,LOW); 
    digitalWrite(MotorB,LOW);
    delayMicroseconds(500); 
  }
 
}

void angle() {
  /*
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  double Ax = (double)AccelX/AccelScaleFactor;
  double Az = (double)AccelZ/AccelScaleFactor;
  accAngle = calcAccAngle(Ax, Az);
  gyroRate = (GyroY - gyroCalli) / GyroScaleFactor;
  gyroAngle = (float)gyroRate*0.005;
  currentAngle = 0.9934*(currentAngle + gyroAngle) + 0.0066*(accAngle);
  */
  Read_OneRawValueYGyro(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_YOUT_H);
  gyroRate = (GyroY - gyroCalli) / GyroScaleFactor;
  gyroAngle = (float)gyroRate*sampleTime;
  currentAngle = currentAngle + gyroAngle;
}

//configure Timerinterrupt for PID Calculation
void Timer_Init(){
  timer1_attachInterrupt(onTimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  timer1_write(25000); //5ms
}

void PID() {
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300); // Prevent Integral windup
  //calculate output from P, I and D values
  //pidout = Kp*(error) + Ki*(errorSum)*sampleTime + Kd *gyroRate;
  pidout = kp*(error) + ki*(errorSum)*sampleTime + kd *gyroRate;

  // --------------------------------------------------------------------------------------- ACHTUNG ------------------------------------------------
  /*
  pidout = constrain(pidout, PID_min, PID_max); //constrain the PWM output
  motorPower = map(pidout, PID_min, PID_max, Motor_min, Motor_max);
  
  //Check for off balance conditon
  /* if(currentAngle > offbalance)
  motorPower = 0; //turn off the motors
  */

  /*
  // Compensate for DC motor "dead" zone
  
  if (motorPower > 0) motorPower = motorPower + MotorSlack;
  if (motorPower < 0) motorPower = motorPower - MotorSlack;
  */
  // --------------------------------------------------------------------------------------- ACHTUNG ------------------------------------------------
  motorPower = pidout;
  if (motorPower < 0) {
    motorPower = -4;
  } else {
    motorPower = 4;
  }
}





void setup() {
  Serial.begin(9600);


  // set the motor control and PWM pins to output mode
  pinMode(MotorA, OUTPUT);
  pinMode(MotorDirA, OUTPUT);
  pinMode(MotorB, OUTPUT);
  pinMode(MotorDirB, OUTPUT);

  pinMode(StepPin,OUTPUT);
  digitalWrite(StepPin, HIGH);
  pinMode(EnableMotorAB, OUTPUT);
  digitalWrite(EnableMotorAB, LOW);
  

  
  // set the status LED to output mode
  pinMode(13, OUTPUT);

  Wire.begin(sda, scl);

  // initialize the MPU6050 and set offset values
  MPU6050_Init();
  
  //Calibrate the GyroSensor, calc the failure
  callibrateGyroValues();
  
 

  //set PID Values
  kp=Kp;
  ki=Ki;
  kd=Kd;

  
  targetAngle = target_Angle ;
 // MotorSlack = Motor_Slack;
  
  Timer_Init(); // initialize PID sampling loop

  
}


void loop() {
/*
  runEvery(5){
    angle();
  }
  */
 // ReceiveData();
 Serial.print (currentAngle);
 Serial.print ("motor: ");
 Serial.println (motorPower); 
 motorspeed(motorPower);
  

}

void onTimerISR(){
  angle();
  PID();
}



//-----------------------HelperFunctions --------------------------------

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}


// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

void Read_OneRawValueYGyro(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)2);
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
}

void callibrateGyroValues(){
  Serial.print("calli");
  delay(150);
  for (int i=0; i<100; i++) {
    Read_OneRawValueYGyro(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_YOUT_H);
    gyroCalli = gyroCalli + GyroY;
  }
  //Serial.print(gyroCalli);
  gyroCalli = gyroCalli/100;
  //Serial.printf("Cali: %d \n",gyroCalli);
  //delay(5);
}

float calcAccAngle(double accX, double accZ) {
  return atan2(accX, accZ)*RAD_TO_DEG;
}

//configure MPU6050
void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
