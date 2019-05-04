#include <Wire.h>
#include <math.h>

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D6;
const uint8_t sda = D7;

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
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;


//
const uint8_t MPU6050_REGISTER_GYRO_XOUT_H = 0x43;
//

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

int16_t gyroRate = 0;
int16_t gyroCalli = 0;
float gyroAngle = 0.0;
float accAngle = 0.0;

float currentAngle = 0.0;

//unsigned long currTime, prevTime=0, loopTime;

void setup() {
  Serial.begin(9600);
  Wire.begin(sda, scl);
  MPU6050_Init();
  callibrateGyroValues();
  Timer_Init();
}

void loop() {
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  
  //GyroRate timer interval calculation
  //currTime = millis();
  //loopTime = currTime - prevTime;
  //prevTime = currTime;
  
  
  //Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
 

  
  //divide each with their sensitivity scale factor
  //Ax = (double)AccelX/AccelScaleFactor;
  //Ay = (double)AccelY/AccelScaleFactor;
  //Az = (double)AccelZ/AccelScaleFactor;
  //T = (double)Temperature/340+36.53; //temperature formula
  //Gx = (double)GyroX/GyroScaleFactor;
  //Gy = (double)GyroY/GyroScaleFactor;
  //Gz = (double)GyroZ/GyroScaleFactor;
 
  //---
  //accAngle = calcAccAngle(Ay, Az);
  //---
  
  //---
  //gyroRate = map(GyroX, -32768, 32767, -250, 250); // brauchen wir glaub ich nicht. int16_t Range ( -32768, 32767 ) werte /131 (GyroScaleFactor) = -250 250
  //gyroAngle = gyroAngle + (float)gyroRate*loopTime/1000;
  //---

  
  /* 
   *  Print RAW Values
  */
  /*
  //Accerleration
  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.println(Az);
 // Serial.print(" T: "); Serial.print(T);
  //Gyrosensor
  Serial.print(" Gx: "); Serial.print(Gx);
  Serial.print(" Gy: "); Serial.print(Gy);
  Serial.print(" Gz: "); Serial.println(Gz);
   */

 //Serial.println(accAngle);
 //Serial.println(gyroAngle);
  Serial.print("currentAngle: "); Serial.println(currentAngle);
  //delay(5);
}

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

void Read_OneRawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)2);
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
}


float calcAccAngle(double accY, double accZ) {
  return atan2(accY, accZ)*RAD_TO_DEG;
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

//configure Interrupt
void Timer_Init(){
  timer1_attachInterrupt(onTimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  timer1_write(25000); //5ms
}

//Interrupt Service Routine on timer event
void onTimerISR(){
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);

  /*
   * Only Acceleration
   */
  //double Ay = (double)AccelY/AccelScaleFactor;
  //double Az = (double)AccelZ/AccelScaleFactor;
  //accAngle = calcAccAngle(Ay, Az);


  /*
   * Only Gyro
   */
  /*
  //Read_OneRawValue(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_XOUT_H);
  //gyroRate = map(GyroX, -32768, 32767, -250, 250); // brauchen wir glaub ich nicht. int16_t Range ( -32768, 32767 ) werte /131 (GyroScaleFactor) = -250 250
  gyroRate = (GyroX - gyroCalli) / GyroScaleFactor;
  gyroAngle = gyroAngle + (float)gyroRate*0.005;   //gyroAngle only
  currentAngle = gyroAngle;
  */

  
  /*
   * Complementary
   */
  double Ay = (double)AccelY/AccelScaleFactor;
  double Az = (double)AccelZ/AccelScaleFactor;
  accAngle = calcAccAngle(Ay, Az);
  gyroRate = (GyroX - gyroCalli) / GyroScaleFactor;
  gyroAngle = (float)gyroRate*0.005;
  currentAngle = 0.9934*(currentAngle + gyroAngle) + 0.0066*(accAngle); 
  
}

void callibrateGyroValues(){
  Serial.print("calli");
  delay(150);
  for (int i=0; i<100; i++) {
    Read_OneRawValue(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_XOUT_H);
    gyroCalli = gyroCalli + GyroX;
  }
  //Serial.print(gyroCalli);
  gyroCalli = gyroCalli/100;
  //Serial.printf("Cali: %d \n",gyroCalli);
}

