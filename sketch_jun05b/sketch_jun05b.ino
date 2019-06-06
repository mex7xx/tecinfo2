#include <Ticker.h> 

Ticker ticker;

int pid_loop_int_millis = 10000; // PID Loop is triggerd every 0.01 sec
float desired_angle = 0.0;
float eint, eprev;
float kp,ki,kd;



int high_pulse_lenght_micro = 2;
int low_pulse_lenght_micro;


void setup() {
  // put your setup code here, to run once:
  //timer1_attachInterrupt(
  timer1_write(pid_loop_int_millis);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  move_stepper();
}

// sets pins on motor driver
void move_stepper() {

  // set low_pulse and high_pulse for stepper input for a given time
  // in or decrease step_counter // will be used for correcting drift

}


// PID Controller-Loop
void pid_loop() {
  
  float e = desired_angle - read_angle();
  
  // Probalbly set maxfor eint to avoid
  eint = eint + e * pid_loop_int_millis;

  float ediv = (e-eprev)/pid_loop_int_millis;  
  
  float u = kp*e + ki*eint + kd*ediv;
  eprev = e;
  
  // printErrorsToSerial(e,eint,ediv)
  // u [-1; 1]
  set_speed(u);
}

// read from IMU ; smooth Angle ( low-pass filter)
float read_angle() {
  
}

// Updates Direction and Speed of Stepper Motors based on u
// u = 1 (= vMax) 
void set_speed(float u) {

  // calculate low_pulse_lenght_micro based on u
}
