#include "encoders.h"
#include "pid.h"
#include "timer.h"
#include "kinematics.h"
//#include "line_sensors.h"


#define BAUD_RATE 9600

//----- Motor definitions -----
//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define FWD LOW
#define BCK HIGH

//----- PID defintions -----
float Kp_left = 0.5; //Proportional gain for position controller
float Kd_left = 0; //Derivative gain for position controller
float Ki_left = 0.1; //Integral gain for position controller

float Kp_right = 0.5; //Proportional gain for position controller
float Kd_right = 0; //Derivative gain for position controller
float Ki_right = 0.1; //Integral gain for position controller

float Kp_head_turn = 1; //Proportional gain for position controller
float Kd_head_turn = 0.0; //Derivative gain for position controller
float Ki_head_turn = 0.0; //Integral gain for position controller

PID left_PID(Kp_left, Ki_left, Kd_left); //Position controller for left wheel position
PID right_PID(Kp_right, Ki_right, Kd_right); //Position controller for left wheel position
PID head_turn_pid(Kp_head_turn, Ki_head_turn, Kd_head_turn); //Turning controller for update direction

float velocity_l = 0;
float velocity_r = 0;

//--- Setup global variables ---
long last_count_el = 0;
long last_count_er = 0;

int direct = 0;

//Setting up timers
unsigned long PID_timer_max = 5000;
unsigned long Kinematics_timer_max = 10000;
Timer PID_timer(PID_timer_max, false);
Timer Kinematics_timer(Kinematics_timer_max, false);
Timer Vel_timer(1000000000, false);
Timer Turn_timer(5000, true);

//Setup Kinematics class
Kinematics k;

// Set our motor driver pins as outputs.
void setupMotorPins()
{
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  digitalWrite( L_DIR_PIN, LOW  );
  digitalWrite( R_DIR_PIN, LOW );
}

void setup() {
  // These two function set up the pin
  // change interrupts for the encoders.
  setupEncoder0();
  setupEncoder1();

  // Initialise the Serial communication
  Serial.begin( BAUD_RATE );
  delay(3000);
  Serial.println("***RESET***");
  tone(6, 440, 200);
  delay(250);
  tone(6, 480, 200);
  delay(250);

  //TODO - Put wait until button pressed
  
  Turn_timer.Reset();
}

void loop() { //Change this to be a large switch statement with global state variable, (See library)
  // put your main code here, to run repeatedly:
  if(PID_timer.IsTime()){
    UpdateDirection(20,20);
  }

  if(Kinematics_timer.IsTime()){
    k.update(count_el, count_er);
  }

  if(Turn_timer.IsTime()){ //TODO Change this from a timer, line detection?
    direct = direct + 180;
    TurnInDirection(direct);
    Turn_timer.Reset();
  }
}

void UpdateDirection(int left_demand, int right_demand){
  int diff_l = (int) count_el - last_count_el;
  int diff_r = (int) count_er - last_count_er;
  last_count_el = count_el;
  last_count_er = count_er;
  
  float float_elapsed = Vel_timer.GetElapsed();

  velocity_l = (float)(diff_l/(float_elapsed)) * 100000;
  velocity_r = (float)(diff_r/(float_elapsed)) * 100000;
  
  float output_l = left_PID.update(left_demand, velocity_l);
  float output_r = right_PID.update(right_demand, velocity_r);

//  --- Debugging prints ---
//  Serial.print("Demand:");
//  Serial.print(left_demand);
//  Serial.print(",diff_l:");
//  Serial.print(diff_l);
//  Serial.print(",Elapsed:");
//  Serial.print(float_elapsed);
//  Serial.print(",velocity:");
//  Serial.print(velocity_l);
//  Serial.print(",output:");
//  Serial.print(output_l);
//  Serial.print('\n');
  
  if(output_l > 0) digitalWrite( L_DIR_PIN, FWD );
  else digitalWrite( L_DIR_PIN, BCK );
  analogWrite(L_PWM_PIN, abs(output_l));

  if(output_r > 0) digitalWrite( R_DIR_PIN, FWD );
  else digitalWrite( R_DIR_PIN, BCK );
  analogWrite(R_PWM_PIN, abs(output_r));
}

void TurnInDirection(float dir){ //Change this to be non blocking
  analogWrite(L_PWM_PIN, 0);
  analogWrite(R_PWM_PIN, 0);
  float current_head = k.update(count_el, count_er);
  PID_timer.Reset();
  Kinematics_timer.Reset();
  float output = head_turn_pid.update(dir, current_head);
  while(abs(output) > 1){
    if(PID_timer.IsTime()){
      current_head = k.update(count_el,count_er);
      output = head_turn_pid.update(dir, current_head);
      UpdateDirection(output, -output);
    }
  }
  analogWrite(L_PWM_PIN, 0);
  analogWrite(R_PWM_PIN, 0);
}
