#include "encoders.h"
#include "pid.h"
#include "line_sensors.h"
#include "timer.h"
#include "kinematics.h"

//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define FWD LOW
#define BCK HIGH

float Kp_left = 0.5; //Proportional gain for position controller
float Kd_left = 0; //Derivative gain for position controller
float Ki_left = 0.1; //Integral gain for position controller

float Kp_right = 0.5; //Proportional gain for position controller
float Kd_right = 0; //Derivative gain for position controller
float Ki_right = 0.1; //Integral gain for position controller

float Kp_head = 0.06; //Proportional gain for position controller
float Kd_head = 0.01; //Derivative gain for position controller
float Ki_head = 0; //Integral gain for position controller

float Kp_head_turn = 1; //Proportional gain for position controller
float Kd_head_turn = 0.0; //Derivative gain for position controller
float Ki_head_turn = 0.0; //Integral gain for position controller

PID left_PID(Kp_left, Ki_left, Kd_left); //Position controller for left wheel position
PID right_PID(Kp_right, Ki_right, Kd_right); //Position controller for left wheel position
PID heading_pid(Kp_head, Ki_head, Kd_head); 
PID head_turn_pid(Kp_head_turn, Ki_head_turn, Kd_head_turn); 

// You may need to change these depending on how you wire
// in your line sensor.
#define LINE_LEFT_PIN A4 //Pin for the left line sensor
#define LINE_CENTER_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN A2 //Pin for the right line sensor

LineSensor line_left(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
LineSensor line_center(LINE_CENTER_PIN); //Create a line sensor object for the centre sensor
LineSensor line_right(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

#define BAUD_RATE = 115200;

float velocity_l = 0;
float velocity_r = 0;

long last_count_el = 0;
long last_count_er = 0;

unsigned long PID_timer_max = 500;
unsigned long Kinematics_timer_max = 1000;
Timer PID_timer(PID_timer_max, false);
Timer Kinematics_timer(Kinematics_timer_max, false);
Timer Vel_timer(1000000000, false);

bool online = false;

int confidence = 0;
int confidence_diff = 5;
int confidence_thresh = 100;

bool has_been_online = false;

Kinematics k;
bool forwards = true;

int state = 0;

void setupMotorPins()
{
    // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  digitalWrite( L_DIR_PIN, LOW  );
  digitalWrite( R_DIR_PIN, LOW );
}

// Remember, setup only runs once.
void setup() 
{

  // These two function set up the pin
  // change interrupts for the encoders.
  setupEncoder0();
  setupEncoder1();

  // Initialise your other globals variables
  // and devices.

  // Initialise the Serial communication
  head_turn_pid.setMax(15);
  Serial.begin( 9600 );
  delay(3000);
  Serial.println("***RESET***");
  tone(6, 440, 250);
  delay(500);
  line_left.calibrate();
  line_center.calibrate();
  line_right.calibrate();
  tone(6, 480, 250);
  delay(250);
  head_turn_pid.setMax(80.0);
  k.setDebug(true);
}

// Remmeber, loop is called again and again.
void loop()
{
  switch(state){
    case 0:
      Serial.println("-- Case 0 --");
      
      if(Kinematics_timer.IsTime()){
        k.update(count_el, count_er);
      }
      
      if(not(CheckForLine())){
        UpdateDirection(30,30);
      }
      else state = 1;
      break;
    case 1:
      Serial.println("-- Case 1 --");
      if(Kinematics_timer.IsTime()){
        float theta =  k.update(count_el, count_er);
        Serial.println(theta);
      }
      if(PID_timer.IsTime()){
        follow_line(10);
      }
      if(!CheckForLine()){
        Serial.println("End of line");
        UpdateDirection(0,0);
        analogWrite(L_PWM_PIN, 0);
        analogWrite(R_PWM_PIN, 0);
        tone(6, 440, 250);
        delay(1000);
        state = 2;
      }
      break;
    case 2:
      Serial.println("Case 2");
      float angle = k.GetAngleToHome();
      Serial.println("angle:");
      Serial.println(angle);
      delay(3000);
      GoHome(20);
      break;
    case 3: 
      Serial.println("Case 3");
      tone(6, 440, 250);
      delay(1000);
      break;
    default:
      Serial.println("Default");
      break;  
  }
}

void TurnInDirection(float dir){
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

void TurnToHome(){
  TurnInDirection(k.GetAngleToHome());
}

bool UpdateLineConfidence(int confidence, int value){
  if (line_left.IsOnline() || line_center.IsOnline() || line_center.IsOnline()) confidence += value;
  else confidence -= value;
}

bool CheckForLine(){
  if(line_left.IsOnline() || line_center.IsOnline() || line_right.IsOnline()) return true;
  if(line_left.read_calibrated() + line_center.read_calibrated() > 500) return true;
  if(line_right.read_calibrated() + line_center.read_calibrated() > 500) return true;
  return false;
}

void GoHome(int bias){
  TurnInDirection(k.GetAngleToHome());
  float current_head = k.update(count_el, count_er);
  PID_timer.Reset();
  Kinematics_timer.Reset();
  float output = head_turn_pid.update(k.GetAngleToHome(), current_head);
  while(!k.IsHome()){
    if(PID_timer.IsTime()){
      current_head = k.update(count_el,count_er);
      output = head_turn_pid.update(k.GetAngleToHome(), current_head);
      UpdateDirection(bias + output, bias - output);
    }
  }
  analogWrite(L_PWM_PIN, 0);
  analogWrite(R_PWM_PIN, 0);
  tone(6, 440, 250);
  delay(5000);
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

void follow_line(int forward_speed){
  int value = CalcLineCenter(line_left.read_calibrated(), line_center.read_calibrated(), line_right.read_calibrated()) - 2000;
  int turn = (int)heading_pid.update(value, 0);
//  --- Debugging prints ---
//  Serial.print("Value:");
//  Serial.print(value);
//  Serial.print(",turn:");
//  Serial.print(turn);
//  Serial.print('\n');
  int diff_l = (int) count_el - last_count_el;
  int diff_r = (int) count_er - last_count_er;
  last_count_el = count_el;
  last_count_er = count_er;
  
  float float_elapsed = Vel_timer.GetElapsed();

  velocity_l = (float)(diff_l/(float_elapsed)) * 100000;
  velocity_r = (float)(diff_r/(float_elapsed)) * 100000;

  int right_demand;
  int left_demand;

  right_demand = forward_speed - turn;
  left_demand = forward_speed + turn;
  
  float output_l = left_PID.update(left_demand, velocity_l);
  float output_r = right_PID.update(right_demand, velocity_r);

//  --- Debugging Prints ---
//  Serial.print("Demand:");
//  Serial.print(left_demand);
//  Serial.print(",diff_l:");
//  Serial.print(diff_l);
//  Serial.print(",distance_l:");
//  Serial.print(distance_l);
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

int CalcLineCenter(int l, int c, int r){
  int total = l + c + r;
  float p1 = l / (float)total;
  float p2 = c / (float)total;
  float p3 = r / (float)total;
  return ((int)((p1 * 1000) + (p2 * 2000) + (p3 * 3000)));
}
