/*
  _____                              _         ____  _         _             _         _____           _                   
 |  __ \                            (_)       / __ \| |       | |           | |       / ____|         | |                  
 | |  | |_   _ _ __   __ _ _ __ ___  _  ___  | |  | | |__  ___| |_ __ _  ___| | ___  | (___  _   _ ___| |_ ___ _ __ ___    
 | |  | | | | | '_ \ / _` | '_ ` _ \| |/ __| | |  | | '_ \/ __| __/ _` |/ __| |/ _ \  \___ \| | | / __| __/ _ \ '_ ` _ \   
 | |__| | |_| | | | | (_| | | | | | | | (__  | |__| | |_) \__ \ || (_| | (__| |  __/  ____) | |_| \__ \ ||  __/ | | | | |  
 |_____/ \__, |_| |_|\__,_|_| |_| |_|_|\___| _\____/|_.__/|___/\__\__,_|\___|_|\___| |_____/ \__, |___/\__\___|_| |_| |_|  
          __/ |                   \ \       / __ \| |       | |           | |          / /    __/ |                        
  ______ |___/_ ______ ______ _____\ \     | |  | | |__  ___| |_ __ _  ___| | ___     / /____|___/___ ______ ______ ______ 
 |______|______|______|______|______\ \    | |  | | '_ \/ __| __/ _` |/ __| |/ _ \   / /______|______|______|______|______|
                                     \ \   | |__| | |_) \__ \ || (_| | (__| |  __/  / /                                    
                                      \_\   \____/|_.__/|___/\__\__,_|\___|_|\___| /_/
 */
 
#include "encoders.h"
#include "pid.h"
#include "timer.h"
#include "kinematics.h"
//#include "line_sensors.h"


#define BAUD_RATE 9600

//================================ Setup parameters ==============================
// Change these to change the behaviour of the roomba
#define forwards_speed 30
#define time_to_turn 3000 //ms
//================================================================================


//============================== Motor definitions ===============================
//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define FWD LOW
#define BCK HIGH
//===============================================================================

//=============================== Global variables ===============================
long last_count_el = 0;
long last_count_er = 0;

float velocity_l = 0;
float velocity_r = 0;

float direct = 0;
//================================================================================

//============================ Class setup variables =============================
unsigned long PID_timer_max = 5000;
unsigned long Kinematics_timer_max = 10000;

float Kp_left = 0.5; //Proportional gain for position controller
float Kd_left = 0; //Derivative gain for position controller
float Ki_left = 0.1; //Integral gain for position controller

float Kp_right = 0.5; //Proportional gain for position controller
float Kd_right = 0; //Derivative gain for position controller
float Ki_right = 0.1; //Integral gain for position controller

float Kp_head_turn = 1; //Proportional gain for position controller
float Kd_head_turn = 0.0; //Derivative gain for position controller
float Ki_head_turn = 0.0; //Integral gain for position controller
//================================================================================


//============================ Class instantiation ===============================
Timer PID_timer(PID_timer_max, false);
Timer Kinematics_timer(Kinematics_timer_max, false);
Timer Vel_timer(1000000000, false);
Timer Turn_timer(time_to_turn, true);

PID left_PID(Kp_left, Ki_left, Kd_left); //Position controller for left wheel position
PID right_PID(Kp_right, Ki_right, Kd_right); //Position controller for left wheel position
PID head_turn_pid(Kp_head_turn, Ki_head_turn, Kd_head_turn); //Turning controller for update direction

Kinematics k;
//=================================================================================

//==================================== States ====================================
int state = 3;
#define STATE_BUTTON_WAIT 0
#define STATE_MOVE_FORWARDS 1
#define STATE_TURN 2
#define STATE_RESET 3
//================================================================================

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
}

void loop() {
  switch(state){
    case STATE_BUTTON_WAIT:
      ButtonWait(); Serial.println("Case 1");
      break;
    case STATE_MOVE_FORWARDS:
      MoveForwards(forwards_speed); //Serial.println("Case 2");
      break;
    case STATE_TURN:
      TurnInDirection(direct); Serial.println("---- Case 3 ---");
    break;
    case STATE_RESET: 
      ResetForMove(); Serial.println("Case 4");
    break;
  }
}

void MoveForwards(int move_speed){
  if(PID_timer.IsTime()){
    UpdateDirection(move_speed, move_speed);
  }

  if(Kinematics_timer.IsTime()){
    k.update(count_el, count_er);
  }

  if(Turn_timer.IsTime()){
    state = STATE_TURN;
  }
}

void ResetForMove(){
  direct += 180;
  Turn_timer.Reset();
  state = STATE_MOVE_FORWARDS;
}

void ButtonWait(){
  if(true){ //TODO
    state = STATE_MOVE_FORWARDS;
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

void TurnInDirection(float dir){
//  analogWrite(L_PWM_PIN, 0);
//  analogWrite(R_PWM_PIN, 0);
  if(PID_timer.IsTime()){
    float current_head = k.update(count_el,count_er);
    int output = (int)head_turn_pid.update(dir, current_head);
    Serial.println(output);
    UpdateDirection(output, -output);
    
    if(output < 1){
      analogWrite(L_PWM_PIN, 0);
      analogWrite(R_PWM_PIN, 0);
      Serial.println(output);
      state = STATE_RESET;
    }
  }
}
