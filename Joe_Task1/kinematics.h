#ifndef _Kinematics
#define _Kinematics_h

//You may want to use some/all of these variables
const float WHEEL_DIAMETER    = 70;
const float WHEEL_RADIUS      = 35;
const float WHEEL_SEPERATION  = 145;
const float GEAR_RATIO        = 120;
const float COUNTS_PER_SHAFT_REVOLUTION = 12;
const float COUNTS_PER_WHEEL_REVOLUTION =  1440;
const float COUNTS_PER_MM               = (COUNTS_PER_WHEEL_REVOLUTION / WHEEL_DIAMETER*M_PI);
  
class Kinematics
{
  public:
    Kinematics();   // Constructor, required.
    float update(long e_l, long e_r);
    // Write your method functions:
    // ...
    float TicksToDistance(long ticks);
    float GetAngleToHome();
    float GetCurrentHeading();
    bool IsHome();
    void setDebug(bool state);
  private:
//    float TicksToDistance(long ticks);  
    unsigned long old_encoder_count;
    long kin_last_count_el;
    long kin_last_count_er;
    float heading;
    float x_pos;
    float y_pos;
    float theta;
    float homex;
    float homey;
    bool debug;
    void printComponents();
    //Private variables and methods go here
};


// Required constructor.  Initialise variables.
Kinematics::Kinematics() {
  old_encoder_count = 0;
  theta = 0;
  x_pos = 0;
  y_pos = 0;
  homex = 0;
  homey = 0;
}

float Kinematics::update(long e_l, long e_r){
  float lDist = (TicksToDistance(e_l - kin_last_count_el));
  float rDist = (TicksToDistance(e_r - kin_last_count_er));
  theta = theta + ((lDist -  rDist)/WHEEL_SEPERATION) * (104.5/90.0);
  float average = (lDist + rDist) /2.0;
  x_pos = x_pos + average*cos(M_PI*(theta/180));
  y_pos = y_pos + average*sin(M_PI*(theta/180));
  if(debug) printComponents();
  kin_last_count_el = e_l;
  kin_last_count_er = e_r;
  return theta;
}

void Kinematics::printComponents(){
  Serial.print("x_pos:");
  Serial.print(x_pos);
  Serial.print(",y_pos:");
  Serial.print(y_pos);
  Serial.print(",theta:");
  Serial.print(theta);
  Serial.print('\n');
}

float Kinematics::TicksToDistance(long ticks){
  if (ticks == 0) return 0;
  return (float)ticks/COUNTS_PER_MM * 1000;
}
/*
float Kinematics::GetHomeHeading(){
  float HomeAngle = GetAngleToHome();
  return 0;
}
*/
float Kinematics::GetAngleToHome(){
  float dy = homey - y_pos;
  float dx = homex - x_pos;
  return (atan2(dy,dx) * (180.0 / M_PI));
}

bool Kinematics::IsHome(){
  float dy = x_pos - homex;
  float dx = y_pos - homey;
  if(abs(dx) < 100 && abs(dy) < 100) return true;
  else return false;
}

void Kinematics::setDebug(bool state){
  debug = state;
}
#endif
