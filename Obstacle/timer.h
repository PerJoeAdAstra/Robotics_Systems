#ifndef timer
#define timer
#include <stdint.h>

class Timer
{
  /*  
   * 
   */
  public:
    //Class constructor. in_timer_max: value timer resets at, in_is_millis: if true the timer uses millis(), otherwise micros().
    Timer(unsigned long in_timer_max, bool in_is_millis);
    unsigned long GetElapsed();            //returns the elapsed time, resets timestamp
    unsigned long GetElapsedNoReset();     //returns the elapsed time, does NOT reset timestamp
    bool IsTime();                         //returns whether the timer is over the max value, if it is it resets the internal variables
    bool IsTimeNoReset();                  //returns whether the timer is over the max value, does NOT reset timestamp
    void SetTimerMax(unsigned long input); //Changes the timer max
    void SetIsMillis(bool input);          //Changes the timer max
    void Reset();                          //Manual timestamp reset
  
  private:
    unsigned long last_timestamp;
    unsigned long timer_max;
    bool is_millis;
    unsigned long GetTimeNow();
};

// Constructor
Timer::Timer(unsigned long in_timer_max, bool in_is_millis)
{
  SetTimerMax(in_timer_max);
  SetIsMillis(in_is_millis);
  if (is_millis) last_timestamp = millis();
  else last_timestamp = micros();
}

void Timer::SetTimerMax(unsigned long input){
  timer_max = input;
}

void Timer::SetIsMillis(bool input){
  is_millis = input;
}


unsigned long Timer::GetTimeNow(){
  if (is_millis) return millis();
  else return micros();
}
 
unsigned long Timer::GetElapsed() {
  unsigned long time_now;
  time_now = GetTimeNow();
  unsigned long elapsed = time_now - last_timestamp;
  last_timestamp = time_now;
  return elapsed;
}

unsigned long Timer::GetElapsedNoReset(){
  unsigned long time_now;
  time_now = GetTimeNow();
  return time_now - last_timestamp;
}

bool Timer::IsTime() {  
  if(GetElapsedNoReset() > timer_max){
    Reset();
    return true;
  }
  else return false;
}

bool Timer::IsTimeNoReset() {
  if(GetElapsed() > timer_max) return true;
  else return false;
}

void Timer::Reset(){
  last_timestamp = GetTimeNow();
}
#endif
