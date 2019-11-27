#ifndef timer
#define timer
#include <stdint.h>

class Timer
{
  /*  
   * 
   */
  public:

    Timer(unsigned long in_timer_max, bool in_is_millis);// This is the class constructor.
    unsigned long GetElapsed(); //returns the elapsed time, resets timestamp
    unsigned long GetElapsedNoReset(); //returns the elapsed time without editing the time state
    bool IsTime(); //returns whether the timer is over the max value, if it is it resets the internal variables
    bool IsTimeNoReset();
    void SetTimerMax(unsigned long input);
    void SetIsMillis(bool input);
    void Reset();                                   // Resets the timer to the current time
  
  private:

    //Control gains
    unsigned long last_timestamp;
    unsigned long timer_max;
    bool is_millis;
    unsigned long GetTimeNow();
};

/*
 * Class constructor
 * This runs whenever we create an instance of the class
 */
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
/*
 * This function prints the individual contributions to the total contol signal
 * You can call this yourself for debugging purposes, or set the debug flag to true to have it called
 * whenever the update function is called.
 */
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
/*
 * This function sets the gains of the PID controller
 */
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
