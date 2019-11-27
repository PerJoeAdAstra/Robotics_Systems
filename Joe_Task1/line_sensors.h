#ifndef _Line_follow_h
#define _Line_follow_h

//Number of readings to take for calibration
//const int NUM_CALIBRATIONS = ????;

/* 
 *  Class to represent a single line sensor
 */
class LineSensor
{
  public:

    // Required function.
    LineSensor(int pin);   //Constructor

    // Suggested functions.
    void calibrate();       //Calibrate
    int read_raw();         //Return the uncalibrated value from the sensor
    int read_calibrated();  //Return the calibrated value from the sensor
    bool IsOnline();
    // You may wish to add other functions!
    // ...
    
  private:
  
    int pin;
    int offset = 0;
    int thresh = 500;
    /*
     * Add any variables needed for calibration here
     */
    
};


// Class Constructor: 
// Sets pin passed in as argument to input
LineSensor::LineSensor(int Line_pin)
{
  pin = Line_pin;
  pinMode(pin, INPUT);
}

// Returns unmodified reading.
int LineSensor::read_raw()
{
  return analogRead(pin);
}

bool LineSensor::IsOnline()
{
  if(read_calibrated() > thresh) return 1;
  else return 0;
}

// Write this function to measure any
// systematic error in your sensor and
// set some bias values.
void LineSensor::calibrate()
{
  int total = 0;
  for(int i = 0; i < 50; i++) total += analogRead(pin);
  offset = (total/50);
}


// Use the above bias values to return a
// compensated ("corrected") sensor reading.
int LineSensor::read_calibrated()
{
  /*
   * Write code to return a calibrated reading here
   */
//   if(analogRead(pin) - offset > thresh) return 1;
//   else return 0;
   return analogRead(pin) - offset;
}


#endif
