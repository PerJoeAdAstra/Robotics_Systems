#include "irproximity.h"
SharpIR proximity(A0);

#define BAUD_RATE 9600
void setup() {

  Serial.begin( BAUD_RATE );
  delay(3000);
  Serial.println("***RESET***");
}

void checkForObstacle(){
  if (proximity.getDistanceInMM() < 200) Serial.println("Stop Romi");
  else Serial.println("Go!");
}

void loop() {
  checkForObstacle();
  delay(300);
  // put your main code here, to run repeatedly:

}
