//TODO: Copy all needed instantiation from the library

//============= States =============
int state = 0;
#define STATE_CALLIBRATE 0
//#define STATE_WAIT_FOR_BUTTON
#define STATE_NO_DETECTION 1
#define STATE_MOVE_DETECTED 2
//==================================


void setup() {
  //TODO: Copy setup from library
}

void loop() {
    switch(state){
    case STATE_CALIBRATE:
        //Do the callibrate
        //swtich to state no detection
    case STATE_NO_DETECTION:
//      Move forwards
//      if move detected
//        Switch state to move detected
    break;
    case STATE_MOVE_DETECTED:
//      don't move
//      if move no longer detected
//        Switch state no detection
    break;
  }
}
