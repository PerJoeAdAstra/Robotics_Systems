/*
  _____                              _         ____  _         _             _         _____           _                 
 |  __ \                            (_)       / __ \| |       | |           | |       / ____|         | |                
 | |  | |_   _ _ __   __ _ _ __ ___  _  ___  | |  | | |__  ___| |_ __ _  ___| | ___  | (___  _   _ ___| |_ ___ _ __ ___  
 | |  | | | | | '_ \ / _` | '_ ` _ \| |/ __| | |  | | '_ \/ __| __/ _` |/ __| |/ _ \  \___ \| | | / __| __/ _ \ '_ ` _ \ 
 | |__| | |_| | | | | (_| | | | | | | | (__  | |__| | |_) \__ \ || (_| | (__| |  __/  ____) | |_| \__ \ ||  __/ | | | | |
 |_____/ \__, |_| |_|\__,_|_| |_| |_|_|\___|  \____/|_.__/|___/\__\__,_|\___|_|\___| |_____/ \__, |___/\__\___|_| |_| |_|
          __/ |                   \ \             \ \    / /_ | / _ \                / /      __/ |                      
  ______ |___/_ ______ ______ _____\ \ ______ _____\ \  / / | || | | |______ ______ / /_____ |___/_ ______ ______ ______ 
 |______|______|______|______|______\ \______|______\ \/ /  | || | | |______|______/ /______|______|______|______|______|
                                     \ \             \  /   | || |_| |            / /                                    
                                      \_\             \/    |_(_)___/            /_/                                     
                                                                                                                         
 */
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
