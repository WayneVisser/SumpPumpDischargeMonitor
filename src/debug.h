
// *********************************************************************
// DEBUG
//
#define DEBUG
// uncomment the following to disable  debug printing.
#undef DEBUG

// enable/disable printing to serial port
#ifdef DEBUG
  #define Sprintln(a) (Serial.println(a))
  #define Sprint(a)   (Serial.print(a))
#else
  #define Sprintln(a)
  #define Sprint(a)
#endif