#ifndef CUSTOMINCLUDES_H
#define CUSTOMINCLUDES_H

// port to use for debugging  #MD
#if (HIL_MODE == HIL_MODE_DISABLED) // Make sure debug messages don't interfere with GPS
#undef MY_DEBUG
#define MY_DEBUG  0
#endif


FastSerial* DBG = &Serial1;

// define macros for printing to serial port
#define DBG_PRINT(x)   	if (MY_DEBUG > 0) DBG->print(x);   else;
#define DBG_PRINTLN(x)  if (MY_DEBUG > 0) DBG->println(x); else;


#include "RelNAV.h"

#endif /*CUSTOMINCLUDES_H*/