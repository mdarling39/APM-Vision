#ifndef CUSTOMINCLUDES_H
#define CUSTOMINCLUDES_H

// port to use for debugging  #MD
#if (HIL_MODE == HIL_MODE_DISABLED) // Make sure debug messages don't interfere with GPS
#undef MY_DEBUG
#define MY_DEBUG  0
#endif

#if (MY_DEBUG > 0)
	FastSerial* DBG = &Serial1;
#else
	FastSerial* DBG = NULL;
#endif

// define macros for printing to serial port
#if (DBG == NULL)
#define DBG_PRINT(x)	;
#define DBG_PRINTLN(x)	;
#else
#define DBG_PRINT(x)   	DBG->print(x);
#define DBG_PRINTLN(x)  DBG->println(x);
#endif

#include "RelNAV.h"

#endif /*CUSTOMINCLUDES_H*/