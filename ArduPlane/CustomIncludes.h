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

#include "RelNAV.h"

#endif /*CUSTOMINCLUDES_H*/