#ifndef DEBUG_H_
#define DEBUG_H_

//Standard Includes
#include <stdio.h>
//Support Includes
#include "fsl_gpio.h"

#define DEBUG_PRINT 2

// https://stackoverflow.com/questions/1644868/c-define-macro-for-debug-printing
#if DEBUG_PRINT == 2
	#define dbg(msg) \
			do { if (DEBUG_PRINT) printf("%s, %d, %s, %s\n", __FILE__, __LINE__, __func__, msg); } while (0)
#elif DEBUG_PRINT == 1
	#define dbg(msg) \
        do { if (DEBUG_PRINT) printf("%s", msg); } while (0)
#else
	#define dbg(msg) (void) 0
#endif

#endif /* DEBUG_H_ */
