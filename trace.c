#include "trace.h"

#ifndef __TRACE__DISABLED__
	void traceInit( void)
	{
		//fdevopen ( sUartPrintChar, NULL); //redirect printf
	}

#else
	void traceInit( void)
	{
	}

#endif
