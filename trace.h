#ifndef __TRACE_H__
#define __TRACE_H__
	#include "soft_uart.h"
	#ifndef __TRACE__DISABLED__

		#define susart_write(format, args...)   sUartPrintf(PSTR(format) , ## args)
		#define TRACE( a_szMSG, ...) susart_write( a_szMSG, ##__VA_ARGS__); susart_write( " Line:%i, File:%s\n\r",__LINE__, __FILE__);
		#define TRACE_RAW( a_szMSG, ...) susart_write( a_szMSG, ##__VA_ARGS__);

	#else

		#define TRACE( a_szMSG, ...) (void)0;
		#define TRACE_RAW( a_szMSG, ...) (void)0;

	#endif

	void traceInit( void);

#endif //__TRACE_H__
