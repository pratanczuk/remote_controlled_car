#ifndef __DESIGN_BY_CONTRACT__
#define __DESIGN_BY_CONTRACT__


#include "trace.h"

#define __dbc_print__ TRACE_RAW

#ifdef __DBC_ON__
    #define REQUIRE( expression) ( expression) ? 0 : __dbc_print__( "\n\rREQUIRE failed: File:%s; Line:%d; Exp:%s;", __FILE__, __LINE__ ,#expression)

    #define CHECK( expression)( expression) ? 0 : __dbc_print__("\n\rCHECK failed: File:%s; Line:%d; Exp:%s;", __FILE__, __LINE__ ,#expression)

    #define ENSURE( expression)(expression) ? 0 : __dbc_print__( "\n\rENSURE failed: File:%s; Line:%d; Exp:%s;", __FILE__, __LINE__ ,#expression)

    #define ILLEGAL_ELSE_PATH( expression) __dbc_print__( "\n\rILLEGAL_ELSE_PATH failed: File:%s; Line:%d; Exp:%s;", __FILE__, __LINE__ ,#expression)

    #define ILLEGAL_DEFAULT_CASE( value) __dbc_print__( "\n\rILLEGAL_DEFAULT_CASE failed: File:%s; Line:%d; Value:%s;", __FILE__, __LINE__ ,#value)

    #define CHECK_POINTER( pointer) ( 0 != pointer) ? 0 : __dbc_print__( "\n\rCHECK_POINTER failed: File:%s; Line:%d; PTR:%s;", __FILE__, __LINE__ ,#pointer)

#else 
    #warning Desingn By Contract has been disabled
    #define REQUIRE( ignore) (void)0
    #define CHECK( ignore) (void)0
    #define ENSURE( ignore) (void)0
    #define ILLEGAL_ELSE_PATH( ignore) (void)0
    #define ILLEGAL_DEFAULT_CASE( ignore) (void)0
    #define ILLEGAL_DEFAULT_CASE( ignore) (void)0
    #define CHECK_POINTER( pointer) (void)0
 
#endif /*__DBC_ON__*/

#endif /*__DESIGN_BY_CONTRACT__*/
