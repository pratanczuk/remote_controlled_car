#ifndef __PSX_PAD_H__
#define __PSX_PAD_H__

#include <util/delay.h>
#include "trace.h"

// configuration
// Reamrk: disable jtag by fuse bits, if you are using PORTC of ATMEGA32

#define PSclock            PC6         // PC6
#define PSdata             PC5         // PC5
#define PSacknolage        PC2         // PC2
#define PScommand          PC4         // PC4
#define PSattention        PC7         // PC7
#define PSPort             PORTC
#define PSDDRPort          DDRC
#define PSPIN              PINC
#define ClockDelay         6


// END of confguration

// Button enum
typedef enum 
{
    PS2_NotButtonPressed = 0,   
    PS2_Select,
    PS2_Start,
    PS2_DpadU,
    PS2_DpadR,
    PS2_DpadD,
    PS2_DpadL,
    PS2_L2,
    PS2_R2,
    PS2_L1,
    PS2_R1,
    PS2_Triangle,
    PS2_Circle,
    PS2_X,
    PS2_Square,
    PS2_AXIS1L,
    PS2_AXIS1R,
    PS2_AXIS1U,
    PS2_AXIS1D,
    PS2_AXIS2L,
    PS2_AXIS2R,
    PS2_AXIS2U,
    PS2_AXIS2D
} ePS2Button;

typedef enum 
{
    PS2_ERROR = -1,
    PS2_OK = 0
} ePS2Result;

unsigned char gameByte( unsigned char a_ui8Command);
ePS2Result initPSX_PAD( void);
ePS2Button testPSX_PAD( void);

#define sbi( port, bit) ( port) |= ( 1 << ( bit))
#define cbi( port, bit) ( port) &= ~( 1 << ( bit))

#endif //__PSX_PAD_H__