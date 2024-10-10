#include "psx_pad.h"

// Polls and stores the PS2 controller data

ePS2Result initPSX_PAD( void)
{
   // PSx controller I/O pin setup:
   sbi(PSDDRPort, PSclock);                 // clock. output. (blue)

   cbi(PSDDRPort, PSdata);                 // data. input. (brown)
   sbi(PSPort, PSdata);               //    enable pullup resistor

   cbi(PSDDRPort, PSacknolage);                 // acknolage. input. (green)
   sbi(PSPort, PSacknolage);               //    enable pullup resistor

   sbi(PSDDRPort, PScommand);                 // command. output. (orange)

   sbi(PSDDRPort, PSattention);                 // attention. output. (yellow)

   // this loop continues to put PSx controller into analouge mode untill the
   // controller responds with 0x73 in the 2nd byte. 
   // (PS2 controller responds with 0x73 when in analouge mode.)
   unsigned char ui8CheckPADMode = 0;
   unsigned char ui8Index = 0;
   unsigned char uiStatus = PS2_OK;
   
   while( ui8CheckPADMode != 0x73)
   {
       // put controller in config mode
       sbi( PSPort, PScommand);
       sbi( PSPort, PSclock);
       cbi( PSPort, PSattention);

       gameByte( 0x01);
       gameByte( 0x43);
       gameByte( 0x00);
       gameByte( 0x01);
       gameByte( 0x00);

       sbi( PSPort, PScommand);
       _delay_us( 1);
       sbi( PSPort, PSattention);

       _delay_ms( 1);

       // put controller in analouge mode
       sbi( PSPort, PScommand);
       sbi( PSPort, PSclock);
       cbi( PSPort, PSattention);

       gameByte( 0x01);
       gameByte( 0x44);
       gameByte( 0x00);
       gameByte( 0x01);
       gameByte( 0x03);
       gameByte( 0x00);
       gameByte( 0x00);
       gameByte( 0x00);
       gameByte( 0x00);

       sbi( PSPort, PScommand);
       _delay_us( 1);
       sbi( PSPort, PSattention);

       _delay_ms( 1);

       // exit config mode
       sbi( PSPort, PScommand);
       sbi( PSPort, PSclock);
       cbi( PSPort, PSattention);

       gameByte( 0x01);
       gameByte( 0x43);
       gameByte( 0x00);
       gameByte( 0x00);
       gameByte( 0x5A);
       gameByte( 0x5A);
       gameByte( 0x5A);
       gameByte( 0x5A);
       gameByte( 0x5A);

       sbi( PSPort, PScommand);
       _delay_us( 1);
       sbi( PSPort, PSattention);

       _delay_ms( 1);

       // poll controller and check in analouge mode.
       sbi( PSPort, PScommand);
       sbi( PSPort, PSclock);
       cbi( PSPort, PSattention);

       gameByte( 0x01);
       ui8CheckPADMode = gameByte( 0x42);            // the 2nd byte to be returned from the controller should = 0x73 for "red" analouge controller.
       gameByte( 0x00);
       gameByte( 0x00);
       gameByte( 0x00);
       gameByte( 0x00);
       gameByte( 0x00);
       gameByte( 0x00);
       gameByte( 0x00);

       sbi( PSPort, PScommand);
       _delay_us( 1);
       sbi( PSPort, PSattention);
       _delay_ms( 100);
       ui8Index++;
       if( 255 == ui8Index)
       {
           uiStatus =  PS2_ERROR;
           break;
       }
   }

   return uiStatus;
}

ePS2Button testPSX_PAD( void)
{

   unsigned char PSData[8] = { 255, 255, 255, 255, 128, 128, 128, 128 };

   sbi( PSPort, PScommand);                          // start communication with PSx controller
   sbi( PSPort, PSclock);
   cbi( PSPort, PSattention);

   gameByte( 0x01);                                       // bite 0. header.
   PSData[ 0] = gameByte( 0x42);                                // bite 1. header. (should possibly put test on this byte to detect unplugging of controller.)
   PSData[ 1] = gameByte( 0x00);                                       // bite 2. header.

   PSData[ 2] = gameByte( 0x00);                         // bite 3. first data bite.
   PSData[ 3] = gameByte( 0x00);                         // bite 4.
   PSData[ 4] = gameByte( 0x00);                         // bite 5.
   PSData[ 5] = gameByte( 0x00);                         // bite 6.
   PSData[ 6] = gameByte( 0x00);                         // bite 7.
   PSData[ 7] = gameByte( 0x00);                         // bite 8.

   _delay_us( 1);
   sbi( PSPort, PScommand);                      // close communication with PSx controller
   _delay_us( 1);
   sbi( PSPort, PSattention);                        // all done.
   
   if( 255 != PSData[ 1])
   {
       if( 255 != PSData [ 2])
       {
           switch( PSData[ 2])
           {
               case 127: return PS2_DpadL;
               case 191: return PS2_DpadD;
               case 223: return PS2_DpadR;
               case 239: return PS2_DpadU;
               case 254: return PS2_Select;
               case 247: return PS2_Start;
               default: return PS2_NotButtonPressed;
           }       
       }
       
       if( 255 != PSData [ 3]) 
       {
           switch( PSData[ 3])
           {
               case 223: return PS2_Circle;
               case 191: return PS2_X;
               case 127: return PS2_Square;
               case 239: return PS2_Triangle;
               case 251: return PS2_L1;
               case 254: return PS2_L2;
               case 247: return PS2_R1;
               case 253: return PS2_R2;
               default: return PS2_NotButtonPressed;
           }
       }

       if( 128 != PSData [ 4])
       {
           switch( PSData[ 4])
           {
               // 127 TO 0
                case 0 ... 117: return PS2_AXIS2L;
               // 128 TO 255
               case 138 ... 255: return PS2_AXIS2R;
               default: return PS2_NotButtonPressed;
           }
       }
       
       if( 128 != PSData [ 5])
       {
           switch( PSData[ 5])
           {
               // 127 TO 0
               case 0 ... 117: return PS2_AXIS2U;
               // 128 TO 255
               case 138 ... 255: return PS2_AXIS2D;
               default: return PS2_NotButtonPressed;
           }
       }
       
       if( 128 != PSData [ 6]) 
       {
           switch( PSData[ 6])
           {
               // 127 TO 0
                case 10 ... 127: return PS2_AXIS1L;
               // 128 TO 255
               case 138 ... 255: return PS2_AXIS1R;
               default: return PS2_NotButtonPressed;
           }

       }
       
       if( 128 != PSData [ 7])
       {
           switch( PSData[ 7])
           {
               // 127 TO 0
               case 10 ... 127: return PS2_AXIS1U;
               // 128 TO 255
               case 138 ... 255: return PS2_AXIS1D;
               default: return PS2_NotButtonPressed;
           }
       }
   }

   return PS2_NotButtonPressed;
} 

// PSx controller communication function.
// send a byte on the command line and receive one on the data line.
// needs Attention pin to have gone low before called to activate controller.
unsigned char gameByte( unsigned char a_ui8Command)
{
    unsigned char ui8Index     = 0x00;
    unsigned char ui8Data      = 0x00;

    for( ui8Index = 0; ui8Index < 8; ui8Index++)
    {
            // bit bang "command" out on PScommand wire.
            if( a_ui8Command & ( 1 << ( ui8Index))) 
            {
                sbi( PSPort, PScommand);                
            }
            else
            {
                cbi( PSPort, PScommand);                
            }

            // CLOCK LOW
            cbi( PSPort, PSclock);

            // wait for output to stabilise                
            _delay_us( ClockDelay);

            if( ( PSPIN & ( 1 <<( PSdata)))) 
            {
                sbi( ui8Data, ui8Index);                
            }
            else
            {
                cbi( ui8Data, ui8Index);                
            }

            // CLOCK HIGH
            sbi( PSPort, PSclock);                             
            _delay_us( ClockDelay);
    }

    sbi( PSPort, PScommand);

    // wait for ack to go low (usually within 10 loops)
    for( ui8Index=0; ui8Index<20; ui8Index++)
    {
        if( !( PSPIN & ( 1 << ( PSacknolage))))	
        {
            break;
        }
    }

    return( ui8Data);
}

