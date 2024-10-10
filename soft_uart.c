
#include "soft_uart.h"

uint16_t g_sUartDelayValue = 113;


void sUartWriteBit( uint8_t a_ui8Bit)
{   
    uint8_t oldSREG = SREG;

	if ( a_ui8Bit == 0) 
    {
		sUartTXPort &= ~( 1<<sUartTXPin);
	} 
    else 
    {
		sUartTXPort |= ( 1<<sUartTXPin);
	}
    
    SREG = oldSREG;
   
}

void sUartDelay( uint16_t a_ui16Delay)
{
    uint8_t tmp=0;

    asm volatile("sbiw    %0, 0x01 \n\t"
	             "ldi %1, 0xFF \n\t"
	             "cpi %A0, 0xFF \n\t"
	             "cpc %B0, %1 \n\t"
	             "brne .-10 \n\t"
	             : "+r" ( a_ui16Delay), "+a" (tmp)
	             : "0" ( a_ui16Delay)
	             );
}


//------------------------------------------------------------------------------
void sUartPrintf (const char *a_pBuffer,...)
{
	va_list ap;
	va_start (ap, a_pBuffer);	
	
	int format_flag;
	char str_buffer[10];
	char str_null_buffer[10];
	char move = 0;
	char Base = 0;
	int tmp = 0;
	char by;
	char *ptr;
		
	//Ausgabe der Zeichen
    for(;;)
	{
		by = pgm_read_byte(a_pBuffer++);
		if(by==0) break; // end of format string
            
		if (by == '%')
		{
            by = pgm_read_byte(a_pBuffer++);
			if (isdigit(by)>0)
				{
                                 
 				str_null_buffer[0] = by;
				str_null_buffer[1] = '\0';
				move = atoi(str_null_buffer);
                by = pgm_read_byte(a_pBuffer++);
				}

			switch (by)
				{
                case 's':
                    ptr = va_arg(ap,char *);
                    while(*ptr) { sUartPrintChar(*ptr++); }
                    break;
				case 'b':
					Base = 2;
					goto ConversionLoop;
				case 'c':
					//Int to char
					format_flag = va_arg(ap,int);
					sUartPrintChar (format_flag++);
					break;
				case 'i':
					Base = 10;
					goto ConversionLoop;
				case 'o':
					Base = 8;
					goto ConversionLoop;
				case 'x':
					Base = 16;
					//****************************
					ConversionLoop:
					//****************************
					itoa(va_arg(ap,int),str_buffer,Base);
					int b=0;
					while (str_buffer[b++] != 0){};
					b--;
					if (b<move)
						{
						move -=b;
						for (tmp = 0;tmp<move;tmp++)
							{
							str_null_buffer[tmp] = '0';
							}
						//tmp ++;
						str_null_buffer[tmp] = '\0';
						strcat(str_null_buffer,str_buffer);
						strcpy(str_buffer,str_null_buffer);
						}
					sUartPrintString (str_buffer);
					move =0;
					break;
				}
			
			}	
		else
		{
			sUartPrintChar ( by );	
		}
	}
	va_end(ap);
}


void sUartPrintChar( uint8_t a_ui8Byte)
{
    unsigned char ucMask;

    sUartWriteBit( 0);  // startbit
    sUartDelay( g_sUartDelayValue * 2);

    for ( ucMask = 0x01; ucMask; ucMask <<= 1) 
    {
        if ( a_ui8Byte & ucMask)
        { // choose bit
            sUartWriteBit( 1); // send 1
        }
        else
        {
            sUartWriteBit( 0); // send 0
        }
        
        sUartDelay( g_sUartDelayValue * 2);
    }
  
    sUartWriteBit( 1);
    sUartDelay( g_sUartDelayValue * 2);
    return;

}

void sUartPrintString( char *a_pString)
{
	while ( *a_pString)
	{
		sUartPrintChar( *a_pString++);
	}
}

void sUartInit( unsigned long a_ulBaudRate)
{
  switch ( a_ulBaudRate) 
  {
      case 115200: 
      {
#if F_CPU == 11059200UL	  	  
          g_sUartDelayValue = 4; 
#else
          g_sUartDelayValue = 7; 
#endif

//#if F_CPU == 14745600	  	  
//          g_sUartDelayValue = 6; 


          break;
      }
      case 57600:
      {
          g_sUartDelayValue = 17; 
          break;
      }
      case 38400:
      {
          g_sUartDelayValue = 27; 
          break;
      }
      case 19200:
      {
          //g_sUartDelayValue = 51; 
          g_sUartDelayValue = 40;
		  break;
      }
      case 9600:
      {
          g_sUartDelayValue = 113; 
          break;
      }
      case 4800:
      {
          g_sUartDelayValue = 232; 
          break;
      }
      case 2400:
      {
          g_sUartDelayValue = 470; 
          break;
      }
      default:
      {
          g_sUartDelayValue = 0;
          break;
      }
  } 


  sUartTXDDRPort |= (1<<sUartTXPin);

  return;
}
