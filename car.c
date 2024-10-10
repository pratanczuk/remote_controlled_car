

#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "trace.h"
#include "contract.h"

#include "usart.h"
#include "soft_uart.h"
#include "mca25.h"
#include "nRF24L01.h"
#include "mirf.h"
#include "spi.h"
#include "psx_pad.h"


// tranciever RFM12 433 MHz currently not used, system is configured to use nRF24L01 2.4 GHz 
//#include "rfm12-1.1/src/rfm12_config.h"
//#include "rfm12-1.1/src/rfm12.h"

void servo_init (void)
{

	//Initialize Timer0
	TCCR0 = _BV(WGM00) | _BV(COM01) | _BV(CS02);


	//Enable output
	DDRB |= _BV(PB3);
	DDRD |= (1<<4);  // Make  PD4 an output port for Servo 1 

   // Select  Timer 1 for Fast PWM using ICR1 by setting bits WGM13 and WGM12
   //  CS10 is selected for prescaler = 1,  CS11 is for prescaler =8
   //  Using Fast PWM number 14 on Datasheet   
   //TCCR1B  = _BV(WGM10) | _BV(COM01) | _BV(CS12); 
   //ICR1 = 11059200*0.02/8-1; 


 TCCR1A = 0b00100010; 
 TCCR1B = 0b00011010;    // clkIO/8 = 1MHz
 ICR1 = 20000;            // TOP value -> period = 20ms
	servo_1_move(49);
	servo_2_move(1500);

	//OCR1B = 1500;  //  OCR1B is output to servo 1 on PD4. This is center 

}


void servo_1_move( unsigned int val){

	OCR0 = val;
}

void servo_2_move( unsigned int val){

	OCR1B =  val;

}

unsigned int byteCount = 0;

void writeBuffer(unsigned char * a_pBuffer, unsigned char a_ui8BytesCount)
{

//	static MIRF_MESSAGE mirf_buff;
//	byteCount = byteCount + a_ui8BytesCount;
//  mirf_buff.msg_type = image_buf;
//	mirf_buff.payload_length = a_ui8BytesCount;
//  mirf_buff.payload[0] = a_pBuffer[0]; 
//	mirf_send( (unsigned char *)&mirf_buff, 2);

	return;

}


int main (void)
//############################################################################
{

    // init HW uart for MCA CAM
    usart_init( 19200);

    // init software uart for traces 115200
    sUartInit( 115200 );
    
    // init trace subsystem
    traceInit();
/*    
    // init MCA camera
    mca25_init();
    TRACE( "MCA Init done!");

    mca25_configure();
    TRACE( "MCA Config done!");
    
    // init MIRF module
    mirf_init();
    TRACE( "MIRF Init done!");
    
    // Activate interrupts

    sei();

    // Configure mirf
    mirf_config();
    TRACE( "MIRF Config done!");
*/

    // initialize PSX PAD
    initPSX_PAD();
    TRACE( "PAD  Init done!");

	//init PWM for servos
	servo_init();

    // Init engines
    // enable  PINS PD4-7 as output and set to 0
    DDRD |= (1<<PD7);
    DDRD |= (1<<PD6);
    DDRD |= (1<<PD5);
    DDRD |= (1<<PD3);
    PORTD &= ~(1<<PD7);
    PORTD &= ~(1<<PD6);
    PORTD &= ~(1<<PD5);
    PORTD &= ~(1<<PD3);

	//init fire

    DDRA |= (1<<PA5);
    PORTA &= ~(1<<PA5);


    TRACE( "System Ready!");
	
	unsigned int servo_1_pos =25;
	unsigned int servo_2_pos = 1000;

    //main loop
    while (1)
    {


        _delay_ms(20);
		TRACE("Servo Position1:%i",servo_1_pos)
		//TRACE("Servo Position2:%i",servo_2_pos)

		// enable watchdog, :( because of camera hangs on
		//wdt_disable();
  		//wdt_enable( WDTO_2S);

    //TRACE( "Dir:%x", (unsigned char)testPSX_PAD());
        switch( testPSX_PAD())
        {
            case PS2_DpadL:
			{
				if( servo_1_pos < 49)	servo_1_pos ++;
				servo_1_move( servo_1_pos);
				break;
			}
            case PS2_AXIS2L:
            {
                //Left Engine
                PORTD |= (1<<PD6);
                PORTD &= ~(1<<PD7);
                
                //Right Engine
                PORTD |= (1<<PD5);
                PORTD &= ~(1<<PD3);

                break;
            }
            case PS2_DpadR:    
			{
				if( servo_1_pos > 14)	servo_1_pos --;
				servo_1_move( servo_1_pos);
				break;
			}
            case PS2_AXIS2R:
            {  
                //Left Engine
                PORTD |= (1<<PD7);
                PORTD &= ~(1<<PD6);
                
                //Right Engine
                PORTD |= (1<<PD3);
                PORTD &= ~(1<<PD5);
                break;
            }
            case PS2_DpadU:
			{
				if( servo_2_pos < 3500) servo_2_pos = 10 +servo_2_pos;
				servo_2_move( servo_2_pos);
				break;
			}
            case PS2_AXIS2U:
            {
                //Left Engine
                PORTD |= (1<<PD6);
                PORTD &= ~(1<<PD7);
                
                //Right Engine
                PORTD |= (1<<PD3);
                PORTD &= ~(1<<PD5);
                break;
            }
            case PS2_DpadD:
			{
				if( servo_2_pos > 1000)	servo_2_pos = servo_2_pos -10;
				servo_2_move( servo_2_pos);
				break;
			}
            case PS2_AXIS2D:
            {
                //Left Engine
                PORTD |= (1<<PD7);
                PORTD &= ~(1<<PD6);
                
                //Right Engine
                PORTD |= (1<<PD5);
                PORTD &= ~(1<<PD3);
                break;
            }
            case PS2_Triangle:
            {
                MIRF_MESSAGE mirf_buff;
				mirf_buff.msg_type = start_new_image;
				mirf_buff.payload_length = 0;
				//mirf_send( (unsigned char *)&mirf_buff, 32);

				TRACE( "Grab. result:%i", mca25GrabPreviewImage( writeBuffer));
				TRACE ( "Bytecount:%i", byteCount);
				byteCount = 0;

				// take garbages from the port - fix for camera reset
		  		/* Wait latest receive character was pull from UDR */
  				int i=0;
				while(UCSRA & (1<<RXC))
				{
					i++;
					if( i ==10000) break;
				}

                // lack of break
            }
            case PS2_Circle:
            {
                //char mirf_buff [32]="Ala ma kota !!!";
                //mirf_send( mirf_buff,sizeof( mirf_buff));
                //TRACE( "MIRF Send done!");
                // lack of break
					PORTA |= (1<<PA5);
					break;
            }

            default:
            {
                //Stop engines
                PORTD &= ~(1<<PD7);
                PORTD &= ~(1<<PD6);
                PORTD &= ~(1<<PD5);
                PORTD &= ~(1<<PD3);
				
				//disable fire
				PORTA &= ~(1<<PA5);

                break;
            }
        }
    }

}
