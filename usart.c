/*----------------------------------------------------------------------------
 Copyright:      Radig Ulrich  mailto: mail@ulrichradig.de
 Author:         Radig Ulrich
 Remarks:        
 known Problems: none
 Version:        24.10.2007
 Description:    RS232 Routinen

 Dieses Programm ist freie Software. Sie können es unter den Bedingungen der 
 GNU General Public License, wie von der Free Software Foundation veröffentlicht, 
 weitergeben und/oder modifizieren, entweder gemäß Version 2 der Lizenz oder 
 (nach Ihrer Option) jeder späteren Version. 

 Die Veröffentlichung dieses Programms erfolgt in der Hoffnung, 
 daß es Ihnen von Nutzen sein wird, aber OHNE IRGENDEINE GARANTIE, 
 sogar ohne die implizite Garantie der MARKTREIFE oder der VERWENDBARKEIT 
 FÜR EINEN BESTIMMTEN ZWECK. Details finden Sie in der GNU General Public License. 

 Sie sollten eine Kopie der GNU General Public License zusammen mit diesem 
 Programm erhalten haben. 
 Falls nicht, schreiben Sie an die Free Software Foundation, 
 Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA. 
------------------------------------------------------------------------------*/


#include "usart.h"
	
//----------------------------------------------------------------------------
//Init serielle Schnittstelle
void usart_init(unsigned long baudrate) 
{ 

	#define PARITY_NONE 0
	#define PARITY_EVEN (1<<UPM1)
	#define PARITY_ODD ((1<<UPM1)|(1<<UPM0))
	#define STOP_BITS_1 0
	#define STOP_BITS_2 (1<<USBS)
	#define DATA_BITS_5 0
	#define DATA_BITS_6 (1<<UCSZ0)
	#define DATA_BITS_7 (1<<UCSZ1)
	#define DATA_BITS_8 ((1<<UCSZ1)|(1<<UCSZ0))
	#define DATA_BITS_9 ((1<<UCSZ2)|(1<<UCSZ1)|(1<<UCSZ0)) 
	#define SYNCHRONOUS_TRANSMISION (1<<UMSEL)
	#define ASYNCHRONOUS_TRANSMISION (0<<UMSEL)

//UCSRC =_BV(URSEL)|_BV(UCSZ1)|_BV(UCSZ0);
//UCSRC = (1<<UMSEL)|(1<<USBS)|(1<<UPM1)|(1<<UCSZ1); //synchrniczna
//Set data frame format: asynchronous mode,no parity, 1 stop bit, 8 bit size
//UCSRC=(1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(0<<USBS)|(0<<UCSZ2)|(1<<UCSZ1)|(1<<UCSZ0);	

	// take garbages from the port
  	/* Wait latest receive character was pull from UDR */
  	while(UCSRA & (1<<RXC));

  	/* Wait end of transmission */
  	while(!(UCSRA & (1<<UDRE)));

  	/* Disable transmitter and receiver */
  	UCSRB &= ~((1<<RXEN)|(1<<TXEN));





      /* Set frame format: 8data, 1stop bit */
//   UCSRC = (1<<URSEL)|(3<<UCSZ0); 11059200

#if F_CPU==14745600UL || 11059200UL || 16000000
        USR |= (1 << U2X);           // activate double speed
        UBRR=(F_CPU / (baudrate * 8L) - 1);

#else
		UBRRH = ((F_CPU / ( baudrate * 16UL) - 1) >>8) ;
		UBRRL = ((F_CPU / ( baudrate * 16UL) - 1)); 
		UCSRA = (0<<U2X); 
#endif



	//UCSRC=(1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(1<<UPM0)|(0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);
	//UCSRB = (0<<RXCIE)|0<<TXCIE|(2<<TXEN)|(1<<RXEN|0<<UCSZ2);




	/*
	U2X: Double the USART Transmission Speed
	This bit only has effect for the asynchronous operation. Write this bit to zero when using synchronous
	operation.
	Writing this bit to one will reduce the divisor of the baud rate
	*/

	UCSRC=(1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(0<<USBS)|(0<<UCSZ2)|(1<<UCSZ1)|(1<<UCSZ0);
   /* Enable receiver and transmitter */
   UCSRB = (0<<RXCIE)|(1<<TXEN)|(1<<RXEN);


//    * UDR - USART Data Register : Actually this is not one but two register but when you read it you will get the data stored in receive buffer and when you write data to it goes into the transmitters buffer. This important to remember it.
//    * UCSRA - USART Control and status Register A : As the name suggests it is used to configure the USART and it also stores some status about the USART. There are two more of this kind the UCSRB and UCSRC.
//    * UBRRH and UBRRH : This is the USART Baud rate register, it is 16BIT wide so UBRRH is the High Byte and UBRRL is Low byte. But as we are using C language it is directly available as UBRR and compiler manages the 16BIT access.

}



//----------------------------------------------------------------------------
//Routine für die Serielle Ausgabe eines Zeichens (Schnittstelle0)
void usart_write_char(char c)
{
    //Warten solange bis Zeichen gesendet wurde
    while(!(USR & (1<<UDRE)));
    //Ausgabe des Zeichens
    UDR = c;
	return;
}



//----------------------------------------------------------------------------
//Ausgabe eines Strings
void usart_write_str(char *str)
{
	while (*str)
	{
		usart_write_char(*str++);
	}
}



