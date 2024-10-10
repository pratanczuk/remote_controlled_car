/**********************************************************
* driver for MCA-25 camera
*
* @Author   : Simon Schulz [avr<AT>auctionant.de]
*
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation; either version 2 of the License, or (at your option) any later
* version.
*
* This program is distributed in the hope that it will be useful, but
*
* WITHOUT ANY WARRANTY;
*
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc., 51
* Franklin St, Fifth Floor, Boston, MA 02110, USA
*
* http://www.gnu.de/gpl-ger.html
***********************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "usart.h"
#include "mca25.h"
#include "soft_uart.h"
#include "trace.h"
#include "contract.h"


/******************************* CONSTANTS **********************************/


PROGMEM char MCA25_START_MONITORING_MODE[] =
"\x83\x00\x69\x71\x00\x3F"
"<monitoring-command version=\"1.0\" take-pic=\"YES\" zoom=\"10\"/>"
"B\x00\x21x-bt/imaging-monitoring-image"
"\x00\x4c\x00\x06\x06\x01\x80";

PROGMEM char MCA25_START_IMAGE_MODE[] =
"\x83\x00\x82\x71\x00\x58"
"<monitoring-command version=\"1.0\" take-pic=\"NO\" send-pixel-size=\"640*480\" zoom=\"10\"/>"
"B\x00\x21x-bt/imaging-monitoring-image"
"\x00\x4c\x00\x06\x06\x01\x80";

PROGMEM char MCA25_CONFIG_640x480[] =  
"\x82\x01\x3B\x01\x00\x03\x49\x01\x35"
"<camera-settings version=\"1.0\" white-balance=\"OFF\" color-compensation=\"13\" fun-layer=\"0\">"
"<monitoring-format encoding=\"EBMP\" pixel-size=\"80*60\" color-depth=\"8\"/>\r\n"
"<thumbnail-format encoding=\"EBMP\" pixel-size=\"101*80\" color-depth=\"8\"/>\r\n"
"<native-format encoding=\"\" pixel-size=\"640*480\"/>\r\n"
"</camera-settings>\r\n";

PROGMEM char MCA25_CAMERA_INFO[] =
"\x83\x00\x17\x42\x00\x14"
"x-bt/camera-info\x00";

/*
"<camera-settings version="1.0" white-balance="OFF" color-compensation="13" fun-layer="0">"
"<monitoring-format encoding="EBMP" pixel-size="80*60" color-depth="8"/>"
"<thumbnail-format encoding="EBMP" pixel-size="101*80" color-depth="8"/>"
"<native-format encoding="" pixel-size="640*480"/>"
"</camera-settings>"

<camera-settings version="1.0" white-balance="ON" color-compensation="13" fun-layer="0">
<monitoring-format encoding="EBMP" pixel-size="80*60" color-depth="8"/>
<thumbnail-format encoding="JPG" pixel-size="101*80" color-depth="8"/>
<native-format encoding="" pixel-size="640x480"/>
</camera-settings>

*/


PROGMEM unsigned char r_crctable[256] = { //reversed, 8-bit, poly=0x07 
  0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 
  0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B, 
  0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69, 
  0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67, 
  0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D, 
  0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43, 
  0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 
  0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F, 
  0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 
  0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B, 
  0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 
  0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17, 
  0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 
  0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33, 
  0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 
  0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F, 
  0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95, 
  0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B, 
  0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89, 
  0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87, 
  0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 
  0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3, 
  0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 
  0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF, 
  0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 
  0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB, 
  0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 
  0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7, 
  0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD, 
  0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3, 
  0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 
  0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF };

/*
typedef enum E_FSM_STATE_NAME
{
    STATE_0 = 0,
    STATE_1 = 1,
    STATE_2 = 2,
    STATE_3 = 3,
    STATE_4 = 4,
    STATE_5 = 5,
    STATE_6 = 6,
    STATE_7 = 7,
    STATE_8 = 8,
    STATE_9 = 9,
    STATE_10 = 10,
    STATE_11 = 11,
    STATE_12 = 12

} E_FSM_STATE_NAME;

typedef struct S_FSM_STATE
{
    E_FSM_STATE_NAME eState;
      
} S_FSM_STATE;

typedef struct FSM
{  
} S_FSM;

*/

unsigned char make_fcs(const unsigned char *input, int count) 
{
  unsigned char fcs = 0xFF;
  int i;
  for (i = 0; i < count; i++) 
  {
      fcs = pgm_read_byte( &r_crctable[ fcs^input[ i]]);
  }
  return (0xFF-fcs);
}


void send_ch1_data_txt( PGM_P a_pDatabuffer, unsigned int a_ui32bufferSize)
{
    unsigned char PacketSize = 31;
    unsigned char CRCBuffer[3] = {0x81, 0xEF, PacketSize*2+1};
    unsigned char NumOfFullPackets = ( a_ui32bufferSize - 1) / PacketSize;

    unsigned int i=0, j;

    for ( i=0; i <= NumOfFullPackets ; i++ )  
    {
        if( i == NumOfFullPackets ) 
        {
            PacketSize = ( a_ui32bufferSize - 1) % PacketSize;
            CRCBuffer[2] = PacketSize*2+1;
        }

		usart_write_char( '\xF9');
		usart_write_char( '\x81');
		usart_write_char( '\xEF');
		//TRACE_RAW(",0x%x", '\xF9');
		//TRACE_RAW(",0x%x", '\x81');
		//TRACE_RAW(",0x%x", '\xEF');
		usart_write_char( CRCBuffer[2]);
		//TRACE_RAW(",0x%x", CRCBuffer[2]);
        for ( j = 0; j < PacketSize; j++) 
		{
			usart_write_char( pgm_read_byte( (a_pDatabuffer +  (i * 31 + j ))));
			//TRACE_RAW(",0x%x", pgm_read_byte( &MCA25_CONFIG_640x480txt[ i * 31 + j]));
        }
		
		usart_write_char(make_fcs( CRCBuffer, sizeof( CRCBuffer) ));
		usart_write_char( '\xF9');

		//TRACE_RAW(",0x%x",make_fcs( CRCBuffer, sizeof( CRCBuffer) ));
		//TRACE_RAW(",0x%x", '\xF9');
    }

	return;
}


//usart_write_char('\x00');

/*======================================================================
| grab the next x byte data frame
| (cam must bei in jpg capture mode!)
`======================================================================*/
/*
void mca25_grab_data(char *buffer, unsigned int *datalen, char *frametype){
	unsigned int j=0;
	unsigned char togo=31;
	char rx=0;
	unsigned char state=0;
	unsigned char firstframe = 1;

	//enable uart:
	
	*datalen = 0;
	
	// we start with len=0,
	// we extract the packetlength 
	// after the first packet and update len
	//
	// if we re in state12 -> continue, we do not have the full len yet
	while( *datalen==0 || (j<*datalen) || state>99 || state == 12 ){

		while (!(UCSRA & (1<<RXC))){}
		rx = UDR;	
		
		switch(state){
			case 0: 
				//no packet header rcv, wait for header:
				if (rx == 0xF9)
					state = 1;
				//else: do nothing
				break;
			case 1:
				//we got a F9
				if (rx == 0x83)
					state = 2; //we have a data packet
				else if (rx == 0xF9){
					state = 1; //this is the real start byte
				}else{
					state = 0; // we missed something, try again.
				}
				break;
			case 2:
				//now we expect EF
				if (rx == 0xEF){
					state = 3;	// packet ok
				}else{
					if (rx == 0xF9){
						//??? happens sometimes
						state = 1;
					}else{
						state = 0; // something went wrong -> retry
					}
				}
				break;
			case 3: 	
				//next byte is frame len:
				togo = (rx-1)/2; //rx/2
				//printf("len=%d\n",togo);
				//if (togo != 31 && togo != 13) printf("len=%d\n",togo);
				if (firstframe==1)
					state = 10; //get frame info
				else
					state = 100; //grab data
				break;

			case 10:
				//90 01 00 48 00 xx
				//this is the first packet and we 
				//have not sampled anything
				//-> this is 0x90 -> ignore
				togo--;
				state = 11; 
				firstframe = 0;
				break;

			case 11:
				//this is the first packet and 
				//this byte is hi(length)
				*datalen = (unsigned int)(rx<<8);
				togo--;
				state = 12;
				break;

			case 12:
				//this is the first packet and 
				//this byte is lo(length)
				
				*datalen = (unsigned int)*datalen + (unsigned int)(rx) - 6; //substract the first
				                                                            //6byte frame info
				
				//make sure len is valid
				if (*datalen > CAM_BUFFER_LEN){
					*datalen = CAM_BUFFER_LEN;
				}
				
				togo--;
				state = 13;
				break;

			case 13:
				//this is the first packet and 
				//this byte is the frame type 
				*frametype = rx;
				togo--;
				state = 14;
				break;

			case 14:
				//this is the first packet and 
				//this byte is ??? -> ignore
				togo--;
				state = 15;
				break;

			case 15:
				//this is the first packet and 
				//this byte is ??? -> ignore
				togo--;
				state = 100; //now sample data
				break;
			
			case 100:
				//now sample data:
				if (j<CAM_BUFFER_LEN)
					buffer[j] = rx;
				else
				//TRACE("OUT[%03d] 0x%02x\n",j,rx);
				j++;
				
				togo--;
				if (togo == 0)
					state = 101;
				break;

			case 101:
				//data is there now we read checksum:
				//ignore CS...
				//TRACE("CSUM=%02x\n",rx);
				state = 102;
				break;
			case 102: 
				//we have frame end:
				if (rx != 0xF9){
						//MCA25_ERROR_LED_ON();
						//TRACE(PSTR("FRAME ERROR! @packet:%d: (0x%02x) len=%d\n"),j,rx,togo);
				}

				//printf("EOF=%d\n\n",rx);
				state = 0;
				break;
			default:
				TRACE("yeah stack problems. out of mem ? :-X\n");
		}
	}


}
*/

int mca25GrabPreviewImage( void(*a_pfwriteBuffer)(unsigned char *, unsigned char))
{
	unsigned char ui8Buf[MCA25_COMM_BUFFER_LEN];
	unsigned char ui8MUXPacketLen=0;
	unsigned char ui8BlockCount=1;
	unsigned int ui8MUXPacketCount=0;

    mca25ReadMuxPacket( ui8Buf, MCA25_COMM_BUFFER_LEN);

	send_ch1_data_txt( MCA25_START_MONITORING_MODE, sizeof( MCA25_START_MONITORING_MODE));
	
	_delay_ms(10);	


    //send an ok for the power consumption message	
    usart_write_str( "\xF9\x21\xEF\x0D\x0D\x0A\x4F\x4B\x0D\x0A\x48\xF9");	  

    mca25_send_data_ack();

    //wait for image header - string with EBMP 
    for( ui8MUXPacketCount=0; ui8MUXPacketCount <163; ui8MUXPacketCount++)
    {
        mca25ReadMuxPacket( ui8Buf, MCA25_COMM_BUFFER_LEN);

		//\xF9\x83\xEF\x3F\x90\x02\x00\xC3\x00\x00\x12\xCF\x48\x01\xF8\x45\x42\x4D\x50
        if( 0 == memcmp( &ui8Buf[15], "\x45\x42\x4D\x50", 4) )
        {
            ui8BlockCount = 0;
			//a_pfwriteBuffer( &ui8Buf[29], 6);
            break;
        }
    }

    // first packet not found
    if( 1 == ui8BlockCount)
    {
        TRACE( "%s","Error: First packet with EBMP not found");
        return -1;
    }

    // EBMP preview image consist of 161 MUX packets + first Packet with EBMP string at the beginning, so 62 in total
    for( ui8MUXPacketCount=0; ui8MUXPacketCount<161; ui8MUXPacketCount++)
    {
        mca25ReadMuxPacket( ui8Buf, MCA25_COMM_BUFFER_LEN); //read MUX packet 

		switch( ui8Buf[3])
		{
			case 0x21: //"\xF9\x83\xEF\x21"
			{
			    
				//a_pfwriteBuffer( &ui8Buf[4], 16);
				//_delay_ms(2);	
				mca25_send_data_ack();

				ui8BlockCount = 1;

				break;
			}
			case 0x3f: //\xF9\x83\xEF\x3F
			{
           		if( 1 == ui8BlockCount)
            	{
                	ui8BlockCount = 0;
                	//a_pfwriteBuffer( &ui8Buf[10], 25);
            	}
            	else
            	{
                	//a_pfwriteBuffer( &ui8Buf[4], 31);
            	}
				break;
			}
			case 0x31: //"\xF9\x83\xEF\x31
			{
	            //a_pfwriteBuffer(&ui8Buf[4], 23);
    			mca25_send_data_ack();

	    		return 0;
						
				break;
			}
			default:
			{
				//TRACE( "ERROR: Frame not suported:%x",ui8Buf[3]);
				break;
			}
			
		}

    }

    mca25_send_data_ack();

	return -1;

}
 
/*======================================================================
| configure the camera
| (mca25_init() has to be called first !)
`======================================================================*/
void mca25_configure(){
	unsigned char state=0;
	unsigned char buf[MCA25_COMM_BUFFER_LEN];
		
	while (state != 100)
	{
		mca25ReadMuxPacket( buf, MCA25_COMM_BUFFER_LEN); //read MUX packet
		switch (state){
			case 0:

				send_ch1_data_txt( MCA25_CONFIG_640x480, sizeof( MCA25_CONFIG_640x480));
				state = 1;
				break;
			case 1:
				// wait for cam ACK:
				// [F9 83 EF 07 A0 00 03 C7 F9
				if ( memcmp(buf,"\xF9\x83\xEF\x07\xA0\x00\x03\xC7\xF9",9) == 0)
				{
					// request MCA25_CAMERA_INFO
					send_ch1_data_txt( MCA25_CAMERA_INFO, sizeof( MCA25_CAMERA_INFO));
	
					state = 2;
				}
				break;
			case 2:
				// ignore camera info ...
				// -> wait for last info packet:
				// [F9 83 EF 33 79 65 72 3D 22 31 30 22 2F 3E 3C 2F 
				//  63 61 6D 65 72 61 2D 69 6E 66 6F 3E 00 E4 F9]
				if (memcmp( buf,"\xF9\x83\xEF\x33\x79\x65\x72\x3D\x22\x31\x30\x22"
											 "\x2F\x3E\x3C\x2F\x63\x61\x6D\x65\x72\x61\x2D\x69"
											 "\x6E\x66\x6F\x3E\x00\xE4\xF9",31)){
						//CAM READY !
						state = 100;
				}
				break;
		}
        

	}
}


/*======================================================================
| initialise the camera
| this has to be done at first, it activates the mux mode also
`======================================================================*/
void mca25_init(void)
{
	unsigned char state=0;
	unsigned char buf[MCA25_COMM_BUFFER_LEN];
	
	MCA25_RESET_PORT_DIR |=  (1<<MCA25_RESET_PIN); //make camreset pin output
	
	
	TRACE( "%s", "CAM_RUNNING");

	MCA25_RESET_LO();
	
	_delay_ms(10);
	
	MCA25_RESET_HI();


	while (state != 100)
    {

        if (state<10)
		{
			mca25ReadATCommand( buf, MCA25_COMM_BUFFER_LEN); //read AT command
			TRACE_RAW( "%s", "read AT command: ");
			TRACE_RAW( "%s",( char *)buf);
			TRACE_RAW( "%s", "\r\n");
		}
		else
        {
			mca25ReadMuxPacket( buf, MCA25_COMM_BUFFER_LEN); //read MUX packet
		}

        TRACE( "State: %i", state);


		switch (state)
		{
            case 0:
            {
				//we have to wait for AT&F:
				if (memcmp(buf,"AT&F",4) == 0)
				{					
					//usart_write_P(buf);
					mca25_send_ok();
					state = 1;
				}

				break;
            }				
            case 1:
            {
				//wait for AT+IPR
				if ( memcmp( buf, "AT+IPR=?", 8) == 0)
				{
				    //usart_write_str( "AT+IPR=?");
					usart_write_str( "+IPR: (),(1200,2400,4800,9600,19200,38400,57600,115200,460800)\r\n\r\nOK\r\n");
                    state = 2;
				}
				break;            
            }
            case 2:
            {
				//wait for AT+IPR SET command
				if ( memcmp(buf,"AT+IPR=460800",13) == 0)
                {
					usart_write_str( "\r\nOK\r\n");
					//set higher baudrate:

					_delay_ms(5);
					usart_init( 460800);
					
                    state = 3;
				}
				break;            
            }
            case 3:
            {
				//wait for mux info req
				if ( memcmp(buf,"AT+CMUX=?",9) == 0)
                {
				    usart_write_str( (char*)buf);
				    usart_write_str( "\r\r\n+CMUX: (0),(0),(1-7),(31),(10),(3),(30),(10),(1-7)\r");
					state = 4;
				}
				break;
            
            }
			case 4:
            {
                //wait for mux enable request
				if (memcmp(buf,"AT+CMUX=0,0,7,31",16) == 0)
                {
					//mca25_send_ok();
					usart_write_str("\r\r\nOK\r\n");
					state = 10;
				}
				break;
            }
			case 10:
            {
                // wait for mux ch0 request:
				// [F9 03 3F 01 1C F9]
				if (memcmp(buf,"\xF9\x03\x3F\x01\x1C\xF9",6) == 0)
                {
					// send mux ch0 ack/open packet:
					// [F9 03 73  01 D7 F9]
					usart_write_str("\xF9\x03\x73\x01\xD7\xF9");
					state = 11;
				}
				break;
            }
			case 11:
            {
                // wait for mux ch3 request:
				// [F9 23 3F 01 C9 F9]
				if (memcmp(buf,"\xF9\x23\x3F\x01\xC9\xF9",6) == 0)
                {
					// send mux ch3 ack/open packet:
					// [F9 23 73 01 02 F9]
					usart_write_str( "\xF9\x23\x73\x01\x02\xF9");
                    state = 12;
				}
				break;
            }
            case 12:
            {
                // wait for config mux ch0 request:
                // [F9 03 EF 09 E3 05 23 8D FB F9]
                if ( memcmp(buf,"\xF9\x03\xEF\x09\xE3\x05\x23\x8D\xFB\xF9",10) == 0)
                {
                    // send mux ch0 config ack/open packet:
                    // [F9 01 EF 0B E3 07 23 0C 01 79 F9]
                    usart_write_str( "\xF9\x01\xEF\x0B\xE3\x07\x23\x0C\x01\x79\xF9");
                    state = 13;
                }
                break;
            }
            case 13:
            {
                // wait for config mux ch3 request:
                // [F9 03 EF 09 E1 07 23 0C 01 FB F9]
                if (memcmp(buf,"\xF9\x03\xEF\x09\xE1\x07\x23\x0C\x01\xFB\xF9",11) == 0)
                {
                    // send mux ch3 config ack/open packet:
                    // [F9 01 EF 09 E1 05 23 8D 9A F9]
                    usart_write_str( "\xF9\x01\xEF\x09\xE1\x05\x23\x8D\x9A\xF9");
                    state = 14;
                }
                break;            
            }
            case 14:
            {
				// wait for AT*EACS.17.1.r:
				// [F9 23 EF 1B 41 54 2A 45 41 43 53 3D 31 37 2C 31 0D D1 F9]
				//if (memcmp(buf,"\xF9\x23\xEF\x1BAT*EACS=17,1\r\xD1\xF9",19) == 0){
				if (memcmp( buf,"\xF9\x23\xEF\x1B\x41\x54\x2A\x45\x41\x43"
                                "\x53\x3D\x31\x37\x2C\x31\x0D\xD1\xF9",19) == 0)
                {
					// send mux "\r\nOK\r\n" packet:
					// [F9 21 EF 0D 0D 0A 4F 4B 0D 0A 48  F9]
					usart_write_str( "\xF9\x21\xEF\x0D\x0D\x0A\x4F\x4B\x0D\x0A\x48\xF9");
					state = 15;
				}
				break;
            
            }
            case 15:
            {
                // wait for AT+CSCC=1,199\r5 peripheral AUTH req:
                // [F9 23 EF 1D 41 54 2B 43 53 43 43 3D 31 2C 31 39 39 0D 35 F9]
                if (memcmp(buf,"\xF9\x23\xEF\x1D\x41\x54\x2B\x43\x53\x43\x43"
                               "\x3D\x31\x2C\x31\x39\x39\x0D\x35\xF9",20) == 0)
                {
                    // send response token:
                    //DONT// [F9 21 EF 1D 41 54 2B 43 53 43 43 3D 31 2C 31 39 39 0D 54 F9]
                    // [F9 21 EF 1B 0D 0A 2B 43 53 43 43 3A 20 45 33 0D 0A B0 F9 ]
                    usart_write_str( "\xF9\x21\xEF\x1B\x0D\x0A\x2B\x43\x53\x43"
                                     "\x43\x3A\x20\x45\x33\x0D\x0A\xB0\xF9");

                    //append ok:
                    usart_write_str( "\xF9\x21\xEF\x0D\x0D\x0A\x4F\x4B\x0D\x0A\x48\xF9");
                    state = 16;
                }
                break;
            
            }
			case 16:
            {
                // wait for AT+CSCC=2,199.B9\r AUTH2 req:
				// AT+CSCC.2.199.B9.r|
				// [F9 23 EF 23 41 54 2B 43 53 43 43 3D 32 2C 31 39 39 2C 42 39 0D FB F9]
				if (memcmp(buf,"\xF9\x23\xEF\x23\x41\x54\x2B\x43\x53\x43\x43"
							   "\x3D\x32\x2C\x31\x39\x39\x2C\x42\x39\x0D\xFB\xF9",23) == 0)
                {
					// send response token: (OK)
					// [F9 21 EF 0D 0D 0A 4F 4B 0D 0A 48  F9]
					usart_write_str( "\xF9\x21\xEF\x0D\x0D\x0A\x4F\x4B\x0D\x0A\x48\xF9");
						
					// now request data mux channel (ch1):
					// [F9 81 3F 01 AB F9]
					usart_write_str( "\xF9\x81\x3F\x01\xAB\xF9");
				    state = 17;
				}
				break;
            }
			case 17:
            {
                // wait for mux ch1 ack:
                // [F9 81 73 01 60 F9]
                if ( memcmp(buf,"\xF9\x81\x73\x01\x60\xF9",6) == 0)
                {
                    // channel1 is now open!
                    state = 18;
                }
                break;
            }
            case 18:
            {
                // wait for ch1 mux config:
                // [F9 03 EF 09 E3 05 83 8D FB F9]
                if (memcmp(buf,"\xF9\x03\xEF\x09\xE3\x05\x83\x8D\xFB\xF9",10) == 0)
                {
                    // send config response:
                    // [F9 01 EF 09 E1 05 83 8D 9A F9]
                    usart_write_str( "\xF9\x01\xEF\x09\xE1\x05\x83\x8D\x9A\xF9");

                    // now configure cam mode:
                    // [ F9 81 EF 37 80 00 1A 10 00 01 00 46 00 13 E3 3D
                    //   95 45 83 74 4A D7 9E C5 C1 6B E3 1E DE 8E 61 82 F9 ]
                    // F9 81 EF 37 			= mux header
                    // 80 00 1A 10 00 	= ???
                    // 01 00 						= (256-6)Byte data, then wait for ACK
                    // 46  00 13 E3 3D 95 45 83 74 4A D7 9E C5 C1 6B E3 1E DE 8E 61 ???
                    usart_write_str( "\xF9\x81\xEF\x37\x80");
                    usart_write_char('\x00');
                    usart_write_str( "\x1A\x10");
                    usart_write_char('\x00');

                    //buffsize
                    usart_write_str( "\x02");
                    usart_write_char('\x00');

                    usart_write_str( "\x46");
                    usart_write_char('\x00');
                    usart_write_str( "\x13\xE3\x3D\x95\x45\x83\x74\x4A\xD7");
                    usart_write_str( "\x9E\xC5\xC1\x6B\xE3\x1E\xDE\x8E\x61\x82\xF9");

                    usart_write_str( "\xF9\x21\xEF\x0D\x0D\x0A\x4F\x4B\x0D\x0A\x48\xF9");	  

                    state = 19;
                }
                break;            
            }
            case 19:
            {
                // cam should now accept our settings:
                // [F9 83 EF 3F A0 00 1F 10 00 20 00 CB 00 00 00 01 4A 00
                //  13 E3 3D 95 45 83 74 4A D7 9E C5 C1 6B E3 1E DE 8E ED F9
                if (memcmp(buf,"\xF9\x83\xEF\x3F\xA0\x00\x1F\x10\x00\x20\x00\xCB\x00"
                               "\x00\x00\x01\x4A\x00\x13\xE3\x3D\x95\x45\x83\x74\x4A"
                               "\xD7\x9E\xC5\xC1\x6B\xE3\x1E\xDE\x8E\xED\xF9",37) == 0)
                {
                    state = 100; //-> exit init loop.
                }
                break;			
            }

			default:
            {
                ILLEGAL_DEFAULT_CASE( state);
                break;                
            }
		}
	}
}



/************************** HELPER ***********************************/
void mca25_send_data_ack(){
	usart_write_str( "\xF9\x81\xEF\x07\x83");
	usart_write_char('\x00');
	usart_write_str("\x03\xA6\xF9");
}


void mca25_send_ok(){
	//send_cmd("\r\r\nOK\r\n");
	usart_write_str("\r\r\nOK\r\n"); //puts adds a newline !!
}



uint16_t mca25ReadMuxPacket( unsigned char *a_pui8Buffer, uint16_t a_ui16BufferLen)
{

	uint16_t uiCount				= 0;

	for( uiCount=0; uiCount < a_ui16BufferLen - 1; uiCount++)
	{
		while ( !( UCSRA & ( 1<<RXC)))
        {
		}
		

		a_pui8Buffer[ uiCount] = UDR;

		if ( uiCount>0 && a_pui8Buffer[ uiCount] == '\xF9')
		{
			break; //we have finished out read.
		}
	}
    
    uiCount++;
    a_pui8Buffer[ uiCount] = '\0';

    return uiCount;

 }

uint16_t mca25ReadATCommand( unsigned char *a_pui8Buffer, uint16_t a_ui16BufferLen)
{
	uint16_t uiCount;

	for( uiCount=0; uiCount < a_ui16BufferLen; uiCount++)
	{

        while ( !(UCSRA & (1<<RXC)))
        {
        }
 
		a_pui8Buffer[ uiCount] = UDR;

		if ( a_pui8Buffer[ uiCount] == '\r' || a_pui8Buffer[ uiCount] == '\n')
		{
			break; //we have finished out read.
		}
        
        /*
        // errors detection
        if((UCSRA & (1<<FE))) sUartPrintString( "RX FE: Frame Error\r\n"); // If frame error or parity error
        if((UCSRA & (1<<PE))) sUartPrintString( "RX PE: Parity Error\r\n"); // If frame error or parity error
        if((UCSRA & (1<<DOR))) sUartPrintString( "RX DOR: Data OverRun Error\r\n"); // If frame error or parity error
        */
	}
    
	a_pui8Buffer[ uiCount] = '\0';

	return uiCount;
}




