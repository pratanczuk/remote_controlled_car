/**********************************************************
* driver for MCA-25 camera
* @Author: Simon Schulz [avr<AT>auctionant.de]
***********************************************************/
#ifndef _MCA25_H_
	#define _MCA25_H_

	//buffer for webcam
	#define CAM_BUFFER_LEN 512
	#define nop()  __asm__ __volatile__ ("nop" ::)

	
	#define MCA25_COMM_BUFFER_LEN 40
	
	//CONFIG:
	#define MCA25_RESET_PORT     PORTB
	#define MCA25_RESET_PORT_DIR DDRB
	#define MCA25_RESET_PIN      0        //PB0
	
	//reset control signal
	#define MCA25_RESET_HI() MCA25_RESET_PORT|=(1<<MCA25_RESET_PIN);
	#define MCA25_RESET_LO() MCA25_RESET_PORT&=~(1<<MCA25_RESET_PIN);
		

	extern PROGMEM char MCA25_CONFIG_640x480[];
	extern PROGMEM unsigned char r_crctable[256];	

	extern void mca25_send_data_ack(void);


	extern void mca25_send_ok(void);

	extern void mca25_configure(void);
	extern void mca25_init(void);

/**
 * Grab one picture in preview mode
 *
 * @param a_pfwriteBuffer pointer to function which store image data
 * @return -1 in case of error, 0 in case of success
 **/

int mca25GrabPreviewImage( void (*a_pfwriteBuffer)(unsigned char *, unsigned char));


/**
 * Read AT packet from the UART in blocking mode
 *
 * @param a_pui8Buffer buffer for incomming AT packet, buffer size has to provided in a_ui16BufferLen
 * @param a_ui16BufferLen contains buffer size
 * @return number of received bytes plus '\0', or error code in case of error
 **/
uint16_t mca25ReadATCommand( unsigned char *a_pui8Buffer, uint16_t a_ui16BufferLen);

/**
 * Read MUX packet from the UART in blocking mode
 *
 * @param a_pui8Buffer buffer for incomming MUX packet, buffer size has to provided in a_ui16BufferLen
 * @param a_ui16BufferLen contains buffer size
 * @return number of received bytes plus '\0', or error code in case of error
 **/
uint16_t mca25ReadMuxPacket( unsigned char *a_pui8Buffer, uint16_t a_ui16BufferLen);

	
#endif
