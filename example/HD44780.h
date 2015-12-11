/*
*	TITLE		LCD drivers for FreeRTOS using M162/M163/HD44780
*					
*	VERSION:	0.1c (Beta)                           
*
*	DATE:		08-Oct-2011
*
*	AUTHOR:		Tom Lafleur  
*				lafleur@lafleur.us
*
*	COMMENTS:	You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or
*				elsewhere without permission. Thanks.
*
*				Copyright �2005-2011 R.  . All rights are reserved.
*
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	08-Oct-2011  0.1c	TRL - Original code for M162 and N2K60 under FreeRTOS
*				 0.2	TRL - 
*				 0.3	TRL - 
*
*
*	Notes:		1)	Tested with FreeRTOS ver 7.0.1 PIC24/33, N2K60 board
*				2)  The LCD is written to by more than one task so is controlled by a
* 					'gatekeeper' task.  This is the only task that is actually permitted to
*					access the LCD directly.  Other tasks wanting to display a message send
* 					the message to the gatekeeper.
*				3)  Commands can be send to the LCD using a code of 0x01, then the CMD, see lcd.h
*				4)	A code of 0x02, then a backlight code of 0x00 for off, > 0x01 for on, see lcd.h
*				5)  A code of 0x03, is MS4B of CMD row, LS4B of CMD is column, see lcd.h
*				6)	Hardware delay are used for short delays of less the 1ms. This is a low priority
*					task and should not cause any real issues
*
*	ToDo:		1)  Check if code is atomic or thread safe
*				2)  Move hello message buffer to a global
*
*
*				xMessage.xLcdMode = 3;		// set for row-col mode
*				xMessage.xLcdCMD  = 0x2 0;	// goto row 2, col 0
*				//Send the message to the LCD gatekeeper for display. 
*				xQueueSend( xLCDQueue, &xMessage, portMAX_DELAY );
*
*				xLcdMode:
*		 			= 0x00, normal mode, just display char
*		 			= 0x01, xLcdCMD is a raw command for LCD display
*		 			= 0x02, Backlight, xLcdCMD = 0, off, >0 =on
*					= 0x03, xLcdCMD MSB is row, LSB of CMD is colume
*		 			= 0x04, 
*	 			The cmd is process prior to writeing to the LCD
*
*
*/

#ifndef LCD_INC_H
#define LCD_INC_H
/* Brief FreeRTOS delay to permit the LCD to catch up with commands. */
#define DELAY_US(n) {unsigned long x; for(x=0; x<(n*50); x++){asm("NOP"); }}
/*	Display type:
 *
 *	M162	is 2 x 16
 *	M163	is 3 x 16
 *	Default is an HD44780 LCD's 2x16, 2x20, 4x20 display
 *
*/
 #define NUM_TIMERS 5
//#define M162		// supports for M162 2x16
#define HD44780		// supports for M163 3x16
#define LPC1768EA
/* LCD commands. */
#define lcdCLEAR			( 0x01 )
#define lcdHOME				( 0x02 )

#if defined (M162)

#define lcdLINE1			( 0x80 )
#define lcdLINE2			( 0xc0 )
#define lcdLINE3			( 0x94 )	// we only have two row in this device, but we can load up RAM
#define lcdLINE4			( 0xd4 )	// in row3 and row4

#elif defined (M163)

#define lcdLINE1			( 0x80 )	// need to check these
#define lcdLINE2			( 0x90 )
#define lcdLINE3			( 0xa0 )
#define lcdLINE4			( 0x80 )	// go back to 1st line

#else // HD44780

#define lcdLINE1			( 0x80 )
#define lcdLINE2			( 0xc0 )
#define lcdLINE3			( 0x94 )
#define lcdLINE4			( 0xd4 )

#endif


// N2K60 project Board
#if defined	(LPC1768EA)

// Define LCD display options
#define 	lcd4bit

#if defined 	(M162)

#define		lcdrow		2						// number of rows
#define		lcdchar		16						// number of char

#elif defined	(M163)

#define		lcdrow		3						// number of rows
#define		lcdchar		16						// number of char

#else

#define		lcdrow		2						// number of rows
#define		lcdchar		20						// number of char
#endif

#define 	lcdsize		(lcdrow*lcdchar)		// number of rows * char per row

// Define device I/O pins
#define RS_HIGH()         Chip_GPIO_SetPinOutHigh(LPC_GPIO,0,8)//CMD
//#define RS_HIGH()        _LATC6 = 1				// RS, Command = 0, Data = 1
//#define RS_LOW()         _LATC6 = 0
#define RS_LOW()         Chip_GPIO_SetPinOutLow(LPC_GPIO,0,8)				//DATA
#define CONFIG_RS()      Chip_GPIO_WriteDirBit(LPC_GPIO,0,8,true)		// set as output = 0

#define RW_HIGH()        Chip_GPIO_SetPinOutHigh(LPC_GPIO,0,7)			// Read-Write, Raed = 1, Write = 0
#define RW_LOW()         Chip_GPIO_SetPinOutLow(LPC_GPIO,0,7)
#define CONFIG_RW()      Chip_GPIO_WriteDirBit(LPC_GPIO,0,7,true)	// set as output = 0

#define E_HIGH()         Chip_GPIO_SetPinOutHigh(LPC_GPIO,0,6)			// Enable, falling edge
#define E_LOW()          Chip_GPIO_SetPinOutLow(LPC_GPIO,0,6)
#define CONFIG_E()       Chip_GPIO_WriteDirBit(LPC_GPIO,0,6,true)	// set as output = 0
#define INPUT_E()        Chip_GPIO_WriteDirBit(LPC_GPIO,0,6,false)

#define BL_HIGH()        Chip_GPIO_SetPinOutHigh(LPC_GPIO,0,0)			// Backlight	= off
#define BL_LOW()         Chip_GPIO_SetPinOutLow(LPC_GPIO,0,0)			// 				= on
#define CONFIG_BL()      Chip_GPIO_WriteDirBit(LPC_GPIO,0,0,true)	// set as output = 0

#define LCD4O			_LATC0					// Data Port bits
#define LCD5O			_LATC1
#define LCD6O          	_LATC2
#define LCD7O          	_LATC3

#define LCD7I         	Chip_GPIO_GetPinState(LPC_GPIO,0,25)				// Busy flag
	
#define CONFIG_LCD4_AS_INPUT()  Chip_GPIO_WriteDirBit(LPC_GPIO,0,0,false)
#define CONFIG_LCD5_AS_INPUT()  Chip_GPIO_WriteDirBit(LPC_GPIO,0,1,false)
#define CONFIG_LCD6_AS_INPUT()  Chip_GPIO_WriteDirBit(LPC_GPIO,0,18,false)
#define CONFIG_LCD7_AS_INPUT()  Chip_GPIO_WriteDirBit(LPC_GPIO,0,17,false)

#define CONFIG_LCD4_AS_OUTPUT() Chip_GPIO_WriteDirBit(LPC_GPIO,0,0,true)
#define CONFIG_LCD5_AS_OUTPUT() Chip_GPIO_WriteDirBit(LPC_GPIO,0,1,true)
#define CONFIG_LCD6_AS_OUTPUT() Chip_GPIO_WriteDirBit(LPC_GPIO,0,18,true)
#define CONFIG_LCD7_AS_OUTPUT() Chip_GPIO_WriteDirBit(LPC_GPIO,0,17,true)

#define GET_BUSY_FLAG()  LCD7I

#else

#warning	" LCD configuration is NOT defined in lcd.c " 

#endif  // Device type


/* Create the task that will control the LCD.  Returned is a handle to the queue
on which messages to get written to the LCD should be written. */
xQueueHandle xStartLCDTask( void );

/*  This struct define the mesage and information on what to do first
 *	
 *				xLcdMode:
 *		 			= 0x00, normal mode, just display char
 *		 			= 0x01, xLcdCMD is a raw command for LCD display
 *		 			= 0x02, Backlight, xLcdCMD = 0, off, >0 =on
 *					= 0x03, xLcdCMD MSB is row, LSB of CMD is colume
 *		 			= 0x04, 
 *	 			The cmd is process prior to writing to the LCD
 *		
*/
typedef struct
{
	portCHAR xLcdMode;				// 0x00 = normal
	portCHAR xLcdCMD;				// LCD command
	portCHAR pcMessage[lcdsize];	//  string to be displayed. LCD size

}  __attribute__((__packed__, aligned(2))) xLCDMessage;

void prvWriteLCD(uint8_t u8_Cmd, uint8_t u8_DataFlag,
		uint8_t u8_CheckBusy, uint8_t u8_Send8Bits);
void microcontroller_delay_us(uint32_t value);
void init_delay(void) ;
void deinit_delay(void);
#endif /* LCD_INC_H */


