
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
 *	08-Oct-2011  0.1c	TRL - Original code for M162/3 and N2K60 under FreeRTOS, minor bug fix
 *				 0.2	TRL -
 *				 0.3	TRL -
 *
 *
 *	Notes:		1)	Tested with FreeRTOS ver 7.0.1 PIC24/33, N2K60 board
 *				2)  The LCD is written to by more than one task so is controlled by a
 * 					'gatekeeper' task.  This is the only task that is actually permitted to
 *					access the LCD directly.  Other tasks wanting to display a message send
 * 					the message to the gatekeeper queue.
 *				3)  Commands can be send to the LCD using a code of 0x01, then the CMD, see lcd.h
 *				4)	A code of 0x02, then a backlight code of 0x00 for off, > 0x01 for on, see lcd.h
 *				5)  A code of 0x03, is MS4B of CMD row, LS4B of CMD is column, see lcd.h
 *				6)	Hardware delay are used for short delays of less the 1ms. This is a low priority
 *					task and should not cause any real issues
 *				7)
 *
 *	ToDo:		1)  Check if code is atomic or thread safe
 *				2)  Move hello message buffer to a global ??
 *				3)  Do a SPI version for M162/M163 display
 *
 *				xLCDMessage defined in lcd.h
 *				xLCDMessage xMessage = { 0, 0, cStringBuffer };
 *
 *				xMessage.xLcdMode = 3;		// set for row-col mode
 *				xMessage.xLcdCMD  = 0x2 0;	// goto row 2, col 0
 *				//Send the message to the LCD gatekeeper queue for display.
 *				xQueueSend( xLCDQueue, &xMessage, portMAX_DELAY );
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

#define usedLCD   // move to project.h later

#if	defined (usedLCD)

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


/* Project includes. */
#include "HD44780.h"
//#include "retarget.h"
#include "gpio_17xx_40xx.h"
#include "cmsis.h"
#include "timers.h"
xQueueHandle pipe;
xTimerHandle xTimers[ NUM_TIMERS ];
void prvSetupLCD( void);
void prvPulseE(void);
long lExpireCounters[ NUM_TIMERS ] = { 0 };
bool ESTATE=false;
bool LCDINITOK=false;
bool EEND;
uint32_t usTicks;
void microcontroller_delay_us(uint32_t value){
	usTicks = 0;
	value*=70;
	while(usTicks <= value){
		usTicks++;
	};

}
/*!
 * \fn init_delay
 */
void init_delay(void){
	//SysTick_Config(SystemCoreClock / 1000000);


}
/*!
 * \fn deinit_delay
 */
void deinit_delay(void){
	//SysTick_Config(SystemCoreClock / 1);


}
/*
 * The LCD is written to by more than one task so is controlled by this
 * 'gatekeeper' task.  This is the only task that is actually permitted to
 * access the LCD directly.  Other tasks wanting to display a message send
 * the message to the gatekeeper.
 */
static void vLCDTask( void *pvParameters );

/* These are private task for use here and not from other task	*/

/*
 * Setup the peripherals required to communicate with the LCD.
 */


/* 
 * Write a string of text to the LCD. 
 */
static void prvLCDPutString( portCHAR *pcString );


/*-----------------------------------------------------------*/

/* Brief FreeRTOS delay to permit the LCD to catch up with commands. */
#define lcdVERY_SHORT_DELAY	(  1  / portTICK_RATE_MS  )
#define lcdSHORT_DELAY		( 3  / portTICK_RATE_MS )
#define lcdLONG_DELAY		( 40/ portTICK_RATE_MS )

//#define LCDQUEUE						// add queues to registry for debug

/* The length of the queue used to send messages to the LCD gatekeeper task. */
/* The queue size is defined by the size of the LCD display plus 2 */
#define lcdQUEUE_SIZE		2				// 2 should be more that enought
#define PipeSize            1
#define LCDQUEUE
/* The queue used to send messages to the LCD task. */
xQueueHandle xLCDQueue;

/* This task initialised the LCD hardware, starts the queue and then starts the task */
xQueueHandle xStartLCDTask( void )
{
	/* Create the queue used by the LCD task.  Messages for display on the LCD
	are received via this queue. */
	pipe = xQueueCreate(PipeSize,sizeof(uint8_t));
	xLCDQueue = xQueueCreate( lcdQUEUE_SIZE, sizeof( xLCDMessage ) );

#if defined (LCDQUEUE)
	vQueueAddToRegistry (xLCDQueue, (signed char*)"LCD Queue" );
#endif

	/* Start the task that will write to the LCD.  The LCD hardware is
	initialised from within the task itself so that FreeRTOS delays can be used. */
	xTaskCreate( vLCDTask, ( signed portCHAR * ) "LCD", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );

	return xLCDQueue;	
}

/* ************************************************** */

// Configure 4-bit data bus for output
void prvConfigBusAsOutLCD(void) {
	RW_LOW();                  // RW=0 to stop LCD from driving pins
	CONFIG_LCD4_AS_OUTPUT();   // D4
	CONFIG_LCD5_AS_OUTPUT();   // D5
	CONFIG_LCD6_AS_OUTPUT();   // D6
	CONFIG_LCD7_AS_OUTPUT();   // D7
}

// Configure 4-bit data bus for input
void prvConfigBusAsInLCD(void) {
	CONFIG_LCD4_AS_INPUT();   // D4
	CONFIG_LCD5_AS_INPUT();   // D5
	CONFIG_LCD6_AS_INPUT();   // D6
	CONFIG_LCD7_AS_INPUT();   // D7
	RW_HIGH();                // R/W = 1, for read

}

// Output lower 4-bits of u8_c to LCD data lines
//Serial to parallel
void prvOutputToBusLCD(uint8_t u8_c)
{
	uint8_t Res40=u8_c & 0x01;         // D4(0.0)
	uint8_t Res50=((u8_c >> 1)& 0x01);   // D5(0.1)
	uint8_t Res60=((u8_c >> 2)& 0x01);    // D6(0.18)
	uint8_t Res70=((u8_c >> 3)& 0x01); 	  //D7(0.17)
	Chip_GPIO_SetPinState(LPC_GPIO,0,0,(bool)Res40);
	Chip_GPIO_SetPinState(LPC_GPIO,0,1,(bool)Res50);
	Chip_GPIO_SetPinState(LPC_GPIO,0,18,(bool)Res60);
	Chip_GPIO_SetPinState(LPC_GPIO,0,17,(bool)Res70);


}

// Configure the control lines for the LCD
void prvConfigControlLCD(void) {
	CONFIG_RS();    // RS
	CONFIG_RW();    // RW
	CONFIG_E();     // E
	//CONFIG_BL();	// Backlight

	RW_LOW();		// set them all low for now
	E_LOW();
	RS_LOW();
	//BL_LOW();		// Turn on backlight
}


/*!
 * \fn prvPulseE
 * \return : Nothings
 * \brief Pulse on E pin
 *
 */
void prvPulseE(void)
{

	init_delay();
	microcontroller_delay_us(1);
	deinit_delay();
	E_HIGH();
	init_delay();
	microcontroller_delay_us(1);
	deinit_delay();
	E_LOW();
	init_delay();
	microcontroller_delay_us(1);
	deinit_delay();



}

/* Write a byte (u8_Cmd) to the LCD.
u8_DataFlag  is '1' if data byte, '0' if command byte
u8_CheckBusy is '1' if must poll busy bit before write, else simply delay before write
u8_Send8Bits is '1' if must send all 8 bits, else send only upper 4-bits
 */
void prvWriteLCD(uint8_t u8_Cmd, uint8_t u8_DataFlag,
		uint8_t u8_CheckBusy, uint8_t u8_Send8Bits) {
	uint8_t u8_BusyFlag;

	if (u8_CheckBusy)			// are we using busy flag?? 
	{
    RS_LOW();            		// RS = 0 to check busy
    							// check busy
    prvConfigBusAsInLCD();  	// set data pins all inputs
	do {
		E_HIGH();

		microcontroller_delay_us(1);		// read upper 4 bits
      	u8_BusyFlag = GET_BUSY_FLAG();
		E_LOW();
		microcontroller_delay_us(1);
		prvPulseE();              			// pulse again for lower 4-bits
	}
	while (u8_BusyFlag);
	} else { 								// don't use busy, just delay
    	vTaskDelay( lcdSHORT_DELAY );
  		}

	prvConfigBusAsOutLCD();

	if (u8_DataFlag)	RS_HIGH();   		// RS=1, data byte
	else    			RS_LOW();           // RS=0, command byte

	prvOutputToBusLCD(u8_Cmd >> 4);  		// send upper 4 bits
	prvPulseE();

	if (u8_Send8Bits) {
    	prvOutputToBusLCD(u8_Cmd);     		// send lower 4 bits
    	prvPulseE();
  	}

}


/*-----------------------------------------------------------
 *
 *	prvLCDPutString send data to the LCD
 *
 */

static void prvLCDPutString( portCHAR *pcString )
{
	while( *pcString )
	{
		//prvWriteLCD();
		prvWriteLCD(*pcString, 1,0,1);
		pcString++;						// move on to the next char
		vTaskDelay( lcdSHORT_DELAY );
	}
}

/*-----------------------------------------------------------*/
#if defined (M162) || defined (M163)

static void prvSetupLCD( void )			// Setup the display
{
	prvConfigControlLCD ();      		// configure the LCD control lines

	/* Wait for proper power up. */
	vTaskDelay( lcdLONG_DELAY );		// wait for device to settle


	prvWriteLCD(0x30,0,0,0); // Function Set, 8 bit
	vTaskDelay( lcdSHORT_DELAY );

	prvWriteLCD(0x30,0,0,0); // Function Set, 8 bit
	prvWriteLCD(0x30,0,0,0); // Function Set, 8 bit

	prvWriteLCD(0x20,0,0,0); // Function Set,4 bit interface
	prvWriteLCD(0x29,0,0,1); // Function Set,4 bit interface
	prvWriteLCD(0x14,0,0,1); // bias, osc freq

	prvWriteLCD(0x55,0,0,1); // power - icon - contrast
	prvWriteLCD(0x6d,0,0,1); // follower control
	prvWriteLCD(0x79,0,0,1); // contrast set

	prvWriteLCD(0x28,0,0,1); // instruction table 0

	prvWriteLCD(0x0c,0,0,1); // display on
	prvWriteLCD(0x01,0,0,1); // clear on
	prvWriteLCD(0x06,0,0,1); // entry mode, auto increment

	vTaskDelay( lcdSHORT_DELAY );
}

#else	// we must have a HD44780

// Initialize the HD44780 LCD's 2x16, 2x20, 4x20 display
void prvSetupLCD( void )
{

	prvConfigControlLCD ();      		// configure the LCD control lines

	/* Wait for proper power up. */
	vTaskDelay( lcdLONG_DELAY );		// wait for device to settle


  	prvWriteLCD(0x30,0,0,0); // Function Set, 8 bit
	vTaskDelay( lcdSHORT_DELAY );

   	prvWriteLCD(0x30,0,0,0); // Function Set, 8 bit
  	prvWriteLCD(0x30,0,0,0); // Function Set, 8 bit

  	prvWriteLCD(0x20,0,0,0); // Function Set,4 bit interface
  	prvWriteLCD(0x29,0,0,1); // Function Set,4 bit interface
  	prvWriteLCD(0x14,0,0,1); // bias, osc freq

  	prvWriteLCD(0x55,0,0,1); // power - icon - contrast
  	prvWriteLCD(0x6d,0,0,1); // follower control
  	prvWriteLCD(0x79,0,0,1); // contrast set

  	prvWriteLCD(0x28,0,0,1); // instruction table 0

  	prvWriteLCD(0x0c,0,0,1); // display on
  	prvWriteLCD(0x01,0,0,1); // clear on
  	prvWriteLCD(0x06,0,0,1); // entry mode, auto increment
	vTaskDelay( lcdSHORT_DELAY );
}

#endif	// defined (M162) || (M163)

/*-----------------------------------------------------------*/
void vLCDTask( void *pvParameters )
{
	xLCDMessage xMessage;
	portBASE_TYPE xStatus;
	portCHAR uRow = 0;
	portCHAR uCol = 0;

	/* Initialise the hardware.  This uses FreeRTOS delay's so must not be called prior
		to the scheduler being started. */
	prvSetupLCD();

	/* Welcome message. */
	//prvLCDPutString( "*WELCOME :)*");
	//                    1234567890123456			M162/3 display is only 16 char long */

	prvWriteLCD(lcdLINE2,0,0,1);				// Move to row 2, char 1

	for( ;; )
	{

		/* Wait for a message to arrive that requires displaying. */
		xStatus=xQueueReceive( xLCDQueue, &xMessage,portMAX_DELAY );
		while(xStatus!=pdTRUE);

		/* Write each character with appropriate delay between each.
		 *		process LCD command prior to the display on the LCD
		 *
		 *		xLcdMode:
		 *		 = 0x00, normal mode, just display char
		 *		 = 0x01, xLcdCMD is a raw command for LCD display
		 *		 = 0x02, Backlight, xLcdCMD = 0, off, >0 =on
		 *		 = 0x03, xLcdCMD MSB is row, LSB of CMD is colume
		 *		 = 0x04,
		 *
		 *		The command is process prior to writing to the LCD
		 */
		switch (xMessage.xLcdMode)
		{
		default:
		case 0x00:								// just print char on display
			break;
		case 0x01:
			prvWriteLCD(xMessage.xLcdCMD,0,0,1);// send command to LCD
			break;
		case 0x02:								// Backlight
			if (xMessage.xLcdCMD == 0x00)		// if 0, Backlight off
				BL_HIGH();						// backlight off
			else
				BL_LOW();						// backlight on
			break;
		case 0x03:								// goto row and col
			uCol = (xMessage.xLcdCMD & 0x0f);	// mask to 4 bits to get col valus
			uRow = (xMessage.xLcdCMD >> 4);		// shift over row data
			switch (uRow)
			{
			default:
			case 1:
				prvWriteLCD(lcdLINE1 + uCol,0,0,1);
				break;
			case 2:
				prvWriteLCD(lcdLINE2 + uCol,0,0,1);
				break;
			case 3:
				prvWriteLCD(lcdLINE3 + uCol,0,0,1);
				break;
			case 4:
				prvWriteLCD(lcdLINE4 + uCol,0,0,1);
				break;

			} // switch (uRow)

			break;

		} // switch (xMessage.xLcdMode)

		prvLCDPutString(xMessage.pcMessage);
		microcontroller_delay_us(60000);
		prvWriteLCD(0x01,0,0,1);

	} // for( ;; )
}

#endif	// #if defined (useLCD)


