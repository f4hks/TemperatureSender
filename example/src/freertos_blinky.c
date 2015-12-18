/*
 * @brief FreeRTOS Blinky example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis.h"
#include "timers.h"
#include "queue.h"
#include "ax25.h"
#include "string.h"
#include "tmpSensor.h"
//#include "lcd.h"
#include "HD44780.h"
#include <stdlib.h>
P_AX_25 trame_ax25;
AX_25_CFG cfgdatas;
AX_25_SEND_DATA datasenvoye;
AX_25 tramegenere;
xQueueHandle xQueue=NULL;
xQueueHandle xLCDQueue=NULL;
xQueueHandle xMesureQueue=NULL;
xQueueHandle xCfgMeusureQueue=NULL;
xQueueHandle xConfigConAX25=NULL;
xQueueHandle xAX25pipedata=NULL;
xQueueHandle xAX25datas=NULL;
xTaskHandle TskcfgAX25=NULL;
xTaskHandle TskTemp=NULL;
xTaskHandle TsksendAX25=NULL;
xTaskHandle TskGetx25=NULL;
portBASE_TYPE xStatusADCCFG=0;
portBASE_TYPE xStatusADCRESULT=0;
portBASE_TYPE xStatusAX25CodingTask;
portBASE_TYPE xStatusAX25sendTask;
portBASE_TYPE xStatusAX25GetData;
xLCDMessage tempe;
xLCDMessage toto= { 0, 0, "LCD TEST" };

struct AMessage
{
  char ucMessageID;
  char ucData[ 20 ];
};
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
xTimerHandle MonTimer;
/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
#if 0
const unsigned short sine[256]={0x8000,0x8647,0x8c8b,0x92c7,0x98f8,0x9f19,0xa527,0xab1f,
    0xb0fb,0xb6b9,0xbc56,0xc1cd,0xc71c,0xcc3f,0xd133,0xd5f5,
    0xda82,0xded7,0xe2f1,0xe6cf,0xea6d,0xedc9,0xf0e2,0xf3b5,
    0xf641,0xf884,0xfa7c,0xfc29,0xfd89,0xfe9c,0xff61,0xffd8,
    0xffff,0xffd8,0xff61,0xfe9c,0xfd89,0xfc29,0xfa7c,0xf884,
    0xf641,0xf3b5,0xf0e2,0xedc9,0xea6d,0xe6cf,0xe2f1,0xded7,
    0xda82,0xd5f5,0xd133,0xcc3f,0xc71c,0xc1cd,0xbc56,0xb6b9,
    0xb0fb,0xab1f,0xa527,0x9f19,0x98f8,0x92c7,0x8c8b,0x8647,
    0x8000,0x79b8,0x7374,0x6d38,0x6707,0x60e6,0x5ad8,0x54e0,
    0x4f04,0x4946,0x43a9,0x3e32,0x38e3,0x33c0,0x2ecc,0x2a0a,
    0x257d,0x2128,0x1d0e,0x1930,0x1592,0x1236,0xf1d,0xc4a,
    0x9be,0x77b,0x583,0x3d6,0x276,0x163,0x9e,0x27,
    0x0,0x27,0x9e,0x163,0x276,0x3d6,0x583,0x77b,
    0x9be,0xc4a,0xf1d,0x1236,0x1592,0x1930,0x1d0e,0x2128,
    0x257d,0x2a0a,0x2ecc,0x33c0,0x38e3,0x3e32,0x43a9,0x4946,
    0x4f04,0x54e0,0x5ad8,0x60e6,0x6707,0x6d38,0x7374,0x79b8};
#endif
/*****************************************************************************
 * Private functions
 ****************************************************************************/
#if 0
/*!
 *
 *
 * @fn vTimerCallback
 * @brief Fonction de retour timer
 * */
static void vTimerCallback( xTimerHandle pxTimer )
{

}
#endif
#if 0
/*Config PWM*/
static void ConfigPwm(int Freq){


}
#endif
static void EnvoieConfig(void){
  for(;;)
    {

      xTmpSensorCfg cfgMe={44000,false};
      if(xCfgMeusureQueue!=NULL){
	  xStatusADCCFG=xQueueSend(xCfgMeusureQueue,&cfgMe,portMAX_DELAY );

	  if(xStatusADCCFG!= pdPASS )
	    {
	      /* The send operation could not complete because the queue was full -
											this must be an error as the queue should never contain more than
											one item! */
	      Board_UARTPutSTR( "ADC config failed !.\r\n" );
	    }
	  else{

	      vTaskSuspend(NULL);
	  }
      }
      else{Board_UARTPutSTR( "Config ADC queue = NULL !.\r\n" );}

    }
}
static void xTaskGetTemperatureResults(void)
{

  xTmpSensorResult Result;
  char buf[4];
  char Celcius[6];
  portBASE_TYPE xStatus;
  portBASE_TYPE xStatus1;
  portBASE_TYPE xStatus2;

  for(;;){
      //vTaskSuspend(TsksendAX25);//Arret de la tache sendAX25
      if(xMesureQueue!=NULL)
	{
	  //vTaskSuspend(TsksendAX25);
	  xStatusADCRESULT=xQueueReceive(xMesureQueue,&Result,portMAX_DELAY );
	  if(xStatusADCRESULT!= pdPASS )
	    {
	      /* The send operation could not complete because the queue was full -
												this must be an error as the queue should never contain more than
												one item! */
	      Board_UARTPutSTR( "ADC get results fail !.\r\n" );
	    }
	  else{
	      tempe.xLcdCMD=0;
	      tempe.xLcdMode=0;
	      //vTaskSuspend(NULL);
	      Board_UARTPutSTR("Value on ADC : ");
	      snprintf(buf,sizeof(buf),"%d",Result.Temperature);
	      snprintf(Celcius,sizeof(Celcius),"%f",Result.temp_finale);
	      Board_UARTPutSTR((char*)buf);
	      Board_UARTPutSTR("\n\r");
	      Board_UARTPutSTR("Temperature :");
	      Board_UARTPutSTR((char*)Celcius);
	      Board_UARTPutSTR("°C\n\r ");
	      //tempe.pcMessage=(char*)Celcius;
	      memset(tempe.pcMessage, 0, sizeof(tempe.pcMessage));
	      strncpy(tempe.pcMessage,(char*)"Temp :",strlen("Temp :"));
	      strncat(tempe.pcMessage,Celcius,sizeof(Celcius));
	      strncat(tempe.pcMessage,(char*)" C",2);

	      xStatus = xQueueSend( xLCDQueue, &tempe, portMAX_DELAY );
	      if( xStatus != pdPASS )
		{
		  /* The send operation could not complete because the queue was full -
	      										this must be an error as the queue should never contain more than
	      										one item! */
		  Board_UARTPutSTR( "Could not send to the queue.\r\n" );
		}
	      else{

		  //vTaskResume(TsksendAX25);
	      }


	      //xStatus2=xQueueSend(xConfigConAX25,&cfgdatas,portMAX_DELAY);
	      //if(xStatus2){
	      //Board_UARTPutSTR("Fail !\n\r");
	      //}



	  }

	  vTaskDelay(3000*portTICK_RATE_MS);
	}

  }


}

static void taskSendParameters(void *pVparam)
{
  for(;;){
      //vTaskSuspend(TsksendAX25);
      cfgdatas.send_id=(unsigned portCHAR*)"F4HKS";
      portBASE_TYPE xStatusCFGSEND;
      xStatusCFGSEND=xQueueSend(xConfigConAX25,&cfgdatas,portMAX_DELAY);
      if(xStatusCFGSEND!=pdTRUE){
	  //vTaskSuspend(TskTemp);
	  Board_UARTPutSTR("Fail on taskSendParameters !\n\r");
	  //vTaskResume(TskTemp);

      }
      else{
	  //vTaskSuspend(TskTemp);
	  //Board_UARTPutSTR("Success on taskCreateMessage !\n\r");
	  //vTaskResume(TskTemp);


      }
      //vTaskResume(TsksendAX25);
      vTaskSuspend(NULL);
      //vTaskDelay(30*portTICK_RATE_MS);
  }
}

static void taskCreateMessage(void *pVparam)
{
  for(;;){

      //vTaskSuspend(TskTemp);
      //vTaskSuspend(TskGetx25);
      cfgdatas.send_id=(unsigned portCHAR*)"F4HKS";
      portBASE_TYPE xStatusCFGSEND;
      xStatusCFGSEND=xQueueSend(xConfigConAX25,&cfgdatas,portMAX_DELAY);
      if(xStatusCFGSEND!=pdTRUE){
	  //vTaskSuspend(TskTemp);
	  Board_UARTPutSTR("Fail on taskSendParameters !\n\r");
	  //vTaskResume(TskTemp);

      }
      else{
	  //vTaskSuspend(TskTemp);
	  //Board_UARTPutSTR("Success on taskCreateMessage !\n\r");
	  //vTaskResume(TskTemp);


      }
      datasenvoye.dest_id=(unsigned portCHAR*)"WIDE";
      datasenvoye.message=(unsigned portCHAR*)"The quick brown fox jumps over the lazy dog ";
      xStatusAX25CodingTask=xQueueSend(xAX25datas,&datasenvoye,portMAX_DELAY);
      if(xStatusAX25CodingTask!=pdPASS){
	  Board_UARTPutSTR("Erreur !\n\r");
      }
      else

	{
	  //vTaskSuspend(TskTemp);
	  //Board_UARTPutSTR("Success on taskCreateMessage !\n\r");


	}

      //cfgdatas.send_id="F4HKS";

      //vTaskResume(TskTemp);
      //vTaskResume(TskGetx25);
      //vPortFree(trame_ax25);//important





  }
}
static void taskGetMessages(void *pVparam){
  for(;;){
      xStatusAX25GetData=xQueueReceive(xAX25pipedata,&tramegenere,portMAX_DELAY);
      if(xStatusAX25GetData!=pdTRUE){
	  Board_UARTPutSTR("Fail on get ax 25 \n\r");
      }
      else{
	  Board_UARTPutSTR((char*)tramegenere.fullmessage);
	  Board_UARTPutSTR("\r\r");
      }
      vTaskDelay(2000/portTICK_RATE_MS);



  }


}
/* Sets up system hardware */
static void prvSetupHardware(void)
{
  SystemCoreClockUpdate();
  Board_Init();


  /* Initial LED0 state is off */
  Board_LED_Set(0, false);
  //Board_LED_Set(2, false);
}
/* AX25 thread
 * Appel de la fonction situé dans ax25.c */
#if 0
static void vAX25CodingTask(void *pvParameters){
  //portTickType xLastWakeTime;
  bool LedState = false;
  portBASE_TYPE xStatus;
  //xLastWakeTime = xTaskGetTickCount();
  for( ;; ){
      Board_UARTPutSTR("vAX25CodingTask(one shot)!\n\r");
      //DEBUGOUT("Test\n\r");
      //DEBUGSTR("Test");
      Board_LED_Set(1, LedState);
      LedState = (bool) !LedState;
      if(trame_ax25!=NULL){
	  trame_ax25=NULL;

      }
      else{
	  /*!
	   *  Allocation d'une structure de trame et mise à zéro de tous les champs
	   *
	   * */
	  trame_ax25=calloc(1,sizeof(P_AX_25));
      }
      makeax25payload((const unsigned char*)"F4HKS",(const unsigned char*)"WIDE",(const unsigned char*)"Test",&trame_ax25);
      xStatus = xQueueSendToBack( xQueue, &trame_ax25, 0 );
      if( xStatus != pdPASS )
	{
	  /* The send operation could not complete because the queue was full -
		this must be an error as the queue should never contain more than
		one item! */
	  Board_UARTPutSTR( "Could not send to the queue.\r\n" );
	}
      /* About a 3Hz on/off toggle rate */
      //vTaskDelay(configTICK_RATE_HZ*10);

      free(trame_ax25);
      trame_ax25=NULL;
      taskYIELD();
      vTaskSuspend(NULL);
      //vTaskDelayUntil( &xLastWakeTime, ( 10000/ portTICK_RATE_MS ) );

  }

}
#endif
/* LED1 toggle thread */
static void vLEDTask1(void *pvParameters) {
  bool LedState = false;
  //portTickType xLastWakeTime;

  P_AX_25 trame;
  xLCDMessage toto1={0,0," vLEDTask1"};

  portBASE_TYPE xStatus;
  portBASE_TYPE xStatus1;
  trame=calloc(1,sizeof(P_AX_25));

  const portTickType xTicksToWait = 1000 / portTICK_RATE_MS;
  //xLastWakeTime = xTaskGetTickCount();
  for( ;; ) {

      Board_LED_Set(4, LedState);
      LedState=Board_LED_Test(1);
      LedState = (bool) !LedState;
      Board_LED_Set(1, LedState);

      //Board_UARTPutSTR("vLEDTask1\n\r");
#if 0
      xStatus1 = xQueueSend( xLCDQueue, &toto, portMAX_DELAY*2 );
      if( xStatus1 != pdPASS )
	{
	  /* The send operation could not complete because the queue was full -
					this must be an error as the queue should never contain more than
					one item! */
	  Board_UARTPutSTR( "Could not send to the queue.\r\n" );
	}

      xStatus = xQueueSend( xLCDQueue, &toto1, portMAX_DELAY );
      if( xStatus != pdPASS )
	{
	  /* The send operation could not complete because the queue was full -
										this must be an error as the queue should never contain more than
										one item! */
	  Board_UARTPutSTR( "Could not send to the queue.\r\n" );
	}
#endif
      /* About a 7Hz on/off toggle rate */
      vTaskDelay(1000/portTICK_RATE_MS);
      //vTaskDelayUntil( &xLastWakeTime, (1000/portTICK_RATE_MS ) );
      //taskYIELD();

  }
}

/* LED2 toggle thread */
static void vLEDTask2(void *pvParameters) {
  bool LedState = false;
  portTickType xLastWakeTime;
  portBASE_TYPE xStatus2;

  xLCDMessage toto2={0,0," vLEDTask2"};
  xLastWakeTime = xTaskGetTickCount();
  for( ;; ) {
      Board_LED_Set(1, LedState);
      LedState=Board_LED_Test(2);
      LedState = (bool) !LedState;
      Board_LED_Set(2, LedState);
      /* About a 7Hz on/off toggle rate */
      //Board_UARTPutSTR("vLEDTask2\n\r");
#if 0
      xStatus2 = xQueueSend( xLCDQueue, &toto2, portMAX_DELAY );
      if( xStatus2 != pdPASS )
	{
	  /* The send operation could not complete because the queue was full -
												this must be an error as the queue should never contain more than
												one item! */
	  Board_UARTPutSTR( "Could not send to the queue.\r\n" );
	}
#endif
      vTaskDelay(1000/portTICK_RATE_MS);
      //vTaskDelayUntil( &xLastWakeTime, (2000/portTICK_RATE_MS ) );

  }
}
static void vLEDTask3(void *pvParameters) {
  bool LedState = false;
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for( ;; ) {
      Board_LED_Set(2, LedState);
      LedState=Board_LED_Test(3);
      LedState = (bool) !LedState;
      Board_LED_Set(3, LedState);
      //Board_UARTPutSTR((char*)"vLEDTask3\n\r");

      /* About a 7Hz on/off toggle rate */

      //vTaskDelayUntil( &xLastWakeTime, (3000/ portTICK_RATE_MS ) );
      vTaskDelay(3000/portTICK_RATE_MS);

  }
}
static void VLEDTask4(void *pvParameters){
  bool LedState = false;
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for( ;; ) {
      Board_LED_Set(3, LedState);
      LedState=Board_LED_Test(4);
      LedState = (bool) !LedState;
      Board_LED_Set(4, LedState);
      vTaskDelayUntil( &xLastWakeTime, (4000/ portTICK_RATE_MS ) );

      /* About a 7Hz on/off toggle rate */
      vTaskDelay(1000/portTICK_RATE_MS);


  }
}
/* UART (or output) thread */
#if 0
static void vUARTTask(void *pvParameters) {
  int tickCnt = 0;

  while (1) {
      DEBUGOUT("Tick: %d\r\n", tickCnt);
      printf((char*)pvParameters);
      tickCnt++;

      /* About a 1s delay here */
      vTaskDelay(configTICK_RATE_HZ);
  }
}
#endif
/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
int main(void)
{
  //Hardware setup function
  prvSetupHardware();
  //Mise en place des files de messages
  xLCDQueue = xStartLCDTask();

  xMesureQueue = xStartTempTask();

  xCfgMeusureQueue=xConfigADC();

  //xQueue=xQueueCreate(2,sizeof(P_AX_25));
  xConfigConAX25=xStartAx25Cfg();
  xAX25pipedata=xStartAX25pipe();
  xAX25datas=xStartAX25task();
  xTaskCreate(EnvoieConfig,(const signed char *) "Envoie Config",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),(xTaskHandle *) NULL);
  /*Get sensor value*/
  xTaskCreate(xTaskGetTemperatureResults,(const signed char*)"Get results",configMINIMAL_STACK_SIZE,NULL,(tskIDLE_PRIORITY + 1UL),&TskTemp);
  /* LED1 toggle thread */
  //xTaskCreate(vLEDTask1, (const signed char *) "vTaskLed1",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),(xTaskHandle *) NULL);

  /* LED2 toggle thread */
  //xTaskCreate(vLEDTask2, (const signed char *) "vTaskLed2",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),(xTaskHandle *) NULL);


  //xTaskCreate(vLEDTask3, (const signed char *) "vTaskLed3",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),(xTaskHandle *) NULL);


  //Led state 4 task handler
  //xTaskCreate(VLEDTask4,(const signed char*)"Led state 4 task",configMINIMAL_STACK_SIZE,NULL,(tskIDLE_PRIORITY+2UL),(xTaskHandle *)NULL);
  /* UART output thread, simply counts seconds */
  //xTaskCreate(vUARTTask, (signed char *) "vTaskUart",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),(xTaskHandle *) NULL);


  //xTaskCreate(taskSendParameters,(const signed char *) "vAx25",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),&TskcfgAX25);

  xTaskCreate(taskCreateMessage,(const signed char *) "vAx25",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),&TsksendAX25);

  xTaskCreate(taskGetMessages,(const signed char *) "vAx25Get",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),&TskGetx25);
  /* Start the scheduler */
  vTaskStartScheduler();

  /* Should never arrive here */
  return 1;
}

/**
 * @}
 */
