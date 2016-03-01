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
#include "semphr.h"
#include "task.h"
#include "cmsis.h"
#include "timers.h"
#include "queue.h"
#include "ax25.h"
#include "string.h"
#include "tmpSensor.h"
#include "trcKernelPort.h"
#include "fsk.h"
#include "HD44780.h"
#include "gensinus.h"
#include <stdlib.h>
P_AX_25 trame_ax25;
AX_25_CFG cfgdatas;
AX_25_SEND_DATA datasenvoye;
AX_25 tramegenere;
AX_25_CFG ConfigurationAX25;
AX_25_SEND_DATA donnes;
tStatusModulator statMOdulator;
DigitalData DatasToModulate;
FSk_param ConfigFsk;
xQueueHandle xQueue=NULL;
xQueueHandle TransfertFSK;
xQueueHandle xLCDQueue=NULL;
xQueueHandle xMesureQueue=NULL;
xQueueHandle xCfgMeusureQueue=NULL;
xQueueHandle xConfigConAX25=NULL;
xQueueHandle xAX25pipedata=NULL;
xQueueHandle xAX25datas=NULL;
xQueueHandle xFskDatas=NULL;
xQueueHandle xFskModulatorStatus=NULL;
xQueueHandle xFskCfgQueue=NULL;
xQueueHandle xSinegne;
xTaskHandle TskcfgAX25=NULL;
xTaskHandle TskTemp=NULL;
xTaskHandle TsksendAX25=NULL;
xTaskHandle TskGetx25=NULL;
xTaskHandle TskSendToFsk=NULL;
xTaskHandle TskStatusFSK;
xTaskHandle TskSineGen1k;
xTaskHandle TskSineGen2k;
xTimerHandle SendDataTaskTimer[2];
xTimerHandle TimerAx25Delay;
portBASE_TYPE TimerFini[2];
portBASE_TYPE  xStatusADCCFG=0;
portBASE_TYPE xStatusADCRESULT=0;
portBASE_TYPE xStatusAX25CodingTask=0;
portBASE_TYPE xStatusAX25sendTask=0;
portBASE_TYPE xStatusAX25GetData=0;

portBASE_TYPE xStatusFSKsendTask=0;
portBASE_TYPE xStatusFSKCtrl=0;
portBASE_TYPE xStatusFSkCgfgSend=pdFALSE;
portBASE_TYPE xStatusModemCtrl=0;
xLCDMessage tempe;
xLCDMessage toto= { 0, 0, "LCD TEST" };
xSemaphoreHandle SemAx25Wait;
xSemaphoreHandle SemAX25cfgLock;//Utiliser pour savoir si la config s'est bien passé
xSemaphoreHandle SemFSKWait;
xSemaphoreHandle Sem;//Semaphore global
bool FSkDone=false;
bool AX25Done=false;
struct AMessage
{
  char ucMessageID;
  char ucData[ 20 ];
};
#define DATA_BAUD_RATE 1200
#define DATA_BAUDRATEMINFREQ 1200
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
xTimerHandle MonTimer;
/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

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

      xTmpSensorCfg cfgMe={44,false};
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
  //portBASE_TYPE xStatus1;
  //portBASE_TYPE xStatus2;

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
	      //Board_UARTPutSTR("Value on ADC : ");
	      snprintf(buf,sizeof(buf),"%d",Result.Temperature);
	      snprintf(Celcius,sizeof(Celcius),"%f",Result.temp_finale);
	      Board_UARTPutSTR((char*)buf);
	      Board_UARTPutSTR("\n\r");
	      Board_UARTPutSTR("Temperature :");
	      Board_UARTPutSTR((char*)Celcius);
	      Board_UARTPutSTR(" C\n\r ");
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

	  vTaskDelay(1000/portTICK_RATE_MS);
	}

  }


}

static void taskSendParameters(void *pVparam)
{
  for(;;){
      //xSemaphoreGive(SemAX25cfgLock);
      //vTaskSuspend(TsksendAX25);
      if(SemAX25cfgLock!=NULL){
	  //
	  if(xSemaphoreTake(SemAX25cfgLock,portTICK_RATE_MS*10)==pdTRUE)
	    {
	      cfgdatas=MakeConfig((const unsigned char*)"F4HKS",1);
	      portBASE_TYPE xStatusCFGSEND;
	      //xSemaphoreGive(SemAX25cfgLock);
	      xStatusCFGSEND=xQueueSend(xConfigConAX25,&cfgdatas,portTICK_RATE_MS*200);
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
	      xSemaphoreGive(SemAX25cfgLock);
	      //vTaskResume(TsksendAX25);
	      vTaskSuspend(NULL);
	      //vTaskDelay(30*portTICK_RATE_MS);
	    }
      }

  }
}

static void taskCreateMessage(void *pVparam)
{
  for(;;){
      //Connaitre l'état du sémaphore de la couche ax25
      if(SemAx25Wait!=NULL){
	  xSemaphoreGive(SemAx25Wait);
	  if(xSemaphoreTake(SemAx25Wait,portTICK_RATE_MS*10)==pdTRUE){
	      ConfigurationAX25=MakeConfig((const unsigned char*)"F4HKS",1);
	      portBASE_TYPE xStatusCFGSEND;
	      xStatusCFGSEND=xQueueSend(xConfigConAX25,&ConfigurationAX25,portTICK_RATE_MS*200);
	      if(xStatusCFGSEND!=pdTRUE){
		  //vTaskSuspend(TskTemp);
		  Board_UARTPutSTR("Fail on taskSendParameters !\n\r");
		  //vTaskResume(TskTemp);

	      }

	      datasenvoye=MakeFrame((const unsigned portCHAR*)"The quick brown fox jumps over the lazy dog ",(const unsigned portCHAR*)"WIDE");
	      xStatusAX25CodingTask=xQueueSend(xAX25datas,&datasenvoye,portTICK_RATE_MS*200);
	      if(xStatusAX25CodingTask!=pdPASS){
		  Board_UARTPutSTR("Erreur !\n\r");
	      }
	      xSemaphoreGive(SemAx25Wait);//Rends le semaphore
	  }
      }
  }

}



static void taskGetMessages(void *pVparam){
  for(;;){

      if(SemAx25Wait!=NULL){

	  if(xSemaphoreTake(SemAx25Wait,portTICK_RATE_MS*100)==pdTRUE){

	      xStatusAX25GetData=xQueueReceive(xAX25pipedata,&tramegenere,portTICK_RATE_MS*200);
	      if(xStatusAX25GetData!=pdTRUE){
		  Board_UARTPutSTR("Fail on get ax 25 \n\r");
	      }
	      else{
		  if(strlen((char*)tramegenere.fullmessage)!=0)
		    {
		      Board_UARTPutSTR("Trame : ");
		      Board_UARTPutSTR((char*)tramegenere.fullmessage);
		      Board_UARTPutSTR("\r\r");
		      AX25Done=true;
		      //xSemaphoreGive(SemAx25Wait);

		    }

	      }

	      portBASE_TYPE stat=xQueueSend(TransfertFSK,&tramegenere.fullmessage,portTICK_RATE_MS*200);
	      if(stat!=pdTRUE){
		  Board_UARTPutSTR("Fail on get ax 25 \n\r");
	      }



	  }

	  //xSemaphoreGive(SemAx25Wait);
	  //vTaskDelay(3000*portTICK_RATE_MS);
      }





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
#if 0
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
#endif
/*!
 * \fn vTaskSendToFsk
 * \brief Envoi de la trame vers le modulateur FSK
 */
static void vTaskSendToFsk(void *pVparameters)
{
  unsigned portCHAR* test=0x00;
  for(;;){
      if(SemAx25Wait!=NULL)
	{
	  //On se synchronise avec le semaphore de la couche X25
	  if(xSemaphoreTake(GetSemStatusX25Send(),portTICK_RATE_MS*400)==pdTRUE){
	      portBASE_TYPE StatuFSK=0;
	      test=0x00;
	      if(AX25Done==true){
		  ConfigFsk =InitConfigurationFSK(TWO_FSK,DATA_BAUD_RATE,DATA_BAUDRATEMINFREQ,(long)1000);
		  xStatusFSkCgfgSend=xQueueSend(xFskCfgQueue,&ConfigFsk,portTICK_RATE_MS*1000);
		  if(xStatusFSkCgfgSend!=pdTRUE){
		      Board_UARTPutSTR("Error in FSK cfg ...\n\r");
		  }
		  else{

		      Board_UARTPutSTR("Transmit in progress ...\n\r");

		  }
		  StatuFSK=xQueueReceive(TransfertFSK,&test,portTICK_RATE_MS*200);
		  if(StatuFSK!=pdTRUE){
		      Board_UARTPutSTR("Fail on get ax 25 \n\r");
		  }
		  DatasToModulate=InitDigitalData(test);
		  xStatusFSKsendTask=xQueueSend(xFskDatas,&DatasToModulate,portTICK_RATE_MS*200);
		  if(xStatusFSKsendTask!=pdTRUE){
		      Board_UARTPutSTR("Error in FSK transmit ...\n\r");
		  }
		  else{

		      Board_UARTPutSTR("Transmit in progress ...\n\r");

		  }

		  FSkDone=true;
	      }
	  }
	  else
	    {
	      Board_UARTPutSTR("X25 task is not finished we can't now encode ! \n\r");
	    }


	}
      vTaskDelay(3000*portTICK_RATE_MS);
  }
}
/*!
 * \fn StatusFSKGet
 * \brief A task that permit to know the status of the transmission for the other tasks
 */
static void taskStatusFSKGet(void *Pvparam){
  for(;;){
      xStatusModemCtrl=xQueueReceive(xFskModulatorStatus,&statMOdulator,portTICK_RATE_MS*300);
      if(xStatusModemCtrl!=pdFALSE){
	  if(statMOdulator.AllTransmit==true){
	      //Unblock the semaphore
	      xSemaphoreGive(SemAx25Wait);
	  }
      }
  }
}

//}
#if 0
static void Singene1k(void *pVparameters){
  for(;;){
      //vTaskDelay(3000/portTICK_RATE_MS);

      xFrequencyGen FreqParam;
      FreqParam.Frequency=1000;
      FreqParam.TX=true;
      portBASE_TYPE xStat=xQueueSend(xSinegne,&FreqParam,portMAX_DELAY);
      if(xStat!=pdTRUE){
	  Board_UARTPutSTR("Error\n\r");
      }
      else{
	  //StartSignalGen();
	  vTaskSuspend(NULL);
      }
      //
      //vTaskDelay(3000/portTICK_RATE_MS);
      //
      //StopSignalGen();

  }
}

static void Singene2k(void *pVparameters){
  for(;;){
      vTaskDelay(3000/portTICK_RATE_MS);
      xFrequencyGen FreqParam;
      FreqParam.Frequency=1200;
      FreqParam.TX=true;
      portBASE_TYPE xStat=xQueueSend(xSinegne,&FreqParam,portMAX_DELAY);
      if(xStat!=pdTRUE){
	  Board_UARTPutSTR("Error\n\r");
      }
      else{
	  //StartSignalGen();
	  //vTaskSuspend(NULL);
      }

      //
      //StopSignalGen();
      //vTaskSuspend(NULL);

  }

}
#endif
void vTimerCallback1(xTimerHandle Timer){
  portBASE_TYPE indexTimer;
  const portBASE_TYPE comptageMAx=60;
  configASSERT(Timer);
  indexTimer=(portBASE_TYPE)pvTimerGetTimerID(Timer);
  TimerFini[indexTimer]+=1;
  if(TimerFini[indexTimer]==comptageMAx){
      xTimerStop(Timer,0);
      xSemaphoreGive(SemAx25Wait);
  }
}
void vTimerCallback2(xTimerHandle Timer){
  portBASE_TYPE indexTimer;
  const portBASE_TYPE comptageMAx=30;
  configASSERT(Timer);
  indexTimer=(portBASE_TYPE)pvTimerGetTimerID(Timer);
  TimerFini[indexTimer]+=1;
  if(TimerFini[indexTimer]==comptageMAx){
      xSemaphoreGive(SemAx25Wait);
      xTimerStop(Timer,0);
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
  Sem=xSemaphoreCreateMutex();
  SemAX25cfgLock=xSemLockAx25Cfg();
  SemAx25Wait=xBeaconTime(30000);//Pour attendre la fin de l'encodageX25
  //Mise en place des files de messages
  xLCDQueue = xStartLCDTask();

  xMesureQueue = xStartTempTask();
  xFskCfgQueue=xStartConfigFSK();
  xCfgMeusureQueue=xConfigADC();
  //Création du sémaphore

  SemFSKWait=xSemaphoreCreateMutex();
  //xQueue=xQueueCreate(2,sizeof(P_AX_25));
  ConfigFsk=InitConfigurationFSK(2,1200,1200,1000);
  xConfigConAX25=xStartAx25Cfg(&SemAX25cfgLock);
  xAX25pipedata=xStartAX25pipe();
  xAX25datas=xStartAX25task();
  //xSinegne=InitGenQueue();
  xFskDatas=xStartQueueFSK();
  xFskModulatorStatus=xStartStatusQueueFSK();
  TimerAx25Delay=CreateAX25Timer(30000);
  StartX25Wait(TimerAx25Delay);
  TransfertFSK=xQueueCreate(1,sizeof(unsigned portCHAR*));
  //uint32_t Clk=SystemCoreClock;
  //uint32_t dividerClk=Chip_Clock_GetCPUClockDiv();

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


  xTaskCreate(taskSendParameters,(const signed char *) "vAx25",32, NULL, (tskIDLE_PRIORITY + 1UL),&TskcfgAX25);

  xTaskCreate(taskCreateMessage,(const signed char *) "vAx25",2048, NULL, (tskIDLE_PRIORITY + 1UL),&TsksendAX25);
  xTaskCreate(taskGetMessages,(const signed char *) "vAx25Get",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),&TskGetx25);
  xTaskCreate(taskStatusFSKGet,(const signed portCHAR*)"TaskGETFSKStatus",32, NULL, (tskIDLE_PRIORITY + 1UL),&TskStatusFSK);
  xTaskCreate(vTaskSendToFsk,(const signed char *)"VtaskSendFSK",256,NULL,(tskIDLE_PRIORITY+1UL),&TskSendToFsk);
  //xTaskCreate(Singene1k,(const signed char *) "sine1k",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),&TskSineGen1k);
  //xTaskCreate(Singene2k,(const signed char *) "sine2k",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),&TskSineGen2k);
  /*Creating timer */

  /* Start the scheduler */
  vTaskStartScheduler();

  /* Should never arrive here */
  return 1;
}

/**
 * @}
 */
