/*
 * tmpSensor.c
 *
 *  @date: 1 déc. 2015
 *  @Author: pierre
 *  @brief : This file is an implementation of use a temperature sensor , it use interruption for the conversion of signal and messages queue 
 */

#include "FreeRTOS.h"
#include "adc_17xx_40xx.h"
//

#include "queue.h"
#include "cmsis.h"
#include "stdbool.h"
#include "portmacro.h"
#include "portable.h"
#include "board.h"
#include "task.h"
#include "timers.h"
#include "tmpSensor.h"
#include "semphr.h"
//#include "lpc17xx_adc.h"

static void xTempTask(void *pvParameters );
static void xConfigTSK(void *pvParameters);
static void prVsetupADC(xTmpSensorCfg config);
static ADC_CLOCK_SETUP_T ADCSetup;
#define _LPC_ADC_IRQ ADC_IRQn
static volatile uint8_t Burst_Mode_Flag = 0, Interrupt_Continue_Flag;
static volatile uint8_t ADC_Interrupt_Done_Flag, channelTC, dmaChannelNum,CountIT;
long ExpireCounters;
xQueueHandle meusure=NULL;
xQueueHandle configADC=NULL;
xTmpSensorResult Res;
xSemaphoreHandle xSemADC = NULL;

xTaskHandle tsk=NULL;
xTaskHandle tsk_cfg=NULL;
xTimerHandle xTimer=NULL;
xQueueHandle xStartTempTask(void){

  meusure=xQueueCreate(1,sizeof(xTmpSensorResult));

  //Démarrer la tache d'acq
  xTaskCreate(xTempTask,(signed char *) "SensorTemp",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),&tsk);
  return meusure;

}
xQueueHandle xConfigADC(void){
  configADC=xQueueCreate(1,sizeof(xTmpSensorCfg));
  //vQueueAddToRegistry (configADC, (signed char*)"CFGADC" );
  xTaskCreate(xConfigTSK,(signed char *)"Sensor CFG",configMINIMAL_STACK_SIZE,NULL,(tskIDLE_PRIORITY+1UL),&tsk_cfg);

  return configADC;
}
/*!
 * \fn xConfigTSK
 *
 */
void xConfigTSK(void *pvParameters)
{
  for(;;){
      portBASE_TYPE xStatusADC;
      xTmpSensorCfg cfg_adc;
      xStatusADC=xQueueReceive( configADC, &cfg_adc,portMAX_DELAY );
      if(xStatusADC!=pdTRUE)
	{
	  Board_UARTPutSTR("Can't receive queue ! \n");
	}
      else{
	  prVsetupADC(cfg_adc);
	  vTaskSuspend(NULL);
      }


  }

  //



}
/*!
 *  \fn xTempTas
 *
 *
 */
void xTempTask(void *pvParameters ){
  vSemaphoreCreateBinary(xSemADC);
  unsigned int moyenneCnt;
  for(;;){
      if(xSemaphoreTake( xSemADC, MAX_TIME ) == pdTRUE ){//Attente de la libération du semaphoqre
	  portBASE_TYPE xStatus;
	  for(moyenneCnt=0;moyenneCnt<=10;moyenneCnt++){
	      Res.Temperature=Res.Temperature+Res.Values[moyenneCnt];

	  }
	  moyenneCnt=0;
	  Res.Temperature/=10;
	  Res.mVolts=(Res.Temperature)/DIV_REPORT;
	  Res.temp_finale=Res.mVolts/5;
	  xStatus=xQueueSend(meusure,&Res,portMAX_DELAY );
	  if( xStatus != pdPASS )
	    {
	      /* The send operation could not complete because the queue was full -
		 												this must be an error as the queue should never contain more than
		 												one item! */
	      Board_UARTPutSTR( "Fail at sensor data .\r\n" );
	    }
	  //xSemaphoreGive(xSemADC);
	  vTaskDelay(1000/portTICK_RATE_MS);


      }

  }

}
void prVsetupADC(xTmpSensorCfg config){
  for(;;){
      NVIC_EnableIRQ(ADC_IRQn);
      Chip_ADC_Init(_LPC_ADC_ID,&ADCSetup);
      Chip_ADC_SetSampleRate(_LPC_ADC_ID,&ADCSetup,(uint32_t)config.Samples);
      Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);
      Chip_ADC_EnableChannel(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);
      /* Enable burst mode if any, the AD converter does repeated conversions
	       at the rate selected by the CLKS field in burst mode automatically */
      if (Burst_Mode_Flag) {
	  //Chip_ADC_Burst_Cmd(_LPC_ADC_ID, ENABLE);
	  Chip_ADC_SetBurstCmd(_LPC_ADC_ID, ENABLE);
      }
      Interrupt_Continue_Flag = 1;
      ADC_Interrupt_Done_Flag = 1;
      while (Interrupt_Continue_Flag) {
	  if (!Burst_Mode_Flag && ADC_Interrupt_Done_Flag) {
	      ADC_Interrupt_Done_Flag = 0;
	      Chip_ADC_SetStartMode(_LPC_ADC_ID, ADC_START_NOW, ADC_TRIGGERMODE_RISING);

	  }
      }
      /* Disable burst mode if any */
      if (Burst_Mode_Flag) {
	  Chip_ADC_SetBurstCmd(_LPC_ADC_ID, DISABLE);
      }

  }
}

void ADC_IRQHandler(void){
  uint16_t dataADC;

  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  /* Interrupt mode: Call the stream interrupt handler */
  NVIC_DisableIRQ(_LPC_ADC_IRQ);
  Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID, _ADC_CHANNLE, DISABLE);
  if(CountIT<=10){
      Chip_ADC_ReadValue(_LPC_ADC_ID, _ADC_CHANNLE, &dataADC);
      Res.Values[CountIT]=dataADC;//Pour calcul
      CountIT++;
  }
  else {
      xSemaphoreGiveFromISR( xSemADC, &xHigherPriorityTaskWoken );
      CountIT=0;
  }

  ADC_Interrupt_Done_Flag = 1;

  NVIC_EnableIRQ(_LPC_ADC_IRQ);
  Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);

  Interrupt_Continue_Flag = 1;

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}


