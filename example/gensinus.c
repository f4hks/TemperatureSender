/*
 * gensinus.c
 *
 *  Created on: 30 déc. 2015
 *      Author: pierre
 *      Brief : Base function of sinewave generation this function can be called for ASK or/and FSK modulation
 *      @todo : Add gaussian filter calculation
 */
#include "FreeRTOS.h"
#include "float.h"
#include "queue.h"
#include "cmsis.h"
#include "stdbool.h"
#include "portmacro.h"
#include "portable.h"
#include "string.h"
#include "board.h"
#include "timers.h"
#include "task.h"
#include "semphr.h"
#include "math.h"
#include "dac_17xx_40xx.h"
#include "chip_lpc175x_6x.h"
#include "timer_17xx_40xx.h"
#include "clock_17xx_40xx.h"
#include "gensinus.h"
uint32_t Timer0_Clk;
uint32_t Clock;
uint16_t MaxFreqz_Sample;
uint8_t divider_Clk;
uint32_t PClk;
uint32_t frequ_qual;
portBASE_TYPE OldFreq;
volatile unsigned int CntSample;
const uint32_t sinus[128]={256,269,281,294,306,318,330,342,
    354,365,377,388,398,408,418,428,
    437,446,454,462,469,476,482,487,
    493,497,501,504,507,509,511,512,
    512,512,511,509,507,504,501,497,
    493,487,482,476,469,462,454,446,
    437,428,418,408,398,388,377,365,
    354,342,330,318,306,294,281,269,
    256,243,231,218,206,194,182,170,
    158,147,135,124,114,104,94,84,
    75,66,58,50,43,36,30,25,
    19,15,11,8,5,3,1,0,
    0,0,1,3,5,8,11,15,
    19,25,30,36,43,50,58,66,
    75,84,94,104,114,124,135,147,
    158,170,182,194,206,218,231,243};
xQueueHandle Config;
xTaskHandle TskCfgSigGen;
portBASE_TYPE Frequency;
static void vInitSigGenDAC(void);
static void vStopGenDAC(void);
static void vInitTimer0(void);
static void vDeInitTimer0(void);
static void vTskFrequencyConfig(void*pvParameters);
static void vT0on(void);
/*!
 * \fn InitGenQueue
 * \brief Création de la file de message du générateur
 */
xQueueHandle InitGenQueue(void){

  xTaskCreate(vTskFrequencyConfig,( signed portCHAR * ) "Frequency Set", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &TskCfgSigGen);
  return Config=xQueueCreate(1,sizeof(xFrequencyGen));
}
xFrequencyGen SetFreq(portBASE_TYPE Freqz){
  xFrequencyGen Internal;
  Internal.Frequency=Freqz;
  Internal.TX=true;
  return Internal;
}
void vT0on(void){
  Chip_TIMER_Enable(LPC_TIMER0);
  NVIC_EnableIRQ(TIMER0_IRQn);
}
/*!
 * \fn : vInitSigGenDAC
 * \brief : Init the digital to analog converter
 */
void vInitSigGenDAC(void)
{
  Chip_DAC_Init(LPC_DAC);
  Chip_DAC_SetBias(LPC_DAC,0);
}
/*!
 * \fn vStopGenDAC
 */
void vStopGenDAC(void)
{
  Chip_DAC_DeInit(LPC_DAC);

}
void vInitTimer0(void){
  Chip_TIMER_Init(LPC_TIMER0);
  Chip_TIMER_MatchEnableInt(LPC_TIMER0,0);
  Clock=SystemCoreClock;
  divider_Clk=Chip_Clock_GetCPUClockDiv();
  Timer0_Clk=Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_TIMER0);
  Chip_TIMER_ResetOnMatchEnable(LPC_TIMER0,0);
  //Chip_TIMER_SetMatch(LPC_TIMER0,0,800);
  Chip_TIMER_Init(LPC_TIMER0);
}
void vDeInitTimer0(void){
  Chip_TIMER_Disable(LPC_TIMER0);
  NVIC_DisableIRQ(TIMER0_IRQn);
}
/*!
 * \fn vTskFrequencyConfig
 *
 */
void vTskFrequencyConfig(void*pvParameters){
  for(;;){
      //
      portBASE_TYPE status;
      xFrequencyGen Conf;
      status=xQueueReceive(Config,&Conf,portMAX_DELAY);
      if(status!=pdTRUE){

      }
      else{
	  //StartSignalGen();
	  switch(Conf.TX){
	    case true:
	      vT0on();
	      if(Conf.Frequency!=OldFreq){
		  pVfrequencyChange(Conf.Frequency);
		  OldFreq=Conf.Frequency;

	      }
	      break;
	    case false:
	      StopSignalGen();
	      break;
	    default:
	      StopSignalGen();

	      break;


	  }
	  //Configurer la fréquence de sortie

      }
  }
}
/*!
 * \fn pVfrequencyChange
 * \brief Change frequency
 */
void pVfrequencyChange(portBASE_TYPE Freqz)
{
  portENTER_CRITICAL();
  float fPeriod=(1/(float_t)Freqz)*200000;
  int NewPero =(int)fPeriod-10;
  uint32_t FReqzNew=NewPero;
  Chip_TIMER_Disable(LPC_TIMER0);

  //Chip_TIMER_ClearMatch(LPC_TIMER0,0);
  NVIC_DisableIRQ(TIMER0_IRQn);

  //NVIC_DisableIRQ(TIMER0_IRQn);
  /*Arreter le timer 0 */
   //
  Chip_TIMER_Reset(LPC_TIMER0);
  Chip_TIMER_ResetOnMatchEnable(LPC_TIMER0,0);
  Chip_TIMER_SetMatch(LPC_TIMER0,0,(uint32_t)FReqzNew);
  NVIC_EnableIRQ(TIMER0_IRQn);
  Chip_TIMER_Enable(LPC_TIMER0);
  portEXIT_CRITICAL();
   //
}
/*!
 * \fn StartSignalGen
 * \brief Start the signal generator
 * This function will called at the start of modulating
 */
void StartSignalGen(void){
  vInitSigGenDAC();
  vInitTimer0();
  //Chip_TIMER_Enable(LPC_TIMER0);


}
/*!
 * \fn StopSignalGen
 * \brief Stop signal generator
 * This function will call at the end of modulating data
 */
void StopSignalGen(void){
  vStopGenDAC();
  vDeInitTimer0();
  //Chip_TIMER_Disable(LPC_TIMER0);
  //NVIC_DisableIRQ(TIMER0_IRQn);
}
void TIMER0_IRQHandler(void){
  //portENTER_CRITICAL();
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  NVIC_ClearPendingIRQ(TIMER0_IRQn);
  Chip_TIMER_ClearMatch(LPC_TIMER0,0);
  NVIC_DisableIRQ(TIMER0_IRQn);
  if(CntSample<=127){
      NVIC_DisableIRQ(TIMER0_IRQn);
      Chip_DAC_UpdateValue(LPC_DAC,sinus[CntSample]);
      NVIC_EnableIRQ(TIMER0_IRQn);
      CntSample++;


  }
  else{
      CntSample=0;
      NVIC_EnableIRQ(TIMER0_IRQn);

      //portEXIT_CRITICAL();
  }
portYIELD_FROM_ISR(xHigherPriorityTaskWoken);


}
