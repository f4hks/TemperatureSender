/*!
 * @file:fsk.c
 *
 *  Created on: 13 déc. 2015
 *  @Author: pierre
 *  @brief FSK modulation
 *  This file implement the Frequency Shift Keying modulation
 *  It use a timer that update dac value
 *
 *  ^DAC VALUE
 *  |
 *  |       *                *
 *  |    *    *           *
 *  |  *        *       *
 *  |*_____________*__*______________________> Time
 *
 *
 */
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis.h"
#include "stdbool.h"
#include "portmacro.h"
#include "portable.h"
#include "string.h"
//#include "lpc17xx_libcfg.h"
#include "board.h"
#include "timers.h"
#include "task.h"
#include "semphr.h"
#include  "fsk.h"
#include "dac_17xx_40xx.h"
#include "chip_lpc175x_6x.h"
#include "timer_17xx_40xx.h"
#include "clock_17xx_40xx.h"
ConfigPWM PwMcFg;
TimerStat StatusTMR;
unsigned int bitConverted=0;
uint32_t Timer0Clk;
uint32_t Clk;
uint16_t MaxFreqzSample;
uint8_t dividerClk;
uint8_t pclk;
uint32_t freququal;
/*Mapping de la FSK */
const unsigned portCHAR FSKMAP[8]={0x00,0x01,0x03,0x02,0x06,0x07,0x05,0x04};
const unsigned portCHAR SHIFTS[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
const uint32_t sineForFSK[128]={256,269,281,294,306,318,330,342,
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
xQueueHandle DataStream;
xQueueHandle ConfigFSK;
FSk_param FskSetup;
DigitalData message;
portBASE_TYPE xStatusFSKEncoder;
portBASE_TYPE xStatusFskCfg;
xSemaphoreHandle xSemFskEncoder;
xTaskHandle tskFSKconv;
xTaskHandle tskFSKconf;
Timer1Set Tmroptions;
static void vInitDac(void);
//static void vKillDac(void);
static void vInitTMR(void);
//static void vKillTMR(void);
static void vTaskEncodeDataStream(void *pvParameters);
static void vTskCfgFskMod(void *pVparameters);
static void vChangeFrequency(unsigned long uFrequency);
static void InitBaudRateTMR(unsigned long BaudRate);
static void StopModem(void);
static void StartModem(void);
static void vStartFSK(void);
static void vStopFSK(void);

//static void vCalculateSinus(void);
xQueueHandle vStartQueueFSK(void){
  DataStream=xQueueCreate(1,sizeof(DigitalData));
  xTaskCreate(vTaskEncodeDataStream,( signed portCHAR * ) "FSK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &tskFSKconv);
  return DataStream;
}
xQueueHandle vStartConfigFSK(void){
  ConfigFSK=xQueueCreate(1,sizeof(FSk_param));
  xTaskCreate(vTskCfgFskMod,( signed portCHAR * ) "FSK_cfg", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &tskFSKconf);
  return ConfigFSK;
}
void vTskCfgFskMod(void *pVparameters){
  for(;;){
      xStatusFskCfg=xQueueReceive(ConfigFSK,&FskSetup,portMAX_DELAY);
      if(xStatusFskCfg!=pdTRUE){
	  Board_UARTPutSTR("Error on receive FSK config\n\r");
      }
      else{
	  vInitTMR();

	  InitBaudRateTMR(FskSetup.debit);
	  vTaskSuspend(NULL);
      }
      vTaskDelay(100/portTICK_RATE_MS);
  }
}
/*!
 * \fn vChangeFrequency
 * \param uFrequency : the frequency we want
 * Changing the period of sampling and change frequency output for this we get the system clock value
 */
void vChangeFrequency(unsigned long uFrequency)
{
  NVIC_DisableIRQ(TIMER0_IRQn);
  MaxFreqzSample=128*uFrequency;
  pclk = Clk/dividerClk;
  freququal=pclk/MaxFreqzSample;
  //Chip_RIT_SetCOMPVAL(LPC_RITIMER,(freququal-1));

  Chip_TIMER_SetMatch(LPC_TIMER0,0,freququal);
  NVIC_EnableIRQ(TIMER0_IRQn);


}
void vInitDac(void){
  Chip_DAC_Init(LPC_DAC);


}
#if 0
void vKillDac(void){
  Chip_DAC_DeInit(LPC_DAC);
}
#endif
void vInitTMR(void){
  //Chip_RIT_Init(LPC_RITIMER);
  //Initialisation du timer de la sinusoidale
  Chip_TIMER_Init(LPC_TIMER0);
   Chip_TIMER_MatchEnableInt(LPC_TIMER0,0);
  Clk=SystemCoreClock;
  dividerClk=Chip_Clock_GetCPUClockDiv();
  Timer0Clk=Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_TIMER0);
  Chip_TIMER_ResetOnMatchEnable(LPC_TIMER0,0);
  Chip_TIMER_Init(LPC_TIMER0);

}

void InitBaudRateTMR(unsigned long BaudRate){
  Chip_TIMER_Init(LPC_TIMER1);
  uint32_t Clk_tmr1=Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_TIMER1);
  Chip_TIMER_Reset(LPC_TIMER1);
  Chip_TIMER_MatchEnableInt(LPC_TIMER1,1);
  Chip_TIMER_SetMatch(LPC_TIMER1,1,((Clk_tmr1/BaudRate)/(FskSetup.MFSK/2)));
  Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1,1);
  //Chip_TIMER_Enable(LPC_TIMER1);
}
void vStartFSK(void){
  NVIC_EnableIRQ(TIMER0_IRQn);
  Chip_TIMER_Enable(LPC_TIMER0);


}
void vStopFSK(void){
  Chip_TIMER_Disable(LPC_TIMER0);
  Chip_DAC_DeInit(LPC_DAC);
  NVIC_DisableIRQ(TIMER0_IRQn);
}
void StartModem(void){
  NVIC_EnableIRQ(TIMER1_IRQn);
  Chip_TIMER_Enable(LPC_TIMER1);
}
void StopModem(void){
  Chip_TIMER_Disable(LPC_TIMER1);
  NVIC_DisableIRQ(TIMER1_IRQn);
}
#if 0
void vKillTMR(void){
  Chip_RIT_DeInit(LPC_RITIMER);
}
#endif
void TIMER0_IRQHandler(void){
  //Acquitement IT
  NVIC_ClearPendingIRQ(TIMER0_IRQn);
  //Desactivation IT timer
  NVIC_DisableIRQ(TIMER0_IRQn);
  //Test du nombre d'échantillons passés
  if(StatusTMR.TimerHitsCounts<MAXSAMPLE){
      StatusTMR.TimerHitsCounts=+1;
  }
  else{
      StatusTMR.TimerHitsCounts=0;
  }
  //Update DAC
  Chip_DAC_UpdateValue(LPC_DAC,(uint32_t)sineForFSK[StatusTMR.TimerHitsCounts]);
  //Remise en marche de l'it sur TMR
  NVIC_EnableIRQ(TIMER0_IRQn);

}
/*!
 * \fn TIMER1_IRQHandler
 * \brief This interrupt change the frequency of DAC every (1/Tb)/(M/2) seconds
 */
void TIMER1_IRQHandler(void){
  static signed portBASE_TYPE xHigherPriorityTaskWoken;

  NVIC_ClearPendingIRQ(TIMER1_IRQn);
  NVIC_DisableIRQ(TIMER1_IRQn);
  if(StatusTMR.TimerHitsCounts==127){
      if(xSemFskEncoder!=NULL){
	  //Tester le nombre de répétitions
	  if(bitConverted<7){
	      vChangeFrequency(FskSetup.Frequencies[Tmroptions.Results[bitConverted]]);
	      bitConverted++;
	  }
	  else{
	      //Libere le sémaphore on peut passer au caractère suivant
	      xSemaphoreGiveFromISR(xSemFskEncoder,&xHigherPriorityTaskWoken);

	  }


      }
  }
  else{
      //StatusTMR.TimerHitsCounts=StatusTMR.TimerHitsCounts+1;
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}
void vTaskEncodeDataStream(void *pvParameters){

  xSemFskEncoder=xQueueCreateCountingSemaphore(1,0);
  for(;;){

      xStatusFSKEncoder=xQueueReceive(DataStream,&message,portMAX_DELAY);
      if(xStatusFSKEncoder!=pdTRUE){
	  //Erreur
      }
      else{


	  //Récupération de la taille de la trame
	  unsigned long size_input=strlen((portCHAR*)message.Message);
	  unsigned long countParcours;
	  unsigned int M_also_Cnt;
	  unsigned int Shift=FskSetup.MFSK/2;
	  unsigned int maskDec=0;
	  unsigned int M_val=0;
	  unsigned int MaskTst=FskSetup.MFSK-1;
	  //Mise en marche du traitement (prévoir timer hardware + sémaphore)
	  vInitDac();
	  vInitTMR();


	  while(countParcours<size_input){
	      //Test par blocs en fonction de M
	      if(xSemaphoreTake(xSemFskEncoder,0xffff)==pdTRUE){
		  while(M_also_Cnt<(FskSetup.MFSK-1)){
		      //Test de chaque partie
		      while((message.Message[countParcours]&(MaskTst<<maskDec))!=FSKMAP[M_val]){
			  //vChangeFrequency(FskSetup.Frequencies[M_also_Cnt]);
			  //Suspendre la tache attendre que les 128 echs sont passé
			  //Rajouter les résultats dans un tableau


		      }
		      Tmroptions.Results[M_also_Cnt]=FSKMAP[M_also_Cnt];
		      M_also_Cnt=M_also_Cnt+Shift;
		      maskDec=maskDec+Shift;
		      Tmroptions.Results[M_also_Cnt]=FSKMAP[M_also_Cnt];
		      if(countParcours==0){
			  vStartFSK();
			  StartModem();
		      }

		  }
		  //Prise du Sémaphore
		  //xSemaphoreTake(xSemFskEncoder,(portTICK_RATE_MS/1.2));
		  //vTaskSuspend(NULL);//Suspendre la tache
		  countParcours=countParcours+1;
	      }
	      //vKillDac();
	      //vKillTMR();

	  }
	  StopModem();
	  vStopFSK();
	  vTaskDelay(3000/portTICK_RATE_MS);

      }


  }

}
