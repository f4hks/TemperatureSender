/*!
 * @file:fsk.c
 *
 *  Created on: 13 déc. 2015
 *  @Author: pierre
 *  @brief FSK modulation
 *  This file implement the Frequency Shift Keying modulation
 *
 *  ^DAC VALUE
 *  |
 *  |       *                *
 *  |    *    *           *
 *  |  *        *       *
 *  |*_____________*__*______________________> Time
 *  This library must be use with functions that located in gensinus.c file
 *  For the moment only M-FSK schemes are supported
 *
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
#include  "fsk.h"
#include "gensinus.h"
#include "dac_17xx_40xx.h"
#include "chip_lpc175x_6x.h"
#include "timer_17xx_40xx.h"
#include "clock_17xx_40xx.h"
ConfigPWM PwMcFg;
TimerStat StatusTMR;
tStatusModulator FskCtrl;
unsigned int bitConverted=0;
volatile unsigned int Bitwasconverted=0;
volatile unsigned long countParcours=0;
volatile unsigned int digitalToFsk=0;
volatile unsigned int MaxBitsVal;
volatile unsigned int InterruptCnt;
uint32_t Timer0Clk;
uint32_t Clk;
uint16_t MaxFreqzSample;
uint8_t dividerClk;
uint8_t pclk;
uint32_t freququal;
/*Mapping de la FSK */
const unsigned portCHAR FSKMAP[8]={0x00,0x01,0x03,0x02,0x06,0x07,0x05,0x04};
const unsigned portCHAR SHIFTS[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
volatile unsigned portCHAR res[16];
xQueueHandle DataStream;
xQueueHandle ConfigFSK;
xQueueHandle StatusModulator;
xQueueHandle ConfigGenerator;
FSk_param FskSetup;
//The parameters for sigen
xFrequencyGen FreqGenSetup;
DigitalData message;
portBASE_TYPE xStatusFSKEncoder;
portBASE_TYPE xStatusFskCfg;
portBASE_TYPE xStatusModemCtrl;
portBASE_TYPE xStatusSendToSigGen;
portBASE_TYPE xStatsFskCtrl;
xSemaphoreHandle xSemFskEncoder;
xSemaphoreHandle xTimerLock;
xTaskHandle tskFSKconv;
xTaskHandle tskFSKconf;
xTaskHandle tskFSKstats;
Timer1Set Tmroptions;
//static void vInitDac(void);
//static void vKillDac(void);
//static void vInitTMR(void);
//static void vKillTMR(void);
static void vTaskEncodeDataStream(void *pvParameters);
static void vTskCfgFskMod(void *pVparameters);
static void vTskSendFskStatus(void *pVparameters);
#if 0
static void vChangeFrequency(unsigned long uFrequency);
#endif
static void InitBaudRateTMR(unsigned long BaudRate);
static void StopModem(void);
static void StartModem(void);
static void StartT1(void);
#if 0
static void vStartFSK(void);
static void vStopFSK(void);
#endif
Bool AllowContinue=false;
//Fonction a faire a chaque fois
//Les donnés viennent de la trame AX25 ou autre
DigitalData InitDigitalData(unsigned char *DataPtr){
  DigitalData Intern;
  Intern.Message=DataPtr;
  return Intern;
}
FSk_param InitConfigurationFSK(portLONG M,portLONG baudRate,portLONG MinFReq,portLONG DeltaF){
  FSk_param Intern;
  unsigned int Compte=0;
  portLONG df=DeltaF;
  Intern.MFSK=M;
  Intern.debit=baudRate;
  for(Compte=0;Compte<=(M/2);Compte++){
      Intern.FreqUency[Compte]=MinFReq+(df*Compte);
  }
  return Intern;
}

void SetFrequence(FSk_param *param,unsigned int num,unsigned freq){
  param->Frequencies[num]=freq;
}
//static void vCalculateSinus(void);
xQueueHandle xStartQueueFSK(void){
  DataStream=xQueueCreate(1,sizeof(DigitalData));
  xTimerLock=xSemaphoreCreateMutex();//Creation of a mutex that lock by encoding function and unlocked by Timer
  //ConfigGenerator=InitGenQueue();
  xSemFskEncoder=xQueueCreateCountingSemaphore(1,0);
  xTaskCreate(vTaskEncodeDataStream,( const signed portCHAR * ) "FSK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &tskFSKconv);
  return DataStream;
}
xQueueHandle xStartConfigFSK(void){
  ConfigFSK=xQueueCreate(1,sizeof(FSk_param));
  xTaskCreate(vTskCfgFskMod,( const signed portCHAR * ) "FSK_cfg", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &tskFSKconf);
  return ConfigFSK;
}
xQueueHandle xStartStatusQueueFSK(void){
  StatusModulator=xQueueCreate(1,sizeof(tStatusModulator));
  return StatusModulator;
}
void vTskCfgFskMod(void *pVparameters){
  for(;;){

      xStatusFskCfg=xQueueReceive(ConfigFSK,&FskSetup,portTICK_RATE_MS*200);
      if(xStatusFskCfg!=pdTRUE){

      }
      else{
	  //vInitTMR();

	  InitBaudRateTMR(FskSetup.debit);
	  vTaskSuspend(NULL);

      }
      //vTaskDelay(100/portTICK_RATE_MS);


  }
}

/*!
 * \fn InitBaudRateTMR
 *  \brief Initialize timer 1
 */
void InitBaudRateTMR(unsigned long BaudRate){
  float fPeriodBaudrate=(1/(float)BaudRate)*80000*128;
  int BaudRateCalc =(int)fPeriodBaudrate;
  Chip_TIMER_Init(LPC_TIMER1);
  //uint32_t Clk_tmr1=Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_TIMER1);
  uint32_t Calculation=((uint32_t)BaudRateCalc);
  Chip_TIMER_Reset(LPC_TIMER1);
  Chip_TIMER_MatchEnableInt(LPC_TIMER1,1);
  Chip_TIMER_SetMatch(LPC_TIMER1,1,(Calculation));
  Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1,1);
}
void StartModem(void){
  //StopSignalGen();
  NVIC_EnableIRQ(TIMER1_IRQn);
  Chip_TIMER_Enable(LPC_TIMER1);
  //StartSignalGen();
}
void StartT1(void){
  NVIC_EnableIRQ(TIMER1_IRQn);
  Chip_TIMER_Enable(LPC_TIMER1);
}
void StopModem(void){
  Chip_TIMER_Disable(LPC_TIMER1);
  NVIC_DisableIRQ(TIMER1_IRQn);
  StopSignalGen();
  //StartSignalGen();
  //pVfrequencyChange(1000);//1000Hz (pour test);
}
/*!
 * \fn vTaskEncodeDataStream
 * \brief FSK modulation of a binary stream
 *This function launches a timer with a period of 1/bitime
 * At every interrupt ,we shift the mask by M/2 (for 2FSK it's by one)
 * and test the result to determine that the value is 0,1 00 ,01 or etc ..
 * after we change the frequency output (wait for the sine to finish) and store the value to avoid unesessary changes
 */
void vTaskEncodeDataStream(void *pvParameters){

  unsigned int result=0;
  xTaskCreate(vTskSendFskStatus,( const signed portCHAR * )"FSK_Stat",configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1,&tskFSKstats);
  for(;;){
      //xStatusFskCfg=xQueueReceive(ConfigFSK,&FskSetup,portMAX_DELAY);
      //while(xStatusFskCfg!=pdTRUE);
      //vTaskDelay(1000*portTICK_RATE_MS);
      InitBaudRateTMR(1200);
      xStatusFSKEncoder=xQueueReceive(DataStream,&message,portTICK_RATE_MS*200);
      //Récupération de la taille de la trame
      unsigned long size_input=strlen((portCHAR*)message.Message);
      unsigned int MaskTst=(unsigned int)FskSetup.MFSK-1;
      unsigned int MaskVariable=0;
      unsigned int Car;
      unsigned int TesterMask=1;
      long Shift=FskSetup.MFSK>>1;
      MaxBitsVal=(MAX_VAL+1)/Shift;
      //Mise en marche du traitement (prévoir timer hardware + sémaphore)
      countParcours=0;
      //We convert all the frame
      if(size_input!=0){
	  while(countParcours<size_input){
	      //On teste chaque bit tout les 1/Tb

	      Car=(unsigned int)message.Message[countParcours];
	      while(Bitwasconverted<MaxBitsVal-1){
		  MaskVariable=(Bitwasconverted+(Shift-1))*Bitwasconverted;
		  TesterMask=MaskTst<<Bitwasconverted;
		  result=Car&TesterMask;
		  result=result>>Bitwasconverted;

		  Tmroptions.Results[Bitwasconverted]=result;

		  Bitwasconverted++;
	      }
	      if(countParcours==0){
		  StartModem();
		  StartSignalGen();
		  AllowContinue=true;
		  FreqGenSetup.TX=true;
	      }

	      while(digitalToFsk<(((MAX_VAL+1)/(FskSetup.MFSK/2)))-1){
		  //The flag will unblock with timer 1 interrupt
		  //We use a semaphore for doing this
		  //StartModem();
		  if(xSemaphoreTake(xTimerLock,0)!=pdTRUE){
		      if(AllowContinue==true){
			  FreqGenSetup.Frequency=FskSetup.FreqUency[Tmroptions.Results[bitConverted]];
			  AllowContinue=false;
			  digitalToFsk++;
			  StopSignalGen();
			  pVfrequencyChange(FreqGenSetup.Frequency);
			  StartSignalGen();
			  Chip_TIMER_Enable(LPC_TIMER1);
			  NVIC_EnableIRQ(TIMER1_IRQn);
		      }
		      else{
			  Chip_TIMER_Enable(LPC_TIMER1);
			  NVIC_EnableIRQ(TIMER1_IRQn);
		      }


		  }

	      }
	      digitalToFsk=0;
	      NVIC_EnableIRQ(TIMER1_IRQn);
	      countParcours++;
	      Bitwasconverted=0;

	  }
	  FskCtrl.AllTransmit=true;
	  countParcours=0;
	  //bitConverted=0;
	  StopModem();
	  StopSignalGen();
      }
  }

}
/*!
 * \fn vTskSendFskStatus
 *  \brief Send the task status to another part of the program
 */
void vTskSendFskStatus(void *pVparameters){
  for(;;){
      xStatsFskCtrl=xQueueSend(StatusModulator,&FskCtrl,portMAX_DELAY);
      if(xStatsFskCtrl!=pdTRUE){

      }
  }
}
/*!
 * \fn TIMER1_IRQHandler
 * \brief This interrupt change the frequency of the output once every (1/Bt)/(M/2) seconds
 */
void TIMER1_IRQHandler(void){
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  //Clear timer 1 interrupt
  NVIC_ClearPendingIRQ(TIMER1_IRQn);
  Chip_TIMER_ClearMatch(LPC_TIMER1,1);
  NVIC_DisableIRQ(TIMER1_IRQn);
  //Tester le nombre de répétitions jusqu'a 7 si 2 FSK
  if(bitConverted<(((MAX_VAL+1)/(FskSetup.MFSK/2)))-1){
      bitConverted++;//Incrémentation du compteur de conversion
      AllowContinue=true;
  }
  else
    {
      //Chip_TIMER_Enable(LPC_TIMER1);
      //NVIC_EnableIRQ(TIMER1_IRQn);

      bitConverted=0;
      AllowContinue=false;
    }
  xSemaphoreGiveFromISR(xTimerLock,&xHigherPriorityTaskWoken);//Release the Semaphore
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  //portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

