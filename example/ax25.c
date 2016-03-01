/*!
 * 	@file ax25.c
 *  @date 31/09/2015
 *  @author: pierre jourdain (F4HKS)
 *  @brief AX25 basic stack
 *  @brief This file implement the HDLC protocol that used by AX25 it does not implement the AFSK modulation
 *  @license LGPL
 *  These functions in this file are individual functions that are been launched in freertos tasks and use message flow
 *  for passing information beetween tasks
 *  @todo : Add calculation of the crc with hardware peripherial(more speed)
 */
#include "FreeRTOS.h"
#include <string.h>
#include <stdlib.h>
#include <crc.h>
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "ax25.h"
#define SizeQueue  1
xQueueHandle xAX25Queue;
xTaskHandle Ax25CFG;
xTaskHandle Ax25Encode;
xTaskHandle Ax25Send;
xQueueHandle xAX25Cfg;
xQueueHandle xAX25pipe;
xTimerHandle TimerAx25;
/*This semapahore are necessary to execute tasks in this order Configuration -> encoding-> Sendto main task*/
xSemaphoreHandle Stage1;//Configuration
xSemaphoreHandle Stage2;//Encoding
xSemaphoreHandle Stage3;//Send to main task
xSemaphoreHandle Sems;
xSemaphoreHandle SemAllowAx25;
xTimerHandle T_ax25;
AX_25_CFG cfg_ax25;//Contain the configuration
AX_25 trame;//This variable contains all the part of the data that will want to transmit
AX_25_SEND_DATA datas;

P_AX_25 *pAX25trame;
static void vAX25taskBase(void *pvParameters);
static void vAX25cfg(void *param);
//static void vAX25send(void *pvParameters);
static void Nrzi(AX_25 *frame);
static void InitTxInsertion(AX_25 *frame,unsigned int TXwaitReapeat,unsigned int *PosiInit);
static void AddFlags(AX_25 *frame,unsigned int FlagNumber,unsigned int *PosiInit);
static void AddAdresses(AX_25 *frame,AX_25_SEND_DATA *datas,AX_25_CFG *cfg_ax25);
static void AddMessage(AX_25 *frame,AX_25_SEND_DATA *datas);
static void AddPid(AX_25 *frame,AX_25_CFG *cfg_ax25);
static  void AddCRC(AX_25 *frame);
static void PrepareFrameForTransmitt(AX_25 *frame);
static unsigned char Bit_Reverse( unsigned char x );
void vTimerX25callback(xTimerHandle Timer);
/*!
 * \fn CreateAX25Timer
 * \brief Create the timer for this module
 * This function create a timer , this timer will be used for task synchronisation
 * */
xTimerHandle CreateAX25Timer(unsigned int Time){
  SemAllowAx25=xBeaconTime(Time);
  return xTimerCreate((const signed char*)"TimerX25",10*Time,pdFALSE,(void*)2,vTimerX25callback);

}
/*!
 * \fn StartX25Wait
 * \brief Start AX25 timer for task sync
 */
portBASE_TYPE StartX25Wait(xTimerHandle Timer){
  //xSemaphoreTake(Sems,portTICK_RATE_MS);
  return xTimerStart(Timer,0);
}
portBASE_TYPE StopX25wait(xTimerHandle Timer){
  return xTimerStop(Timer,0);
}
/*!
 * \fn MakeFrame
 * \brief Create the structure that contains all the needed data
 */
AX_25_SEND_DATA MakeFrame(const unsigned portCHAR* Message,const unsigned portCHAR* Dest){
  AX_25_SEND_DATA localFrame;

  //strncpy((portCHAR*)localFrame.dest_id,(portCHAR*)Message,strlen((portCHAR*)Dest));
  localFrame.dest_id=Dest;
  localFrame.message=Message;
  return localFrame;
}
/*!
 * \fn MakeConfig
 * \param1 Sender Adresse d'envoi
 * \param2 PID Indicateur de protocole
 *
 */
AX_25_CFG MakeConfig(const unsigned portCHAR* Sender,unsigned int PID){
  AX_25_CFG localCfg;
  localCfg.pid=(unsigned char)PID;
  localCfg.beaconInterval=30000;
  localCfg.send_id=(unsigned char *)Sender;
  return localCfg;
}
/*!
 * \fn xStartAX25task
 * \brief Start the encoding task and initialize the queue that contains the data encoded in ax25 (HDLC) format
 */
xQueueHandle xStartAX25task(void){
  //Stage2=xSemaphoreCreateMutex();

  xAX25Queue= xQueueCreate(SizeQueue,sizeof(AX_25_SEND_DATA));
  xTaskCreate(vAX25taskBase,(signed portCHAR*)"AX25",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+1,&Ax25Encode);
  return xAX25Queue;
}
xQueueHandle xStartAx25Cfg(xSemaphoreHandle *Sem){
  vSemaphoreCreateBinary(Sems);
  xSemaphoreTake(Sems,portTICK_RATE_MS*10);
  xAX25Cfg=xQueueCreate(SizeQueue,sizeof(AX_25_CFG));
  xTaskCreate(vAX25cfg,(signed portCHAR*)"AX25conf",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+1,&Ax25CFG);
  return xAX25Cfg;

}
xQueueHandle xStartAX25pipe(void){
  xAX25pipe=xQueueCreate(SizeQueue,sizeof(AX_25));
  //xTaskCreate(vAX25send,(signed portCHAR*)"AX25send",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+1,&Ax25Send);
  return xAX25pipe;
}
/*!
 * \fn xSemLockX25
 * \brief SemtolockX25
 */
xSemaphoreHandle xSemLockX25(void){
  Stage3=xSemaphoreCreateMutex();
  xSemaphoreTake(Stage3,portMAX_DELAY);
  return  Stage3;

}
xSemaphoreHandle xSemLockAx25Cfg(void){
  Stage1=xSemaphoreCreateMutex();
  //xSemaphoreTake(Stage1,portMAX_DELAY);
  return Stage1;
}
xSemaphoreHandle xSemLockEncodeX25(void){
  Stage2=xSemaphoreCreateMutex();
  return Stage2;

}
/*!
 * \fn xBeaconTime
 * \returns xSemaphore
 * This function create a semaphore for wait
 */
xSemaphoreHandle xBeaconTime(unsigned portBASE_TYPE time){
  SemAllowAx25=xSemaphoreCreateMutex();
  //xSemaphoreTake(SemAllowAx25,portTICK_RATE_MS*time*2);
  return SemAllowAx25;
}
xSemaphoreHandle GetSemStatusX25cfg(void){
  return Stage1;
}
xSemaphoreHandle GetSemStatusX25Enc(void){
  return SemAllowAx25;
}
xSemaphoreHandle GetSemStatusX25Send(void){
  return SemAllowAx25;
}

/*!
 * \fn InitTxInsertion
 * \brief Insersion de 010101 en début de trame
 * Cette fonction est à executer après le passage des valeurs en NRZI
 * Elle permet d'attendre que l'émeteur et démarré mais aussi de tester la fonction de modulation FSK
 * */
void InitTxInsertion(AX_25 *frame,unsigned int TXwaitReapeat,unsigned int *PosiInit){
  unsigned int CountMess=0;
  unsigned int CountMax=*PosiInit;
  frame->endmessage=pvPortMalloc(frame->MessSizeWithNoPreamble+(TXwaitReapeat+1)*sizeof(unsigned portCHAR));
  *frame->endmessage=(unsigned char)AX25_initmod_start;
  if(frame->endmessage!=NULL){
      while(CountMess++<=CountMax+TXwaitReapeat)
	{
	  frame->endmessage[CountMess]=(unsigned char)AX25_initmod_start;
	}
      frame->endmessage[CountMess]='\0';
      strncat((char*)frame->endmessage,(char*)frame->messageNrzi,frame->MessSizeWithNoPreamble);
      *PosiInit=strlen((char*)frame->endmessage);
      frame->AX25FrameSize=strlen((char*)frame->endmessage);
      vPortFree(frame->messageNrzi);
      frame->messageNrzi=NULL;

  }


}
/*!
 * \fn AddFlags
 *  \brief Insert AX25 flags
 *
 */
void AddFlags(AX_25 *frame,unsigned int FlagNumber,unsigned int *PosiInit){

  unsigned int Count=*PosiInit;
  unsigned int PositionInitaile=*PosiInit;
  unsigned int Endsize;
  unsigned char flagAx25=(unsigned char)AX25_Flags;
  frame->messageWithAX25Flags=pvPortCalloc(frame->MessSizeWithNoStartAndStopFlags+(FlagNumber<<1),sizeof(unsigned portCHAR));
  if(frame->MessSizeWithNoStartAndStopFlags!=0){
      strncpy((portCHAR*)frame->messageWithAX25Flags,(portCHAR*)"~",1);
      while(Count++<((FlagNumber>>1)+PositionInitaile)-1){
	  //strncat((portCHAR*)frame->messageWithAX25Flags,(portCHAR*)"~",2);
	  frame->messageWithAX25Flags[Count]=flagAx25;



      }
      frame->messageWithAX25Flags[Count]='\0';
      strncat((portCHAR*)frame->messageWithAX25Flags,(portCHAR*)frame->messageBitStuffedBeforeTX,frame->MessSizeWithNoStartAndStopFlags);

      //PosiInit=Count+frame->MessSizeWithNoStartAndStopFlags+1;
      Count=strlen((portCHAR*)frame->messageWithAX25Flags);
      Endsize=Count+(FlagNumber>>1)+1;
      frame->messageWithAX25Flags[Count]='\0';
      strcat((portCHAR*)frame->messageWithAX25Flags,(portCHAR*)"~");
      while(Count++<=Endsize){
	  frame->messageWithAX25Flags[Count]=flagAx25;
      }
      frame->MessSizeNoNrzi=Count;
  }
}
/*!
 * \fn vAX25taskBase
 * \brief In this task we create the frame
 */
void vAX25taskBase(void *pvParameters){
  for(;;)
    {//Verifier que l'on a passé l'étape 1 avant
      if(Sems!=NULL){//Au cas où
	  if(xSemaphoreTake(SemAllowAx25,portTICK_RATE_MS*10==pdTRUE)){
	      if(xSemaphoreTake(Sems,portTICK_RATE_MS*10==pdTRUE))
		{

		  portBASE_TYPE  xSendDataStat;
		  portBASE_TYPE  xGetDataAX25stat;
		  portBASE_TYPE  xGetCfgStat;
		  unsigned portBASE_TYPE size_fullmessage=0;
		  unsigned int posi=0;
		  //vTaskDelay(1000*portTICK_RATE_MS);
		  xGetDataAX25stat=xQueueReceive(xAX25Queue,&datas,portTICK_RATE_MS*100);
		  while(xGetDataAX25stat!=pdTRUE);
		  xGetCfgStat=xQueueReceive(xAX25Cfg,&cfg_ax25,portTICK_RATE_MS*100);
		  while(xGetCfgStat!=pdTRUE);
		  //Attente de données
		  //Copie des parametres
		  taskENTER_CRITICAL();
		  AddAdresses(&trame,&datas,&cfg_ax25);
		  AddPid(&trame,&cfg_ax25);
		  AddMessage(&trame,&datas);
		  AddCRC(&trame);
		  PrepareFrameForTransmitt(&trame);
		  ZeroInsert(&trame);
		  posi=0;
		  AddFlags(&trame,10,&posi);
		  Nrzi(&trame);
		  posi=size_fullmessage;
		  InitTxInsertion(&trame,5,&posi);
		  portEXIT_CRITICAL();
		  //Tou envoyer
		  xSendDataStat=xQueueSend(xAX25pipe,&trame,portMAX_DELAY);

		  if(xSendDataStat!=pdTRUE)
		    {

		      vPortFree(trame.endmessage);

		      trame.endmessage=NULL;
		    }
		  else{

		      vPortFree(trame.endmessage);
		      trame.endmessage=NULL;
		  }

		  xSemaphoreGive(Sems);//Libération du sémaphore 2


		}
	  }


      }else{
	  //Ne rien faire
      }
    }
}

/*!
 *
 * \fn vAX25cfg
 * \brief Configuration de la connection X25
 *
 *
 */
void vAX25cfg(void *param){
  portBASE_TYPE xStatusCFGax25;
  for(;;)
    {    //Prise du semaphore Stage1!
      if(Sems!=NULL){
	  xSemaphoreGive(Sems);
	  if(xSemaphoreTake(Sems,portTICK_RATE_MS*1==pdTRUE)){//Débloqué par la tache principale
	      //Le sémphore est prit on configure la couche x25
	      /*
	       * Dans cette partie je récupère la configuration de la connection x25
	       * */
	      xStatusCFGax25=xQueueReceive(xAX25Cfg,&cfg_ax25,portMAX_DELAY);//We get the configuration
	      if(xStatusCFGax25!=pdTRUE){
		  Board_UARTPutSTR("Config AX25 erreur!\n\r");

	      }
	      else{
		  //trame.send_id=cfg_ax25.send_id;
		  //Suspendre la tache car config faite en général une fois

	      }
	      //xSemaphoreGive(Stage1);//Plus besoin du sémaphore
	      //xSemaphoreTake(Stage1,portMAX_DELAY);//Block
	      xSemaphoreGive(Sems);//Unblock
	      vTaskSuspend(NULL);

	  }
      }
    }

}
#if 0
void vAX25send(void *pvParameters){
  for(;;){
      //xSemaphoreGive(Sems);
      if(xSemaphoreTake(Sems,portTICK_RATE_MS*10==pdTRUE)){
	  taskENTER_CRITICAL();
	  portBASE_TYPE  xSendDataStat;
	  xSendDataStat=xQueueSend(xAX25pipe,&trame,portMAX_DELAY);

	  if(xSendDataStat!=pdTRUE)
	    {
	      vPortFree(trame.dest_id);
	      vPortFree(trame.send_id);
	      vPortFree(trame.message);
	      vPortFree(trame.fullmessage);
	      vPortFree(trame.stuffedFrame);
	      trame.message=NULL;
	      trame.dest_id=NULL;
	      trame.send_id=NULL;
	      trame.fullmessage=NULL;
	      trame.stuffedFrame=NULL;
	    }
	  else{
	      vPortFree(trame.dest_id);
	      vPortFree(trame.send_id);
	      vPortFree(trame.message);
	      vPortFree(trame.fullmessage);
	      vPortFree(trame.stuffedFrame);
	      trame.message=NULL;
	      trame.dest_id=NULL;
	      trame.send_id=NULL;
	      trame.fullmessage=NULL;
	      trame.stuffedFrame=NULL;
	  }
	  portEXIT_CRITICAL();
	  //vTaskDelay(1000/portTICK_RATE_MS);
	  xSemaphoreGive(Sems);

      }
  }

}
#endif
/*!
 * @fn shiftmessageleft
 * @brief Shift message bits to left
 *
 */
void shiftmessageleft(unsigned char *p_Message ){
  unsigned int cnt=0;
  while(cnt<=strlen((char*)p_Message)){
      p_Message[cnt]=p_Message[cnt]<<1;
      cnt++;
  }
  p_Message[cnt]='\0';
}
/*!
 * \fn shiftmessageright
 * \brief Shift message bits to right
 * This function modifies the pointer on a characters value the value of each caracter is shifted to left
 *
 */
void shiftmessageright(unsigned char *p_Message ){
  unsigned int cnt=0;
  while(cnt<=strlen((char*)p_Message)){
      p_Message[cnt]=p_Message[cnt]>>1;
      cnt++;
  }
}
/*!
 * \fn stuffingframe
 * \brief insert null values in the message
 * This function inserts null bits in the message at each five consecutive ones , it's necessary for avoid 0x7E in frame
 * This function is launched in a freertos task
 */
void stuffingframe(P_AX_25 *ax25_frame)
{
  unsigned int cnt=0;
  (*ax25_frame)->stuffedFrame=NULL;
  (*ax25_frame)->stuffedFrame=pvPortMalloc(256*sizeof(unsigned char));
  for(cnt=0;cnt<=strlen((char*)(*ax25_frame)->fullmessage)-1;cnt++)
    {
      if(*(*ax25_frame)->fullmessage=='~')
	{
	  *(*ax25_frame)->stuffedFrame=(unsigned char)0x7d;
	  *++(*ax25_frame)->stuffedFrame=(unsigned char)0x5e;
	}
      else
	{
	  *(*ax25_frame)->stuffedFrame++=*(*ax25_frame)->fullmessage++;
	}
    }
}
/*!
 * \fn Nrzi
 * \brief NRZI encoding
 */
void Nrzi(AX_25 *frame){
  unsigned int Cnt=0;
  unsigned int CntBits=0;
  bool wasZero=false;
  frame->messageNrzi=NULL;
  frame->messageNrzi=pvPortMalloc(frame->MessSizeNoNrzi+1*sizeof(unsigned char));
  while(Cnt++<frame->MessSizeNoNrzi)
    {
      //0 to 8
      frame->messageNrzi[Cnt]=frame->messageWithAX25Flags[Cnt];
      while(CntBits++<8)
	{
	  if(frame->messageNrzi[Cnt]&(1u<<CntBits))//If 1 we make a state transition
	    {
	      if(wasZero==true){//Was zero before
		  wasZero=false;
		  frame->messageNrzi[Cnt]=frame->messageNrzi[Cnt]|1<<CntBits;//Output to one

	      }
	      else{//à 1 avant
		  wasZero=false;
		  frame->messageNrzi[Cnt]=frame->messageNrzi[Cnt]&1<<CntBits;//Output to out
	      }
	      wasZero=false;


	    }
	  else{//If zero was detected

	  }
	  wasZero=true;


	}
      CntBits=0;

    }

  frame->messageNrzi[Cnt]='\0';
  frame->MessSizeWithNoPreamble=Cnt+1;
  vPortFree(frame->messageWithAX25Flags);
  frame->messageWithAX25Flags=NULL;



}
/*!
 * @fn ZeroInsert
 * @brief Insert zeros in the stream
 * This function insert 0 to avoid 01111110 chars in the frame (for eleminate flags in the stream)
 */
void ZeroInsert(AX_25 *frame)
{
  unsigned int ParcourFrameIn=0;
  unsigned int ParcourFrameOut=0;
  unsigned int MaskInput=1u;
  unsigned int maskOutput=1u;
  unsigned int bitCountIn=0;
  unsigned int bitCountOut=0;
  unsigned int OnesCounter=0;
  if(frame->messageBitStuffedBeforeTX==NULL){
      frame->messageBitStuffedBeforeTX=(unsigned portCHAR*)pvPortCalloc(frame->MessSizeWithNoStuffed+10,sizeof(unsigned portCHAR));
  }
  for(ParcourFrameIn=0;ParcourFrameIn<strlen((portCHAR*)frame->messageShiftedBeforeTX);ParcourFrameIn++){
      //Tester si outputCont=8
      while(bitCountIn<=7){//To 0 to 7 (8 bits)
	  if(bitCountOut>=7){//7 bits in output
	      bitCountOut=0;//Reset the output bit count
	      maskOutput=1;
	      ParcourFrameOut++;//Pass to the next char in output stream
	  }
	  if((frame->messageShiftedBeforeTX[ParcourFrameIn])&(1u<<bitCountIn)){//Si 1
	      OnesCounter++;

	      if(OnesCounter==5){//Five contiguous "1"detected
		  //We construct the byte bit by bit
		  //In this case no incrementing the input counter
		  frame->messageBitStuffedBeforeTX[ParcourFrameOut]=frame->messageBitStuffedBeforeTX[ParcourFrameOut]&(~(1u<<maskOutput));
		  bitCountOut++;//Un bit de plus dans le stream(cas spécial)
		  maskOutput++;
		  OnesCounter=0;
		  frame->DebuGstuffingFunction++;
	      }
	      else{//Normal case
		  frame->messageBitStuffedBeforeTX[ParcourFrameOut]=(unsigned char)frame->messageShiftedBeforeTX[ParcourFrameOut]|(1u<<maskOutput);
		  MaskInput=MaskInput+1;
		  maskOutput++;
		  bitCountOut++;//Un bit de plus également en sortie
		  bitCountIn++;//One increment of bit counter in input


	      }
	  }
	  else{//0 was detected
	      frame->messageBitStuffedBeforeTX[ParcourFrameOut]=(unsigned char)frame->messageShiftedBeforeTX[ParcourFrameOut]&(~(1u<<maskOutput));
	      OnesCounter=0;//Raz du compteur
	      MaskInput++;
	      maskOutput++;
	      bitCountOut++;
	      bitCountIn++;//Un bit de plus en entrée traité

	  }
      }
      bitCountIn=0;
      MaskInput=0;
      ParcourFrameIn++;
  }
  frame->messageBitStuffedBeforeTX[ParcourFrameOut]='\0';
  frame->MessSizeNoNrzi=strlen((portCHAR*)frame->messageBitStuffedBeforeTX);
  vPortFree(frame->messageShiftedBeforeTX);
  frame->messageShiftedBeforeTX=NULL;
}
void ZeroRemove(AX_25 *frame)
{
  unsigned int ParcourFrameIn=0;
  unsigned int ParcourFrameOut=0;
  unsigned int MaskInput=0;
  unsigned int maskOutput;
  unsigned int bitCountIn=0;
  unsigned int bitCountOut=0;
  unsigned int OnesCounter=0;
  unsigned int zeroCounter;
  //Sécurité(au cas ou:))
  if(frame->UndecodedMessFromDemod==NULL){
      frame->ErrorDecoding=true;
  }
  else{
      frame->DestuffedFrame=pvPortCalloc(strlen((portCHAR*)frame->UndecodedMessFromDemod)+10,sizeof(unsigned portCHAR));

      for(ParcourFrameIn=0;ParcourFrameIn<strlen((portCHAR*)frame->UndecodedMessFromDemod);ParcourFrameIn++){
	  //Tester si outputCont=8

	  if((frame->UndecodedMessFromDemod[ParcourFrameIn])&MaskInput<<bitCountIn){
	      OnesCounter=+1;
	      zeroCounter=0;//Raz du compteur de zero
	      if(OnesCounter==5){//5 '1' detectés
		  //We construct the byte bit by bit
		  //Passer la sortie à 1
		  frame->DestuffedFrame[ParcourFrameOut]=frame->DestuffedFrame[ParcourFrameOut]|(1u<<maskOutput);
		  //bitCountOut=+1;//Un bit de plus dans le stream(cas spécial)
		  //maskOutput=+1;
		  MaskInput=+1;
		  bitCountIn=+1;
		  //OnesCounter=0;
		  //frame->DebuGstuffingFunction=+1;
	      }


	  }

	  else{//Zero
	      if(OnesCounter==5){//Si 5 1 détectés avant ignorer le zero
		  //Incrément du masque d'entrée
		  //Passer la valeur à 1
		  //frame->DestuffedFrame[ParcourFrameOut]=frame->DestuffedFrame[ParcourFrameOut]|(1u<<maskOutput));
		  maskOutput=+1;
		  OnesCounter=0;
		  zeroCounter=zeroCounter+1;

	      }
	      else{//Pas 5 '1' détectés traitement normal càd passage de la valeur de sortie à zéro
		  OnesCounter=0;
		  frame->DestuffedFrame[ParcourFrameOut]=frame->DestuffedFrame[ParcourFrameOut]&(~(1u<<maskOutput));
		  MaskInput=+1;


	      }
	      OnesCounter=0;//Raz du compteur

	      maskOutput=+1;
	      bitCountIn=+1;//Un bit de plus en entrée traité
	      bitCountOut=+1;//Un bit de plus également en sortie

	  }


	  //Sync
	  if(bitCountOut==7){
	      bitCountOut=0;
	      maskOutput=0;
	      ParcourFrameOut=+1;
	  }
	  if(bitCountIn==7)
	    {
	      bitCountIn=0;
	      MaskInput=0;

	    }

      }
  }




}
void AddAdresses(AX_25 *frame,AX_25_SEND_DATA *datas,AX_25_CFG *cfg_ax25){
  unsigned portBASE_TYPE ullenght_sendid,ullenght_destid;
  unsigned portBASE_TYPE ulcntdestid=0;
  unsigned portBASE_TYPE ulcntsendid=0;
  unsigned int counter=0;
  unsigned int CounterFrameOut=0;
  frame->send_id=NULL;
  frame->send_id=pvPortCalloc(10,sizeof(unsigned portCHAR));
  //frame->send_id=pvPortCalloc(10,sizeof(unsigned portCHAR));
  frame->dest_id=NULL;
  if(frame->dest_id==NULL){
      frame->dest_id=(unsigned portCHAR*)pvPortCalloc(10,sizeof(unsigned portCHAR));
  }
  ullenght_sendid=strlen((portCHAR*)cfg_ax25->send_id);
  ullenght_destid=strlen((portCHAR*)datas->dest_id);
  frame->MessSize=strlen((portCHAR*)datas->message);

  strcpy((portCHAR*)frame->send_id,(portCHAR*)cfg_ax25->send_id);
  strcpy((portCHAR*)frame->dest_id,(portCHAR*)datas->dest_id);
  frame->ulenght_destination=ullenght_destid;
  frame->ulenght_sender=ullenght_sendid;
  frame->messageWithOutPID=NULL;
  frame->messageWithOutPID=pvPortCalloc(strlen((portCHAR*)datas->message)+(2*(unsigned int)AX25_ADD_MAX_Size)+1,sizeof(unsigned portCHAR));
  for(counter=0;counter<=ullenght_sendid;counter++){
      frame->messageWithOutPID[CounterFrameOut++]=(unsigned char)frame->send_id[counter];
  }
  counter=0;

  if(frame->ulenght_sender<AX25_ADD_MAX_Size){
      for(ulcntsendid=AX25_ADD_MAX_Size-(AX25_ADD_MAX_Size-frame->ulenght_sender);ulcntsendid<AX25_ADD_MAX_Size;ulcntsendid++){
	  //frame->message[CounterFrameOut++]=' ';
	  strcat((portCHAR*)frame->messageWithOutPID," ");
	  CounterFrameOut++;
      }

  }
  strcat((portCHAR*)frame->messageWithOutPID,(portCHAR*)frame->dest_id);
  //for(counter=0;counter<=ullenght_destid;counter++){
  //frame->message[CounterFrameOut++]=(unsigned char)frame->dest_id[counter];
  //}
  counter=0;
  //strncat((portCHAR*)frame->message,(portCHAR*)frame->dest_id,frame->ulenght_destination);
  if(frame->ulenght_destination<AX25_ADD_MAX_Size){
      for(ulcntdestid=AX25_ADD_MAX_Size-(AX25_ADD_MAX_Size-frame->ulenght_destination);ulcntdestid<AX25_ADD_MAX_Size;ulcntdestid++){
	  strcat((portCHAR*)frame->messageWithOutPID," ");
	  CounterFrameOut++;
      }
  }
  //frame->message[CounterFrameOut]='\0';
  vPortFree(frame->dest_id);
  vPortFree(frame->send_id);
  frame->dest_id=NULL;
  frame->send_id=NULL;
  frame->MessSizeWithNoPID=strlen((portCHAR*)frame->messageWithOutPID);

}
void AddMessage(AX_25 *frame,AX_25_SEND_DATA *datas){
  frame->messageWithOutCRCOctet=NULL;
  frame->MessSize=strlen((portCHAR*)datas->message);
  frame->messageWithOutCRCOctet=pvPortMalloc(frame->MessSizeWithNoPayload+strlen((portCHAR*)datas->message)+1);
  strncpy((portCHAR*)frame->messageWithOutCRCOctet,(portCHAR*)frame->messageWithoutPayload,frame->MessSizeWithNoPayload);
  vPortFree(frame->messageWithoutPayload);
  frame->messageWithoutPayload=NULL;
  strncat((portCHAR*)frame->messageWithOutCRCOctet,(portCHAR*)datas->message,frame->MessSize);
  frame->MessSizeWithNoCRC=strlen((portCHAR*)frame->messageWithOutCRCOctet);

}
void AddPid(AX_25 *frame,AX_25_CFG *cfg_ax25){
  unsigned portSHORT sizemess=frame->MessSizeWithNoPID;
  frame->messageWithoutPayload=pvPortMalloc(strlen((portCHAR*)frame->messageWithOutPID)+3*sizeof(unsigned char ));
  strncpy((portCHAR*)frame->messageWithoutPayload,(portCHAR*)frame->messageWithOutPID,strlen((portCHAR*)frame->messageWithOutPID));
  vPortFree(frame->messageWithOutPID);
  frame->messageWithOutPID=NULL;
  frame->messageWithoutPayload[sizemess+1]=cfg_ax25->control;
  frame->messageWithoutPayload[sizemess+2]=cfg_ax25->pid;
  frame->messageWithoutPayload[sizemess+2]='\0';
  frame->MessSizeWithNoPayload=strlen((portCHAR*)frame->messageWithoutPayload);

}
void AddCRC(AX_25 *frame){
  //frame->message=frame->message+frame->MessSizeWithNoCRC+1;
  frame->messageWithoutSyncFlags=NULL;
  frame->messageWithoutSyncFlags=pvPortMalloc(frame->MessSizeWithNoCRC+3*sizeof(unsigned portCHAR));
  strncpy((portCHAR*)frame->messageWithoutSyncFlags,(portCHAR*)frame->messageWithOutCRCOctet,frame->MessSizeWithNoCRC);
  frame->crc_hight=(Crc16(CCIT_INIT_FCS_CALC,frame->messageWithOutCRCOctet,frame->MessSizeWithNoCRC)>>GET_HIGHT_BITS)&EIGHT_BIT_MASK;
  frame->crc_low=(Crc16(CCIT_INIT_FCS_CALC,frame->messageWithOutCRCOctet,frame->MessSizeWithNoCRC))&EIGHT_BIT_MASK;
  vPortFree(frame->messageWithOutCRCOctet);
  frame->messageWithOutCRCOctet=NULL;
  frame->crc_low=~frame->crc_low;
  frame->crc_hight=~frame->crc_hight;
  frame->crc_low=Bit_Reverse(frame->crc_low);
  frame->crc_hight=Bit_Reverse(frame->crc_hight);
  frame->messageWithoutSyncFlags[frame->MessSizeWithNoCRC+1]=(unsigned portCHAR)frame->crc_low;
  frame->messageWithoutSyncFlags[frame->MessSizeWithNoCRC+2]=(unsigned portCHAR)frame->crc_hight;
  frame->messageWithoutSyncFlags[frame->MessSizeWithNoCRC+3]='\0';
  frame->MessSizeWithNoStartAndStopFlags=strlen((portCHAR*)frame->messageWithoutSyncFlags);

}
void PrepareFrameForTransmitt(AX_25 *frame){
  frame->messageShiftedBeforeTX=NULL;
  frame->messageShiftedBeforeTX=pvPortMalloc(frame->MessSizeWithNoStartAndStopFlags+1*sizeof(unsigned portCHAR));
  strncpy((portCHAR*)frame->messageShiftedBeforeTX,(portCHAR*)frame->messageWithoutSyncFlags,frame->MessSizeWithNoStartAndStopFlags);
  shiftmessageleft(frame->messageShiftedBeforeTX);
  frame->MessSizeWithNoStuffed=strlen((portCHAR*)frame->messageShiftedBeforeTX);
  vPortFree(frame->messageWithoutSyncFlags);
  frame->messageWithoutSyncFlags=NULL;
}
void AddStartAndStopFlags(AX_25 *frame){
  frame->messageWithAX25Flags=NULL;
  frame->messageWithAX25Flags=(unsigned portCHAR*)pvPortMalloc((frame->MessSizeWithNoStartAndStopFlags+MAX_AX25_FLAGS)*sizeof(unsigned portCHAR));
}
/*!
 * \fn Bit_Reverse
 * \brief Bit Inversion
 * Invert bits in a byte
 */
unsigned char Bit_Reverse( unsigned char x )
{
  x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
  x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
  x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
  return x;
}
/*!
 * @fn insertdata
 * @brief Insert data in the frame
 * @return Position of the counter
 * This function permit the insertion of datas into a character string The character is a pointer and
 * it's incremented after insert of each insertion and we also count the numbers of insertions by
 * incremeted a counter
 * Finaly we will return an unsigned int for count of inserted characters
 */
unsigned int  insertdata(unsigned char *p_Message,unsigned char *p_Data,unsigned int iPos){
  unsigned int cnt=iPos;
  //shiftmessageleft(p_Data);
  while(cnt-strlen((char*)p_Data)<=cnt+strlen((char*)p_Data)){
      p_Message[cnt]=p_Data[cnt];
      cnt++;
  }
  return cnt;
}
/*!
 * \fn
 */
void vTimerX25callback(xTimerHandle Timer){
  //In this function unlock a semaphore
  xSemaphoreGive(SemAllowAx25);
}
