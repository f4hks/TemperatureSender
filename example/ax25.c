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
#include "ax25.h"
#define SizeQueue  1
xQueueHandle xAX25Queue;
xTaskHandle Ax25CFG;
xTaskHandle Ax25Encode;
xQueueHandle xAX25Cfg;
xQueueHandle xAX25pipe;
AX_25_CFG cfg_ax25;


P_AX_25 *pAX25trame;
static void vAX25taskBase(void *pvParameters);
static void vAX25cfg(void *pvParameters);
static void Nrzi(unsigned char *Mess);
static unsigned char Bit_Reverse( unsigned char x );
/*!
 * \fn xStartAX25task
 * \brief Start the encoding task and initialize the queue that contains the data encoded in ax25 (HDLC) format
 */
xQueueHandle xStartAX25task(void){

  xAX25Queue= xQueueCreate(SizeQueue,sizeof(AX_25_SEND_DATA));
  xTaskCreate(vAX25taskBase,(signed portCHAR*)"AX25",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+1,&Ax25Encode);
  return xAX25Queue;
}
xQueueHandle xStartAx25Cfg(void){
  //vTaskSuspend(Ax25Encode);
  xAX25Cfg=xQueueCreate(SizeQueue,sizeof(AX_25_CFG));
  //xTaskCreate(vAX25cfg,(signed portCHAR*)"AX25conf",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+1,&Ax25CFG);
  //vTaskResume(Ax25Encode);
  return xAX25Cfg;

}
xQueueHandle xStartAX25pipe(void){
  xAX25pipe=xQueueCreate(SizeQueue,sizeof(AX_25));
  return xAX25pipe;
}
/*!
 * \fn vAX25taskBase
 * \brief In this task we create the frame
 */
void vAX25taskBase(void *pvParameters){
  for(;;)
    {
      AX_25 trame;
      AX_25_SEND_DATA datas;
      portBASE_TYPE  xGetDataAX25stat;
      portBASE_TYPE  xSendDataStat;
      portBASE_TYPE  xGetCfgStat;
      unsigned portBASE_TYPE size;
      unsigned portBASE_TYPE size_fullmessage;
      unsigned portBASE_TYPE lenght_sendid,cntsendid,lenght_dest_id,cntdestid;
      vTaskDelay(1000*portTICK_RATE_MS);
      xGetDataAX25stat=xQueueReceive(xAX25Queue,&datas,portMAX_DELAY);
      while(xGetDataAX25stat!=pdTRUE);
      xGetCfgStat=xQueueReceive(xAX25Cfg,&cfg_ax25,portMAX_DELAY);
      while(xGetCfgStat!=pdTRUE);
      //Attente de données
      //Copie des parametres
      taskENTER_CRITICAL();
      trame.send_id=NULL;
      //trame.send_id=pvPortMalloc(10*sizeof(unsigned portCHAR));
      trame.send_id=pvPortCalloc(10,sizeof(unsigned portCHAR));
      trame.dest_id=NULL;
      trame.dest_id=pvPortCalloc(10,sizeof(unsigned portCHAR));
      trame.message=NULL;
      trame.message=pvPortCalloc(100,sizeof(unsigned portCHAR));
      trame.crc_hight=0xff;
      trame.crc_low=0xff;
      lenght_sendid=strlen((portCHAR*)cfg_ax25.send_id);
      lenght_dest_id=strlen((portCHAR*)datas.dest_id);
      strncat((portCHAR*)trame.dest_id,(portCHAR*)datas.dest_id,strlen((portCHAR*)datas.dest_id));
      strncat((portCHAR*)trame.send_id,(portCHAR*)cfg_ax25.send_id,strlen((portCHAR*)cfg_ax25.send_id));
      //strncat((portCHAR*)trame.message,(portCHAR*)datas.message,strlen((portCHAR*)datas.message));
      //trame.message[0]='~';
      //trame.message[1]='~';
      strncat((portCHAR*)trame.message,(portCHAR*)trame.send_id,strlen((portCHAR*)trame.send_id));
      size=strlen((portCHAR*)trame.message);
      if(lenght_sendid<AX25_ADD_MAX_Size){
	  cntsendid=AX25_ADD_MAX_Size-(AX25_ADD_MAX_Size-lenght_sendid);
	  while(cntsendid++<AX25_ADD_MAX_Size){
	      trame.message[size++]=' ';
	  }

      }
      size=strlen((portCHAR*)trame.message);
      strncat((portCHAR*)trame.message,(portCHAR*)trame.dest_id,strlen((portCHAR*)trame.dest_id));
      size=strlen((portCHAR*)trame.message);
      if(lenght_dest_id<AX25_ADD_MAX_Size){
	  cntdestid=AX25_ADD_MAX_Size-(AX25_ADD_MAX_Size-lenght_dest_id);
	  while(cntdestid++<AX25_ADD_MAX_Size)
	    {
	      trame.message[size++]=' ';
	    }
      }
      /*Insertion du PID */
      size=strlen((portCHAR*)trame.message);
      strncat((portCHAR*)trame.message,(portCHAR*)datas.message,strlen((portCHAR*)datas.message));
      size=strlen((portCHAR*)trame.message);
      /*Dans cette partie le programme calcule la checksum du paquet généré */
      trame.crc_hight=(Crc16(AX25_initCRC,trame.message,size)>>4)&Mask8bit;
      trame.crc_low=(Crc16(AX25_initCRC,trame.message,size))&Mask8bit;
      trame.crc_hight=Bit_Reverse(trame.crc_hight);
      trame.crc_low=Bit_Reverse(trame.crc_low);
      /*Insertion du low fcs dans la trame */
      trame.message[size++]=(unsigned char)trame.crc_low;
      /*Insertion du high fcs dans la trame*/
      size=strlen((portCHAR*)trame.message);
      trame.message[size++]=(unsigned char)trame.crc_hight;
      size=strlen((portCHAR*)trame.message);
      /*Ajout des fanions de fin et de debut*/

      trame.stuffedFrame=NULL;
      trame.stuffedFrame=pvPortCalloc(size+10,sizeof(unsigned portCHAR));
      strncpy((portCHAR*)trame.stuffedFrame,(portCHAR*)trame.message,size);

      shiftmessageleft(trame.stuffedFrame);
      ZeroInsert(trame.stuffedFrame);
      size=strlen((portCHAR*)trame.stuffedFrame);
      trame.fullmessage=NULL;
      trame.fullmessage=pvPortCalloc(size+10,sizeof(unsigned portCHAR));
      //shiftmessageright(trame.stuffedFrame);
      /*FSK wake up */
      trame.fullmessage[0]=(unsigned char)AX25_initmod_start;
      trame.fullmessage[1]=(unsigned char)AX25_initmod_start;
      trame.fullmessage[2]=(unsigned char)AX25_Flags;
      trame.fullmessage[3]=(unsigned char)AX25_Flags;
      trame.fullmessage[4]='\0';
      strncat((portCHAR*)trame.fullmessage,(portCHAR*)trame.stuffedFrame,size);
      size_fullmessage=strlen((portCHAR*)trame.fullmessage);
      trame.fullmessage[size_fullmessage++]=(unsigned char)AX25_Flags;
      size_fullmessage=strlen((portCHAR*)trame.fullmessage);
      trame.fullmessage[size_fullmessage++]=(unsigned char)AX25_Flags;
      Nrzi(trame.fullmessage);
      //Envoyer le tout vers une file de message
      xSendDataStat=xQueueSend(xAX25pipe,&trame,portMAX_DELAY);
      if(xSendDataStat!=pdTRUE){
	  Board_UARTPutSTR("frame was send !\n\r");
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
	  Board_UARTPutSTR("frame was failed on send !\n\r");
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

      taskEXIT_CRITICAL();



    }
}
/*!
 *
 * \fn vAX25cfg
 *
 */
void vAX25cfg(void *pvParameters){
  portBASE_TYPE xStatusCFGax25;
  for(;;)
    {
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
	  vTaskSuspend(NULL);
      }
    }
}
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
void Nrzi(unsigned char *Mess){
  unsigned int Cnt=0;
  unsigned int CntBits=0;
  bool wasZero;
  while(Cnt++<strlen((portCHAR*)Mess))
    {
      //0 à 8
      unsigned char Car=(unsigned portCHAR)Mess[Cnt];
      while(CntBits++<8)
	{
	  if(Car&(1u<<CntBits))//Si 1
	    {
	      if(wasZero==true){//A zéro avant
		  Car=Car;
		  wasZero=false;
	      }
	      else{//à 1 avant
		  Car=Car;
		  wasZero=false;
	      }
	    }
	  else{//à zero
	      if(wasZero==true){//à zero avant
		  wasZero=true;
		  Car=Car|1u<<CntBits;//On le met à 1

	      }
	      else{//à 1 avant
		  wasZero=true;
		  Car=Car&1u<<CntBits;//On le met à 0
	      }
	  }
	  Mess[Cnt]=Car;

	}

    }

}
/*!
 * @fn ZeroInsert
 * @brief Insert zeros in the stream
 * This function insert 0 to avoid 01111110 chars in the frame (for eleminate flags in the stream)
 */
void ZeroInsert(unsigned char *frame){
  unsigned int SiZeStReAm=strlen((portCHAR*)frame);
  unsigned int CnT=0;
  unsigned portCHAR CaR;
  unsigned portCHAR CaRNew;
  unsigned int NumberOfStuffs=0;
  unsigned int BiTcNt=0;
  unsigned int NextBit=0;
  unsigned int onesCounter=0;
  /*On this part the function works character by character */
  while(CnT<SiZeStReAm){
      /*We get the character*/
      CaR=frame[CnT];
      /*Work bit by bit*/
      CaRNew=CaR;
      NextBit=CaR&(1u<<(BiTcNt+1));
      while(BiTcNt<(sizeofchar-1))
	{
	  /*Sauvegarde de l'état du bit suivant*/
	  NextBit=CaR&(1u<<(BiTcNt+1));
	  if(CaR&(1u<<BiTcNt))
	    {
	      //Construction de l'octet
	      if(onesCounter==5){
		  CaRNew=(CaRNew&(unsigned portCHAR)1<<BiTcNt);
		  CaRNew=(CaRNew|NextBit);
		  onesCounter=0;
		  BiTcNt=+1;
		  NumberOfStuffs=NumberOfStuffs+1;
	      }
	      else
		{
		  CaRNew=(CaRNew|((unsigned portCHAR)1<<BiTcNt));
		  onesCounter++;//Cas à 1
		}
	    }
	  else//à zero
	    {
	      unsigned char Msk=~(1u<<BiTcNt);
	      CaRNew=(CaRNew&(Msk));
	      onesCounter=0;//Reset the ones counter

	    }
	  BiTcNt++;
	}
      BiTcNt=0;
      frame[CnT]=CaRNew;
      CnT++;
  }


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
#if 0
/*!
 * \fn makeax25payload
 * \brief Make an AX25 frame
 * This function assemble all datas needded for an AX25 frame this function is called by a freertos task and use a pointer for pass values
 */
void makeax25payload(const unsigned char*stationSSID,const unsigned char*SSIDDest,const unsigned char *Pmessage,P_AX_25 *ax25_frame){


  unsigned char CRCLOW=0xff;
  unsigned char CRCHIGH=0xff;
  (*ax25_frame)->send_id=pvPortMalloc((sizeof(unsigned char*)*strlen((char*)stationSSID)));
  (*ax25_frame)->dest_id=malloc(sizeof(unsigned char*)*strlen((char*)SSIDDest));
  (*ax25_frame)->send_id=(unsigned char*)stationSSID;
  (*ax25_frame)->dest_id=(unsigned char*)SSIDDest;
  (*ax25_frame)->message=malloc(strlen((char*)Pmessage)*sizeof(unsigned char *));
  //strncpy((char*)(*ax25_frame)->message,(char*)Pmessage,strlen((char*)Pmessage)-4);
  (*ax25_frame)->message=(unsigned char*)Pmessage;
  (*ax25_frame)->fullmessage=NULL;
  (*ax25_frame)->fullmessage=malloc(strlen((char*)Pmessage)+strlen((char*)stationSSID)+strlen((char*)SSIDDest)+2);
  strcat((char*)(*ax25_frame)->fullmessage,(const char*)(unsigned char*)(*ax25_frame)->send_id);
  strcat((char*)(*ax25_frame)->fullmessage,(const char*)32);
  strcat((char*)(*ax25_frame)->fullmessage,(const char*)(unsigned char*)(*ax25_frame)->dest_id);
  strcat((char*)(*ax25_frame)->fullmessage,(const char*)(unsigned char*)(*ax25_frame)->message);
  //Calcul du crc sur la chaine non décalée
  //Cette fonction est faite par une librairie externe fournie par freertos
  //crc=Crc8CCITT(crc,ax25_frame->message,strlen((char*)ax25_frame->message));
  //crc=Crc16(crc,ax25_frame->)
  CRCLOW=Crc8CCITT(CRCLOW,(unsigned char*)(*ax25_frame)->fullmessage,strlen((char*)(*ax25_frame)->fullmessage));
  CRCHIGH=(Crc16(0xffff,(*ax25_frame)->fullmessage,strlen((char*)(*ax25_frame)->fullmessage))>>4)&0xF;
  strncat((char*)(*ax25_frame)->fullmessage,(char*)&CRCHIGH,strlen((char*)(*ax25_frame)->fullmessage));
  strncat((char*)(*ax25_frame)->fullmessage,(char*)&CRCLOW,strlen((char*)(*ax25_frame)->fullmessage));
  stuffingframe(ax25_frame);
  (*ax25_frame)->endmessage=NULL;
  (*ax25_frame)->endmessage=malloc(strlen((char*)(*ax25_frame)->fullmessage)+strlen((char*)SSIDDest)+2);
  (*ax25_frame)->endmessage[0]=0b10101010;//010101010
  (*ax25_frame)->endmessage[1]=0b10101010;//010101010
  strcat((char*)(*ax25_frame)->endmessage,"~");
  strcat((char*)(*ax25_frame)->endmessage,(const char*)(unsigned char*)(*ax25_frame)->stuffedFrame);
  strcat((char*)(*ax25_frame)->endmessage,"~");
  strcat((char*)(*ax25_frame)->endmessage,"~");

}

#endif
