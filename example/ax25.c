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
#include "ax25.h"
#include <string.h>
#include <stdlib.h>
#include <crc.h>
#include "queue.h"
#include "task.h"
#define SizeQueue  5
xQueueHandle xAX25Queue;
static void vAX25taskBase(void *pvParameters);
/*!
 * \fn xStartAX25task
 * \brief Start the encoding task and initialize the queue that contains the data encoded in ax25 (HDLC) format
 */
xQueueHandle xStartAX25task(void){

  xAX25Queue= xQueueCreate(SizeQueue,sizeof(AX_25));
  xTaskCreate(vAX25taskBase,(signed portCHAR*)"AX25",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+1,NULL);
  return xAX25Queue;
}
/*!
 * \fn vAX25taskBase
 * \brief In this task we get the configuration
 */
void vAX25taskBase(void *pvParameters){




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


