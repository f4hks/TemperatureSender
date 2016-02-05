/*
 * ax25.h
 *
 *  Created on: 31 août 2015
 *      Author: pierre
 *
 */

#ifndef AX25_H_
#define AX25_H_
typedef struct
{
	unsigned char *message;
	unsigned char *messageShiftedBeforeTX;
	unsigned char *messageBitStuffedBeforeTX;
	unsigned char *dest_id;
	unsigned char *send_id;
	unsigned char *crc;
	unsigned char crc_low;
	unsigned char crc_hight;
	unsigned char *messageWithAX25Flags;
	unsigned char *messageWithPreambleSync;
	unsigned char *messageFromAX25func;
	unsigned char *fullmessage;
	unsigned char *NrziChaine;
	unsigned char *endmessage;
	unsigned char *flag;
	unsigned char *stuffedFrame;
	unsigned int CurrBitStat;
	unsigned int LastBitValue;
	unsigned int NexTbitValue;
	unsigned int BitPosINframe;
	unsigned int BitPosOUTframe;
	unsigned portSHORT MessSize;
	unsigned portSHORT MessSizeWithNoCRC;
	unsigned portSHORT MessSizeWithNoPID;
	unsigned portSHORT MessSizeWithNoPayload;
	unsigned portSHORT MessSizeWithNoStartAndStopFlags;
	unsigned portSHORT MessSizeWithNoPreamble;
	unsigned portBASE_TYPE ulenght_destination;
	unsigned portBASE_TYPE ulenght_sender;
	unsigned int DebuGstuffingFunction;
	unsigned char *UndecodedMessFromDemod;
	unsigned char *DestuffedFrame;
	bool ErrorDecoding;
}__attribute__((__packed__, aligned(2))) AX_25;
typedef struct
{
	unsigned portCHAR *send_id;//Station de base
	unsigned portCHAR pid;
	unsigned portCHAR control;
	//Autres champs possibles(relay etc)
}__attribute__((__packed__, aligned(2))) AX_25_CFG;
/*!
 * \struct AX_25_SEND_DATA
 * \brief The structure that contains messages and dest addresses
 */
typedef struct
{
	const unsigned char *dest_id;//Destinataire
	const unsigned char *message;//Message
	//Autres champs possibles
}__attribute__((__packed__, aligned(2))) AX_25_SEND_DATA;
typedef struct {
	      unsigned char *Input;
	      unsigned char *Output;
	      unsigned int OutputCnt;
	      unsigned int InputCnt;
	      unsigned int MaskIn;
	      unsigned int MaskOut;

}__attribute__((__packed__, aligned(2))) AX25_LOW_LEVEL_PREP;

typedef enum {
  NOPROTO=0x03,

  POLLMODEON=0x10,
  POLLMODEOFF=0x00,
  UIFRAME_POLL_MODE_OFF=NOPROTO^POLLMODEOFF,
  UIFRAME_POLL_MODE_ON=NOPROTO^POLLMODEON

}AX25_PID_CTRL;
typedef enum {
  EIGHT_BIT_MASK=0xff,
  GET_HIGHT_BITS=4,
  CCIT_INIT_FCS_CALC=0xffff

}AX25_CRC_CALCS;
typedef enum {
  MAX_AX25_FLAGS=4,
  MAX_SYNC_FLAGS=16
}AX25_FLAGS_SEC;
typedef AX_25 *P_AX_25;
xQueueHandle xStartAX25task(void);
xQueueHandle xStartAx25Cfg(xSemaphoreHandle *Sem);
xQueueHandle xStartAX25pipe(void);
xSemaphoreHandle xSemLockX25(void);
xSemaphoreHandle xSemLockAx25Cfg(void);
xSemaphoreHandle GetSemStatusX25cfg(void);
xSemaphoreHandle GetSemStatusX25Enc(void);
xSemaphoreHandle GetSemStatusX25Send(void);
void shiftmessageleft(unsigned char *p_Message );
void shiftmessageright(unsigned char *p_Message );
void stuffingframe(P_AX_25 *ax25_frame);
unsigned int insertdata(unsigned char *p_Message,unsigned char *p_Data,unsigned int iPos);
void makeax25payload(const unsigned char*stationSSID,const unsigned char*SSIDDest,const unsigned char *Pmessage,P_AX_25 *ax25_frame);
void decodeax25(unsigned char *p_ax25,unsigned char *Pmessage );
void ZeroInsert(AX_25 *frame);
AX_25_SEND_DATA MakeFrame(const unsigned portCHAR* Message,const unsigned portCHAR* Dest);
AX_25_CFG MakeConfig(const unsigned portCHAR* Sender,unsigned int PID);
#endif /* AX25_H_ */
#define AX25_FLAG 0x7E
#define AX25_ADD_MAX_Size 7
#define AX25_initCRC 0xffff
#define Mask8bit 0xff
#define AX25_initmod_start 170
#define AX25_Flags 126
#define LasTValueMask 0x80
#define sizeofchar 8
