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
	unsigned char *dest_id;
	unsigned char *send_id;
	unsigned char *crc;
	unsigned char crc_low;
	unsigned char crc_hight;
	unsigned char *fullmessage;
	unsigned char *endmessage;
	unsigned char *flag;
	unsigned char *stuffedFrame;
	unsigned int CurrBitStat;
	unsigned int LastBitValue;
	unsigned int NexTbitValue;
	unsigned int BitPosINframe;
	unsigned int BitPosOUTframe;
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
typedef AX_25 *P_AX_25;
xQueueHandle xStartAX25task(void);
xQueueHandle xStartAx25Cfg(void);
xQueueHandle xStartAX25pipe(void);

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
