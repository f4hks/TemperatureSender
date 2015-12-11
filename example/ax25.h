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
	unsigned char *fullmessage;
	unsigned char *endmessage;
	unsigned char *flag;
}__attribute__((__packed__, aligned(2))) AX_25;
typedef AX_25 *P_AX_25;

void shiftmessageleft(unsigned char *p_Message );
void shiftmessageright(unsigned char *p_Message );
void stuffingframe(unsigned char *p_Message);
unsigned int insertdata(unsigned char *p_Message,unsigned char *p_Data,unsigned int iPos);
void makeax25payload(const unsigned char*stationSSID,const unsigned char*SSIDDest,const unsigned char *Pmessage,P_AX_25 *ax25_frame);
void decodeax25(unsigned char *p_ax25,unsigned char *Pmessage );

#endif /* AX25_H_ */
#define AX25_FLAG 0x7E
