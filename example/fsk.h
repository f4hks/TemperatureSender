/*
 * fsk.h
 *
 *  Created on: 13 déc. 2015
 *      Author: pierre
 */

#ifndef FSK_H_
#define FSK_H_
typedef struct{
portLONG FreqUency[32];
	portLONG MFSK;
	portLONG debit;
}__attribute__((__packed__, aligned(2))) FSk_param;
typedef struct{
       portCHAR Message[255];
}__attribute__((__packed__, aligned(2))) DigitalData;
xQueueHandle vStartQueueFSK(void);
xQueueHandle vStartConfigFSK(void);

#endif /* FSK_H_ */
