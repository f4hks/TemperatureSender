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
	unsigned int Frequencies[16];

}__attribute__((__packed__, aligned(2))) FSk_param;
typedef struct{
       portCHAR Message[255];
}__attribute__((__packed__, aligned(2))) DigitalData;
typedef struct{
    unsigned int TimerHitsCounts;
}__attribute__((__packed__, aligned(2))) TimerStat;
typedef struct{
    portLONG EventsOccur;
    portLONG FrequencyChangeRequested;//Pour informer de charger jusqu'a n-1 echantillon (évite la coupure de la phase)
    portLONG Pos_in_frame;
    portLONG SizeDatas;
}__attribute__((__packed__, aligned(2))) StatusFSK;
typedef struct{
    portLONG FrequencyChangeRequested;//Pour informer de charger jusqu'a n-1 echantillon (évite la coupure de la phase)
    uint32_t PwmMode;
    uint32_t PrVal;//(96/4)24=1µs
    uint32_t FReq1;//=9
    uint32_t Freq2;//=18

}__attribute__((__packed__, aligned(2))) ConfigPWM;
typedef struct {
    uint32_t Results[16];
}__attribute__((__packed__, aligned(2))) Timer1Set;
typedef enum{
  MAXSAMPLE=128
}FSKSET;
xQueueHandle vStartQueueFSK(void);
xQueueHandle vStartConfigFSK(void);

#endif /* FSK_H_ */
