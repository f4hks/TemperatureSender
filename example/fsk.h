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
       unsigned portCHAR *Message;
}__attribute__((__packed__, aligned(2))) DigitalData;
typedef struct{
    unsigned int TimerHitsCounts;
}__attribute__((__packed__, aligned(2))) TimerStat;
typedef struct{
    volatile portLONG EventsOccur;
    volatile portLONG FrequencyChangeRequested;//Pour informer de charger jusqu'a n-1 echantillon (évite la coupure de la phase)
    volatile portLONG Pos_in_frame;
    volatile portLONG SizeDatas;
}__attribute__((__packed__, aligned(2))) StatusFSK;
typedef struct{
   _Bool AllTransmit;
}__attribute__((__packed__, aligned(2)))tStatusModulator;
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
xQueueHandle xStartQueueFSK(void);
xQueueHandle xStartConfigFSK(void);
xQueueHandle xStartStatusQueueFSK(void);
DigitalData InitDigitalData(unsigned char *DataPtr);
FSk_param InitConfigurationFSK(portLONG M,portLONG baudRate,portLONG MinFReq,portLONG DeltaF);
void SetFrequence(FSk_param *param,unsigned int num,unsigned freq);
#define MAX_VAL 7
#define TWO_FSK 2
#define FOUR_FSK 4
#define EIGHT_FSK 8
#endif /* FSK_H_ */
