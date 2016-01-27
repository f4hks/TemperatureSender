/*
 * gensinus.h
 *
 *  Created on: 30 déc. 2015
 *      Author: pierre
 */

#ifndef GENSINUS_H_
#define GENSINUS_H_
typedef struct{
  portBASE_TYPE Frequency;
  bool TX;
}xFrequencyGen;
xQueueHandle InitGenQueue(void);
xFrequencyGen SetFreq(portBASE_TYPE Freqz);
void pVfrequencyChange(portBASE_TYPE Freqz);
void StartSignalGen(void);
void StopSignalGen(void);
#endif /* GENSINUS_H_ */
