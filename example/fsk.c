/*
 * fsk.c
 *
 *  Created on: 13 déc. 2015
 *      Author: pierre
 */
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis.h"
#include "stdbool.h"
#include "portmacro.h"
#include "portable.h"
#include "board.h"
#include "timers.h"
#include "task.h"
#include  "fsk.h"
#include "dac_17xx_40xx.h"
xQueueHandle DataStream;
xQueueHandle ConfigFSK;
xQueueHandle vStartQueueFSK(void){
DataStream=xQueueCreate(1,sizeof(DigitalData));
return DataStream;
}
xQueueHandle vStartConfigFSK(void){
  ConfigFSK=xQueueCreate(1,sizeof(ConfigFSK));
  return ConfigFSK;
}

