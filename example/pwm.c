/*!
 *  @file :pwm.c
 *
 *  @date: 4 sept. 2015
 *  @author: pierre
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
#include "pwm.h"
xQueueHandle pwmCfg;

xQueueHandle vCreateQueuePwm(void)
{
    pwmCfg=xQueueCreate(1,sizeof(xPWMCfg));
    return pwmCfg;

}

