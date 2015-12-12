/*
 * pwm.h
 *
 *  Created on: 4 sept. 2015
 *      Author: pierre
 */

#ifndef PWM_H_
#define PWM_H_
xQueueHandle vCreateQueuePwm(void);
/*!
 *  \struct xPWMcfg
 * 	\brief Sensor configuration
 *
 */
typedef struct{

	portLONG FreqUency[32];
	portLONG NFSK;
	portLONG debit;
}__attribute__((__packed__, aligned(2))) xPWMCfg;


#endif /* PWM_H_ */
