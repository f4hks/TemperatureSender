/*
 * tmpSensor.h
 *
 *  Created on: 1 déc. 2015
 *      Author: pierre
 */

#ifndef TMPSENSOR_H_
#define TMPSENSOR_H_

xQueueHandle xStartTempTask(void);
xQueueHandle xConfigADC(void);
/*!
 *  \struct xTmpSensorCfg
 * 	\brief Sensor configuration
 *
 */
typedef struct{

	long Samples;
	bool BurstMode;
}__attribute__((__packed__, aligned(2))) xTmpSensorCfg;
/*!
 *  \struct xTmpSensorCfg
 * 	\brief Sensor Results
 *
 */
typedef struct{
	portFLOAT mVolts;
	uint16_t Temperature;
	portFLOAT temp_finale;
}__attribute__((__packed__, aligned(2))) xTmpSensorResult;
#define VOLTAGEDIV (3.3/4096)
#define CONVERT_MV_TO_C 500
#define DIV_REPORT 10
#endif /* TMPSENSOR_H_ */
