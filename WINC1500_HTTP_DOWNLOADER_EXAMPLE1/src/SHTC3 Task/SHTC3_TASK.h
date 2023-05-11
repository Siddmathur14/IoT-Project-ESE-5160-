/*
 * SHTC3_TASK.h
 *               
 * Created: 5/3/2023 4:32:36 PM
 *  Author: siddm
 */ 


#ifndef SHTC3_TASK_H_
#define SHTC3_TASK_H_


#include "I2cDriver/I2cDriver.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "TEMPERATURE/SHTC3.h"
#include "Pulse rate/pulse.h"

#define SHTC3_TASK_SIZE	200
#define SHTC3_PRIORITY (configMAX_PRIORITIES - 1)

void SHTC3Task(void *pvParameters);


#endif /* SHTC3_TASK_H_ */