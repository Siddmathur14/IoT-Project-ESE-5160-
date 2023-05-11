/*
 * heartrate_task.h
 *
 * Created: 5/4/2023 7:31:29 PM
 *  Author: siddm
 */ 


#ifndef HEARTRATE_TASK_H_
#define HEARTRATE_TASK_H_



#include "I2cDriver/I2cDriver.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "TEMPERATURE/SHTC3.h"
#include "Pulse rate/pulse.h"

#define HEART_RATE_TASK_SIZE	500
#define HEART_RATE_PRIORITY (configMAX_PRIORITIES - 1)

void HeartRateTask(void *pvParameters);





#endif /* HEARTRATE_TASK_H_ */