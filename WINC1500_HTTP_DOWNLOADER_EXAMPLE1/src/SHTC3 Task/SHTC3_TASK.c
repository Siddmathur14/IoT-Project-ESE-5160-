/*
 * SHTC3_TASK.c
 *
 * Created: 5/3/2023 4:32:06 PM
 *  Author: siddm
 */ 

#include "SHTC3 Task/SHTC3_TASK.h"
#include "I2cDriver/I2cDriver.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "TEMPERATURE/SHTC3.h"
#include "Pulse rate/pulse.h"
#include "DRV2605\DRV2605.h"

/**
 * @fn		    void SHTC3Task(void *pvParameters)
 * @brief       Runs temperature measurement task in parallel
 * @details 	Runs temperature measurement task in parallel

 * @return		Returns nothing.
 * @note
 */

void SHTC3Task(void *pvParameters)	
{   
	uint8_t buffer[64];
	uint8_t buffer1[64];
	SerialConsoleWriteString("SHTC3");
	int32_t res = SHTC3_Init();  //(buffer, 2);
	snprintf((char *) buffer1, sizeof(buffer1), "Status of wakeup : %d\r\n", res);
	SerialConsoleWriteString(buffer1);
		
	int32_t temperature = 0;
	
		
	while(1)
	{    int task;
		// port_pin_toggle_output_level(PIN_PA23);
		res = SHTC3_Read_Data(buffer,2);
		// snprintf((char *) buffer1, sizeof(buffer1), "Status of read cmd : %d\r\n", res);
		// SerialConsoleWriteString(buffer1);
		temperature = (-45 + (((buffer[0] << 8 | buffer[1]) * 175) / 65536 ) );
		// snprintf((char *) buffer1, sizeof(buffer1), "Temp : %d\r\n",temperature );
		// SerialConsoleWriteString(buffer1);
		if(temperature>50){
			 int state= DRV2605_INIT();
			 state= DRV2605_MODE_SELECT();
			 state= DRV2605_DRIVE();
			 WifiAddTempDataToQueue(&temperature);
		}
		WifiAddTempDataToQueue(&temperature);
		vTaskDelay(1000);
	
}

}
			  
