
  /******************************************************************************
  * @file    SHTC3.h
  * @author  Sahil Mangaonkar and Siddhant Mathur
  * @brief   SHTC3 driver file
  * @date    2023-04-20
  ******************************************************************************/
  
/******************************************************************************
 * Includes
 ******************************************************************************/

#include "SHTC3.h"
#include "i2c_master.h"
#include "i2c_master_interrupt.h"
//#include "I2cDriver\I2cDriver.h"
#include "stdint.h"

static int32_t temp_write(uint8_t *bufp, uint16_t len);

static int32_t temp_read(uint8_t *bufp, uint16_t len);
int SHTC3_SendI2cCommand(uint8_t *buf, uint8_t size);
int SHTC3_Read_Data(uint8_t *buf, uint8_t size);
int SHTC3_Init(void);
/*int SHTC3_Init(void)
{
    return SHTC3_SendI2cCommand(SHTC3_WAKEUP_CMD, sizeof(SHTC3_WAKEUP_CMD));
}

int SHTC3_SendI2cCommand(uint8_t *buf, uint8_t size)
{
    // Write WAKE command to the device address
    struct i2c_master_packet sensorPacketWrite;

    // Prepare to write
    sensorPacketWrite.address = SHTC3_ADDR;             // Address to send to
    sensorPacketWrite.data = (uint8_t *)buf;      // Requires a pointer to the data to send. Why dont we put a #define here?
    sensorPacketWrite.data_length = size;  // Length to send.

    return i2c_master_write_packet_job(&i2cSensorBusInstance, &sensorPacketWrite);
}
int SHTC3_Read_Data(uint8_t *buf, uint8_t size)
{
	// Write WAKE command to the device address
	struct i2c_master_packet sensorPacketWrite;

	// Prepare to write
	sensorPacketWrite.address = SHTC3_ADDR;             // Address to send to
	sensorPacketWrite.data = buf;      // Requires a pointer to the data to send. Why dont we put a #define here?
	sensorPacketWrite.data_length = size;  // Length to send.

	return i2c_master_write_packet_job(&i2cSensorBusInstance, &sensorPacketWrite);
}*/
I2C_Data SHTC3Data;
int SHTC3_Init(void)
{
     uint8_t cmd[] = {0x35, 0x17};

     seesawData.address = SHTC3_ADDR;
     seesawData.msgOut = (const uint8_t *) &cmd[0];
     seesawData.lenOut = sizeof(cmd);
     seesawData.lenIn = 0;
     int32_t error = I2cWriteDataWait(&seesawData, 100);
     return error;
}
/*
int SHTC3_SendI2cCommand(void)
{
	uint8_t cmd[] = {0x35, 0x17};

	seesawData.address = SHTC3_ADDR;
	seesawData.msgOut = (const uint8_t *) &cmd[0];
	seesawData.lenOut = sizeof(cmd);
	seesawData.lenIn = 0;
	int32_t error = I2cWriteDataWait(&seesawData, 100);
	return error;
}
*/
int32_t SHTC3_Read_Data(uint8_t *buffer, uint8_t count)
{
	
	uint8_t cmd[] = {0x78, 0x66};
	SHTC3Data.address = SHTC3_ADDR;
	SHTC3Data.msgOut = (const uint8_t*) &cmd[0];
	SHTC3Data.lenOut = sizeof(cmd);
	SHTC3Data.msgIn = buffer;
	SHTC3Data.lenIn = count;

	int error = I2cReadDataWait(&seesawData, 0, 100);

	if (ERROR_NONE != error) {
		SerialConsoleWriteString("Error reading Seesaw counts!/r/n");
	}
	return error;
}
/*
uint8_t msgOutImu[64]; ///<USE ME AS A BUFFER FOR platform_write and platform_read
uint8_t msgInImu[64]; ///<USE ME AS A BUFFER FOR platform_write and platform_read


I2C_Data imuData; ///<Use me as a structure to communicate with the IMU on platform_write and platform_read

/**************************************************************************//**
 * @fn			static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
 * @brief       Function to write data to a register
 * @details     Function to write data (bufp) to a register (reg)
				
 * @param[in]   handle IGNORE
 * @param[in]   reg Register to write to. In an I2C transaction, this gets sent first
 * @param[in]   bufp Pointer to the data to be sent
 * @param[in]   len Length of the data sent
 * @return      Returns what the function "I2cWriteDataWait" returns
 * @note        STUDENTS TO FILL  
*****************************************************************************/
/*int32_t temp_write(uint8_t *bufp,uint16_t len)
{
	//YOUR JOB: Fill out the structure "imuData" to send to the device
	//TIP: Use the array "msgOutImu" to copy the data to be sent. Remember that the position [0] of the array you send must be the register, and
	//starting from position [1] you can copy the data to be sent. Remember to adjust the length accordingly
	
	
	
	for (uint16_t i=0;i<len;i++){
		msgOutImu[i]=bufp[i];
	}
	imuData.address=SHTC3_ADDR;
	imuData.msgOut=&msgOutImu;
	imuData.lenOut=len+1;
	imuData.lenIn=0;
	
return I2cWriteDataWait(&imuData,portMAX_DELAY);

}*/

/**************************************************************************//**
 * @fn			static  int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
 * @brief       Function to read data from a register
 * @details     Function to read data (bufp) from a register (reg)
				
 * @param[in]   handle IGNORE
 * @param[in]   reg Register to read from. In an I2C transaction, this gets sent first
 * @param[out]   bufp Pointer to the data to write to (write what was read)
 * @param[in]   len Length of the data to be read
 * @return      Returns what the function "I2cReadDataWait" returns
 * @note        STUDENTS TO FILL  
*****************************************************************************/
/*int32_t temp_read(uint8_t *bufp, uint16_t len)
{
	//YOUR JOB: Fill out the structure "imuData" to send to the device
	//TIP: Check the structure "imuData" and notice that it has a msgOut and msgIn parameter. How do we fill this to our advantage?
	//IfIMU=true;

	imuData.address=SHTC3_ADDR;
	imuData.msgIn=bufp;
	imuData.lenOut=1;
	imuData.lenIn=len;
	return I2cReadDataWait(&imuData,50,portMAX_DELAY);


}*/