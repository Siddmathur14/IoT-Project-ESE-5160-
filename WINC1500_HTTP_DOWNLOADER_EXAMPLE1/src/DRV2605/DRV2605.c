/*
 * DRV2605.c
 *
 * Created: 4/22/2023 4:13:56 PM
 *  Author: sahil
 */ 
#include "I2cDriver\I2cDriver.h"
#include "stdint.h"
#include "SerialConsole.h"
#include "stdint.h"
#include "DRV2605.h"
I2C_Data DRV2605Data;
uint8_t msgOutImu[64];
/******************************************************************************
 * Functions
 ******************************************************************************/

/**
 * @fn		 int32_t DRV2605_INIT(void)
 * @brief	 Initialize DRV2605
 * @details  Fetches product ID

 * @return	Returns 0 if no errors.
 * @note
 */
int32_t DRV2605_INIT(void)
{
		uint8_t buffer1[64];
		struct port_config config_port_pin;//Define structure needed to configure a pin
		port_get_config_defaults(&config_port_pin); //Initialize structure with default configurations.
		config_port_pin.direction  = PORT_PIN_DIR_OUTPUT;
		port_pin_set_config(PIN_PA20, &config_port_pin);
		port_pin_set_output_level(PIN_PA20, LED_0_INACTIVE); // make enable pin high

		//Get a read from the status register
		//Want this to Read 0xE0, any other value than 0 and you have tripped the over-current protection=wrong motor

		uint16_t reg=STATUS_REG ;
		uint8_t buffer[2];
		DRV2605Data.address = DRV2605_ADDR;
		DRV2605Data.msgOut = &reg;
		DRV2605Data.lenOut = 1;
		DRV2605Data.msgIn = buffer;
		DRV2605Data.lenIn = sizeof(buffer);

		int error = I2cReadDataWait(&DRV2605Data, 0xff, 0xff);
		//snprintf((char *) buffer1, sizeof(buffer1), "Status Register 0x %x\r\n", buffer[0]);
		//	SerialConsoleWriteString(buffer1);
		return error;
}
/**
 * @fn		 int32_t DRV2605_MODE_SELECT(void)
 * @brief	 Sets up the motor driver
 * @details  Sets up the motor driver

 * @return	Returns 0 if no errors.
 * @note
 */
int32_t DRV2605_MODE_SELECT(void)
{
	uint8_t buffer1[64];

	// out of standby
	uint16_t reg=DRV2605_REG_MODE;
	msgOutImu[0]=reg;

	msgOutImu[1]=MODE;
	uint8_t buffer[2];
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;
	
	int error = I2cWriteDataWait(&DRV2605Data, 0xff);
	
	
	// no real-time-playback
	msgOutImu[0]=DRV2605_REG_RTPIN ;
	msgOutImu[1]=RTPIN;	
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;
	error = I2cWriteDataWait(&DRV2605Data, 0xff);
	
	
	// strong click	
	msgOutImu[0]=DRV2605_REG_WAVESEQ1;
	msgOutImu[1]=WAVE_START;

	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;
	error = I2cWriteDataWait(&DRV2605Data, 0xff);
	
	

	// end sequence
	msgOutImu[0]=DRV2605_REG_WAVESEQ2;
	msgOutImu[1]=WAVE_STOP;
	
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;

	error = I2cWriteDataWait(&DRV2605Data, 0xff);
	
	
	// no overdrive 
	msgOutImu[0]=DRV2605_REG_OVERDRIVE;
	msgOutImu[1]=NO_DRIVE;
	
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;

	error = I2cWriteDataWait(&DRV2605Data, 0xff);

	 
	 
	msgOutImu[0]=DRV2605_REG_SUSTAINPOS;	 
	msgOutImu[1]=SUSTAINPOS;
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;

	 error = I2cWriteDataWait(&DRV2605Data, 0xff);
	 
	 
	 
	msgOutImu[0]=DRV2605_REG_SUSTAINNEG;	 
	msgOutImu[1]=SUSTAINNEG;
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;

	error = I2cWriteDataWait(&DRV2605Data, 0xff);
	 
	
    msgOutImu[0]=DRV2605_REG_BREAK ;	 
	msgOutImu[1]=BREAK;
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;

	error = I2cWriteDataWait(&DRV2605Data, 0xff);
	 
	 
	
    msgOutImu[0]= DRV2605_REG_AUDIOMAX ;	 
	msgOutImu[1]=AUDIOMAX;
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;

	 error = I2cWriteDataWait(&DRV2605Data, 0xff);


     // Reg Feedback 
	 reg=DRV2605_REG_FEEDBACK ;
	 DRV2605Data.address = DRV2605_ADDR;
	 DRV2605Data.msgOut = &reg;
	 DRV2605Data.lenOut = 1;
	 DRV2605Data.msgIn = buffer;
	 DRV2605Data.lenIn = sizeof(buffer);

	 error = I2cReadDataWait(&DRV2605Data, 0xff, 0xff);	 
	 msgOutImu[0]= DRV2605_REG_FEEDBACK ;	 
	 msgOutImu[1]= buffer[0] & 0x7F;

	 DRV2605Data.address = DRV2605_ADDR;
	 DRV2605Data.msgOut = &msgOutImu;
	 DRV2605Data.lenOut = 2;
	 DRV2605Data.msgIn = buffer;
	 DRV2605Data.lenIn = 0;

	 error = I2cWriteDataWait(&DRV2605Data, 0xff);
	
	
	// Reg control 
    reg=DRV2605_REG_CONTROL3;

	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &reg;
	DRV2605Data.lenOut = 1;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = sizeof(buffer);

	 error = I2cReadDataWait(&DRV2605Data, 0xff, 0xff);	 
	 msgOutImu[0]= DRV2605_REG_CONTROL3;	 
	 msgOutImu[1]= buffer[0]  | 0x20;
	 DRV2605Data.address = DRV2605_ADDR;
	 DRV2605Data.msgOut = &msgOutImu;
	 DRV2605Data.lenOut = 2;
	 DRV2605Data.msgIn = buffer;
	 DRV2605Data.lenIn = 0;

	 error = I2cWriteDataWait(&DRV2605Data, 0xff);	 
	 
	 
	 
	 //  writeRegister8(DRV2605_REG_MODE, mode);
	 // Mode Select
	 msgOutImu[0]= DRV2605_REG_MODE ;	 
	 msgOutImu[1]=MODE;
	 DRV2605Data.address = DRV2605_ADDR;
	 DRV2605Data.msgOut = &msgOutImu;
	 DRV2605Data.lenOut = 2;
	 DRV2605Data.msgIn = buffer;
	 DRV2605Data.lenIn = 0;

	 error = I2cWriteDataWait(&DRV2605Data, 0xff);	
	 
	 
	 /// set waveform end waveform   writeRegister8(DRV2605_REG_WAVESEQ1 + slot, w);
	 
	 
	// set waveform end waveform
	msgOutImu[0]= DRV2605_REG_WAVESEQ1 ;	 
	msgOutImu[1]=WAVE_START;
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;

	error = I2cWriteDataWait(&DRV2605Data, 0xff);	
	 
	 
	 
	msgOutImu[0]= DRV2605_REG_WAVESEQ2 ;	 
	msgOutImu[1]=WAVE_STOP;
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;

	error = I2cWriteDataWait(&DRV2605Data, 0xff);	
	 
	 
	return error;
}
/*
int32_t DRV2605_MOTOR_SELECT(void)
{
	uint8_t buffer1[64];
	uint16_t reg=0x1A ;
	msgOutImu[0]=reg;
msgOutImu[1]=0x0A;
uint8_t buffer[2];
DRV2605Data.address = DRV2605_ADDR;
DRV2605Data.msgOut = &msgOutImu;
DRV2605Data.lenOut = 2;
DRV2605Data.msgIn = buffer;
DRV2605Data.lenIn = 0;

int error = I2cWriteDataWait(&DRV2605Data,  0xff);

return error;
}
int32_t DRV2605_LIB_SELECT(void)
{
	uint8_t buffer1[64];
	uint16_t reg=0x03 ;
	msgOutImu[0]=reg;
	msgOutImu[1]=7;
	uint8_t buffer[2];
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;

	int error = I2cWriteDataWait(&DRV2605Data,  0xff);

	return error;
}
*/
/**
 * @fn		 int32_t DRV2605_DRIVE(void)
 * @brief	 Drive the motor
 * @details  Drive the motor

 * @return	Returns 0 if no errors.
 * @note
 */
int32_t DRV2605_DRIVE(void)
{
		int error;
		for(uint32_t i=0; i<1000; i++){
			error =DRV2605_GO(0x01);
		}
		//vTaskDelay(5000);
	    //error =DRV2605_GO(0x00);
		//vTaskDelay(5000);
		 return error;

	}
	/*
int32_t DRV2605_WAVEFORM(void){
	uint8_t buffer1[64];
	uint16_t reg=0x04 ;
	msgOutImu[0]=reg;
	msgOutImu[1]=0x00;
	uint8_t buffer[2];
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;

	int error = I2cWriteDataWait(&DRV2605Data, 0xff);

	return error;
}
int32_t DRV2605_Vibrations(void){
	uint8_t buffer1[64];
	uint16_t reg=0x0B ;
	msgOutImu[0]=reg;
msgOutImu[2]=0xFF;
msgOutImu[3]=0x60;
	msgOutImu[1]=0x80;
	uint8_t buffer[2];
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 4;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;

	int error = I2cWriteDataWait(&DRV2605Data, 0xff);

	return error;
}*/
	/**
 * @fn		 int32_t DRV2605_GO(uint16_t state)
 * @brief	 Fill the GO register with 0x01
 * @details  Fill the GO register with 0x01

 * @return	Returns 0 if no errors.
 * @note
 */
int32_t DRV2605_GO(uint16_t state){
	uint8_t buffer1[64];
	uint16_t reg=DRV2605_REG_GO ;
	msgOutImu[0]=reg;
	msgOutImu[1]=state;
	uint8_t buffer[2];
	DRV2605Data.address = DRV2605_ADDR;
	DRV2605Data.msgOut = &msgOutImu;
	DRV2605Data.lenOut = 2;
	DRV2605Data.msgIn = buffer;
	DRV2605Data.lenIn = 0;

	int error = I2cWriteDataWait(&DRV2605Data, 0xff);

	return error;
}









