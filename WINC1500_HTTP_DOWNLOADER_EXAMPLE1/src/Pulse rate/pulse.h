/*
 * PulseRateSensor.h
 *
 * Created: 5/4/2023 10:23:10 AM
 *  Author: sahil
 */ 


#ifndef PULSERATESENSOR_H_
#define PULSERATESENSOR_H_


struct adc_module adc_instance;
uint16_t pulse;
uint16_t pulsemain(void);
bool begin(void);
bool sawNewSample(void);
void  outputSample(void);
void readNextSample(void) ;
void onSampleTime(void);
void processLatestSample(void);
bool sawStartOfBeat(void);
#endif /* PULSERATESENSOR_H_ */