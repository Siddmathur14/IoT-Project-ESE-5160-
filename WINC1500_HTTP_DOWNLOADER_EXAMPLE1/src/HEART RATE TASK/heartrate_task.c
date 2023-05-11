/*
 * heartrate_task.c
 *
 * Created: 5/4/2023 7:31:02 PM
 *  Author: siddm
 */ 
#include "SerialConsole.h"
#include "heartrate_task.h"
#include "adc.h"
#include "Pulse rate/pulse.h"
#include "SerialConsole.h"
uint8_t samplesUntilReport;
const uint8_t SAMPLES_PER_SERIAL_SAMPLE = 10;
static const unsigned long MICROS_PER_READ = (2 * 1000L);
volatile unsigned long NextSampleMicros; // Desired time to sample next.
const int THRESHOLD = 550;
uint8_t buffer1[64];
int counter=0;
volatile bool SawNewSample; // "A sample has arrived from the ISR"
enum status_code status;
bool Paused;
volatile int BPM;                // int that holds raw Analog in 0. updated every call to readSensor()
volatile int Signal;             // holds the latest incoming raw data (0..1023)
volatile int IBI;                // int that holds the time interval (ms) between beats! Must be seeded!
volatile bool Pulse;          // "True" when User's live heartbeat is detected. "False" when not a "live beat".
volatile bool QS;             // The start of beat has been detected and not read by the Sketch.
volatile int FadeLevel;          // brightness of the FadePin, in scaled PWM units. See FADE_SCALE
volatile int threshSetting;      // used to seed and reset the thresh variable
volatile int amp;                         // used to hold amplitude of pulse waveform, seeded (sample value)
volatile unsigned long lastBeatTime;      // used to find IBI. Time (sampleCounter) of the previous detected beat start.

// Variables internal to the pulse detection algorithm.
// Not volatile because we use them only internally to the pulse detection.
unsigned long sampleIntervalMs;  // expected time between calls to readSensor(), in milliseconds.
int rate[10];                    // array to hold last ten IBI values (ms)
unsigned long sampleCounter;     // used to determine pulse timing. Milliseconds since we started.
int N;                           // used to monitor duration between beats
int P;                           // used to find peak in pulse wave, seeded (sample value)
int T;                           // used to find trough in pulse wave, seeded (sample value)
int thresh;                      // used to find instant moment of heart beat, seeded (sample value)
bool firstBeat;               // used to seed rate array so we startup with reasonable BPM
bool secondBeat;              // used to seed rate array so we startup with reasonable BPM

/**
 * @fn		    void configure_adc(void)
 * @brief       Configure ADC
 * @details 	Configure ADC

 * @return		Returns nothing.
 * @note
 */
void configure_adc(void)
{

	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);
	#if !(SAML21)
	#if !(SAMC21)
	config_adc.gain_factor     = ADC_GAIN_FACTOR_DIV2;
	#endif
	config_adc.resolution      = ADC_RESOLUTION_10BIT;
	#endif
	
	config_adc.clock_source = GCLK_GENERATOR_1;

	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
	config_adc.reference       = ADC_REFERENCE_INTVCC1;
	config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN0;
	config_adc.freerunning     = true;
	config_adc.left_adjust     = false;
	#if (SAMC21)
	adc_init(&adc_instance, ADC, &config_adc);
	#else
	adc_init(&adc_instance, ADC, &config_adc);
	#endif
	adc_enable(&adc_instance);
}
/**
 * @fn		    void HeartRateTask(void *pvParameters)
 * @brief       Runs heart rate measurement task in parallel
 * @details 	Runs heart rate measurement task in parallel

 * @return		Returns nothing.
 * @note
 */
void HeartRateTask(void *pvParameters)
{
	//vTaskDelay(5000);

	uint8_t rate=1000;
	uint8_t buffer1[64];
	samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;
	configure_adc();

	if (!begin()) {
	//SerialConsoleWriteString("Can't Initialize Pulserate Sensor\r\n");
	}
	
	while(1)
	{    
		 if (sawNewSample()) {
    /*
       Every so often, send the latest Sample.
       We don't print every sample, because our baud rate
       won't support that much I/O.
    */
				//	SerialConsoleWriteString("Saw New Sample \r\n");
			if (--samplesUntilReport == (uint8_t) 0) {
				samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;
				//SerialConsoleWriteString("samplesUntilReport \r\n");
				outputSample();
			 /*
			At about the beginning of every heartbeat,
			report the heart rate and inter-beat-interval.
			*/

			}	
		}
	
		vTaskDelay(10);
	}
}

/**
 * @fn		    bool begin(void)
 * @brief       Initialize heart rate sensor
 * @details 	Initialize heart rate sensor

 * @return		Returns nothing.
 * @note
 */
bool begin(void){
	 NextSampleMicros = (xTaskGetTickCount()*configTICK_RATE_HZ)+ MICROS_PER_READ;
	 sampleIntervalMs = MICROS_PER_READ / 1000;
	 SawNewSample = false;
	 Paused = false;
	 return true;
}

/**
 * @fn		    bool sawNewSample(void)
 * @brief       Checks if there's a new sample
 * @details 	Checks if there's a new sample

 * @return		Returns if there's a new sample or not (bool).
 * @note
 */
bool sawNewSample(void) {
	if(!Paused){
		unsigned long nowMicros =  xTaskGetTickCount()*configTICK_RATE_HZ ;// micros();
		if ((long) (NextSampleMicros - nowMicros) > 0L) {
			return false;  // not time yet.
		}
		NextSampleMicros = nowMicros + MICROS_PER_READ;
		onSampleTime();
		return true;
	}
}

/**
 * @fn		    void  outputSample(void)
 * @brief       Adds the sample new sample queue
 * @details 	Adds the sample new sample queue

 * @return		Returns nothing.
 * @note
 */
void  outputSample(void){
		//SerialConsoleWriteString("Beat Detected \r\n");
		if(BPM>120){
			BPM=120;
		}
		if(BPM<30){
			BPM=0;
		}
			//snprintf((char *) buffer1, sizeof(buffer1), "BPM: %d, IBI: %d, Signal: %d \r\n", BPM,IBI,Signal);
// 			snprintf((char *) buffer1, sizeof(buffer1), "%d,%d,%d,%d \r\n", BPM,IBI,Signal,xTaskGetTickCount()*configTICK_RATE_HZ );
// 			SerialConsoleWriteString(buffer1);
			//counter++;
//			if(counter++>10){
		WifiAddHeartrateDataToQueue(&BPM);
// 			counter=0;
// 			}
//		vTaskDelay(500);			
}

/**
 * @fn		    void onSampleTime(void) 
 * @brief       Sample time
 * @details 	Sample time

 * @return		Returns nothing.
 * @note
 */
void onSampleTime(void) {
  // Typically called from the ISR at 500Hz
  // digitalWrite(timingPin,HIGH); // optionally connect timingPin to oscilloscope to time algorithm run time
  /*
     Read the voltage from each PulseSensor.
     We do this separately from processing the samples
     to minimize jitter in acquiring the signal.
  */

	readNextSample();

  // Process those samples.

	processLatestSample();

  // Set the flag that says we've read a sample since the Sketch checked.
	SawNewSample = true;

  // digitalWrite(timingPin,LOW); // optionally connect timingPin to oscilloscope to time algorithm run time
 }

/**
 * @fn		    void readNextSample(void)
 * @brief       Read Next Sample from sensor 
 * @details 	Read Next Sample from sensor 

 * @return		Returns nothing.
 * @note
 */
void readNextSample(void) {
	
	// We assume assigning to an int is atomic.
	 adc_start_conversion(&adc_instance);

	do {
		
		status = adc_read(&adc_instance, &pulse);
		
	} while (status == STATUS_BUSY);
	Signal=pulse;
	
}

/**
 * @fn		    bool sawStartOfBeat(void) 
 * @brief       Checks if there's a start of Beat
 * @details 	Checks if there's a start of Beat

 * @return		Returns boolean value.
 * @note
 */
bool sawStartOfBeat(void) {
	// Disable interrupts to avoid a race with the ISR.

	bool started = QS;
	QS = false;

	return started;
}

/**
 * @fn		    void processLatestSample(void)
 * @brief       Process new sample
 * @details 	Process new sample

 * @return		Returns nothing.
 * @note
 */
void processLatestSample(void) {
	// Serial.println(threshSetting);
	// Serial.print('\t');
	// Serial.println(thresh);
	sampleCounter += sampleIntervalMs;         // keep track of the time in mS with this variable
	N = sampleCounter - lastBeatTime;      // monitor the time since the last beat to avoid noise

	// Fade the Fading LED


	//  find the peak and trough of the pulse wave
	if (Signal < THRESHOLD && N > (IBI / 5) * 3) { // avoid dichrotic noise by waiting 3/5 of last IBI
		if (Signal < T) {                        // T is the trough
			T = Signal;                            // keep track of lowest point in pulse wave
			//SerialConsoleWriteString("(Signal < THRESHOLD && N > (IBI / 5) * 3) \r\n");
			
		}
	}

	if (Signal > THRESHOLD && Signal > P) {       // thresh condition helps avoid noise
		P = Signal;                              // P is the peak
	//SerialConsoleWriteString("(Signal > THRESHOLD && Signal > P) \r\n");
	}                                          // keep track of highest point in pulse wave

	//  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
	// signal surges up in value every time there is a pulse
	if (N > 250) {                             // avoid high frequency noise
		if ( (Signal > THRESHOLD) && (Pulse == false) && (N > (IBI / 5) * 3) ) {
			Pulse = true;                          // set the Pulse flag when we think there is a pulse
			IBI = sampleCounter - lastBeatTime;    // measure time between beats in mS
			lastBeatTime = sampleCounter;          // keep track of time for next pulse

			if (secondBeat) {                      // if this is the second beat, if secondBeat == TRUE
				secondBeat = false;                  // clear secondBeat flag
				for (int i = 0; i <= 9; i++) {       // seed the running total to get a realisitic BPM at startup
					rate[i] = IBI;
				}
			}

			if (firstBeat) {                       // if it's the first time we found a beat, if firstBeat == TRUE
				firstBeat = false;                   // clear firstBeat flag
				secondBeat = true;                   // set the second beat flag
				// IBI value is unreliable so discard it
				return;
			}

		//SerialConsoleWriteString("(Signal > THRESHOLD) && (Pulse == false) && (N > (IBI / 5) * 3) \r\n");

			// keep a running total of the last 10 IBI values
			uint16_t runningTotal = 0;                  // clear the runningTotal variable

			for (int i = 0; i <= 8; i++) {          // shift data in the rate array
				rate[i] = rate[i + 1];                // and drop the oldest IBI value
				runningTotal += rate[i];              // add up the 9 oldest IBI values
			}

			rate[9] = IBI;                          // add the latest IBI to the rate array
			runningTotal += rate[9];                // add the latest IBI to runningTotal
			runningTotal /= 10;                     // average the last 10 IBI values
			BPM = 60000 / runningTotal;             // how many beats can fit into a minute? that's BPM!
			QS = true;                              // set Quantified Self flag (we detected a beat)
		}
	}

	if (Signal < THRESHOLD && Pulse == true) {  // when the values are going down, the beat is over
		Pulse = false;                         // reset the Pulse flag so we can do it again
		amp = P - T;                           // get amplitude of the pulse wave
		thresh = amp / 2 + T;                  // set thresh at 50% of the amplitude
		P = thresh;                            // reset these for next time
		T = thresh;
		//	SerialConsoleWriteString("(Signal < THRESHOLD && Pulse == true) \r\n");

	}

	if (N > 2500) {                          // if 2.5 seconds go by without a beat
		thresh = THRESHOLD;                // set thresh default
		P = 512;                               // set P default
		T = 512;                               // set T default
		lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date
		firstBeat = true;                      // set these to avoid noise
		secondBeat = false;                    // when we get the heartbeat back
		QS = false;
		BPM = 0;
		IBI = 600;                  // 600ms per beat = 100 Beats Per Minute (BPM)
		Pulse = false;
		amp = 100;                  // beat amplitude 1/10 of input range.
		//SerialConsoleWriteString(" (N > 2500) \r\n");
	}
}