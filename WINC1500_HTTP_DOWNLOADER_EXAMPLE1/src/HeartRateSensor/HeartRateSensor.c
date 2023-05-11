/*
 * HeartRateSensor.c
 *
 * Created: 4/30/2023 2:12:30 AM
 *  Author: sahil
 */ 
/*
Arduino-MAX30100 oximetry / heart rate integrated sensor library
Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "MAX30100_PulseOximeter.h"
#include "stdint.h"
#include "SerialConsole.h"
#define REPORTING_PERIOD_MS     1000

// PulseOximeter is the higher level interface to the sensor
// it offers:
//  * beat detection reporting
//  * heart rate calculation
//  * SpO2 (oxidation level) calculation
PulseOximeter pox;

uint32_t tsLastReport = 0;

// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
	SerialConsoleWriteString("Beat!");
	//Serial.println("Beat!");
}

void setup()
{

	SerialConsoleWriteString("Initializing pulse oximeter..");
	//Serial.print("Initializing pulse oximeter..");

	// Initialize the PulseOximeter instance
	// Failures are generally due to an improper I2C wiring, missing power supply
	// or wrong target chip
	if (!pox.begin()) {
		//Serial.println("FAILED");
		SerialConsoleWriteString("FAILED");
		
		for(;;);
		} else {
		//Serial.println("SUCCESS");
		SerialConsoleWriteString("SUCCESS");
	}

	// The default current for the IR LED is 50mA and it could be changed
	//   by uncommenting the following line. Check MAX30100_Registers.h for all the
	//   available options.
	// pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

	// Register a callback for the beat detection
	pox.setOnBeatDetectedCallback(onBeatDetected);


while(1)
{
	// Make sure to call update as fast as possible
	pox.update();
	uint8_t buffer1[64];
	// Asynchronously dump heart rate and oxidation levels to the serial
	// For both, a value of 0 means "invalid"
	if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
		
		snprintf((char *) buffer1, sizeof(buffer1), "Heart rate: %d bpm\r\n", pox.getHeartRate());
		SerialConsoleWriteString(buffer1);
		/*
		Serial.print("Heart rate:");
		Serial.print(pox.getHeartRate());
		Serial.print("bpm / SpO2:");
		Serial.print(pox.getSpO2());
		Serial.println("%");*/
		snprintf((char *) buffer1, sizeof(buffer1), " SpO2: %d %\r\n", pox.getSpO2());
		SerialConsoleWriteString(buffer1);
		tsLastReport = millis();
		break;
	}
}
	}