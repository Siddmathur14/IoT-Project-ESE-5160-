/*
 * DRV2605.h
 *
 * Created: 4/22/2023 4:14:20 PM
 *  Author: sahil
 */ 


#ifndef DRV2605_H_
#define DRV2605_H_

#ifdef __cplusplus
extern "C" {
	#endif

#define DRV2605_ADDR 0x5A
#define STATUS_REG 0x00
#define DRV2605_REG_MODE 0x01
#define MODE 0x00
#define DRV2605_REG_RTPIN 0x02
#define RTPIN 0x00
#define DRV2605_REG_WAVESEQ1 0x04
#define DRV2605_REG_WAVESEQ2 0x05
#define WAVE_START 1
#define DRV2605_REG_OVERDRIVE 0x0D  ///< Overdrive time offset register
#define WAVE_STOP 0
#define NO_DRIVE 0
#define DRV2605_REG_SUSTAINPOS 0x0E ///< Sustain time offset, positive register
#define SUSTAINPOS 0 
#define DRV2605_REG_SUSTAINNEG 0x0F ///< Sustain time offset, negative register
#define SUSTAINNEG 0
#define DRV2605_REG_BREAK 0x10      ///< Brake time offset register
#define BREAK 0
#define DRV2605_REG_AUDIOMAX 0x13 ///< Audio-to-vibe maximum input level register
#define AUDIOMAX 0x64 ///< Audio-to-vibe maximum input level register
#define DRV2605_REG_FEEDBACK 0x1A ///< Feedback control register
#define DRV2605_REG_CONTROL3 0x1D ///< Control3 Register
#define DRV2605_REG_GO 0x0C         ///< Go register


int32_t DRV2605_INIT(void);
int32_t DRV2605_MODE_SELECT(void);
int32_t DRV2605_MOTOR_SELECT(void);
int32_t DRV2605_LIB_SELECT(void);
int32_t DRV2605_DRIVE(void);
int32_t DRV2605_WAVEFORM(void);
int32_t DRV2605_GO(uint16_t state);
int32_t DRV2605_Vibrations(void);
#ifdef __cplusplus
}
#endif
#endif /* DRV2605_H_ */