/*
 *  GrainField-5000.h
 *  Author: Daniel Dole-Muinos
 */

#ifndef GRAINFIELD_5000_H_
#define GRAINFIELD_5000_H_
////////////////////////////////////////////////////////////////
// Defines /////////////////////////////////////////////////////
#define     delay_200ms()       for(volatile long j=0; j < 1400000; j++)
#define     SAMP_RATE           48000.0
#define     RESOLUTION          0.000738435546875
#define     WEIGHTING           0.4
#define     DEADZONE            70
// ADC output value range
#define     ADC_MIN             0
#define     ADC_MAX             4095
// SRAM pointer range
#define     SAMP_MIN            10000
#define     SAMP_MAX            0x3fff0     // 262143
// Loop length range
#define     LEN_MIN             320
#define     LEN_MAX             24000.0     // 0.5 Hz
// rand() range
#define     RAND_MIN            0
#define     RAND_MAX            32767
// Start spray range
#define     START_SPRAY_MIN     0
#define     START_SPRAY_MAX     100000
// Length spray range
#define     LENGTH_SPRAY_MIN    0
#define     LENGTH_SPRAY_MAX    12000
// Loop Pointer Start
#define     PPTR_START          10000
// Loop Length
#define     PPTR_LENGTH         5000
// Loop Pointer End
#define     PPTR_END            15000
////////////////////////////////////////////////////////////////
// Initialization functions ////////////////////////////////////
void InitMcBSPb();
void InitSPIA();
void InitAIC23();
void IO_GPIO_Init();
void InitTimer1(void);
void InitAdca(void);
void InitAdcb(void);
////////////////////////////////////////////////////////////////
// Operation Functions /////////////////////////////////////////
void ButtonCheck(void);
void SwitchCheck(void);
void Record(void);
void Playback(void);
void Randomizer(void);
void UpdatePtrs(void);
void UpdateADC(void);
////////////////////////////////////////////////////////////////
// ISRs ////////////////////////////////////////////////////////
interrupt void Timer1_isr(void);
interrupt void McBSP_RX_ISR(void);
////////////////////////////////////////////////////////////////
#endif /* GRAINFIELD_5000_H_ */
