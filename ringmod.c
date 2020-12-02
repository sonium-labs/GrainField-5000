////////////////////////////////////////////////////////////////
/* Lab 6 - Part 4
 * Author: Daniel Dole-Muinos
 * Description: Real-Time Reverb & Echo*/
////////////////////////////////////////////////////////////////
//Includes//////////////////////////////////////////////////////
#include <F28x_Project.h>
#include "AIC23.h"
#include <math.h>
#include <float.h>

#define delay_200ms() for(volatile long j=0; j < 1400000; j++)

void InitMcBSPb();
void InitSPIA();
void InitAIC23();
void InitAIC23_16bit();
void IO_GPIO_Init();
float ringmod();

interrupt void McBSP_RX_ISR(void);

#define  SAMP_RATE 48000.0
#define  R_DELTA   (1.0/SAMP_RATE)

volatile Uint16 flag = 0;
volatile int16 LS,RS,sample,output,samp_ready;
volatile float gain = 0.5, ring_time = 0,ring_factor = 0;
volatile int32 sptr = 0;
volatile float r_freq = 1000;

int main(){

    //Local vars
    float r_factor = 0; //holds ringmod_factor

    InitSysCtrl();      // Set SYSCLK to 200 MHz, disables watchdog timer, turns on peripheral clocks

    DINT;               // Disable CPU interrupts on startup

    // Init PIE
    InitPieCtrl();      // Initialize PIE -> disable PIE and clear all PIE registers
    IER = 0x0000;       // Clear CPU interrupt register
    IFR = 0x0000;       // Clear CPU interrupt flag register
    InitPieVectTable(); // Initialize PIE vector table to known state

    EALLOW;             // EALLOW for rest of program (unless later function disables it)

    PieVectTable.MCBSPB_RX_INT = &McBSP_RX_ISR;
    IER |= M_INT6;
    PieCtrlRegs.PIEIER6.bit.INTx7 = 1;          //Enable McBSPBRx interrupt
    EnableInterrupts();

    IO_GPIO_Init();
    InitSPIA();
    InitAIC23();
    InitMcBSPb();

    while(1) {
        sptr &= 0x3FFFF;    //circular buffer implementation

        //BUTTON CHECK
        //Center button PB1
        if(GpioDataRegs.GPADAT.bit.GPIO15 == 0) {
            r_freq+=500;
            delay_200ms();
            delay_200ms();
        }
        //Left button PB2
        else if (GpioDataRegs.GPADAT.bit.GPIO16 == 0) {
            if(r_freq - 500 >= 0) {
                r_freq-=500;
            }
            delay_200ms();
            delay_200ms();
        }
        //SAMPLE OPERATION
        //input  -> sample
        //output -> output
        if(samp_ready) {
            sample = (LS+RS)/2;
            //Debug
            //output = sample;
            //output = (int16)(1000.0 * ringmod());
            output = (int16)(((1-gain) * sample) + (gain * sample * ringmod()));
            //Acknowledge
            samp_ready = 0;
        }
    }
}

//Usage:    returns ring_factor for a given frequency
//Input:    desired frequency
//Output:   ringmod_factor
float ringmod() {
    //y(t) = gain * sin(2 * pi * Hz * time + phase)
    //gain is controlled by main, phase is 0
    //time based on delta which is based on sampling freq
    ring_factor = -99;
    //Ensure time isn't out of bounds
    if(ring_time >= 5) {
        ring_time = 0;
    }
    else {
        //ring_time += 1;
        ring_time += R_DELTA;
    }

    ring_factor = 0.5 * sin(2 * M_PI * r_freq * ring_time);
    return ring_factor;
}

interrupt void McBSP_RX_ISR(void) {
    // Check ISR rate w DAD
    //GpioDataRegs.GPATOGGLE.bit.GPIO11 = 1;

    LS = McbspbRegs.DRR2.all;
    RS = McbspbRegs.DRR1.all;
    samp_ready = 1;

    McbspbRegs.DXR2.all = output;
    McbspbRegs.DXR1.all = output;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    return;
}

void IO_GPIO_Init() {
    EALLOW;

    // SW0 -> GPIO0
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 0;

    // PB0 -> GPIO14
    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;

    // PB1 -> GPIO15
    GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 0;

    // PB2 -> GPIO16
    GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;

    // LED0 -> GPIO4
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;
    GpioDataRegs.GPADAT.bit.GPIO4 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;

    // LED1 -> GPIO5
    GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;
    GpioDataRegs.GPADAT.bit.GPIO5 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;

    // LED2 -> GPIO6
    GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
    GpioDataRegs.GPADAT.bit.GPIO6 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;

    // LED3 -> GPIO7
    GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;
    GpioDataRegs.GPADAT.bit.GPIO7 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;

    // LED4 -> GPIO8
    GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;
    GpioDataRegs.GPADAT.bit.GPIO8 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;

    // LED5 -> GPIO9
    GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
    GpioDataRegs.GPADAT.bit.GPIO9 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;

    // LED6 -> GPIO10
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
    GpioDataRegs.GPADAT.bit.GPIO10 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;

    // LED7 -> GPIO11
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;
    GpioDataRegs.GPADAT.bit.GPIO11 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;

    return;
}
