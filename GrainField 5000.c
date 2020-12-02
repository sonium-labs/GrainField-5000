////////////////////////////////////////////////////////////////
/* Lab 6 - Part 1
 * Author: Daniel Dole-Muinos
 * Description: SRAM Audio Mixing using Codec*/
////////////////////////////////////////////////////////////////
//Includes//////////////////////////////////////////////////////
#include <F28x_Project.h>
#include "SRAM_Driver.h"
#include "AIC23.h"
#include <math.h>

#define     delay_200ms()   for(volatile long j=0; j < 1400000; j++)

#define     SAMP_RATE       48000.0

void InitMcBSPb();
void InitSPIA();
void InitAIC23();
void InitAIC23_16bit();
void IO_GPIO_Init();

interrupt void McBSP_RX_ISR(void);

volatile Uint16 flag = 0, mixenable = 0;

volatile int16 LS,RS,throwaway,samp_ready,play_samp,play_ready,sample,prevsample,mixsample;
volatile Uint32 sptr = 0, pptr = 0, pptr_start = 10000, pptr_end = 50000;
volatile float fade_factor = 0.0f;

int main(){
    volatile float fade_time = 0.1f;
    volatile float fade_delta = 1.0f/(fade_time*SAMP_RATE);
    volatile float fade_in_mark  = pptr_start + fade_time * SAMP_RATE;
    volatile float fade_out_mark = pptr_end - pptr_start - fade_time * SAMP_RATE;

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

    SRAM_GPIO_Init();
    IO_GPIO_Init();
    SPIB_Init();
    InitSPIA();
    InitAIC23();
    InitMcBSPb();

    SRAM_CLEAR();

    while(1) {
        if(play_ready) {
            play_samp = SRAM_READ(pptr);

            fade_delta      = 1.0f/(fade_time*SAMP_RATE);
            fade_in_mark    = pptr_start + fade_time * SAMP_RATE;
            fade_out_mark   = pptr_end - fade_time * SAMP_RATE;

            if(pptr < fade_in_mark)
            {
                fade_factor += fade_delta;
            }

            if(pptr > fade_out_mark)
            {
                fade_factor -= fade_delta;
            }

            play_samp = (int16)(play_samp * fade_factor);

            //HANNING BROKE SOUND :(
            //hannoutput = 0.5f * (1 - cosf(2.0f * M_PI * (pptr - pptr_start) / (((pptr_end-pptr_start) - 1))));
            //play_samp *= hannoutput;
            play_ready = 0;
        }

        //BUTTON CHECKS
        //DIP Switch Check//////////////////////////////////
        if(GpioDataRegs.GPADAT.bit.GPIO0 == 0) {
            mixenable = 1;
            GpioDataRegs.GPADAT.bit.GPIO7 = 0;

        }
        else {
            mixenable = 0;
            GpioDataRegs.GPADAT.bit.GPIO7 = 1;
        }

        //Check center button (Recording)///////////////////
        if(GpioDataRegs.GPADAT.bit.GPIO15 == 0) {
            delay_200ms();
            if(GpioDataRegs.GPADAT.bit.GPIO16 == 0) {
                // Double press found!
                // Turn on all LEDS
                GpioDataRegs.GPADAT.all &= 0xF00F;
                // Clear SRAM
                SRAM_CLEAR();
                // Turn off all LEDS
                GpioDataRegs.GPADAT.all |= 0x0FF0;
            }
            else if(flag == 1) {
                flag = 0;
                GpioDataRegs.GPADAT.bit.GPIO4 = 1; //Turn LED[0] off
            }
            else {
                flag = 1;
                sptr = 0;
                GpioDataRegs.GPADAT.bit.GPIO4 = 0; //Turn LED[0] on
            }
            delay_200ms();
            delay_200ms();
        }
        //Check left button (Playback)///////////////////////
        else if(GpioDataRegs.GPADAT.bit.GPIO16 == 0) {
            delay_200ms();
            if(GpioDataRegs.GPADAT.bit.GPIO15 == 0) {
                // Double press found!
                // Turn on all LEDS
                GpioDataRegs.GPADAT.all &= 0xF00F;
                // Clear SRAM
                SRAM_CLEAR();
                // Turn off all LEDS
                GpioDataRegs.GPADAT.all |= 0x0FF0;
            }
            else if(flag == 3) {
                flag = 0;
                GpioDataRegs.GPADAT.bit.GPIO5 = 1; //Turn LED[0] off
            }
            else {
                GpioDataRegs.GPADAT.bit.GPIO5 = 0; //Turn LED[1] on
                // set ptr to beginning
                if(flag == 0) {
                    pptr = pptr_start;
                    // begin playback
                    flag = 3;
                }
            }
            delay_200ms();
            delay_200ms();
        }
        //SRAM OPERATION/////////////////////////////////////
        if(samp_ready && flag == 1) {
            if(mixenable) {
                sample = (LS+RS)/2;
                prevsample = SRAM_READ(sptr);
                mixsample = (sample + prevsample) / 2;
                SRAM_WRITE(sptr, mixsample);
            }
            else {
                sample = (LS+RS)/2;
                SRAM_WRITE(sptr, sample);
            }
            samp_ready = 0;
        }
    }
}

 interrupt void McBSP_RX_ISR(void) {
     // Check ISR rate w DAD
     //GpioDataRegs.GPATOGGLE.bit.GPIO11 = 1;

     if(sptr >= 0x40000) {
         if(flag == 1) {
             GpioDataRegs.GPADAT.bit.GPIO4 = 1; //Turn LED[0] off
             flag = 0;   //end recording
             sptr = 0;
         }
     }
     if(pptr >= 0x40000 || pptr >= pptr_end)
     {
         if(flag == 3) {
             GpioDataRegs.GPADAT.bit.GPIO5 = 1; //Turn LED[1] off
             sptr = 0;
             pptr = pptr_start;
             fade_factor = 0.0f;
         }
     }

     // Case: Do nothing
     if(flag == 0) {
             throwaway = McbspbRegs.DRR2.all;
             throwaway = McbspbRegs.DRR1.all;

             McbspbRegs.DXR2.all = 0x0000;
             McbspbRegs.DXR1.all = 0x0000;
     }
     // Case: Record sound into buffer ///////////////////////////////////
     else if(flag == 1) {
             LS = McbspbRegs.DRR2.all;
             RS = McbspbRegs.DRR1.all;
             samp_ready = 1;
             sptr++;
     }
     // Case: Playback sound /////////////////////////////////////////////
     else if(flag == 3) {
             throwaway = McbspbRegs.DRR2.all;
             throwaway = McbspbRegs.DRR1.all;

             if(play_ready == 0) {
                 McbspbRegs.DXR2.all = play_samp;
                 McbspbRegs.DXR1.all = play_samp;
                 play_ready = 1;
                 pptr++;
             }
     }
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
