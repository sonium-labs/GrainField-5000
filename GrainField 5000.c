////////////////////////////////////////////////////////////////
/* Lab 10
 * Author: Daniel Dole-Muinos
 * Description: Sampler*/
////////////////////////////////////////////////////////////////
//Includes//////////////////////////////////////////////////////
#include <F28x_Project.h>
#include "SRAM_Driver.h"
#include "AIC23.h"
#include <math.h>
#include "LCD_Driver.h"

#define     delay_200ms()   for(volatile long j=0; j < 1400000; j++)

#define     SAMP_RATE       48000.0
#define     RESOLUTION      0.000738435546875
//ADC output value range
#define     ADC_MIN         0
#define     ADC_MAX         4095
//SRAM pointer range
#define     SAMP_MIN        10000
#define     SAMP_MAX        0x3ffff //262143
//Loop length range
#define     LEN_MIN         320
#define     LEN_MAX         24000.0   //0.25 Hz

void InitMcBSPb();
void InitSPIA();
void InitAIC23();
void InitAIC23_16bit();
void IO_GPIO_Init();

interrupt void Timer1_isr(void);
void InitTimer1(void);
void InitAdca(void);
void InitAdcb(void);

void ButtonCheck(void);
void SwitchCheck(void);
void Record(void);
void UpdateValues(void);

interrupt void McBSP_RX_ISR(void);

volatile Uint16 flag = 0, mix_enable = 0, rev_enable = 0, pptr_0_trig = 0, pptr_1_trig = 0;

volatile int16 LS,RS,throwaway,samp_ready,play_samp,play_samp_0,play_samp_1,play_ready,sample,prevsample,mixsample;
volatile Uint32 sptr = 0, pptr_0 = 0, pptr_1 = 0, pptr_start = 10000, pptr_length = 5000, pptr_end = 15000, pptr_half_0 = 12500, pptr_half_1 = 12500;
volatile float fade_factor_0 = 0.0f, fade_factor_1 = 0.0f, fade_delta_0 = 0.0f, fade_delta_1 = 0.0f;
//ADC:
volatile Uint16 AdcData0 = 0,AdcData1 = 0;
int decRes,ones,tenths,hundredths;

//LCD Commands:
Uint16  nextLine     = 0xC0;
Uint16  clear        = 0x01;
Uint16  home         = 0x02;

//DEBUG:
volatile Uint32 fade_count_pos_0 = 0, fade_count_neg_0 = 0,fade_count_pos_1 = 0, fade_count_neg_1 = 0;

int main(void){
     InitSysCtrl();      // Set SYSCLK to 200 MHz, disables watchdog timer, turns on peripheral clocks

    LCD_init();         // Initialize LCD output

    LCD_SEND_COMMAND(&clear);
    LCD_SEND_STRING("GRAINFIELD-5000");

    DINT;               // Disable CPU interrupts on startup

    // Init PIE
    InitPieCtrl();      // Initialize PIE -> disable PIE and clear all PIE registers
    IER = 0x0000;       // Clear CPU interrupt register
    IFR = 0x0000;       // Clear CPU interrupt flag register
    InitPieVectTable(); // Initialize PIE vector table to known state

    EALLOW;             // EALLOW for rest of program (unless later function disables it)

    InitTimer1();       // Initialize CPU timer 1
    InitAdca();         // Initialize ADC A channel 0
    InitAdcb();         // Initialize ADC B channel 3

    PieVectTable.MCBSPB_RX_INT = &McBSP_RX_ISR;
    IER |= M_INT6;
    PieCtrlRegs.PIEIER6.bit.INTx7 = 1;          //Enable McBSPBRx interrupt

    SRAM_GPIO_Init();
    IO_GPIO_Init();
    SPIB_Init();
    InitSPIA();
    InitAIC23();
    InitMcBSPb();

    SRAM_CLEAR();

    EnableInterrupts();

    LCD_SEND_COMMAND(&clear);
    LCD_SEND_STRING("    STANDBY");
    while(1) {
        //MODE CHECKS
        SwitchCheck();

        if(play_ready && flag == 3) {
            //Reverse mode (playback starts from pptr_end to pptr_start)
            if(rev_enable) {
//                if(pptr <= pptr_start && flag == 3)
//                {
//                    //GpioDataRegs.GPADAT.bit.GPIO5 = 1; //Turn LED[1] off
//                    pptr = pptr_end;
//                    fade_factor = 0.0f;
//                    //DEBUG:
//                    fade_count_pos = 0;
//                    fade_count_neg = 0;
//                    //
//                    pptr_start  = (((AdcData0 - ADC_MIN) * (SAMP_MAX - SAMP_MIN)) / (ADC_MAX - ADC_MIN)) + SAMP_MIN;
//                    pptr_length = (((AdcData1 - ADC_MIN) * (LEN_MAX - LEN_MIN)) / (ADC_MAX - ADC_MIN)) + LEN_MIN;
//                    pptr_end = (pptr_start + pptr_length);
//                    pptr_half = pptr_start + (pptr_length/2);
//                }
//                play_samp = SRAM_READ(pptr--);
            }
            //Forward mode
            else {
                // pptr_0 passes midpoint and triggers pptr_1
                if(!pptr_1_trig && pptr_0 >= pptr_half_0) {
                    UpdateValues();
                    pptr_half_1 = pptr_start + (pptr_length/2);
                    fade_delta_1 = 1.0f/(pptr_length/2);
                    fade_factor_1 = 0.0f;
                    pptr_1 = pptr_start;
                    pptr_1_trig = 1;
                    pptr_0_trig = 0;

                    //DEBUG:
                    fade_count_pos_1 = 0;
                    fade_count_neg_1 = 0;
                    //
                }
                // pptr_1 passes midpoint and triggers pptr_0
                if(!pptr_0_trig && pptr_1 >= pptr_half_1) {
                    UpdateValues();
                    pptr_half_0 = pptr_start + (pptr_length/2);
                    fade_delta_0 = 1.0f/(pptr_length/2);
                    fade_factor_0 = 0.0f;
                    pptr_0 = pptr_start;
                    pptr_0_trig = 1;
                    pptr_1_trig = 0;

                    //DEBUG:
                    fade_count_pos_0 = 0;
                    fade_count_neg_0 = 0;
                    //
                }

                if(pptr_0 < pptr_half_0)
                {
                    fade_factor_0 += fade_delta_0;
                    fade_count_pos_0++;
                }
                if(pptr_0 >= pptr_half_0)
                {
                    fade_factor_0 -= fade_delta_0;
                    fade_count_neg_0++;
                }
                if(pptr_1 < pptr_half_1)
                {
                    fade_factor_1 += fade_delta_1;
                    fade_count_pos_1++;
                }
                if(pptr_1 >= pptr_half_1)
                {
                    fade_factor_1 -= fade_delta_1;
                    fade_count_neg_1++;
                }
                play_samp_0 = SRAM_READ(pptr_0);
                play_samp_1 = SRAM_READ(pptr_1);
            }
            play_samp = (int16)(((play_samp_0*fade_factor_0) + (play_samp_1*fade_factor_1))/2);
            pptr_0++;
            pptr_1++;
            play_ready = 0;
        }
        //BUTTON CHECKS
        ButtonCheck();

        //RECORD TO SRAM
        Record();
    }

}

void UpdateValues(void) {
    pptr_start  = (((AdcData0 - ADC_MIN) * (SAMP_MAX - SAMP_MIN)) / (ADC_MAX - ADC_MIN)) + SAMP_MIN;
    pptr_length = (((AdcData1 - ADC_MIN) * (LEN_MAX - LEN_MIN)) / (ADC_MAX - ADC_MIN)) + LEN_MIN;
    pptr_end = (pptr_start + pptr_length);
}

void SwitchCheck(void) {
    //DIP Switch Check//////////////////////////////////
    //SW0 - Mix Enable
    if(GpioDataRegs.GPADAT.bit.GPIO0 == 0) {
        mix_enable = 1;
        GpioDataRegs.GPADAT.bit.GPIO7 = 0;

    }
    else {
        mix_enable = 0;
        GpioDataRegs.GPADAT.bit.GPIO7 = 1;
    }
    //SW1 - Reverse Mode
    if(GpioDataRegs.GPADAT.bit.GPIO1 == 0) {
        rev_enable = 1;
        GpioDataRegs.GPADAT.bit.GPIO8 = 0;

    }
    else {
        rev_enable = 0;
        GpioDataRegs.GPADAT.bit.GPIO8 = 1;
    }
}

void ButtonCheck(void) {
    //Check center button (Recording)///////////////////
    if(GpioDataRegs.GPADAT.bit.GPIO15 == 0) {
        delay_200ms();
        if(GpioDataRegs.GPADAT.bit.GPIO16 == 0) {
            // Double press found!
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING("CLEARING SRAM...");
            SRAM_CLEAR();
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING("SRAM CLEARED");
            delay_200ms();
            delay_200ms();
            delay_200ms();
            delay_200ms();
            delay_200ms();
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING("    STANDBY");
        }
        else if(flag == 1) {
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING("    STANDBY");
            flag = 0;
            GpioDataRegs.GPADAT.bit.GPIO4 = 1; //Turn LED[0] off
        }
        else {
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING("  RECORDING...");
            GpioDataRegs.GPADAT.bit.GPIO4 = 0; //Turn LED[0] on
            sptr = 0;
            flag = 1;
        }
        delay_200ms();
        delay_200ms();
    }

    //Check left button (Playback)///////////////////////
    else if(GpioDataRegs.GPADAT.bit.GPIO16 == 0) {
        delay_200ms();
        if(GpioDataRegs.GPADAT.bit.GPIO15 == 0) {
            // Double press found!
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING("  CLEARING SRAM...");
            SRAM_CLEAR();
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING("  SRAM CLEARED");
            delay_200ms();
            delay_200ms();
            delay_200ms();
            delay_200ms();
            delay_200ms();
            LCD_SEND_COMMAND(&clear);
            // Turn off all LEDS
            GpioDataRegs.GPADAT.all |= 0x0FF0;
        }
        else if(flag == 3) {
            flag = 0;
            GpioDataRegs.GPADAT.bit.GPIO5 = 1; //Turn LED[0] off
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING("    STANDBY");
        }
        else {
            GpioDataRegs.GPADAT.bit.GPIO5 = 0; //Turn LED[1] on
            // set ptr to beginning
            if(flag == 0) {
                // Set initial values for window
                LCD_SEND_COMMAND(&clear);
                LCD_SEND_STRING("   PLAYING...");
                UpdateValues();
                pptr_0 = pptr_start;
                fade_factor_0 = 0.0f;
                fade_factor_1 = 0.0f;

                // begin playback
                flag = 3;
            }
        }
        delay_200ms();
        delay_200ms();
    }
}

void Record(void) {
    if(sptr >= 0x40000 && flag == 1) {
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING(" REC COMPLETE!");
            delay_200ms();
            delay_200ms();
            delay_200ms();
            delay_200ms();
            delay_200ms();
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING("    STANDBY");
            GpioDataRegs.GPADAT.bit.GPIO4 = 1; //Turn LED[0] off
            flag = 0;   //end recording
            sptr = 0;
    }

    if(samp_ready && flag == 1) {
        if(mix_enable) {
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
void InitAdca(void) {
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;                                 // Set ADCCLK to SYSCLK/4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // Initializes ADCA to 12-bit and single-ended mode. Performs internal calibration
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;                                 // Powers up ADC
    DELAY_US(1000);                                                    // Delay to allow ADC to power up
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;                                 // Sets SOC0 to channel 0 -> pin ADCINA0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;                                // Sets sample and hold window -> must be at least 1 ADC clock long
}

void InitAdcb(void) {
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;                                 // Set ADCCLK to SYSCLK/4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // Initializes ADCA to 12-bit and single-ended mode. Performs internal calibration
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;                                 // Powers up ADC
    DELAY_US(1000);                                                    // Delay to allow ADC to power up
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 3;                                 // Sets SOC0 to channel 0 -> pin ADCINA0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;                                // Sets sample and hold window -> must be at least 1 ADC clock long
}

void InitTimer1(void) {
    InitCpuTimers();                            // Initialize all timers to known state
    ConfigCpuTimer(&CpuTimer1, 200, 200000);    // Configure CPU timer 1. 200 -> SYSCLK in MHz, 500000 -> period in usec. NOTE: Does NOT start timer
    PieVectTable.TIMER1_INT = &Timer1_isr;      // Assign timer 1 ISR to PIE vector table
    IER |= M_INT13;                             // Enable INT13 in CPU
    CpuTimer1.RegsAddr->TCR.bit.TSS = 0;        // Start timer 1
}

interrupt void Timer1_isr(void) {
    AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1;          // Force conversion on channel 0
    AdcData0 = AdcaResultRegs.ADCRESULT0;    // Read ADC result into global variable
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;  // Toggle blue LED
    AdcbRegs.ADCSOCFRC1.bit.SOC0 = 1;          // Force conversion on channel 1
    AdcData1 = AdcbResultRegs.ADCRESULT0;    // Read ADC result into global variable
}

 interrupt void McBSP_RX_ISR(void) {
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

   //SW1 -> GPIO1
   GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
   GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO1 = 0;

   //SW2 -> GPIO2
   GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
   GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;

   //SW3 -> GPIO3
   GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
   GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;

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
