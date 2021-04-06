////////////////////////////////////////////////////////////////
/* GrainField-5000
 * Author: Daniel Dole-Muinos
 * Description: Granular hardware sampler*/
////////////////////////////////////////////////////////////////
//Includes//////////////////////////////////////////////////////
#include <F28x_Project.h>
#include <stdlib.h>
#include "GrainField-5000.h"
#include "SRAM_Driver.h"
#include "AIC23.h"
#include "LCD_Driver.h"
////////////////////////////////////////////////////////////////
// Global Control Flags ////////////////////////////////////////
volatile Uint16 flag = 0;
volatile bool   rand_enable = false, smooth_enable = true, pptr_0_trig = false, pptr_1_trig = false,
                new_adc = false, samp_ready = false, play_ready = false, rand_val_update = false;
////////////////////////////////////////////////////////////////
// Sampling & Playback Variables ///////////////////////////////
volatile int16  play_samp = -99, rec_samp = -99;
volatile Uint32 sptr = 0, pptr_0 = 0, pptr_1 = 0, pptr_start = PPTR_START, pptr_length = PPTR_LENGTH, pptr_end = PPTR_END,
                pptr_half_0 = (PPTR_START+PPTR_LENGTH/2), pptr_half_1 = (PPTR_START+PPTR_LENGTH/2), pptr_start_rand_range = 0, pptr_length_rand_range = 0;
volatile int32  pptr_start_rand = 0, pptr_length_rand = 0;
volatile float  fade_factor_0 = 0.0f, fade_factor_1 = 0.0f;
////////////////////////////////////////////////////////////////
// ADC Variables ///////////////////////////////////////////////
volatile Uint16 adc_data_0 = 0, adc_data_1 = 0;
volatile float  adc_curr_0 = 0, adc_curr_1 = 0;
////////////////////////////////////////////////////////////////
// LCD Commands ////////////////////////////////////////////////
extern  Uint16  clear;
////////////////////////////////////////////////////////////////
// Functions ///////////////////////////////////////////////////
int main(void){
    InitSysCtrl();      // Set SYSCLK to 200 MHz, disables watchdog timer, turns on peripheral clocks

    LCD_init();         // Initialize LCD output

    LCD_SEND_COMMAND(&clear);
    LCD_SEND_STRING("GRAINFIELD-5000");

    DINT;               // Disable CPU interrupts on startup

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
    PieCtrlRegs.PIEIER6.bit.INTx7 = 1;          // Enable McBSPBRx interrupt

    SRAM_GPIO_Init();       // Initialize peripherals
    IO_GPIO_Init();
    SPIB_Init();
    InitSPIA();
    InitAIC23();
    InitMcBSPb();
    SRAM_CLEAR();           // Zero out SRAMs

    EnableInterrupts();

    LCD_SEND_COMMAND(&clear);
    LCD_SEND_STRING("    STANDBY");

    while(1) {
        SwitchCheck();      // MODE CHECKS
        Playback();         // PLAYBACK CONTROL
        ButtonCheck();      // BUTTON CHECKS
        Record();           // RECORD TO SRAM
    }
}

////////////////////////////////////////////////////////////////
/* Handles audio output from sound buffer (SRAM) */
void Playback(void) {
    int16 play_samp_0, play_samp_1;
    static float fade_delta_0, fade_delta_1;
    if(play_ready && flag == 3)
    {
        if(!pptr_1_trig && pptr_0 >= pptr_half_0) {             // pptr_0 passes midpoint and triggers pptr_1
            UpdatePtrs();
            pptr_half_1 = pptr_start + (pptr_length/2);
            fade_delta_1 = 1.0f/(pptr_length/2);
            fade_factor_1 = 0.0f;
            pptr_1 = pptr_start;
            pptr_1_trig = true;
            pptr_0_trig = false;
        }
        if(!pptr_0_trig && pptr_1 >= pptr_half_1) {             // pptr_1 passes midpoint and triggers pptr_0
            UpdatePtrs();
            pptr_half_0 = pptr_start + (pptr_length/2);
            fade_delta_0 = 1.0f/(pptr_length/2);
            fade_factor_0 = 0.0f;
            pptr_0 = pptr_start;
            pptr_0_trig = true;
            pptr_1_trig = false;
        }

        if(pptr_0 < pptr_half_0) fade_factor_0 += fade_delta_0; // Manages fade factor
        if(pptr_0 >= pptr_half_0) fade_factor_0 -= fade_delta_0;
        if(pptr_1 < pptr_half_1) fade_factor_1 += fade_delta_1;
        if(pptr_1 >= pptr_half_1) fade_factor_1 -= fade_delta_1;

        if(fade_factor_0 > 1) fade_factor_0 = 1;                // Ensures fade factor cannot leave bounds
        if(fade_factor_0 < 0) fade_factor_0 = 0;                // (mismatch of fade inc and dec can occur with changing sample A/B lengths)
        if(fade_factor_1 > 1) fade_factor_1 = 1;
        if(fade_factor_1 < 0) fade_factor_1 = 0;

        play_samp_0 = SRAM_READ(pptr_0++);                      // Read SRAM at final ptr addr
        play_samp_1 = SRAM_READ(pptr_1++);
        play_samp = (int16)(((play_samp_0*fade_factor_0) + (play_samp_1*fade_factor_1))/2);
        play_ready = false;
    }
}

////////////////////////////////////////////////////////////////
/* Updates loop params based on ADC data (by default)
 * Updates random values as well if right push button pressed */
void UpdatePtrs(void) {
    UpdateADC();
    if(rand_val_update) {               // Set bounds for random mode if enabled
        GpioDataRegs.GPADAT.bit.GPIO4 = 0;                  // Turn on LED that corresponds to random mode selection
        pptr_start  = (((adc_curr_0 - ADC_MIN) * (SAMP_MAX - SAMP_MIN)) / (ADC_MAX - ADC_MIN)) + SAMP_MIN;
        pptr_length = (((adc_curr_1 - ADC_MIN) * (LEN_MAX - LEN_MIN)) / (ADC_MAX - ADC_MIN)) + LEN_MIN;
        pptr_end = (pptr_start + pptr_length);
        // Sets max value for start pptr spray
        pptr_start_rand_range = (((adc_curr_0 - ADC_MIN) * (START_SPRAY_MAX - START_SPRAY_MIN)) / (ADC_MAX - ADC_MIN)) + START_SPRAY_MIN;
        pptr_length_rand_range = (((adc_curr_1 - ADC_MIN) * (LENGTH_SPRAY_MAX - LENGTH_SPRAY_MIN)) / (ADC_MAX - ADC_MIN)) + LENGTH_SPRAY_MIN;
        rand_val_update = false;
    }
    // Otherwise, continue with normal mode
    else {
        pptr_start  = (((adc_curr_0 - ADC_MIN) * (SAMP_MAX - SAMP_MIN)) / (ADC_MAX - ADC_MIN)) + SAMP_MIN;
        pptr_length = (((adc_curr_1 - ADC_MIN) * (LEN_MAX - LEN_MIN)) / (ADC_MAX - ADC_MIN)) + LEN_MIN;
        pptr_end = (pptr_start + pptr_length);
        GpioDataRegs.GPADAT.bit.GPIO4 = 1;
    }
    if(rand_enable) Randomizer();
}

////////////////////////////////////////////////////////////////
/* Shuffles pptr_start and pptr_length based on random values */
void Randomizer(void) {
    pptr_start_rand     = (((rand()) * (pptr_start_rand_range + pptr_start_rand_range)) / (RAND_MAX - RAND_MIN)) - pptr_start_rand_range;
    pptr_length_rand    = (((rand()) * (pptr_length_rand_range + pptr_length_rand_range)) / (RAND_MAX - RAND_MIN)) - pptr_length_rand_range;
    // Ensure proper start function
    if((int32)pptr_start + pptr_start_rand >= SAMP_MAX)     pptr_start = SAMP_MAX - LEN_MAX;
    else if((int32)pptr_start + pptr_start_rand < SAMP_MIN) pptr_start = SAMP_MIN;
    else                                                    pptr_start += pptr_start_rand;

    // Ensure proper length function
    if((int32)pptr_start + pptr_length + pptr_length_rand >= SAMP_MAX)      pptr_length = SAMP_MAX - pptr_start;
    else if((int32)pptr_length + pptr_length_rand < LEN_MIN)                pptr_length = LEN_MIN;
    else if((int32)pptr_start + pptr_length + pptr_length_rand < SAMP_MIN)  pptr_length = pptr_start - SAMP_MIN;
    else                                                                    pptr_length += pptr_length_rand;
}

////////////////////////////////////////////////////////////////
/* Record samples into buffer */
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
        flag = 0;                                       // end recording
        sptr = 0;
    }
    if(samp_ready && flag == 1) {
        SRAM_WRITE(sptr, rec_samp);
        samp_ready = false;
    }
}

////////////////////////////////////////////////////////////////
/* Set mode flags based on switch position */
void SwitchCheck(void) {
    if(GpioDataRegs.GPADAT.bit.GPIO0 == 0) {                // SW0 - Rand Enable
        rand_enable = true;
        GpioDataRegs.GPADAT.bit.GPIO7 = 0;
    }
    else {
        rand_enable = false;
        GpioDataRegs.GPADAT.bit.GPIO7 = 1;
    }

    if(GpioDataRegs.GPADAT.bit.GPIO1 == 0) {                // SW1 - Smooth Disable Mode
        smooth_enable = false;
        GpioDataRegs.GPADAT.bit.GPIO11 = 0;
    }
    else {
        smooth_enable = true;
        GpioDataRegs.GPADAT.bit.GPIO11 = 1;
    }
}

////////////////////////////////////////////////////////////////
/* Update pointers for loop */
void ButtonCheck(void) {
    if(GpioDataRegs.GPADAT.bit.GPIO15 == 0) {               // Check center button (Recording)
        delay_200ms();
        if(GpioDataRegs.GPADAT.bit.GPIO16 == 0) {
            LCD_SEND_COMMAND(&clear);                       // Double press found!
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
        }
        else {
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING("  RECORDING...");
            sptr = 0;
            flag = 1;
        }
        delay_200ms();
        delay_200ms();
    }
    else if(GpioDataRegs.GPADAT.bit.GPIO16 == 0) {          // Check left button (Playback)
        delay_200ms();
        if(GpioDataRegs.GPADAT.bit.GPIO15 == 0) {
            LCD_SEND_COMMAND(&clear);                       // Double press found!
            LCD_SEND_STRING("CLEARING SRAM...");
            SRAM_CLEAR();
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING("  SRAM CLEARED");
            delay_200ms();
            delay_200ms();
            delay_200ms();
            delay_200ms();
            delay_200ms();
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING("    STANDBY");
        }
        else if(flag == 3) {
            flag = 0;
            LCD_SEND_COMMAND(&clear);
            LCD_SEND_STRING("    STANDBY");
        }
        else {
            if(flag == 0) {
                LCD_SEND_COMMAND(&clear);
                LCD_SEND_STRING("   PLAYING...");
                UpdatePtrs();                               // Set initial values for window ///////////////////////////////
                pptr_0 = pptr_start;
                fade_factor_0 = 0.0f;
                fade_factor_1 = 0.0f;
                flag = 3;                                   // Begin playback
            }
        }
        delay_200ms();
        delay_200ms();
    }
    if(flag == 0 && GpioDataRegs.GPADAT.bit.GPIO14 == 0) {  // Check right button (Random Mode)
        GpioDataRegs.GPADAT.bit.GPIO4 = 0;
        UpdateADC();
        pptr_start_rand_range = (((adc_curr_0 - ADC_MIN) * (START_SPRAY_MAX - START_SPRAY_MIN)) / (ADC_MAX - ADC_MIN)) + START_SPRAY_MIN;
        pptr_length_rand_range = (((adc_curr_1 - ADC_MIN) * (LENGTH_SPRAY_MAX - LENGTH_SPRAY_MIN)) / (ADC_MAX - ADC_MIN)) + LENGTH_SPRAY_MIN;
        rand_val_update = true;
    }
    else if(flag == 0){
        GpioDataRegs.GPADAT.bit.GPIO4 = 1;
    }
}
////////////////////////////////////////////////////////////////
/* Update ADC values and applies smoothing if smooth mode enabled */
void UpdateADC(void) {
    float adc_smoov_0 = 0, adc_smoov_1 = 0, adc_diff_0 = 0, adc_diff_1 = 0;
    static float adc_prev_0 = 0, adc_prev_1 = 0;
    if(new_adc) {
        if(smooth_enable) {         // Manages smoothing of ADC values
            adc_smoov_0 = adc_smoov_0 + (WEIGHTING * (adc_data_0 - adc_smoov_0));
            adc_smoov_1 = adc_smoov_1 + (WEIGHTING * (adc_data_1 - adc_smoov_1));
            adc_diff_0 = adc_smoov_0 - adc_prev_0;
            if(abs(adc_diff_0) > DEADZONE) {
                adc_curr_0 = adc_smoov_0;
                adc_prev_0 = adc_smoov_0;
            }
            adc_diff_1 = adc_smoov_1 - adc_prev_1;
            if(abs(adc_diff_1) > DEADZONE) {
                adc_curr_1 = adc_smoov_1;
                adc_prev_1 = adc_smoov_1;
            }
        }
        else {
            adc_curr_0 = adc_data_0;
            adc_curr_1 = adc_data_1;
        }
        new_adc = false;
    }
}
////////////////////////////////////////////////////////////////
/* Timer ISR: polls ADCs for new data */
interrupt void Timer1_isr(void) {
    AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1;
    adc_data_0 = AdcaResultRegs.ADCRESULT0;
    AdcbRegs.ADCSOCFRC1.bit.SOC0 = 1;
    adc_data_1 = AdcbResultRegs.ADCRESULT0;
    new_adc = true;
}
////////////////////////////////////////////////////////////////
/* McBSP ISR: interact with McBSP peripheral based on mode */
interrupt void McBSP_RX_ISR(void) {
     int16 throwaway, LS, RS;
     if(flag == 0) {                             // Case: Standby (passthrough)
         LS = McbspbRegs.DRR2.all;
         RS = McbspbRegs.DRR1.all;

         McbspbRegs.DXR2.all = LS;
         McbspbRegs.DXR1.all = RS;
     }
     else if(flag == 1) {                       // Case: Record sound into buffer
         LS = McbspbRegs.DRR2.all;
         RS = McbspbRegs.DRR1.all;

         McbspbRegs.DXR2.all = LS;
         McbspbRegs.DXR1.all = RS;
         samp_ready = true;
         sptr++;
     }
     else if(flag == 3) {                       // Case: Playback sound
         throwaway = McbspbRegs.DRR2.all;
         throwaway = McbspbRegs.DRR1.all;

         if(!play_ready) {
             McbspbRegs.DXR2.all = play_samp;
             McbspbRegs.DXR1.all = play_samp;
             play_ready = true;
         }
     }
     rec_samp = (LS+RS)/2;
     PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

////////////////////////////////////////////////////////////////
/* Init AdcA (for left potentiometer) */
void InitAdca(void) {
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    DELAY_US(1000);
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;
}

////////////////////////////////////////////////////////////////
/* Init AdcB (for right potentiometer) */
void InitAdcb(void) {
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    DELAY_US(1000);
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 3;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;
}

////////////////////////////////////////////////////////////////
/* Init timer used in ADC poll interrupt */
void InitTimer1(void) {
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer1, 200, 400000);
    PieVectTable.TIMER1_INT = &Timer1_isr;
    IER |= M_INT13;
    CpuTimer1.RegsAddr->TCR.bit.TSS = 0;
}

////////////////////////////////////////////////////////////////
/* Init GPIO pins used in IO (as opposed to SRAM usage) */
void IO_GPIO_Init() {
   EALLOW;
   // SW0 -> GPIO0
   GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
   GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO0 = 0;

   // SW1 -> GPIO1
   GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
   GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO1 = 0;

   // SW2 -> GPIO2
   GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
   GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;

   // SW3 -> GPIO3
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
}
