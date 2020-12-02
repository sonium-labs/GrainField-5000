/*
 * SRAM_Driver.c
 *
 *  Created on: Oct 19, 2020
 *      Author: Dan
 */
#include <F28x_Project.h>
#include "SRAM_Driver.h"

////////////////////////////////////////////////////////////////
void SRAM_WRITE(Uint32 addr, Uint16 data) {

    Uint16 byte0 = data & 0x00FF;               //Lower byte
    Uint16 byte1 = ((data & 0xFF00) >> 8);      //Upper byte

    SRAM0_WRITE(addr,byte0);                    //Write lower byte to SRAM0
    SRAM1_WRITE(addr,byte1);                    //Write upper byte to SRAM1

    return;
}
////////////////////////////////////////////////////////////////
Uint16 SRAM_READ(Uint32 addr) {

    Uint16 byte0 = SRAM0_READ(addr);            //Lower byte
    Uint16 byte1 = SRAM1_READ(addr);            //Upper byte

    return byte0 | (byte1 << 8);
}
////////////////////////////////////////////////////////////////
void SRAM0_WRITE(Uint32 addr, Uint16 data) {
//Desc: Write to SRAM0
    GpioDataRegs.GPCDAT.bit.GPIO66 = 0;         //CS0 low
    SRAM_COMMAND(0x02);                         //Send write command
    SRAM_ADDR(addr);                            //Send addr
    SRAM_DATA(data);                            //Send data
    GpioDataRegs.GPCDAT.bit.GPIO66 = 1;         //CS0 high

    return;
}
////////////////////////////////////////////////////////////////
void SRAM1_WRITE(Uint32 addr, Uint16 data) {
//Desc: Write to SRAM1
    GpioDataRegs.GPCDAT.bit.GPIO67 = 0;         //CS1 low
    SRAM_COMMAND(0x02);                         //Send write command
    SRAM_ADDR(addr);                            //Send addr
    SRAM_DATA(data);                            //Send data
    GpioDataRegs.GPCDAT.bit.GPIO67 = 1;         //CS1 high

    return;
}
////////////////////////////////////////////////////////////////
Uint16 SRAM0_READ(Uint32 addr) {
//Desc: Read from SRAM0
    Uint16 received;

    GpioDataRegs.GPCDAT.bit.GPIO66 = 0;         //CS0 low
    SRAM_COMMAND(0x03);                         //Send read command
    SRAM_ADDR(addr);                            //Send addr
    SRAM_DATA(0x22);                            //Dummy data
    received = SRAM_DATA(0x33);                 //Dummy data and read data
    GpioDataRegs.GPCDAT.bit.GPIO66 = 1;         //CS0 high

    return received;
}
////////////////////////////////////////////////////////////////
Uint16 SRAM1_READ(Uint32 addr) {
//Desc: Read from SRAM1
    Uint16 received;

    GpioDataRegs.GPCDAT.bit.GPIO67 = 0;         //CS1 low
    SRAM_COMMAND(0x03);                         //Send read command
    SRAM_ADDR(addr);                            //Send addr
    SRAM_DATA(0x22);                            //Dummy data
    received = SRAM_DATA(0x33);                 //Dummy data and read data
    GpioDataRegs.GPCDAT.bit.GPIO67 = 1;         //CS1 high

    return received;
}
////////////////////////////////////////////////////////////////
void SRAM_GPIO_Init() {
//Desc:Initializes GPIO required for use in SPI & testing
//
//Breakdown:
//GPIO63 => SPISIMOB
//GPIO64 => SPISOMIB
//GPIO65 => SPICLKB
//GPIO (66/67) => CS0/CS1 (both outputs)
//GPIO 16 => PB2
//GPIO 15 => PB1
//GPIO 14 => PB0

    EALLOW;

    //Core SPI Signals
    //Set MUX
    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 3;       //SPISIMOB
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 3;       //SPISOMIB
    GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 3;       //SPICLKB
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 3;
    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;
    //Enable pullups
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;

    //Chip Select GPIO (set as outputs)
    //Set MUX
    GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = 0;       //~CS0
    GpioCtrlRegs.GPCGMUX1.bit.GPIO67 = 0;       //~CS1
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;
    //Set as output
    GpioCtrlRegs.GPCDIR.bit.GPIO66 = 1;
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;
    //Set default to high
    GpioDataRegs.GPCDAT.bit.GPIO66 = 1;
    GpioDataRegs.GPCDAT.bit.GPIO67 = 1;

    //Push-buttons
    GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 0;       //PB0
    GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = 0;       //PB1
    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;       //PB2
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;

    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;

    return;
}
////////////////////////////////////////////////////////////////
void SPIB_Init() {
//Desc: Initializes SPI Peripheral

    EALLOW;

    //Clear reset bit
    SpibRegs.SPICCR.bit.SPISWRESET = 0;

    //Select Master Mode
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;

    //Select rising edge w/ phase delay mode
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpibRegs.SPICTL.bit.CLK_PHASE = 1;

    //Transmit Enable
    SpibRegs.SPICTL.bit.TALK = 1;

    //Baud Rate Settings
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;        //Set LSPCLK = SYSCLK (200MHz)
    SpibRegs.SPICCR.bit.HS_MODE = 1;            //Enable high-speed mode
    SpibRegs.SPIBRR.all = 10;                    //Set baud rate to 200MHz/10 = 20 MHz

    //Set SPI character length to 8-bits
    SpibRegs.SPICCR.bit.SPICHAR = 0x7;

    //Overrun interrupt
    SpibRegs.SPICTL.bit.OVERRUNINTENA = 1;      //Overrun interrupt enable
    SpibRegs.SPICTL.bit.SPIINTENA = 1;          //SPI interrupt enable

    //Software reset SPI
    SpibRegs.SPICCR.bit.SPISWRESET = 1;

    //Run SPI even when simulation suspended
    SpibRegs.SPIPRI.bit.FREE = 1;

    return;
}
////////////////////////////////////////////////////////////////
void SRAM_COMMAND(Uint16 command) {
//Desc: Send out SRAM command byte through SPI
     SPI_TX(command);

     return;
}
////////////////////////////////////////////////////////////////
void SRAM_ADDR(Uint32 addr) {
//Desc: Send out 3 bytes of SRAM addr through SPI
    Uint32 temp;

    temp = addr;

    temp >>= 16;        //top 8 bits

    SPI_TX(temp);

    temp = addr;

    temp >>= 8;         //top 8 bits

    SPI_TX(temp);

    temp = addr;

    SPI_TX(temp);       //last 8 bits

    return;
}
////////////////////////////////////////////////////////////////
Uint16 SRAM_DATA(Uint16 data){
//Desc: Write to SPI data register
    return SPI_TX(data);
}

void SRAM_CLEAR() {
//Desc: Zero out SRAM memory
    for(Uint32 i = 0; i < 0x3FFFF;i++) {
        SRAM_WRITE(i,0x0000);
    }
}
////////////////////////////////////////////////////////////////
Uint16 SPI_TX(Uint16 data) {
//Desc: Write 16 bytes out through SPI

    data <<= 8;      //Left adjust command

    //Send data to SPI Register
    SpibRegs.SPIDAT = data;

    //Wait until data has been sent
    while(!SpibRegs.SPISTS.bit.INT_FLAG);

    //Clear flag
    return SpibRegs.SPIRXBUF;
}
////////////////////////////////////////////////////////////////
Uint16 SPI_RX() {
//Desc: Receive 16 bytes in through SPI

    //Send dummy data to SPI Register
    SpibRegs.SPIDAT = 0x37;

    //Wait until data has been sent
    while(!SpibRegs.SPISTS.bit.INT_FLAG);

    //Return and clear register
    return SpibRegs.SPIRXBUF;
}
////////////////////////////////////////////////////////////////
