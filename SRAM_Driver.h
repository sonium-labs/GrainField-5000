/*
 * SRAM_Driver.h
 *
 *  Created on: Oct 19, 2020
 *      Author: Dan
 */

#ifndef SRAM_DRIVER_H_
#define SRAM_DRIVER_H_

void SRAM_GPIO_Init();
void SPIB_Init();
Uint16 SPI_TX(Uint16 data);
Uint16 SPI_RX();
//SRAM SPI
void SRAM_COMMAND(Uint16 command);
void SRAM_ADDR(Uint32 addr);
Uint16 SRAM_DATA(Uint16 data);
void SRAM_CLEAR();
//SRAM R/W
void SRAM_WRITE(Uint32 addr, Uint16 data);
Uint16 SRAM_READ(Uint32 addr);
void SRAM0_WRITE(Uint32 addr, Uint16 data);
void SRAM1_WRITE(Uint32 addr, Uint16 data);
Uint16 SRAM0_READ(Uint32 addr);
Uint16 SRAM1_READ(Uint32 addr);

#endif /* SRAM_DRIVER_H_ */
