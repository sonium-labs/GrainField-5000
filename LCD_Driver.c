/*
 * LCD_Driver.c
 *
 *  Created on: Oct 1, 2020
 *      Author: Dan
 */
#include <F28x_Project.h>
#include "OneToOneI2CDriver.h"
#include "LCD_Driver.h"

#define delay_5ms() for(volatile long j=0; j < 1000000; j++)
#define COMMAND_EN  0xC
#define COMMAND_DIS 0x8
#define DATA_EN     0xD
#define DATA_DIS    0x9

Uint16  initvals[20] = {0x3C, 0x38, 0x3C, 0x38, 0x3C, 0x38, 0x2C, 0x28,
                        0x2C, 0x28, 0x8C, 0x88, 0x0C, 0x08,
                        0xFC, 0xF8, 0x0C, 0x08, 0x1C, 0x18};
Uint16  printtest[4] = {0x4D, 0x49, 0x4D, 0x49};

//Uint16  nextLine     = 0xC0;
//Uint16  clear        = 0x01;
//Uint16  home         = 0x02;

void LCD_init() {

    I2C_O2O_Master_Init(0x0027, 200, 100);  //send address and set up clocks
    delay_5ms();
    I2C_O2O_SendBytes(initvals, 20);
    delay_5ms();

    return;
}

void LCD_SEND_STRING(char *string) {
    Uint16 temp, upper, lower;

    while(*string != 0x0000) {
        upper = *string;
        lower = *string;
        upper &= 0xF0;

        lower &= 0x0F;
        lower <<= 4;

        //Upper Step 1
        temp    =   upper;
        temp    |=  DATA_EN;
        I2C_O2O_SendBytes(&temp, 1);

        //Upper Step 2
        temp    =   upper;
        temp    |=  DATA_DIS;
        I2C_O2O_SendBytes(&temp, 1);

        //Lower Step 1
        temp    =   lower;
        temp    |=  DATA_EN;
        I2C_O2O_SendBytes(&temp, 1);

        //Lower Step 2
        temp    =   lower;
        temp    |=  DATA_DIS;
        I2C_O2O_SendBytes(&temp, 1);

        string++;
    }

    return;
}

void LCD_SEND_CHAR(char *string) {
    Uint16 temp, upper, lower;

        upper = *string;
        lower = *string;
        upper &= 0xF0;

        lower &= 0x0F;
        lower <<= 4;

        //Upper Step 1
        temp    =   upper;
        temp    |=  DATA_EN;
        I2C_O2O_SendBytes(&temp, 1);

        //Upper Step 2
        temp    =   upper;
        temp    |=  DATA_DIS;
        I2C_O2O_SendBytes(&temp, 1);

        //Lower Step 1
        temp    =   lower;
        temp    |=  DATA_EN;
        I2C_O2O_SendBytes(&temp, 1);

        //Lower Step 2
        temp    =   lower;
        temp    |=  DATA_DIS;
        I2C_O2O_SendBytes(&temp, 1);

    return;
}

void LCD_SEND_COMMAND(Uint16 *command) {
    Uint16 temp, upper, lower;

        upper = *command;
        lower = *command;
        upper &= 0xF0;

        lower &= 0x0F;
        lower <<= 4;

        //Upper Step 1
        temp    =   upper;
        temp    |=  COMMAND_EN;
        I2C_O2O_SendBytes(&temp, 1);

        //Upper Step 2
        temp    =   upper;
        temp    |=  COMMAND_DIS;
        I2C_O2O_SendBytes(&temp, 1);

        //Lower Step 1
        temp    =   lower;
        temp    |=  COMMAND_EN;
        I2C_O2O_SendBytes(&temp, 1);

        //Lower Step 2
        temp    =   lower;
        temp    |=  COMMAND_DIS;
        I2C_O2O_SendBytes(&temp, 1);

        delay_5ms();

        return;
}
