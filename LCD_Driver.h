/*
 * LCD_Driver.h
 *
 *  Created on: Oct 1, 2020
 *      Author: Dan
 */

#ifndef LCD_DRIVER_H_
#define LCD_DRIVER_H_

void LCD_SEND_STRING(char *string);
void LCD_SEND_COMMAND(Uint16 *command);
void LCD_init();
void LCD_SEND_CHAR(char *string);

#endif /* LCD_DRIVER_H_ */
