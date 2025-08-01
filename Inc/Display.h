/*
 * Display.h
 *
 *  Created on: Feb 8, 2016
 *      Author: user
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stdint.h>

#define FFT_H 40
#define FFT_Resolution 6.25 // 8000/2/1280
#define MESSAGE_SIZE 40

extern uint16_t cursor;
extern int FT8_Touch_Flag;
extern int FT_8_TouchIndex;

void show_variable(uint16_t x, uint16_t y, int variable);

void show_short(uint16_t x, uint16_t y, uint8_t variable);

void show_wide(uint16_t x, uint16_t y, int variable);

void show_UTC_time(uint16_t x, uint16_t y, int utc_hours, int utc_minutes,
				   int utc_seconds, int color);

void show_Real_Date(uint16_t x, uint16_t y, int date, int month, int year);

void setup_display(void);

void Process_Touch(void);

void Display_WF(void);

void Set_Cursor_Frequency(void);

uint16_t testButton(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

#endif /* DISPLAY_H_ */
